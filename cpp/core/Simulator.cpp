#include "core/Simulator.hpp"
#include <fstream>
#include <cmath>
#include <algorithm>

namespace aerofleet {

namespace {

Vec3 worldToBody(const Vec3& value, double yaw) {
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    return Vec3(c * value.x + s * value.y, -s * value.x + c * value.y, value.z);
}

std::string resolveConfigPath(const std::string& path) {
    std::ifstream f(path);
    if (f.good()) return path;

    std::string fallback1 = "../" + path;
    std::ifstream f1(fallback1);
    if (f1.good()) return fallback1;

    std::string fallback2 = "../../" + path;
    std::ifstream f2(fallback2);
    if (f2.good()) return fallback2;

    return path;
}

} // namespace

std::vector<float> Observation::toVector() const {
    std::vector<float> vec;
    vec.reserve(vectorSize());

    // Ego velocity [3]
    vec.push_back(static_cast<float>(egoVelocity.x));
    vec.push_back(static_cast<float>(egoVelocity.y));
    vec.push_back(static_cast<float>(egoVelocity.z));

    // Body-frame acceleration [3]
    vec.push_back(static_cast<float>(egoAcceleration.x));
    vec.push_back(static_cast<float>(egoAcceleration.y));
    vec.push_back(static_cast<float>(egoAcceleration.z));

    // Relative target vector [3]
    vec.push_back(static_cast<float>(relativeTarget.x));
    vec.push_back(static_cast<float>(relativeTarget.y));
    vec.push_back(static_cast<float>(relativeTarget.z));

    // Scalar states [4]
    vec.push_back(static_cast<float>(distanceToTarget));
    vec.push_back(static_cast<float>(batteryFraction));
    vec.push_back(static_cast<float>(payloadMass));
    vec.push_back(static_cast<float>(speed));

    // Lidar beams [44]
    if (lidarBeams.size() == 44) {
        vec.insert(vec.end(), lidarBeams.begin(), lidarBeams.end());
    } else {
        vec.insert(vec.end(), 44, 1.0f);
    }

    // Nearby UAVs [18]
    if (nearbyUavs.size() == 18) {
        vec.insert(vec.end(), nearbyUavs.begin(), nearbyUavs.end());
    } else {
        vec.insert(vec.end(), 18, 0.0f);
    }

    // Heading encoding [2] and UAV task/operational phase one-hot [10]
    vec.push_back(static_cast<float>(headingSin));
    vec.push_back(static_cast<float>(headingCos));
    vec.insert(vec.end(), modeOneHot.begin(), modeOneHot.end());

    return vec;
}

Simulator::Simulator(const SimulationConfig& config)
    : m_config(config),
      m_map(WorldGenerator::generate(m_config.world, m_config.simulation.randomSeed)),
      m_fleet(m_config.fleet, m_config.energy, m_map, m_config.world.defaultSpawnAltitude),
      m_requestGen(m_config.requests, m_map, m_config.fleet.maxHorizontalSpeed,
                   m_config.simulation.randomSeed,
                   m_config.world.zMin + m_config.world.defaultSpawnAltitude),
      m_navigator(WaypointControllerConfig{
          1.0, 30.0, 20.0, 1.0, 1.0,
          m_config.fleet.maxHorizontalSpeed,
          m_config.fleet.maxVerticalSpeed,
          1.5, 0.2
      }),
      m_coordinator(),
      m_collisions(m_config.fleet.uavRadius),
      m_integrator(KinematicsConfig::fromConfig(m_config.fleet, m_config.world)),
      m_clock(m_config.simulation.dtPhysics, 1.0),
      m_metrics(),
      m_logger(131072),
      m_currentSeed(m_config.simulation.randomSeed) {
    m_prevPositions.resize(m_fleet.size());
    for (size_t i = 0; i < m_fleet.size(); ++i) {
        m_prevPositions[i] = m_fleet.getUAV(i).position;
    }
}

Simulator::Simulator(const std::string& configPath)
    : Simulator(SimulationConfig::loadFromFile(resolveConfigPath(configPath))) {}

void Simulator::reset(uint64_t seed) {
    m_navigationTask = false;
    m_navigationTaskSucceeded = false;
    if (seed != 0) {
        m_currentSeed = seed;
        m_config.simulation.randomSeed = seed;
    }

    m_map = WorldGenerator::generate(m_config.world, m_currentSeed);
    m_fleet.initialize(m_config.fleet, m_config.energy, m_map, m_config.world.defaultSpawnAltitude);
    m_requestGen.reset(m_currentSeed);
    m_clock.reset(0.0);
    m_metrics.reset();

    m_pendingRequests.clear();
    m_deliveredRequests.clear();
    m_requeuedRequests.clear();
    m_collisionEvents.clear();

    m_prevPositions.resize(m_fleet.size());
    for (size_t i = 0; i < m_fleet.size(); ++i) {
        m_prevPositions[i] = m_fleet.getUAV(i).position;
    }

    m_lastCoordinatorDispatchTime = -10.0;
}

void Simulator::resetNavigationTask(uint64_t seed, const Vec3& start, const Vec3& target, double initialYaw) {
    reset(seed);
    if (!m_config.world.contains(start, m_config.fleet.uavRadius) ||
        !m_config.world.contains(target, m_config.fleet.uavRadius)) {
        throw std::invalid_argument("navigation start and target must be inside world bounds");
    }
    if (start.distance(target) <= m_navigationArrivalRadius) {
        throw std::invalid_argument("navigation target must be farther than the arrival radius");
    }
    if (!std::isnan(initialYaw) && !std::isfinite(initialYaw)) {
        throw std::invalid_argument("navigation initial yaw must be finite when specified");
    }

    // Stage 1 is an obstacle-free, single-UAV waypoint task. Later curriculum
    // stages will re-enable map geometry and fleet traffic explicitly.
    m_map.buildings.clear();
    m_map.rebuildBroadphase();
    FleetSettings taskFleet = m_config.fleet;
    taskFleet.numUavs = 1;
    m_fleet.initialize(taskFleet, m_config.energy, m_map, m_config.world.defaultSpawnAltitude);

    auto& uav = m_fleet.getUAV(0);
    const double yaw = std::isnan(initialYaw)
        ? std::atan2(target.y - start.y, target.x - start.x)
        : initialYaw;
    uav.reset(start, yaw);
    uav.targetWaypoint = target;
    uav.mode = UAVMode::FLYING_TO_DESTINATION;
    uav.isGrounded = false;

    m_navigationTask = true;
    m_navigationTaskSucceeded = false;
    m_navigationTarget = target;
    m_prevPositions.assign(1, start);
}

void Simulator::integratePhysicsSubStep(double dt, const std::vector<VelocityCommand>& actions) {
    // 1. Record previous positions for Continuous Collision Detection
    for (size_t i = 0; i < m_fleet.size(); ++i) {
        m_prevPositions[i] = m_fleet.getUAV(i).position;
    }

    m_clock.advance();
    const double currentTime = m_clock.now();

    if (!m_navigationTask) {
        // 2. Generate Poisson delivery requests
        auto newRequests = m_requestGen.step(currentTime, dt);
        if (!newRequests.empty()) {
            for (const auto& req : newRequests) {
                m_metrics.recordRequestCreated(req);
            }
            m_pendingRequests.insert(m_pendingRequests.end(), newRequests.begin(), newRequests.end());
        }

        for (auto it = m_pendingRequests.begin(); it != m_pendingRequests.end();) {
            const bool timedOut = currentTime > it->creationTime + m_config.requests.requestTimeoutSeconds;
            if (it->status == RequestStatus::PENDING && (it->isExpired(currentTime) || timedOut)) {
                it->transitionTo(RequestStatus::EXPIRED, currentTime);
                m_metrics.recordRequestExpired(*it, currentTime);
                m_logger.log(currentTime, SimulationEventType::REQUEST_EXPIRED, it->id);
                it = m_pendingRequests.erase(it);
            } else {
                ++it;
            }
        }

        // 3. Dispatch heuristic coordinator (if auto-dispatch is enabled)
        if (m_autoCoordinatorDispatch && (currentTime - m_lastCoordinatorDispatchTime >= m_config.simulation.dtCoordinator)) {
            m_coordinator.dispatch(m_fleet, m_pendingRequests, m_map, currentTime);
            for (const auto& request : m_pendingRequests) {
                if (request.status == RequestStatus::ASSIGNED) {
                    m_logger.log(currentTime, SimulationEventType::REQUEST_ASSIGNED,
                                 request.id, request.assignedUavId);
                }
            }
            m_pendingRequests.erase(
                std::remove_if(m_pendingRequests.begin(), m_pendingRequests.end(),
                               [](const DeliveryRequest& request) { return request.status != RequestStatus::PENDING; }),
                m_pendingRequests.end());
            m_lastCoordinatorDispatchTime = currentTime;
        }

        // 4. Step fleet lifecycle (charging, dwells, transitions)
        const size_t prevDeliveredCount = m_deliveredRequests.size();
        m_fleet.step(dt, currentTime, m_map, m_deliveredRequests, m_requeuedRequests);
        for (size_t d = prevDeliveredCount; d < m_deliveredRequests.size(); ++d) {
            m_metrics.recordRequestDelivered(m_deliveredRequests[d], currentTime);
        }
        if (!m_requeuedRequests.empty()) {
            m_pendingRequests.insert(m_pendingRequests.end(), m_requeuedRequests.begin(), m_requeuedRequests.end());
            m_requeuedRequests.clear();
        }
    }

    // 5. Apply actions, integrate kinematics, and discharge battery
    double totalPowerWatts = 0.0;
    const std::vector<UAVState> trafficSnapshot = m_fleet.getAllUAVs();
    for (size_t i = 0; i < m_fleet.size(); ++i) {
        auto& uav = m_fleet.getUAV(i);
        VelocityCommand cmd;
        if (!actions.empty() && i < actions.size()) {
            cmd = actions[i];
        } else if (uav.mode == UAVMode::FLYING_TO_PICKUP ||
                   uav.mode == UAVMode::FLYING_TO_DESTINATION ||
                   uav.mode == UAVMode::RETURNING_TO_CHARGE) {
            if (m_navPolicy) {
                cmd = m_navPolicy->computeAction(uav, m_map, dt);
                cmd = m_navigator.applyTrafficAvoidance(uav, cmd, trafficSnapshot);
            } else {
                cmd = m_navigator.computeActionWithTraffic(uav, m_map, trafficSnapshot, dt);
            }
        }
        m_integrator.stepVelocityCommand(uav, cmd, dt);
        m_fleet.getBatteryModel().stepDischarge(uav, dt);
        totalPowerWatts += m_fleet.getBatteryModel().computePowerWatts(uav.velocity, uav.payloadMass);
    }
    m_metrics.recordStep(m_fleet.getAllUAVs(), dt, totalPowerWatts, currentTime);

    // 6. Continuous collision detection and resolution
    auto newCollisions = m_collisions.evaluateAndResolve(m_fleet.getAllUAVs(), m_prevPositions, m_map, currentTime);
    if (!newCollisions.empty()) {
        for (const auto& c : newCollisions) {
            m_metrics.recordCollision(c);
            const auto abortCrashedAssignment = [&](UAVId id) {
                DeliveryRequest requeued;
                if (m_fleet.abortAssignment(id, requeued)) {
                    m_requeuedRequests.push_back(requeued);
                    m_logger.log(currentTime, SimulationEventType::REQUEST_REQUEUED, requeued.id, id);
                }
                if (id >= 0 && static_cast<size_t>(id) < m_fleet.size()) {
                    m_fleet.getUAV(static_cast<size_t>(id)).mode = UAVMode::EMERGENCY;
                }
            };
            abortCrashedAssignment(c.uavIdA);
            if (c.type == CollisionType::UAV_UAV) abortCrashedAssignment(c.uavIdB);
        }
        if (!m_requeuedRequests.empty()) {
            m_pendingRequests.insert(m_pendingRequests.end(), m_requeuedRequests.begin(), m_requeuedRequests.end());
            m_requeuedRequests.clear();
        }
        m_collisionEvents.insert(m_collisionEvents.end(), newCollisions.begin(), newCollisions.end());
    }

    if (m_navigationTask && !m_navigationTaskSucceeded &&
        m_fleet.getUAV(0).mode != UAVMode::EMERGENCY &&
        m_fleet.getUAV(0).position.distance(m_navigationTarget) <= m_navigationArrivalRadius) {
        m_navigationTaskSucceeded = true;
    }
}

StepResult Simulator::step(const std::vector<VelocityCommand>& actions) {
    const double dtPolicy = (m_config.simulation.dtPolicy > 0.0) ? m_config.simulation.dtPolicy : 0.1;
    const double dtPhysics = (m_config.simulation.dtPhysics > 0.0) ? m_config.simulation.dtPhysics : 0.02;
    const size_t numSubSteps = (dtPolicy > dtPhysics) ? static_cast<size_t>(std::round(dtPolicy / dtPhysics)) : 1;
    const double actualStepDuration = static_cast<double>(numSubSteps) * dtPhysics;

    std::vector<UAVState> prevUavs = m_fleet.getAllUAVs();
    const size_t initialCollisionCount = m_collisionEvents.size();
    const bool wasNavigationTaskSucceeded = m_navigationTaskSucceeded;

    for (size_t s = 0; s < numSubSteps; ++s) {
        integratePhysicsSubStep(dtPhysics, actions);
    }

    StepResult result;
    const size_t n = m_fleet.size();
    result.observations.reserve(n);
    result.rewards.reserve(n);
    result.terminated.reserve(n);
    result.truncated.reserve(n);

    for (size_t i = 0; i < n; ++i) {
        const auto& uav = m_fleet.getUAV(i);
        const auto& prevUav = prevUavs[i];

        result.observations.push_back(getObservation(uav.id));

        bool crashed = false;
        for (size_t c = initialCollisionCount; c < m_collisionEvents.size(); ++c) {
            const auto& col = m_collisionEvents[c];
            if (col.uavIdA == uav.id || (col.type == CollisionType::UAV_UAV && col.uavIdB == uav.id)) {
                crashed = true;
                break;
            }
        }

        bool reached = (prevUav.mode == UAVMode::FLYING_TO_PICKUP && uav.mode == UAVMode::PICKING_UP) ||
                       (prevUav.mode == UAVMode::FLYING_TO_DESTINATION && uav.mode == UAVMode::DROPPING_OFF) ||
                       (prevUav.mode == UAVMode::RETURNING_TO_CHARGE && uav.mode == UAVMode::CHARGING);

        const bool failed = crashed || uav.mode == UAVMode::EMERGENCY || uav.batteryPercent <= 0.0;
        const bool succeeded = m_navigationTask && m_navigationTaskSucceeded && !wasNavigationTaskSucceeded;
        result.rewards.push_back(m_navigationTask
            ? computeNavigationReward(uav, prevUav, succeeded, failed, actualStepDuration)
            : computeReward(uav, prevUav, reached, crashed));
        result.terminated.push_back(
            (m_navigationTask && m_navigationTaskSucceeded) || uav.mode == UAVMode::EMERGENCY || uav.batteryPercent <= 0.0
        );
        result.truncated.push_back(false);
    }

    result.allDone = std::all_of(result.terminated.begin(), result.terminated.end(), [](bool b) { return b; });
    result.metrics = getSnapshot().metrics;

    return result;
}

StepResult Simulator::stepHeuristic() {
    std::vector<VelocityCommand> emptyActions;
    return step(emptyActions);
}

Observation Simulator::getObservation(UAVId uavId) const {
    Observation obs;
    obs.uavId = uavId;

    const UAVState* uavPtr = nullptr;
    for (const auto& u : m_fleet.getAllUAVs()) {
        if (u.id == uavId) {
            uavPtr = &u;
            break;
        }
    }

    if (!uavPtr) {
        obs.lidarBeams.resize(44, 1.0f);
        obs.nearbyUavs.resize(18, 0.0f);
        return obs;
    }

    const auto& uav = *uavPtr;
    obs.egoAcceleration = worldToBody(uav.acceleration, uav.yaw);
    obs.egoVelocity = worldToBody(uav.velocity, uav.yaw);
    obs.relativeTarget = worldToBody(uav.targetWaypoint - uav.position, uav.yaw);
    obs.distanceToTarget = obs.relativeTarget.length();
    obs.batteryFraction = uav.batteryPercent / 100.0;
    obs.payloadMass = uav.payloadMass;
    obs.speed = uav.speed();
    obs.headingSin = std::sin(uav.yaw);
    obs.headingCos = std::cos(uav.yaw);
    const auto modeIndex = static_cast<size_t>(uav.mode);
    if (modeIndex < obs.modeOneHot.size()) obs.modeOneHot[modeIndex] = 1.0f;

    obs.lidarBeams = computeLidarBeams(uav.position, uav.yaw, 100.0);
    obs.nearbyUavs = computeNearbyUavs(uavId, 3);

    return obs;
}

std::vector<Observation> Simulator::getAllObservations() const {
    std::vector<Observation> obs;
    obs.reserve(m_fleet.size());
    for (const auto& u : m_fleet.getAllUAVs()) {
        obs.push_back(getObservation(u.id));
    }
    return obs;
}

WorldSnapshot Simulator::getSnapshot() const {
    WorldSnapshot snap;
    snap.simulationTime = m_clock.now();
    snap.wallTime = m_clock.getWallTime();
    snap.stepCount = m_clock.getStepCount();
    snap.uavs = m_fleet.getAllUAVs();
    snap.activeRequests = m_pendingRequests;
    snap.chargingStations = m_map.chargingStations;
    snap.recentEvents = m_collisionEvents;

    snap.metrics.totalCreated = m_metrics.getSnapshot().totalRequestsCreated;
    snap.metrics.totalDelivered = m_deliveredRequests.size();
    snap.metrics.totalPending = m_pendingRequests.size();
    snap.metrics.totalActive = m_fleet.countInMode(UAVMode::FLYING_TO_PICKUP) +
                               m_fleet.countInMode(UAVMode::PICKING_UP) +
                               m_fleet.countInMode(UAVMode::FLYING_TO_DESTINATION) +
                               m_fleet.countInMode(UAVMode::DROPPING_OFF);
    snap.metrics.collisionCount = m_collisionEvents.size();
    snap.metrics.deliverySuccessRate = m_metrics.getDeadlineSuccessRate();
    snap.metrics.averageDeliveryDuration = m_metrics.getMeanLatency();
    snap.metrics.realFPS = 60.0;

    return snap;
}

std::vector<float> Simulator::computeLidarBeams(const Vec3& position, double yaw, double maxRange) const {
    std::vector<float> beams;
    beams.reserve(44);

    auto candidates = m_map.broadphase.querySphere(position, maxRange);

    // 1. 36 horizontal beams in azimuth around yaw
    for (int i = 0; i < 36; ++i) {
        double angle = yaw + (i * 2.0 * MathUtils::PI / 36.0);
        Vec3 dir(std::cos(angle), std::sin(angle), 0.0);
        Ray ray(position, dir);

        double minHitDist = maxRange;
        for (int32_t bIdx : candidates) {
            if (bIdx >= 0 && static_cast<size_t>(bIdx) < m_map.buildings.size()) {
                auto hit = raycastAABB(ray, m_map.buildings[bIdx].bounds, minHitDist);
                if (hit.hit && hit.distance < minHitDist) {
                    minHitDist = hit.distance;
                }
            }
        }
        beams.push_back(static_cast<float>(minHitDist / maxRange));
    }

    // 2. 8 elevation beams (4 up +30 deg, 4 down -30 deg)
    const double pitchUp = MathUtils::PI / 6.0;
    const double cosPitch = std::cos(pitchUp);
    const double sinPitchUp = std::sin(pitchUp);
    const double sinPitchDown = -std::sin(pitchUp);

    // 4 up beams at 90-degree quadrant angles
    for (int i = 0; i < 4; ++i) {
        double angle = yaw + (i * MathUtils::PI / 2.0);
        Vec3 dir(cosPitch * std::cos(angle), cosPitch * std::sin(angle), sinPitchUp);
        Ray ray(position, dir);

        double minHitDist = maxRange;
        for (int32_t bIdx : candidates) {
            if (bIdx >= 0 && static_cast<size_t>(bIdx) < m_map.buildings.size()) {
                auto hit = raycastAABB(ray, m_map.buildings[bIdx].bounds, minHitDist);
                if (hit.hit && hit.distance < minHitDist) {
                    minHitDist = hit.distance;
                }
            }
        }
        beams.push_back(static_cast<float>(minHitDist / maxRange));
    }

    // 4 down beams at 90-degree quadrant angles
    for (int i = 0; i < 4; ++i) {
        double angle = yaw + (i * MathUtils::PI / 2.0);
        Vec3 dir(cosPitch * std::cos(angle), cosPitch * std::sin(angle), sinPitchDown);
        Ray ray(position, dir);

        double minHitDist = maxRange;
        for (int32_t bIdx : candidates) {
            if (bIdx >= 0 && static_cast<size_t>(bIdx) < m_map.buildings.size()) {
                auto hit = raycastAABB(ray, m_map.buildings[bIdx].bounds, minHitDist);
                if (hit.hit && hit.distance < minHitDist) {
                    minHitDist = hit.distance;
                }
            }
        }
        beams.push_back(static_cast<float>(minHitDist / maxRange));
    }

    return beams;
}

std::vector<float> Simulator::computeNearbyUavs(UAVId egoId, size_t maxNeighbors) const {
    std::vector<float> nearby;
    nearby.reserve(maxNeighbors * 6);

    const UAVState* ego = nullptr;
    for (const auto& u : m_fleet.getAllUAVs()) {
        if (u.id == egoId) {
            ego = &u;
            break;
        }
    }

    if (!ego) {
        nearby.resize(maxNeighbors * 6, 0.0f);
        return nearby;
    }

    struct NeighborDist {
        double distSq;
        const UAVState* uav;
    };
    std::vector<NeighborDist> neighbors;
    neighbors.reserve(m_fleet.size());

    for (const auto& other : m_fleet.getAllUAVs()) {
        if (other.id != egoId && other.mode != UAVMode::OFFLINE) {
            neighbors.push_back({ego->position.distanceSquared(other.position), &other});
        }
    }

    std::sort(neighbors.begin(), neighbors.end(), [](const NeighborDist& a, const NeighborDist& b) {
        return a.distSq < b.distSq;
    });

    for (size_t i = 0; i < maxNeighbors; ++i) {
        if (i < neighbors.size()) {
            const auto* other = neighbors[i].uav;
            Vec3 relPos = worldToBody(other->position - ego->position, ego->yaw);
            Vec3 relVel = worldToBody(other->velocity - ego->velocity, ego->yaw);
            nearby.push_back(static_cast<float>(relPos.x));
            nearby.push_back(static_cast<float>(relPos.y));
            nearby.push_back(static_cast<float>(relPos.z));
            nearby.push_back(static_cast<float>(relVel.x));
            nearby.push_back(static_cast<float>(relVel.y));
            nearby.push_back(static_cast<float>(relVel.z));
        } else {
            for (int k = 0; k < 6; ++k) {
                nearby.push_back(0.0f);
            }
        }
    }

    return nearby;
}

float Simulator::computeReward(const UAVState& uav, const UAVState& prevUav, bool reached, bool crashed) const {
    float reward = -0.01f; // Living penalty to incentivize timely completion

    if (crashed) {
        reward -= 50.0f;
    }

    if (reached) {
        reward += 20.0f;
    }

    double prevDist = prevUav.position.distance(prevUav.targetWaypoint);
    double currDist = uav.position.distance(uav.targetWaypoint);
    double progress = prevDist - currDist;

    // Progress shaping (bounded to avoid waypoint switch discontinuities)
    if (std::abs(progress) < 10.0) {
        reward += static_cast<float>(progress * 0.5);
    }

    // Speed / effort regularization
    reward -= static_cast<float>(uav.speed() * 0.001);

    return reward;
}

float Simulator::computeNavigationReward(
    const UAVState& uav, const UAVState& prevUav, bool succeeded, bool failed, double dtPolicy
) const {
    double progress = prevUav.position.distance(m_navigationTarget) -
                      uav.position.distance(m_navigationTarget);
    const double maxProgress = m_config.fleet.maxHorizontalSpeed * dtPolicy +
                               m_config.fleet.maxVerticalSpeed * dtPolicy;
    progress = std::clamp(progress, -maxProgress, maxProgress);

    float reward = -0.01f + static_cast<float>(0.1 * progress);
    if (succeeded) reward += 10.0f;
    if (failed) reward -= 10.0f;
    return reward;
}

bool Simulator::dispatchRequest(UAVId uavId, RequestId requestId) {
    if (uavId < 0 || static_cast<size_t>(uavId) >= m_fleet.size()) return false;
    auto it = std::find_if(m_pendingRequests.begin(), m_pendingRequests.end(),
                           [requestId](const DeliveryRequest& r) { return r.id == requestId; });
    if (it == m_pendingRequests.end()) return false;

    DeliveryRequest req = *it;
    if (req.status != RequestStatus::PENDING) return false;

    if (m_fleet.assignRequest(uavId, req)) {
        req.transitionTo(RequestStatus::ASSIGNED, m_clock.now());
        req.assignedUavId = uavId;
        m_logger.log(m_clock.now(), SimulationEventType::REQUEST_ASSIGNED, req.id, uavId);
        m_pendingRequests.erase(it);
        return true;
    }
    return false;
}

StepResult Simulator::stepCoordinator(double intervalSeconds) {
    const double dtPolicy = m_config.simulation.dtPolicy;
    const int steps = std::max(1, static_cast<int>(std::round(intervalSeconds / dtPolicy)));
    StepResult finalRes;
    for (int s = 0; s < steps; ++s) {
        finalRes = step({});
    }
    return finalRes;
}

} // namespace aerofleet

