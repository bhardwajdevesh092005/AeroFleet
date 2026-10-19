#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <iomanip>
#include <algorithm>

#include "core/Config.hpp"
#include "core/Types.hpp"
#include "core/WorldState.hpp"
#include "core/SimulationClock.hpp"
#include "math/Vec3.hpp"
#include "world/WorldGenerator.hpp"
#include "fleet/FleetManager.hpp"
#include "requests/RequestGenerator.hpp"
#include "control/WaypointController.hpp"
#include "coordinator/HeuristicCoordinator.hpp"
#include "physics/CollisionSystem.hpp"
#include "physics/Kinematics.hpp"
#include "visualization/Renderer.hpp"
#include "visualization/CameraController.hpp"
#include "telemetry/Metrics.hpp"
#include "telemetry/Logger.hpp"
#include <fstream>
#include <filesystem>

using namespace aerofleet;

namespace {

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

int main(int argc, char* argv[]) {
    std::string configPath = "config/simulation.json";
    std::string outputDir = "logs";
    bool headless = false;
    uint64_t maxSteps = 0; // 0 = unlimited
    double timeScale = 1.0;
    uint64_t seedOverride = 0;
    bool hasSeedOverride = false;
    bool logEvents = false;
    bool exportMetrics = true;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--config" && i + 1 < argc) {
            configPath = argv[++i];
        } else if (arg == "--headless") {
            headless = true;
        } else if (arg == "--steps" && i + 1 < argc) {
            maxSteps = std::stoull(argv[++i]);
        } else if (arg == "--speed" && i + 1 < argc) {
            timeScale = std::stod(argv[++i]);
        } else if (arg == "--seed" && i + 1 < argc) {
            seedOverride = std::stoull(argv[++i]);
            hasSeedOverride = true;
        } else if (arg == "--output" && i + 1 < argc) {
            outputDir = argv[++i];
        } else if (arg == "--log-events") {
            logEvents = true;
        } else if (arg == "--no-metrics") {
            exportMetrics = false;
        }
    }

    configPath = resolveConfigPath(configPath);

    std::cout << "========================================\n";
    std::cout << "      AeroFleet Simulation Engine       \n";
    std::cout << "========================================\n";
    std::cout << "[System] Configuration: " << configPath << "\n";

    SimulationConfig config;
    try {
        config = SimulationConfig::loadFromFile(configPath);
        if (hasSeedOverride) {
            config.simulation.randomSeed = seedOverride;
        }
        std::cout << "[System] Loaded: " << config.simulation.name << " v" << config.simulation.version << "\n";
        std::cout << "[System] World Volume: "
                  << config.world.xMin << ".." << config.world.xMax << " m (X), "
                  << config.world.yMin << ".." << config.world.yMax << " m (Y), "
                  << config.world.zMin << ".." << config.world.zMax << " m (Z)\n";
        std::cout << "[System] Fleet Size: " << config.fleet.numUavs << " UAVs\n";
        std::cout << "[System] Seed: " << config.simulation.randomSeed << "\n";
    } catch (const std::exception& ex) {
        std::cerr << "[Error] Configuration initialization failed: " << ex.what() << "\n";
        return 1;
    }

    // 1. Generate procedural city environment
    CityMap map = WorldGenerator::generate(config.world, config.simulation.randomSeed);
    std::cout << "[System] City Map Generated: " << map.buildings.size() << " buildings, "
              << map.stores.size() << " stores, " << map.chargingStations.size() << " charging docks.\n";

    // 2. Initialize simulation components
    FleetManager fleet(config.fleet, config.energy, map, config.world.defaultSpawnAltitude);
    RequestGenerator requestGen(config.requests, map, config.fleet.maxHorizontalSpeed,
                                config.simulation.randomSeed,
                                config.world.zMin + config.world.defaultSpawnAltitude);

    WaypointControllerConfig navConfig;
    navConfig.maxHorizontalSpeed = config.fleet.maxHorizontalSpeed;
    navConfig.maxVerticalSpeed = config.fleet.maxVerticalSpeed;
    WaypointController navigator(navConfig);

    HeuristicCoordinator coordinator;
    CollisionSystem collisions(config.fleet.uavRadius);
    KinematicsIntegrator integrator(KinematicsConfig::fromConfig(config.fleet, config.world));
    SimulationClock clock(config.simulation.dtPhysics, timeScale);

    std::vector<DeliveryRequest> pendingRequests;
    std::vector<DeliveryRequest> deliveredRequests;
    std::vector<DeliveryRequest> requeuedRequests;
    std::vector<CollisionEvent> allCollisionEvents;

    double lastCoordinatorDispatchTime = -10.0;
    size_t totalCreated = 0;

    MetricsTracker metrics;
    EventLogger logger(131072); // High-speed 128k event ring buffer

    // 3. Initialize visualizer (unless headless requested)
    RendererConfig rendererConfig;
    rendererConfig.targetFPS = 60;
    Renderer renderer(rendererConfig);
    CameraController camera(Vec3(0.0, 0.0, 15.0), 350.0);

    bool visualMode = !headless;
    if (visualMode) {
        if (!renderer.init()) {
            std::cout << "[System] Graphical display not available; switching to headless execution.\n";
            visualMode = false;
        } else {
            std::cout << "[System] 3D raylib renderer initialized successfully.\n";
        }
    } else {
        std::cout << "[System] Running in headless mode.\n";
    }

    // Previous positions vector for continuous collision detection
    std::vector<Vec3> prevPositions(fleet.size());
    for (size_t i = 0; i < fleet.size(); ++i) {
        prevPositions[i] = fleet.getUAV(i).position;
    }

    WorldSnapshot prevSnapshot;
    WorldSnapshot currSnapshot;

    auto wallClockStart = std::chrono::steady_clock::now();
    auto lastFrameTime = wallClockStart;

    std::cout << "[System] Simulation running...\n";

    // Master Simulation Loop
    while (true) {
        if (visualMode && renderer.shouldClose()) {
            break;
        }
        if (maxSteps > 0 && clock.getStepCount() >= maxSteps) {
            break;
        }

        // Measure real elapsed frame time
        auto currentFrameTime = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = currentFrameTime - lastFrameTime;
        lastFrameTime = currentFrameTime;

        if (visualMode) {
            clock.update(elapsed.count());
        } else {
            // In headless mode, step as fast as possible without real-world throttling
            clock.advance();
        }

        // Integrate fixed physics steps
        while (!visualMode || clock.hasStep()) {
            const double dt = clock.getDtPhysics();
            const double currentTime = clock.now();

            // Record previous positions for CCD
            for (size_t i = 0; i < fleet.size(); ++i) {
                prevPositions[i] = fleet.getUAV(i).position;
            }

            // 1. Generate stochastic Poisson customer delivery requests
            auto newRequests = requestGen.step(currentTime, dt);
            if (!newRequests.empty()) {
                totalCreated += newRequests.size();
                for (const auto& req : newRequests) {
                    metrics.recordRequestCreated(req);
                    if (logEvents) {
                        logger.log(currentTime, SimulationEventType::REQUEST_CREATED,
                                   req.id, req.storeId, req.payloadMass);
                    }
                }
                pendingRequests.insert(pendingRequests.end(), newRequests.begin(), newRequests.end());
            }

            // Expire requests while they are still waiting. Assigned requests
            // leave this queue immediately after dispatch below.
            for (auto it = pendingRequests.begin(); it != pendingRequests.end();) {
                const bool timedOut = currentTime > it->creationTime + config.requests.requestTimeoutSeconds;
                if (it->status == RequestStatus::PENDING && (it->isExpired(currentTime) || timedOut)) {
                    it->transitionTo(RequestStatus::EXPIRED, currentTime);
                    metrics.recordRequestExpired(*it, currentTime);
                    if (logEvents) logger.log(currentTime, SimulationEventType::REQUEST_EXPIRED, it->id);
                    it = pendingRequests.erase(it);
                } else {
                    ++it;
                }
            }

            // 2. Dispatch coordinator at coordinator rate (e.g. 1 Hz)
            if (currentTime - lastCoordinatorDispatchTime >= config.simulation.dtCoordinator) {
                coordinator.dispatch(fleet, pendingRequests, map, currentTime);
                for (const auto& req : pendingRequests) {
                    if (req.status == RequestStatus::ASSIGNED && logEvents) {
                        logger.log(currentTime, SimulationEventType::REQUEST_ASSIGNED,
                                   req.id, req.assignedUavId);
                    }
                }
                pendingRequests.erase(
                    std::remove_if(pendingRequests.begin(), pendingRequests.end(),
                                   [](const DeliveryRequest& req) { return req.status != RequestStatus::PENDING; }),
                    pendingRequests.end());
                lastCoordinatorDispatchTime = currentTime;
            }

            // 3. Step fleet lifecycle (dwell times, state changes, charging)
            const size_t prevDeliveredCount = deliveredRequests.size();
            fleet.step(dt, currentTime, map, deliveredRequests, requeuedRequests);
            for (size_t d = prevDeliveredCount; d < deliveredRequests.size(); ++d) {
                metrics.recordRequestDelivered(deliveredRequests[d], currentTime);
                if (logEvents) {
                    logger.log(currentTime, SimulationEventType::REQUEST_DELIVERED,
                               deliveredRequests[d].id, deliveredRequests[d].assignedUavId,
                               deliveredRequests[d].deliveryTime - deliveredRequests[d].creationTime);
                }
            }
            if (!requeuedRequests.empty()) {
                for (const auto& req : requeuedRequests) {
                    if (logEvents) {
                        logger.log(currentTime, SimulationEventType::REQUEST_REQUEUED, req.id, req.assignedUavId);
                    }
                }
                pendingRequests.insert(pendingRequests.end(), requeuedRequests.begin(), requeuedRequests.end());
                requeuedRequests.clear();
            }

            // 4. Compute navigation actions and integrate rigid kinematics
            double totalPowerWatts = 0.0;
            const std::vector<UAVState> trafficSnapshot = fleet.getAllUAVs();
            for (auto& uav : fleet.getAllUAVs()) {
                VelocityCommand action;
                if (uav.mode == UAVMode::FLYING_TO_PICKUP ||
                    uav.mode == UAVMode::FLYING_TO_DESTINATION ||
                    uav.mode == UAVMode::RETURNING_TO_CHARGE) {
                    action = navigator.computeActionWithTraffic(uav, map, trafficSnapshot, dt);
                }
                integrator.stepVelocityCommand(uav, action, dt);
                fleet.getBatteryModel().stepDischarge(uav, dt);
                totalPowerWatts += fleet.getBatteryModel().computePowerWatts(uav.velocity, uav.payloadMass);
            }
            metrics.recordStep(fleet.getAllUAVs(), dt, totalPowerWatts, currentTime);

            // 5. Continuous collision detection and deterministic rollback
            auto newCollisions = collisions.evaluateAndResolve(fleet.getAllUAVs(), prevPositions, map, currentTime);
            if (!newCollisions.empty()) {
                for (const auto& c : newCollisions) {
                    metrics.recordCollision(c);
                    if (logEvents) {
                        logger.log(currentTime,
                                   c.type == CollisionType::BUILDING ? SimulationEventType::COLLISION_BUILDING : SimulationEventType::COLLISION_UAV,
                                   c.uavIdA,
                                   c.type == CollisionType::BUILDING ? c.buildingId : c.uavIdB);
                    }

                    const auto abortCrashedAssignment = [&](UAVId id) {
                        DeliveryRequest requeued;
                        if (fleet.abortAssignment(id, requeued)) {
                            requeuedRequests.push_back(requeued);
                            if (logEvents) logger.log(currentTime, SimulationEventType::REQUEST_REQUEUED,
                                                      requeued.id, id);
                        }
                        if (id >= 0 && static_cast<size_t>(id) < fleet.size()) {
                            fleet.getUAV(static_cast<size_t>(id)).mode = UAVMode::EMERGENCY;
                        }
                    };
                    abortCrashedAssignment(c.uavIdA);
                    if (c.type == CollisionType::UAV_UAV) abortCrashedAssignment(c.uavIdB);
                }
                if (!requeuedRequests.empty()) {
                    pendingRequests.insert(pendingRequests.end(), requeuedRequests.begin(), requeuedRequests.end());
                    requeuedRequests.clear();
                }
                allCollisionEvents.insert(allCollisionEvents.end(), newCollisions.begin(), newCollisions.end());
            }

            if (visualMode) {
                clock.advance();
            } else {
                break; // One step completed in headless iteration
            }
        }

        // Build snapshot
        prevSnapshot = currSnapshot;
        currSnapshot.simulationTime = clock.now();
        currSnapshot.wallTime = std::chrono::duration<double>(std::chrono::steady_clock::now() - wallClockStart).count();
        currSnapshot.stepCount = clock.getStepCount();
        currSnapshot.uavs = fleet.getAllUAVs();
        currSnapshot.chargingStations = map.chargingStations;
        currSnapshot.activeRequests = pendingRequests;
        currSnapshot.recentEvents = allCollisionEvents;

        currSnapshot.metrics.totalCreated = totalCreated;
        currSnapshot.metrics.totalDelivered = deliveredRequests.size();
        currSnapshot.metrics.totalPending = pendingRequests.size();
        currSnapshot.metrics.totalActive = fleet.countInMode(UAVMode::FLYING_TO_PICKUP) +
                                           fleet.countInMode(UAVMode::PICKING_UP) +
                                           fleet.countInMode(UAVMode::FLYING_TO_DESTINATION) +
                                           fleet.countInMode(UAVMode::DROPPING_OFF);
        currSnapshot.metrics.collisionCount = allCollisionEvents.size();
        currSnapshot.metrics.deliverySuccessRate = metrics.getDeadlineSuccessRate();
        currSnapshot.metrics.averageDeliveryDuration = metrics.getMeanLatency();

        // Render frame with sub-step interpolation
        if (visualMode) {
            WorldSnapshot renderSnapshot = WorldSnapshot::interpolate(prevSnapshot, currSnapshot, clock.getAlpha());
            renderer.draw(renderSnapshot, map, camera);
        }
    }

    auto totalElapsedWall = std::chrono::duration<double>(std::chrono::steady_clock::now() - wallClockStart).count();

    metrics.finalize(totalElapsedWall);
    metrics.printSummary(std::cout);

    if (exportMetrics && !outputDir.empty()) {
        std::error_code ec;
        std::filesystem::create_directories(outputDir, ec);
        std::string jsonPath = outputDir + "/metrics.json";
        std::string csvPath = outputDir + "/metrics.csv";
        if (metrics.saveToJsonFile(jsonPath)) {
            std::cout << "[System] Performance metrics saved: " << jsonPath << "\n";
        }
        if (metrics.saveToCsvFile(csvPath)) {
            std::cout << "[System] Performance metrics saved: " << csvPath << "\n";
        }
    }

    if (logEvents && !outputDir.empty()) {
        std::error_code ec;
        std::filesystem::create_directories(outputDir, ec);
        std::string eventsPath = outputDir + "/simulation_events.jsonl";
        if (logger.flushToFile(eventsPath)) {
            std::cout << "[System] Telemetry events logged: " << eventsPath
                      << " (" << logger.totalLogged() << " events)\n";
        }
    }

    renderer.close();
    return 0;
}
