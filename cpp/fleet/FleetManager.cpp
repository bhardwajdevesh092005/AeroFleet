#include "fleet/FleetManager.hpp"
#include <algorithm>
#include <limits>

namespace aerofleet {

FleetManager::FleetManager(
    const FleetSettings& fleetSettings,
    const EnergySettings& energySettings,
    const CityMap& map,
    double defaultAltitude
) : m_settings(fleetSettings),
    m_energySettings(energySettings),
    m_batteryModel(fleetSettings, energySettings) {
    initialize(fleetSettings, energySettings, map, defaultAltitude);
}

void FleetManager::initialize(
    const FleetSettings& fleetSettings,
    const EnergySettings& energySettings,
    const CityMap& map,
    double defaultAltitude
) {
    m_settings = fleetSettings;
    m_energySettings = energySettings;
    m_batteryModel = BatteryModel(fleetSettings, energySettings);

    m_uavs.clear();
    m_contexts.clear();

    const size_t numDrones = static_cast<size_t>(std::max(1, fleetSettings.numUavs));
    m_uavs.reserve(numDrones);

    for (size_t i = 0; i < numDrones; ++i) {
        Vec3 spawnPos(0.0, 0.0, defaultAltitude);
        if (!map.chargingStations.empty()) {
            size_t stationIdx = i % map.chargingStations.size();
            spawnPos = map.chargingStations[stationIdx].position;
            // Ground level docking pad
            spawnPos.z = map.chargingStations[stationIdx].position.z;
        }

        UAVState uav(static_cast<UAVId>(i), spawnPos, 0.0);
        uav.batteryPercent = 100.0;
        uav.payloadMass = 0.0;
        uav.mode = UAVMode::IDLE;
        uav.activeRequestId = -1;
        uav.targetWaypoint = spawnPos;
        uav.isGrounded = true;

        UAVContext ctx;
        ctx.dwellTimer = 0.0;
        ctx.assignedRequestId = -1;
        ctx.homeBasePosition = spawnPos;
        ctx.targetChargerId = -1;

        m_uavs.push_back(uav);
        m_contexts[static_cast<UAVId>(i)] = ctx;
    }
}

bool FleetManager::assignRequest(UAVId uavId, const DeliveryRequest& request) {
    auto ctxIt = m_contexts.find(uavId);
    if (ctxIt == m_contexts.end()) return false;

    if (static_cast<size_t>(uavId) >= m_uavs.size()) return false;
    auto& uav = m_uavs[static_cast<size_t>(uavId)];

    if (uav.mode != UAVMode::IDLE) {
        return false; // Busy
    }

    if (m_batteryModel.isReserve(uav.batteryPercent)) {
        return false; // Insufficient battery to accept new missions
    }

    uav.mode = UAVMode::ASSIGNED;
    uav.activeRequestId = request.id;
    if (uav.position.z < 10.0) {
        uav.targetWaypoint = Vec3(uav.position.x, uav.position.y, 15.0);
    } else {
        uav.targetWaypoint = request.pickupLocation;
    }

    auto& ctx = ctxIt->second;
    ctx.assignedRequestId = request.id;
    ctx.activeRequest = request;
    ctx.activeRequest.transitionTo(RequestStatus::ASSIGNED);
    ctx.activeRequest.assignedUavId = uav.id;
    ctx.dwellTimer = 0.0;

    return true;
}

bool FleetManager::abortAssignment(UAVId uavId, DeliveryRequest& outRequeuedRequest) {
    auto ctxIt = m_contexts.find(uavId);
    if (ctxIt == m_contexts.end()) return false;

    if (static_cast<size_t>(uavId) >= m_uavs.size()) return false;
    auto& uav = m_uavs[static_cast<size_t>(uavId)];
    auto& ctx = ctxIt->second;

    if (ctx.assignedRequestId == -1) {
        return false;
    }

    // If payload has not been picked up, re-queue request
    if (ctx.activeRequest.status == RequestStatus::ASSIGNED ||
        ctx.activeRequest.status == RequestStatus::PICKING_UP ||
        ctx.activeRequest.status == RequestStatus::IN_TRANSIT) {
        ctx.activeRequest.transitionTo(RequestStatus::PENDING);
        outRequeuedRequest = ctx.activeRequest;
    }

    uav.activeRequestId = -1;
    uav.payloadMass = 0.0;
    ctx.assignedRequestId = -1;
    ctx.dwellTimer = 0.0;
    uav.mode = UAVMode::RETURNING_TO_CHARGE;
    return true;
}

int32_t FleetManager::findNearestAvailableCharger(const Vec3& position, const CityMap& map) const {
    int32_t bestId = -1;
    double bestDistSq = std::numeric_limits<double>::infinity();

    for (const auto& station : map.chargingStations) {
        if (station.hasAvailableSlot()) {
            double dSq = position.distanceSquared(station.position);
            if (dSq < bestDistSq) {
                bestDistSq = dSq;
                bestId = station.id;
            }
        }
    }

    // Fall back to any station if all report full
    if (bestId == -1 && !map.chargingStations.empty()) {
        for (const auto& station : map.chargingStations) {
            double dSq = position.distanceSquared(station.position);
            if (dSq < bestDistSq) {
                bestDistSq = dSq;
                bestId = station.id;
            }
        }
    }

    return bestId;
}

void FleetManager::step(
    double dt,
    double currentTime,
    CityMap& map,
    std::vector<DeliveryRequest>& outDeliveredRequests,
    std::vector<DeliveryRequest>& outRequeuedRequests
) {
    if (dt <= 0.0) return;

    for (auto& uav : m_uavs) {
        auto& ctx = m_contexts[uav.id];

        // 1. Critical battery emergency check
        if (m_batteryModel.isCritical(uav.batteryPercent)) {
            if (uav.mode != UAVMode::EMERGENCY && uav.mode != UAVMode::CHARGING) {
                DeliveryRequest req;
                if (abortAssignment(uav.id, req)) {
                    outRequeuedRequests.push_back(req);
                }
                uav.mode = UAVMode::EMERGENCY;
                uav.velocity = Vec3(0.0, 0.0, 0.0);
                uav.acceleration = Vec3(0.0, 0.0, 0.0);
                continue;
            }
        } else if (m_batteryModel.isReserve(uav.batteryPercent)) {
            // Reserve battery low-power trigger: in-flight RTH safety abort
            if (uav.mode == UAVMode::ASSIGNED ||
                uav.mode == UAVMode::FLYING_TO_PICKUP ||
                uav.mode == UAVMode::PICKING_UP) {
                DeliveryRequest req;
                if (abortAssignment(uav.id, req)) {
                    outRequeuedRequests.push_back(req);
                }
                uav.mode = UAVMode::RETURNING_TO_CHARGE;
            }
        }

        // 2. State-specific lifecycle transitions
        switch (uav.mode) {
            case UAVMode::ASSIGNED:
                handleAssigned(uav, ctx);
                break;
            case UAVMode::FLYING_TO_PICKUP:
                handleFlyingToPickup(uav, ctx, map, dt);
                break;
            case UAVMode::PICKING_UP:
                handlePickingUp(uav, ctx, dt, currentTime);
                break;
            case UAVMode::FLYING_TO_DESTINATION:
                handleFlyingToDestination(uav, ctx, map, dt);
                break;
            case UAVMode::DROPPING_OFF:
                handleDroppingOff(uav, ctx, dt, currentTime, outDeliveredRequests);
                break;
            case UAVMode::RETURNING_TO_CHARGE:
                handleReturningToCharge(uav, ctx, map);
                break;
            case UAVMode::CHARGING:
                handleCharging(uav, ctx, map, dt);
                break;
            case UAVMode::IDLE:
                // Proactive battery management: if idle and battery <= 25% or below reserve, return to charge
                if (uav.batteryPercent <= 25.0 || m_batteryModel.isReserve(uav.batteryPercent)) {
                    uav.mode = UAVMode::RETURNING_TO_CHARGE;
                }
                break;
            case UAVMode::EMERGENCY: {
                if (uav.batteryPercent >= 80.0) {
                    uav.mode = UAVMode::IDLE;
                    uav.velocity = Vec3(0, 0, 0);
                    uav.acceleration = Vec3(0, 0, 0);
                } else {
                    for (auto& s : map.chargingStations) {
                        double dist = uav.position.distance(s.position);
                        if (dist <= 15.0 && s.hasAvailableSlot()) {
                            uav.position = s.position;
                            uav.velocity = Vec3(0, 0, 0);
                            uav.acceleration = Vec3(0, 0, 0);
                            if (m_batteryModel.dock(uav, s)) {
                                ctx.targetChargerId = s.id;
                                uav.mode = UAVMode::CHARGING;
                                break;
                            }
                        }
                    }
                }
                break;
            }
            case UAVMode::OFFLINE:
            default:
                break;
        }
    }
}

void FleetManager::handleAssigned(UAVState& uav, UAVContext& ctx) {
    uav.mode = UAVMode::FLYING_TO_PICKUP;
    if (uav.position.z < 10.0) {
        uav.targetWaypoint = Vec3(ctx.homeBasePosition.x, ctx.homeBasePosition.y, 15.0);
    } else {
        uav.targetWaypoint = ctx.activeRequest.pickupLocation;
    }
    uav.isGrounded = false;
}

void FleetManager::handleFlyingToPickup(UAVState& uav, UAVContext& ctx, const CityMap& map, double /*dt*/) {
    if (uav.position.z < 10.0) {
        // Maintain vertical climb corridor to clear ground charging station
        uav.targetWaypoint = Vec3(ctx.homeBasePosition.x, ctx.homeBasePosition.y, 15.0);
        return;
    }

    if (ctx.activeRequest.id == -1 || ctx.assignedRequestId == -1) {
        uav.mode = UAVMode::IDLE;
        return;
    }

    const Vec3& pickup = ctx.activeRequest.pickupLocation;
    const double horizDist = std::hypot(uav.position.x - pickup.x, uav.position.y - pickup.y);
    if (!map.buildings.empty() && horizDist > 20.0) {
        double localMaxHeight = 35.0;
        const double minX = std::min(uav.position.x, pickup.x) - 15.0;
        const double maxX = std::max(uav.position.x, pickup.x) + 15.0;
        const double minY = std::min(uav.position.y, pickup.y) - 15.0;
        const double maxY = std::max(uav.position.y, pickup.y) + 15.0;
        for (const auto& b : map.buildings) {
            if (b.bounds.maxPoint.x >= minX && b.bounds.minPoint.x <= maxX &&
                b.bounds.maxPoint.y >= minY && b.bounds.minPoint.y <= maxY) {
                localMaxHeight = std::max(localMaxHeight, b.bounds.maxPoint.z + 5.0);
            }
        }
        const double cruiseAltitude = std::min(map.worldMax.z - 5.0, localMaxHeight);
        uav.targetWaypoint = Vec3(pickup.x, pickup.y, cruiseAltitude);
    } else {
        uav.targetWaypoint = pickup;
    }

    double dist = uav.position.distance(ctx.activeRequest.pickupLocation);
    if (dist <= 3.0) {
        uav.mode = UAVMode::PICKING_UP;
        ctx.dwellTimer = 0.0;
        uav.velocity = Vec3(0.0, 0.0, 0.0);
    }
}

void FleetManager::handlePickingUp(UAVState& uav, UAVContext& ctx, double dt, double currentTime) {
    if (ctx.activeRequest.id == -1 || ctx.assignedRequestId == -1) {
        uav.mode = UAVMode::IDLE;
        return;
    }

    ctx.dwellTimer += dt;
    if (ctx.activeRequest.status == RequestStatus::ASSIGNED) {
        ctx.activeRequest.transitionTo(RequestStatus::PICKING_UP, currentTime);
    }

    if (ctx.dwellTimer >= m_settings.pickupDwellSeconds) {
        uav.payloadMass = ctx.activeRequest.payloadMass;
        ctx.activeRequest.transitionTo(RequestStatus::IN_TRANSIT, currentTime);
        uav.mode = UAVMode::FLYING_TO_DESTINATION;
        uav.targetWaypoint = ctx.activeRequest.dropoffLocation;
        ctx.dwellTimer = 0.0;
    }
}

void FleetManager::handleFlyingToDestination(UAVState& uav, UAVContext& ctx, const CityMap& map, double /*dt*/) {
    if (ctx.activeRequest.id == -1 || ctx.assignedRequestId == -1) {
        // Waypoint navigation without delivery request (e.g., flight demo or repositioning)
        double dist = uav.position.distance(uav.targetWaypoint);
        if (dist <= 3.0) {
            uav.velocity = Vec3(0.0, 0.0, 0.0);
            if (uav.batteryPercent <= 25.0 || m_batteryModel.isReserve(uav.batteryPercent)) {
                uav.mode = UAVMode::RETURNING_TO_CHARGE;
            } else {
                uav.mode = UAVMode::IDLE;
                uav.targetWaypoint = ctx.homeBasePosition;
            }
        }
        return;
    }

    const Vec3& dropoff = ctx.activeRequest.dropoffLocation;
    const double horizDist = std::hypot(uav.position.x - dropoff.x, uav.position.y - dropoff.y);
    if (!map.buildings.empty() && horizDist > 20.0) {
        double localMaxHeight = 35.0;
        const double minX = std::min(uav.position.x, dropoff.x) - 15.0;
        const double maxX = std::max(uav.position.x, dropoff.x) + 15.0;
        const double minY = std::min(uav.position.y, dropoff.y) - 15.0;
        const double maxY = std::max(uav.position.y, dropoff.y) + 15.0;
        for (const auto& b : map.buildings) {
            if (b.bounds.maxPoint.x >= minX && b.bounds.minPoint.x <= maxX &&
                b.bounds.maxPoint.y >= minY && b.bounds.minPoint.y <= maxY) {
                localMaxHeight = std::max(localMaxHeight, b.bounds.maxPoint.z + 5.0);
            }
        }
        const double cruiseAltitude = std::min(map.worldMax.z - 5.0, localMaxHeight);
        uav.targetWaypoint = Vec3(dropoff.x, dropoff.y, cruiseAltitude);
    } else {
        uav.targetWaypoint = dropoff;
    }

    double dist = uav.position.distance(ctx.activeRequest.dropoffLocation);
    if (dist <= 3.0) {
        uav.mode = UAVMode::DROPPING_OFF;
        ctx.dwellTimer = 0.0;
        uav.velocity = Vec3(0.0, 0.0, 0.0);
    }
}

void FleetManager::handleDroppingOff(
    UAVState& uav,
    UAVContext& ctx,
    double dt,
    double currentTime,
    std::vector<DeliveryRequest>& outDelivered
) {
    ctx.dwellTimer += dt;
    if (ctx.dwellTimer >= m_settings.dropoffDwellSeconds) {
        uav.payloadMass = 0.0;
        if (ctx.activeRequest.id != -1 && ctx.activeRequest.canTransitionTo(RequestStatus::DELIVERED)) {
            ctx.activeRequest.transitionTo(RequestStatus::DELIVERED, currentTime);
            outDelivered.push_back(ctx.activeRequest);
        }

        uav.activeRequestId = -1;
        ctx.assignedRequestId = -1;
        ctx.dwellTimer = 0.0;

        if (uav.batteryPercent <= 25.0 || m_batteryModel.isReserve(uav.batteryPercent)) {
            uav.mode = UAVMode::RETURNING_TO_CHARGE;
        } else {
            uav.mode = UAVMode::IDLE;
            uav.targetWaypoint = ctx.homeBasePosition;
        }
    }
}

void FleetManager::handleReturningToCharge(UAVState& uav, UAVContext& ctx, CityMap& map) {
    if (ctx.targetChargerId == -1) {
        ctx.targetChargerId = findNearestAvailableCharger(uav.position, map);
    }

    ChargingStation* targetStation = nullptr;
    for (auto& station : map.chargingStations) {
        if (station.id == ctx.targetChargerId) {
            targetStation = &station;
            break;
        }
    }

    // Dynamic station re-routing if target station became full
    if (targetStation && !targetStation->hasAvailableSlot()) {
        int32_t altId = findNearestAvailableCharger(uav.position, map);
        if (altId != -1 && altId != ctx.targetChargerId) {
            ctx.targetChargerId = altId;
            for (auto& station : map.chargingStations) {
                if (station.id == ctx.targetChargerId) {
                    targetStation = &station;
                    break;
                }
            }
        }
    }

    if (!targetStation && !map.chargingStations.empty()) {
        targetStation = &map.chargingStations.front();
        ctx.targetChargerId = targetStation->id;
    }

    if (targetStation) {
        double localMaxHeight = 35.0; // Standard transit clearance
        const double minX = std::min(uav.position.x, targetStation->position.x) - 15.0;
        const double maxX = std::max(uav.position.x, targetStation->position.x) + 15.0;
        const double minY = std::min(uav.position.y, targetStation->position.y) - 15.0;
        const double maxY = std::max(uav.position.y, targetStation->position.y) + 15.0;
        for (const auto& b : map.buildings) {
            if (b.bounds.maxPoint.x >= minX && b.bounds.minPoint.x <= maxX &&
                b.bounds.maxPoint.y >= minY && b.bounds.minPoint.y <= maxY) {
                localMaxHeight = std::max(localMaxHeight, b.bounds.maxPoint.z + 5.0);
            }
        }
        const double rthAltitude = std::min(map.worldMax.z - 5.0, localMaxHeight);

        const double horizDist = std::hypot(uav.position.x - targetStation->position.x,
                                            uav.position.y - targetStation->position.y);
        if (horizDist > 15.0) {
            uav.targetWaypoint = Vec3(targetStation->position.x, targetStation->position.y, rthAltitude);
        } else {
            uav.targetWaypoint = targetStation->position;
        }

        if (m_batteryModel.isInDockingRange(uav, *targetStation)) {
            uav.velocity = Vec3(0, 0, 0);
            uav.acceleration = Vec3(0, 0, 0);
            if (m_batteryModel.dock(uav, *targetStation)) {
                uav.mode = UAVMode::CHARGING;
            } else if (!targetStation->hasAvailableSlot()) {
                ctx.targetChargerId = findNearestAvailableCharger(uav.position, map);
            }
        }
    }
}

void FleetManager::handleCharging(UAVState& uav, UAVContext& ctx, CityMap& map, double dt) {
    m_batteryModel.stepCharge(uav, dt);

    if (uav.batteryPercent >= 99.9) {
        uav.batteryPercent = 100.0;
        ChargingStation* station = nullptr;
        for (auto& s : map.chargingStations) {
            if (s.id == ctx.targetChargerId) {
                station = &s;
                break;
            }
        }
        if (station) {
            m_batteryModel.undock(uav, *station);
        }
        ctx.targetChargerId = -1;
        uav.mode = UAVMode::IDLE;
        uav.targetWaypoint = ctx.homeBasePosition;
    }
}

const UAVContext& FleetManager::getContext(UAVId uavId) const {
    return m_contexts.at(uavId);
}

UAVContext& FleetManager::getContext(UAVId uavId) {
    return m_contexts.at(uavId);
}

size_t FleetManager::countInMode(UAVMode mode) const noexcept {
    size_t count = 0;
    for (const auto& u : m_uavs) {
        if (u.mode == mode) count++;
    }
    return count;
}

void FleetManager::setUAVBatteryPercent(UAVId id, double percent) {
    if (static_cast<size_t>(id) < m_uavs.size()) {
        auto& u = m_uavs[static_cast<size_t>(id)];
        u.batteryPercent = std::clamp(percent, 0.0, 100.0);
        if (u.batteryPercent >= 80.0 && (u.mode == UAVMode::EMERGENCY || u.mode == UAVMode::RETURNING_TO_CHARGE)) {
            u.mode = UAVMode::IDLE;
            u.velocity = Vec3(0, 0, 0);
            u.acceleration = Vec3(0, 0, 0);
        }
    }
}

double FleetManager::getUAVBatteryPercent(UAVId id) const {
    if (static_cast<size_t>(id) < m_uavs.size()) {
        return m_uavs[static_cast<size_t>(id)].batteryPercent;
    }
    return 0.0;
}

} // namespace aerofleet

