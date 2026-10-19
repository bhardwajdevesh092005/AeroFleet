#include "physics/CollisionSystem.hpp"
#include <algorithm>
#include <cmath>

namespace aerofleet {

CollisionSystem::CollisionSystem(double uavRadius)
    : m_uavRadius(uavRadius > 0.05 ? uavRadius : 0.5) {}

bool CollisionSystem::checkStaticCollision(
    const Vec3& start,
    const Vec3& end,
    const CityMap& map,
    CollisionEvent& outEvent
) const {
    Capsule sweptCapsule(start, end, m_uavRadius);

    AABB queryBox(
        Vec3(std::min(start.x, end.x) - m_uavRadius,
             std::min(start.y, end.y) - m_uavRadius,
             std::min(start.z, end.z) - m_uavRadius),
        Vec3(std::max(start.x, end.x) + m_uavRadius,
             std::max(start.y, end.y) + m_uavRadius,
             std::max(start.z, end.z) + m_uavRadius)
    );

    const auto candidateIndices = map.broadphase.queryAABB(queryBox);
    for (int32_t idx : candidateIndices) {
        const auto& building = map.buildings[static_cast<size_t>(idx)];
        if (sweptCapsule.intersects(building.bounds)) {
            outEvent.type = CollisionType::BUILDING;
            outEvent.buildingId = building.id;

            // Approximate contact point on building surface
            Vec3 closestOnBuilding = building.bounds.closestPoint(end);
            outEvent.contactPoint = closestOnBuilding;
            Vec3 diff = end - closestOnBuilding;
            outEvent.contactNormal = diff.normalized();
            return true;
        }
    }
    return false;
}

bool CollisionSystem::checkDynamicCollision(
    const UAVState& uavA, const Vec3& prevPosA,
    const UAVState& uavB, const Vec3& prevPosB,
    CollisionEvent& outEvent
) const {
    const double combinedRadius = m_uavRadius + m_uavRadius;
    const double combinedRadiusSq = combinedRadius * combinedRadius;

    const double distSq = Capsule::segmentSegmentDistanceSquared(
        prevPosA, uavA.position,
        prevPosB, uavB.position
    );

    if (distSq <= combinedRadiusSq) {
        outEvent.type = CollisionType::UAV_UAV;
        // Strictly canonical sorting of pair IDs
        outEvent.uavIdA = std::min(uavA.id, uavB.id);
        outEvent.uavIdB = std::max(uavA.id, uavB.id);
        outEvent.contactPoint = (uavA.position + uavB.position) * 0.5;
        Vec3 diff = uavA.position - uavB.position;
        outEvent.contactNormal = diff.normalized();
        return true;
    }
    return false;
}

std::vector<CollisionEvent> CollisionSystem::evaluateAndResolve(
    std::vector<UAVState>& uavs,
    const std::vector<Vec3>& prevPositions,
    const CityMap& map,
    double currentTime
) {
    std::vector<CollisionEvent> events;

    // 1. Static building collisions
    for (size_t i = 0; i < uavs.size() && i < prevPositions.size(); ++i) {
        auto& uav = uavs[i];
        const auto& prevPos = prevPositions[i];

        CollisionEvent staticEvent;
        if (checkStaticCollision(prevPos, uav.position, map, staticEvent)) {
            staticEvent.uavIdA = uav.id;
            staticEvent.simulationTime = currentTime;
            events.push_back(staticEvent);

            // Roll back position and trigger emergency safety state
            uav.position = prevPos;
            uav.velocity = Vec3(0.0, 0.0, 0.0);
            uav.acceleration = Vec3(0.0, 0.0, 0.0);
            uav.mode = UAVMode::EMERGENCY;
        }
    }

    // 2. Dynamic UAV-UAV pairwise conflicts (canonical order i < j)
    for (size_t i = 0; i < uavs.size(); ++i) {
        for (size_t j = i + 1; j < uavs.size(); ++j) {
            auto& uavA = uavs[i];
            auto& uavB = uavs[j];
            const auto& prevA = prevPositions[i];
            const auto& prevB = prevPositions[j];

            CollisionEvent dynamicEvent;
            if (checkDynamicCollision(uavA, prevA, uavB, prevB, dynamicEvent)) {
                dynamicEvent.simulationTime = currentTime;
                events.push_back(dynamicEvent);

                // Deterministic rollback and emergency stop for both agents
                uavA.position = prevA;
                uavA.velocity = Vec3(0.0, 0.0, 0.0);
                uavA.mode = UAVMode::EMERGENCY;

                uavB.position = prevB;
                uavB.velocity = Vec3(0.0, 0.0, 0.0);
                uavB.mode = UAVMode::EMERGENCY;
            }
        }
    }

    return events;
}

} // namespace aerofleet

