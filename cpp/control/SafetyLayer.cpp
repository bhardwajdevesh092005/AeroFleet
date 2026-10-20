#include "control/SafetyLayer.hpp"

#include <algorithm>
#include <cmath>

namespace aerofleet {

SafetyLayer::SafetyLayer(const SafetyLayerConfig& config)
    : m_config(config) {}

bool SafetyLayer::isTrajectorySafe(
    const UAVState& uav,
    const CityMap& map,
    const VelocityCommand& cmd,
    double horizon
) const {
    const double effectiveRadius = m_config.uavRadius + m_config.safetyMargin;
    const double effectiveRadiusSq = effectiveRadius * effectiveRadius;
    const double uavRadiusSq = m_config.uavRadius * m_config.uavRadius;
    const Vec3 vel = cmd.desiredVelocity;
    const double speed = vel.length();
    const Vec3 startPos = uav.position;

    const double queryRadius = speed * horizon + effectiveRadius + 5.0;
    const auto candidates = map.broadphase.querySphere(startPos, queryRadius);

    const int numSteps = std::max(1, static_cast<int>(std::ceil(horizon / m_config.timeStep)));
    const double dtStep = horizon / numSteps;

    for (int step = 1; step <= numSteps; ++step) {
        const double t = step * dtStep;
        const Vec3 pt = startPos + vel * t;

        // Check world bounds
        if (pt.x < map.worldMin.x + effectiveRadius ||
            pt.x > map.worldMax.x - effectiveRadius ||
            pt.y < map.worldMin.y + effectiveRadius ||
            pt.y > map.worldMax.y - effectiveRadius ||
            pt.z < map.worldMin.z ||
            pt.z > map.worldMax.z - effectiveRadius) {
            return false;
        }

        // Check building obstacles
        for (int32_t bIdx : candidates) {
            if (bIdx >= 0 && static_cast<size_t>(bIdx) < map.buildings.size()) {
                const auto& b = map.buildings[static_cast<size_t>(bIdx)];
                const double dSq = b.bounds.distanceSquared(pt);
                if (dSq < uavRadiusSq) {
                    return false; // Physical boundary penetration
                }
                if (dSq < effectiveRadiusSq) {
                    const double initDSq = b.bounds.distanceSquared(startPos);
                    // If within safety buffer, moving CLOSER to the obstacle is unsafe
                    if (dSq < initDSq) {
                        return false;
                    }
                }
            }
        }
    }

    return true;
}

VelocityCommand SafetyLayer::filterAction(
    const UAVState& uav,
    const CityMap& map,
    const VelocityCommand& candidate,
    double /*dt*/
) {
    const double distToTarget = uav.position.distance(uav.targetWaypoint);
    const double speed = candidate.desiredVelocity.length();
    double effectiveHorizon = m_config.lookaheadTime;
    if (speed > 0.1 && distToTarget > 0.1) {
        effectiveHorizon = std::min(effectiveHorizon, (distToTarget + m_config.safetyMargin) / speed);
    }

    if (isTrajectorySafe(uav, map, candidate, effectiveHorizon)) {
        return candidate;
    }

    ++m_interventions;

    // First evasive option: Emergency climb if ceiling allows
    VelocityCommand climb{Vec3(0.0, 0.0, m_config.emergencyClimbSpeed)};
    if (isTrajectorySafe(uav, map, climb, effectiveHorizon)) {
        return climb;
    }

    // Second evasive option: Repulsive deflection away from closest obstacle combined with vertical climb
    const auto candidates = map.broadphase.querySphere(uav.position, 20.0);
    Vec3 away(0.0, 0.0, 0.0);
    double closestDistSq = 1e9;
    for (int32_t bIdx : candidates) {
        if (bIdx >= 0 && static_cast<size_t>(bIdx) < map.buildings.size()) {
            const auto& b = map.buildings[static_cast<size_t>(bIdx)];
            Vec3 cp = b.bounds.closestPoint(uav.position);
            Vec3 diff = uav.position - cp;
            double dSq = diff.lengthSquared();
            if (dSq < closestDistSq) {
                closestDistSq = dSq;
                away = diff;
            }
        }
    }
    if (away.lengthSquared() > 1e-6) {
        Vec3 awayDir = away.normalized();
        Vec3 deflect = awayDir * 5.0 + Vec3(0.0, 0.0, m_config.emergencyClimbSpeed);
        VelocityCommand evasive{deflect};
        if (isTrajectorySafe(uav, map, evasive, m_config.lookaheadTime * 0.5)) {
            return evasive;
        }
    }

    // Third evasive option: Hover in place
    return VelocityCommand{Vec3(0.0, 0.0, 0.0)};
}

} // namespace aerofleet
