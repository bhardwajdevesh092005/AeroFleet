#include "control/WaypointController.hpp"

#include <algorithm>
#include <cmath>

namespace aerofleet {
namespace {

double sanitizePositive(double value, double fallback) noexcept {
    return std::isfinite(value) && value > 0.0 ? value : fallback;
}

Vec3 clampHorizontal(const Vec3& value, double maxSpeed) noexcept {
    Vec3 result = value;
    const double horizontalSq = result.x * result.x + result.y * result.y;
    const double maxSq = maxSpeed * maxSpeed;
    if (horizontalSq > maxSq && horizontalSq > 1e-12) {
        const double scale = maxSpeed / std::sqrt(horizontalSq);
        result.x *= scale;
        result.y *= scale;
    }
    result.z = MathUtils::clamp(result.z, -maxSpeed, maxSpeed);
    return result;
}

} // namespace

WaypointController::WaypointController() = default;

WaypointController::WaypointController(const WaypointControllerConfig& config)
    : m_config(config) {
    m_config.attractiveGain = sanitizePositive(m_config.attractiveGain, 1.0);
    m_config.repulsiveGain = sanitizePositive(m_config.repulsiveGain, 30.0);
    m_config.influenceDistance = sanitizePositive(m_config.influenceDistance, 20.0);
    m_config.arrivalRadius = sanitizePositive(m_config.arrivalRadius, 1.0);
    m_config.safetyRadius = sanitizePositive(m_config.safetyRadius, 1.0);
    m_config.maxHorizontalSpeed = sanitizePositive(m_config.maxHorizontalSpeed, 15.0);
    m_config.maxVerticalSpeed = sanitizePositive(m_config.maxVerticalSpeed, 5.0);
    if (!std::isfinite(m_config.verticalEscapeBias)) m_config.verticalEscapeBias = 0.0;
    if (!std::isfinite(m_config.tangentialGain)) m_config.tangentialGain = 0.0;
}

WaypointController::WaypointController(const KinematicsConfig& kinematics)
    : WaypointController([&kinematics] {
          WaypointControllerConfig config;
          config.maxHorizontalSpeed = kinematics.maxHorizontalSpeed;
          config.maxVerticalSpeed = kinematics.maxVerticalSpeed;
          return config;
      }()) {}

VelocityCommand WaypointController::computeAction(
    const UAVState& uav,
    const CityMap& map,
    double dt
) const {
    VelocityCommand command;
    if (!std::isfinite(dt) || dt <= 0.0) {
        command.desiredVelocity = Vec3{};
        return command;
    }

    const Vec3 toTarget = uav.targetWaypoint - uav.position;
    const double targetDistance = toTarget.length();
    if (!std::isfinite(targetDistance) || targetDistance <= m_config.arrivalRadius) {
        command.desiredVelocity = Vec3{};
        return command;
    }

    Vec3 force = toTarget * m_config.attractiveGain;
    const Vec3 goalDirection = toTarget.normalized();
    const Vec3 lookAhead = uav.position + goalDirection * std::min(m_config.influenceDistance, targetDistance);
    const double queryRadius = m_config.influenceDistance + m_config.safetyRadius;
    const auto obstacleIndices = map.broadphase.querySphere(lookAhead, queryRadius);
    for (int32_t index : obstacleIndices) {
        if (index < 0 || static_cast<size_t>(index) >= map.buildings.size()) continue;

        const AABB& bounds = map.buildings[static_cast<size_t>(index)].bounds;
        const Vec3 closest = bounds.closestPoint(lookAhead);
        Vec3 away = lookAhead - closest;
        double distance = away.length();

        // When inside the AABB, the closest point equals the UAV position.
        // Select the nearest face normal deterministically to escape the solid.
        if (distance < 1e-6) {
            const double toMinX = std::abs(uav.position.x - bounds.minPoint.x);
            const double toMaxX = std::abs(bounds.maxPoint.x - uav.position.x);
            const double toMinY = std::abs(uav.position.y - bounds.minPoint.y);
            const double toMaxY = std::abs(bounds.maxPoint.y - uav.position.y);
            const double toMinZ = std::abs(uav.position.z - bounds.minPoint.z);
            const double toMaxZ = std::abs(bounds.maxPoint.z - uav.position.z);
            const double nearest = std::min({toMinX, toMaxX, toMinY, toMaxY, toMinZ, toMaxZ});
            if (nearest == toMinX) away = Vec3(-1.0, 0.0, 0.0);
            else if (nearest == toMaxX) away = Vec3(1.0, 0.0, 0.0);
            else if (nearest == toMinY) away = Vec3(0.0, -1.0, 0.0);
            else if (nearest == toMaxY) away = Vec3(0.0, 1.0, 0.0);
            else if (nearest == toMinZ) away = Vec3(0.0, 0.0, -1.0);
            else away = Vec3(0.0, 0.0, 1.0);
            distance = 0.0;
        }

        const double surfaceDistance = std::max(0.1, distance - m_config.safetyRadius);
        if (surfaceDistance >= m_config.influenceDistance) continue;

        const double repulsionMagnitude = m_config.repulsiveGain *
            (1.0 / surfaceDistance - 1.0 / m_config.influenceDistance) /
            (surfaceDistance * surfaceDistance);
        const Vec3 direction = away.normalized();
        force += direction * repulsionMagnitude;

        // A small deterministic vortex component reduces symmetric APF stalls.
        const Vec3 tangent = direction.cross(Vec3(0.0, 0.0, 1.0)).normalized();
        force += tangent * (m_config.tangentialGain * repulsionMagnitude);
        if (std::abs(direction.z) < 0.5) {
            force.z += m_config.verticalEscapeBias * repulsionMagnitude;
        }
    }

    // Convert force direction into speed, tapering to zero near the waypoint.
    if (force.lengthSquared() <= 1e-12) {
        force = toTarget;
    }
    Vec3 desired = force.normalized() * m_config.maxHorizontalSpeed;
    desired = clampHorizontal(desired, m_config.maxHorizontalSpeed);
    desired.z = MathUtils::clamp(desired.z, -m_config.maxVerticalSpeed, m_config.maxVerticalSpeed);

    // Approach the waypoint gently to avoid oscillating across the arrival sphere.
    const double brakingSpeed = std::sqrt(std::max(0.0, 2.0 * 3.0 * (targetDistance - m_config.arrivalRadius)));
    const double maxAllowed = std::min(m_config.maxHorizontalSpeed, brakingSpeed);
    const double horizontalSpeed = std::sqrt(desired.x * desired.x + desired.y * desired.y);
    if (horizontalSpeed > maxAllowed && horizontalSpeed > 1e-12) {
        const double scale = maxAllowed / horizontalSpeed;
        desired.x *= scale;
        desired.y *= scale;
    }

    // Align desired velocity toward the goal near arrival to avoid looping on
    // residual obstacle-field forces after the obstacle has been cleared.
    if (targetDistance < m_config.influenceDistance * 0.75) {
        const Vec3 targetDirection = toTarget.normalized();
        const double blend = 1.0 - targetDistance / (m_config.influenceDistance * 0.75);
        desired = desired * (1.0 - blend) + targetDirection * (maxAllowed * blend);
        desired = clampHorizontal(desired, m_config.maxHorizontalSpeed);
    }

    command.desiredVelocity = desired;
    return command;
}

} // namespace aerofleet
