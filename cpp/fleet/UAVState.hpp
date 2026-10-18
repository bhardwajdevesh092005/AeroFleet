#pragma once

#include <string>
#include <cstdint>
#include <cmath>
#include <sstream>
#include "core/Types.hpp"
#include "math/Vec3.hpp"
#include "math/MathUtils.hpp"

namespace aerofleet {

/**
 * @brief Authoritative kinematic and operational state for an individual UAV.
 * Coordinates are represented in East-North-Up convention:
 * X = East, Y = North, Z = Up / Altitude.
 */
struct UAVState {
    UAVId id{0};
    Vec3 position{0.0, 0.0, 0.0};
    Vec3 velocity{0.0, 0.0, 0.0};
    Vec3 acceleration{0.0, 0.0, 0.0};
    double yaw{0.0};                      ///< Heading angle in radians [-PI, PI] (0 = +X / East)
    double yawRate{0.0};                  ///< Angular turn velocity in rad/s
    double batteryPercent{100.0};         ///< State of Charge [0.0, 100.0]%
    double payloadMass{0.0};              ///< Carried cargo mass in kg
    UAVMode mode{UAVMode::IDLE};          ///< Current lifecycle operational state
    RequestId activeRequestId{-1};        ///< Assigned delivery request (-1 if none)
    Vec3 targetWaypoint{0.0, 0.0, 0.0};   ///< Current 3D waypoint target
    bool isGrounded{true};                ///< True if UAV is stationed on pad/ground

    UAVState() = default;

    explicit UAVState(UAVId uavId, const Vec3& initPos = Vec3(0.0, 0.0, 0.0), double initYaw = 0.0)
        : id(uavId), position(initPos), yaw(MathUtils::wrapAnglePi(initYaw)), isGrounded(initPos.z <= 0.01) {}

    /**
     * @brief 3D scalar speed magnitude ||v|| (m/s).
     */
    [[nodiscard]] double speed() const noexcept {
        return velocity.length();
    }

    /**
     * @brief Horizontal plane speed in XY plane (m/s).
     */
    [[nodiscard]] double horizontalSpeed() const noexcept {
        return std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y);
    }

    /**
     * @brief Vertical velocity component (m/s), positive upwards.
     */
    [[nodiscard]] double verticalSpeed() const noexcept {
        return velocity.z;
    }

    /**
     * @brief 2D unit heading vector [cos(yaw), sin(yaw), 0].
     */
    [[nodiscard]] Vec3 headingVector() const noexcept {
        return Vec3(std::cos(yaw), std::sin(yaw), 0.0);
    }

    /**
     * @brief Euclidean distance to an arbitrary 3D target point.
     */
    [[nodiscard]] double distanceTo(const Vec3& pt) const noexcept {
        return position.distance(pt);
    }

    /**
     * @brief Euclidean distance in the horizontal XY plane.
     */
    [[nodiscard]] double horizontalDistanceTo(const Vec3& pt) const noexcept {
        const double dx = position.x - pt.x;
        const double dy = position.y - pt.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    /**
     * @brief Check whether the drone is currently bearing a physical payload.
     */
    [[nodiscard]] bool hasPayload() const noexcept {
        return payloadMass > 1e-4;
    }

    /**
     * @brief Reset kinematic state at a specific spawn location.
     */
    void reset(const Vec3& spawnPos, double initYaw = 0.0) {
        position = spawnPos;
        velocity = Vec3(0.0, 0.0, 0.0);
        acceleration = Vec3(0.0, 0.0, 0.0);
        yaw = MathUtils::wrapAnglePi(initYaw);
        yawRate = 0.0;
        batteryPercent = 100.0;
        payloadMass = 0.0;
        mode = UAVMode::IDLE;
        activeRequestId = -1;
        targetWaypoint = spawnPos;
        isGrounded = (spawnPos.z <= 0.01);
    }

    /**
     * @brief String representation of state for logging and debugging.
     */
    [[nodiscard]] std::string toString() const {
        std::ostringstream oss;
        oss << "UAV #" << id << " [" << uavModeToString(mode) << "] "
            << "Pos: " << position.toString() << " "
            << "Vel: " << velocity.toString() << " (" << speed() << " m/s) "
            << "Yaw: " << MathUtils::radToDeg(yaw) << " deg "
            << "Bat: " << batteryPercent << "% "
            << "Payload: " << payloadMass << "kg";
        return oss.str();
    }
};

} // namespace aerofleet
