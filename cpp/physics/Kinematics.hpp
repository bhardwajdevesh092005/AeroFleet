#pragma once

#include "fleet/UAVState.hpp"
#include "core/Config.hpp"
#include "math/Vec3.hpp"
#include "math/MathUtils.hpp"

namespace aerofleet {

/**
 * @brief Kinematic boundaries and rate-of-change limits governing UAV motion.
 */
struct KinematicsConfig {
    double maxHorizontalSpeed{15.0};      ///< Maximum XY linear speed in m/s
    double maxVerticalSpeed{5.0};         ///< Maximum |Z| vertical speed in m/s
    double maxAcceleration{6.0};          ///< Maximum linear acceleration magnitude in m/s^2
    double maxYawRate{MathUtils::PI};     ///< Maximum turning rate in rad/s (default 180 deg/s)
    double maxJerk{30.0};                 ///< Maximum rate of change of acceleration in m/s^3
    double groundAltitude{0.0};           ///< Minimum physical flight altitude floor in meters
    double ceilingAltitude{150.0};        ///< Maximum physical flight altitude ceiling in meters

    /**
     * @brief Construct kinematic limits directly from validated simulation configuration.
     */
    static KinematicsConfig fromConfig(const FleetSettings& fleet, const WorldSettings& world) {
        KinematicsConfig kc;
        kc.maxHorizontalSpeed = fleet.maxHorizontalSpeed;
        kc.maxVerticalSpeed = fleet.maxVerticalSpeed;
        kc.maxAcceleration = fleet.maxAcceleration;
        kc.groundAltitude = world.zMin;
        kc.ceilingAltitude = world.zMax;
        return kc;
    }
};

/**
 * @brief High-level velocity control command supplied by navigation policy or waypoint planner.
 */
struct VelocityCommand {
    Vec3 desiredVelocity{0.0, 0.0, 0.0};  ///< Desired 3D velocity vector (m/s)
    double desiredYaw{0.0};               ///< Target heading in radians (if useExplicitYaw is true)
    bool useExplicitYaw{false};           ///< If false, yaw automatically aligns with velocity
};

/**
 * @brief Low-level direct acceleration control command.
 */
struct AccelerationCommand {
    Vec3 desiredAcceleration{0.0, 0.0, 0.0};  ///< Commanded acceleration vector (m/s^2)
    double desiredYawRate{0.0};                ///< Commanded angular yaw velocity (rad/s)
};

/**
 * @brief Symplectic (Semi-Implicit) Euler motion integrator for UAV rigid dynamics.
 * Enforces acceleration limits, asymmetric velocity limits (horizontal vs vertical),
 * angular turning rate constraints, and ground floor boundaries.
 */
class KinematicsIntegrator {
public:
    KinematicsIntegrator() = default;
    explicit KinematicsIntegrator(const KinematicsConfig& config) : m_config(config) {}

    /**
     * @brief Integrates UAV motion using a target velocity command.
     * Computes required linear acceleration to reach desired velocity within maxAcceleration limits.
     */
    void stepVelocityCommand(UAVState& state, const VelocityCommand& cmd, double dt) const;

    /**
     * @brief Integrates UAV motion using direct 3D velocity input (auto-aligning yaw with heading).
     */
    void stepVelocity(UAVState& state, const Vec3& targetVelocity, double dt) const {
        VelocityCommand cmd;
        cmd.desiredVelocity = targetVelocity;
        cmd.useExplicitYaw = false;
        stepVelocityCommand(state, cmd, dt);
    }

    /**
     * @brief Integrates UAV motion using an explicit acceleration command.
     */
    void stepAccelerationCommand(UAVState& state, const AccelerationCommand& cmd, double dt) const;

    /**
     * @brief Integrates UAV motion using direct acceleration vector input.
     */
    void stepAcceleration(UAVState& state, const Vec3& targetAcceleration, double dt) const {
        AccelerationCommand cmd;
        cmd.desiredAcceleration = targetAcceleration;
        cmd.desiredYawRate = 0.0;
        stepAccelerationCommand(state, cmd, dt);
    }

    /**
     * @brief Saturate acceleration vector to maximum linear acceleration magnitude.
     */
    [[nodiscard]] Vec3 clampAcceleration(const Vec3& rawAcc) const noexcept;

    /**
     * @brief Saturate velocity vector adhering to asymmetric horizontal and vertical maximums.
     */
    [[nodiscard]] Vec3 clampVelocity(const Vec3& rawVel) const noexcept;

    /**
     * @brief Step yaw angle towards target yaw bounded by maxYawRate using shortest angular path.
     */
    [[nodiscard]] double stepYaw(double currentYaw, double targetYaw, double dt) const noexcept;

    /**
     * @brief Clamp altitude to [groundAltitude, ceilingAltitude] and update grounded flag.
     */
    void enforceBoundaries(UAVState& state) const noexcept;

    [[nodiscard]] const KinematicsConfig& getConfig() const noexcept { return m_config; }
    void setConfig(const KinematicsConfig& config) noexcept { m_config = config; }

private:
    KinematicsConfig m_config;
};

} // namespace aerofleet
