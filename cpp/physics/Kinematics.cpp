#include "physics/Kinematics.hpp"
#include <algorithm>
#include <cmath>

namespace aerofleet {

Vec3 KinematicsIntegrator::clampAcceleration(const Vec3& rawAcc) const noexcept {
    const double magSq = rawAcc.lengthSquared();
    const double maxAcc = m_config.maxAcceleration;
    if (magSq > (maxAcc * maxAcc) && magSq > 1e-12) {
        const double scale = maxAcc / std::sqrt(magSq);
        return rawAcc * scale;
    }
    return rawAcc;
}

Vec3 KinematicsIntegrator::clampVelocity(const Vec3& rawVel) const noexcept {
    // Horizontal XY saturation
    const double horizMagSq = rawVel.x * rawVel.x + rawVel.y * rawVel.y;
    const double maxHoriz = m_config.maxHorizontalSpeed;
    double vx = rawVel.x;
    double vy = rawVel.y;

    if (horizMagSq > (maxHoriz * maxHoriz) && horizMagSq > 1e-12) {
        const double scale = maxHoriz / std::sqrt(horizMagSq);
        vx *= scale;
        vy *= scale;
    }

    // Vertical Z saturation
    const double vz = MathUtils::clamp(rawVel.z, -m_config.maxVerticalSpeed, m_config.maxVerticalSpeed);

    return Vec3(vx, vy, vz);
}

double KinematicsIntegrator::stepYaw(double currentYaw, double targetYaw, double dt) const noexcept {
    if (dt <= 0.0) return currentYaw;

    const double deltaYaw = MathUtils::wrapAnglePi(targetYaw - currentYaw);
    const double maxStep = m_config.maxYawRate * dt;
    const double actualStep = MathUtils::clamp(deltaYaw, -maxStep, maxStep);

    return MathUtils::wrapAnglePi(currentYaw + actualStep);
}

void KinematicsIntegrator::enforceBoundaries(UAVState& state) const noexcept {
    // Ground floor collision and contact resting state
    if (state.position.z <= m_config.groundAltitude) {
        state.position.z = m_config.groundAltitude;
        if (state.velocity.z < 0.0) {
            state.velocity.z = 0.0;
        }
        if (state.acceleration.z < 0.0) {
            state.acceleration.z = 0.0;
        }
        state.isGrounded = true;
    } else {
        state.isGrounded = false;
    }

    // Flight ceiling clamp
    if (state.position.z >= m_config.ceilingAltitude) {
        state.position.z = m_config.ceilingAltitude;
        if (state.velocity.z > 0.0) {
            state.velocity.z = 0.0;
        }
        if (state.acceleration.z > 0.0) {
            state.acceleration.z = 0.0;
        }
    }
}

void KinematicsIntegrator::stepVelocityCommand(UAVState& state, const VelocityCommand& cmd, double dt) const {
    if (dt <= 0.0) return;

    // 1. Clamp desired velocity command to physical envelope
    const Vec3 targetVel = clampVelocity(cmd.desiredVelocity);

    // 2. Compute required acceleration to reach target velocity
    const Vec3 neededAcc = (targetVel - state.velocity) / dt;
    Vec3 commandedAcc = clampAcceleration(neededAcc);

    // 3. Apply jerk limiting if configured
    if (m_config.maxJerk > 0.0) {
        const Vec3 deltaAcc = commandedAcc - state.acceleration;
        const double maxDeltaMag = m_config.maxJerk * dt;
        if (deltaAcc.lengthSquared() > (maxDeltaMag * maxDeltaMag) && deltaAcc.lengthSquared() > 1e-12) {
            commandedAcc = state.acceleration + deltaAcc.normalized() * maxDeltaMag;
            commandedAcc = clampAcceleration(commandedAcc);
        }
    }

    state.acceleration = commandedAcc;

    // 4. Semi-implicit Euler integration: update velocity first, then position
    const Vec3 candidateVel = state.velocity + state.acceleration * dt;
    state.velocity = clampVelocity(candidateVel);
    state.position += state.velocity * dt;

    // 5. Enforce ground floor and altitude boundaries
    enforceBoundaries(state);

    // 6. Update attitude / yaw heading
    double targetYaw = state.yaw;
    if (cmd.useExplicitYaw) {
        targetYaw = cmd.desiredYaw;
    } else if (state.horizontalSpeed() > 0.1) {
        targetYaw = std::atan2(state.velocity.y, state.velocity.x);
    }

    const double prevYaw = state.yaw;
    state.yaw = stepYaw(state.yaw, targetYaw, dt);
    state.yawRate = MathUtils::wrapAnglePi(state.yaw - prevYaw) / dt;
}

void KinematicsIntegrator::stepAccelerationCommand(UAVState& state, const AccelerationCommand& cmd, double dt) const {
    if (dt <= 0.0) return;

    Vec3 commandedAcc = clampAcceleration(cmd.desiredAcceleration);

    // Jerk limiting
    if (m_config.maxJerk > 0.0) {
        const Vec3 deltaAcc = commandedAcc - state.acceleration;
        const double maxDeltaMag = m_config.maxJerk * dt;
        if (deltaAcc.lengthSquared() > (maxDeltaMag * maxDeltaMag) && deltaAcc.lengthSquared() > 1e-12) {
            commandedAcc = state.acceleration + deltaAcc.normalized() * maxDeltaMag;
            commandedAcc = clampAcceleration(commandedAcc);
        }
    }

    state.acceleration = commandedAcc;

    // Semi-implicit Euler integration
    const Vec3 candidateVel = state.velocity + state.acceleration * dt;
    state.velocity = clampVelocity(candidateVel);
    state.position += state.velocity * dt;

    enforceBoundaries(state);

    // Commanded turning rate integration
    const double clampedYawRate = MathUtils::clamp(cmd.desiredYawRate, -m_config.maxYawRate, m_config.maxYawRate);
    state.yawRate = clampedYawRate;
    state.yaw = MathUtils::wrapAnglePi(state.yaw + clampedYawRate * dt);
}

} // namespace aerofleet
