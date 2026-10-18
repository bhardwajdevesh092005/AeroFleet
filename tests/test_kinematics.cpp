#include <iostream>
#include <cassert>
#include <cmath>
#include "physics/Kinematics.hpp"
#include "core/Config.hpp"

using namespace aerofleet;

static int g_testsPassed = 0;
static int g_testsFailed = 0;

#define TEST_ASSERT(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "[-] FAILED: " << msg << " (" << __FILE__ << ":" << __LINE__ << ")\n"; \
            g_testsFailed++; \
            return; \
        } \
    } while(0)

#define RUN_TEST(fn) \
    do { \
        std::cout << "[*] Running " << #fn << "... "; \
        int initialFailed = g_testsFailed; \
        fn(); \
        if (g_testsFailed == initialFailed) { \
            std::cout << "PASSED\n"; \
            g_testsPassed++; \
        } \
    } while(0)

void test_velocity_saturation() {
    KinematicsConfig cfg;
    cfg.maxHorizontalSpeed = 15.0;
    cfg.maxVerticalSpeed = 5.0;
    cfg.maxAcceleration = 6.0;
    KinematicsIntegrator integrator(cfg);

    UAVState uav(1, Vec3(0.0, 0.0, 50.0));

    // Excessive horizontal command: 100 m/s East
    for (int i = 0; i < 200; ++i) {
        integrator.stepVelocity(uav, Vec3(100.0, 0.0, 0.0), 0.02);
    }

    TEST_ASSERT(uav.horizontalSpeed() <= cfg.maxHorizontalSpeed + 1e-6, "Horizontal speed must clamp to maxHorizontalSpeed");
    TEST_ASSERT(std::abs(uav.velocity.x - 15.0) < 1e-4, "Velocity X must saturate at 15.0 m/s");
    TEST_ASSERT(std::abs(uav.velocity.y) < 1e-6, "Velocity Y must remain zero");

    // Excessive vertical ascent: 100 m/s Up
    for (int i = 0; i < 200; ++i) {
        integrator.stepVelocity(uav, Vec3(0.0, 0.0, 100.0), 0.02);
    }
    TEST_ASSERT(std::abs(uav.velocity.z - 5.0) < 1e-4, "Velocity Z must saturate at maxVerticalSpeed (5.0 m/s)");
}

void test_acceleration_limits() {
    KinematicsConfig cfg;
    cfg.maxAcceleration = 6.0;
    cfg.maxHorizontalSpeed = 15.0;
    cfg.maxJerk = 0.0; // test pure acceleration clamping
    KinematicsIntegrator integrator(cfg);

    UAVState uav(2, Vec3(0.0, 0.0, 20.0));

    // From rest, request full speed (15.0 m/s) in a single dt = 0.02 step
    integrator.stepVelocity(uav, Vec3(15.0, 0.0, 0.0), 0.02);

    // Delta velocity in 1 step must not exceed a_max * dt = 6.0 * 0.02 = 0.12 m/s
    const double maxDeltaV = cfg.maxAcceleration * 0.02;
    TEST_ASSERT(uav.speed() <= maxDeltaV + 1e-6, "Speed delta in one step must respect maxAcceleration");
    TEST_ASSERT(uav.acceleration.length() <= cfg.maxAcceleration + 1e-6, "Acceleration magnitude must be bounded by maxAcceleration");
}

void test_asymmetric_velocity_limits() {
    KinematicsConfig cfg;
    cfg.maxHorizontalSpeed = 15.0;
    cfg.maxVerticalSpeed = 5.0;
    cfg.maxAcceleration = 10.0;
    cfg.maxJerk = 0.0;
    KinematicsIntegrator integrator(cfg);

    UAVState uav(3, Vec3(0.0, 0.0, 50.0));

    // Request large 3D diagonal velocity: (100, 100, 100)
    for (int i = 0; i < 300; ++i) {
        integrator.stepVelocity(uav, Vec3(100.0, 100.0, 100.0), 0.02);
    }

    TEST_ASSERT(uav.horizontalSpeed() <= cfg.maxHorizontalSpeed + 1e-6, "Horizontal XY speed must strictly obey horizontal limit");
    TEST_ASSERT(std::abs(uav.verticalSpeed()) <= cfg.maxVerticalSpeed + 1e-6, "Vertical Z speed must strictly obey vertical limit");
}

void test_yaw_integration_and_wrapping() {
    KinematicsConfig cfg;
    cfg.maxYawRate = MathUtils::PI; // 180 deg/s
    KinematicsIntegrator integrator(cfg);

    // Start yaw near +PI (e.g. 3.0 rad = ~171.9 deg)
    UAVState uav(4, Vec3(0.0, 0.0, 30.0), 3.0);

    // Target yaw near -PI (e.g. -3.0 rad = -171.9 deg)
    // Shortest angular path wraps across +PI / -PI (delta is +0.283 rad, not -6.0 rad)
    VelocityCommand cmd;
    cmd.desiredVelocity = Vec3(0.0, 0.0, 0.0);
    cmd.useExplicitYaw = true;
    cmd.desiredYaw = -3.0;

    integrator.stepVelocityCommand(uav, cmd, 0.05);

    // Drone should have turned towards +PI (increasing yaw across wrap boundary)
    TEST_ASSERT(uav.yaw > 3.0 || uav.yaw < -3.0, "Shortest path turn across +/- PI boundary must be chosen");
    TEST_ASSERT(uav.yaw >= -MathUtils::PI && uav.yaw <= MathUtils::PI, "Yaw must remain bounded in [-PI, PI]");

    // Integrate to completion
    for (int i = 0; i < 50; ++i) {
        integrator.stepVelocityCommand(uav, cmd, 0.02);
    }
    TEST_ASSERT(std::abs(uav.yaw - (-3.0)) < 1e-4, "Yaw must smoothly converge to target angle");
}

void test_velocity_command_tracking() {
    KinematicsConfig cfg;
    cfg.maxHorizontalSpeed = 15.0;
    cfg.maxVerticalSpeed = 5.0;
    cfg.maxAcceleration = 6.0;
    KinematicsIntegrator integrator(cfg);

    UAVState uav(5, Vec3(0.0, 0.0, 20.0));

    const Vec3 targetVel(8.0, 6.0, 2.5); // horizontal mag = 10 m/s <= 15, vert = 2.5 <= 5
    for (int i = 0; i < 200; ++i) {
        integrator.stepVelocity(uav, targetVel, 0.02);
    }

    TEST_ASSERT(uav.velocity.isApprox(targetVel, 1e-3), "UAV velocity must converge accurately to feasible target velocity");
    TEST_ASSERT(uav.acceleration.length() < 1e-3, "Acceleration must reach zero when cruising at steady state");
}

void test_integrator_numerical_stability() {
    KinematicsConfig cfg;
    KinematicsIntegrator integrator(cfg);

    UAVState uav(6, Vec3(0.0, 0.0, 30.0));

    // Run 10,000 steps (200 simulated seconds) with sinusoidal varying velocity
    const double dt = 0.02;
    for (int step = 0; step < 10000; ++step) {
        const double t = step * dt;
        Vec3 cmdVel(
            10.0 * std::sin(t * 0.5),
            8.0 * std::cos(t * 0.3),
            2.0 * std::sin(t * 0.2)
        );
        integrator.stepVelocity(uav, cmdVel, dt);

        TEST_ASSERT(std::isfinite(uav.position.x), "Position X must be finite");
        TEST_ASSERT(std::isfinite(uav.position.y), "Position Y must be finite");
        TEST_ASSERT(std::isfinite(uav.position.z), "Position Z must be finite");
        TEST_ASSERT(std::isfinite(uav.velocity.x), "Velocity X must be finite");
        TEST_ASSERT(std::isfinite(uav.velocity.y), "Velocity Y must be finite");
        TEST_ASSERT(std::isfinite(uav.velocity.z), "Velocity Z must be finite");
        TEST_ASSERT(std::isfinite(uav.yaw), "Yaw must be finite");
    }

    TEST_ASSERT(true, "Integrator demonstrated numerical stability over 10,000 steps");
}

void test_ground_collision_floor() {
    KinematicsConfig cfg;
    cfg.groundAltitude = 0.0;
    KinematicsIntegrator integrator(cfg);

    // Drone 1 meter above ground, moving down fast
    UAVState uav(7, Vec3(0.0, 0.0, 1.0));

    // Steer hard downwards: 100 steps (2.0 seconds) ensures drone reaches floor under bounded acceleration
    for (int i = 0; i < 100; ++i) {
        integrator.stepVelocity(uav, Vec3(0.0, 0.0, -10.0), 0.02);
    }

    TEST_ASSERT(uav.position.z >= cfg.groundAltitude, "Position Z must never penetrate below groundAltitude");
    TEST_ASSERT(uav.velocity.z >= 0.0, "Vertical velocity must be arrested upon ground contact");
    TEST_ASSERT(uav.isGrounded, "UAV must be marked as grounded when resting on floor");
}

int main() {
    std::cout << "========================================\n";
    std::cout << "      AeroFleet Kinematics Tests       \n";
    std::cout << "========================================\n";

    RUN_TEST(test_velocity_saturation);
    RUN_TEST(test_acceleration_limits);
    RUN_TEST(test_asymmetric_velocity_limits);
    RUN_TEST(test_yaw_integration_and_wrapping);
    RUN_TEST(test_velocity_command_tracking);
    RUN_TEST(test_integrator_numerical_stability);
    RUN_TEST(test_ground_collision_floor);

    std::cout << "========================================\n";
    std::cout << "Summary: " << g_testsPassed << " Passed, " << g_testsFailed << " Failed\n";
    std::cout << "========================================\n";

    return (g_testsFailed == 0) ? 0 : 1;
}
