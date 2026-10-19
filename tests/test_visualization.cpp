#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
#include <cmath>

#include "core/WorldState.hpp"
#include "core/SimulationClock.hpp"
#include "visualization/CameraController.hpp"
#include "visualization/Renderer.hpp"
#include "world/WorldGenerator.hpp"
#include "fleet/FleetManager.hpp"
#include "control/WaypointController.hpp"
#include "coordinator/HeuristicCoordinator.hpp"

using namespace aerofleet;

namespace {
int passed = 0;
int failed = 0;

void expect(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

void expectNear(double actual, double expected, double tolerance, const std::string& message) {
    if (std::abs(actual - expected) > tolerance) {
        throw std::runtime_error(message + " (Expected " + std::to_string(expected) +
                                 ", Got " + std::to_string(actual) + ")");
    }
}

void run(const char* name, void (*test)()) {
    std::cout << "[*] Running " << name << "... ";
    try {
        test();
        ++passed;
        std::cout << "PASSED\n";
    } catch (const std::exception& ex) {
        ++failed;
        std::cout << "FAILED: " << ex.what() << "\n";
    }
}

void test_world_snapshot_purity() {
    CityMap map;
    map.chargingStations.push_back(ChargingStation{1, "Dock", Vec3(0, 0, 0), 4, 0});
    FleetSettings fs;
    fs.numUavs = 2;
    FleetManager fleet(fs, EnergySettings{}, map, 15.0);

    // Create WorldSnapshot
    WorldSnapshot snapshot;
    snapshot.simulationTime = 12.5;
    snapshot.uavs = fleet.getAllUAVs();
    snapshot.chargingStations = map.chargingStations;

    expect(snapshot.uavs.size() == 2, "snapshot must contain 2 UAVs");
    expect(snapshot.uavs[0].batteryPercent == 100.0, "initial battery should be 100%");
    const Vec3 originalPos = snapshot.uavs[0].position;

    // Mutate live simulation fleet
    fleet.getUAV(0).batteryPercent = 45.0;
    fleet.getUAV(0).position = Vec3(99.0, 99.0, 99.0);
    fleet.getUAV(0).mode = UAVMode::EMERGENCY;

    // Assert snapshot values are pure and immutable
    expect(snapshot.uavs[0].batteryPercent == 100.0, "snapshot battery was mutated by simulation changes!");
    expect(snapshot.uavs[0].position == originalPos, "snapshot position was mutated by simulation changes!");
    expect(snapshot.uavs[0].mode == UAVMode::IDLE, "snapshot mode was mutated by simulation changes!");
}

void test_world_snapshot_interpolation() {
    WorldSnapshot prev;
    prev.simulationTime = 10.0;
    UAVState uPrev(0, Vec3(0.0, 0.0, 10.0), 0.0);
    prev.uavs.push_back(uPrev);

    WorldSnapshot curr;
    curr.simulationTime = 10.02;
    UAVState uCurr(0, Vec3(10.0, 20.0, 30.0), MathUtils::PI * 0.5);
    curr.uavs.push_back(uCurr);

    // Halfway sub-step interpolation (alpha = 0.5)
    WorldSnapshot interp = WorldSnapshot::interpolate(prev, curr, 0.5);

    expectNear(interp.simulationTime, 10.01, 1e-6, "interpolated simulation time mismatch");
    const auto* uInterp = interp.findUAV(0);
    expect(uInterp != nullptr, "interpolated UAV not found");
    expectNear(uInterp->position.x, 5.0, 1e-6, "interpolated position X mismatch");
    expectNear(uInterp->position.y, 10.0, 1e-6, "interpolated position Y mismatch");
    expectNear(uInterp->position.z, 20.0, 1e-6, "interpolated position Z mismatch");
    expectNear(uInterp->yaw, MathUtils::PI * 0.25, 1e-6, "interpolated yaw mismatch");
}

void test_simulation_clock_fixed_steps() {
    SimulationClock clock(0.02, 1.0); // 50 Hz, dt = 0.02s
    expect(clock.now() == 0.0, "clock initial time should be 0");
    expect(!clock.hasStep(), "empty accumulator should not have step");

    // Feed 0.05 seconds of wall time -> accumulator = 0.05s
    clock.update(0.05);
    expect(clock.hasStep(), "accumulator >= 0.02s should have step");

    // Advance first step (0.00 -> 0.02s)
    clock.advance();
    expectNear(clock.now(), 0.02, 1e-6, "clock now after 1 step mismatch");
    expect(clock.hasStep(), "accumulator (0.03s) should still have step");

    // Advance second step (0.02 -> 0.04s)
    clock.advance();
    expectNear(clock.now(), 0.04, 1e-6, "clock now after 2 steps mismatch");

    // Accumulator has 0.01s left (< 0.02s)
    expect(!clock.hasStep(), "accumulator (0.01s) should not have step");
    expectNear(clock.getAlpha(), 0.5, 1e-4, "alpha interpolation fraction should be 0.5");
    expect(clock.getStepCount() == 2, "total steps should be 2");
}

void test_simulation_clock_spiral_of_death_clamp() {
    SimulationClock clock(0.02, 1.0);

    // Simulate huge frame drop (e.g. 5.0 seconds of freeze)
    clock.update(5.0);

    // Accumulator must be clamped to 0.20s (max 10 physics steps) to avoid freeze
    int stepsTaken = 0;
    while (clock.hasStep()) {
        clock.advance();
        stepsTaken++;
    }

    expect(stepsTaken == 10, "spiral-of-death clamp failed: took " + std::to_string(stepsTaken) + " steps, expected 10");
    expect(!clock.hasStep(), "accumulator should be fully drained");
}

void test_camera_controller_orbit_math() {
    const Vec3 target(100.0, 50.0, 0.0);
    const double distance = 200.0;

    // 1. Azimuth = 0, Elevation = 0 -> purely along +X
    Vec3 pos0 = CameraController::computeOrbitPosition(target, distance, 0.0, 0.0);
    expectNear(pos0.x, 300.0, 1e-4, "orbit pos0 X mismatch");
    expectNear(pos0.y, 50.0, 1e-4, "orbit pos0 Y mismatch");
    expectNear(pos0.z, 0.0, 1e-4, "orbit pos0 Z mismatch");

    // 2. Azimuth = pi/2, Elevation = 0 -> purely along +Y
    Vec3 pos90 = CameraController::computeOrbitPosition(target, distance, MathUtils::PI * 0.5, 0.0);
    expectNear(pos90.x, 100.0, 1e-4, "orbit pos90 X mismatch");
    expectNear(pos90.y, 250.0, 1e-4, "orbit pos90 Y mismatch");
    expectNear(pos90.z, 0.0, 1e-4, "orbit pos90 Z mismatch");

    // 3. Elevation = pi/2 -> zenith above target
    Vec3 posZenith = CameraController::computeOrbitPosition(target, distance, 0.0, MathUtils::PI * 0.5);
    expectNear(posZenith.x, 100.0, 1e-4, "orbit posZenith X mismatch");
    expectNear(posZenith.y, 50.0, 1e-4, "orbit posZenith Y mismatch");
    expectNear(posZenith.z, 200.0, 1e-4, "orbit posZenith Z mismatch");
}

void test_camera_controller_chase_math() {
    const Vec3 uavPos(50.0, 60.0, 25.0);
    const double yawEast = 0.0; // Facing +X
    const double distance = 15.0;
    const double height = 5.0;

    // Chase camera should be 15m behind along -X and 5m above along +Z
    Vec3 chasePos = CameraController::computeChasePosition(uavPos, yawEast, distance, height);
    expectNear(chasePos.x, 35.0, 1e-4, "chasePos X mismatch");
    expectNear(chasePos.y, 60.0, 1e-4, "chasePos Y mismatch");
    expectNear(chasePos.z, 30.0, 1e-4, "chasePos Z mismatch");

    // Facing North (+Y, yaw = pi/2)
    const double yawNorth = MathUtils::PI * 0.5;
    Vec3 chaseNorth = CameraController::computeChasePosition(uavPos, yawNorth, distance, height);
    expectNear(chaseNorth.x, 50.0, 1e-4, "chaseNorth X mismatch");
    expectNear(chaseNorth.y, 45.0, 1e-4, "chaseNorth Y mismatch");
    expectNear(chaseNorth.z, 30.0, 1e-4, "chaseNorth Z mismatch");
}

void test_frame_rate_decoupling() {
    // Decoupling verification:
    // Compare 15 FPS rendering throttle vs 120 FPS high-refresh rendering
    // Both simulate 2.0 seconds of real wall time at 1x speed.
    // Fixed dtPhysics = 0.02s (50 Hz).
    // Both MUST execute exactly 100 physics steps!

    SimulationClock clock15(0.02, 1.0);
    const double dt15 = 1.0 / 15.0; // ~0.0667s per frame
    const int frames15 = 30; // 30 frames @ 15 FPS = 2.0 seconds
    for (int f = 0; f < frames15; ++f) {
        clock15.update(dt15);
        while (clock15.hasStep()) {
            clock15.advance();
        }
    }

    SimulationClock clock120(0.02, 1.0);
    const double dt120 = 1.0 / 120.0; // ~0.00833s per frame
    const int frames120 = 240; // 240 frames @ 120 FPS = 2.0 seconds
    for (int f = 0; f < frames120; ++f) {
        clock120.update(dt120);
        while (clock120.hasStep()) {
            clock120.advance();
        }
    }

    expect(clock15.getStepCount() == 100, "15 FPS simulation should execute exactly 100 physics steps (got " +
           std::to_string(clock15.getStepCount()) + ")");
    expect(clock120.getStepCount() == 100, "120 FPS simulation should execute exactly 100 physics steps (got " +
           std::to_string(clock120.getStepCount()) + ")");
    expectNear(clock15.now(), 2.0, 1e-5, "15 FPS clock time should equal 2.0s");
    expectNear(clock120.now(), 2.0, 1e-5, "120 FPS clock time should equal 2.0s");
}

void test_renderer_color_mappings() {
    // Battery colors
    Color c100 = Renderer::getBatteryColor(100.0);
    Color c50 = Renderer::getBatteryColor(50.0);
    Color c10 = Renderer::getBatteryColor(10.0);

    expect(c100.g > c100.r, "100% battery must have green dominant");
    expect(c50.r > 200 && c50.g > 150, "50% battery must be yellow/amber");
    expect(c10.r > c10.g, "10% battery must be red dominant");

    // Mode colors must have full opacity
    Color mIdle = Renderer::getModeColor(UAVMode::IDLE);
    Color mEmerg = Renderer::getModeColor(UAVMode::EMERGENCY);
    Color mPick = Renderer::getModeColor(UAVMode::FLYING_TO_PICKUP);
    expect(mIdle.a == 255, "mode color alpha must be 255");
    expect(mEmerg.r > mEmerg.g, "emergency mode must be red dominant");
    expect(mPick.g > mPick.r, "pickup mode must be green dominant");
}

void test_renderer_headless_purity_safety() {
    RendererConfig config;
    Renderer renderer(config);
    // In headless test execution, window is not open
    expect(!renderer.isReady(), "renderer should report not ready without active window");

    WorldSnapshot snapshot;
    snapshot.simulationTime = 5.0;
    snapshot.uavs.push_back(UAVState(0, Vec3(1, 2, 3), 0.0));
    CityMap map;
    CameraController camera;

    // Calling draw() on uninitialized / headless renderer must be a safe no-op
    renderer.draw(snapshot, map, camera);

    // Assert snapshot remains untouched
    expect(snapshot.simulationTime == 5.0, "snapshot simulationTime modified");
    expect(snapshot.uavs[0].position == Vec3(1, 2, 3), "snapshot UAV position modified");
}

} // namespace

int main() {
    std::cout << "========================================\n"
              << "   AeroFleet Visualization Tests        \n"
              << "========================================\n";

    run("test_world_snapshot_purity", test_world_snapshot_purity);
    run("test_world_snapshot_interpolation", test_world_snapshot_interpolation);
    run("test_simulation_clock_fixed_steps", test_simulation_clock_fixed_steps);
    run("test_simulation_clock_spiral_of_death_clamp", test_simulation_clock_spiral_of_death_clamp);
    run("test_camera_controller_orbit_math", test_camera_controller_orbit_math);
    run("test_camera_controller_chase_math", test_camera_controller_chase_math);
    run("test_frame_rate_decoupling", test_frame_rate_decoupling);
    run("test_renderer_color_mappings", test_renderer_color_mappings);
    run("test_renderer_headless_purity_safety", test_renderer_headless_purity_safety);

    std::cout << "========================================\nSummary: " << passed
              << " Passed, " << failed << " Failed\n========================================\n";
    return failed == 0 ? 0 : 1;
}

