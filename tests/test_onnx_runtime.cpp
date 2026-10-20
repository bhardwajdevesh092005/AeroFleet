#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>
#include <vector>

#include "control/ONNXNavigationPolicy.hpp"
#include "control/SafetyLayer.hpp"
#include "core/Config.hpp"
#include "core/Simulator.hpp"
#include "inference/ONNXModel.hpp"
#include "world/WorldGenerator.hpp"

using namespace aerofleet;

void testONNXModelLoadingAndMetadata() {
    std::cout << "[Test 1] Testing ONNXModel loading and metadata..." << std::endl;
    ONNXModel model("models/navigation/nav_policy.onnx");

    assert(model.getInputDim() == 87);
    assert(model.getOutputDim() == 3);
    assert(model.getInputName() == "observation");
    assert(model.getOutputName() == "action");

    // Single-vector inference
    std::vector<float> input(87, 0.0f);
    auto output = model.run(input);
    assert(output.size() == 3);
    for (float val : output) {
        (void)val;
        assert(std::isfinite(val));
        assert(val >= -1.0f && val <= 1.0f);
    }

    // Batched inference (batch size 4)
    std::vector<float> batchInputs(87 * 4, 0.5f);
    auto batchOutputs = model.runBatch(batchInputs, 4);
    assert(batchOutputs.size() == 12);
    for (float val : batchOutputs) {
        (void)val;
        assert(std::isfinite(val));
        assert(val >= -1.0f && val <= 1.0f);
    }

    std::cout << "  -> PASSED: Model loaded, metadata verified, single and batched inference OK." << std::endl;
}

void testSafetyLayerInterventions() {
    std::cout << "[Test 2] Testing SafetyLayer collision lookahead and emergency overrides..." << std::endl;
    SafetyLayerConfig cfg;
    cfg.lookaheadTime = 1.0;
    cfg.uavRadius = 0.5;
    cfg.safetyMargin = 0.3;
    cfg.emergencyClimbSpeed = 3.0;
    SafetyLayer safety(cfg);

    WorldSettings world;
    world.xMin = -100.0;
    world.xMax = 100.0;
    world.yMin = -100.0;
    world.yMax = 100.0;
    world.zMin = 0.0;
    world.zMax = 100.0;
    CityMap map = WorldGenerator::generate(world, 42);

    UAVState uav;
    uav.id = 0;
    uav.position = Vec3(0.0, 0.0, 30.0);
    uav.velocity = Vec3(0.0, 0.0, 0.0);
    uav.yaw = 0.0;

    // 1. Safe trajectory in open space
    VelocityCommand safeCmd{Vec3(2.0, 0.0, 0.0)};
    assert(safety.isTrajectorySafe(uav, map, safeCmd, 1.0));
    auto filteredSafe = safety.filterAction(uav, map, safeCmd, 0.1);
    (void)filteredSafe;
    assert(filteredSafe.desiredVelocity.x == safeCmd.desiredVelocity.x);
    assert(filteredSafe.desiredVelocity.y == safeCmd.desiredVelocity.y);
    assert(filteredSafe.desiredVelocity.z == safeCmd.desiredVelocity.z);
    assert(safety.getInterventionCount() == 0);

    // 2. Unsafe trajectory directly into world boundary
    uav.position = Vec3(99.0, 0.0, 30.0); // 1 meter from boundary at x=100
    VelocityCommand crashBoundaryCmd{Vec3(15.0, 0.0, 0.0)}; // Flying toward x=100 at 15 m/s
    assert(!safety.isTrajectorySafe(uav, map, crashBoundaryCmd, 1.0));
    auto overrideBoundary = safety.filterAction(uav, map, crashBoundaryCmd, 0.1);
    (void)overrideBoundary;
    assert(safety.getInterventionCount() == 1);
    // Must NOT fly into boundary
    assert(overrideBoundary.desiredVelocity.x <= 0.0);

    // 3. Unsafe trajectory directly into building AABB
    if (!map.buildings.empty()) {
        const auto& b = map.buildings[0];
        // Position drone 1m in front of building minPoint.x, facing +X
        uav.position = Vec3(b.bounds.minPoint.x - 1.0, (b.bounds.minPoint.y + b.bounds.maxPoint.y) * 0.5, (b.bounds.minPoint.z + b.bounds.maxPoint.z) * 0.5);
        VelocityCommand crashBuildingCmd{Vec3(15.0, 0.0, 0.0)};
        assert(!safety.isTrajectorySafe(uav, map, crashBuildingCmd, 1.0));
        auto overrideBuilding = safety.filterAction(uav, map, crashBuildingCmd, 0.1);
        (void)overrideBuilding;
        assert(safety.getInterventionCount() == 2);
        // Overridden action climbs or stops, does not penetrate building
        assert(overrideBuilding.desiredVelocity.x <= 0.0);
    }

    std::cout << "  -> PASSED: SafetyLayer accurately detects hazards and overrides commands ("
              << safety.getInterventionCount() << " interventions)." << std::endl;
}

void testONNXNavigationPolicyObservationAndAction() {
    std::cout << "[Test 3] Testing ONNXNavigationPolicy observation pipeline and action computation..." << std::endl;
    ONNXNavigationPolicy policy(
        "models/navigation/nav_policy.onnx",
        "models/navigation/normalization.json"
    );

    assert(policy.getRunningMean().size() == 87);
    assert(policy.getRunningVar().size() == 87);

    WorldSettings world;
    world.xMin = -500.0;
    world.xMax = 500.0;
    world.yMin = -500.0;
    world.yMax = 500.0;
    world.zMin = 0.0;
    world.zMax = 150.0;
    CityMap map = WorldGenerator::generate(world, 100);

    UAVState uav;
    uav.id = 0;
    uav.position = Vec3(0.0, 0.0, 25.0);
    uav.velocity = Vec3(0.0, 0.0, 0.0);
    uav.yaw = 0.5; // Radians
    uav.targetWaypoint = Vec3(100.0, 50.0, 25.0);
    uav.batteryPercent = 95.0;
    uav.payloadMass = 0.0;
    uav.mode = UAVMode::ASSIGNED;

    auto rawObs = policy.buildRawObservation(uav, map);
    assert(rawObs.size() == 87);
    for (float f : rawObs) {
        (void)f;
        assert(std::isfinite(f));
    }

    auto normObs = policy.buildNormalizedObservation(uav, map);
    assert(normObs.size() == 87);
    for (float f : normObs) {
        (void)f;
        assert(std::isfinite(f));
        assert(f >= -10.0f && f <= 10.0f);
    }

    // Compute policy action
    auto action = policy.computeAction(uav, map, 0.1);
    assert(std::isfinite(action.desiredVelocity.x));
    assert(std::isfinite(action.desiredVelocity.y));
    assert(std::isfinite(action.desiredVelocity.z));

    double horizontalSpeed = std::hypot(action.desiredVelocity.x, action.desiredVelocity.y);
    (void)horizontalSpeed;
    assert(horizontalSpeed <= 15.01); // Within horizontal limits
    assert(std::abs(action.desiredVelocity.z) <= 5.01); // Within vertical limits

    std::cout << "  -> PASSED: Observations bounded, computed action: vx="
              << action.desiredVelocity.x << ", vy=" << action.desiredVelocity.y << ", vz=" << action.desiredVelocity.z << std::endl;
}

void testInferenceLatencyBenchmark() {
    std::cout << "[Test 4] Benchmarking C++ ONNX Runtime inference latency (1,000 iterations)..." << std::endl;
    ONNXModel model("models/navigation/nav_policy.onnx");
    std::vector<float> input(87, 0.1f);

    // Warmup
    for (int i = 0; i < 50; ++i) {
        auto dummy = model.run(input);
        (void)dummy;
    }

    const int iterations = 1000;
    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < iterations; ++i) {
        auto out = model.run(input);
        (void)out;
    }
    auto end = std::chrono::high_resolution_clock::now();

    double totalMs = std::chrono::duration<double, std::milli>(end - start).count();
    double latencyPerStepMs = totalMs / iterations;
    double fps = 1000.0 / latencyPerStepMs;

    std::cout << "  -> Total time: " << totalMs << " ms for " << iterations << " inferences." << std::endl;
    std::cout << "  -> Mean latency: " << latencyPerStepMs << " ms (" << fps << " inferences/sec)." << std::endl;

    // Requirement: < 0.5 ms per single-core step (development_plan.md line 639)
    assert(latencyPerStepMs < 0.5);
    std::cout << "  -> PASSED: Latency " << latencyPerStepMs << " ms < 0.50 ms threshold." << std::endl;
}

void testEndToEndAutonomousFlight() {
    std::cout << "[Test 5] Executing end-to-end 20-waypoint autonomous flight in C++..." << std::endl;
    SimulationConfig config = SimulationConfig::createDefault();
    config.simulation.randomSeed = 42;
    config.fleet.numUavs = 1;

    Simulator sim(config);
    ONNXNavigationPolicy policy(
        "models/navigation/nav_policy.onnx",
        "models/navigation/normalization.json"
    );

    int waypointsReached = 0;
    const int targetWaypoints = 20;

    std::vector<Vec3> waypointList = {
        Vec3(50.0, 20.0, 25.0),
        Vec3(-40.0, 60.0, 30.0),
        Vec3(80.0, -50.0, 20.0),
        Vec3(-70.0, -80.0, 35.0),
        Vec3(30.0, 90.0, 25.0),
        Vec3(100.0, 100.0, 30.0),
        Vec3(-90.0, 10.0, 20.0),
        Vec3(10.0, -90.0, 25.0),
        Vec3(60.0, -30.0, 30.0),
        Vec3(-50.0, 40.0, 35.0),
        Vec3(40.0, 50.0, 20.0),
        Vec3(-30.0, -60.0, 25.0),
        Vec3(70.0, 80.0, 30.0),
        Vec3(-80.0, -30.0, 35.0),
        Vec3(20.0, 70.0, 25.0),
        Vec3(90.0, -70.0, 20.0),
        Vec3(-60.0, 80.0, 30.0),
        Vec3(50.0, -60.0, 35.0),
        Vec3(-40.0, -40.0, 25.0),
        Vec3(0.0, 0.0, 20.0)
    };

    Vec3 currentStart(0.0, 0.0, 25.0);
    for (int wpIdx = 0; wpIdx < targetWaypoints; ++wpIdx) {
        Vec3 target = waypointList[wpIdx];
        sim.resetNavigationTask(1000 + wpIdx, currentStart, target, 0.0);

        bool reached = false;
        const int maxSteps = 1200; // 120 seconds max per waypoint
        for (int step = 0; step < maxSteps; ++step) {
            auto obs = sim.getObservation(0);
            if (obs.distanceToTarget <= 2.0) {
                reached = true;
                break;
            }

            const auto& uav = sim.getFleet().getUAV(0);
            auto cmd = policy.computeAction(uav, sim.getMap(), config.simulation.dtPolicy);
            auto res = sim.step({cmd});
            if (res.terminated[0] && sim.navigationTaskSucceeded()) {
                reached = true;
                break;
            }
        }

        (void)reached;
        assert(reached && "UAV must reach waypoint within time limit");
        ++waypointsReached;
        currentStart = target;
    }

    assert(waypointsReached == targetWaypoints);
    assert(sim.getMetrics().getBuildingCollisionCount() == 0);
    assert(sim.getMetrics().getUavCollisionCount() == 0);
    std::cout << "  -> PASSED: Successfully navigated all " << waypointsReached
              << " consecutive waypoints with 0 crashes!" << std::endl;
}

int main() {
    std::cout << "===========================================================" << std::endl;
    std::cout << "     AeroFleet C++ Embedded ONNX Runtime Test Suite        " << std::endl;
    std::cout << "===========================================================" << std::endl;

    testONNXModelLoadingAndMetadata();
    testSafetyLayerInterventions();
    testONNXNavigationPolicyObservationAndAction();
    testInferenceLatencyBenchmark();
    testEndToEndAutonomousFlight();

    std::cout << "\n[ALL TESTS PASSED] 100% Embedded C++ ONNX Runtime tests verified!" << std::endl;
    return 0;
}
