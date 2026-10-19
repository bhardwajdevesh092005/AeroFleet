#include <iostream>
#include <cassert>
#include <cmath>
#include <vector>
#include "core/Simulator.hpp"
#include "core/Config.hpp"
#include "math/MathUtils.hpp"

using namespace aerofleet;

void testSimulatorInitialization() {
    std::cout << "[Test 1] Testing Simulator Initialization and Config Loading...\n";
    Simulator sim("config/simulation.json");

    assert(sim.getFleetSize() == 10);
    assert(sim.getStepCount() == 0);
    assert(sim.getSimulatedTime() == 0.0);
    assert(!sim.getMap().buildings.empty());
    assert(!sim.getMap().stores.empty());
    assert(!sim.getMap().chargingStations.empty());

    WorldSnapshot snap = sim.getSnapshot();
    assert(snap.uavs.size() == 10);
    assert(snap.stepCount == 0);
    assert(snap.metrics.totalDelivered == 0);

    std::cout << "  -> Passed: Simulator successfully initialized with "
              << sim.getFleetSize() << " UAVs and "
              << sim.getMap().buildings.size() << " buildings.\n";
}

void testObservationExtraction() {
    std::cout << "[Test 2] Testing Observation Extraction and Vectorization...\n";
    Simulator sim("config/simulation.json");
    sim.reset(42);

    for (size_t i = 0; i < sim.getFleetSize(); ++i) {
        Observation obs = sim.getObservation(static_cast<UAVId>(i));
        assert(obs.uavId == static_cast<UAVId>(i));
        assert(obs.lidarBeams.size() == 44);
        assert(obs.nearbyUavs.size() == 18);
        assert(obs.batteryFraction >= 0.0 && obs.batteryFraction <= 1.0);

        for (float beam : obs.lidarBeams) {
            assert(beam >= 0.0f && beam <= 1.0f);
        }

        std::vector<float> vec = obs.toVector();
        assert(vec.size() == Observation::vectorSize());
        assert(vec.size() == 72);

        for (float val : vec) {
            assert(!std::isnan(val) && !std::isinf(val));
        }
    }

    auto allObs = sim.getAllObservations();
    assert(allObs.size() == sim.getFleetSize());

    std::cout << "  -> Passed: Observation dimensionality verified (72 floats, 44 lidar, 18 neighbor states).\n";
}

void testHeuristicStepping() {
    std::cout << "[Test 3] Testing Heuristic Multi-Step Stepping...\n";
    Simulator sim("config/simulation.json");
    sim.reset(100);

    const size_t steps = 50;
    for (size_t s = 0; s < steps; ++s) {
        StepResult res = sim.stepHeuristic();

        assert(res.observations.size() == sim.getFleetSize());
        assert(res.rewards.size() == sim.getFleetSize());
        assert(res.terminated.size() == sim.getFleetSize());
        assert(res.truncated.size() == sim.getFleetSize());

        for (size_t i = 0; i < sim.getFleetSize(); ++i) {
            assert(!std::isnan(res.rewards[i]));
            assert(!std::isinf(res.rewards[i]));
            assert(res.observations[i].toVector().size() == 72);
        }
    }

    assert(sim.getStepCount() > 0);
    assert(sim.getSimulatedTime() > 0.0);

    WorldSnapshot snap = sim.getSnapshot();
    assert(snap.metrics.totalCreated > 0);

    std::cout << "  -> Passed: 50 heuristic policy steps executed smoothly. Time: "
              << sim.getSimulatedTime() << "s, Requests Created: "
              << snap.metrics.totalCreated << ".\n";
}

void testActionStepping() {
    std::cout << "[Test 4] Testing Direct Velocity Command Stepping...\n";
    Simulator sim("config/simulation.json");
    sim.reset(200);

    // Initial position of UAV 0
    Vec3 initialPos = sim.getFleet().getUAV(0).position;

    // Command forward velocity along X axis
    std::vector<VelocityCommand> actions(sim.getFleetSize());
    actions[0] = VelocityCommand(Vec3(8.0, 0.0, 2.0));

    for (int s = 0; s < 20; ++s) {
        sim.step(actions);
    }

    Vec3 finalPos = sim.getFleet().getUAV(0).position;
    assert(finalPos.x > initialPos.x); // Moved forward in X
    assert(sim.getFleet().getUAV(0).velocity.x > 0.0);

    std::cout << "  -> Passed: Commanded velocity integrated into rigid kinematics. Delta X = "
              << (finalPos.x - initialPos.x) << " m.\n";
}

void testSimulatorDeterminism() {
    std::cout << "[Test 5] Testing Deterministic Reproducibility Across Resets...\n";
    Simulator simA("config/simulation.json");
    simA.reset(999);

    for (int i = 0; i < 30; ++i) {
        simA.stepHeuristic();
    }

    Simulator simB("config/simulation.json");
    simB.reset(999);

    for (int i = 0; i < 30; ++i) {
        simB.stepHeuristic();
    }

    for (size_t i = 0; i < simA.getFleetSize(); ++i) {
        const auto& uavA = simA.getFleet().getUAV(i);
        const auto& uavB = simB.getFleet().getUAV(i);

        assert(uavA.position.x == uavB.position.x);
        assert(uavA.position.y == uavB.position.y);
        assert(uavA.position.z == uavB.position.z);
        assert(uavA.velocity.x == uavB.velocity.x);
        assert(uavA.batteryPercent == uavB.batteryPercent);
    }

    std::cout << "  -> Passed: Bit-for-bit identical trajectory across independent instances with seed 999.\n";
}

int main() {
    std::cout << "========================================\n";
    std::cout << "  AeroFleet Simulator C++ Test Suite    \n";
    std::cout << "========================================\n";

    testSimulatorInitialization();
    testObservationExtraction();
    testHeuristicStepping();
    testActionStepping();
    testSimulatorDeterminism();

    std::cout << "========================================\n";
    std::cout << "  All Simulator C++ Tests PASSED! (5/5) \n";
    std::cout << "========================================\n";
    return 0;
}

