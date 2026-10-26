#include <cassert>
#include <iostream>
#include <vector>
#include "core/Simulator.hpp"
#include "core/Config.hpp"

using namespace aerofleet;

void testAcceleratedDischarge() {
    std::cout << "[C++ Battery Stress 1] Testing accelerated discharge scaling..." << std::endl;
    Simulator sim("config/simulation.json");
    sim.reset(42);

    sim.setBatteryDischargeMultiplier(1.0);
    for (int i = 0; i < 50; ++i) {
        sim.stepHeuristic();
    }
    double soc1 = sim.getUAVBatteryPercent(0);
    double drain1 = 100.0 - soc1;

    sim.reset(42);
    sim.setBatteryDischargeMultiplier(10.0);
    for (int i = 0; i < 50; ++i) {
        sim.stepHeuristic();
    }
    double soc10 = sim.getUAVBatteryPercent(0);
    double drain10 = 100.0 - soc10;

    std::cout << "  -> 1x Drain: " << drain1 << "%, 10x Drain: " << drain10 << "%" << std::endl;
    assert(drain10 > drain1 * 5.0);
    std::cout << "  -> PASSED: Accelerated discharge correctly verified." << std::endl;
}

void testReserveBatteryTriggerAndRejection() {
    std::cout << "[C++ Battery Stress 2] Testing reserve battery trigger and mission rejection..." << std::endl;
    Simulator sim("config/simulation.json");
    sim.reset(42);
    sim.setAutoCoordinatorDispatch(false);

    // Set battery below reserve
    sim.setUAVBatteryPercent(1, 15.0);
    assert(sim.getUAVBatteryPercent(1) == 15.0);

    // Step coordinator
    sim.stepCoordinator(1.0);
    UAVMode mode = sim.getUAVMode(1);
    assert(mode == UAVMode::RETURNING_TO_CHARGE || mode == UAVMode::CHARGING);

    // Verify rejection of new missions
    for (int i = 0; i < 30; ++i) {
        sim.stepCoordinator(1.0);
        if (!sim.getPendingRequests().empty()) break;
    }
    assert(!sim.getPendingRequests().empty());
    RequestId reqId = sim.getPendingRequests().front().id;

    assert(!sim.isEnergyFeasible(1, reqId));
    assert(!sim.dispatchRequest(1, reqId));
    std::cout << "  -> PASSED: Mission rejection on low battery verified." << std::endl;
}

void testEmergencyDepletion() {
    std::cout << "[C++ Battery Stress 3] Testing emergency mode on complete battery depletion..." << std::endl;
    Simulator sim("config/simulation.json");
    sim.reset(42);
    sim.setAutoCoordinatorDispatch(false);

    sim.setUAVBatteryPercent(2, 0.0);
    sim.stepCoordinator(1.0);

    assert(sim.getUAVBatteryPercent(2) == 0.0);
    assert(sim.getUAVMode(2) == UAVMode::EMERGENCY);
    const auto& uav = sim.getFleet().getUAV(2);
    assert(uav.speed() == 0.0);
    std::cout << "  -> PASSED: Emergency shutdown on 0% battery verified." << std::endl;
}

int main() {
    std::cout << "===========================================================" << std::endl;
    std::cout << "     AeroFleet C++ Battery Drain Stress Test Suite         " << std::endl;
    std::cout << "===========================================================" << std::endl;

    testAcceleratedDischarge();
    testReserveBatteryTriggerAndRejection();
    testEmergencyDepletion();

    std::cout << "\n[ALL TESTS PASSED] C++ Battery Stress suite verified!" << std::endl;
    return 0;
}
