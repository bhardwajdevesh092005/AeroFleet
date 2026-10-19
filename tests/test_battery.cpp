#include <iostream>
#include <cassert>
#include <vector>
#include "physics/BatteryModel.hpp"
#include "world/CityMap.hpp"
#include "fleet/UAVState.hpp"

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

void test_payload_sensitivity() {
    FleetSettings fleet;
    EnergySettings energy;
    BatteryModel model(fleet, energy);

    UAVState unladenUav(1, Vec3(0, 0, 50));
    unladenUav.velocity = Vec3(10.0, 0.0, 0.0);
    unladenUav.payloadMass = 0.0;
    unladenUav.batteryPercent = 100.0;

    UAVState ladenUav(2, Vec3(0, 0, 50));
    ladenUav.velocity = Vec3(10.0, 0.0, 0.0);
    ladenUav.payloadMass = 2.5; // 2.5 kg package
    ladenUav.batteryPercent = 100.0;

    double pUnladen = model.computePowerWatts(unladenUav.velocity, unladenUav.payloadMass);
    double pLaden = model.computePowerWatts(ladenUav.velocity, ladenUav.payloadMass);

    TEST_ASSERT(pLaden > pUnladen, "Laden UAV must consume strictly higher power than unladen UAV");

    // Integrate discharge for 100 seconds
    for (int i = 0; i < 100; ++i) {
        model.stepDischarge(unladenUav, 1.0);
        model.stepDischarge(ladenUav, 1.0);
    }

    TEST_ASSERT(ladenUav.batteryPercent < unladenUav.batteryPercent,
        "Laden UAV must have strictly lower remaining SoC after identical duration flight");
}

void test_vertical_climb_penalty() {
    FleetSettings fleet;
    EnergySettings energy;
    BatteryModel model(fleet, energy);

    Vec3 climbingVel(0.0, 0.0, 4.0);
    Vec3 levelVel(4.0, 0.0, 0.0);
    Vec3 descendingVel(0.0, 0.0, -4.0);

    double pClimb = model.computePowerWatts(climbingVel, 0.0);
    double pLevel = model.computePowerWatts(levelVel, 0.0);
    double pDescend = model.computePowerWatts(descendingVel, 0.0);

    TEST_ASSERT(pClimb > pLevel, "Climbing UAV must consume strictly more power than level flight at same speed");
    TEST_ASSERT(pClimb > pDescend, "Climbing UAV must consume strictly more power than descending UAV at same speed");
    TEST_ASSERT(std::abs(pDescend - pLevel) < 1e-6,
        "Descending UAV must not pay vertical climb penalty and equal horizontal flight at same speed");
    TEST_ASSERT(std::abs((pClimb - pDescend) - (energy.climbPenaltyCoefficient * 4.0)) < 1e-6,
        "Climb power excess must equal climbPenaltyCoefficient * climbSpeed exactly");
}

void test_aerodynamic_drag_cubic() {
    FleetSettings fleet;
    EnergySettings energy;
    BatteryModel model(fleet, energy);

    Vec3 v1(5.0, 0.0, 0.0);
    Vec3 v2(10.0, 0.0, 0.0);
    Vec3 v3(15.0, 0.0, 0.0);

    double p1 = model.computePowerWatts(v1, 0.0);
    double p2 = model.computePowerWatts(v2, 0.0);
    double p3 = model.computePowerWatts(v3, 0.0);

    // Delta between 10m/s and 15m/s must be significantly larger than between 5m/s and 10m/s
    double delta1 = p2 - p1;
    double delta2 = p3 - p2;

    TEST_ASSERT(delta2 > delta1, "Airspeed power scaling must be superlinear (cubic drag law)");
}

void test_docking_and_recharging() {
    FleetSettings fleet;
    EnergySettings energy;
    fleet.dockingRadius = 1.5;
    fleet.chargeRatePercentPerSec = 2.0;
    BatteryModel model(fleet, energy);

    ChargingStation station;
    station.id = 1;
    station.position = Vec3(50.0, 50.0, 0.0);
    station.capacity = 2;
    station.occupiedCount = 0;

    UAVState uav(10, Vec3(50.5, 50.5, 0.0)); // distance = sqrt(0.5) ~ 0.707m <= 1.5m
    uav.velocity = Vec3(0.0, 0.0, 0.0);
    uav.batteryPercent = 40.0;
    uav.mode = UAVMode::RETURNING_TO_CHARGE;

    // 1. Docking
    bool docked = model.dock(uav, station);
    TEST_ASSERT(docked, "Stationary UAV in range must dock successfully");
    TEST_ASSERT(uav.mode == UAVMode::CHARGING, "Docked UAV mode must transition to CHARGING");
    TEST_ASSERT(station.occupiedCount == 1, "Station occupied slots must increment to 1");
    TEST_ASSERT(uav.isGrounded, "Docked UAV must be grounded");

    // 2. Charging for 10 seconds at 2.0%/s
    double added = model.stepCharge(uav, 10.0);
    TEST_ASSERT(std::abs(added - 20.0) < 1e-4, "Charge step must add exactly 20% SoC");
    TEST_ASSERT(std::abs(uav.batteryPercent - 60.0) < 1e-4, "Battery SoC must equal 60%");

    // 3. Undocking
    bool undocked = model.undock(uav, station);
    TEST_ASSERT(undocked, "UAV must undock successfully");
    TEST_ASSERT(uav.mode == UAVMode::IDLE, "Undocked UAV mode must transition to IDLE");
    TEST_ASSERT(station.occupiedCount == 0, "Station occupied slots must decrement to 0");
}

void test_docking_speed_and_range_rejection() {
    FleetSettings fleet;
    EnergySettings energy;
    fleet.dockingRadius = 1.5;
    BatteryModel model(fleet, energy);

    ChargingStation station;
    station.id = 1;
    station.position = Vec3(0.0, 0.0, 0.0);
    station.capacity = 2;

    // Moving too fast (speed > 0.5 m/s)
    UAVState fastUav(1, Vec3(0.5, 0.0, 0.0));
    fastUav.velocity = Vec3(5.0, 0.0, 0.0);
    TEST_ASSERT(!model.dock(fastUav, station), "High-speed UAV must be rejected from docking");

    // Out of range (distance = 5.0m > 1.5m)
    UAVState farUav(2, Vec3(5.0, 0.0, 0.0));
    farUav.velocity = Vec3(0.0, 0.0, 0.0);
    TEST_ASSERT(!model.dock(farUav, station), "Out-of-range UAV must be rejected from docking");
}

void test_charging_station_capacity_limit() {
    FleetSettings fleet;
    EnergySettings energy;
    BatteryModel model(fleet, energy);

    ChargingStation station;
    station.id = 1;
    station.position = Vec3(0.0, 0.0, 0.0);
    station.capacity = 1; // Only 1 slot
    station.occupiedCount = 0;

    UAVState uavA(1, Vec3(0.2, 0.0, 0.0));
    UAVState uavB(2, Vec3(0.3, 0.0, 0.0));

    TEST_ASSERT(model.dock(uavA, station), "First UAV docks into single available slot");
    TEST_ASSERT(station.occupiedCount == 1, "Station occupiedCount must be 1");
    TEST_ASSERT(!station.hasAvailableSlot(), "Station must report full");

    TEST_ASSERT(!model.dock(uavB, station), "Second UAV must be rejected when station is at capacity");
    TEST_ASSERT(station.occupiedCount == 1, "Station occupiedCount must remain 1");

    // Free slot
    model.undock(uavA, station);
    TEST_ASSERT(station.hasAvailableSlot(), "Station must have available slot after undocking");

    TEST_ASSERT(model.dock(uavB, station), "Second UAV can now dock after slot freed");
}

void test_battery_depletion_emergency_cutoff() {
    FleetSettings fleet;
    EnergySettings energy;
    BatteryModel model(fleet, energy);

    UAVState uav(1, Vec3(0.0, 0.0, 50.0));
    uav.velocity = Vec3(10.0, 0.0, 0.0);
    uav.batteryPercent = 0.05; // 0.05% remaining
    uav.mode = UAVMode::FLYING_TO_DESTINATION;

    // One large discharge step should deplete battery
    model.stepDischarge(uav, 10.0);

    TEST_ASSERT(uav.batteryPercent == 0.0, "Battery percentage must clamp exactly at 0.0%");
    TEST_ASSERT(uav.mode == UAVMode::EMERGENCY, "Depleted battery must trigger EMERGENCY mode");
    TEST_ASSERT(uav.velocity == Vec3(0.0, 0.0, 0.0), "Depleted battery must zero UAV velocity");

    // Subsequent steps must not produce negative SoC or NaN
    model.stepDischarge(uav, 5.0);
    TEST_ASSERT(uav.batteryPercent == 0.0, "Subsequent steps must keep battery at 0.0%");
    TEST_ASSERT(!std::isnan(uav.batteryPercent), "Battery percentage must not be NaN");
}

void test_battery_recharge_clamping() {
    FleetSettings fleet;
    EnergySettings energy;
    fleet.chargeRatePercentPerSec = 5.0;
    BatteryModel model(fleet, energy);

    UAVState uav(1, Vec3(0.0, 0.0, 0.0));
    uav.batteryPercent = 95.0;
    uav.mode = UAVMode::CHARGING;

    // Charging for 5 seconds at 5%/s would add 25%, reaching 120% without clamping
    model.stepCharge(uav, 5.0);

    TEST_ASSERT(uav.batteryPercent == 100.0, "Battery recharge must strictly clamp to 100.0%");
}

void test_reserve_and_critical_thresholds() {
    FleetSettings fleet;
    EnergySettings energy;
    fleet.reserveBatteryPercent = 20.0;
    fleet.criticalBatteryPercent = 10.0;
    BatteryModel model(fleet, energy);

    TEST_ASSERT(!model.isReserve(25.0) && !model.isCritical(25.0), "25% is above reserve and critical");
    TEST_ASSERT(model.isReserve(20.0) && !model.isCritical(20.0), "20% triggers reserve");
    TEST_ASSERT(model.isReserve(15.0) && !model.isCritical(15.0), "15% is in reserve range");
    TEST_ASSERT(model.isReserve(10.0) && model.isCritical(10.0), "10% triggers critical");
    TEST_ASSERT(model.isCritical(5.0), "5% is critical");
    TEST_ASSERT(model.isDepleted(0.0), "0% is depleted");
}

int main() {
    std::cout << "========================================\n";
    std::cout << "      AeroFleet Battery Tests           \n";
    std::cout << "========================================\n";

    RUN_TEST(test_payload_sensitivity);
    RUN_TEST(test_vertical_climb_penalty);
    RUN_TEST(test_aerodynamic_drag_cubic);
    RUN_TEST(test_docking_and_recharging);
    RUN_TEST(test_docking_speed_and_range_rejection);
    RUN_TEST(test_charging_station_capacity_limit);
    RUN_TEST(test_battery_depletion_emergency_cutoff);
    RUN_TEST(test_battery_recharge_clamping);
    RUN_TEST(test_reserve_and_critical_thresholds);

    std::cout << "========================================\n";
    std::cout << "Summary: " << g_testsPassed << " Passed, " << g_testsFailed << " Failed\n";
    std::cout << "========================================\n";

    return (g_testsFailed == 0) ? 0 : 1;
}
