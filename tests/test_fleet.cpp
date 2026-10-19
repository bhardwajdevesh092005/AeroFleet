#include <iostream>
#include <cassert>
#include <vector>
#include "fleet/FleetManager.hpp"
#include "world/WorldGenerator.hpp"
#include "requests/DeliveryRequest.hpp"

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

void test_fleet_initialization() {
    FleetSettings fs;
    fs.numUavs = 5;
    EnergySettings es;
    WorldSettings ws;
    CityMap map = WorldGenerator::generate(ws, 42);

    FleetManager fleet(fs, es, map);

    TEST_ASSERT(fleet.size() == 5, "Fleet size must match configuration");
    TEST_ASSERT(fleet.getIdleCount() == 5, "All newly spawned UAVs must be IDLE");

    for (size_t i = 0; i < fleet.size(); ++i) {
        const auto& uav = fleet.getUAV(i);
        TEST_ASSERT(uav.id == static_cast<UAVId>(i), "UAV ID must match index");
        TEST_ASSERT(uav.batteryPercent == 100.0, "Initial battery must be 100%");
        TEST_ASSERT(uav.payloadMass == 0.0, "Initial payload must be 0");
        TEST_ASSERT(uav.activeRequestId == -1, "No request must be active initially");
    }
}

void test_full_delivery_cycle() {
    FleetSettings fs;
    fs.numUavs = 2;
    fs.pickupDwellSeconds = 2.0;
    fs.dropoffDwellSeconds = 2.0;
    EnergySettings es;
    WorldSettings ws;
    CityMap map = WorldGenerator::generate(ws, 42);

    FleetManager fleet(fs, es, map);

    Vec3 pickup(10.0, 10.0, 15.0);
    Vec3 dropoff(80.0, 80.0, 15.0);
    DeliveryRequest req(101, 1, pickup, 2, dropoff, 0.0, 120.0, 2.0);

    // 1. Assign to UAV 0
    bool assigned = fleet.assignRequest(0, req);
    TEST_ASSERT(assigned, "Assignment to IDLE UAV must succeed");
    TEST_ASSERT(fleet.getUAV(0).mode == UAVMode::ASSIGNED, "UAV mode must be ASSIGNED");

    std::vector<DeliveryRequest> delivered;
    std::vector<DeliveryRequest> requeued;

    // 2. Step: transition to FLYING_TO_PICKUP
    fleet.step(0.1, 0.1, map, delivered, requeued);
    TEST_ASSERT(fleet.getUAV(0).mode == UAVMode::FLYING_TO_PICKUP, "Mode must transition to FLYING_TO_PICKUP");

    // 3. Simulate arrival at pickup
    fleet.getUAV(0).position = pickup;
    fleet.getUAV(0).velocity = Vec3(0, 0, 0);
    fleet.step(0.1, 0.2, map, delivered, requeued);
    TEST_ASSERT(fleet.getUAV(0).mode == UAVMode::PICKING_UP, "Mode must transition to PICKING_UP upon arrival");

    // 4. Dwell at pickup for 2.0s
    for (int i = 0; i < 20; ++i) {
        fleet.step(0.1, 0.3 + i * 0.1, map, delivered, requeued);
    }
    TEST_ASSERT(fleet.getUAV(0).mode == UAVMode::FLYING_TO_DESTINATION,
        "Mode must transition to FLYING_TO_DESTINATION after dwell time");
    TEST_ASSERT(fleet.getUAV(0).payloadMass == 2.0, "UAV must have acquired payload mass 2.0kg");

    // 5. Simulate arrival at dropoff
    fleet.getUAV(0).position = dropoff;
    fleet.getUAV(0).velocity = Vec3(0, 0, 0);
    fleet.step(0.1, 3.0, map, delivered, requeued);
    TEST_ASSERT(fleet.getUAV(0).mode == UAVMode::DROPPING_OFF, "Mode must transition to DROPPING_OFF upon arrival");

    // 6. Dwell at dropoff for 2.0s
    for (int i = 0; i < 20; ++i) {
        fleet.step(0.1, 3.1 + i * 0.1, map, delivered, requeued);
    }
    TEST_ASSERT(fleet.getUAV(0).mode == UAVMode::IDLE, "Mode must return to IDLE after delivery");
    TEST_ASSERT(fleet.getUAV(0).payloadMass == 0.0, "Payload must be detached (0kg)");
    TEST_ASSERT(delivered.size() == 1, "Delivered list must record exactly 1 request");
    TEST_ASSERT(delivered[0].status == RequestStatus::DELIVERED, "Request status must be DELIVERED");
}

void test_reject_busy_uav_assignment() {
    FleetSettings fs;
    fs.numUavs = 1;
    EnergySettings es;
    WorldSettings ws;
    CityMap map = WorldGenerator::generate(ws, 42);

    FleetManager fleet(fs, es, map);

    DeliveryRequest req1(1, 1, Vec3(0, 0, 15), 2, Vec3(50, 50, 15), 0.0, 100.0, 1.0);
    DeliveryRequest req2(2, 1, Vec3(0, 0, 15), 3, Vec3(60, 60, 15), 0.0, 100.0, 1.0);

    TEST_ASSERT(fleet.assignRequest(0, req1), "First assignment must succeed");
    TEST_ASSERT(!fleet.assignRequest(0, req2), "Second assignment to busy UAV must be rejected");
}

void test_reject_low_battery_assignment() {
    FleetSettings fs;
    fs.numUavs = 1;
    fs.reserveBatteryPercent = 20.0;
    EnergySettings es;
    WorldSettings ws;
    CityMap map = WorldGenerator::generate(ws, 42);

    FleetManager fleet(fs, es, map);

    // Drain battery below reserve
    fleet.getUAV(0).batteryPercent = 18.0;

    DeliveryRequest req(1, 1, Vec3(0, 0, 15), 2, Vec3(50, 50, 15), 0.0, 100.0, 1.0);
    TEST_ASSERT(!fleet.assignRequest(0, req), "Assignment to low-battery UAV must be rejected");
}

void test_low_battery_abort_and_requeue() {
    FleetSettings fs;
    fs.numUavs = 1;
    fs.criticalBatteryPercent = 10.0;
    EnergySettings es;
    WorldSettings ws;
    CityMap map = WorldGenerator::generate(ws, 42);

    FleetManager fleet(fs, es, map);

    DeliveryRequest req(1, 1, Vec3(10, 10, 15), 2, Vec3(50, 50, 15), 0.0, 100.0, 1.0);
    fleet.assignRequest(0, req);

    std::vector<DeliveryRequest> delivered;
    std::vector<DeliveryRequest> requeued;

    // Start flying
    fleet.step(0.1, 0.1, map, delivered, requeued);
    TEST_ASSERT(fleet.getUAV(0).mode == UAVMode::FLYING_TO_PICKUP, "Flying to pickup");

    // Critical battery occurs mid-flight
    fleet.getUAV(0).batteryPercent = 8.0;
    fleet.step(0.1, 0.2, map, delivered, requeued);

    TEST_ASSERT(fleet.getUAV(0).mode == UAVMode::EMERGENCY, "UAV must switch to EMERGENCY mode");
    TEST_ASSERT(requeued.size() == 1, "Unfulfilled request must be re-queued");
    TEST_ASSERT(requeued[0].status == RequestStatus::PENDING, "Re-queued request status must be PENDING");
    TEST_ASSERT(requeued[0].assignedUavId == -1, "Re-queued request assigned UAV must be cleared");
}

void test_charging_lifecycle_restoration() {
    FleetSettings fs;
    fs.numUavs = 1;
    fs.reserveBatteryPercent = 20.0;
    fs.chargeRatePercentPerSec = 10.0; // Fast charge for test
    EnergySettings es;
    WorldSettings ws;
    CityMap map = WorldGenerator::generate(ws, 42);

    FleetManager fleet(fs, es, map);

    // Set UAV to low battery and step
    fleet.getUAV(0).batteryPercent = 15.0;
    std::vector<DeliveryRequest> del, req;
    fleet.step(0.1, 0.1, map, del, req);

    TEST_ASSERT(fleet.getUAV(0).mode == UAVMode::RETURNING_TO_CHARGE,
        "Low-battery UAV must switch to RETURNING_TO_CHARGE");

    // Move UAV to charging station
    fleet.getUAV(0).position = map.chargingStations[0].position;
    fleet.getUAV(0).velocity = Vec3(0, 0, 0);
    fleet.step(0.1, 0.2, map, del, req);

    TEST_ASSERT(fleet.getUAV(0).mode == UAVMode::CHARGING, "UAV in station dock must be CHARGING");

    // Charge for 10 seconds at 10%/sec (adds 100%)
    for (int i = 0; i < 100; ++i) {
        fleet.step(0.1, 0.3 + i * 0.1, map, del, req);
    }

    TEST_ASSERT(fleet.getUAV(0).batteryPercent == 100.0, "Battery must be fully restored to 100%");
    TEST_ASSERT(fleet.getUAV(0).mode == UAVMode::IDLE, "Fully charged UAV must return to IDLE mode");
}

int main() {
    std::cout << "========================================\n";
    std::cout << "      AeroFleet Fleet Manager Tests     \n";
    std::cout << "========================================\n";

    RUN_TEST(test_fleet_initialization);
    RUN_TEST(test_full_delivery_cycle);
    RUN_TEST(test_reject_busy_uav_assignment);
    RUN_TEST(test_reject_low_battery_assignment);
    RUN_TEST(test_low_battery_abort_and_requeue);
    RUN_TEST(test_charging_lifecycle_restoration);

    std::cout << "========================================\n";
    std::cout << "Summary: " << g_testsPassed << " Passed, " << g_testsFailed << " Failed\n";
    std::cout << "========================================\n";

    return (g_testsFailed == 0) ? 0 : 1;
}

