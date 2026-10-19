#include "coordinator/HeuristicCoordinator.hpp"
#include "control/WaypointController.hpp"
#include "physics/CollisionSystem.hpp"
#include "physics/Kinematics.hpp"
#include "requests/RequestGenerator.hpp"

#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace aerofleet;

namespace {
int passed = 0;
int failed = 0;

void expect(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
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

CityMap integrationMap() {
    CityMap map;
    map.worldMin = Vec3(-100.0, -100.0, 0.0);
    map.worldMax = Vec3(100.0, 100.0, 100.0);
    map.chargingStations.push_back(ChargingStation{0, "Hub", Vec3(0.0, 0.0, 15.0), 2, 0});
    map.stores.push_back(StoreLocation{1, "Store", Vec3(20.0, 0.0, 15.0)});
    map.customers.push_back(CustomerLocation{1, "Customer", Vec3(40.0, 0.0, 15.0)});
    map.rebuildBroadphase(10.0);
    return map;
}

void test_generated_request_to_completed_delivery() {
    CityMap map = integrationMap();
    FleetSettings fleetSettings;
    fleetSettings.numUavs = 1;
    fleetSettings.pickupDwellSeconds = 0.04;
    fleetSettings.dropoffDwellSeconds = 0.04;
    fleetSettings.maxHorizontalSpeed = 10.0;
    fleetSettings.maxVerticalSpeed = 4.0;
    fleetSettings.maxAcceleration = 8.0;
    EnergySettings energy;

    FleetManager fleet(fleetSettings, energy, map, 15.0);
    RequestSettings requestSettings;
    requestSettings.minPayloadKg = 1.0;
    requestSettings.maxPayloadKg = 1.0;
    requestSettings.slackFactor = 20.0;
    requestSettings.bufferTimeSeconds = 1000.0;
    RequestGenerator generator(requestSettings, map, 10.0, 12345);
    std::vector<DeliveryRequest> requests{generator.generateSingle(0.0)};
    const RequestId requestId = requests.front().id;

    HeuristicCoordinatorConfig coordinatorConfig;
    coordinatorConfig.nominalSpeed = 10.0;
    coordinatorConfig.payloadCapacityKg = 2.0;
    HeuristicCoordinator coordinator(coordinatorConfig);
    coordinator.dispatch(fleet, requests, map, 0.0);
    expect(requests.front().status == RequestStatus::ASSIGNED, "generated request was not dispatched");

    KinematicsConfig kinematicsConfig;
    kinematicsConfig.maxHorizontalSpeed = fleetSettings.maxHorizontalSpeed;
    kinematicsConfig.maxVerticalSpeed = fleetSettings.maxVerticalSpeed;
    kinematicsConfig.maxAcceleration = fleetSettings.maxAcceleration;
    kinematicsConfig.maxJerk = 100.0;
    kinematicsConfig.groundAltitude = 0.0;
    kinematicsConfig.ceilingAltitude = 100.0;
    KinematicsIntegrator integrator(kinematicsConfig);
    WaypointControllerConfig navConfig;
    navConfig.maxHorizontalSpeed = fleetSettings.maxHorizontalSpeed;
    navConfig.maxVerticalSpeed = fleetSettings.maxVerticalSpeed;
    WaypointController navigator(navConfig);
    CollisionSystem collisions(fleetSettings.uavRadius);
    std::vector<DeliveryRequest> delivered;
    std::vector<DeliveryRequest> requeued;

    constexpr double dt = 0.02;
    double time = 0.0;
    bool sawPickup = false;
    bool sawPayload = false;
    for (int step = 0; step < 30000 && delivered.empty(); ++step) {
        fleet.step(dt, time, map, delivered, requeued);
        auto& uav = fleet.getUAV(0);
        sawPickup = sawPickup || uav.mode == UAVMode::PICKING_UP;
        sawPayload = sawPayload || uav.payloadMass > 0.0;

        const Vec3 previous = uav.position;
        VelocityCommand action;
        if (uav.mode == UAVMode::FLYING_TO_PICKUP || uav.mode == UAVMode::FLYING_TO_DESTINATION ||
            uav.mode == UAVMode::RETURNING_TO_CHARGE) {
            action = navigator.computeAction(uav, map, dt);
        }
        integrator.stepVelocityCommand(uav, action, dt);
        CollisionEvent collision;
        expect(!collisions.checkStaticCollision(previous, uav.position, map, collision),
               "integrated delivery trajectory collided with a building");
        fleet.getBatteryModel().stepDischarge(uav, dt);
        time += dt;
    }

    expect(delivered.size() == 1, "delivery did not complete before timeout");
    expect(delivered.front().id == requestId, "completed request ID changed during fleet lifecycle");
    expect(delivered.front().status == RequestStatus::DELIVERED, "fleet did not mark request delivered");
    expect(sawPickup && sawPayload, "integration did not exercise pickup dwell and payload transit");
    expect(fleet.getUAV(0).mode == UAVMode::IDLE, "UAV should return to idle after successful delivery");
    expect(fleet.getUAV(0).batteryPercent > fleetSettings.criticalBatteryPercent,
           "UAV exhausted battery during short integration mission");
    expect(requeued.empty(), "successful mission should not requeue request");
}

void test_integration_collision_forces_fail_safe_mode() {
    CityMap map = integrationMap();
    map.buildings.emplace_back(5, AABB(Vec3(4.0, -3.0, 0.0), Vec3(8.0, 3.0, 30.0)));
    map.rebuildBroadphase(10.0);
    FleetSettings fs;
    fs.numUavs = 1;
    FleetManager fleet(fs, EnergySettings{}, map, 15.0);
    auto& uav = fleet.getUAV(0);
    uav.position = Vec3(0.0, 0.0, 15.0);
    const Vec3 previous = uav.position;
    uav.position = Vec3(12.0, 0.0, 15.0);
    std::vector<Vec3> previousPositions{previous};
    auto events = CollisionSystem(0.5).evaluateAndResolve(fleet.getAllUAVs(), previousPositions, map, 0.02);
    expect(events.size() == 1, "swept collision system did not detect integrated building collision");
    expect(fleet.getUAV(0).mode == UAVMode::EMERGENCY, "collision did not place UAV in fail-safe mode");
    expect(fleet.getUAV(0).position.isApprox(previous), "collision response did not roll UAV back");
}
} // namespace

int main() {
    std::cout << "========================================\n"
              << " AeroFleet Cross-Module Integration Tests\n"
              << "========================================\n";
    run("test_generated_request_to_completed_delivery", test_generated_request_to_completed_delivery);
    run("test_integration_collision_forces_fail_safe_mode", test_integration_collision_forces_fail_safe_mode);
    std::cout << "========================================\nSummary: " << passed
              << " Passed, " << failed << " Failed\n========================================\n";
    return failed == 0 ? 0 : 1;
}
