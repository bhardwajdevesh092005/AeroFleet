#include "coordinator/HeuristicCoordinator.hpp"

#include <iostream>
#include <stdexcept>
#include <string>

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

CityMap makeMap() {
    CityMap map;
    map.worldMin = Vec3(-500.0, -500.0, 0.0);
    map.worldMax = Vec3(500.0, 500.0, 150.0);
    map.chargingStations.push_back(ChargingStation{0, "Home", Vec3(0.0, 0.0, 0.0), 10, 0});
    map.rebuildBroadphase();
    return map;
}

FleetSettings fleetSettings(int count) {
    FleetSettings settings;
    settings.numUavs = count;
    return settings;
}

DeliveryRequest request(RequestId id, const Vec3& pickup, const Vec3& dropoff, double created = 0.0, double deadline = 1000.0) {
    return DeliveryRequest(id, 1, pickup, 1, dropoff, created, deadline, 1.0);
}

void test_dispatch_does_not_double_book_uav() {
    auto map = makeMap();
    FleetManager fleet(fleetSettings(1), EnergySettings{}, map, 15.0);
    std::vector<DeliveryRequest> requests{
        request(1, Vec3(10.0, 0.0, 15.0), Vec3(20.0, 0.0, 15.0)),
        request(2, Vec3(20.0, 0.0, 15.0), Vec3(30.0, 0.0, 15.0))
    };
    HeuristicCoordinator coordinator;
    coordinator.dispatch(fleet, requests, map, 0.0);
    expect(fleet.getUAV(0).mode == UAVMode::ASSIGNED, "available UAV was not assigned");
    const int assignedCount = static_cast<int>(requests[0].status == RequestStatus::ASSIGNED) +
                              static_cast<int>(requests[1].status == RequestStatus::ASSIGNED);
    expect(assignedCount == 1, "one UAV was assigned to multiple requests");
}

void test_battery_and_charger_feasibility() {
    auto map = makeMap();
    FleetSettings settings = fleetSettings(2);
    FleetManager fleet(settings, EnergySettings{}, map, 15.0);
    fleet.getUAV(0).batteryPercent = 16.0;
    fleet.getUAV(1).batteryPercent = 100.0;
    std::vector<DeliveryRequest> requests{
        request(10, Vec3(80.0, 0.0, 15.0), Vec3(140.0, 0.0, 15.0))
    };
    HeuristicCoordinator coordinator;
    coordinator.dispatch(fleet, requests, map, 0.0);
    expect(requests[0].status == RequestStatus::ASSIGNED, "request should be feasible for the charged UAV");
    expect(requests[0].assignedUavId == fleet.getUAV(1).id, "coordinator selected an under-reserved UAV");

    auto noDockMap = map;
    noDockMap.chargingStations.clear();
    FleetManager noDockFleet(fleetSettings(1), EnergySettings{}, noDockMap, 15.0);
    std::vector<DeliveryRequest> noDockRequests{request(11, Vec3(10.0, 0.0, 15.0), Vec3(20.0, 0.0, 15.0))};
    coordinator.dispatch(noDockFleet, noDockRequests, noDockMap, 0.0);
    expect(noDockRequests[0].status == RequestStatus::PENDING, "must not dispatch when no return charger exists");
}

void test_overload_queues_without_deadlock() {
    auto map = makeMap();
    FleetManager fleet(fleetSettings(5), EnergySettings{}, map, 15.0);
    std::vector<DeliveryRequest> requests;
    requests.reserve(200);
    for (int i = 0; i < 200; ++i) {
        requests.push_back(request(i + 1,
            Vec3(20.0 + i, 0.0, 15.0), Vec3(40.0 + i, 10.0, 15.0),
            static_cast<double>(i), 2000.0));
    }
    HeuristicCoordinator coordinator;
    coordinator.dispatch(fleet, requests, map, 300.0);

    int assignedCount = 0;
    int pendingCount = 0;
    for (const auto& item : requests) {
        if (item.status == RequestStatus::ASSIGNED) ++assignedCount;
        else if (item.status == RequestStatus::PENDING) ++pendingCount;
    }
    expect(assignedCount == 5, "expected one request per available UAV");
    expect(pendingCount == 195, "unassigned requests should remain queued");
    expect(fleet.getIdleCount() == 0, "dispatch should use every feasible idle UAV");

    coordinator.dispatch(fleet, requests, map, 301.0);
    int assignedAfterSecondCall = 0;
    for (const auto& item : requests) if (item.status == RequestStatus::ASSIGNED) ++assignedAfterSecondCall;
    expect(assignedAfterSecondCall == assignedCount, "second dispatch double-booked an in-flight UAV");
}

void test_expired_oversize_and_stale_requests_are_skipped() {
    auto map = makeMap();
    FleetManager fleet(fleetSettings(2), EnergySettings{}, map, 15.0);
    auto expired = request(1, Vec3(1.0, 0.0, 15.0), Vec3(2.0, 0.0, 15.0), 0.0, 5.0);
    auto oversize = request(2, Vec3(1.0, 0.0, 15.0), Vec3(2.0, 0.0, 15.0));
    oversize.payloadMass = 4.0;
    auto terminal = request(3, Vec3(1.0, 0.0, 15.0), Vec3(2.0, 0.0, 15.0));
    terminal.transitionTo(RequestStatus::REJECTED);
    std::vector<DeliveryRequest> requests{expired, oversize, terminal};
    HeuristicCoordinator coordinator;
    coordinator.dispatch(fleet, requests, map, 10.0);
    for (const auto& item : requests) expect(item.status != RequestStatus::ASSIGNED, "infeasible request was dispatched");
}

void test_deadline_and_wait_age_priority() {
    auto map = makeMap();
    FleetManager fleet(fleetSettings(1), EnergySettings{}, map, 15.0);
    auto relaxed = request(1, Vec3(10.0, 0.0, 15.0), Vec3(20.0, 0.0, 15.0), 0.0, 1000.0);
    auto urgent = request(2, Vec3(10.0, 0.0, 15.0), Vec3(20.0, 0.0, 15.0), 0.0, 100.0);
    std::vector<DeliveryRequest> requests{relaxed, urgent};
    HeuristicCoordinator coordinator;
    coordinator.dispatch(fleet, requests, map, 10.0);
    expect(requests[1].status == RequestStatus::ASSIGNED, "more urgent deadline should win an otherwise equal assignment");

    FleetManager agedFleet(fleetSettings(1), EnergySettings{}, map, 15.0);
    auto newRequest = request(3, Vec3(10.0, 0.0, 15.0), Vec3(20.0, 0.0, 15.0), 9.0, 1000.0);
    auto oldRequest = request(4, Vec3(10.0, 0.0, 15.0), Vec3(20.0, 0.0, 15.0), 0.0, 1000.0);
    std::vector<DeliveryRequest> agedRequests{newRequest, oldRequest};
    coordinator.dispatch(agedFleet, agedRequests, map, 10.0);
    expect(agedRequests[1].status == RequestStatus::ASSIGNED, "older queued request should receive age priority");
}
} // namespace

int main() {
    std::cout << "========================================\n"
              << " AeroFleet Heuristic Coordinator Tests\n"
              << "========================================\n";
    run("test_dispatch_does_not_double_book_uav", test_dispatch_does_not_double_book_uav);
    run("test_battery_and_charger_feasibility", test_battery_and_charger_feasibility);
    run("test_overload_queues_without_deadlock", test_overload_queues_without_deadlock);
    run("test_expired_oversize_and_stale_requests_are_skipped", test_expired_oversize_and_stale_requests_are_skipped);
    run("test_deadline_and_wait_age_priority", test_deadline_and_wait_age_priority);
    std::cout << "========================================\nSummary: " << passed
              << " Passed, " << failed << " Failed\n========================================\n";
    return failed == 0 ? 0 : 1;
}
