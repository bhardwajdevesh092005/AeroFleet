#include "control/WaypointController.hpp"
#include "physics/CollisionSystem.hpp"
#include "physics/Kinematics.hpp"
#include "world/CityMap.hpp"

#include <cmath>
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

CityMap emptyMap() {
    CityMap map;
    map.worldMin = Vec3(-200.0, -200.0, 0.0);
    map.worldMax = Vec3(200.0, 200.0, 100.0);
    map.rebuildBroadphase(20.0);
    return map;
}

void test_open_space_reaches_waypoint() {
    CityMap map = emptyMap();
    KinematicsConfig kin;
    kin.maxHorizontalSpeed = 15.0;
    kin.maxVerticalSpeed = 5.0;
    kin.maxAcceleration = 10.0;
    kin.maxJerk = 100.0;
    kin.groundAltitude = 0.0;
    kin.ceilingAltitude = 100.0;
    KinematicsIntegrator integrator(kin);
    WaypointController controller(kin);

    UAVState uav(1, Vec3(0.0, 0.0, 10.0));
    uav.targetWaypoint = Vec3(100.0, 100.0, 30.0);
    std::vector<UAVState> fleet{uav};
    CollisionSystem collisions(0.5);
    bool arrived = false;
    double pathLength = 0.0;
    constexpr double dt = 0.02;
    for (int step = 0; step < 30000; ++step) {
        const Vec3 previous = fleet[0].position;
        const auto action = controller.computeAction(fleet[0], map, dt);
        integrator.stepVelocityCommand(fleet[0], action, dt);
        pathLength += previous.distance(fleet[0].position);
        CollisionEvent event;
        if (collisions.checkStaticCollision(previous, fleet[0].position, map, event)) {
            throw std::runtime_error("unexpected collision in open space");
        }
        if (fleet[0].position.distance(fleet[0].targetWaypoint) <= 1.0 && fleet[0].speed() < 0.5) {
            arrived = true;
            break;
        }
    }
    expect(arrived, "UAV did not reach waypoint within 600 seconds");
    if (!(pathLength >= 140.0 && pathLength < 220.0)) {
        throw std::runtime_error("open-space path length outside expected bounds: " + std::to_string(pathLength));
    }
}

void test_static_obstacle_avoidance() {
    CityMap map = emptyMap();
    map.buildings.emplace_back(7, AABB(Vec3(38.0, -12.0, 0.0), Vec3(62.0, 12.0, 32.0)));
    map.rebuildBroadphase(20.0);

    KinematicsConfig kin;
    kin.maxHorizontalSpeed = 12.0;
    kin.maxVerticalSpeed = 4.0;
    kin.maxAcceleration = 8.0;
    kin.maxJerk = 100.0;
    kin.groundAltitude = 0.0;
    kin.ceilingAltitude = 100.0;
    KinematicsIntegrator integrator(kin);
    WaypointControllerConfig navConfig;
    navConfig.maxHorizontalSpeed = kin.maxHorizontalSpeed;
    navConfig.maxVerticalSpeed = kin.maxVerticalSpeed;
    navConfig.influenceDistance = 35.0;
    navConfig.repulsiveGain = 100.0;
    navConfig.safetyRadius = 2.0;
    navConfig.verticalEscapeBias = 0.15;
    navConfig.tangentialGain = 0.25;
    WaypointController controller(navConfig);

    UAVState uav(1, Vec3(0.0, 0.0, 15.0));
    uav.targetWaypoint = Vec3(100.0, 0.0, 15.0);
    std::vector<UAVState> fleet{uav};
    CollisionSystem collisions(0.5);
    bool arrived = false;
    double pathLength = 0.0;
    constexpr double dt = 0.02;
    for (int step = 0; step < 30000; ++step) {
        const Vec3 previous = fleet[0].position;
        const auto action = controller.computeAction(fleet[0], map, dt);
        integrator.stepVelocityCommand(fleet[0], action, dt);
        pathLength += previous.distance(fleet[0].position);
        CollisionEvent event;
        if (collisions.checkStaticCollision(previous, fleet[0].position, map, event)) {
            throw std::runtime_error("controller trajectory collided with the building at " + fleet[0].position.toString());
        }
        if (fleet[0].position.distance(fleet[0].targetWaypoint) <= 1.0 && fleet[0].speed() < 0.5) {
            arrived = true;
            break;
        }
    }
    expect(arrived, "UAV failed to navigate around the building");
    expect(pathLength > 100.0, "obstacle-avoidance path did not make progress");
}

void test_arrival_and_invalid_timestep() {
    CityMap map = emptyMap();
    WaypointController controller;
    UAVState uav(1, Vec3(0.0, 0.0, 10.0));
    uav.targetWaypoint = Vec3(0.5, 0.0, 10.0);
    expect(controller.computeAction(uav, map, 0.1).desiredVelocity.lengthSquared() == 0.0,
           "controller should stop inside the arrival radius");
    uav.targetWaypoint = Vec3(20.0, 0.0, 10.0);
    expect(controller.computeAction(uav, map, 0.0).desiredVelocity.lengthSquared() == 0.0,
           "controller should return a safe zero command for non-positive dt");
}

void test_deterministic_obstacle_action() {
    CityMap map = emptyMap();
    map.buildings.emplace_back(1, AABB(Vec3(8.0, -4.0, 0.0), Vec3(12.0, 4.0, 20.0)));
    map.rebuildBroadphase(20.0);
    UAVState uav(3, Vec3(0.0, 0.0, 10.0));
    uav.targetWaypoint = Vec3(30.0, 0.0, 10.0);
    WaypointController controller;
    const auto first = controller.computeAction(uav, map, 0.1).desiredVelocity;
    const auto second = controller.computeAction(uav, map, 0.1).desiredVelocity;
    expect(first.isApprox(second, 1e-12), "same state must produce identical action");
}
} // namespace

int main() {
    std::cout << "========================================\n"
              << "   AeroFleet Heuristic Navigation Tests\n"
              << "========================================\n";
    run("test_open_space_reaches_waypoint", test_open_space_reaches_waypoint);
    run("test_static_obstacle_avoidance", test_static_obstacle_avoidance);
    run("test_arrival_and_invalid_timestep", test_arrival_and_invalid_timestep);
    run("test_deterministic_obstacle_action", test_deterministic_obstacle_action);
    std::cout << "========================================\nSummary: " << passed
              << " Passed, " << failed << " Failed\n========================================\n";
    return failed == 0 ? 0 : 1;
}
