#include <iostream>
#include <cassert>
#include <vector>
#include "physics/CollisionSystem.hpp"
#include "world/CityMap.hpp"

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

void test_swept_building_collision() {
    CityMap map;
    // Add thin wall obstacle at X in [10.0, 10.2], Y in [-10.0, 10.0], Z in [0.0, 20.0]
    AABB wallBounds(Vec3(10.0, -10.0, 0.0), Vec3(10.2, 10.0, 20.0));
    map.buildings.emplace_back(101, wallBounds);
    map.rebuildBroadphase(50.0);

    CollisionSystem colSys(0.5);

    // UAV moving from X=8.0 to X=12.0 in a single timestep
    UAVState uav;
    uav.id = 1;
    uav.position = Vec3(12.0, 0.0, 10.0);
    Vec3 prevPos(8.0, 0.0, 10.0);

    std::vector<UAVState> uavs = { uav };
    std::vector<Vec3> prevPositions = { prevPos };

    auto events = colSys.evaluateAndResolve(uavs, prevPositions, map, 1.0);

    TEST_ASSERT(events.size() == 1, "Exactly 1 static collision event must be registered");
    TEST_ASSERT(events[0].type == CollisionType::BUILDING, "Event type must be BUILDING");
    TEST_ASSERT(events[0].buildingId == 101, "Building ID must match");
    TEST_ASSERT(uavs[0].position == prevPos, "UAV position must roll back to previous safe position");
    TEST_ASSERT(uavs[0].mode == UAVMode::EMERGENCY, "UAV mode must switch to EMERGENCY upon collision");
}

void test_safe_building_clearance() {
    CityMap map;
    AABB wallBounds(Vec3(10.0, -10.0, 0.0), Vec3(10.2, 10.0, 20.0));
    map.buildings.emplace_back(101, wallBounds);
    map.rebuildBroadphase(50.0);

    CollisionSystem colSys(0.5);

    // UAV moving parallel to wall at Y=12.0 (wall ends at Y=10.0, clearance is 2.0m > radius 0.5m)
    UAVState uav;
    uav.id = 1;
    uav.position = Vec3(20.0, 12.0, 10.0);
    Vec3 prevPos(0.0, 12.0, 10.0);

    std::vector<UAVState> uavs = { uav };
    std::vector<Vec3> prevPositions = { prevPos };

    auto events = colSys.evaluateAndResolve(uavs, prevPositions, map, 1.0);

    TEST_ASSERT(events.empty(), "Safe flight with adequate clearance must produce 0 collision events");
    TEST_ASSERT(uavs[0].position == Vec3(20.0, 12.0, 10.0), "Position must remain unchanged");
}

void test_uav_uav_head_on_collision() {
    CityMap map; // Empty map
    map.rebuildBroadphase(50.0);

    CollisionSystem colSys(0.5); // radius 0.5m, combined collision distance = 1.0m

    // UAV 1 flying East: from (0, 0, 10) to (10, 0, 10)
    UAVState uav1;
    uav1.id = 1;
    uav1.position = Vec3(10.0, 0.0, 10.0);
    Vec3 prev1(0.0, 0.0, 10.0);

    // UAV 2 flying West: from (10, 0, 10) to (0, 0, 10)
    UAVState uav2;
    uav2.id = 2;
    uav2.position = Vec3(0.0, 0.0, 10.0);
    Vec3 prev2(10.0, 0.0, 10.0);

    std::vector<UAVState> uavs = { uav1, uav2 };
    std::vector<Vec3> prevPositions = { prev1, prev2 };

    auto events = colSys.evaluateAndResolve(uavs, prevPositions, map, 2.5);

    TEST_ASSERT(events.size() == 1, "Head-on collision must register exactly 1 UAV_UAV event");
    TEST_ASSERT(events[0].type == CollisionType::UAV_UAV, "Event type must be UAV_UAV");
    TEST_ASSERT(events[0].uavIdA == 1 && events[0].uavIdB == 2, "Incident must reference both UAV IDs");
    TEST_ASSERT(uavs[0].mode == UAVMode::EMERGENCY && uavs[1].mode == UAVMode::EMERGENCY,
        "Both UAVs must transition to EMERGENCY mode");
    TEST_ASSERT(uavs[0].position == prev1 && uavs[1].position == prev2,
        "Both UAVs must roll back to previous positions");
}

void test_uav_uav_safe_vertical_separation() {
    CityMap map;
    map.rebuildBroadphase(50.0);
    CollisionSystem colSys(0.5);

    // UAV 1 flying at Z = 10m
    UAVState uav1;
    uav1.id = 1;
    uav1.position = Vec3(10.0, 0.0, 10.0);
    Vec3 prev1(0.0, 0.0, 10.0);

    // UAV 2 flying directly crossing at Z = 15m (vertical clearance = 5m > 2*radius)
    UAVState uav2;
    uav2.id = 2;
    uav2.position = Vec3(5.0, 5.0, 15.0);
    Vec3 prev2(5.0, -5.0, 15.0);

    std::vector<UAVState> uavs = { uav1, uav2 };
    std::vector<Vec3> prevPositions = { prev1, prev2 };

    auto events = colSys.evaluateAndResolve(uavs, prevPositions, map, 1.0);

    TEST_ASSERT(events.empty(), "Vertically separated UAVs crossing paths must not collide");
}

void test_deterministic_conflict_resolution() {
    CityMap map;
    map.rebuildBroadphase(50.0);
    CollisionSystem colSys(0.5);

    UAVState uavA; uavA.id = 5; uavA.position = Vec3(5.0, 0.0, 10.0); Vec3 prevA(0.0, 0.0, 10.0);
    UAVState uavB; uavB.id = 9; uavB.position = Vec3(5.0, 0.0, 10.0); Vec3 prevB(10.0, 0.0, 10.0);

    // Order 1: [A, B]
    std::vector<UAVState> uavs1 = { uavA, uavB };
    std::vector<Vec3> prevs1 = { prevA, prevB };
    auto events1 = colSys.evaluateAndResolve(uavs1, prevs1, map, 1.0);

    // Order 2: [B, A]
    std::vector<UAVState> uavs2 = { uavB, uavA };
    std::vector<Vec3> prevs2 = { prevB, prevA };
    auto events2 = colSys.evaluateAndResolve(uavs2, prevs2, map, 1.0);

    TEST_ASSERT(events1.size() == events2.size(), "Collision event count must match regardless of array order");
    TEST_ASSERT(events1[0].uavIdA == events2[0].uavIdA && events1[0].uavIdB == events2[0].uavIdB,
        "Canonical pair IDs must match regardless of vector input ordering");
}

void test_world_boundary_collision() {
    CityMap map;
    map.worldMin = Vec3(-100.0, -100.0, 0.0);
    map.worldMax = Vec3(100.0, 100.0, 50.0);
    map.rebuildBroadphase(50.0);

    CollisionSystem colSys(0.5);

    // UAV stepping out of bounds across East boundary (X = 101.0 > 100.0 - 0.5)
    UAVState uav;
    uav.id = 3;
    uav.position = Vec3(101.0, 0.0, 20.0);
    Vec3 prevPos(98.0, 0.0, 20.0);

    std::vector<UAVState> uavs = { uav };
    std::vector<Vec3> prevPositions = { prevPos };

    auto events = colSys.evaluateAndResolve(uavs, prevPositions, map, 1.0);

    TEST_ASSERT(events.size() == 1, "Exactly 1 boundary collision event must be registered");
    TEST_ASSERT(events[0].type == CollisionType::WORLD_BOUNDARY, "Event type must be WORLD_BOUNDARY");
    TEST_ASSERT(events[0].uavIdA == 3, "UAV ID must match");
    TEST_ASSERT(uavs[0].position == prevPos, "UAV position must roll back to previous safe position");
    TEST_ASSERT(uavs[0].mode == UAVMode::EMERGENCY, "UAV mode must switch to EMERGENCY upon boundary collision");
}

int main() {
    std::cout << "========================================\n";
    std::cout << "     AeroFleet Collision Tests          \n";
    std::cout << "========================================\n";

    RUN_TEST(test_swept_building_collision);
    RUN_TEST(test_safe_building_clearance);
    RUN_TEST(test_uav_uav_head_on_collision);
    RUN_TEST(test_uav_uav_safe_vertical_separation);
    RUN_TEST(test_deterministic_conflict_resolution);
    RUN_TEST(test_world_boundary_collision);

    std::cout << "========================================\n";
    std::cout << "Summary: " << g_testsPassed << " Passed, " << g_testsFailed << " Failed\n";
    std::cout << "========================================\n";

    return (g_testsFailed == 0) ? 0 : 1;
}

