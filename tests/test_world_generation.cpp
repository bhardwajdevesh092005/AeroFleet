#include <iostream>
#include <cassert>
#include <fstream>
#include "world/WorldGenerator.hpp"
#include "core/Config.hpp"

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

static std::string resolveFilePath(const std::string& relativePath) {
    std::ifstream f1(relativePath);
    if (f1.good()) return relativePath;
    std::string fallback = "../" + relativePath;
    std::ifstream f2(fallback);
    if (f2.good()) return fallback;
    return relativePath;
}

void test_seed_determinism() {
    WorldSettings settings;
    uint64_t seed = 0xDEADBEEF;

    CityMap map1 = WorldGenerator::generate(settings, seed);
    CityMap map2 = WorldGenerator::generate(settings, seed);

    TEST_ASSERT(map1.buildings.size() == map2.buildings.size(), "Building count must be identical for same seed");
    TEST_ASSERT(map1.stores.size() == map2.stores.size(), "Store count must be identical for same seed");
    TEST_ASSERT(map1.chargingStations.size() == map2.chargingStations.size(), "Charging station count identical");

    for (size_t i = 0; i < map1.buildings.size(); ++i) {
        TEST_ASSERT(map1.buildings[i].bounds.minPoint == map2.buildings[i].bounds.minPoint, "Building minPoint identical");
        TEST_ASSERT(map1.buildings[i].bounds.maxPoint == map2.buildings[i].bounds.maxPoint, "Building maxPoint identical");
    }

    CityMap map3 = WorldGenerator::generate(settings, 12345);
    bool anyDifference = false;
    for (size_t i = 0; i < std::min(map1.buildings.size(), map3.buildings.size()); ++i) {
        if (map1.buildings[i].bounds.maxPoint != map3.buildings[i].bounds.maxPoint) {
            anyDifference = true;
            break;
        }
    }
    TEST_ASSERT(anyDifference, "Different seeds must produce different building geometries");
}

void test_building_non_overlap() {
    WorldSettings settings;
    CityMap map = WorldGenerator::generate(settings, 42);

    TEST_ASSERT(!map.buildings.empty(), "City must have buildings");

    for (size_t i = 0; i < map.buildings.size(); ++i) {
        for (size_t j = i + 1; j < map.buildings.size(); ++j) {
            const auto& b1 = map.buildings[i].bounds;
            const auto& b2 = map.buildings[j].bounds;
            TEST_ASSERT(!b1.intersects(b2), "Procedural buildings must not overlap each other");
        }
    }
}

void test_landmark_clearance() {
    WorldSettings settings;
    CityMap map = WorldGenerator::generate(settings, 42);

    for (const auto& store : map.stores) {
        TEST_ASSERT(map.isLocationClear(store.position, 1.0), "Store location must have clearance from buildings");
    }

    for (const auto& station : map.chargingStations) {
        TEST_ASSERT(map.isLocationClear(station.position, 1.0), "Charging dock must have clearance from buildings");
    }
}

void test_broadphase_consistency() {
    WorldSettings settings;
    CityMap map = WorldGenerator::generate(settings, 42);

    AABB queryBox(Vec3(-100.0, -100.0, 0.0), Vec3(100.0, 100.0, 50.0));

    // Broadphase query
    const auto broadCandidates = map.broadphase.queryAABB(queryBox);
    std::unordered_set<int32_t> broadSet(broadCandidates.begin(), broadCandidates.end());

    // Brute force check: every building that intersects queryBox MUST be returned by broadphase
    for (size_t i = 0; i < map.buildings.size(); ++i) {
        if (map.buildings[i].bounds.intersects(queryBox)) {
            TEST_ASSERT(broadSet.find(static_cast<int32_t>(i)) != broadSet.end(),
                "Broadphase query must never miss an intersecting building");
        }
    }
}

void test_load_city_json() {
    WorldSettings settings;
    std::string path = resolveFilePath("config/city_default.json");
    CityMap map = WorldGenerator::loadFromJsonFile(path, settings);

    TEST_ASSERT(map.name == "Downtown Metro Grid", "City name mismatch from JSON");
    TEST_ASSERT(map.stores.size() == 3, "Stores count mismatch from JSON");
    TEST_ASSERT(map.chargingStations.size() == 3, "Charging stations count mismatch from JSON");
    TEST_ASSERT(map.stores[0].name == "North Hub", "Store name mismatch");
    TEST_ASSERT(map.chargingStations[0].capacity == 4, "Charging capacity mismatch");
}

int main() {
    std::cout << "========================================\n";
    std::cout << "   AeroFleet World Generator Tests     \n";
    std::cout << "========================================\n";

    RUN_TEST(test_seed_determinism);
    RUN_TEST(test_building_non_overlap);
    RUN_TEST(test_landmark_clearance);
    RUN_TEST(test_broadphase_consistency);
    RUN_TEST(test_load_city_json);

    std::cout << "========================================\n";
    std::cout << "Summary: " << g_testsPassed << " Passed, " << g_testsFailed << " Failed\n";
    std::cout << "========================================\n";

    return (g_testsFailed == 0) ? 0 : 1;
}

