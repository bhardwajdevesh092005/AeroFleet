#include <iostream>
#include <cassert>
#include <cmath>
#include <fstream>
#include "core/Config.hpp"
#include "core/Types.hpp"
#include "math/Vec3.hpp"

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

static std::string resolveConfigPath(const std::string& relativePath) {
    std::ifstream f1(relativePath);
    if (f1.good()) return relativePath;
    std::string fallback = "../" + relativePath;
    std::ifstream f2(fallback);
    if (f2.good()) return fallback;
    return relativePath;
}

void test_load_valid_config() {
    try {
        std::string path = resolveConfigPath("config/simulation.json");
        SimulationConfig config = SimulationConfig::loadFromFile(path);
        TEST_ASSERT(config.simulation.name == "AeroFleet Urban Simulation", "Simulation name mismatch");
        TEST_ASSERT(config.simulation.randomSeed == 42, "Seed mismatch");
        TEST_ASSERT(std::abs(config.simulation.dtPhysics - 0.02) < 1e-6, "dtPhysics mismatch");
        TEST_ASSERT(std::abs(config.simulation.dtPolicy - 0.10) < 1e-6, "dtPolicy mismatch");
        TEST_ASSERT(std::abs(config.simulation.dtCoordinator - 1.00) < 1e-6, "dtCoordinator mismatch");
        TEST_ASSERT(config.fleet.numUavs == 10, "Fleet size mismatch");
        TEST_ASSERT(config.world.xMin == -500.0 && config.world.xMax == 500.0, "World X bounds mismatch");
        TEST_ASSERT(config.world.zMin == 0.0 && config.world.zMax == 150.0, "World Z bounds mismatch");
        TEST_ASSERT(config.fleet.reserveBatteryPercent > config.fleet.criticalBatteryPercent, "Battery bounds check");
    } catch (const std::exception& e) {
        TEST_ASSERT(false, std::string("Unexpected exception: ") + e.what());
    }
}

void test_missing_file_throws() {
    bool threw = false;
    try {
        SimulationConfig::loadFromFile("config/non_existent_file_404.json");
    } catch (const JsonParseException&) {
        threw = true;
    } catch (const std::exception&) {
        threw = true;
    }
    TEST_ASSERT(threw, "Loading non-existent file must throw an exception");
}

void test_malformed_json_throws() {
    bool threw = false;
    try {
        std::string badJson = "{\"simulation\": { \"dt_physics\": 0.02, "; // missing closing braces
        JsonValue::parse(badJson);
    } catch (const JsonParseException&) {
        threw = true;
    }
    TEST_ASSERT(threw, "Malformed JSON syntax must throw JsonParseException");
}

void test_invalid_dt_physics() {
    SimulationConfig config = SimulationConfig::createDefault();
    config.simulation.dtPhysics = -0.01; // Negative timestep

    bool threw = false;
    try {
        SimulationConfig::validate(config);
    } catch (const ConfigValidationException&) {
        threw = true;
    }
    TEST_ASSERT(threw, "Negative dtPhysics must throw ConfigValidationException");

    config.simulation.dtPhysics = 0.5; // Excessively large timestep
    threw = false;
    try {
        SimulationConfig::validate(config);
    } catch (const ConfigValidationException&) {
        threw = true;
    }
    TEST_ASSERT(threw, "Excessively large dtPhysics must throw ConfigValidationException");
}

void test_dt_policy_smaller_than_physics() {
    SimulationConfig config = SimulationConfig::createDefault();
    config.simulation.dtPhysics = 0.05;
    config.simulation.dtPolicy = 0.02; // Policy faster than physics

    bool threw = false;
    try {
        SimulationConfig::validate(config);
    } catch (const ConfigValidationException&) {
        threw = true;
    }
    TEST_ASSERT(threw, "dtPolicy < dtPhysics must throw ConfigValidationException");
}

void test_invalid_world_bounds() {
    SimulationConfig config = SimulationConfig::createDefault();
    config.world.xMin = 500.0;
    config.world.xMax = -500.0; // Inverted bounds

    bool threw = false;
    try {
        SimulationConfig::validate(config);
    } catch (const ConfigValidationException&) {
        threw = true;
    }
    TEST_ASSERT(threw, "Inverted X bounds must throw ConfigValidationException");

    config = SimulationConfig::createDefault();
    config.world.zMin = -10.0; // Sub-ground level
    threw = false;
    try {
        SimulationConfig::validate(config);
    } catch (const ConfigValidationException&) {
        threw = true;
    }
    TEST_ASSERT(threw, "Sub-ground zMin must throw ConfigValidationException");
}

void test_invalid_battery_thresholds() {
    SimulationConfig config = SimulationConfig::createDefault();
    config.fleet.reserveBatteryPercent = 10.0;
    config.fleet.criticalBatteryPercent = 15.0; // Reserve <= Critical

    bool threw = false;
    try {
        SimulationConfig::validate(config);
    } catch (const ConfigValidationException&) {
        threw = true;
    }
    TEST_ASSERT(threw, "Reserve battery <= Critical battery must throw ConfigValidationException");
}

void test_vec3_math_operations() {
    Vec3 a(1.0, 2.0, 3.0);
    Vec3 b(4.0, 5.0, 6.0);

    Vec3 sum = a + b;
    TEST_ASSERT(sum == Vec3(5.0, 7.0, 9.0), "Vec3 addition");

    Vec3 diff = b - a;
    TEST_ASSERT(diff == Vec3(3.0, 3.0, 3.0), "Vec3 subtraction");

    double dotProd = a.dot(b);
    // 1*4 + 2*5 + 3*6 = 4 + 10 + 18 = 32
    TEST_ASSERT(std::abs(dotProd - 32.0) < 1e-9, "Vec3 dot product");

    Vec3 crossProd = a.cross(b);
    // (2*6 - 3*5, 3*4 - 1*6, 1*5 - 2*4) = (-3, 6, -3)
    TEST_ASSERT(crossProd == Vec3(-3.0, 6.0, -3.0), "Vec3 cross product");

    Vec3 v(3.0, 4.0, 0.0);
    TEST_ASSERT(std::abs(v.length() - 5.0) < 1e-9, "Vec3 length");

    Vec3 unit = v.normalized();
    TEST_ASSERT(std::abs(unit.length() - 1.0) < 1e-6, "Vec3 unit length");
    TEST_ASSERT(std::abs(unit.x - 0.6) < 1e-6 && std::abs(unit.y - 0.8) < 1e-6, "Vec3 normalized components");

    // Safe zero-vector normalization test
    Vec3 zeroVec(0.0, 0.0, 0.0);
    Vec3 normZero = zeroVec.normalized();
    TEST_ASSERT(!std::isnan(normZero.x) && !std::isnan(normZero.y) && !std::isnan(normZero.z), "Safe zero normalize no NaN");
    TEST_ASSERT(normZero == Vec3(0.0, 0.0, 0.0), "Zero vector normalize returns zero vector");

    // Lerp test
    Vec3 start(0.0, 0.0, 0.0);
    Vec3 end(10.0, 20.0, 30.0);
    Vec3 mid = start.lerp(end, 0.5);
    TEST_ASSERT(mid == Vec3(5.0, 10.0, 15.0), "Vec3 lerp");
}

void test_type_string_conversions() {
    TEST_ASSERT(uavModeToString(UAVMode::IDLE) == "IDLE", "UAVMode::IDLE string conversion");
    TEST_ASSERT(uavModeToString(UAVMode::FLYING_TO_PICKUP) == "FLYING_TO_PICKUP", "UAVMode::FLYING_TO_PICKUP string conversion");
    TEST_ASSERT(uavModeToString(UAVMode::EMERGENCY) == "EMERGENCY", "UAVMode::EMERGENCY string conversion");

    TEST_ASSERT(requestStatusToString(RequestStatus::PENDING) == "PENDING", "RequestStatus::PENDING string conversion");
    TEST_ASSERT(requestStatusToString(RequestStatus::IN_TRANSIT) == "IN_TRANSIT", "RequestStatus::IN_TRANSIT string conversion");
    TEST_ASSERT(requestStatusToString(RequestStatus::DELIVERED) == "DELIVERED", "RequestStatus::DELIVERED string conversion");
}

int main() {
    std::cout << "========================================\n";
    std::cout << "     AeroFleet Configuration Tests      \n";
    std::cout << "========================================\n";

    RUN_TEST(test_load_valid_config);
    RUN_TEST(test_missing_file_throws);
    RUN_TEST(test_malformed_json_throws);
    RUN_TEST(test_invalid_dt_physics);
    RUN_TEST(test_dt_policy_smaller_than_physics);
    RUN_TEST(test_invalid_world_bounds);
    RUN_TEST(test_invalid_battery_thresholds);
    RUN_TEST(test_vec3_math_operations);
    RUN_TEST(test_type_string_conversions);

    std::cout << "========================================\n";
    std::cout << "Summary: " << g_testsPassed << " Passed, " << g_testsFailed << " Failed\n";
    std::cout << "========================================\n";

    return (g_testsFailed == 0) ? 0 : 1;
}

