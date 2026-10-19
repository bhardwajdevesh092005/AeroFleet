#include <iostream>
#include <vector>
#include <cmath>
#include <chrono>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <filesystem>

#include "core/Config.hpp"
#include "core/Types.hpp"
#include "math/Vec3.hpp"
#include "world/WorldGenerator.hpp"
#include "fleet/FleetManager.hpp"
#include "requests/RequestGenerator.hpp"
#include "control/WaypointController.hpp"
#include "coordinator/HeuristicCoordinator.hpp"
#include "physics/CollisionSystem.hpp"
#include "physics/Kinematics.hpp"
#include "telemetry/Metrics.hpp"
#include "telemetry/Logger.hpp"

using namespace aerofleet;

namespace {

int passed = 0;
int failed = 0;

void expect(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void expectNear(double actual, double expected, double tolerance, const std::string& message) {
    if (std::abs(actual - expected) > tolerance) {
        throw std::runtime_error(message + " (Expected " + std::to_string(expected) +
                                 ", Got " + std::to_string(actual) + ")");
    }
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

void test_metrics_mathematical_accuracy() {
    MetricsTracker metrics;

    // Feed 10 known delivery latencies:
    // 10, 20, 30, 40, 50, 60, 70, 80, 90, 100 seconds
    // 8 on-time, 2 late
    std::vector<double> latencies = {10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0, 100.0};
    for (size_t i = 0; i < latencies.size(); ++i) {
        bool onTime = (i < 8);
        metrics.addDeliveryLatency(latencies[i], onTime);
    }

    // Add 2 expired requests
    DeliveryRequest dummyReq;
    metrics.recordRequestExpired(dummyReq, 120.0);
    metrics.recordRequestExpired(dummyReq, 130.0);

    // Set simulated time to 7200 seconds (2.0 hours)
    metrics.setSimulatedTime(7200.0);

    // 1. Throughput: 10 deliveries / 2 hours = 5.0 deliveries/hr
    expectNear(metrics.getThroughputPerHour(), 5.0, 1e-5, "Throughput per hour mismatch");

    // 2. Mean latency: sum(10..100) / 10 = 550 / 10 = 55.0s
    expectNear(metrics.getMeanLatency(), 55.0, 1e-5, "Mean latency mismatch");

    // 3. Median latency: (50 + 60) / 2 = 55.0s
    expectNear(metrics.getMedianLatency(), 55.0, 1e-5, "Median latency mismatch");

    // 4. P95 latency: rank = 0.95 * 9 = 8.55 -> lerp(latencies[8], latencies[9], 0.55) = 90 + 0.55*10 = 95.5s
    expectNear(metrics.getP95Latency(), 95.5, 1e-4, "P95 latency mismatch");

    // 5. Deadline success rate: 8 on-time / (10 delivered + 2 expired) = 8/12 = 66.6667%
    expectNear(metrics.getDeadlineSuccessRate(), (8.0 / 12.0) * 100.0, 1e-4, "Deadline success rate mismatch");

    // 6. Energy efficiency: Add 360,000 Joules = 100 Wh
    // 100 Wh / 10 deliveries = 10 Wh/delivery
    metrics.addEnergyConsumedJoules(360000.0);
    expectNear(metrics.getEnergyPerDeliveryWh(), 10.0, 1e-5, "Energy per delivery Wh mismatch");

    // 7. Fleet utilization: 500 active seconds out of 2000 available seconds = 25.0%
    metrics.setFleetUtilization(500.0, 2000.0);
    expectNear(metrics.getUavUtilization(), 25.0, 1e-5, "Fleet UAV utilization mismatch");
}

void test_metrics_json_csv_serialization() {
    MetricsTracker metrics;
    metrics.addDeliveryLatency(45.0, true);
    metrics.addDeliveryLatency(65.0, false);
    metrics.setSimulatedTime(100.0);
    metrics.addEnergyConsumedJoules(72000.0); // 20 Wh
    metrics.finalize(0.05);

    std::string json = metrics.toJson();
    expect(json.find("\"total_requests_delivered\": 2") != std::string::npos, "JSON missing total delivered");
    expect(json.find("\"simulated_time_s\": 100.0000") != std::string::npos, "JSON missing simulated time");
    expect(json.find("\"mean_latency_s\": 55.0000") != std::string::npos, "JSON missing mean latency");

    std::string csv = metrics.toCsv();
    expect(csv.find("simulated_time_s,real_time_s") != std::string::npos, "CSV missing header");
    expect(csv.find("100.0000,0.0500") != std::string::npos, "CSV missing data line");

    // Test file exports
    std::filesystem::create_directories("build/test_logs");
    std::string jsonPath = "build/test_logs/metrics_test.json";
    std::string csvPath = "build/test_logs/metrics_test.csv";

    expect(metrics.saveToJsonFile(jsonPath), "Failed to save JSON file");
    expect(metrics.saveToCsvFile(csvPath), "Failed to save CSV file");

    std::ifstream jf(jsonPath);
    expect(jf.is_open() && jf.good(), "Saved JSON file cannot be read");
    std::ifstream cf(csvPath);
    expect(cf.is_open() && cf.good(), "Saved CSV file cannot be read");
}

void test_event_logger_buffering_and_wrap() {
    EventLogger logger(5); // Capacity 5

    logger.log(0.1, SimulationEventType::REQUEST_CREATED, 101, -1, 0.0, "req 1");
    logger.log(0.2, SimulationEventType::REQUEST_ASSIGNED, 101, 0, 0.0, "uav 0");
    logger.log(0.3, SimulationEventType::REQUEST_PICKED_UP, 101, 0, 1.5, "picked up");
    expect(logger.size() == 3, "logger size should be 3");

    logger.log(0.4, SimulationEventType::REQUEST_DELIVERED, 101, 0, 0.0, "delivered");
    logger.log(0.5, SimulationEventType::BATTERY_LOW, 0, -1, 19.5, "battery 19.5%");
    expect(logger.size() == 5, "logger size should be 5");

    // Overflow event: should overwrite index 0 (oldest)
    logger.log(0.6, SimulationEventType::CHARGING_STARTED, 0, 1, 0.0, "charging dock 1");
    expect(logger.size() == 5, "ring buffer size should stay at capacity 5");
    expect(logger.totalLogged() == 6, "total logged should be 6");

    auto events = logger.getEvents();
    expect(events.size() == 5, "retrieved events size must be 5");
    // Oldest remaining should be event with timestamp 0.2
    expectNear(events[0].timestamp, 0.2, 1e-5, "first event in wrapped ring mismatch");
    // Newest should be event with timestamp 0.6
    expectNear(events[4].timestamp, 0.6, 1e-5, "last event in wrapped ring mismatch");

    // Test JSONL flush
    std::filesystem::create_directories("build/test_logs");
    std::string jsonlPath = "build/test_logs/events_test.jsonl";
    expect(logger.flushToFile(jsonlPath), "Failed to flush event log to file");

    std::ifstream ef(jsonlPath);
    std::string line;
    int lineCount = 0;
    while (std::getline(ef, line)) {
        if (!line.empty()) {
            lineCount++;
            expect(line.find("\"event\":") != std::string::npos, "JSONL line malformed");
        }
    }
    expect(lineCount == 5, "JSONL line count must equal 5 (got " + std::to_string(lineCount) + ")");
}

struct SimulationResult {
    std::vector<Vec3> finalPositions;
    std::vector<double> finalBatteries;
    size_t createdRequests{0};
    size_t deliveredRequests{0};
    double simulatedTime{0.0};
};

SimulationResult runDeterministicSim(uint64_t seed, size_t steps) {
    WorldSettings ws;
    FleetSettings fs;
    fs.numUavs = 4;
    EnergySettings es;
    RequestSettings rs;

    CityMap map = WorldGenerator::generate(ws, seed);
    FleetManager fleet(fs, es, map);
    RequestGenerator reqGen(rs, map, fs.maxHorizontalSpeed, seed);

    WaypointControllerConfig navConfig;
    navConfig.maxHorizontalSpeed = fs.maxHorizontalSpeed;
    navConfig.maxVerticalSpeed = fs.maxVerticalSpeed;
    WaypointController navigator(navConfig);

    HeuristicCoordinator coordinator;
    CollisionSystem collisions(fs.uavRadius);
    KinematicsIntegrator integrator(KinematicsConfig::fromConfig(fs, ws));

    std::vector<DeliveryRequest> pendingRequests;
    std::vector<DeliveryRequest> deliveredRequests;
    std::vector<DeliveryRequest> requeuedRequests;
    std::vector<Vec3> prevPositions(fleet.size());

    const double dt = 0.02;
    double simTime = 0.0;
    double lastDispatchTime = -10.0;
    size_t createdCount = 0;

    for (size_t step = 0; step < steps; ++step) {
        simTime = step * dt;

        for (size_t i = 0; i < fleet.size(); ++i) {
            prevPositions[i] = fleet.getUAV(i).position;
        }

        auto newRequests = reqGen.step(simTime, dt);
        if (!newRequests.empty()) {
            createdCount += newRequests.size();
            pendingRequests.insert(pendingRequests.end(), newRequests.begin(), newRequests.end());
        }

        if (simTime - lastDispatchTime >= 1.0) {
            coordinator.dispatch(fleet, pendingRequests, map, simTime);
            lastDispatchTime = simTime;
        }

        fleet.step(dt, simTime, map, deliveredRequests, requeuedRequests);
        if (!requeuedRequests.empty()) {
            pendingRequests.insert(pendingRequests.end(), requeuedRequests.begin(), requeuedRequests.end());
            requeuedRequests.clear();
        }

        for (auto& uav : fleet.getAllUAVs()) {
            VelocityCommand action;
            if (uav.mode == UAVMode::FLYING_TO_PICKUP ||
                uav.mode == UAVMode::FLYING_TO_DESTINATION ||
                uav.mode == UAVMode::RETURNING_TO_CHARGE) {
                action = navigator.computeAction(uav, map, dt);
            }
            integrator.stepVelocityCommand(uav, action, dt);
            fleet.getBatteryModel().stepDischarge(uav, dt);
        }

        collisions.evaluateAndResolve(fleet.getAllUAVs(), prevPositions, map, simTime);
    }

    SimulationResult res;
    res.simulatedTime = simTime;
    res.createdRequests = createdCount;
    res.deliveredRequests = deliveredRequests.size();
    for (size_t i = 0; i < fleet.size(); ++i) {
        res.finalPositions.push_back(fleet.getUAV(i).position);
        res.finalBatteries.push_back(fleet.getUAV(i).batteryPercent);
    }
    return res;
}

void test_seed_reproducibility() {
    // Run two identical 2,500-step (50 simulated seconds) simulations with seed 42
    const uint64_t seed = 42;
    const size_t steps = 2500;

    SimulationResult run1 = runDeterministicSim(seed, steps);
    SimulationResult run2 = runDeterministicSim(seed, steps);

    expect(run1.createdRequests == run2.createdRequests, "Created requests mismatch between identical seed runs");
    expect(run1.deliveredRequests == run2.deliveredRequests, "Delivered requests mismatch between identical seed runs");
    expect(run1.finalPositions.size() == run2.finalPositions.size(), "Fleet size mismatch");

    for (size_t i = 0; i < run1.finalPositions.size(); ++i) {
        expectNear(run1.finalPositions[i].x, run2.finalPositions[i].x, 1e-9,
                   "UAV " + std::to_string(i) + " X position diverged on identical seed");
        expectNear(run1.finalPositions[i].y, run2.finalPositions[i].y, 1e-9,
                   "UAV " + std::to_string(i) + " Y position diverged on identical seed");
        expectNear(run1.finalPositions[i].z, run2.finalPositions[i].z, 1e-9,
                   "UAV " + std::to_string(i) + " Z position diverged on identical seed");
        expectNear(run1.finalBatteries[i], run2.finalBatteries[i], 1e-9,
                   "UAV " + std::to_string(i) + " battery diverged on identical seed");
    }
}

void test_seed_divergence() {
    // Run two simulations with different seeds (42 vs 99999)
    const size_t steps = 1500;
    SimulationResult runA = runDeterministicSim(42, steps);
    SimulationResult runB = runDeterministicSim(99999, steps);

    bool anyPositionDifferent = false;
    for (size_t i = 0; i < runA.finalPositions.size(); ++i) {
        if (!runA.finalPositions[i].isApprox(runB.finalPositions[i], 0.1)) {
            anyPositionDifferent = true;
            break;
        }
    }
    expect(anyPositionDifferent, "Simulations with different seeds produced identical UAV trajectories");
}

void test_high_speed_benchmark_performance() {
    // Step 12 requirement: Run 100,000 steps headless; verify execution finishes in < 2.0s (> 1000x real-time)
    const size_t benchmarkSteps = 100000;
    auto tStart = std::chrono::steady_clock::now();

    SimulationResult res = runDeterministicSim(12345, benchmarkSteps);

    auto tEnd = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = tEnd - tStart;

    const double simulatedTime = benchmarkSteps * 0.02; // 2,000 seconds
    const double speedMultiplier = simulatedTime / elapsed.count();

    std::cout << "[Benchmark: " << benchmarkSteps << " steps in "
              << elapsed.count() << "s, " << speedMultiplier << "x real-time] ";

    expect(elapsed.count() < 2.0, "Benchmark took " + std::to_string(elapsed.count()) +
           "s; expected < 2.0s (> 1000x real-time)");
    expect(speedMultiplier > 1000.0, "Speed multiplier below 1000x requirement");
}

} // namespace

int main() {
    std::cout << "========================================\n"
              << "   AeroFleet Reproducibility & Telemetry Tests\n"
              << "========================================\n";

    run("test_metrics_mathematical_accuracy", test_metrics_mathematical_accuracy);
    run("test_metrics_json_csv_serialization", test_metrics_json_csv_serialization);
    run("test_event_logger_buffering_and_wrap", test_event_logger_buffering_and_wrap);
    run("test_seed_reproducibility", test_seed_reproducibility);
    run("test_seed_divergence", test_seed_divergence);
    run("test_high_speed_benchmark_performance", test_high_speed_benchmark_performance);

    std::cout << "========================================\nSummary: " << passed
              << " Passed, " << failed << " Failed\n========================================\n";
    return failed == 0 ? 0 : 1;
}

