#include <iostream>
#include <cassert>
#include <vector>
#include <cmath>
#include "requests/DeliveryRequest.hpp"
#include "requests/RequestGenerator.hpp"
#include "world/WorldGenerator.hpp"

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

void test_poisson_goodness_of_fit() {
    RequestSettings settings;
    settings.arrivalRatePerMin = 12.0; // lambda = 0.2 req/sec, expected mean = 5.0 seconds
    CityMap map;
    RequestGenerator generator(settings, map, 10.0, 12345);

    const size_t sampleCount = 10000;
    double sum = 0.0;
    for (size_t i = 0; i < sampleCount; ++i) {
        sum += generator.sampleInterarrivalTime();
    }

    double sampleMean = sum / static_cast<double>(sampleCount);
    double expectedMean = 60.0 / settings.arrivalRatePerMin; // 5.0 seconds
    double relativeError = std::abs(sampleMean - expectedMean) / expectedMean;

    TEST_ASSERT(relativeError < 0.03,
        "Sample mean inter-arrival time must match theoretical 1/lambda within 3% tolerance");
}

void test_request_lifecycle_transitions() {
    DeliveryRequest req(
        1, 10, Vec3(0, 0, 15), 20, Vec3(100, 100, 15), 10.0, 70.0, 1.5
    );

    TEST_ASSERT(req.status == RequestStatus::PENDING, "New request must be PENDING");

    // Valid forward lifecycle
    req.transitionTo(RequestStatus::ASSIGNED, 12.0);
    req.assignedUavId = 5;
    TEST_ASSERT(req.status == RequestStatus::ASSIGNED, "Request must transition to ASSIGNED");

    req.transitionTo(RequestStatus::PICKING_UP, 25.0);
    TEST_ASSERT(req.status == RequestStatus::PICKING_UP, "Request must transition to PICKING_UP");
    TEST_ASSERT(req.pickupTime == 25.0, "Pickup time must record 25.0");

    req.transitionTo(RequestStatus::IN_TRANSIT, 28.0);
    TEST_ASSERT(req.status == RequestStatus::IN_TRANSIT, "Request must transition to IN_TRANSIT");

    req.transitionTo(RequestStatus::DELIVERED, 50.0);
    TEST_ASSERT(req.status == RequestStatus::DELIVERED, "Request must transition to DELIVERED");
    TEST_ASSERT(req.deliveryTime == 50.0, "Delivery time must record 50.0");

    // Illegal transition from terminal state DELIVERED
    bool caughtIllegal = false;
    try {
        req.transitionTo(RequestStatus::PENDING, 55.0);
    } catch (const RequestTransitionException&) {
        caughtIllegal = true;
    }
    TEST_ASSERT(caughtIllegal, "Transition from terminal DELIVERED state must throw RequestTransitionException");
}

void test_illegal_lifecycle_skips() {
    DeliveryRequest req(
        2, 10, Vec3(0, 0, 15), 20, Vec3(100, 100, 15), 0.0, 100.0, 1.0
    );

    bool caught = false;
    try {
        // Attempting to jump directly from PENDING to DELIVERED without assignment/pickup
        req.transitionTo(RequestStatus::DELIVERED);
    } catch (const RequestTransitionException&) {
        caught = true;
    }
    TEST_ASSERT(caught, "Jumping directly from PENDING to DELIVERED must throw");
}

void test_request_requeue() {
    DeliveryRequest req(
        3, 10, Vec3(0, 0, 15), 20, Vec3(100, 100, 15), 0.0, 100.0, 1.0
    );

    req.transitionTo(RequestStatus::ASSIGNED);
    req.assignedUavId = 2;

    // UAV cancels or fails before pickup -> re-queue to PENDING
    req.transitionTo(RequestStatus::PENDING);
    TEST_ASSERT(req.status == RequestStatus::PENDING, "Re-queued request must be PENDING");
    TEST_ASSERT(req.assignedUavId == -1, "Assigned UAV ID must reset to -1 on re-queue");
}

void test_seed_determinism() {
    WorldSettings ws;
    CityMap map = WorldGenerator::generate(ws, 0xCAFE);

    RequestSettings rs;
    RequestGenerator genA(rs, map, 10.0, 0xBEEF);
    RequestGenerator genB(rs, map, 10.0, 0xBEEF);

    for (int i = 0; i < 50; ++i) {
        DeliveryRequest rA = genA.generateSingle(static_cast<double>(i * 5));
        DeliveryRequest rB = genB.generateSingle(static_cast<double>(i * 5));

        TEST_ASSERT(rA.id == rB.id, "Request IDs must match for identical seed");
        TEST_ASSERT(rA.storeId == rB.storeId, "Store IDs must match for identical seed");
        TEST_ASSERT(rA.customerId == rB.customerId, "Customer IDs must match for identical seed");
        TEST_ASSERT(rA.pickupLocation == rB.pickupLocation, "Pickup locations must match");
        TEST_ASSERT(rA.dropoffLocation == rB.dropoffLocation, "Dropoff locations must match");
        TEST_ASSERT(std::abs(rA.payloadMass - rB.payloadMass) < 1e-9, "Payload mass must match");
        TEST_ASSERT(std::abs(rA.deadline - rB.deadline) < 1e-9, "Deadlines must match");
    }
}

void test_deadline_and_expiration() {
    DeliveryRequest req(
        4, 1, Vec3(0, 0, 15), 2, Vec3(100, 0, 15), 10.0, 50.0, 2.0
    );

    TEST_ASSERT(!req.isExpired(10.0), "Request created at 10.0 is not expired at 10.0");
    TEST_ASSERT(!req.isExpired(49.9), "Request is not expired before deadline");
    TEST_ASSERT(req.isExpired(50.1), "Request is expired after deadline");

    // Terminal delivered request is not expired even after deadline has passed
    req.transitionTo(RequestStatus::ASSIGNED);
    req.transitionTo(RequestStatus::PICKING_UP);
    req.transitionTo(RequestStatus::IN_TRANSIT);
    req.transitionTo(RequestStatus::DELIVERED, 40.0);
    TEST_ASSERT(!req.isExpired(100.0), "Delivered request must never report expired");
}

void test_payload_mass_bounds() {
    RequestSettings rs;
    rs.minPayloadKg = 0.5;
    rs.maxPayloadKg = 2.5;
    CityMap map;
    RequestGenerator gen(rs, map, 10.0, 9999);

    for (int i = 0; i < 500; ++i) {
        DeliveryRequest req = gen.generateSingle(static_cast<double>(i));
        TEST_ASSERT(req.payloadMass >= rs.minPayloadKg && req.payloadMass <= rs.maxPayloadKg,
            "Payload mass must stay strictly within configured min/max bounds");
    }
}

void test_generator_step_emissions() {
    RequestSettings rs;
    rs.arrivalRatePerMin = 60.0; // 1 request per second
    CityMap map;
    RequestGenerator gen(rs, map, 10.0, 42);

    // Step across 10 seconds in 0.1s increments
    uint64_t totalEmitted = 0;
    double t = 0.0;
    for (int i = 0; i < 100; ++i) {
        t += 0.1;
        auto reqs = gen.step(t, 0.1);
        totalEmitted += reqs.size();
    }

    // Over 10 seconds at 1 req/sec, expect roughly 5-15 requests
    TEST_ASSERT(totalEmitted >= 4 && totalEmitted <= 20,
        "Poisson step emissions over 10 seconds must produce realistic request volume");
}

int main() {
    std::cout << "========================================\n";
    std::cout << "      AeroFleet Request Tests           \n";
    std::cout << "========================================\n";

    RUN_TEST(test_poisson_goodness_of_fit);
    RUN_TEST(test_request_lifecycle_transitions);
    RUN_TEST(test_illegal_lifecycle_skips);
    RUN_TEST(test_request_requeue);
    RUN_TEST(test_seed_determinism);
    RUN_TEST(test_deadline_and_expiration);
    RUN_TEST(test_payload_mass_bounds);
    RUN_TEST(test_generator_step_emissions);

    std::cout << "========================================\n";
    std::cout << "Summary: " << g_testsPassed << " Passed, " << g_testsFailed << " Failed\n";
    std::cout << "========================================\n";

    return (g_testsFailed == 0) ? 0 : 1;
}
