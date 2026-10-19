#include "requests/RequestGenerator.hpp"
#include <algorithm>
#include <cmath>

namespace aerofleet {

RequestGenerator::RequestGenerator(
    const RequestSettings& settings,
    const CityMap& map,
    double nominalSpeed,
    uint64_t seed
) : m_settings(settings),
    m_map(map),
    m_nominalSpeed(nominalSpeed > 0.1 ? nominalSpeed : 10.0),
    m_initialSeed(seed),
    m_rng(seed) {
    m_arrivalRatePerSec = std::max(1e-4, m_settings.arrivalRatePerMin / 60.0);
    reset(seed);
}

void RequestGenerator::reset(uint64_t seed) {
    m_rng.seed(seed);
    m_nextId = 1;
    m_totalGenerated = 0;
    m_initialized = false;
    m_nextArrivalTime = 0.0;
}

double RequestGenerator::sampleInterarrivalTime() {
    // Inverse Transform Sampling for Exponential distribution:
    // F(t) = 1 - exp(-lambda * t) => Delta_t = -ln(U) / lambda
    double u = std::max(1e-9, m_uniformDist(m_rng));
    return -std::log(u) / m_arrivalRatePerSec;
}

DeliveryRequest RequestGenerator::generateSingle(double currentTime) {
    RequestId reqId = m_nextId++;
    m_totalGenerated++;

    int32_t storeId = 0;
    Vec3 pickup(0.0, 0.0, 15.0);
    if (!m_map.stores.empty()) {
        std::uniform_int_distribution<size_t> storeDist(0, m_map.stores.size() - 1);
        size_t sIdx = storeDist(m_rng);
        storeId = m_map.stores[sIdx].id;
        pickup = m_map.stores[sIdx].position;
    }

    int32_t customerId = 0;
    Vec3 dropoff(100.0, 100.0, 15.0);
    if (!m_map.customers.empty()) {
        std::uniform_int_distribution<size_t> custDist(0, m_map.customers.size() - 1);
        size_t cIdx = custDist(m_rng);
        customerId = m_map.customers[cIdx].id;
        dropoff = m_map.customers[cIdx].position;
    }

    // Sample payload mass in [minPayloadKg, maxPayloadKg]
    std::uniform_real_distribution<double> payloadDist(m_settings.minPayloadKg, m_settings.maxPayloadKg);
    double payloadMass = payloadDist(m_rng);

    // Compute dynamic delivery deadline
    double distance = pickup.distance(dropoff);
    double transitTime = distance / m_nominalSpeed;
    double slack = std::max(1.0, m_settings.slackFactor);
    double deadline = currentTime + (transitTime * slack) + m_settings.bufferTimeSeconds;

    return DeliveryRequest(
        reqId,
        storeId,
        pickup,
        customerId,
        dropoff,
        currentTime,
        deadline,
        payloadMass
    );
}

std::vector<DeliveryRequest> RequestGenerator::step(double currentTime, double dt) {
    std::vector<DeliveryRequest> generated;
    if (dt <= 0.0) return generated;

    if (!m_initialized) {
        m_nextArrivalTime = currentTime + sampleInterarrivalTime();
        m_initialized = true;
    }

    // Emit all arrivals that occurred up to currentTime
    while (currentTime >= m_nextArrivalTime) {
        generated.push_back(generateSingle(m_nextArrivalTime));
        m_nextArrivalTime += sampleInterarrivalTime();
    }

    return generated;
}

} // namespace aerofleet

