#pragma once

#include <vector>
#include <random>
#include <memory>
#include <cstdint>
#include "core/Config.hpp"
#include "world/CityMap.hpp"
#include "requests/DeliveryRequest.hpp"

namespace aerofleet {

/**
 * @brief Stochastic Poisson demand generator synthesizing timestamped customer delivery requests.
 */
class RequestGenerator {
public:
    RequestGenerator(
        const RequestSettings& settings,
        const CityMap& map,
        double nominalSpeed = 10.0,
        uint64_t seed = 42
    );

    /**
     * @brief Steps simulation clock and emits new delivery requests according to Poisson arrivals.
     * @param currentTime Current simulation clock in seconds
     * @param dt Elapsed physics time step in seconds
     * @return List of delivery requests generated during this step
     */
    std::vector<DeliveryRequest> step(double currentTime, double dt);

    /**
     * @brief Immediately generates a single delivery request on demand.
     */
    DeliveryRequest generateSingle(double currentTime);

    /**
     * @brief Draws a single inter-arrival time from the exponential distribution:
     * Delta_t = -ln(U) / lambda, where lambda = arrivalRatePerSec.
     */
    double sampleInterarrivalTime();

    /**
     * @brief Reseeds PRNG and resets sequence counters for deterministic reproduction.
     */
    void reset(uint64_t seed);

    [[nodiscard]] uint64_t getTotalGenerated() const noexcept { return m_totalGenerated; }
    [[nodiscard]] RequestId getNextRequestId() const noexcept { return m_nextId; }
    [[nodiscard]] double getArrivalRatePerSec() const noexcept { return m_arrivalRatePerSec; }

private:
    RequestSettings m_settings;
    CityMap m_map;
    double m_nominalSpeed{10.0};
    uint64_t m_initialSeed{42};
    std::mt19937_64 m_rng;
    std::uniform_real_distribution<double> m_uniformDist{0.0, 1.0};

    double m_arrivalRatePerSec{0.2};   ///< Requests per second
    double m_nextArrivalTime{0.0};     ///< Next scheduled event timestamp
    RequestId m_nextId{1};             ///< Monotonically increasing request ID
    uint64_t m_totalGenerated{0};      ///< Total requests synthesized
    bool m_initialized{false};
};

} // namespace aerofleet

