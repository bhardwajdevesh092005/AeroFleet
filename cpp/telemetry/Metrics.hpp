#pragma once

#include <vector>
#include <string>
#include <cstdint>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <iostream>
#include "core/Types.hpp"
#include "fleet/UAVState.hpp"
#include "requests/DeliveryRequest.hpp"
#include "physics/CollisionSystem.hpp"

namespace aerofleet {

struct MetricsSnapshot {
    // Counts
    size_t totalRequestsCreated{0};
    size_t totalRequestsDelivered{0};
    size_t totalRequestsExpired{0};
    size_t totalRequestsOnTime{0};
    size_t totalBuildingCollisions{0};
    size_t totalUavCollisions{0};

    // Time & Energy
    double simulatedTimeSeconds{0.0};
    double realExecutionTimeSeconds{0.0};
    double totalEnergyConsumedJoules{0.0};
    double totalUavActiveSeconds{0.0};
    double totalUavAvailableSeconds{0.0};

    // Derived KPIs
    double throughputPerHour{0.0};
    double meanLatencySeconds{0.0};
    double medianLatencySeconds{0.0};
    double p95LatencySeconds{0.0};
    double deadlineSuccessRatePercent{0.0};
    double energyPerDeliveryWattHours{0.0};
    double uavUtilizationPercent{0.0};
    double speedMultiplier{0.0};
};

/**
 * @brief High-precision fleet KPI metrics recorder and exporter.
 */
class MetricsTracker {
public:
    MetricsTracker() = default;

    void reset();

    void recordRequestCreated(const DeliveryRequest& request);
    void recordRequestDelivered(const DeliveryRequest& request, double completionTime);
    void recordRequestExpired(const DeliveryRequest& request, double expirationTime);
    void recordCollision(const CollisionEvent& event);
    void recordStep(
        const std::vector<UAVState>& uavs,
        double dt,
        double totalPowerWatts,
        double currentTime
    );

    void finalize(double totalRealWallTimeSeconds);

    [[nodiscard]] MetricsSnapshot getSnapshot() const;

    [[nodiscard]] std::string toJson() const;
    [[nodiscard]] std::string toCsv() const;

    bool saveToJsonFile(const std::string& filepath) const;
    bool saveToCsvFile(const std::string& filepath) const;

    void printSummary(std::ostream& os = std::cout) const;

    // Direct accessors
    [[nodiscard]] size_t getCreatedCount() const noexcept { return m_totalCreated; }
    [[nodiscard]] size_t getDeliveredCount() const noexcept { return m_totalDelivered; }
    [[nodiscard]] size_t getExpiredCount() const noexcept { return m_totalExpired; }
    [[nodiscard]] size_t getOnTimeCount() const noexcept { return m_totalOnTime; }
    [[nodiscard]] size_t getBuildingCollisionCount() const noexcept { return m_buildingCollisions; }
    [[nodiscard]] size_t getUavCollisionCount() const noexcept { return m_uavCollisions; }

    [[nodiscard]] double getThroughputPerHour() const;
    [[nodiscard]] double getMeanLatency() const;
    [[nodiscard]] double getMedianLatency() const;
    [[nodiscard]] double getP95Latency() const;
    [[nodiscard]] double getDeadlineSuccessRate() const;
    [[nodiscard]] double getEnergyPerDeliveryWh() const;
    [[nodiscard]] double getUavUtilization() const;
    [[nodiscard]] double getSpeedMultiplier() const;

    // Helper for manual metric feeding in unit tests
    void addDeliveryLatency(double latencySeconds, bool onTime);
    void addEnergyConsumedJoules(double joules);
    void setSimulatedTime(double simTimeSeconds);
    void setFleetUtilization(double activeSeconds, double availableSeconds);

private:
    size_t m_totalCreated{0};
    size_t m_totalDelivered{0};
    size_t m_totalExpired{0};
    size_t m_totalOnTime{0};
    size_t m_buildingCollisions{0};
    size_t m_uavCollisions{0};

    double m_simulatedTime{0.0};
    double m_realExecutionTime{0.0};
    double m_totalEnergyJoules{0.0};
    double m_totalActiveUavSeconds{0.0};
    double m_totalAvailableUavSeconds{0.0};

    std::vector<double> m_latencies;
};

} // namespace aerofleet

