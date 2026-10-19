#include "telemetry/Metrics.hpp"
#include <fstream>
#include <sstream>
#include <iomanip>

namespace aerofleet {

void MetricsTracker::reset() {
    m_totalCreated = 0;
    m_totalDelivered = 0;
    m_totalExpired = 0;
    m_totalOnTime = 0;
    m_buildingCollisions = 0;
    m_uavCollisions = 0;

    m_simulatedTime = 0.0;
    m_realExecutionTime = 0.0;
    m_totalEnergyJoules = 0.0;
    m_totalActiveUavSeconds = 0.0;
    m_totalAvailableUavSeconds = 0.0;

    m_latencies.clear();
}

void MetricsTracker::recordRequestCreated(const DeliveryRequest& /*request*/) {
    m_totalCreated++;
}

void MetricsTracker::recordRequestDelivered(const DeliveryRequest& request, double completionTime) {
    m_totalDelivered++;
    double latency = completionTime - request.creationTime;
    if (latency < 0.0) latency = 0.0;
    m_latencies.push_back(latency);

    if (completionTime <= request.deadline) {
        m_totalOnTime++;
    }
}

void MetricsTracker::recordRequestExpired(const DeliveryRequest& /*request*/, double /*expirationTime*/) {
    m_totalExpired++;
}

void MetricsTracker::recordCollision(const CollisionEvent& event) {
    if (event.type == CollisionType::BUILDING) {
        m_buildingCollisions++;
    } else if (event.type == CollisionType::UAV_UAV) {
        m_uavCollisions++;
    }
}

void MetricsTracker::recordStep(
    const std::vector<UAVState>& uavs,
    double dt,
    double totalPowerWatts,
    double currentTime
) {
    m_simulatedTime = currentTime;
    m_totalEnergyJoules += (totalPowerWatts * dt);

    for (const auto& uav : uavs) {
        m_totalAvailableUavSeconds += dt;
        if (uav.mode == UAVMode::FLYING_TO_PICKUP ||
            uav.mode == UAVMode::PICKING_UP ||
            uav.mode == UAVMode::FLYING_TO_DESTINATION ||
            uav.mode == UAVMode::DROPPING_OFF ||
            uav.mode == UAVMode::RETURNING_TO_CHARGE) {
            m_totalActiveUavSeconds += dt;
        }
    }
}

void MetricsTracker::finalize(double totalRealWallTimeSeconds) {
    m_realExecutionTime = totalRealWallTimeSeconds > 0.0 ? totalRealWallTimeSeconds : 0.0;
}

double MetricsTracker::getThroughputPerHour() const {
    if (m_simulatedTime <= 0.0) return 0.0;
    return (static_cast<double>(m_totalDelivered) / m_simulatedTime) * 3600.0;
}

double MetricsTracker::getMeanLatency() const {
    if (m_latencies.empty()) return 0.0;
    double sum = std::accumulate(m_latencies.begin(), m_latencies.end(), 0.0);
    return sum / static_cast<double>(m_latencies.size());
}

double MetricsTracker::getMedianLatency() const {
    if (m_latencies.empty()) return 0.0;
    std::vector<double> sorted = m_latencies;
    std::sort(sorted.begin(), sorted.end());
    size_t n = sorted.size();
    if (n % 2 == 1) {
        return sorted[n / 2];
    }
    return (sorted[(n / 2) - 1] + sorted[n / 2]) * 0.5;
}

double MetricsTracker::getP95Latency() const {
    if (m_latencies.empty()) return 0.0;
    std::vector<double> sorted = m_latencies;
    std::sort(sorted.begin(), sorted.end());
    size_t n = sorted.size();
    if (n == 1) return sorted[0];

    double rank = 0.95 * static_cast<double>(n - 1);
    size_t low = static_cast<size_t>(std::floor(rank));
    size_t high = static_cast<size_t>(std::ceil(rank));
    double frac = rank - static_cast<double>(low);
    return sorted[low] + frac * (sorted[high] - sorted[low]);
}

double MetricsTracker::getDeadlineSuccessRate() const {
    size_t total = m_totalDelivered + m_totalExpired;
    if (total == 0) return 100.0;
    return (static_cast<double>(m_totalOnTime) / static_cast<double>(total)) * 100.0;
}

double MetricsTracker::getEnergyPerDeliveryWh() const {
    if (m_totalDelivered == 0) return 0.0;
    double totalWh = m_totalEnergyJoules / 3600.0;
    return totalWh / static_cast<double>(m_totalDelivered);
}

double MetricsTracker::getUavUtilization() const {
    if (m_totalAvailableUavSeconds <= 0.0) return 0.0;
    return (m_totalActiveUavSeconds / m_totalAvailableUavSeconds) * 100.0;
}

double MetricsTracker::getSpeedMultiplier() const {
    if (m_realExecutionTime <= 0.0) return 0.0;
    return m_simulatedTime / m_realExecutionTime;
}

void MetricsTracker::addDeliveryLatency(double latencySeconds, bool onTime) {
    m_totalDelivered++;
    m_latencies.push_back(latencySeconds);
    if (onTime) {
        m_totalOnTime++;
    }
}

void MetricsTracker::addEnergyConsumedJoules(double joules) {
    if (joules > 0.0) {
        m_totalEnergyJoules += joules;
    }
}

void MetricsTracker::setSimulatedTime(double simTimeSeconds) {
    if (simTimeSeconds >= 0.0) {
        m_simulatedTime = simTimeSeconds;
    }
}

void MetricsTracker::setFleetUtilization(double activeSeconds, double availableSeconds) {
    m_totalActiveUavSeconds = activeSeconds;
    m_totalAvailableUavSeconds = availableSeconds;
}

MetricsSnapshot MetricsTracker::getSnapshot() const {
    MetricsSnapshot s;
    s.totalRequestsCreated = m_totalCreated;
    s.totalRequestsDelivered = m_totalDelivered;
    s.totalRequestsExpired = m_totalExpired;
    s.totalRequestsOnTime = m_totalOnTime;
    s.totalBuildingCollisions = m_buildingCollisions;
    s.totalUavCollisions = m_uavCollisions;

    s.simulatedTimeSeconds = m_simulatedTime;
    s.realExecutionTimeSeconds = m_realExecutionTime;
    s.totalEnergyConsumedJoules = m_totalEnergyJoules;
    s.totalUavActiveSeconds = m_totalActiveUavSeconds;
    s.totalUavAvailableSeconds = m_totalAvailableUavSeconds;

    s.throughputPerHour = getThroughputPerHour();
    s.meanLatencySeconds = getMeanLatency();
    s.medianLatencySeconds = getMedianLatency();
    s.p95LatencySeconds = getP95Latency();
    s.deadlineSuccessRatePercent = getDeadlineSuccessRate();
    s.energyPerDeliveryWattHours = getEnergyPerDeliveryWh();
    s.uavUtilizationPercent = getUavUtilization();
    s.speedMultiplier = getSpeedMultiplier();

    return s;
}

std::string MetricsTracker::toJson() const {
    MetricsSnapshot s = getSnapshot();
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(4);
    ss << "{\n";
    ss << "  \"simulated_time_s\": " << s.simulatedTimeSeconds << ",\n";
    ss << "  \"real_execution_time_s\": " << s.realExecutionTimeSeconds << ",\n";
    ss << "  \"speed_multiplier\": " << s.speedMultiplier << ",\n";
    ss << "  \"total_requests_created\": " << s.totalRequestsCreated << ",\n";
    ss << "  \"total_requests_delivered\": " << s.totalRequestsDelivered << ",\n";
    ss << "  \"total_requests_expired\": " << s.totalRequestsExpired << ",\n";
    ss << "  \"total_requests_on_time\": " << s.totalRequestsOnTime << ",\n";
    ss << "  \"deadline_success_rate_percent\": " << s.deadlineSuccessRatePercent << ",\n";
    ss << "  \"throughput_per_hour\": " << s.throughputPerHour << ",\n";
    ss << "  \"mean_latency_s\": " << s.meanLatencySeconds << ",\n";
    ss << "  \"median_latency_s\": " << s.medianLatencySeconds << ",\n";
    ss << "  \"p95_latency_s\": " << s.p95LatencySeconds << ",\n";
    ss << "  \"total_energy_consumed_wh\": " << (s.totalEnergyConsumedJoules / 3600.0) << ",\n";
    ss << "  \"energy_per_delivery_wh\": " << s.energyPerDeliveryWattHours << ",\n";
    ss << "  \"uav_utilization_percent\": " << s.uavUtilizationPercent << ",\n";
    ss << "  \"building_collisions\": " << s.totalBuildingCollisions << ",\n";
    ss << "  \"uav_collisions\": " << s.totalUavCollisions << "\n";
    ss << "}\n";
    return ss.str();
}

std::string MetricsTracker::toCsv() const {
    MetricsSnapshot s = getSnapshot();
    std::ostringstream ss;
    ss << "simulated_time_s,real_time_s,speed_mult,created,delivered,expired,on_time,"
          "deadline_rate_pct,throughput_per_hr,mean_latency_s,median_latency_s,p95_latency_s,"
          "total_energy_wh,wh_per_delivery,uav_utilization_pct,building_collisions,uav_collisions\n";
    ss << std::fixed << std::setprecision(4)
       << s.simulatedTimeSeconds << ","
       << s.realExecutionTimeSeconds << ","
       << s.speedMultiplier << ","
       << s.totalRequestsCreated << ","
       << s.totalRequestsDelivered << ","
       << s.totalRequestsExpired << ","
       << s.totalRequestsOnTime << ","
       << s.deadlineSuccessRatePercent << ","
       << s.throughputPerHour << ","
       << s.meanLatencySeconds << ","
       << s.medianLatencySeconds << ","
       << s.p95LatencySeconds << ","
       << (s.totalEnergyConsumedJoules / 3600.0) << ","
       << s.energyPerDeliveryWattHours << ","
       << s.uavUtilizationPercent << ","
       << s.totalBuildingCollisions << ","
       << s.totalUavCollisions << "\n";
    return ss.str();
}

bool MetricsTracker::saveToJsonFile(const std::string& filepath) const {
    std::ofstream file(filepath, std::ios::out | std::ios::trunc);
    if (!file.is_open()) return false;
    file << toJson();
    return true;
}

bool MetricsTracker::saveToCsvFile(const std::string& filepath) const {
    std::ofstream file(filepath, std::ios::out | std::ios::trunc);
    if (!file.is_open()) return false;
    file << toCsv();
    return true;
}

void MetricsTracker::printSummary(std::ostream& os) const {
    MetricsSnapshot s = getSnapshot();
    os << "========================================\n";
    os << "       AeroFleet Performance Report     \n";
    os << "========================================\n";
    os << std::fixed << std::setprecision(2);
    os << "  Simulated Time:        " << s.simulatedTimeSeconds << " s\n";
    os << "  Wall Execution Time:   " << s.realExecutionTimeSeconds << " s\n";
    os << "  Simulation Speed:      " << s.speedMultiplier << "x real-time\n";
    os << "----------------------------------------\n";
    os << "  Requests Created:      " << s.totalRequestsCreated << "\n";
    os << "  Requests Delivered:    " << s.totalRequestsDelivered << "\n";
    os << "  Requests Expired:      " << s.totalRequestsExpired << "\n";
    os << "  On-Time Deliveries:    " << s.totalRequestsOnTime << " ("
       << s.deadlineSuccessRatePercent << "%)\n";
    os << "  Throughput:            " << s.throughputPerHour << " deliveries/hr\n";
    os << "----------------------------------------\n";
    os << "  Mean Delivery Latency: " << s.meanLatencySeconds << " s\n";
    os << "  Median Latency:        " << s.medianLatencySeconds << " s\n";
    os << "  P95 Latency:           " << s.p95LatencySeconds << " s\n";
    os << "----------------------------------------\n";
    os << "  Fleet Energy Consumed: " << (s.totalEnergyConsumedJoules / 3600.0) << " Wh\n";
    os << "  Efficiency:            " << s.energyPerDeliveryWattHours << " Wh / delivery\n";
    os << "  Fleet UAV Utilization: " << s.uavUtilizationPercent << "%\n";
    os << "----------------------------------------\n";
    os << "  Building Collisions:   " << s.totalBuildingCollisions << "\n";
    os << "  UAV-UAV Collisions:    " << s.totalUavCollisions << "\n";
    os << "========================================\n";
}

} // namespace aerofleet

