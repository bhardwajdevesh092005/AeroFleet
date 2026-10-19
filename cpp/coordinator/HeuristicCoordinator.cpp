#include "coordinator/HeuristicCoordinator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace aerofleet {
namespace {

double positiveOr(double value, double fallback) noexcept {
    return std::isfinite(value) && value > 0.0 ? value : fallback;
}

} // namespace

HeuristicCoordinator::HeuristicCoordinator() = default;

HeuristicCoordinator::HeuristicCoordinator(const HeuristicCoordinatorConfig& config)
    : m_config(config) {
    m_config.nominalSpeed = positiveOr(m_config.nominalSpeed, 10.0);
    m_config.payloadCapacityKg = positiveOr(m_config.payloadCapacityKg, 3.0);
    if (!std::isfinite(m_config.reserveMarginPercent)) m_config.reserveMarginPercent = 15.0;
    m_config.reserveMarginPercent = MathUtils::clamp(m_config.reserveMarginPercent, 0.0, 99.0);
    if (!std::isfinite(m_config.distanceWeight)) m_config.distanceWeight = 1.0;
    if (!std::isfinite(m_config.urgencyWeight)) m_config.urgencyWeight = 0.25;
    if (!std::isfinite(m_config.ageWeight)) m_config.ageWeight = 0.1;
    if (!std::isfinite(m_config.batteryWeight)) m_config.batteryWeight = 20.0;
}

bool HeuristicCoordinator::energyFeasible(
    const UAVState& uav,
    const DeliveryRequest& request,
    const CityMap& map,
    const BatteryModel& battery
) const noexcept {
    if (map.chargingStations.empty()) return false;

    const double speed = m_config.nominalSpeed;
    const double toPickup = uav.position.distance(request.pickupLocation);
    const double loadedFlight = request.pickupLocation.distance(request.dropoffLocation);
    double returnDistance = std::numeric_limits<double>::infinity();
    for (const auto& charger : map.chargingStations) {
        returnDistance = std::min(returnDistance, request.dropoffLocation.distance(charger.position));
    }

    const Vec3 cruiseVelocity(speed, 0.0, 0.0);
    const double outboundPower = battery.computePowerWatts(cruiseVelocity, 0.0);
    const double loadedPower = battery.computePowerWatts(cruiseVelocity, request.payloadMass);
    const double returnPower = battery.computePowerWatts(cruiseVelocity, 0.0);
    const double requiredJoules = outboundPower * (toPickup / speed) +
        loadedPower * (loadedFlight / speed) + returnPower * (returnDistance / speed);

    const auto& batteryConfig = battery.getConfig();
    const double capacity = batteryConfig.totalCapacityJoules();
    if (!std::isfinite(requiredJoules) || capacity <= 0.0) return false;
    const double requiredPercent = 100.0 * requiredJoules / capacity;
    const double reserve = std::max(m_config.reserveMarginPercent, batteryConfig.reservePercent);
    return uav.batteryPercent - requiredPercent >= reserve;
}

double HeuristicCoordinator::score(
    const UAVState& uav,
    const DeliveryRequest& request,
    double currentTime,
    const BatteryModel& battery
) const noexcept {
    const double firstLeg = uav.position.distance(request.pickupLocation);
    const double serviceDistance = request.pickupLocation.distance(request.dropoffLocation);
    const double estimatedArrival = currentTime + (firstLeg + serviceDistance) / m_config.nominalSpeed;
    const double slack = request.deadline - estimatedArrival;
    const double urgency = std::max(0.0, slack);
    const double age = std::max(0.0, currentTime - request.creationTime);
    const double socFraction = MathUtils::clamp(uav.batteryPercent / 100.0, 0.0, 1.0);
    const double joules = (firstLeg + serviceDistance) / m_config.nominalSpeed *
        battery.computePowerWatts(Vec3(m_config.nominalSpeed, 0.0, 0.0), request.payloadMass);
    return m_config.distanceWeight * (firstLeg + serviceDistance) +
           m_config.urgencyWeight * urgency - m_config.ageWeight * age -
           m_config.batteryWeight * socFraction + joules * 1e-6;
}

void HeuristicCoordinator::dispatch(
    FleetManager& fleet,
    std::vector<DeliveryRequest>& requests,
    const CityMap& map,
    double currentTime
) const {
    if (!std::isfinite(currentTime) || fleet.size() == 0 || requests.empty()) return;

    const BatteryModel& battery = fleet.getBatteryModel();
    std::vector<bool> assigned(fleet.size(), false);
    for (size_t i = 0; i < fleet.size(); ++i) {
        assigned[i] = fleet.getUAV(i).mode != UAVMode::IDLE;
    }

    // Repeatedly choose the globally lowest-cost feasible pair. Each request
    // and each UAV can be consumed at most once per dispatch call.
    while (true) {
        size_t bestRequest = requests.size();
        size_t bestUav = fleet.size();
        double bestScore = std::numeric_limits<double>::infinity();

        for (size_t ri = 0; ri < requests.size(); ++ri) {
            const auto& request = requests[ri];
            if (request.status != RequestStatus::PENDING || request.isExpired(currentTime) ||
                !std::isfinite(request.payloadMass) || request.payloadMass < 0.0 ||
                request.payloadMass > m_config.payloadCapacityKg) continue;

            for (size_t ui = 0; ui < fleet.size(); ++ui) {
                if (assigned[ui]) continue;
                const auto& uav = fleet.getUAV(ui);
                if (uav.mode != UAVMode::IDLE || !energyFeasible(uav, request, map, battery)) continue;
                const double candidate = score(uav, request, currentTime, battery);
                if (candidate < bestScore ||
                    (candidate == bestScore && request.id < requests[bestRequest].id)) {
                    bestScore = candidate;
                    bestRequest = ri;
                    bestUav = ui;
                }
            }
        }

        if (bestRequest == requests.size()) break;
        auto& request = requests[bestRequest];
        const UAVId uavId = fleet.getUAV(bestUav).id;
        if (fleet.assignRequest(uavId, request)) {
            request.transitionTo(RequestStatus::ASSIGNED, currentTime);
            request.assignedUavId = uavId;
            assigned[bestUav] = true;
        } else {
            assigned[bestUav] = true;
        }
    }
}

} // namespace aerofleet
