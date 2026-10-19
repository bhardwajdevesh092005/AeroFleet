#pragma once

#include "coordinator/ICoordinator.hpp"

namespace aerofleet {

struct HeuristicCoordinatorConfig {
    double nominalSpeed{10.0};
    double payloadCapacityKg{3.0};
    double reserveMarginPercent{15.0};
    double distanceWeight{1.0};
    double urgencyWeight{0.25};
    double ageWeight{0.1};
    double batteryWeight{20.0};
};

/** @brief Greedy, deterministic baseline request-to-UAV dispatcher. */
class HeuristicCoordinator final : public ICoordinator {
public:
    HeuristicCoordinator();
    explicit HeuristicCoordinator(const HeuristicCoordinatorConfig& config);

    void dispatch(
        FleetManager& fleet,
        std::vector<DeliveryRequest>& requests,
        const CityMap& map,
        double currentTime
    ) const override;

    [[nodiscard]] const HeuristicCoordinatorConfig& getConfig() const noexcept { return m_config; }

private:
    HeuristicCoordinatorConfig m_config;

    [[nodiscard]] bool energyFeasible(
        const UAVState& uav,
        const DeliveryRequest& request,
        const CityMap& map,
        const BatteryModel& battery
    ) const noexcept;
    [[nodiscard]] double score(
        const UAVState& uav,
        const DeliveryRequest& request,
        double currentTime,
        const BatteryModel& battery
    ) const noexcept;
};

} // namespace aerofleet
