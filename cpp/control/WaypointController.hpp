#pragma once

#include "control/INavigationPolicy.hpp"
#include "physics/Kinematics.hpp"

namespace aerofleet {

struct WaypointControllerConfig {
    double attractiveGain{1.0};
    double repulsiveGain{30.0};
    double influenceDistance{20.0};
    double arrivalRadius{1.0};
    double safetyRadius{1.0};
    double maxHorizontalSpeed{15.0};
    double maxVerticalSpeed{5.0};
    double verticalEscapeBias{1.5};
    double tangentialGain{0.2};
};

/**
 * @brief Deterministic artificial-potential-field waypoint navigator.
 *
 * `computeAction` returns a desired velocity. `KinematicsIntegrator` remains
 * responsible for enforcing physical acceleration and speed limits.
 */
class WaypointController final : public INavigationPolicy {
public:
    WaypointController();
    explicit WaypointController(const WaypointControllerConfig& config);
    explicit WaypointController(const KinematicsConfig& kinematics);

    [[nodiscard]] VelocityCommand computeAction(
        const UAVState& uav,
        const CityMap& map,
        double dt
    ) const override;

    [[nodiscard]] const WaypointControllerConfig& getConfig() const noexcept { return m_config; }

private:
    WaypointControllerConfig m_config;
};

} // namespace aerofleet
