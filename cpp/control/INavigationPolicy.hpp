#pragma once

#include "fleet/UAVState.hpp"
#include "world/CityMap.hpp"
#include "physics/Kinematics.hpp"

namespace aerofleet {

/**
 * @brief Interface for high-level navigation policies.
 *
 * Policies produce a velocity command; the authoritative kinematics integrator
 * applies acceleration, speed, and altitude constraints.
 */
class INavigationPolicy {
public:
    virtual ~INavigationPolicy() = default;

    [[nodiscard]] virtual VelocityCommand computeAction(
        const UAVState& uav,
        const CityMap& map,
        double dt
    ) const = 0;
};

} // namespace aerofleet
