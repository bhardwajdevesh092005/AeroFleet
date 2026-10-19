#pragma once

#include "fleet/FleetManager.hpp"

namespace aerofleet {

/**
 * @brief Interface for fleet-level request dispatch policies.
 *
 * The current project has no aggregate WorldState type, so dispatch operates
 * directly on the authoritative fleet, pending request queue, and city map.
 */
class ICoordinator {
public:
    virtual ~ICoordinator() = default;

    virtual void dispatch(
        FleetManager& fleet,
        std::vector<DeliveryRequest>& requests,
        const CityMap& map,
        double currentTime
    ) const = 0;
};

} // namespace aerofleet
