#pragma once

#include <string>
#include <cstdint>

namespace aerofleet {

/**
 * @brief Representation of continuous simulation time in seconds.
 */
using SimulationTime = double;

/**
 * @brief Unique identifier types for fleet entities.
 */
using UAVId = int32_t;
using RequestId = int32_t;
using StationId = int32_t;

/**
 * @brief Operational lifecycle states for an individual UAV.
 */
enum class UAVMode : uint8_t {
    IDLE = 0,               ///< Stationed at hub/charging pad, available for dispatch
    ASSIGNED,               ///< Request allocated, preparing flight trajectory
    FLYING_TO_PICKUP,       ///< En route to store pickup coordinates
    PICKING_UP,             ///< Docked/hovering at store, attaching payload
    FLYING_TO_DESTINATION,  ///< En route to customer dropoff with payload
    DROPPING_OFF,           ///< Hovering at customer site, releasing payload
    RETURNING_TO_CHARGE,    ///< Low battery or repositioning to nearest charger
    CHARGING,               ///< Replenishing battery at charging dock
    EMERGENCY,              ///< Critical failure, collision, or battery exhaustion stop
    OFFLINE                 ///< Decommissioned or disabled for maintenance
};

/**
 * @brief Convert UAVMode enum to human-readable string representation.
 */
inline std::string uavModeToString(UAVMode mode) {
    switch (mode) {
        case UAVMode::IDLE: return "IDLE";
        case UAVMode::ASSIGNED: return "ASSIGNED";
        case UAVMode::FLYING_TO_PICKUP: return "FLYING_TO_PICKUP";
        case UAVMode::PICKING_UP: return "PICKING_UP";
        case UAVMode::FLYING_TO_DESTINATION: return "FLYING_TO_DESTINATION";
        case UAVMode::DROPPING_OFF: return "DROPPING_OFF";
        case UAVMode::RETURNING_TO_CHARGE: return "RETURNING_TO_CHARGE";
        case UAVMode::CHARGING: return "CHARGING";
        case UAVMode::EMERGENCY: return "EMERGENCY";
        case UAVMode::OFFLINE: return "OFFLINE";
        default: return "UNKNOWN";
    }
}

/**
 * @brief Lifecycle states for customer delivery requests.
 */
enum class RequestStatus : uint8_t {
    PENDING = 0,    ///< Created by generator, waiting for coordinator assignment
    ASSIGNED,       ///< Assigned to an eligible UAV
    PICKING_UP,     ///< UAV arrived at pickup store
    IN_TRANSIT,     ///< Payload collected, currently in transit to customer
    DELIVERED,      ///< Successfully delivered to customer within deadline
    EXPIRED,        ///< Unassigned or late past tolerable deadline
    CANCELLED,      ///< Aborted due to UAV emergency or customer cancellation
    REJECTED        ///< Infeasible request outside service boundary
};

/**
 * @brief Convert RequestStatus enum to human-readable string representation.
 */
inline std::string requestStatusToString(RequestStatus status) {
    switch (status) {
        case RequestStatus::PENDING: return "PENDING";
        case RequestStatus::ASSIGNED: return "ASSIGNED";
        case RequestStatus::PICKING_UP: return "PICKING_UP";
        case RequestStatus::IN_TRANSIT: return "IN_TRANSIT";
        case RequestStatus::DELIVERED: return "DELIVERED";
        case RequestStatus::EXPIRED: return "EXPIRED";
        case RequestStatus::CANCELLED: return "CANCELLED";
        case RequestStatus::REJECTED: return "REJECTED";
        default: return "UNKNOWN";
    }
}

} // namespace aerofleet

