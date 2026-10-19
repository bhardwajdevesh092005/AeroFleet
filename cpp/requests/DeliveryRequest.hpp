#pragma once

#include <string>
#include <cstdint>
#include <stdexcept>
#include "core/Types.hpp"
#include "math/Vec3.hpp"

namespace aerofleet {

class RequestTransitionException : public std::runtime_error {
public:
    explicit RequestTransitionException(const std::string& message)
        : std::runtime_error(message) {}
};

/**
 * @brief Authoritative lifecycle record for a customer delivery request.
 */
struct DeliveryRequest {
    RequestId id{-1};
    int32_t storeId{-1};
    Vec3 pickupLocation{0.0, 0.0, 0.0};
    int32_t customerId{-1};
    Vec3 dropoffLocation{0.0, 0.0, 0.0};
    double creationTime{0.0};
    double deadline{0.0};
    double payloadMass{1.0};               ///< Package mass in kg
    RequestStatus status{RequestStatus::PENDING};
    UAVId assignedUavId{-1};
    double pickupTime{0.0};
    double deliveryTime{0.0};

    DeliveryRequest() = default;

    DeliveryRequest(
        RequestId rId,
        int32_t sId,
        const Vec3& pickup,
        int32_t cId,
        const Vec3& dropoff,
        double created,
        double due,
        double mass
    ) : id(rId),
        storeId(sId),
        pickupLocation(pickup),
        customerId(cId),
        dropoffLocation(dropoff),
        creationTime(created),
        deadline(due),
        payloadMass(mass),
        status(RequestStatus::PENDING),
        assignedUavId(-1) {}

    /**
     * @brief Direct Euclidean distance between pickup and dropoff points in meters.
     */
    [[nodiscard]] double transitDistance() const noexcept {
        return pickupLocation.distance(dropoffLocation);
    }

    /**
     * @brief Checks if request deadline has elapsed without successful delivery.
     */
    [[nodiscard]] bool isExpired(double currentTime) const noexcept {
        return (status != RequestStatus::DELIVERED &&
                status != RequestStatus::CANCELLED &&
                status != RequestStatus::REJECTED &&
                currentTime > deadline);
    }

    /**
     * @brief Validates permissible state transitions in delivery lifecycle.
     */
    [[nodiscard]] bool canTransitionTo(RequestStatus next) const noexcept {
        switch (status) {
            case RequestStatus::PENDING:
                return next == RequestStatus::ASSIGNED ||
                       next == RequestStatus::EXPIRED ||
                       next == RequestStatus::CANCELLED ||
                       next == RequestStatus::REJECTED;
            case RequestStatus::ASSIGNED:
                return next == RequestStatus::PICKING_UP ||
                       next == RequestStatus::PENDING || // Re-queued
                       next == RequestStatus::EXPIRED ||
                       next == RequestStatus::CANCELLED;
            case RequestStatus::PICKING_UP:
                return next == RequestStatus::IN_TRANSIT ||
                       next == RequestStatus::PENDING || // Aborted / re-queued
                       next == RequestStatus::EXPIRED ||
                       next == RequestStatus::CANCELLED;
            case RequestStatus::IN_TRANSIT:
                return next == RequestStatus::DELIVERED ||
                       next == RequestStatus::PENDING || // Emergency re-queued
                       next == RequestStatus::EXPIRED ||
                       next == RequestStatus::CANCELLED;
            case RequestStatus::DELIVERED:
            case RequestStatus::EXPIRED:
            case RequestStatus::CANCELLED:
            case RequestStatus::REJECTED:
                // Terminal states
                return false;
            default:
                return false;
        }
    }

    /**
     * @brief Atomically advances request state machine, updating timestamps.
     * Throws RequestTransitionException on illegal transition attempt.
     */
    void transitionTo(RequestStatus next, double currentTime = 0.0) {
        if (!canTransitionTo(next)) {
            throw RequestTransitionException(
                "Illegal delivery request state transition from " +
                requestStatusToString(status) + " to " + requestStatusToString(next) +
                " for Request ID: " + std::to_string(id)
            );
        }
        status = next;
        if (next == RequestStatus::PICKING_UP && pickupTime <= 0.0) {
            pickupTime = currentTime;
        } else if (next == RequestStatus::DELIVERED) {
            deliveryTime = currentTime;
        } else if (next == RequestStatus::PENDING) {
            assignedUavId = -1;
        }
    }
};

} // namespace aerofleet

