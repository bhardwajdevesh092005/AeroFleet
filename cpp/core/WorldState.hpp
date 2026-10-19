#pragma once

#include <vector>
#include <string>
#include <cstdint>
#include "core/Types.hpp"
#include "fleet/UAVState.hpp"
#include "requests/DeliveryRequest.hpp"
#include "world/CityMap.hpp"
#include "physics/CollisionSystem.hpp"
#include "math/MathUtils.hpp"

namespace aerofleet {

/**
 * @brief High-level aggregated metrics packaged inside a WorldSnapshot for HUD and telemetry.
 */
struct SnapshotMetrics {
    size_t totalCreated{0};
    size_t totalDelivered{0};
    size_t totalExpired{0};
    size_t totalActive{0};
    size_t totalPending{0};
    size_t collisionCount{0};
    double deliverySuccessRate{100.0};
    double averageDeliveryDuration{0.0};
    double realFPS{60.0};
};

/**
 * @brief Immutable snapshot of the simulation world state at a specific point in time.
 * Designed for pure, read-only consumption by raylib renderer, telemetry loggers,
 * and Gymnasium observation builders without mutating physics or simulator memory.
 */
struct WorldSnapshot {
    double simulationTime{0.0};
    double wallTime{0.0};
    uint64_t stepCount{0};

    std::vector<UAVState> uavs;
    std::vector<DeliveryRequest> activeRequests;
    std::vector<ChargingStation> chargingStations;
    std::vector<CollisionEvent> recentEvents;
    SnapshotMetrics metrics;

    /**
     * @brief Lookup UAV state by ID in the snapshot.
     */
    [[nodiscard]] const UAVState* findUAV(UAVId id) const noexcept {
        for (const auto& u : uavs) {
            if (u.id == id) return &u;
        }
        return nullptr;
    }

    /**
     * @brief Lookup DeliveryRequest by ID in the snapshot.
     */
    [[nodiscard]] const DeliveryRequest* findRequest(RequestId id) const noexcept {
        for (const auto& r : activeRequests) {
            if (r.id == id) return &r;
        }
        return nullptr;
    }

    /**
     * @brief Linearly interpolate between two consecutive snapshots for smooth graphics rendering.
     * @param prev Snapshot from preceding physics step
     * @param curr Snapshot from most recent physics step
     * @param alpha Fractional factor in [0.0, 1.0] representing (t_render - t_prev) / dt_physics
     */
    [[nodiscard]] static WorldSnapshot interpolate(
        const WorldSnapshot& prev,
        const WorldSnapshot& curr,
        double alpha
    ) {
        WorldSnapshot result = curr;
        result.simulationTime = MathUtils::lerp(prev.simulationTime, curr.simulationTime, alpha);

        // Smoothly interpolate UAV physical positions and yaw orientations
        for (auto& uCurr : result.uavs) {
            const auto* uPrev = prev.findUAV(uCurr.id);
            if (uPrev) {
                uCurr.position = uPrev->position.lerp(uCurr.position, alpha);
                const double deltaYaw = MathUtils::wrapAnglePi(uCurr.yaw - uPrev->yaw);
                uCurr.yaw = MathUtils::wrapAnglePi(uPrev->yaw + deltaYaw * alpha);
            }
        }

        return result;
    }
};

} // namespace aerofleet

