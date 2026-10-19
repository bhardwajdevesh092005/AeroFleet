#pragma once

#include <vector>
#include <string>
#include <utility>
#include <cstdint>
#include "fleet/UAVState.hpp"
#include "world/CityMap.hpp"
#include "math/Geometry.hpp"

namespace aerofleet {

enum class CollisionType : uint8_t {
    NONE = 0,
    BUILDING,
    UAV_UAV,
    WORLD_BOUNDARY,
    GROUND
};

inline std::string collisionTypeToString(CollisionType type) {
    switch (type) {
        case CollisionType::BUILDING: return "BUILDING";
        case CollisionType::UAV_UAV: return "UAV_UAV";
        case CollisionType::WORLD_BOUNDARY: return "WORLD_BOUNDARY";
        case CollisionType::GROUND: return "GROUND";
        default: return "NONE";
    }
}

struct CollisionEvent {
    CollisionType type{CollisionType::NONE};
    int32_t uavIdA{-1};
    int32_t uavIdB{-1};
    int32_t buildingId{-1};
    Vec3 contactPoint{0.0, 0.0, 0.0};
    Vec3 contactNormal{0.0, 0.0, 0.0};
    double simulationTime{0.0};
};

/**
 * @brief Continuous Collision Detection (CCD) and deterministic conflict resolution system.
 */
class CollisionSystem {
public:
    explicit CollisionSystem(double uavRadius = 0.5);

    /**
     * @brief Checks swept continuous collisions for all UAVs against buildings and mutual UAV conflicts.
     * @param uavs Vector of active UAV states (modified if position rollback is applied)
     * @param prevPositions Positions of UAVs prior to the current physics step
     * @param map Authoritative city environment
     * @param currentTime Current simulation clock
     * @return List of collision events detected in this step
     */
    std::vector<CollisionEvent> evaluateAndResolve(
        std::vector<UAVState>& uavs,
        const std::vector<Vec3>& prevPositions,
        const CityMap& map,
        double currentTime
    );

    /**
     * @brief Check single swept trajectory against static city buildings.
     */
    [[nodiscard]] bool checkStaticCollision(
        const Vec3& start,
        const Vec3& end,
        const CityMap& map,
        CollisionEvent& outEvent
    ) const;

    /**
     * @brief Check pairwise continuous conflict between two moving UAVs.
     */
    [[nodiscard]] bool checkDynamicCollision(
        const UAVState& uavA, const Vec3& prevPosA,
        const UAVState& uavB, const Vec3& prevPosB,
        CollisionEvent& outEvent
    ) const;

    [[nodiscard]] double getUavRadius() const noexcept { return m_uavRadius; }

private:
    double m_uavRadius{0.5};
};

} // namespace aerofleet

