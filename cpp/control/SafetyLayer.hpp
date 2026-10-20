#pragma once

#include <cstddef>
#include "fleet/UAVState.hpp"
#include "world/CityMap.hpp"
#include "physics/Kinematics.hpp"

namespace aerofleet {

struct SafetyLayerConfig {
    double lookaheadTime{2.0};      // Lookahead projection horizon in seconds
    double timeStep{0.1};           // Discrete check interval in seconds
    double uavRadius{0.5};          // Collision sphere radius
    double safetyMargin{0.8};       // Buffer clearance around obstacles
    double emergencyClimbSpeed{5.0};// Vertical climb velocity for obstacle evasion
};

/**
 * @brief Deterministic safety supervisor and trajectory validator.
 *
 * Intercepts candidate policy velocity commands and performs short-horizon
 * predictive collision detection against static buildings and world bounds.
 * If a collision or boundary breach is projected, overrides the command
 * with an evasive emergency climb or hover stop.
 */
class SafetyLayer {
public:
    explicit SafetyLayer(const SafetyLayerConfig& config = SafetyLayerConfig{});

    /**
     * @brief Validate candidate command and return either the candidate or a safe override.
     */
    [[nodiscard]] VelocityCommand filterAction(
        const UAVState& uav,
        const CityMap& map,
        const VelocityCommand& candidate,
        double dt
    );

    /**
     * @brief Test whether a linear velocity projection is collision-free over a horizon.
     */
    [[nodiscard]] bool isTrajectorySafe(
        const UAVState& uav,
        const CityMap& map,
        const VelocityCommand& cmd,
        double horizon
    ) const;

    [[nodiscard]] size_t getInterventionCount() const noexcept { return m_interventions; }
    void resetInterventionCount() noexcept { m_interventions = 0; }
    [[nodiscard]] const SafetyLayerConfig& getConfig() const noexcept { return m_config; }

private:
    SafetyLayerConfig m_config;
    size_t m_interventions{0};
};

} // namespace aerofleet
