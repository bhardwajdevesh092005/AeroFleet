#pragma once

#include <vector>
#include <string>
#include <memory>
#include <array>
#include "core/Config.hpp"
#include "core/Types.hpp"
#include "core/WorldState.hpp"
#include "core/SimulationClock.hpp"
#include "math/Vec3.hpp"
#include "math/Geometry.hpp"
#include "world/WorldGenerator.hpp"
#include "fleet/FleetManager.hpp"
#include "requests/RequestGenerator.hpp"
#include "control/WaypointController.hpp"
#include "coordinator/HeuristicCoordinator.hpp"
#include "physics/CollisionSystem.hpp"
#include "physics/Kinematics.hpp"
#include "telemetry/Metrics.hpp"
#include "telemetry/Logger.hpp"

namespace aerofleet {

/**
 * @brief High-dimensional vectorized observation for RL navigation agents.
 */
struct Observation {
    UAVId uavId{0};
    Vec3 egoVelocity{0.0, 0.0, 0.0};
    Vec3 egoAcceleration{0.0, 0.0, 0.0};
    Vec3 relativeTarget{0.0, 0.0, 0.0};
    double distanceToTarget{0.0};
    double batteryFraction{1.0};
    double payloadMass{0.0};
    double speed{0.0};
    std::vector<float> lidarBeams; // 44 beams normalized [0.0, 1.0]
    std::vector<float> nearbyUavs; // K nearest UAVs (dx, dy, dz, vx, vy, vz)
    double headingSin{0.0};
    double headingCos{1.0};
    std::array<float, 10> modeOneHot{};

    [[nodiscard]] std::vector<float> toVector() const;
    [[nodiscard]] static constexpr size_t vectorSize() noexcept { return 87; }
};

/**
 * @brief Gymnasium step transition result for multi-agent or single-agent RL.
 */
struct StepResult {
    std::vector<Observation> observations;
    std::vector<float> rewards;
    std::vector<bool> terminated;
    std::vector<bool> truncated;
    bool allDone{false};
    SnapshotMetrics metrics;
};

/**
 * @brief Comprehensive simulation engine encapsulating procedural generation,
 * rigid kinematics, continuous collision detection, battery mechanics,
 * request lifecycle, and observation extraction.
 */
class Simulator {
public:
    explicit Simulator(const SimulationConfig& config);
    explicit Simulator(const std::string& configPath = "config/simulation.json");

    void reset(uint64_t seed = 0);
    void resetNavigationTask(uint64_t seed, const Vec3& start, const Vec3& target);

    /**
     * @brief Step simulation by 1 policy control interval (dtPolicy) using provided velocity commands.
     * Integrates internal fixed physics sub-steps at dtPhysics.
     * @param actions Vector of velocity commands for each UAV
     */
    StepResult step(const std::vector<VelocityCommand>& actions);

    /**
     * @brief Step simulation using baseline heuristic APF navigation controllers.
     */
    StepResult stepHeuristic();

    /**
     * @brief Extract current observation for a specific UAV.
     */
    [[nodiscard]] Observation getObservation(UAVId uavId) const;

    /**
     * @brief Extract observations for all UAVs in fleet.
     */
    [[nodiscard]] std::vector<Observation> getAllObservations() const;

    /**
     * @brief Get immutable world snapshot for rendering or telemetry.
     */
    [[nodiscard]] WorldSnapshot getSnapshot() const;

    // Direct accessors
    [[nodiscard]] const SimulationConfig& getConfig() const noexcept { return m_config; }
    [[nodiscard]] const CityMap& getMap() const noexcept { return m_map; }
    [[nodiscard]] const FleetManager& getFleet() const noexcept { return m_fleet; }
    [[nodiscard]] FleetManager& getFleet() noexcept { return m_fleet; }
    [[nodiscard]] const MetricsTracker& getMetrics() const noexcept { return m_metrics; }
    [[nodiscard]] const EventLogger& getLogger() const noexcept { return m_logger; }
    [[nodiscard]] double getSimulatedTime() const noexcept { return m_clock.now(); }
    [[nodiscard]] uint64_t getStepCount() const noexcept { return m_clock.getStepCount(); }
    [[nodiscard]] size_t getFleetSize() const noexcept { return m_fleet.size(); }
    [[nodiscard]] bool navigationTaskSucceeded() const noexcept { return m_navigationTaskSucceeded; }

    void setSeed(uint64_t seed) { m_config.simulation.randomSeed = seed; reset(seed); }

private:
    SimulationConfig m_config;
    CityMap m_map;
    FleetManager m_fleet;
    RequestGenerator m_requestGen;
    WaypointController m_navigator;
    HeuristicCoordinator m_coordinator;
    CollisionSystem m_collisions;
    KinematicsIntegrator m_integrator;
    SimulationClock m_clock;
    MetricsTracker m_metrics;
    EventLogger m_logger;

    std::vector<DeliveryRequest> m_pendingRequests;
    std::vector<DeliveryRequest> m_deliveredRequests;
    std::vector<DeliveryRequest> m_requeuedRequests;
    std::vector<CollisionEvent> m_collisionEvents;
    std::vector<Vec3> m_prevPositions;

    double m_lastCoordinatorDispatchTime{-10.0};
    uint64_t m_currentSeed{42};
    bool m_navigationTask{false};
    bool m_navigationTaskSucceeded{false};
    Vec3 m_navigationTarget{0.0, 0.0, 0.0};
    double m_navigationArrivalRadius{2.0};

    void integratePhysicsSubStep(double dt, const std::vector<VelocityCommand>& actions);
    [[nodiscard]] std::vector<float> computeLidarBeams(const Vec3& position, double yaw, double maxRange = 100.0) const;
    [[nodiscard]] std::vector<float> computeNearbyUavs(UAVId egoId, size_t maxNeighbors = 3) const;
    [[nodiscard]] float computeReward(const UAVState& uav, const UAVState& prevUav, bool reached, bool crashed) const;
    [[nodiscard]] float computeNavigationReward(
        const UAVState& uav, const UAVState& prevUav, bool succeeded, bool failed, double dtPolicy
    ) const;
};

} // namespace aerofleet

