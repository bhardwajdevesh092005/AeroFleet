#pragma once

#include <vector>
#include <unordered_map>
#include <string>
#include <memory>
#include "core/Config.hpp"
#include "fleet/UAVState.hpp"
#include "requests/DeliveryRequest.hpp"
#include "world/CityMap.hpp"
#include "physics/BatteryModel.hpp"

namespace aerofleet {

/**
 * @brief Context data associated with an individual active UAV in the fleet.
 */
struct UAVContext {
    double dwellTimer{0.0};              ///< Accumulated dwell time in PICKING_UP or DROPPING_OFF
    int32_t assignedRequestId{-1};       ///< Currently assigned request ID (-1 if none)
    DeliveryRequest activeRequest{};     ///< Cached request metadata
    int32_t targetChargerId{-1};         ///< Target charging dock ID when returning to charge
    Vec3 homeBasePosition{0.0, 0.0, 0.0};///< Return coordinate when idle
};

/**
 * @brief Authoritative fleet state machine and operational lifecycle manager.
 */
class FleetManager {
public:
    FleetManager() = default;
    FleetManager(
        const FleetSettings& fleetSettings,
        const EnergySettings& energySettings,
        const CityMap& map,
        double defaultAltitude = 15.0
    );

    /**
     * @brief Initializes the fleet, spawning UAVs at default pads or charging stations.
     */
    void initialize(
        const FleetSettings& fleetSettings,
        const EnergySettings& energySettings,
        const CityMap& map,
        double defaultAltitude = 15.0
    );

    /**
     * @brief Attempts to allocate a pending delivery request to an eligible UAV.
     * Rejects assignment if UAV is not in IDLE mode or if payload exceeds capacity.
     * @return true if assignment succeeded, false otherwise
     */
    bool assignRequest(UAVId uavId, const DeliveryRequest& request);

    /**
     * @brief Aborts active assignment for a UAV, transitioning request back to PENDING if not delivered.
     * @return true if an assignment was aborted, false otherwise
     */
    bool abortAssignment(UAVId uavId, DeliveryRequest& outRequeuedRequest);

    /**
     * @brief Steps the fleet state machine over time step dt.
     * Evaluates arrival at waypoints, dwell times, payload latching/release,
     * low-battery RTH interrupts, and mode transitions.
     */
    void step(
        double dt,
        double currentTime,
        CityMap& map,
        std::vector<DeliveryRequest>& outDeliveredRequests,
        std::vector<DeliveryRequest>& outRequeuedRequests
    );

    /**
     * @brief Finds the nearest charging station with available slots to a given coordinate.
     */
    [[nodiscard]] int32_t findNearestAvailableCharger(const Vec3& position, const CityMap& map) const;

    // Fleet accessors
    [[nodiscard]] size_t size() const noexcept { return m_uavs.size(); }
    [[nodiscard]] const UAVState& getUAV(size_t index) const { return m_uavs.at(index); }
    [[nodiscard]] UAVState& getUAV(size_t index) { return m_uavs.at(index); }
    [[nodiscard]] const std::vector<UAVState>& getAllUAVs() const noexcept { return m_uavs; }
    [[nodiscard]] std::vector<UAVState>& getAllUAVs() noexcept { return m_uavs; }

    [[nodiscard]] const UAVContext& getContext(UAVId uavId) const;
    [[nodiscard]] UAVContext& getContext(UAVId uavId);

    // Operational statistics
    [[nodiscard]] size_t countInMode(UAVMode mode) const noexcept;
    [[nodiscard]] size_t getIdleCount() const noexcept { return countInMode(UAVMode::IDLE); }
    [[nodiscard]] size_t getChargingCount() const noexcept { return countInMode(UAVMode::CHARGING); }
    [[nodiscard]] size_t getEmergencyCount() const noexcept { return countInMode(UAVMode::EMERGENCY); }

    [[nodiscard]] const BatteryModel& getBatteryModel() const noexcept { return m_batteryModel; }
    [[nodiscard]] const FleetSettings& getSettings() const noexcept { return m_settings; }

private:
    FleetSettings m_settings;
    EnergySettings m_energySettings;
    BatteryModel m_batteryModel;
    std::vector<UAVState> m_uavs;
    std::unordered_map<UAVId, UAVContext> m_contexts;

    void handleAssigned(UAVState& uav, UAVContext& ctx);
    void handleFlyingToPickup(UAVState& uav, UAVContext& ctx, double dt);
    void handlePickingUp(UAVState& uav, UAVContext& ctx, double dt, double currentTime);
    void handleFlyingToDestination(UAVState& uav, UAVContext& ctx, double dt);
    void handleDroppingOff(UAVState& uav, UAVContext& ctx, double dt, double currentTime, std::vector<DeliveryRequest>& outDelivered);
    void handleReturningToCharge(UAVState& uav, UAVContext& ctx, CityMap& map);
    void handleCharging(UAVState& uav, UAVContext& ctx, CityMap& map, double dt);
};

} // namespace aerofleet

