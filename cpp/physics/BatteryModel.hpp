#pragma once

#include <cmath>
#include <algorithm>
#include <cstdint>
#include "core/Config.hpp"
#include "core/Types.hpp"
#include "fleet/UAVState.hpp"
#include "world/CityMap.hpp"

namespace aerofleet {

/**
 * @brief Configuration parameters governing UAV battery capacity and aerodynamic power consumption.
 */
struct BatteryConfig {
    double capacityMah{5000.0};             ///< Total battery capacity in milliamp-hours
    double nominalVoltage{22.2};            ///< Battery nominal voltage (e.g. 22.2V for 6S LiPo)
    double droneMassKg{1.5};                ///< Dry mass of the UAV airframe in kg
    double reservePercent{20.0};            ///< Return-to-base battery threshold (%)
    double criticalPercent{10.0};           ///< Emergency landing battery threshold (%)
    double chargeRatePercentPerSec{1.0};    ///< Battery recharge replenishment rate (%/sec)
    double dockingRadius{1.5};              ///< Capture radius of charging docks in meters
    double maxDockingSpeed{0.5};            ///< Maximum permissible speed to latch onto a charging dock (m/s)

    // Aerodynamic energy parameters
    double idlePowerWatts{40.0};            ///< Baseline avionics and hover motor draw in Watts
    double dragCoefficient{0.15};           ///< Aerodynamic drag power penalty coefficient (~ ||v||^3)
    double liftCoefficient{9.81};           ///< Gravitational payload lift coefficient
    double climbPenaltyCoefficient{25.0};   ///< Upward vertical climb power penalty coefficient
    double gravity{9.81};                   ///< Standard gravitational acceleration (m/s^2)
    double dischargeMultiplier{1.0};        ///< Accelerated discharge scaling factor

    BatteryConfig() = default;

    BatteryConfig(const FleetSettings& fleet, const EnergySettings& energy, double dryMass = 1.5)
        : capacityMah(fleet.batteryCapacityMah),
          nominalVoltage(fleet.nominalVoltage),
          droneMassKg(dryMass),
          reservePercent(fleet.reserveBatteryPercent),
          criticalPercent(fleet.criticalBatteryPercent),
          chargeRatePercentPerSec(fleet.chargeRatePercentPerSec),
          dockingRadius(fleet.dockingRadius),
          maxDockingSpeed(0.5),
          idlePowerWatts(energy.idlePowerWatts),
          dragCoefficient(energy.dragCoefficient),
          liftCoefficient(energy.liftCoefficient),
          climbPenaltyCoefficient(energy.climbPenaltyCoefficient),
          gravity(9.81),
          dischargeMultiplier(energy.dischargeMultiplier) {}

    /**
     * @brief Total battery stored energy capacity in Joules (Watt-seconds).
     * Energy (Joules) = (capacityMah / 1000) * nominalVoltage * 3600
     */
    [[nodiscard]] double totalCapacityJoules() const noexcept {
        return (capacityMah / 1000.0) * nominalVoltage * 3600.0;
    }
};

/**
 * @brief High-fidelity electrical battery model and charging station mechanics.
 */
class BatteryModel {
public:
    BatteryModel() = default;
    explicit BatteryModel(const BatteryConfig& config);
    BatteryModel(const FleetSettings& fleet, const EnergySettings& energy, double dryMass = 1.5);

    /**
     * @brief Computes instantaneous electrical power consumption in Watts.
     * P = P_idle + c_drag * ||v||^3 + c_lift * (m_drone + m_payload) + c_climb * max(0, v_z)
     * @param velocity 3D linear velocity vector of the UAV (m/s)
     * @param payloadMass Carried cargo weight in kg
     * @return Instantaneous power draw in Watts
     */
    [[nodiscard]] double computePowerWatts(
        const Vec3& velocity,
        double payloadMass = 0.0
    ) const noexcept;

    /**
     * @brief Integrates battery discharge over a time step dt based on physical power draw.
     * Decrements batteryPercent and clamps strictly within [0.0, 100.0]%.
     * If battery drops to 0.0%, zeroes velocity/acceleration and shifts UAV to EMERGENCY mode.
     * @param uav UAV state to update
     * @param dt Time step in seconds
     * @return Electrical energy consumed during this step in Joules
     */
    double stepDischarge(UAVState& uav, double dt) const;

    /**
     * @brief Integrates battery charging replenishment for a docked UAV over time step dt.
     * Increments batteryPercent by chargeRate * dt, strictly clamping to 100.0%.
     * Only applies if UAV mode is CHARGING.
     * @param uav UAV state to recharge
     * @param dt Time step in seconds
     * @return Percentage points of charge added
     */
    double stepCharge(UAVState& uav, double dt) const;

    /**
     * @brief Attempts to dock a UAV at a charging dock.
     * Requires UAV to be within dockingRadius, traveling below maxDockingSpeed,
     * and the station to have an available slot.
     * On success, increments station.occupiedCount, sets uav.mode = CHARGING,
     * sets uav.isGrounded = true, and stops velocity/acceleration.
     * @return true if docked successfully, false otherwise
     */
    bool dock(UAVState& uav, ChargingStation& station) const;

    /**
     * @brief Undocks a UAV from a charging station, freeing the occupied dock slot.
     * Transitions UAV mode from CHARGING to IDLE.
     * @return true if successfully undocked, false otherwise
     */
    bool undock(UAVState& uav, ChargingStation& station) const;

    /**
     * @brief Proximity test checking if UAV is within docking capture sphere of the station.
     */
    [[nodiscard]] bool isInDockingRange(const UAVState& uav, const ChargingStation& station) const noexcept;

    /**
     * @brief State of charge classification helpers.
     */
    [[nodiscard]] bool isReserve(double batteryPercent) const noexcept {
        return batteryPercent <= m_config.reservePercent;
    }

    [[nodiscard]] bool isCritical(double batteryPercent) const noexcept {
        return batteryPercent <= m_config.criticalPercent;
    }

    [[nodiscard]] bool isDepleted(double batteryPercent) const noexcept {
        return batteryPercent <= 0.0;
    }

    [[nodiscard]] const BatteryConfig& getConfig() const noexcept { return m_config; }
    void setDischargeMultiplier(double mult) noexcept { m_config.dischargeMultiplier = std::max(0.0, mult); }
    [[nodiscard]] double getDischargeMultiplier() const noexcept { return m_config.dischargeMultiplier; }

private:
    BatteryConfig m_config;
};

} // namespace aerofleet

