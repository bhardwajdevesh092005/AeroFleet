#include "physics/BatteryModel.hpp"
#include <algorithm>
#include <cmath>

namespace aerofleet {

BatteryModel::BatteryModel(const BatteryConfig& config)
    : m_config(config) {}

BatteryModel::BatteryModel(const FleetSettings& fleet, const EnergySettings& energy, double dryMass)
    : m_config(fleet, energy, dryMass) {}

double BatteryModel::computePowerWatts(const Vec3& velocity, double payloadMass) const noexcept {
    const double speed = velocity.length();
    const double vz = velocity.z;
    const double climbSpeed = std::max(0.0, vz);

    // 1. Idle avionics, onboard computing & baseline motor draw
    const double pIdle = m_config.idlePowerWatts;

    // 2. Aerodynamic drag power scaling with airspeed cubed
    const double pDrag = m_config.dragCoefficient * speed * speed * speed;

    // 3. Gravitational lift hover power scaling with total weight
    const double effectiveMass = std::max(0.0, m_config.droneMassKg + payloadMass);
    const double pLift = m_config.liftCoefficient * effectiveMass;

    // 4. Vertical climb penalty for positive vertical velocity
    const double pClimb = m_config.climbPenaltyCoefficient * climbSpeed;

    return pIdle + pDrag + pLift + pClimb;
}

double BatteryModel::stepDischarge(UAVState& uav, double dt) const {
    if (dt <= 0.0 || uav.batteryPercent <= 0.0) {
        if (uav.batteryPercent <= 0.0) {
            uav.batteryPercent = 0.0;
            uav.velocity = Vec3(0.0, 0.0, 0.0);
            uav.acceleration = Vec3(0.0, 0.0, 0.0);
            uav.mode = UAVMode::EMERGENCY;
        }
        return 0.0;
    }

    const double powerWatts = computePowerWatts(uav.velocity, uav.payloadMass);
    const double energyJoules = powerWatts * dt;
    const double capacityJoules = m_config.totalCapacityJoules();

    double deltaPercent = 0.0;
    if (capacityJoules > 1.0) {
        deltaPercent = (energyJoules / capacityJoules) * 100.0;
    }

    uav.batteryPercent = std::max(0.0, uav.batteryPercent - deltaPercent);

    if (uav.batteryPercent <= 0.0) {
        uav.batteryPercent = 0.0;
        uav.velocity = Vec3(0.0, 0.0, 0.0);
        uav.acceleration = Vec3(0.0, 0.0, 0.0);
        uav.mode = UAVMode::EMERGENCY;
    }

    return energyJoules;
}

double BatteryModel::stepCharge(UAVState& uav, double dt) const {
    if (dt <= 0.0 || uav.mode != UAVMode::CHARGING) {
        return 0.0;
    }

    const double addedPercent = m_config.chargeRatePercentPerSec * dt;
    const double prevPercent = uav.batteryPercent;
    uav.batteryPercent = std::min(100.0, uav.batteryPercent + addedPercent);
    return uav.batteryPercent - prevPercent;
}

bool BatteryModel::dock(UAVState& uav, ChargingStation& station) const {
    if (!isInDockingRange(uav, station)) {
        return false;
    }

    if (uav.speed() > m_config.maxDockingSpeed) {
        return false;
    }

    if (!station.hasAvailableSlot()) {
        return false;
    }

    station.occupiedCount++;
    uav.mode = UAVMode::CHARGING;
    uav.isGrounded = true;
    uav.velocity = Vec3(0.0, 0.0, 0.0);
    uav.acceleration = Vec3(0.0, 0.0, 0.0);
    return true;
}

bool BatteryModel::undock(UAVState& uav, ChargingStation& station) const {
    if (station.occupiedCount > 0) {
        station.occupiedCount--;
    }
    uav.mode = UAVMode::IDLE;
    return true;
}

bool BatteryModel::isInDockingRange(const UAVState& uav, const ChargingStation& station) const noexcept {
    return uav.position.distanceSquared(station.position) <= (m_config.dockingRadius * m_config.dockingRadius);
}

} // namespace aerofleet

