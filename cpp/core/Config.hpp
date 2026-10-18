#pragma once

#include <string>
#include <cstdint>
#include <stdexcept>
#include "core/Json.hpp"
#include "math/Vec3.hpp"

namespace aerofleet {

class ConfigValidationException : public std::runtime_error {
public:
    explicit ConfigValidationException(const std::string& message) : std::runtime_error(message) {}
};

struct SimulationSettings {
    std::string name{"AeroFleet Simulation"};
    std::string version{"1.0.0"};
    uint64_t randomSeed{42};
    double dtPhysics{0.02};       ///< 50 Hz physics integration step
    double dtPolicy{0.10};        ///< 10 Hz UAV navigation policy step
    double dtCoordinator{1.00};   ///< 1 Hz Central coordinator dispatch step
    double maxSimulationTime{3600.0}; ///< Max simulation duration in seconds
};

struct WorldSettings {
    double xMin{-500.0};
    double xMax{500.0};
    double yMin{-500.0};
    double yMax{500.0};
    double zMin{0.0};
    double zMax{150.0};
    double defaultSpawnAltitude{15.0};
    double safetyMargin{2.0};

    [[nodiscard]] Vec3 getDimensions() const noexcept {
        return Vec3(xMax - xMin, yMax - yMin, zMax - zMin);
    }

    [[nodiscard]] bool contains(const Vec3& pt, double margin = 0.0) const noexcept {
        return pt.x >= (xMin + margin) && pt.x <= (xMax - margin) &&
               pt.y >= (yMin + margin) && pt.y <= (yMax - margin) &&
               pt.z >= (zMin + margin) && pt.z <= (zMax - margin);
    }
};

struct FleetSettings {
    int32_t numUavs{10};
    double uavRadius{0.5};                  ///< UAV bounding sphere radius in meters
    double maxHorizontalSpeed{15.0};        ///< m/s
    double maxVerticalSpeed{5.0};           ///< m/s
    double maxAcceleration{6.0};            ///< m/s^2
    double batteryCapacityMah{5000.0};      ///< mAh
    double nominalVoltage{22.2};            ///< Volts (6S LiPo)
    double reserveBatteryPercent{20.0};     ///< Return-to-home trigger (%)
    double criticalBatteryPercent{10.0};    ///< Emergency landing threshold (%)
    double dockingRadius{1.5};              ///< Docking capture sphere radius in meters
    double chargeRatePercentPerSec{1.0};    ///< Battery replenishment rate (%/sec)
    double pickupDwellSeconds{3.0};         ///< Payload pickup dwell time
    double dropoffDwellSeconds{3.0};        ///< Payload dropoff dwell time
};

struct RequestSettings {
    double arrivalRatePerMin{12.0};         ///< Poisson arrival rate (lambda)
    double minPayloadKg{0.2};               ///< Minimum package weight (kg)
    double maxPayloadKg{3.0};               ///< Maximum package weight (kg)
    double slackFactor{2.0};                ///< Multiplier for deadline estimation
    double bufferTimeSeconds{30.0};         ///< Additional delivery buffer time
    double requestTimeoutSeconds{300.0};    ///< Expiration limit if unassigned
};

struct EnergySettings {
    double idlePowerWatts{40.0};            ///< Baseline avionics/hover power
    double dragCoefficient{0.15};           ///< Velocity aerodynamic drag penalty
    double liftCoefficient{9.81};           ///< Gravitational payload lift coefficient
    double climbPenaltyCoefficient{25.0};   ///< Upward vertical climb power penalty
};

struct SimulationConfig {
    SimulationSettings simulation;
    WorldSettings world;
    FleetSettings fleet;
    RequestSettings requests;
    EnergySettings energy;

    static SimulationConfig loadFromFile(const std::string& filepath);
    static SimulationConfig fromJson(const JsonValue& json);
    static SimulationConfig createDefault();
    static void validate(const SimulationConfig& config);
    void saveToFile(const std::string& filepath) const;
};

} // namespace aerofleet

