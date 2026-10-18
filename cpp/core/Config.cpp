#include "core/Config.hpp"
#include <fstream>
#include <iomanip>

namespace aerofleet {

SimulationConfig SimulationConfig::createDefault() {
    return SimulationConfig{};
}

SimulationConfig SimulationConfig::fromJson(const JsonValue& root) {
    SimulationConfig config;

    if (root.contains("simulation")) {
        const auto& s = root["simulation"];
        config.simulation.name = s.getOrDefault<std::string>("name", config.simulation.name);
        config.simulation.version = s.getOrDefault<std::string>("version", config.simulation.version);
        config.simulation.randomSeed = static_cast<uint64_t>(s.getOrDefault<int64_t>("random_seed", static_cast<int64_t>(config.simulation.randomSeed)));
        config.simulation.dtPhysics = s.getOrDefault<double>("dt_physics", config.simulation.dtPhysics);
        config.simulation.dtPolicy = s.getOrDefault<double>("dt_policy", config.simulation.dtPolicy);
        config.simulation.dtCoordinator = s.getOrDefault<double>("dt_coordinator", config.simulation.dtCoordinator);
        config.simulation.maxSimulationTime = s.getOrDefault<double>("max_simulation_time", config.simulation.maxSimulationTime);
    }

    if (root.contains("world")) {
        const auto& w = root["world"];
        config.world.xMin = w.getOrDefault<double>("x_min", config.world.xMin);
        config.world.xMax = w.getOrDefault<double>("x_max", config.world.xMax);
        config.world.yMin = w.getOrDefault<double>("y_min", config.world.yMin);
        config.world.yMax = w.getOrDefault<double>("y_max", config.world.yMax);
        config.world.zMin = w.getOrDefault<double>("z_min", config.world.zMin);
        config.world.zMax = w.getOrDefault<double>("z_max", config.world.zMax);
        config.world.defaultSpawnAltitude = w.getOrDefault<double>("default_spawn_altitude", config.world.defaultSpawnAltitude);
        config.world.safetyMargin = w.getOrDefault<double>("safety_margin", config.world.safetyMargin);
    }

    if (root.contains("fleet")) {
        const auto& f = root["fleet"];
        config.fleet.numUavs = static_cast<int32_t>(f.getOrDefault<int64_t>("num_uavs", config.fleet.numUavs));
        config.fleet.uavRadius = f.getOrDefault<double>("uav_radius", config.fleet.uavRadius);
        config.fleet.maxHorizontalSpeed = f.getOrDefault<double>("max_horizontal_speed", config.fleet.maxHorizontalSpeed);
        config.fleet.maxVerticalSpeed = f.getOrDefault<double>("max_vertical_speed", config.fleet.maxVerticalSpeed);
        config.fleet.maxAcceleration = f.getOrDefault<double>("max_acceleration", config.fleet.maxAcceleration);
        config.fleet.batteryCapacityMah = f.getOrDefault<double>("battery_capacity_mah", config.fleet.batteryCapacityMah);
        config.fleet.nominalVoltage = f.getOrDefault<double>("nominal_voltage", config.fleet.nominalVoltage);
        config.fleet.reserveBatteryPercent = f.getOrDefault<double>("reserve_battery_percent", config.fleet.reserveBatteryPercent);
        config.fleet.criticalBatteryPercent = f.getOrDefault<double>("critical_battery_percent", config.fleet.criticalBatteryPercent);
        config.fleet.dockingRadius = f.getOrDefault<double>("docking_radius", config.fleet.dockingRadius);
        config.fleet.chargeRatePercentPerSec = f.getOrDefault<double>("charge_rate_percent_per_sec", config.fleet.chargeRatePercentPerSec);
        config.fleet.pickupDwellSeconds = f.getOrDefault<double>("pickup_dwell_seconds", config.fleet.pickupDwellSeconds);
        config.fleet.dropoffDwellSeconds = f.getOrDefault<double>("dropoff_dwell_seconds", config.fleet.dropoffDwellSeconds);
    }

    if (root.contains("requests")) {
        const auto& r = root["requests"];
        config.requests.arrivalRatePerMin = r.getOrDefault<double>("arrival_rate_per_min", config.requests.arrivalRatePerMin);
        config.requests.minPayloadKg = r.getOrDefault<double>("min_payload_kg", config.requests.minPayloadKg);
        config.requests.maxPayloadKg = r.getOrDefault<double>("max_payload_kg", config.requests.maxPayloadKg);
        config.requests.slackFactor = r.getOrDefault<double>("slack_factor", config.requests.slackFactor);
        config.requests.bufferTimeSeconds = r.getOrDefault<double>("buffer_time_seconds", config.requests.bufferTimeSeconds);
        config.requests.requestTimeoutSeconds = r.getOrDefault<double>("request_timeout_seconds", config.requests.requestTimeoutSeconds);
    }

    if (root.contains("energy")) {
        const auto& e = root["energy"];
        config.energy.idlePowerWatts = e.getOrDefault<double>("idle_power_watts", config.energy.idlePowerWatts);
        config.energy.dragCoefficient = e.getOrDefault<double>("drag_coefficient", config.energy.dragCoefficient);
        config.energy.liftCoefficient = e.getOrDefault<double>("lift_coefficient", config.energy.liftCoefficient);
        config.energy.climbPenaltyCoefficient = e.getOrDefault<double>("climb_penalty_coefficient", config.energy.climbPenaltyCoefficient);
    }

    validate(config);
    return config;
}

SimulationConfig SimulationConfig::loadFromFile(const std::string& filepath) {
    JsonValue root = JsonValue::fromFile(filepath);
    return fromJson(root);
}

void SimulationConfig::validate(const SimulationConfig& config) {
    // 1. Simulation clock validations
    if (config.simulation.dtPhysics <= 0.0 || config.simulation.dtPhysics > 0.1) {
        throw ConfigValidationException("dtPhysics must be strictly positive and <= 0.1s for numerical stability, got: " +
            std::to_string(config.simulation.dtPhysics));
    }
    if (config.simulation.dtPolicy < config.simulation.dtPhysics) {
        throw ConfigValidationException("dtPolicy (" + std::to_string(config.simulation.dtPolicy) +
            ") cannot be smaller than dtPhysics (" + std::to_string(config.simulation.dtPhysics) + ")");
    }
    if (config.simulation.dtCoordinator < config.simulation.dtPolicy) {
        throw ConfigValidationException("dtCoordinator (" + std::to_string(config.simulation.dtCoordinator) +
            ") cannot be smaller than dtPolicy (" + std::to_string(config.simulation.dtPolicy) + ")");
    }
    if (config.simulation.maxSimulationTime <= 0.0) {
        throw ConfigValidationException("maxSimulationTime must be strictly positive");
    }

    // 2. World dimensions validations
    if (config.world.xMin >= config.world.xMax) {
        throw ConfigValidationException("world.xMin must be strictly less than world.xMax");
    }
    if (config.world.yMin >= config.world.yMax) {
        throw ConfigValidationException("world.yMin must be strictly less than world.yMax");
    }
    if (config.world.zMin >= config.world.zMax) {
        throw ConfigValidationException("world.zMin must be strictly less than world.zMax");
    }
    if (config.world.zMin < 0.0) {
        throw ConfigValidationException("world.zMin cannot be below ground level (0.0m)");
    }
    if (config.world.safetyMargin < 0.0) {
        throw ConfigValidationException("world.safetyMargin cannot be negative");
    }

    // 3. Fleet limits validations
    if (config.fleet.numUavs <= 0) {
        throw ConfigValidationException("fleet.numUavs must be at least 1");
    }
    if (config.fleet.uavRadius <= 0.0) {
        throw ConfigValidationException("fleet.uavRadius must be strictly positive");
    }
    if (config.fleet.maxHorizontalSpeed <= 0.0 || config.fleet.maxVerticalSpeed <= 0.0) {
        throw ConfigValidationException("UAV flight speeds must be strictly positive");
    }
    if (config.fleet.maxAcceleration <= 0.0) {
        throw ConfigValidationException("fleet.maxAcceleration must be strictly positive");
    }
    if (config.fleet.batteryCapacityMah <= 0.0) {
        throw ConfigValidationException("fleet.batteryCapacityMah must be strictly positive");
    }
    if (config.fleet.criticalBatteryPercent < 0.0 || config.fleet.criticalBatteryPercent >= 100.0) {
        throw ConfigValidationException("criticalBatteryPercent must be in [0.0, 100.0)");
    }
    if (config.fleet.reserveBatteryPercent <= config.fleet.criticalBatteryPercent) {
        throw ConfigValidationException("reserveBatteryPercent must be strictly greater than criticalBatteryPercent");
    }
    if (config.fleet.dockingRadius < config.fleet.uavRadius) {
        throw ConfigValidationException("dockingRadius cannot be smaller than uavRadius");
    }

    // 4. Request generation validations
    if (config.requests.arrivalRatePerMin <= 0.0) {
        throw ConfigValidationException("requests.arrivalRatePerMin must be strictly positive");
    }
    if (config.requests.minPayloadKg <= 0.0 || config.requests.maxPayloadKg < config.requests.minPayloadKg) {
        throw ConfigValidationException("Invalid request payload limits (min > 0, max >= min)");
    }
    if (config.requests.slackFactor < 1.0) {
        throw ConfigValidationException("requests.slackFactor must be at least 1.0");
    }

    // 5. Energy parameters validations
    if (config.energy.idlePowerWatts < 0.0 || config.energy.dragCoefficient < 0.0) {
        throw ConfigValidationException("Energy power and drag coefficients cannot be negative");
    }
}

void SimulationConfig::saveToFile(const std::string& filepath) const {
    std::ofstream out(filepath);
    if (!out.is_open()) {
        throw std::runtime_error("Failed to open file for writing config: " + filepath);
    }
    out << "{\n"
        << "  \"simulation\": {\n"
        << "    \"name\": \"" << simulation.name << "\",\n"
        << "    \"version\": \"" << simulation.version << "\",\n"
        << "    \"random_seed\": " << simulation.randomSeed << ",\n"
        << "    \"dt_physics\": " << simulation.dtPhysics << ",\n"
        << "    \"dt_policy\": " << simulation.dtPolicy << ",\n"
        << "    \"dt_coordinator\": " << simulation.dtCoordinator << ",\n"
        << "    \"max_simulation_time\": " << simulation.maxSimulationTime << "\n"
        << "  },\n"
        << "  \"world\": {\n"
        << "    \"x_min\": " << world.xMin << ",\n"
        << "    \"x_max\": " << world.xMax << ",\n"
        << "    \"y_min\": " << world.yMin << ",\n"
        << "    \"y_max\": " << world.yMax << ",\n"
        << "    \"z_min\": " << world.zMin << ",\n"
        << "    \"z_max\": " << world.zMax << ",\n"
        << "    \"default_spawn_altitude\": " << world.defaultSpawnAltitude << ",\n"
        << "    \"safety_margin\": " << world.safetyMargin << "\n"
        << "  },\n"
        << "  \"fleet\": {\n"
        << "    \"num_uavs\": " << fleet.numUavs << ",\n"
        << "    \"uav_radius\": " << fleet.uavRadius << ",\n"
        << "    \"max_horizontal_speed\": " << fleet.maxHorizontalSpeed << ",\n"
        << "    \"max_vertical_speed\": " << fleet.maxVerticalSpeed << ",\n"
        << "    \"max_acceleration\": " << fleet.maxAcceleration << ",\n"
        << "    \"battery_capacity_mah\": " << fleet.batteryCapacityMah << ",\n"
        << "    \"nominal_voltage\": " << fleet.nominalVoltage << ",\n"
        << "    \"reserve_battery_percent\": " << fleet.reserveBatteryPercent << ",\n"
        << "    \"critical_battery_percent\": " << fleet.criticalBatteryPercent << ",\n"
        << "    \"docking_radius\": " << fleet.dockingRadius << ",\n"
        << "    \"charge_rate_percent_per_sec\": " << fleet.chargeRatePercentPerSec << ",\n"
        << "    \"pickup_dwell_seconds\": " << fleet.pickupDwellSeconds << ",\n"
        << "    \"dropoff_dwell_seconds\": " << fleet.dropoffDwellSeconds << "\n"
        << "  },\n"
        << "  \"requests\": {\n"
        << "    \"arrival_rate_per_min\": " << requests.arrivalRatePerMin << ",\n"
        << "    \"min_payload_kg\": " << requests.minPayloadKg << ",\n"
        << "    \"max_payload_kg\": " << requests.maxPayloadKg << ",\n"
        << "    \"slack_factor\": " << requests.slackFactor << ",\n"
        << "    \"buffer_time_seconds\": " << requests.bufferTimeSeconds << ",\n"
        << "    \"request_timeout_seconds\": " << requests.requestTimeoutSeconds << "\n"
        << "  },\n"
        << "  \"energy\": {\n"
        << "    \"idle_power_watts\": " << energy.idlePowerWatts << ",\n"
        << "    \"drag_coefficient\": " << energy.dragCoefficient << ",\n"
        << "    \"lift_coefficient\": " << energy.liftCoefficient << ",\n"
        << "    \"climb_penalty_coefficient\": " << energy.climbPenaltyCoefficient << "\n"
        << "  }\n"
        << "}\n";
}

} // namespace aerofleet

