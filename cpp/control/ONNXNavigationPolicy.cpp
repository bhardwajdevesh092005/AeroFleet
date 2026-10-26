#include "control/ONNXNavigationPolicy.hpp"

#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>

#include "core/Json.hpp"
#include "math/MathUtils.hpp"
#include "math/Geometry.hpp"

namespace aerofleet {

namespace {

Vec3 worldToBody(const Vec3& v, double yaw) {
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    return Vec3(c * v.x + s * v.y, -s * v.x + c * v.y, v.z);
}

} // namespace

ONNXNavigationPolicy::ONNXNavigationPolicy()
    : ONNXNavigationPolicy(Config{}) {}

ONNXNavigationPolicy::ONNXNavigationPolicy(const Config& config)
    : m_config(config),
      m_model(std::make_shared<ONNXModel>(config.modelPath)),
      m_safetyLayer(config.safetyConfig) {
    loadNormalization(config.normalizationPath);
}

ONNXNavigationPolicy::ONNXNavigationPolicy(
    const std::string& modelPath,
    const std::string& normalizationPath,
    const SafetyLayerConfig& safetyConfig
) : ONNXNavigationPolicy([&]() {
        Config cfg;
        cfg.modelPath = modelPath;
        cfg.normalizationPath = normalizationPath;
        cfg.safetyConfig = safetyConfig;
        return cfg;
    }()) {}

void ONNXNavigationPolicy::loadNormalization(const std::string& normPath) {
    m_mean.assign(87, 0.0f);
    m_var.assign(87, 1.0f);
    m_epsilon = 1e-8f;
    m_clipObs = 10.0f;

    try {
        std::ifstream file(normPath);
        if (!file.is_open()) {
            std::cerr << "[WARN] ONNXNavigationPolicy: could not open normalization file: " << normPath << std::endl;
            return;
        }
        std::string content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
        JsonValue root = JsonValue::parse(content);

        if (root.contains("mean") && root["mean"].isArray()) {
            const auto& mArr = root["mean"].asArray();
            for (size_t i = 0; i < std::min<size_t>(mArr.size(), 87); ++i) {
                m_mean[i] = static_cast<float>(mArr[i].asDouble());
            }
        }
        if (root.contains("var") && root["var"].isArray()) {
            const auto& vArr = root["var"].asArray();
            for (size_t i = 0; i < std::min<size_t>(vArr.size(), 87); ++i) {
                m_var[i] = static_cast<float>(vArr[i].asDouble());
            }
        }
        if (root.contains("epsilon")) {
            m_epsilon = static_cast<float>(root["epsilon"].asDouble());
        }
        if (root.contains("clip_obs")) {
            m_clipObs = static_cast<float>(root["clip_obs"].asDouble());
        }
    } catch (const std::exception& e) {
        std::cerr << "[WARN] ONNXNavigationPolicy: failed to parse normalization JSON: " << e.what() << std::endl;
    }
}

std::vector<float> ONNXNavigationPolicy::buildRawObservation(
    const UAVState& uav,
    const CityMap& map,
    const std::vector<UAVState>& traffic
) const {
    std::vector<float> raw;
    raw.reserve(87);

    // 0..2: Ego velocity in body frame
    Vec3 bodyVel = worldToBody(uav.velocity, uav.yaw);
    raw.push_back(static_cast<float>(bodyVel.x));
    raw.push_back(static_cast<float>(bodyVel.y));
    raw.push_back(static_cast<float>(bodyVel.z));

    // 3..5: Body acceleration
    Vec3 bodyAcc = worldToBody(uav.acceleration, uav.yaw);
    raw.push_back(static_cast<float>(bodyAcc.x));
    raw.push_back(static_cast<float>(bodyAcc.y));
    raw.push_back(static_cast<float>(bodyAcc.z));

    // 6..8: Relative target vector in body frame
    Vec3 relTarget = worldToBody(uav.targetWaypoint - uav.position, uav.yaw);
    raw.push_back(static_cast<float>(relTarget.x));
    raw.push_back(static_cast<float>(relTarget.y));
    raw.push_back(static_cast<float>(relTarget.z));

    // 9..12: Scalar states
    raw.push_back(static_cast<float>(relTarget.length()));
    raw.push_back(static_cast<float>(uav.batteryPercent / 100.0));
    raw.push_back(static_cast<float>(uav.payloadMass));
    raw.push_back(static_cast<float>(uav.speed()));

    // 13..56: 44 Lidar beams (36 azimuth, 4 up 30 deg, 4 down 30 deg)
    const double maxRange = 100.0;
    auto candidates = map.broadphase.querySphere(uav.position, maxRange);

    for (int i = 0; i < 36; ++i) {
        double angle = uav.yaw + (i * 2.0 * MathUtils::PI / 36.0);
        Vec3 dir(std::cos(angle), std::sin(angle), 0.0);
        Ray ray(uav.position, dir);

        double minHitDist = maxRange;
        for (int32_t bIdx : candidates) {
            if (bIdx >= 0 && static_cast<size_t>(bIdx) < map.buildings.size()) {
                auto hit = raycastAABB(ray, map.buildings[bIdx].bounds, minHitDist);
                if (hit.hit && hit.distance < minHitDist) {
                    minHitDist = hit.distance;
                }
            }
        }
        raw.push_back(static_cast<float>(minHitDist / maxRange));
    }

    const double pitchUp = MathUtils::PI / 6.0;
    const double cosPitch = std::cos(pitchUp);
    const double sinPitchUp = std::sin(pitchUp);
    const double sinPitchDown = -std::sin(pitchUp);

    for (int i = 0; i < 4; ++i) {
        double angle = uav.yaw + (i * MathUtils::PI / 2.0);
        Vec3 dir(cosPitch * std::cos(angle), cosPitch * std::sin(angle), sinPitchUp);
        Ray ray(uav.position, dir);

        double minHitDist = maxRange;
        for (int32_t bIdx : candidates) {
            if (bIdx >= 0 && static_cast<size_t>(bIdx) < map.buildings.size()) {
                auto hit = raycastAABB(ray, map.buildings[bIdx].bounds, minHitDist);
                if (hit.hit && hit.distance < minHitDist) {
                    minHitDist = hit.distance;
                }
            }
        }
        raw.push_back(static_cast<float>(minHitDist / maxRange));
    }

    for (int i = 0; i < 4; ++i) {
        double angle = uav.yaw + (i * MathUtils::PI / 2.0);
        Vec3 dir(cosPitch * std::cos(angle), cosPitch * std::sin(angle), sinPitchDown);
        Ray ray(uav.position, dir);

        double minHitDist = maxRange;
        for (int32_t bIdx : candidates) {
            if (bIdx >= 0 && static_cast<size_t>(bIdx) < map.buildings.size()) {
                auto hit = raycastAABB(ray, map.buildings[bIdx].bounds, minHitDist);
                if (hit.hit && hit.distance < minHitDist) {
                    minHitDist = hit.distance;
                }
            }
        }
        raw.push_back(static_cast<float>(minHitDist / maxRange));
    }

    // 57..74: Nearby UAVs (3 * 6 = 18 values)
    struct NeighborDist {
        double distSq;
        const UAVState* uav;
    };
    std::vector<NeighborDist> neighbors;
    neighbors.reserve(traffic.size());
    for (const auto& other : traffic) {
        if (other.id != uav.id && other.mode != UAVMode::OFFLINE) {
            neighbors.push_back({uav.position.distanceSquared(other.position), &other});
        }
    }
    std::sort(neighbors.begin(), neighbors.end(), [](const NeighborDist& a, const NeighborDist& b) {
        return a.distSq < b.distSq;
    });

    for (size_t i = 0; i < 3; ++i) {
        if (i < neighbors.size()) {
            const auto* other = neighbors[i].uav;
            Vec3 relPos = worldToBody(other->position - uav.position, uav.yaw);
            Vec3 relVel = worldToBody(other->velocity - uav.velocity, uav.yaw);
            raw.push_back(static_cast<float>(relPos.x));
            raw.push_back(static_cast<float>(relPos.y));
            raw.push_back(static_cast<float>(relPos.z));
            raw.push_back(static_cast<float>(relVel.x));
            raw.push_back(static_cast<float>(relVel.y));
            raw.push_back(static_cast<float>(relVel.z));
        } else {
            for (int k = 0; k < 6; ++k) raw.push_back(0.0f);
        }
    }

    // 75..76: Heading sine and cosine
    raw.push_back(static_cast<float>(std::sin(uav.yaw)));
    raw.push_back(static_cast<float>(std::cos(uav.yaw)));

    // 77..86: UAV mode one-hot (10 values)
    const auto modeIdx = static_cast<size_t>(uav.mode);
    for (size_t m = 0; m < 10; ++m) {
        raw.push_back(m == modeIdx ? 1.0f : 0.0f);
    }

    return raw;
}

std::vector<float> ONNXNavigationPolicy::buildNormalizedObservation(
    const UAVState& uav,
    const CityMap& map,
    const std::vector<UAVState>& traffic
) const {
    const auto raw = buildRawObservation(uav, map, traffic);

    const double extentX = map.worldMax.x - map.worldMin.x;
    const double extentY = map.worldMax.y - map.worldMin.y;
    const double extentZ = map.worldMax.z - map.worldMin.z;
    const double horizDiag = std::hypot(extentX, extentY);
    const double maxDist = std::sqrt(extentX * extentX + extentY * extentY + extentZ * extentZ);

    std::vector<double> scales(87, 1.0);
    scales[0] = m_config.maxHorizontalSpeed;
    scales[1] = m_config.maxHorizontalSpeed;
    scales[2] = m_config.maxVerticalSpeed;
    scales[3] = m_config.maxAcceleration;
    scales[4] = m_config.maxAcceleration;
    scales[5] = m_config.maxAcceleration;
    scales[6] = horizDiag;
    scales[7] = horizDiag;
    scales[8] = extentZ;
    scales[9] = maxDist;
    scales[11] = std::max(m_config.maxPayloadMass, 1e-6);
    scales[12] = m_config.maxObservationSpeed;

    const double maxRelH = 2.0 * m_config.maxHorizontalSpeed;
    const double maxRelV = 2.0 * m_config.maxVerticalSpeed;
    for (size_t k = 0; k < 3; ++k) {
        size_t base = 57 + k * 6;
        scales[base + 0] = horizDiag;
        scales[base + 1] = horizDiag;
        scales[base + 2] = extentZ;
        scales[base + 3] = maxRelH;
        scales[base + 4] = maxRelH;
        scales[base + 5] = maxRelV;
    }

    std::vector<float> normalized(87);
    for (size_t i = 0; i < 87; ++i) {
        float scaled = static_cast<float>(raw[i] / scales[i]);
        float centered = (scaled - m_mean[i]) / std::sqrt(m_var[i] + m_epsilon);
        normalized[i] = std::clamp(centered, -m_clipObs, m_clipObs);
    }

    return normalized;
}

VelocityCommand ONNXNavigationPolicy::computeAction(
    const UAVState& uav,
    const CityMap& map,
    double dt
) const {
    return computeActionWithTraffic(uav, map, {}, dt);
}

VelocityCommand ONNXNavigationPolicy::computeActionWithTraffic(
    const UAVState& uav,
    const CityMap& map,
    const std::vector<UAVState>& traffic,
    double dt
) const {
    const auto normObs = buildNormalizedObservation(uav, map, traffic);
    const auto action = m_model->run(normObs);

    const float a0 = std::clamp(action[0], -1.0f, 1.0f);
    const float a1 = std::clamp(action[1], -1.0f, 1.0f);
    const float a2 = std::clamp(action[2], -1.0f, 1.0f);

    // Convert from normalized body action to physical body velocity
    const double forward = a0 * m_config.maxHorizontalSpeed;
    const double left = a1 * m_config.maxHorizontalSpeed;
    const double up = a2 * m_config.maxVerticalSpeed;

    // Rotate body velocity to world coordinates
    const double cosYaw = std::cos(uav.yaw);
    const double sinYaw = std::sin(uav.yaw);
    double vx = cosYaw * forward - sinYaw * left;
    double vy = sinYaw * forward + cosYaw * left;
    double vz = up;

    // Enforce pure vertical takeoff corridor to safely clear ground charging station
    if (uav.position.z < 10.0 && uav.targetWaypoint.z > uav.position.z) {
        vx = 0.0;
        vy = 0.0;
        vz = m_config.maxVerticalSpeed;
    }

    const VelocityCommand candidate{Vec3(vx, vy, vz)};
    return m_safetyLayer.filterAction(uav, map, candidate, dt);
}

} // namespace aerofleet
