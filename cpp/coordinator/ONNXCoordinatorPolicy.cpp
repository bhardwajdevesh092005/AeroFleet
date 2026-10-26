#include "coordinator/ONNXCoordinatorPolicy.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>

namespace aerofleet {

ONNXCoordinatorPolicy::ONNXCoordinatorPolicy()
    : ONNXCoordinatorPolicy(CoordinatorPolicyConfig{}) {}

ONNXCoordinatorPolicy::ONNXCoordinatorPolicy(const CoordinatorPolicyConfig& config)
    : m_config(config) {
    if (!m_config.modelPath.empty() && std::filesystem::exists(m_config.modelPath)) {
        try {
            m_model = std::make_unique<ONNXModel>(m_config.modelPath, "AeroFleetCoordinatorInference", 1);
            std::cout << "[ONNXCoordinator] Successfully loaded model from '" << m_config.modelPath << "'\n";
        } catch (const std::exception& e) {
            std::cerr << "[ONNXCoordinator] Warning: Failed to load ONNX model ('"
                      << m_config.modelPath << "'): " << e.what()
                      << ". Using HeuristicCoordinator fallback.\n";
            m_model = nullptr;
        }
    }
}

ONNXCoordinatorPolicy::ONNXCoordinatorPolicy(const std::string& modelPath)
    : ONNXCoordinatorPolicy([&modelPath]() {
          CoordinatorPolicyConfig cfg;
          cfg.modelPath = modelPath;
          return cfg;
      }()) {}

std::vector<float> ONNXCoordinatorPolicy::buildObservation(
    const FleetManager& fleet,
    const std::vector<DeliveryRequest>& requests,
    const CityMap& map,
    double currentTime
) const {
    constexpr size_t FLEET_SIZE = 10;
    constexpr size_t MAX_OBSERVED_REQUESTS = 10;
    constexpr size_t OBSERVATION_DIM = FLEET_SIZE * 8 + MAX_OBSERVED_REQUESTS * 8 + 4; // 164

    std::vector<float> obs(OBSERVATION_DIM, 0.0f);

    const double halfX = std::max(std::abs(map.worldMin.x), std::abs(map.worldMax.x));
    const double halfY = std::max(std::abs(map.worldMin.y), std::abs(map.worldMax.y));
    const double maxZ = std::max(map.worldMax.z, 150.0);
    const double maxHSpeed = 15.0;
    const double maxVSpeed = 5.0;

    size_t offset = 0;

    // 1. Fleet features (10 UAVs x 8 features)
    for (size_t j = 0; j < FLEET_SIZE; ++j) {
        if (j < fleet.size()) {
            const auto& u = fleet.getUAV(j);
            obs[offset + 0] = static_cast<float>(MathUtils::clamp(u.position.x / halfX, -1.0, 1.0));
            obs[offset + 1] = static_cast<float>(MathUtils::clamp(u.position.y / halfY, -1.0, 1.0));
            obs[offset + 2] = static_cast<float>(MathUtils::clamp(u.position.z / maxZ, 0.0, 1.0));
            obs[offset + 3] = static_cast<float>(MathUtils::clamp(u.velocity.x / maxHSpeed, -1.0, 1.0));
            obs[offset + 4] = static_cast<float>(MathUtils::clamp(u.velocity.y / maxHSpeed, -1.0, 1.0));
            obs[offset + 5] = static_cast<float>(MathUtils::clamp(u.velocity.z / maxVSpeed, -1.0, 1.0));
            obs[offset + 6] = static_cast<float>(MathUtils::clamp(u.batteryPercent / 100.0, 0.0, 1.0));
            obs[offset + 7] = (u.mode == UAVMode::IDLE && u.batteryPercent > m_config.minBatteryDispatchPercent) ? 1.0f : 0.0f;
        }
        offset += 8;
    }

    // Identify pending active requests
    std::vector<size_t> pendingIndices;
    for (size_t r = 0; r < requests.size() && pendingIndices.size() < MAX_OBSERVED_REQUESTS; ++r) {
        if (requests[r].status == RequestStatus::PENDING) {
            pendingIndices.push_back(r);
        }
    }

    // 2. Pending Request features (10 requests x 8 features)
    for (size_t i = 0; i < MAX_OBSERVED_REQUESTS; ++i) {
        if (i < pendingIndices.size()) {
            const auto& r = requests[pendingIndices[i]];
            obs[offset + 0] = static_cast<float>(MathUtils::clamp(r.pickupLocation.x / halfX, -1.0, 1.0));
            obs[offset + 1] = static_cast<float>(MathUtils::clamp(r.pickupLocation.y / halfY, -1.0, 1.0));
            obs[offset + 2] = static_cast<float>(MathUtils::clamp(r.pickupLocation.z / maxZ, 0.0, 1.0));
            obs[offset + 3] = static_cast<float>(MathUtils::clamp(r.dropoffLocation.x / halfX, -1.0, 1.0));
            obs[offset + 4] = static_cast<float>(MathUtils::clamp(r.dropoffLocation.y / halfY, -1.0, 1.0));
            obs[offset + 5] = static_cast<float>(MathUtils::clamp(r.dropoffLocation.z / maxZ, 0.0, 1.0));
            obs[offset + 6] = static_cast<float>(MathUtils::clamp(r.payloadMass / m_config.maxPayloadKg, 0.0, 1.0));
            const double slack = (r.deadline - currentTime) / 300.0;
            obs[offset + 7] = static_cast<float>(MathUtils::clamp(slack, -1.0, 1.0));
        }
        offset += 8;
    }

    // 3. Global context features (4 features)
    obs[offset + 0] = static_cast<float>(std::fmod(currentTime, 600.0) / 600.0);
    obs[offset + 1] = static_cast<float>(std::min(pendingIndices.size() / 20.0, 1.0));

    size_t idleCount = 0;
    size_t chargingCount = 0;
    for (size_t j = 0; j < fleet.size(); ++j) {
        if (fleet.getUAV(j).mode == UAVMode::IDLE) idleCount++;
        else if (fleet.getUAV(j).mode == UAVMode::CHARGING) chargingCount++;
    }
    const float totalUavs = std::max(static_cast<float>(fleet.size()), 1.0f);
    obs[offset + 2] = static_cast<float>(idleCount) / totalUavs;
    obs[offset + 3] = static_cast<float>(chargingCount) / totalUavs;

    return obs;
}

void ONNXCoordinatorPolicy::dispatch(
    FleetManager& fleet,
    std::vector<DeliveryRequest>& requests,
    const CityMap& map,
    double currentTime
) const {
    if (!std::isfinite(currentTime) || fleet.size() == 0 || requests.empty()) return;

    if (!m_model) {
        m_fallback.dispatch(fleet, requests, map, currentTime);
        return;
    }

    const BatteryModel& battery = fleet.getBatteryModel();
    std::vector<bool> assigned(fleet.size(), false);
    for (size_t i = 0; i < fleet.size(); ++i) {
        assigned[i] = (fleet.getUAV(i).mode != UAVMode::IDLE);
    }

    // Collect candidate pending requests
    std::vector<size_t> pendingIndices;
    for (size_t r = 0; r < requests.size() && pendingIndices.size() < 10; ++r) {
        if (requests[r].status == RequestStatus::PENDING && !requests[r].isExpired(currentTime) &&
            requests[r].payloadMass <= m_config.maxPayloadKg) {
            pendingIndices.push_back(r);
        }
    }

    if (pendingIndices.empty()) return;

    // Iterative Maskable PPO Action Selection
    const size_t maxIterations = std::min(pendingIndices.size(), fleet.size());
    for (size_t iter = 0; iter < maxIterations; ++iter) {
        std::vector<float> obs = buildObservation(fleet, requests, map, currentTime);
        std::vector<float> logits;
        try {
            logits = m_model->run(obs);
        } catch (const std::exception& e) {
            std::cerr << "[ONNXCoordinator] Inference error: " << e.what()
                      << ". Falling back to HeuristicCoordinator.\n";
            m_fallback.dispatch(fleet, requests, map, currentTime);
            return;
        }

        if (logits.size() < 101) {
            break;
        }

        // Action 0 is WAIT
        int bestAction = 0;
        float bestLogit = logits[0]; // WAIT threshold

        for (size_t pi = 0; pi < pendingIndices.size(); ++pi) {
            size_t reqIndex = pendingIndices[pi];
            const auto& req = requests[reqIndex];
            if (req.status != RequestStatus::PENDING) continue;

            for (size_t ui = 0; ui < std::min(fleet.size(), size_t(10)); ++ui) {
                if (assigned[ui]) continue;
                const auto& uav = fleet.getUAV(ui);
                if (uav.mode != UAVMode::IDLE || uav.batteryPercent <= m_config.minBatteryDispatchPercent) continue;

                // Predictive multi-leg energy feasibility: UAV -> Pickup -> Dropoff -> Charger >= 20%
                if (!m_fallback.energyFeasible(uav, req, map, battery)) continue;

                int actionIdx = 1 + static_cast<int>(pi * 10 + ui);
                if (actionIdx < static_cast<int>(logits.size())) {
                    if (logits[actionIdx] > bestLogit) {
                        bestLogit = logits[actionIdx];
                        bestAction = actionIdx;
                    }
                }
            }
        }

        // If WAIT was selected or no feasible assignment beats WAIT threshold:
        // Pick the highest-logit feasible candidate action so requests are not stalled
        if (bestAction == 0) {
            float maxCandidateLogit = -std::numeric_limits<float>::infinity();
            for (size_t pi = 0; pi < pendingIndices.size(); ++pi) {
                size_t reqIndex = pendingIndices[pi];
                const auto& req = requests[reqIndex];
                if (req.status != RequestStatus::PENDING) continue;

                for (size_t ui = 0; ui < std::min(fleet.size(), size_t(10)); ++ui) {
                    if (assigned[ui]) continue;
                    const auto& uav = fleet.getUAV(ui);
                    if (uav.mode != UAVMode::IDLE || uav.batteryPercent <= m_config.minBatteryDispatchPercent) continue;
                    if (!m_fallback.energyFeasible(uav, req, map, battery)) continue;

                    int actionIdx = 1 + static_cast<int>(pi * 10 + ui);
                    if (actionIdx < static_cast<int>(logits.size()) && logits[actionIdx] > maxCandidateLogit) {
                        maxCandidateLogit = logits[actionIdx];
                        bestAction = actionIdx;
                    }
                }
            }
            if (bestAction == 0) {
                break;
            }
        }

        int pairIdx = bestAction - 1;
        size_t chosenPi = static_cast<size_t>(pairIdx / 10);
        size_t chosenUi = static_cast<size_t>(pairIdx % 10);

        if (chosenPi < pendingIndices.size() && chosenUi < fleet.size()) {
            size_t reqIndex = pendingIndices[chosenPi];
            auto& req = requests[reqIndex];
            const UAVId uavId = fleet.getUAV(chosenUi).id;
            if (fleet.assignRequest(uavId, req)) {
                req.transitionTo(RequestStatus::ASSIGNED, currentTime);
                req.assignedUavId = uavId;
                assigned[chosenUi] = true;
            } else {
                assigned[chosenUi] = true;
            }
        } else {
            break;
        }
    }

    // Proactive opportunity charging for idle UAVs when queue is empty and battery < 85%
    for (size_t ui = 0; ui < fleet.size(); ++ui) {
        if (assigned[ui]) continue;
        auto& uav = fleet.getUAV(ui);
        if (uav.mode == UAVMode::IDLE && uav.batteryPercent < 85.0 && requests.empty()) {
            int32_t chargerId = fleet.findNearestAvailableCharger(uav.position, map);
            if (chargerId != -1) {
                auto& ctx = fleet.getContext(uav.id);
                ctx.targetChargerId = chargerId;
                uav.mode = UAVMode::RETURNING_TO_CHARGE;
                assigned[ui] = true;
            }
        }
    }
}

} // namespace aerofleet
