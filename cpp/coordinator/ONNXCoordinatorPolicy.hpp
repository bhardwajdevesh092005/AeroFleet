#pragma once

#include <memory>
#include <string>
#include <vector>
#include "coordinator/ICoordinator.hpp"
#include "coordinator/HeuristicCoordinator.hpp"
#include "inference/ONNXModel.hpp"

namespace aerofleet {

struct CoordinatorPolicyConfig {
    std::string modelPath{"models/coordinator/coord_policy.onnx"};
    double nominalSpeed{10.0};
    double maxPayloadKg{3.0};
    double minBatteryDispatchPercent{25.0};
};

/**
 * @brief Autonomous central fleet coordinator executing trained deep reinforcement learning models.
 *
 * Evaluates the trained MaskablePPO coordinator ONNX policy at runtime, constructing the
 * 164-dimensional fleet observation, applying strict physical action masks (multi-leg energy
 * feasibility, drone availability, cargo capacity), and dispatching requests optimally.
 */
class ONNXCoordinatorPolicy final : public ICoordinator {
public:
    ONNXCoordinatorPolicy();
    explicit ONNXCoordinatorPolicy(const CoordinatorPolicyConfig& config);
    explicit ONNXCoordinatorPolicy(const std::string& modelPath);

    void dispatch(
        FleetManager& fleet,
        std::vector<DeliveryRequest>& requests,
        const CityMap& map,
        double currentTime
    ) const override;

    [[nodiscard]] bool isModelLoaded() const noexcept { return m_model != nullptr; }
    [[nodiscard]] const ONNXModel* getModel() const noexcept { return m_model.get(); }
    [[nodiscard]] const HeuristicCoordinator& getHeuristicCoordinator() const noexcept { return m_fallback; }
    [[nodiscard]] const CoordinatorPolicyConfig& getConfig() const noexcept { return m_config; }

    [[nodiscard]] std::vector<float> buildObservation(
        const FleetManager& fleet,
        const std::vector<DeliveryRequest>& requests,
        const CityMap& map,
        double currentTime
    ) const;

private:
    CoordinatorPolicyConfig m_config;
    std::unique_ptr<ONNXModel> m_model;
    HeuristicCoordinator m_fallback;
};

} // namespace aerofleet
