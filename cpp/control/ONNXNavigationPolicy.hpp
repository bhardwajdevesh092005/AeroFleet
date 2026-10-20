#pragma once

#include <memory>
#include <string>
#include <vector>
#include "control/INavigationPolicy.hpp"
#include "control/SafetyLayer.hpp"
#include "inference/ONNXModel.hpp"

namespace aerofleet {

/**
 * @brief Autonomous navigation policy executing trained deep reinforcement learning models.
 *
 * Implements the INavigationPolicy interface by querying a loaded ONNX actor network,
 * normalizing incoming observation features according to production running statistics,
 * converting normalized actions to physical velocity commands in the body frame,
 * rotating into the world frame, and validating the final command through the SafetyLayer.
 */
struct NavigationPolicyConfig {
    std::string modelPath{"models/navigation/nav_policy.onnx"};
    std::string normalizationPath{"models/navigation/normalization.json"};
    double maxHorizontalSpeed{15.0};
    double maxVerticalSpeed{5.0};
    double maxAcceleration{6.0};
    double maxPayloadMass{5.0};
    double maxObservationSpeed{15.8113883}; // sqrt(15^2 + 5^2)
    SafetyLayerConfig safetyConfig{};
};

class ONNXNavigationPolicy final : public INavigationPolicy {
public:
    using Config = NavigationPolicyConfig;

    ONNXNavigationPolicy();
    explicit ONNXNavigationPolicy(const Config& config);
    explicit ONNXNavigationPolicy(
        const std::string& modelPath,
        const std::string& normalizationPath,
        const SafetyLayerConfig& safetyConfig = SafetyLayerConfig{}
    );

    [[nodiscard]] VelocityCommand computeAction(
        const UAVState& uav,
        const CityMap& map,
        double dt
    ) const override;

    [[nodiscard]] const ONNXModel& getModel() const noexcept { return *m_model; }
    [[nodiscard]] const SafetyLayer& getSafetyLayer() const noexcept { return m_safetyLayer; }
    [[nodiscard]] SafetyLayer& getSafetyLayer() noexcept { return m_safetyLayer; }
    [[nodiscard]] const std::vector<float>& getRunningMean() const noexcept { return m_mean; }
    [[nodiscard]] const std::vector<float>& getRunningVar() const noexcept { return m_var; }

    [[nodiscard]] std::vector<float> buildRawObservation(
        const UAVState& uav,
        const CityMap& map
    ) const;

    [[nodiscard]] std::vector<float> buildNormalizedObservation(
        const UAVState& uav,
        const CityMap& map
    ) const;

private:
    Config m_config;
    std::shared_ptr<ONNXModel> m_model;
    std::vector<float> m_mean;
    std::vector<float> m_var;
    float m_epsilon{1e-8f};
    float m_clipObs{10.0f};
    mutable SafetyLayer m_safetyLayer;

    void loadNormalization(const std::string& normPath);
};

} // namespace aerofleet
