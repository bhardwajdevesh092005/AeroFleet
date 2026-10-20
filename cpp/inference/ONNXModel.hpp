#pragma once

#include <memory>
#include <string>
#include <vector>

namespace aerofleet {

/**
 * @brief High-performance C++ wrapper for ONNX Runtime inference sessions.
 *
 * Encapsulates Ort::Env, Ort::Session, Ort::MemoryInfo, dynamic tensor allocation,
 * and memory management for ultra-low latency inference (< 0.05 ms).
 */
class ONNXModel {
public:
    /**
     * @brief Construct and initialize an ONNX session from a model file.
     * @param modelPath Path to the .onnx model file.
     * @param envName Environment identifier for logging.
     * @param numThreads Thread pool size for intra-op parallelism (default 1 for minimum latency).
     */
    explicit ONNXModel(
        const std::string& modelPath,
        const std::string& envName = "AeroFleetInference",
        int numThreads = 1
    );

    ~ONNXModel();

    // Movable, non-copyable
    ONNXModel(ONNXModel&&) noexcept;
    ONNXModel& operator=(ONNXModel&&) noexcept;
    ONNXModel(const ONNXModel&) = delete;
    ONNXModel& operator=(const ONNXModel&) = delete;

    /**
     * @brief Execute forward inference for a single input vector.
     * @param input Input feature vector of size inputDim.
     * @return Output prediction vector of size outputDim.
     */
    [[nodiscard]] std::vector<float> run(const std::vector<float>& input) const;

    /**
     * @brief Execute forward inference for a batch of input vectors.
     * @param inputs Contiguous buffer of size batchSize * inputDim.
     * @param batchSize Number of samples in the batch.
     * @return Contiguous output buffer of size batchSize * outputDim.
     */
    [[nodiscard]] std::vector<float> runBatch(const std::vector<float>& inputs, size_t batchSize) const;

    [[nodiscard]] size_t getInputDim() const noexcept;
    [[nodiscard]] size_t getOutputDim() const noexcept;
    [[nodiscard]] const std::string& getInputName() const noexcept;
    [[nodiscard]] const std::string& getOutputName() const noexcept;
    [[nodiscard]] const std::string& getModelPath() const noexcept;

private:
    class Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace aerofleet
