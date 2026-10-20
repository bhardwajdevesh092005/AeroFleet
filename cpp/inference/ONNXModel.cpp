#include "inference/ONNXModel.hpp"

#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "onnxruntime_cxx_api.h"

namespace aerofleet {

class ONNXModel::Impl {
public:
    std::string modelPath;
    Ort::Env env;
    Ort::SessionOptions sessionOptions;
    std::unique_ptr<Ort::Session> session;
    Ort::MemoryInfo memoryInfo;

    std::string inputName{"observation"};
    std::string outputName{"action"};
    size_t inputDim{87};
    size_t outputDim{3};

    Impl(const std::string& path, const std::string& envName, int numThreads)
        : modelPath(path),
          env(ORT_LOGGING_LEVEL_WARNING, envName.c_str()),
          memoryInfo(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)) {
        sessionOptions.SetIntraOpNumThreads(numThreads);
        sessionOptions.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

        try {
            session = std::make_unique<Ort::Session>(env, modelPath.c_str(), sessionOptions);
        } catch (const Ort::Exception& e) {
            throw std::runtime_error("Failed to load ONNX model from '" + modelPath + "': " + e.what());
        }

        // Query input and output info
        Ort::AllocatorWithDefaultOptions allocator;
        try {
            auto inNameAlloc = session->GetInputNameAllocated(0, allocator);
            if (inNameAlloc) {
                inputName = inNameAlloc.get();
            }
            auto outNameAlloc = session->GetOutputNameAllocated(0, allocator);
            if (outNameAlloc) {
                outputName = outNameAlloc.get();
            }

            auto inTypeInfo = session->GetInputTypeInfo(0);
            auto inTensorInfo = inTypeInfo.GetTensorTypeAndShapeInfo();
            auto inShape = inTensorInfo.GetShape();
            if (!inShape.empty()) {
                inputDim = static_cast<size_t>(inShape.back());
            }

            auto outTypeInfo = session->GetOutputTypeInfo(0);
            auto outTensorInfo = outTypeInfo.GetTensorTypeAndShapeInfo();
            auto outShape = outTensorInfo.GetShape();
            if (!outShape.empty()) {
                outputDim = static_cast<size_t>(outShape.back());
            }
        } catch (const std::exception&) {
            // Defaults (87, 3) remain valid for AeroFleet models
        }
    }

    std::vector<float> runBatch(const std::vector<float>& inputs, size_t batchSize) const {
        if (inputs.size() != batchSize * inputDim) {
            throw std::invalid_argument(
                "ONNXModel::runBatch input buffer size (" + std::to_string(inputs.size()) +
                ") does not match batchSize * inputDim (" + std::to_string(batchSize * inputDim) + ")"
            );
        }

        std::vector<int64_t> inputShape = {static_cast<int64_t>(batchSize), static_cast<int64_t>(inputDim)};
        
        // Ort::Value::CreateTensor requires non-const pointer, but does not modify input in inference
        float* rawData = const_cast<float*>(inputs.data());
        Ort::Value inputTensor = Ort::Value::CreateTensor<float>(
            memoryInfo, rawData, inputs.size(), inputShape.data(), inputShape.size()
        );

        const char* inNames[] = {inputName.c_str()};
        const char* outNames[] = {outputName.c_str()};

        auto outputTensors = session->Run(
            Ort::RunOptions{nullptr}, inNames, &inputTensor, 1, outNames, 1
        );

        if (outputTensors.empty()) {
            throw std::runtime_error("ONNX inference produced no output tensors");
        }

        const float* outData = outputTensors[0].GetTensorData<float>();
        size_t totalOutputElements = batchSize * outputDim;
        return std::vector<float>(outData, outData + totalOutputElements);
    }
};

ONNXModel::ONNXModel(const std::string& modelPath, const std::string& envName, int numThreads)
    : m_impl(std::make_unique<Impl>(modelPath, envName, numThreads)) {}

ONNXModel::~ONNXModel() = default;
ONNXModel::ONNXModel(ONNXModel&&) noexcept = default;
ONNXModel& ONNXModel::operator=(ONNXModel&&) noexcept = default;

std::vector<float> ONNXModel::run(const std::vector<float>& input) const {
    return m_impl->runBatch(input, 1);
}

std::vector<float> ONNXModel::runBatch(const std::vector<float>& inputs, size_t batchSize) const {
    return m_impl->runBatch(inputs, batchSize);
}

size_t ONNXModel::getInputDim() const noexcept {
    return m_impl->inputDim;
}

size_t ONNXModel::getOutputDim() const noexcept {
    return m_impl->outputDim;
}

const std::string& ONNXModel::getInputName() const noexcept {
    return m_impl->inputName;
}

const std::string& ONNXModel::getOutputName() const noexcept {
    return m_impl->outputName;
}

const std::string& ONNXModel::getModelPath() const noexcept {
    return m_impl->modelPath;
}

} // namespace aerofleet
