# Step 17: ONNX Model Export, Optimization & Dual-Inference Parity

## Status: COMPLETED & VERIFIED

**The trained Stage 1 navigation policy has been successfully extracted, exported to ONNX format, validated with native `onnx.checker`, and verified for dual-inference numerical parity against PyTorch.** Parity across 10,000 synthetic test samples achieved a maximum absolute error of $\mathbf{1.07 \times 10^{-6}}$, substantially outperforming the required threshold of $\epsilon < 10^{-5}$. Single-step inference latency measured at $\mathbf{0.011\text{ ms}}$ ($> 90,000\text{ inferences/sec}$ on a single CPU core). All 18 CTest suites and 43 Pytest cases pass with a 100% pass rate.

---

## 1. Objectives & Architectural Scope

1. **Pure Actor Network Extraction**:
   - Extract only the deterministic actor network from the Stable-Baselines3 PPO checkpoint (`MlpPolicy`), discarding the value/critic network and entropy heads.
   - The actor maps normalized observations $[B, 87] \to [B, 3]$ desired velocity commands in UAV body coordinates, strictly clamped to $[-1.0, 1.0]^3$.
2. **Standardized ONNX Export**:
   - Export via `torch.onnx.export` with Opset version 17 using standard linear and hyperbolic tangent layers (`nn.Linear`, `nn.Tanh`, `torch.clamp`).
   - Dynamic batch axis configuration: `input_names=['observation']`, `output_names=['action']`, supporting both single-UAV real-time step execution ($B=1$) and parallel multi-UAV batch execution ($B=N$).
3. **Dual-Inference Numerical Parity**:
   - Compare forward inference between PyTorch (`NavigationActor.eval()`) and ONNX Runtime (`onnxruntime.InferenceSession`) across 10,000 synthetic observations sampled from the $[-10.0, 10.0]$ normalized feature domain.
   - Assert $\max |a_{\text{pytorch}} - a_{\text{onnx}}| < 10^{-5}$.
4. **Production Deployment Manifest**:
   - Save `models/navigation/manifest.json` detailing model I/O tensor names, dimensions, opset version, SHA-256 model checksum, and git commit hash.
   - Save `models/navigation/normalization.json` exporting observation running means $\mu$ and variances $\sigma^2$ for C++ inference in Step 18.

---

## 2. Components Created & Modified

| File | Purpose |
| :--- | :--- |
| [python/export_onnx.py](file:///c:/Users/Prateek/Desktop/Coding/project/AeroFleet/python/export_onnx.py) | CLI tool to extract PPO actor, export ONNX model, validate graph, and generate production manifest |
| [python/verify_onnx_parity.py](file:///c:/Users/Prateek/Desktop/Coding/project/AeroFleet/python/verify_onnx_parity.py) | CLI tool to execute 10,000-sample dual-inference parity test and benchmark inference latency |
| [tests/test_onnx_export.py](file:///c:/Users/Prateek/Desktop/Coding/project/AeroFleet/tests/test_onnx_export.py) | Comprehensive pytest suite covering ONNX checker, parity bounds, dynamic batching, action limits, and manifest consistency |
| [CMakeLists.txt](file:///c:/Users/Prateek/Desktop/Coding/project/AeroFleet/CMakeLists.txt) | Registered `ONNXExportTests` into CTest test suite |
| [models/navigation/nav_policy.onnx](file:///c:/Users/Prateek/Desktop/Coding/project/AeroFleet/models/navigation/nav_policy.onnx) | Exported ONNX model artifact (349.3 KB, SHA-256: `ff776157...`) |
| [models/navigation/manifest.json](file:///c:/Users/Prateek/Desktop/Coding/project/AeroFleet/models/navigation/manifest.json) | Complete metadata manifest linking model, checksum, git commit, and feature scales |
| [models/navigation/normalization.json](file:///c:/Users/Prateek/Desktop/Coding/project/AeroFleet/models/navigation/normalization.json) | Observation normalization parameters ($\mu, \sigma^2, \epsilon$) for native C++ inference |

---

## 3. Mathematical & Numerical Verification

### 3.1 Dual-Inference Parity Benchmark (`python/verify_onnx_parity.py`)
Tested across 10,000 synthetic random observation vectors generated with independent seed:

| Metric | Measured Value | Requirement / Threshold | Status |
| :--- | :--- | :--- | :--- |
| **Maximum Absolute Error ($\epsilon$)** | $\mathbf{1.072884 \times 10^{-6}}$ | $\epsilon < 1.0 \times 10^{-5}$ | **PASSED** (Order of magnitude margin) |
| **Mean Absolute Error** | $\mathbf{7.683998 \times 10^{-8}}$ | - | **PASSED** |
| **Dynamic Batch Maximum Error** | $\mathbf{4.768372 \times 10^{-7}}$ | $< 1.0 \times 10^{-5}$ | **PASSED** |
| **Single-Step Latency ($B=1$)** | $\mathbf{0.0111\text{ ms}}$ ($11.1\text{ }\mu\text{s}$) | $< 0.50\text{ ms}$ | **PASSED** ($90,210\text{ inferences/sec}$) |
| **Output Action Bounds** | $\min \ge -1.0, \max \le 1.0$ | $[-1.0, 1.0]^3$ | **PASSED** |

### 3.2 Dynamic Batch Verification
Evaluated batch sizes $B \in \{1, 2, 7, 16, 64\}$:
- Single-sample evaluation matches batched slices identically:
  $$\max_{i \in [1, B]} |a_i(B) - a(1)| < 10^{-6}$$
- Verifies that ONNX Runtime dynamic tensor sizing does not introduce batch-stride or memory-alignment artifacts.

---

## 4. Test Suite Execution & Integration

1. **CTest in Build Directory (18/18 Passed)**:
   ```bash
   cd build && ctest --output-on-failure
   ```
   Output:
   ```
   100% tests passed, 0 tests failed out of 18
   Total Test time (real) = 10.83 sec
   ```

2. **Pytest Suite (43/43 Passed)**:
   ```bash
   PYTHONPATH=python pytest tests/test_nav_env.py tests/test_bindings.py tests/test_differential_physics.py tests/test_onnx_export.py
   ```
   Output:
   ```
   tests/test_nav_env.py .................                                  [ 39%]
   tests/test_bindings.py ........                                          [ 58%]
   tests/test_differential_physics.py .........                             [ 79%]
   tests/test_onnx_export.py .........                                      [100%]
   ======================== 43 passed in 3.96s ========================
   ```

---

## 5. Transition to Step 18

With the verified ONNX model and observation normalization parameters staged in `models/navigation/`:
- **Step 18** will implement:
  1. `cpp/inference/ONNXModel.hpp` and `.cpp`: Encapsulating ONNX Runtime C++ API (`Ort::Env`, `Ort::Session`).
  2. `cpp/control/ONNXNavigationPolicy.hpp` and `.cpp`: Implementing `INavigationPolicy` with observation feature normalization matching `normalization.json`.
  3. `cpp/control/SafetyLayer.hpp` and `.cpp`: Deterministic collision lookahead probe and emergency override.
  4. `tests/test_onnx_runtime.cpp`: C++ unit testing and 20-waypoint autonomous flight benchmark.
