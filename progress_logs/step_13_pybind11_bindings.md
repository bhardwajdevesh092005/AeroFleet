# Step 13: pybind11 C++ Simulator Bindings

> **Schema update:** The 72-element observation described in this historical Step 13 record was superseded during Step 14. Current bindings expose the 87-element raw observation schema v2; the Gymnasium wrapper applies finite config-derived normalization. See [Step 14](step_14_gymnasium_navigation.md).

## 1. Objectives & Scope
Step 13 connects the high-performance C++20 simulation core with the Python data science and Reinforcement Learning ecosystem. It encapsulates procedural world generation, multi-UAV kinematic integration, continuous collision detection (CCD), battery discharge/charging, customer request lifecycle, and spatial raycasting into a unified C++ `Simulator` engine, and exposes it through zero-overhead `pybind11` Python bindings.

Key deliverables:
1. **Unified C++ `Simulator` Engine**: Manages the multi-rate simulation hierarchy (physics integration at 50 Hz, coordinator dispatch at 1 Hz, policy stepping at 10 Hz) with complete state encapsulation.
2. **72-Dimensional Vectorized Observation**: High-density feature vector representing ego kinematics, target vector, lidar rangefinders, and multi-neighbor dynamic tracking.
3. **pybind11 C++ Bindings Module (`aerofleet_cpp`)**: Direct C++ to Python translation with zero-copy NumPy buffers (`py::array_t<float>`), automatic type conversions, and GIL release during heavy simulation stepping.
4. **Comprehensive Test Suite**: C++ unit tests (`test_simulator.cpp`) and Python verification tests (`test_bindings.py`) covering lifetime, observation shapes, action execution, and deterministic seed reproducibility.

---

## 2. Components & Files Created/Modified

| File | Purpose |
| :--- | :--- |
| `cpp/core/Simulator.hpp` | Definition of `Simulator`, `Observation` (72-dim), and `StepResult` structures. |
| `cpp/core/Simulator.cpp` | Implementation of multi-rate sub-step physics stepping, 44-beam lidar raycasting, neighbor tracking, and dense reward computation. |
| `python/bindings/aerofleet_bindings.cpp` | pybind11 module exposing `Simulator`, `Observation`, `StepResult`, `WorldSnapshot`, `VelocityCommand`, `UAVMode`, and `RequestStatus`. |
| `python/aerofleet/__init__.py` | Top-level Python package interface importing `aerofleet_cpp` symbols. |
| `tests/test_simulator.cpp` | C++ test suite validating `Simulator` reset, observation dimensions, heuristic control, and determinism. |
| `tests/test_bindings.py` | Python test suite verifying module imports, NumPy array shapes, action stepping, and stepping throughput. |
| `CMakeLists.txt` | Configured `-fPIC`, added `Simulator.cpp` to `AeroFleetCore`, added `aerofleet_cpp` module target with `SYSTEM` headers, and registered tests. |

---

## 3. Mathematical Models & Data Structures

### 3.1 72-Dimensional Vectorized Observation Specification

For an ego UAV $i$, the observation vector $\mathbf{o}_i \in \mathbb{R}^{72}$ is packed as follows:

$$\mathbf{o}_i = \left[ \mathbf{v}_{\text{ego}}, \Delta \mathbf{p}_{\text{target}}, d_{\text{target}}, \beta, m_{\text{payload}}, \|\mathbf{v}_{\text{ego}}\|, \mathbf{b}_{\text{lidar}}, \mathbf{u}_{\text{neighbors}} \right]^T$$

| Component | Dimensions | Type | Description |
| :--- | :---: | :---: | :--- |
| Ego Velocity $\mathbf{v}_{\text{ego}}$ | 3 | `float32` | 3D velocity vector $(v_x, v_y, v_z)$ in m/s |
| Relative Target $\Delta \mathbf{p}_{\text{target}}$ | 3 | `float32` | Vector to current active waypoint $(\mathbf{p}_{\text{target}} - \mathbf{p}_{\text{ego}})$ in meters |
| Target Distance $d_{\text{target}}$ | 1 | `float32` | Euclidean distance $\|\Delta \mathbf{p}_{\text{target}}\|$ in meters |
| Battery Fraction $\beta$ | 1 | `float32` | State of Charge normalized in $[0.0, 1.0]$ |
| Payload Mass $m_{\text{payload}}$ | 1 | `float32` | Carried cargo mass in kg |
| Linear Speed $\|\mathbf{v}_{\text{ego}}\|$ | 1 | `float32` | Scalar velocity magnitude in m/s |
| Lidar Beams $\mathbf{b}_{\text{lidar}}$ | 44 | `float32` | 36 horizontal azimuth beams ($10^\circ$ res) $+ 8$ elevation beams ($\pm 30^\circ$) normalized in $[0.0, 1.0]$ |
| Nearby UAVs $\mathbf{u}_{\text{neighbors}}$ | 18 | `float32` | $K=3$ nearest neighbors: $(\Delta x, \Delta y, \Delta z, \Delta v_x, \Delta v_y, \Delta v_z)$ relative to ego UAV |
| **Total Dimensionality** | **72** | `float32` | Contiguous single-precision float array |

### 3.2 Lidar Raycasting
Lidar raycasting queries the city's 2D uniform spatial hash grid broadphase (`broadphase.querySphere(position, maxRange)`). For each ray:
1. Candidate building bounding boxes (AABBs) within $R_{\max} = 100\,\text{m}$ are gathered.
2. Analytical slab-method intersection (`raycastAABB`) computes distance $d_{\text{hit}}$.
3. Beam output is normalized: $b_k = \min(d_{\text{hit}}, R_{\max}) / R_{\max} \in [0.0, 1.0]$.

### 3.3 Zero-Overhead Python Stepping with GIL Release
During execution of `sim.step(actions)` or `sim.step_heuristic()`, the C++ engine releases Python's Global Interpreter Lock (`py::gil_scoped_release release;`). This allows multi-threaded Python training runners (e.g. multi-worker PPO or SubprocVecEnv) to execute without interpreter contention.

---

## 4. Verification & Testing Results

### 4.1 C++ Simulator Unit Test (`test_simulator`)
```
========================================
  AeroFleet Simulator C++ Test Suite    
========================================
[Test 1] Testing Simulator Initialization and Config Loading...
  -> Passed: Simulator successfully initialized with 10 UAVs and 35 buildings.
[Test 2] Testing Observation Extraction and Vectorization...
  -> Passed: Observation dimensionality verified (72 floats, 44 lidar, 18 neighbor states).
[Test 3] Testing Heuristic Multi-Step Stepping...
  -> Passed: 50 heuristic policy steps executed smoothly. Time: 5s, Requests Created: 4.
[Test 4] Testing Direct Velocity Command Stepping...
  -> Passed: Commanded velocity integrated into rigid kinematics. Delta X = 9.96437 m.
[Test 5] Testing Deterministic Reproducibility Across Resets...
  -> Passed: Bit-for-bit identical trajectory across independent instances with seed 999.
========================================
  All Simulator C++ Tests PASSED! (5/5) 
========================================
```

### 4.2 Python Bindings Test Suite (`test_bindings.py`)
```
========================================
  AeroFleet Python Bindings Test Suite  
========================================
[Python Test 1] Testing Enums and Types...
  -> Passed: UAVMode and RequestStatus enums verified.
[Python Test 2] Testing Vec3 and VelocityCommand...
  -> Passed: Vec3 vector algebra and VelocityCommand initialization verified.
[Python Test 3] Testing Simulator Lifecycle and Snapshot...
  -> Passed: Simulator initialization, reset, and world snapshot inspection verified.
[Python Test 4] Testing Observation Vectorization and NumPy Integration...
  -> Passed: 72-dim Observation vectorization and NumPy (10, 72) matrix verified.
[Python Test 5] Testing Heuristic Stepping and StepResult...
  -> Passed: 30 heuristic steps executed. Sim time: 3.00s.
[Python Test 6] Testing Commanded Actions Stepping...
  -> Passed: Commanded velocity executed. UAV 0 delta X: 15.93 m.
[Python Test 7] Testing Cross-Instance Reproducibility...
  -> Passed: Bit-for-bit identical trajectory between Python simulator instances.
[Python Test 8] Benchmarking High-Speed Python Stepping Throughput...
  -> Performance: 1000 steps in 0.0304s = 32,841 steps/s (3,284x real-time speedup).
  -> Passed: High-throughput benchmark satisfied (>1,000 steps/sec).
========================================
  All Python Binding Tests PASSED! (8/8)
========================================
```

### 4.3 CTest Full Suite Status (15/15 Passed)
```
100% tests passed, 0 tests failed out of 15
Total Test time (real) = 1.55 sec
```

---

## 5. Encountered Edge Cases & Resolutions

### Current checkout validation note (2026-10-19)

The binding source and Python tests are present, but the current checkout does not contain the workspace-local `third_party` headers and no system pybind11 package is available. CMake now discovers supported Python/pybind11 installations rather than pinning Python 3.14's extension suffix, and clearly skips the optional target when dependencies are absent. The Python module itself could not be rebuilt or re-tested in this environment; the earlier results above are historical results from the environment where the dependencies were installed.

1. **User-Space Dependencies Without Sudo**:
   - *Problem*: System lacked root privileges and interactive terminal access (`sudo` prohibited).
   - *Resolution*: Downloaded standard distribution packages (`python3.14-dev`, `pybind11-dev`, `python3-numpy`, `libblas3`, `liblapack3`, `libgfortran5`) via non-root `apt-get download` and unpacked via `dpkg -x` directly into `<workspace>/third_party/`.
2. **Third-Party Header Compiler Warnings**:
   - *Problem*: Global compiler flags (`-Wall -Wextra -Wshadow -Wold-style-cast`) produced warnings inside `pybind11/numpy.h`.
   - *Resolution*: Marked `${PYBIND11_INC_DIR}` and `${PYTHON_INC_DIR}` as `SYSTEM` includes in CMake (`target_include_directories(aerofleet_cpp SYSTEM PRIVATE ...)`), ensuring zero compiler warnings while preserving strict compilation flags for project code.
3. **CollisionType and DeliveryRequest Enum/Field Alignment**:
   - *Problem*: `Simulator.cpp` initially referenced `CollisionType::UAV` instead of `CollisionType::UAV_UAV`, and bindings referenced `pickupPosition` instead of `pickupLocation`.
   - *Resolution*: Updated to authoritative enum names and added convenience property aliases in Python bindings so both `pickup_location` and `pickup_position` resolve seamlessly.

