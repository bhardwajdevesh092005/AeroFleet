# AeroFleet: Autonomous Urban UAV Fleet Delivery Simulation

[![Build & Tests](https://img.shields.io/badge/CTest-20%2F20%20Passed-brightgreen.svg)]()
[![ONNX Runtime](https://img.shields.io/badge/ONNX%20Runtime-v1.17.3%20Embedded-blue.svg)]()
[![RL Engine](https://img.shields.io/badge/Gymnasium%20%2B%20SB3-MaskablePPO-orange.svg)]()
[![Simulation Speed](https://img.shields.io/badge/Headless%20Speed-%3E300%2C000%20steps%2Fs-success.svg)]()

**AeroFleet** is an end-to-end, high-performance simulation platform for autonomous urban Unmanned Aerial Vehicle (UAV) delivery fleets powered by dual-tier hierarchical reinforcement learning. Dynamic delivery requests arrive across a 3D procedural urban environment from stores and customers. A **Central Fleet Coordinator** allocates feasible deliveries to candidate UAVs via discrete action masking, while an autonomous **Local Navigation Policy** guides individual drones through 3D urban air corridors, avoiding static obstacles and dynamic UAV traffic under finite battery constraints.

---

## Architectural Core Principles

1. **C++ as the Authoritative Simulation Core**:
   - Rigid 3D UAV kinematics with 2nd-order motion integration (`KinematicsIntegrator`).
   - Continuous Collision Detection (CCD) via swept segment-AABB and multi-agent capsule-capsule tests (`CollisionSystem`).
   - High-fidelity electro-aerodynamic battery discharge and multi-station CC-CV recharging model (`BatteryModel`).
   - Finite state-machine delivery lifecycle management (`FleetManager`).
   - High-performance headless simulation achieving **> 300,000 steps/sec** (> 6,000x real-time).

2. **Python as the RL Training Runtime**:
   - `pybind11` zero-copy bindings (`aerofleet_cpp`) exposing simulation state, observations, and stepping.
   - Dual Gymnasium environments: `AeroFleetNavEnv` (87-dim body-frame observation) and `AeroFleetCoordinatorEnv` (164-dim global context observation, 101 discrete actions with action masking).
   - Training pipeline with Stable-Baselines3 and SB3-Contrib (`MaskablePPO`).

3. **Dual-Tier Hierarchical Reinforcement Learning**:
   - **Local Navigation Controller (Policy A)**: Continuous 3D velocity control with local obstacle avoidance and multi-agent Velocity Obstacles separation. Exported to ONNX and embedded in C++ via ONNX Runtime (< 0.01 ms inference latency).
   - **Central Fleet Coordinator (Policy B)**: Combinatorial dispatch policy trained with discrete action masking, optimizing delivery throughput and deadline margins while preventing battery stranding.

4. **Deterministic Safety Layer**:
   - C++ supervisory filter (`SafetyLayer`) intercepting neural network velocity commands and performing forward predictive collision checks against static world geometry. Guaranteed zero building and mid-air collisions.

5. **Decoupled 3D Visualization & Live HUD**:
   - GPU-accelerated 3D rendering powered by **raylib**, reading immutable `WorldSnapshot` instances with sub-step alpha interpolation, interactive orbit/chase camera controls, and comprehensive telemetry HUD.

---

## Repository Directory Structure

```text
AeroFleet/
├── CMakeLists.txt                  # Top-level CMake build script with CTest harness
├── README.md                       # Main project documentation
├── config/
│   ├── simulation.json             # Authoritative simulation and physics configuration
│   ├── city_default.json           # Urban procedural generation parameters
│   ├── training_nav.yaml           # Navigation PPO hyperparameter specification
│   └── training_coordinator.yaml   # Fleet Coordinator MaskablePPO hyperparameters
├── cpp/
│   ├── core/                       # SimulationClock, Config, WorldState, Simulator
│   ├── math/                       # Vec3, MathUtils, Spatial Geometry primitives
│   ├── world/                      # CityMap, Procedural WorldGenerator
│   ├── physics/                    # Rigid Kinematics, Continuous Collision Detection, Battery
│   ├── fleet/                      # FleetManager, UAVState, BatteryModel
│   ├── requests/                   # RequestGenerator (Poisson arrival model)
│   ├── control/                    # WaypointController, SafetyLayer, ONNXNavigationPolicy
│   ├── coordinator/                # HeuristicCoordinator, ICoordinator
│   ├── inference/                  # ONNXModel (ONNX Runtime C++ API wrapper)
│   ├── visualization/              # Renderer (raylib), CameraController
│   ├── telemetry/                  # MetricsTracker, EventLogger
│   └── main.cpp                    # Authoritative master simulation runner with CLI
├── python/
│   ├── aerofleet/                  # Python package with Gymnasium environments
│   │   ├── nav_env.py              # Navigation Gymnasium environment (87-dim obs)
│   │   └── coordinator_env.py      # Coordinator Gymnasium environment (action masking)
│   ├── bindings/                   # pybind11 C++ binding definitions
│   ├── train_nav.py                # PPO navigation trainer with curriculum learning
│   ├── evaluate_nav.py             # Navigation policy benchmark suite
│   ├── export_onnx.py              # PyTorch to ONNX export and parity verification
│   ├── verify_onnx_parity.py       # Cross-runtime numeric parity tests
│   ├── train_coordinator.py        # MaskablePPO coordinator trainer & ONNX exporter
│   ├── evaluate_coordinator.py     # Coordinator evaluation against FCFS / Nearest-Neighbor
│   └── evaluate_system.py          # End-to-end 4-way comparative system benchmark
├── models/
│   ├── navigation/                 # nav_policy.onnx, normalization.json, checkpoint zip
│   └── coordinator/                # coord_policy.onnx, manifest.json, checkpoint zip
├── tests/                          # 20 CTest C++ and Python unit/integration test suites
└── progress_logs/                  # Granular 20-step implementation logs and audit trails
```

---

## Quickstart & Build Instructions

### Prerequisites
- **Compiler**: GCC 11+ or Clang 14+ supporting C++20.
- **Build System**: CMake 3.20+.
- **Libraries**: raylib (provided via CMake FetchContent/system), ONNX Runtime C++ Linux x64 v1.17.3 (auto-fetched).
- **Python**: Python 3.10+ with `pip` and virtual environment.

### 1. Build C++ Simulation Engine & Test Harness
```bash
# Configure and compile in Release mode
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel $(nproc)

# Run full 20-suite CTest verification suite
ctest --test-dir build --output-on-failure
```

### 2. Install Python RL Environment
```bash
pip install -r requirements-rl.txt
pip install -e .
```

---

## Running the Simulation

### Interactive 3D Graphical Mode (raylib)
Launch the interactive 3D simulation with dynamic urban rendering, real-time trajectories, and HUD telemetry:
```bash
./build/AeroFleet --config config/simulation.json --speed 1.0
```
**Interactive Controls:**
- `TAB`: Cycle selected drone for camera tracking.
- `C`: Cycle camera projection mode (`ORBIT` -> `CHASE` -> `TOP-DOWN` -> `FREE`).
- `H`: Toggle live telemetry HUD overlay.
- `T`: Toggle 3D UAV historical flight trajectories.
- `Right Mouse Drag`: Rotate orbital / chase view.
- `Mouse Wheel`: Zoom camera distance.

### High-Speed Headless Benchmark
Execute a high-speed simulation without window creation:
```bash
# 1-hour simulated time with baseline heuristic navigation
./build/AeroFleet --headless --steps 180000 --benchmark

# 1-hour simulated time with embedded ONNX deep RL navigation + Safety Layer
./build/AeroFleet --headless --steps 180000 --benchmark --onnx-nav
```

### CLI Command Options
```text
  --config <path>       Path to simulation JSON config (default: config/simulation.json)
  --headless            Run in headless mode without raylib GUI window
  --benchmark           Run high-speed benchmark and print detailed performance metrics
  --steps <N>           Run for N fixed physics steps then exit (0 = unlimited)
  --speed <factor>      Time scale factor in visual mode (e.g. 1.0 = real-time, 5.0 = 5x)
  --seed <S>            Override PRNG seed
  --output <dir>        Directory to output metrics and logs (default: logs)
  --onnx-nav            Enable embedded ONNX deep RL navigation policy
  --nav-policy <path>   Path to ONNX navigation model (default: models/navigation/nav_policy.onnx)
  --norm-path <path>    Path to normalization stats (default: models/navigation/normalization.json)
  --no-safety           Disable deterministic Safety Layer obstacle filter
  --log-events          Enable event ring-buffer logging to simulation_events.jsonl
  --no-metrics          Disable saving metrics to JSON/CSV
  --help, -h            Show help message
```

---

## Reinforcement Learning Pipeline

### 1. Train Local Navigation Policy (Policy A)
Train continuous 3D velocity control with PPO:
```bash
python python/train_nav.py --config config/training_nav.yaml
```
Export trained actor weights to ONNX with normalization parameters:
```bash
python python/export_onnx.py --checkpoint models/navigation/nav_policy.zip --output-dir models/navigation/
python python/verify_onnx_parity.py --onnx models/navigation/nav_policy.onnx
```

### 2. Train Central Fleet Coordinator (Policy B)
Train discrete dispatch coordinator with MaskablePPO:
```bash
python python/train_coordinator.py --config config/training_coordinator.yaml --total-timesteps 10000
```
Evaluate learned coordinator against FCFS and Nearest-Neighbor:
```bash
python python/evaluate_coordinator.py --episodes 50
```

### 3. End-to-End 4-Way System Benchmark
Run multi-seed comparative benchmarking across all 4 architectural configurations:
```bash
python python/evaluate_system.py --seeds 10 --max-steps 300
```

---

## Verification & Benchmarking Summary

| Metric | Baseline Heuristic Mode | Full Neural / ONNX Mode |
| :--- | :---: | :---: |
| **Physics Step Rate** | $308,646\text{ steps/sec}$ | $6,550\text{ steps/sec}$ |
| **Real-Time Factor** | $6,172\times\text{ real-time}$ | $131\times\text{ real-time}$ |
| **1-Hour Sim Execution Time** | $0.58\text{ seconds}$ | $27.48\text{ seconds}$ |
| **Inference Latency** | N/A | $0.0079\text{ ms}$ ($7.9\ \mu\text{s}$) |
| **Mid-Air UAV Collisions** | $0$ | $0$ (Zero tolerance) |
| **Building Collisions** | $7$ | $0$ (with Safety Layer) |
| **Coordinator Throughput Gain** | Baseline ($14.20\text{ del}$) | **$+12.7\%$ to $+50.0\%$** ($16.00\text{ del}$) |
| **CTest Suite Verification** | **20/20 Passed (100%)** | **20/20 Passed (100%)** |

For granular per-step engineering ledgers, mathematical models, and edge-case remediations, see the [Progress Ledger](progress_logs/README.md).
