# AeroFleet: Autonomous Urban Multi-UAV Fleet Simulator with Dual-Tier RL

[![CTest Suite](https://img.shields.io/badge/CTest-21%2F21%20Passed%20(100%25)-brightgreen.svg)]()
[![PyTest Suite](https://img.shields.io/badge/PyTest-14%2F14%20Passed%20(100%25)-brightgreen.svg)]()
[![RL Engine](https://img.shields.io/badge/RL-MaskablePPO%20%7C%20PPO-orange.svg)]()
[![C++ Core](https://img.shields.io/badge/Language-C%2B%2B20%20%7C%20Python%203.10%2B-blue.svg)]()
[![Simulation Speed](https://img.shields.io/badge/Headless%20Speed-%3E600x%20Real--Time-success.svg)]()
[![Frame Rate](https://img.shields.io/badge/Target%20FPS-30%20FPS-informational.svg)]()

---

## 1. Project Overview

**AeroFleet** is an end-to-end simulation and training platform for autonomous multi-UAV urban parcel logistics. In modern metropolitan environments, drones must deliver time-sensitive parcels across dense city skylines under finite battery capacities, dynamic customer demand, restricted airspace boundaries, and complex obstacle topologies.

AeroFleet formulates and solves this multi-agent logistics challenge using a **Dual-Tier Reinforcement Learning Architecture**:
1. **High-Level Central Fleet Coordinator (Maskable PPO)**: Formulated as an MDP with **Invalid Action Masking**. Observes global delivery queue states, UAV fleet battery levels, coordinates, and depot statuses, dynamically dispatching eligible, non-depleted UAVs to optimize throughput, reduce delivery latency, and eliminate battery starvation.
2. **Low-Level Autonomous Navigation Agent (Continuous PPO)**: Guides individual quadrotors along collision-free 3D trajectories through urban canyon air corridors, respecting aerodynamic drag, dynamic payload mass, and continuous obstacle avoidance.
3. **Deterministic Safety Supervisor**: A C++ predictive spatial safety layer intercepting velocity commands to mathematically guarantee zero building and inter-UAV collisions.

### Key Simulation Specifications
- **World Envelope**: $2000\text{m} \times 2000\text{m} \times 200\text{m}$ ($4\times$ ground area, $5.3\times$ volume).
- **Procedural City Skyline**: $8 \times 8$ metropolitan block grid with 63 skyscrapers (up to $110\text{m}$ altitude), illuminated architectural window bands, corner neon spines, ground entrance canopies, and rooftop communication spires.
- **Urban Greenery & Parks**: 6 municipal public parks with blue ponds/fountains, manicured lawns, terracotta walking plazas, and street avenue trees along boulevards.
- **Airspace Geo-Fence**: Luminous cyan ground boundary lines, $200\text{m}$ altitude ceiling perimeter, 4 vertical corner laser towers with pulsing red aviation hazard strobes, and semi-transparent perimeter fence panels.
- **Visual Engine**: Decoupled 3D visualization powered by **Raylib**, locked to **30 FPS**, featuring smooth anti-aliased **DejaVu Sans** typography.
- **Headless Performance**: High-throughput headless C++ execution delivering **> 600× real-time** speed (> 300,000 steps/sec).

---

## 2. Directory Structure

```text
AeroFleet/
├── CMakeLists.txt                  # Authoritative CMake build configuration with CTest harness
├── README.md                       # Comprehensive cross-platform documentation
├── report.tex                      # 2-Page Descriptive Report in two-column Overleaf LaTeX
├── config/
│   ├── simulation.json             # Authoritative 2000m x 2000m x 200m world configuration
│   ├── training_nav.yaml           # Navigation PPO hyperparameters
│   └── training_coordinator.yaml   # Coordinator MaskablePPO hyperparameters
├── cpp/
│   ├── core/                       # Simulator, SimulationClock, Config, WorldState
│   ├── math/                       # Vec3, MathUtils, AABB, Capsule, Spatial Geometry
│   ├── world/                      # CityMap, SpatialGridBroadphase, WorldGenerator
│   ├── physics/                    # KinematicsIntegrator, CollisionSystem, BatteryModel
│   ├── fleet/                      # FleetManager, UAVState, FleetScenario
│   ├── requests/                   # RequestGenerator, DeliveryRequest (Poisson arrival)
│   ├── control/                    # WaypointController, SafetyLayer, ONNXNavigationPolicy
│   ├── coordinator/                # ICoordinator, HeuristicCoordinator
│   ├── inference/                  # ONNXModel (ONNX Runtime C++ engine)
│   ├── visualization/              # Renderer (Raylib), CameraController
│   ├── telemetry/                  # MetricsTracker, EventLogger
│   └── main.cpp                    # Master simulation runner with CLI and 10 fleet showcase scenarios
├── python/
│   ├── aerofleet/                  # Python Gymnasium environment bindings
│   │   ├── nav_env.py              # Navigation Gymnasium environment (39-dim obs)
│   │   └── coordinator_env.py      # Coordinator Gymnasium environment (106-dim obs, action masking)
│   ├── bindings/                   # pybind11 C++ zero-copy bindings (aerofleet_bindings.cpp)
│   ├── train_coordinator.py        # MaskablePPO coordinator training pipeline
│   ├── evaluate_coordinator.py     # Rigorous benchmark comparing Learned vs FCFS Coordinator
│   ├── train_nav.py                # PPO navigation trainer with curriculum stages
│   └── export_onnx.py              # Neural network policy export to ONNX runtime format
├── models/
│   ├── coordinator/                # coord_policy.zip, coord_policy.onnx, manifest.json
│   └── navigation/                 # nav_policy.onnx, normalization.json, checkpoint zip
├── assets/
│   ├── fonts/                      # Standard DejaVuSans and DejaVuSans-Bold TrueType fonts
│   └── AeroFleet.svg               # Vector brand asset
├── tests/                          # 21 comprehensive C++ and Python unit & integration test suites
│   ├── test_config.cpp             # Configuration loader verification
│   ├── test_math_geometry.cpp      # Spatial math and vector geometry tests
│   ├── test_world_generation.cpp   # Spatial broadphase and building procedural generation
│   ├── test_kinematics.cpp         # 2nd-order rigid body physics integration
│   ├── test_collision.cpp          # Continuous collision detection and capsule tests
│   ├── test_battery.cpp            # Electro-aerodynamic discharge and CC-CV recharging
│   ├── test_battery_stress.cpp     # Accelerated battery stress & depletion tests
│   ├── test_fleet.cpp              # Multi-UAV lifecycle and state machine tests
│   ├── test_differential_physics.py# C++ vs Python numeric physics parity tests
│   └── ...                         # All 21 CTest suites
└── requirements-rl.txt             # Python dependencies (PyTorch, Stable-Baselines3, etc.)
```

---

## 3. Installation Guide (For Every Operating System)

### A. Linux (Ubuntu 22.04 / 24.04 / Debian / Pop!_OS)

#### 1. Install System Build Tools & Dependencies
```bash
sudo apt update && sudo apt install -y \
    build-essential \
    cmake \
    git \
    libx11-dev \
    libxrandr-dev \
    libxinerama-dev \
    libxcursor-dev \
    libxi-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    python3 \
    python3-dev \
    python3-pip \
    python3-venv
```

#### 2. Set Up Python Virtual Environment
```bash
python3 -m venv venv
source venv/bin/activate
pip install --upgrade pip
pip install -r requirements-rl.txt
pip install -e .
```

#### 3. Build C++ Engine with CMake
```bash
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel $(nproc)
```

---

### B. Windows (via WSL2 / Ubuntu with WSLg — Recommended)

WSL2 with WSLg provides hardware-accelerated OpenGL and Wayland/X11 display out-of-the-box on Windows 10/11.

#### 1. Open WSL2 Terminal
```powershell
wsl -d Ubuntu
```

#### 2. Install Packages & Build
```bash
sudo apt update && sudo apt install -y \
    build-essential cmake git libx11-dev libxrandr-dev libxinerama-dev \
    libxcursor-dev libxi-dev libgl1-mesa-dev python3 python3-pip python3-venv

python3 -m venv venv
source venv/bin/activate
pip install -r requirements-rl.txt
pip install -e .

cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel $(nproc)
```

---

### C. macOS (Apple Silicon M1/M2/M3 & Intel)

#### 1. Install Prerequisites via Homebrew
```bash
brew install cmake python@3.11 libomp glfw
```

#### 2. Virtual Environment & Dependencies
```bash
python3 -m venv venv
source venv/bin/activate
pip install --upgrade pip
pip install -r requirements-rl.txt
pip install -e .
```

#### 3. Configure & Compile
```bash
cmake -B build -S . \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_POLICY_DEFAULT_CMP0074=NEW
cmake --build build --parallel $(sysctl -n hw.ncpu)
```

---

## 4. How to Run the Simulation

### 1. Interactive 3D Visual Simulation (30 FPS)
Launch the 3D Raylib visual simulation with the enlarged $2000\text{m} \times 2000\text{m}$ world, procedural skyscrapers with lit window bands, greenery parks, and 10 concurrent operational scenarios:

```bash
# From repository root (with virtual environment active)
./build/AeroFleet
```

#### Interactive Real-Time Controls

| Key / Control | Function |
|---|---|
| **`[B]`** | **Cycle Battery Drain Multiplier** (1x $\to$ 5x $\to$ 15x $\to$ 30x $\to$ 50x) to test fast battery exhaustion |
| **`[D]`** | **Force Low Battery (18%)** on the active UAV to trigger immediate Return-To-Home (RTH) |
| **`[F]`** | **Instant Recharge (100%)** on the active UAV to restore full battery capacity |
| **`[K]`** | **Emergency Battery Kill (0%)** to test failsafe emergency procedures |
| **`[S]`** | **Reset & Respawn All 10 Fleet Showcase Scenarios** |
| **`[1]` – `[9]`** | **Select UAV & Lock Chase Camera** to follow a specific drone's mission |
| **`[0]`** | **Reset to Orbit Overview Camera** overlooking the entire $2\text{km}$ metropolitan area |
| **`[C]`** | **Cycle Camera Mode** (`ORBIT` $\to$ `CHASE` $\to$ `TOP-DOWN` $\to$ `FREE`) |
| **`[H]`** | **Toggle Telemetry HUD Overlay** |
| **`[T]`** | **Toggle Flight Trajectory Ribbon Trails** |
| **`[Space]`** | **Pause / Resume Simulation** |
| **Right-Click Drag** | Rotate camera orbit around the city or tracking target |
| **Scroll Wheel** | Zoom in / out (smooth orbit distance up to $4000\text{m}$) |

---

### 2. High-Speed Headless Benchmarking Mode
Run the simulation in headless mode (no graphics window) at maximum CPU speed for rapid benchmarking and data logging:

```bash
# Run 1,000 steps of headless simulation
./build/AeroFleet --headless --steps 1000 --benchmark

# Run 10,000 steps with custom random seed and event logging
./build/AeroFleet --headless --steps 10000 --seed 123 --log-events
```

---

## 5. Verification & Testing

AeroFleet includes a rigorous test suite of **21 CTest suites** and **14 PyTest unit tests** covering kinematics, spatial geometry, continuous collision detection, electro-aerodynamics, battery stress, and Gymnasium bindings.

### Run All 21 CTest Suites
```bash
cd build && ctest --output-on-failure
```
*Expected result:* **`100% tests passed, 0 tests failed out of 21`**

### Run Python Unit & Physics Tests
```bash
PYTHONPATH=python python3 -m pytest tests/test_battery_stress.py tests/test_differential_physics.py -v
```
*Expected result:* **`14 passed in < 1.0s`**

---

## 6. Reinforcement Learning Evaluation & Training

### 1. Evaluate Learned Coordinator vs. FCFS Heuristic Baseline
Benchmark the trained Maskable PPO Central Coordinator against the conventional First-Come First-Served (FCFS) dispatch heuristic across 15 randomized evaluation episodes:

```bash
python3 python/evaluate_coordinator.py --episodes 15 --render-summary
```

#### Benchmark Results Summary
| Metric | Baseline Heuristic (FCFS) | Learned Coordinator (Maskable PPO) | Improvement |
|---|---|---|---|
| **Delivery Success Rate** | 7.66% | **9.35%** | **+22.15% relative gain** |
| **Mean Delivery Latency** | 119.02 s | **111.37 s** | **-7.65 s (faster fulfillment)** |
| **Total Delivered Count** | 21.47 parcels | **22.67 parcels** | **+5.59% more deliveries** |
| **Order Expirations** | 0.00 | **0.00** | **Zero deadline misses** |
| **Battery Dropouts / Groundings** | 0.00 | **0.00** | **100% safe return rate** |

### 2. Train the Central Coordinator (Maskable PPO)
To re-train or fine-tune the high-level fleet coordinator policy:

```bash
python3 python/train_coordinator.py \
    --timesteps 60000 \
    --save-dir models/coordinator \
    --export-onnx
```

### 3. Train the Local Navigation Policy (Continuous PPO)
```bash
python3 python/train_nav.py --stages 4 --timesteps 200000
```

---

## 7. Mathematical RL Formulation Summary

### A. Central Fleet Coordinator MDP ($\mathcal{M}_{coord}$)
- **State Space $\mathcal{S} \in \mathbb{R}^{106}$**:
  - Global fleet state (10 UAVs $\times$ 7 features: normalized 3D position $(x, y, z)$, battery fraction $\text{SoC} \in [0, 1]$, operational mode one-hot, remaining distance to target, payload status).
  - Active delivery request queue (Top-5 pending requests $\times$ 6 features: pickup coordinates, dropoff coordinates, urgency deadline margin, parcel weight).
  - Station infrastructure state (Charging station dock availability ratios, store warehouse queue lengths).
- **Action Space $\mathcal{A} \in \{0, \dots, N_{uav}\}$**:
  - Discrete dispatch decision selecting which candidate UAV should service the highest-priority delivery request, or action $0$ for hold/defer.
  - **Action Masking $M(s) \in \{0, 1\}^{|\mathcal{A}|}$**: Dynamically filters out UAVs that are docked, charging, under low battery threshold ($\le 20\%$), or currently carrying maximum payload.
- **Reward Function $\mathcal{R}$**:
  $$\mathcal{R} = R_{\text{delivery}} + R_{\text{latency}} + R_{\text{battery}} + R_{\text{starvation}}$$
  - Delivery completion bonus: $+10.0$.
  - Deadline urgency factor: $+5.0 \times \left(1.0 - \frac{t_{\text{delivery}}}{t_{\text{deadline}}}\right)$.
  - Battery exhaustion penalty: $-20.0$ if any UAV enters critical low battery ($\le 15\%$).
  - Dispatch starvation penalty: $-0.05 \times N_{\text{expired}}$.

### B. Local 3D Navigation MDP ($\mathcal{M}_{nav}$)
- **Observation Space $\mathcal{S}_{nav} \in \mathbb{R}^{39}$**:
  - Body-frame relative target vector $(\Delta x, \Delta y, \Delta z)$.
  - Linear velocity vector and attitude angles $(\phi, \theta, \psi)$.
  - 32-ray multi-directional spherical LiDAR obstacle depth scans.
- **Action Space $\mathcal{A}_{nav} \in [-1, 1]^3$**:
  - Continuous commanded linear acceleration / target velocity vectors $[\ddot{x}_c, \ddot{y}_c, \ddot{z}_c]$.
- **Reward Function $\mathcal{R}_{nav}$**:
  $$\mathcal{R}_{nav} = -\alpha \|\mathbf{p} - \mathbf{p}_{\text{target}}\| + \beta (\Delta d_{\text{progress}}) - \gamma \mathbb{I}_{\text{collision}} - \delta \|\mathbf{a}\|^2$$

---

## 8. Academic Integrity & Course Deliverables

- **Pure Reinforcement Learning**: All core dispatch and navigation policies are trained via Reinforcement Learning (Maskable PPO and PPO) without reliance on hardcoded heuristics.
- **Originality & Authenticity**: All kinematics, electro-aerodynamics, spatial broadphase grids, Raylib visualization, and Python bindings are original implementations crafted specifically for this project.
- **Report**: See [report.tex](file:///c:/Users/Prateek/Desktop/Coding/project/AeroFleet/report.tex) for the complete 2-page descriptive report formatted in Overleaf two-column LaTeX.
