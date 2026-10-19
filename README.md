# AeroFleet

AeroFleet is a research simulator for urban UAV delivery with two planned
reinforcement-learning policies:

1. A shared navigation policy controls an assigned UAV's local motion toward
   its current pickup, delivery, or charging waypoint.
2. A fleet coordinator chooses which feasible request to assign to which UAV.

C++ is the authoritative simulation layer for world state, UAV dynamics,
requests, energy, and collisions. RL algorithms and training loops run in
Python through Gymnasium, calling the C++ simulator through pybind11. C++ is
not used to train the policies; the intended final runtime may run exported
policy inference from C++ and render read-only simulation snapshots with
raylib.

## Current implementation

The C++ simulator, heuristic navigation and dispatch baselines, delivery
lifecycle, visualization, telemetry, pybind11 module, and a single-UAV
Gymnasium environment are implemented. The environment defaults to a
deterministic open-space waypoint task with a finite, normalized 87-feature
observation. Debug and Release validation each pass 16/16 CTest tests; the
Gymnasium checker passes without observation-bound warnings.

No RL agent has been trained yet. The waypoint task is Stage 1 only; obstacle,
battery/delivery, and multi-UAV curriculum stages remain. Both UAV navigation
and central fleet coordination are planned RL policies.

See [the progress ledger](progress_logs/README.md), [the implementation plan](progress_logs/development_plan.md), [the complete design guide](progress_logs/AeroFleet_Complete_Project_Design.md), and [the architecture review](progress_logs/architecture_review.md).

## Build and test

The C++ project uses CMake. The optional Python module requires matching
Python development headers and pybind11. Gymnasium environment tests also
require the packages listed in [requirements-rl.txt](requirements-rl.txt).

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

Python RL training, model export, and ONNX Runtime deployment are future
milestones. See the progress ledger for their status.
