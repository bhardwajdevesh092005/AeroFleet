# Post-Pull Validation and Baseline Fixes

## Scope

Validated the merged code at commit `9caf0d5` and followed up on the headless mission behavior. The Step 14 wrapper now defines a provisional RL action contract: a normalized three-vector mapped to desired 3D velocity. This wrapper still needs Python runtime validation; no RL policy has been trained.

## Changes made

- Request generation now raises ground-level store/customer coordinates to the configured delivery flight altitude while preserving their horizontal coordinates.
- The main runner and `Simulator` expire waiting requests using the request deadline or configured pending timeout, record the expiration, and remove assigned/terminal requests from the pending queue.
- Dispatch and expiration events are now recorded by the master simulator.
- Heuristic navigation has a separate short-horizon fleet-traffic action path for deterministic UAV separation; it does not change the underlying `VelocityCommand` interface.
- Assignments are requeued after collision when their request is still recoverable, including requests already in transit. Collided UAVs remain in emergency mode.
- CMake now discovers installed Python/pybind11 packages and uses the detected Python ABI instead of hard-coding CPython 3.14. Python binding compilation remains optional.
- The simulator test keeps C `assert` checks enabled in Release builds.
- Step 14 adds a single-agent Gymnasium wrapper, exposes C++ speed limits to Python, converts metrics to deterministic plain-Python info data, and tests action validation/scaling, seeded resets, transition outputs, truncation, `check_env`, and random rollouts.

## Verification

- Before Step 14, the post-pull Debug build passed **14/14 C++ suites**.
- Debug and Release configure/builds after Step 14 completed; CTest passed **16/16 tests** in each configuration, including the native binding and Gymnasium tests.
- The Gymnasium suite passed **10 tests**, including the environment checker and 100 randomized short episodes. The direct binding script passed all **8 checks**.
- Python source syntax compilation and CRLF-aware `git diff --check` passed.
- Seed 42, 10 UAVs, 100,000 physics steps (2,000 simulated seconds): **64 deliveries**, **336 expirations**, 8 building collisions, 2 UAV collisions, 3.00% deadline success rate; runtime about 2.05 seconds in this Debug build.
- Before these fixes, the same run had zero deliveries, zero expirations, 7 building collisions, and 2 UAV collisions.
- One-UAV, 100-second smoke scenario: one on-time delivery and zero collisions.

## Remaining issues

- The ten-UAV baseline still has building collisions and poor deadline success. The APF controller is not yet a reliable fleet-scale benchmark.
- The Gymnasium checker reports warnings for infinite observation-space bounds. Tight finite bounds and feature/reward normalization should be reviewed before PPO training.
- Step 14's normalized desired-velocity action is implemented and tested; no RL model has been trained.
