# Step 14: Gymnasium Navigation Environment

## Status

**Completed.** The single-agent Gymnasium wrapper, native bindings, and focused integration coverage have been built and exercised under Python 3.14.4 with Gymnasium 1.4.0, NumPy 2.5.4, pytest 9.1.1, and pybind11 3.1.0.

## Contract implemented

- The learning policy controls UAV 0; remaining mission-active UAVs use the existing heuristic controller through the simulator's short action-vector fallback.
- Observation: the C++ simulator's fixed 72-element `float32` vector.
- Action: `Box(-1, 1, shape=(3,))`, interpreted as normalized desired `(vx, vy, vz)`.
- X/Y are scaled by configured maximum horizontal speed; Z is scaled by configured maximum vertical speed. The C++ kinematics integrator remains authoritative for acceleration, speed, and altitude bounds.
- Reward: use the C++ simulator's per-UAV transition reward.
- Termination: controlled UAV enters `EMERGENCY`. Truncation: configured episode step limit, with terminal state taking precedence if both happen together.
- Seed: explicit Gymnasium seed is forwarded to the C++ simulator; when omitted, a seed is drawn from Gymnasium's seeded RNG.

## Files

- `python/aerofleet/nav_env.py`: `AeroFleetNavEnv` wrapper with Gymnasium reset/step API and input/output validation.
- `python/bindings/aerofleet_bindings.cpp`: exposes configured speed limits so Python uses the same values as the authoritative C++ configuration.
- `requirements-rl.txt`: Python Gymnasium and NumPy dependencies; native extension remains a separate CMake build.
- `tests/test_nav_env.py`: unit tests for action scaling/validation plus integration tests for seeded reset, observation/reward transition, episode truncation, and Gymnasium's environment checker.
- `CMakeLists.txt`: registers the environment tests with CTest only when the native binding and Gymnasium/NumPy/pytest dependencies are available.
- `tests/test_bindings.py`: verifies the newly exposed C++ horizontal/vertical speed-limit properties.

## Validation record (2026-10-19)

- pybind11 extension built successfully in both Debug and Release configurations against Python 3.14.4.
- Debug CTest: **16/16 passed**, including 14 C++ suites, `PythonBindingTests`, and `GymnasiumNavigationTests`.
- Release CTest: **16/16 passed** with the same coverage.
- Binding API test script: **8/8 checks passed**, including NumPy conversion, direct action stepping, reproducibility, and throughput.
- Gymnasium pytest suite: **10/10 passed**, including `check_env` and a randomized 100-episode smoke test.
- Python syntax compilation and CRLF-aware `git diff --check` passed.
- `check_env` emits two warnings because the observation Box currently has `-inf`/`+inf` bounds. These are warnings, not checker failures; finite per-feature bounds and reward/observation normalization should be reviewed before PPO training.
