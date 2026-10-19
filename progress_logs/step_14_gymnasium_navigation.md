# Step 14: Gymnasium Navigation Environment

## Status

**Stage 1 environment implemented; policy learning not yet demonstrated.** The default Gymnasium reset remains a deterministic one-UAV waypoint task. Training/evaluation use randomized route positions and initial heading. The stochastic fleet simulator remains selectable for integration checks. Current observation schema is v3; earlier v2 validation below is retained as a historical record.

**Scope boundary:** Stage 1 establishes a learnable fixed-target task and observation/reward contract. Obstacle/boundary, battery and delivery lifecycle, multi-UAV traffic, and coordinator-training environments remain future curriculum stages.

## Contract implemented

- The Stage 1 policy controls the only active UAV. In `full_fleet` mode, the wrapper controls UAV 0 and remaining mission-active UAVs use the existing heuristic controller through the simulator's short action-vector fallback.
- Observation: schema v3, 87 ordered features. Kinematics, target displacement, and nearby-UAV state use the UAV body frame; orientation is encoded as heading sine/cosine. Horizontal body-relative positions are normalized by the map diagonal so arbitrary heading cannot exceed the declared finite bounds.
- Feature groups: velocity (3), acceleration (3), relative target (3), target distance (1), battery fraction (1), payload fraction (1), speed fraction (1), 44 lidar distances, three neighboring UAV states (18), heading (2), and UAV mode one-hot (10).
- Python normalizes configuration-dependent values into a finite Box and exposes `observation_schema` (version, feature order, coordinate frame, and scales).
- Action: `Box(-1, 1, shape=(3,))`, interpreted as normalized desired `(vx, vy, vz)`.
- X/Y are scaled by configured maximum horizontal speed; Z is scaled by configured maximum vertical speed. The C++ kinematics integrator remains authoritative for acceleration, speed, and altitude bounds.
- Stage 1 reward: `-0.01 + 0.1 * clamp(d_previous - d_current, -dt_effective * (v_horizontal + v_vertical), +dt_effective * (v_horizontal + v_vertical))`; reaching within 2 m adds 10; collision/emergency or battery depletion subtracts 10. `dt_effective = rounded_physics_substeps * dt_physics`, matching the duration actually integrated by C++.
- Termination: waypoint success, collision/emergency, or battery depletion. Truncation: configured episode step limit, with terminal state taking precedence.
- Default reset selects `open_waypoint`, starts with an active reachable target, disables request arrivals and fleet dispatch, and runs one UAV. Select `options={"scenario": "full_fleet"}` for the stochastic delivery simulator.
- Seed: explicit Gymnasium seed is forwarded to the C++ simulator; when omitted, a seed is drawn from Gymnasium's seeded RNG.
- Training/evaluation random resets sample initial yaw uniformly from `[-pi, pi]`, independently of target bearing; the sampled yaw uses Gymnasium's episode RNG and is passed through pybind11 to C++.

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
- Gymnasium pytest suite: **13/13 passed**, including the standard checker, deterministic seeded reset, a successful waypoint episode, full-fleet observation bounds, and 100 short random-action smoke episodes.
- `check_env` passes without the previous infinite-bound warnings.

## Remaining curriculum work

1. Add controlled building/boundary scenarios, then battery and pickup/delivery transitions.
2. Add multi-UAV traffic and dynamic demand while keeping the learned navigation action isolated from the coordinator policy.
3. Extend Step 15 with direct binding-versus-wrapper action/observation/reward parity checks and independent expected-trajectory physics checks.
4. Train and evaluate Stage 1 PPO before advancing the curriculum; random-action smoke episodes are API checks, not policy performance evidence.

## Strict training-contract reassessment (2026-10-19)

- Schema v3 corrects the horizontal normalization bound for body-frame relative positions. Schema v2 models must not be evaluated against the v3 environment.
- The prior randomized route generator initialized yaw toward the target. A constant-forward action then succeeded on all 20 sampled routes, showing that the experiment did not exercise steering. Training/evaluation now randomize initial heading; this is recorded as navigation task version 2.
- The C++ reward remains bounded progress shaping plus explicit success/failure outcomes. The previous progress clip used nominal `dt_policy`; C++ integrates a rounded number of fixed physics steps. The clip now uses the actual integrated duration, preserving its intended conservative bound for non-integral timestep ratios.
- Direct binding/Gymnasium parity covers body-frame action rotation, observation, reward, and termination. Gymnasium and SB3 validators run on the same randomized-heading configuration used for training.
- Earlier training runs (schema v2, aligned initial heading, `gamma=0.99`) achieved 0% success and are superseded diagnostics, not usable checkpoints. Under the current 0.1-second policy interval, `gamma=0.99` halves a terminal outcome's weight after about 6.9 seconds; the trainer default is now `gamma=0.999` (about 69 seconds), with GAE lambda 0.99. Revalidation and successful held-out learning remain open gates.
