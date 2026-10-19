# Step 16: Navigation Policy Training and Evaluation

## Status

**Pipeline implemented; successful learning is not yet demonstrated.** Training and optimization run in Python using Gymnasium and Stable-Baselines3 PPO. The pybind11 C++ module only advances the authoritative simulator and returns state, reward, and terminal flags. No C++ training is used. The current task is obstacle-free waypoint steering with randomized routes and initial yaw; obstacle avoidance, delivery, multi-UAV navigation, and coordinator training are future stages.

## Audited transition and optimization flow

1. `AeroFleetNavEnv.reset(seed)` seeds Gymnasium's RNG and C++ simulator, samples a start/target route, and samples yaw independently of target bearing for the training/evaluation configuration.
2. The 87-value schema v3 observation is normalized to finite `float32` bounds. Position features in the body frame use the map horizontal diagonal; yaw is represented by sine/cosine. The model manifest stores the ordered fields and scales.
3. PPO emits normalized desired velocity `[forward, left, up]` in `Box(-1, 1, (3,))`. Python scales horizontal/vertical components to configured velocity limits and rotates the body-frame command into world coordinates using the current yaw. C++ retains acceleration, speed, altitude, battery, and collision authority.
4. Each policy action is held for the C++ physics substeps. The C++ reward compares Euclidean target distance before and after the action, clips distance progress by the conservative speed bound `(v_horizontal + v_vertical) * dt_effective`, applies a `-0.01` step cost, `+10` success reward at 2 m, and `-10` on collision/emergency/battery failure. `dt_effective` is the actual rounded physics-substep duration, not nominal policy dt. C++ terminates on success/failure; Python truncates on the time limit only when no terminal event happened.
5. SB3 PPO collects vectorized Gymnasium transitions, computes returns/GAE, and updates the policy in Python. The current defaults are 1,024 steps per env per rollout, batch 64, 10 epochs, learning rate `3e-4`, `gamma=0.999`, `gae_lambda=0.99`, and clip range `0.2`.
6. Before training, the exact randomized-heading/body-action environment is checked with both Gymnasium and Stable-Baselines3 validators. During training, deterministic evaluations use the same fixed seed set at every checkpoint; the checkpoint ranking is success rate, lower collision rate, lower mean final distance, then mean return. A final evaluation is forced if the last update did not land on the periodic evaluation boundary.
7. `evaluate_nav.py` checks the model's action frame, task version, observation schema/scales, and model spaces against the current environment. It evaluates deterministic actions on a separately seeded held-out route set and reports success, return, episode length, collisions, battery use, and final distance.

## Mathematical and logical review

- For the default `dt_policy=0.10s` and `dt_physics=0.02s`, `dt_effective=0.10s`. If a configuration ratio rounds to a different number of substeps, reward progress bounds now use the duration actually integrated.
- With default limits `v_h=15m/s`, `v_z=5m/s`, clipped progress is at most 2m per action. The nonterminal reward therefore lies in `[-0.21, 0.19]`; a success transition lies in `[9.79, 10.19]`, and a failure transition in `[-10.21, -9.81]` (assuming one terminal outcome). The progress bound is conservative because Euclidean displacement is no greater than `(v_h+v_z)dt`.
- `gamma=0.99` at 0.1s/action gives a reward half-life of about 6.9s, too short for routes that take tens of seconds. The new `gamma=0.999` gives a half-life of about 69s. `gae_lambda=0.99` lengthens advantage traces relative to the old 0.95 setting.
- Distance-progress shaping aligns with reaching the fixed waypoint, but it is not the formal policy-invariant potential shaping term for a discounted MDP. The intended objective is waypoint arrival and the shaping is an explicit task reward, not a claim of preserving an underlying optimal policy.
- The action Box maps square corners through the C++ horizontal-speed clamp, so several large diagonal commands saturate to the same velocity envelope. This is valid but creates redundant actions; revisit the action parameterization if PPO shows persistent saturation.
- The prior route sampler pointed yaw directly at the goal. A constant-forward command succeeded on all 20 sampled routes, so those runs did not validate steering. Training/evaluation now sample yaw independently. A target-direction diagnostic controller succeeds on 20/20 current routes (mean 318 steps), confirming that the randomized task and body-frame action transform are solvable. This controller is a simulator sanity check, not an RL result.

## Training/evaluation evidence

- Earlier schema-v2, target-aligned PPO runs used `gamma=0.99`. A 50k pilot and a continuation from a 40k checkpoint to 140k timesteps both had 0% success. A held-out 100-episode evaluation of the selected aligned-heading model also had 0% success; all episodes truncated at 2,000 steps. These are superseded failed experiments and must not be presented as trained policies.
- A current-contract 2,048-step PPO smoke run (seed 39, schema v3, randomized heading, `gamma=0.999`) exercised validation, rollout/update, checkpointing, seeded evaluation, saving, and loading. Its internal fixed-set success rate was 0% at both 1,024 and 2,048 steps.
- Held-out evaluation of that smoke checkpoint on seeds 500–507: 0/8 success, 2,000 mean steps, 0 collision episodes, and 511.0m mean final distance. This is expected to be too little data to judge convergence; it is evidence that the evaluator catches poor behavior.
- The evaluator and trainer both load/validate the manifest and schema. Schema-v2 checkpoints and old task-version checkpoints are rejected instead of silently evaluated on a different contract.

## Verification

- Debug CMake build completed; CTest passed **16/16**, including pybind11 and Gymnasium integration.
- Release CMake build completed; CTest passed **16/16**.
- `PYTHONPATH=python ... -m pytest tests/test_nav_env.py tests/test_bindings.py`: **24 passed**. Added coverage for randomized route/yaw reproducibility, exact success reward math, actual timestep reward clipping, and direct C++ versus body-action wrapper transition/reward/termination parity.
- Python byte-compilation and both CLI `--help` paths passed.
- Stable-Baselines3 `check_env` ran automatically on the training environment in the short end-to-end run.
- A separate 320-timestep run with a 256-step evaluation interval verified that the callback performs a final evaluation when the final update does not align with the interval.
- The current evaluator loaded the saved PPO checkpoint with the live Gymnasium env, checked the saved 87-feature schema and model spaces, and produced the held-out JSON report.

## Remaining work / gate to advance

1. Run a materially longer current-contract PPO experiment with multiple seeds and enough paired evaluation episodes to judge learning; do not advance curriculum based on the 2,048-step smoke run.
2. Diagnose training curves and action saturation if waypoint success remains low; tune PPO/reward/task only from measured evidence.
3. Only after Stage 1 success is reliable, add obstacle/boundary and delivery/battery stages.
4. Build the Python Gymnasium environment and RL policy for the central coordinator as a separate RL workstream; it is not implemented yet. Both UAV and coordinator are RL course objectives.
5. Keep ONNX/C++ inference after policies are trained and validated; those are deployment steps, not RL training steps.
