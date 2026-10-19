# AeroFleet Architecture and Flow Review

**Review date:** 2026-10-19  
**Scope:** Intended design, implementation roadmap, current C++ runtime,
pybind11 interface, Gymnasium wrapper, and progress records.

## Project intent

AeroFleet is intended to be a hierarchical RL delivery simulation, not only a
visual interface. The individual UAV navigation policy learns how to move
safely toward a waypoint. The higher-level fleet coordinator selects feasible
request/UAV assignments. The C++ engine remains authoritative for the world,
delivery lifecycle, energy, kinematics, and collision outcomes. Python calls
that same engine from a Gymnasium environment while Python RL code collects
rollouts and updates both policies. No RL training loop or optimizer runs in
C++. Trained actors may later be exported and run for inference in a C++
application. Heuristic navigation and dispatch are baselines and useful
fallbacks during development; both learned policies remain RL objectives.

This intent is consistent across the complete design guide, the canonical
20-step plan, and the implementation. The project has a deterministic Stage 1
navigation task and a randomized route/heading distribution for single-UAV
experiments. Initial PPO runs did not produce a successful policy, so this is
training infrastructure rather than a demonstrated learned capability. It is
not yet ready for the full navigation curriculum or coordinator training.

## End-to-end flow and current state

| Stage | Intended responsibility | Current state |
|---|---|---|
| C++ simulation | Deterministic world, orders, fleet state, motion, energy, collisions | Implemented; Debug/Release suites pass |
| Baselines and visualization | Exercise delivery flow and inspect authoritative snapshots | Heuristic baselines, renderer, and telemetry implemented |
| Python boundary | Expose simulator state and actions through pybind11 | Built and tested for Python 3.14 in this review |
| Navigation environment | Give a navigation policy a controlled, repeatable task | Stage 1 open-space waypoint scenario implemented; full-fleet mode remains available |
| Navigation training | Train and evaluate shared UAV policy through a curriculum | Python PPO trainer/evaluator implemented and validated; initial checkpoints have 0% success; no usable policy yet |
| Native navigation inference | Run the exported policy in C++ with a measured safety layer | Not implemented |
| Coordinator environment/training | Learn fleet dispatch against a frozen navigator | Not implemented; confirmed RL objective |
| System evaluation | Compare heuristic and learned combinations on paired seeds | Not implemented |

## Navigation environment status and remaining curriculum work

### 1. Stage 1 waypoint task is implemented; later scenarios remain pending

By default, `AeroFleetNavEnv.reset()` now creates a reproducible one-UAV,
obstacle-free waypoint task with an active reachable target. Training and
evaluation use seeded random routes and initial heading independent of the
goal. The stochastic delivery simulator remains available with
`scenario="full_fleet"`. The goal-reaching integration test verifies success
termination.

The next curriculum stages still need deterministic setup and task-level
success criteria for:

1. the waypoint task with buildings and boundaries;
2. battery and pickup/delivery transitions;
3. multi-UAV traffic;
4. full dynamic demand and fleet dispatch.

Stage 1 is implemented, and Python PPO training/evaluation entry points now
target it. Random-action episode stability remains an API smoke check and does
not demonstrate learned obstacle avoidance or delivery skills. Training has
run, but earlier schema-v2/aligned-heading experiments achieved zero waypoint
success. The audited pipeline uses a corrected discount horizon, randomizes
initial heading, and has not yet produced a usable policy.

### 2. Observation schema v3 adds state and finite normalization

The 87-value schema includes body-frame velocity and acceleration, target
displacement and distance, battery, payload, speed, lidar, three relative
neighbors, heading sine/cosine, and a UAV-mode one-hot vector. Relative
kinematics share the body frame with yaw-relative lidar. Python scales
configuration-dependent features into a finite Box and exposes the schema
version, feature order, frame, and scales.

Validate every feature against configured extrema in obstacle, delivery, and
multi-UAV scenarios before freezing the schema for model export.

### 3. Finite bounds and preprocessing are implemented

The Gymnasium observation Box now has finite per-feature bounds, and
configuration-derived scales are available through `env.observation_schema`.
The standard checker passes without the prior infinite-bound warnings.

Serialize the schema and preprocessing values with every trained model and
reuse them during C++ inference.

### 4. Stage 1 reward and episode outcomes are explicit

The open-waypoint task uses a fixed target and a C++ reward based on bounded
distance progress, a per-step cost, a success bonus, and a failure penalty.
Success means reaching within 2 m; collision/emergency and battery depletion
terminate as failures; the Python time limit truncates. Target-switch shaping
does not apply to this fixed-target task. Full-delivery reward semantics
remain to be defined for coordinator training.

Add task-level reward and termination tests as obstacle and multi-stage
delivery scenarios are introduced.

### 5. Safety is currently detection, not action shielding — deployment gate

The simulator applies a command, integrates it, then performs swept collision
detection and can move a collided UAV into emergency mode. The planned
short-horizon Safety Layer that inspects and overrides unsafe actions is not
implemented. This is acceptable for early training experiments if collisions
are task failures, but it must not be described as a safety shield or used as
a deployment guarantee. Implement and measure the intervention layer before
native policy deployment.

### 6. The interactive runner and training simulator have separate orchestration

The interactive C++ runner composes simulation modules in `cpp/main.cpp`,
while the Python binding advances `cpp/core/Simulator`. Both use core modules,
but separate orchestration can produce different update order or behavior as
features evolve. This does not invalidate the current Python interface tests;
it means those tests do not prove that a policy trained through the binding
will behave identically in the interactive application.

**Required before deployment:** either route both front ends through the same
simulation orchestration, or add seeded end-to-end parity tests for mission
events, update ordering, and policy-controlled actions.

## Flow corrections for the remaining roadmap

The current high-level sequence is sound, with these explicit prerequisites:

1. Step 14 now includes Stage 1 open-waypoint training scaffolding and tests.
2. Extend the deterministic curriculum through obstacles, delivery/battery,
   and multi-UAV traffic. Extend Step 15 to verify adapter/action/observation
   parity against direct C++ calls; it cannot independently prove physics when
   both paths call the same C++ core.
3. Train navigation by curriculum, evaluate on held-out seeds/maps, and freeze
   a selected checkpoint only after it beats named baselines on agreed metrics.
4. Export the actor together with feature schema, normalization, action
   scaling, and training configuration. Verify inference parity.
5. Add the C++ inference adapter and proactive safety layer; run closed-loop
   tests before using the policy in the delivery fleet.
6. Train the coordinator policy after the navigation policy and environment
   are stable/frozen. Keep heuristic dispatch as a paired baseline. Evaluate
   the heuristic/heuristic, RL-navigation/heuristic-dispatch, and
   RL-navigation/RL-coordinator combinations on identical demand seeds.
7. Run final stress/performance tests and document limits before calling the
   system integrated.

## Documentation and implementation consistency issues corrected

- The root README was an incomplete outline with an empty training-architecture
  section; it now states the intended hierarchy, current status, and links to
  the canonical plans.
- `development plan.md` duplicated the canonical plan with a stale status; it
  now redirects to `development_plan.md`.
- The design guide now distinguishes its target architecture from implemented
  and trained behavior.
- Replaced 33 broken Pandoc TeX remnants in the design guide with readable
  plain-text equations.

## Confirmed learning scope

The user confirmed that **both** the UAV navigation policy and central fleet
coordinator are to be trained with RL as part of the RL course project. The
heuristic navigator and dispatcher remain baselines for staged development
and paired evaluation; they are not the final learning scope.
