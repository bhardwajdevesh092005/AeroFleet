# AeroFleet: Complete System Design and Implementation Guide

## 1. Project overview

AeroFleet is a reinforcement-learning-based simulation of an urban UAV
delivery fleet. Delivery requests continuously arrive from stores and
customers. A central coordinator chooses which UAV should serve each
request. Each UAV navigates from its current location to a pickup point
and then to the destination while respecting battery limits, avoiding
buildings and other UAVs, and eventually returning to a charging station
when needed.

The final application should run as one coherent simulation:

1.  A city map and its physical state exist in a C++ simulation engine.
2.  A request generator continuously creates delivery requests.
3.  The coordinator selects a feasible request/UAV assignment.
4.  The assigned UAV follows a target sequence using a trained
    navigation policy.
5.  Physics and collision detection validate the movement and update the
    world.
6.  The delivery state, battery, fleet status, and performance metrics
    are updated.
7.  raylib renders the latest world state in a live 3D view.
8.  The process repeats until the user stops the simulation.

The goal is not merely to show drones flying. The goal is to build a
reproducible environment, train useful policies, deploy them into a C++
runtime, and demonstrate measurable improvements over heuristic
baselines.

## 2. Core architectural decision

Use **C++ as the authoritative simulator** and **Python as the
reinforcement-learning/training layer**.

-   C++ owns world geometry, simulation time, UAV state, motion
    integration, collision detection, battery updates, delivery state
    transitions, and the final runtime.
-   Python uses Gymnasium to define the learning interface and uses an
    RL library to train policies.
-   A binding layer such as pybind11 allows Python to call the same C++
    simulation library during training.
-   The trained neural-network actor is exported to ONNX and loaded in
    the C++ runtime using ONNX Runtime.
-   raylib reads snapshots of the C++ world and renders them. Rendering
    must not determine physics or change the authoritative state.

This arrangement reduces the chance that the Python training environment
and the C++ deployment environment disagree about what constitutes a
collision or a successful delivery.

### Important distinction: Gymnasium is not a physics engine

Gymnasium defines a standard interface for reinforcement-learning
environments. It does not automatically generate a city, move drones,
detect collisions, or enforce battery rules. Those behaviors must be
implemented by AeroFleet.

A Gymnasium environment usually implements:

-   `reset(seed=...)`: initialize a new episode and return the first
    observation.
-   `step(action)`: apply an action and return observation, reward,
    terminated, truncated, and info.
-   `observation_space`: describe valid observations.
-   `action_space`: describe valid actions.

The underlying simulator performs the actual world update. Gymnasium is
the interface through which the RL algorithm interacts with it.

## 3. High-level system architecture

``` text
                 TRAINING TIME
  Python training scripts
          |
          v
  Gymnasium environment wrapper
          |
       pybind11
          |
          v
  Shared C++ simulation library
  - world generation
  - UAV dynamics
  - collision detection
  - battery and delivery rules
          |
          v
  PPO / other RL algorithm
          |
          v
  Trained policy checkpoints
          |
      export actor
          |
          v
      ONNX models
          |
=========================================================
                 FINAL EXECUTION TIME
  Request generator ---> Delivery queue
                              |
                              v
                       Central coordinator
                              |
                    assignments / targets
                              |
                              v
                      Per-UAV controller
                              |
                    ONNX policy inference
                              |
                              v
                    Safety/action validation
                              |
                              v
                     C++ simulation engine
                   /         |            \
              physics   collisions    state updates
                   \         |            /
                              v
                     World-state snapshot
                              |
                              v
                        raylib renderer
                              |
                              v
                    metrics / logs / HUD
```

## 4. Main components and their responsibilities

### 4.1 World and city generator

The world generator creates the static and semi-static environment in
which UAVs operate.

It should generate or load:

-   World boundaries.
-   Building footprints and heights.
-   Store locations.
-   Customer delivery locations.
-   Charging stations.
-   Optional restricted/no-fly zones.
-   Initial UAV spawn locations.
-   Optional moving obstacles or other dynamic entities.

Use a simple 2D city layout for the underlying map, but represent UAVs
and buildings in 3D for visualization. For example, each building can
have an `(x, y)` footprint in the map plane and a height `z`. A UAV's
position has three coordinates `(x, y, z)`.

For the first version, a grid or hand-authored map is often better than
procedural complexity. It makes debugging and collision tests easier.
Add randomized city generation after the basic simulator works.

A generated world must satisfy validity constraints:

-   Buildings should lie inside world boundaries.
-   Building footprints should not overlap unless overlap is
    intentionally supported.
-   Store, customer, and charging points should not spawn inside
    buildings.
-   UAV spawn positions must be collision-free.
-   Every target should be reachable under the chosen movement rules, or
    explicitly marked infeasible.
-   The generator should use a seed so a scenario can be reproduced.

World generation and collision detection are separate modules. The
generator creates geometry and initial positions; the collision system
tests the geometry against moving UAVs.

### 4.2 World state

The world state is the authoritative record of everything currently
happening.

A useful conceptual structure is:

``` cpp
struct WorldState {
    double simulationTime;
    CityMap city;
    std::vector<UAVState> uavs;
    std::vector<DeliveryRequest> requests;
    std::vector<ChargingStation> chargingStations;
};
```

The exact C++ types can evolve. The principle is more important: avoid
maintaining separate, inconsistent copies of the physical state in the
renderer, the RL wrapper, and the coordinator.

A UAV state could contain:

``` cpp
struct UAVState {
    int id;
    Vec3 position;
    Vec3 velocity;
    double batteryPercent;
    double payload;
    UAVMode mode;
    int activeRequestId;
    Vec3 currentTarget;
};
```

Add fields such as maximum speed, maximum acceleration, battery
capacity, and payload capacity as configuration or per-UAV properties.

### 4.3 Simulation clock and time steps

Use one authoritative simulation clock. A fixed physics time step makes
the simulation easier to reproduce and test.

For example, the first version could use:

-   Physics integration: 0.02 seconds per step, equivalent to 50
    simulation steps per second.
-   UAV policy control: every 0.05 to 0.1 seconds, if a slower policy
    rate is desired.
-   Coordinator dispatch: every 0.5 to 1.0 simulated seconds, plus
    immediate handling of urgent events if required.
-   Rendering: driven by the real-time frame loop, independently of the
    physics step.

These are initial design values, not universal constants. Profile the
system and tune them.

If rendering runs slower than the physics rate, use a fixed-step
accumulator. The renderer should display interpolated or latest state;
it should not change the simulation time step.

### 4.4 UAV motion and dynamics

The policy should not teleport a drone to a requested position. It
should produce a control command, such as desired velocity or
acceleration, which is constrained by the simulator.

A simple first model is:

1.  The policy outputs a desired 3D velocity.
2.  The simulator clamps the velocity to the UAV's maximum speed.
3.  The simulator optionally limits acceleration and turning rate.
4.  The physics integrator updates velocity and position.
5.  Collision detection checks the proposed movement.
6.  The simulator commits a safe result or applies a defined collision
    response.

For a simple constant-velocity step:

\[ p\_{t+`\Delta `{=tex}t} = p_t + v_t `\Delta `{=tex}t \]

For an acceleration-based model:

\[ v\_{t+`\Delta `{=tex}t} = v_t + a_t `\Delta `{=tex}t \]

\[ p\_{t+`\Delta `{=tex}t} = p_t +
v\_{t+`\Delta `{=tex}t}`\Delta `{=tex}t \]

Choose one update convention and use it consistently in training and
deployment. A more realistic model can be added later, but it is better
to start with a small, explicit, testable dynamics model.

### 4.5 Collision system

Collision detection is a property of the simulator, not something the
neural network is trusted to enforce.

At minimum, consider:

-   UAV versus building.
-   UAV versus world boundary.
-   UAV versus another UAV.
-   UAV versus restricted zones, if represented.
-   Swept movement collisions, where a UAV crosses an obstacle between
    two discrete positions.

A simple first collision model treats a UAV as a sphere or capsule and
buildings as axis-aligned 3D boxes. More complex geometry can be added
after the basic collision system is reliable.

**Discrete-position checks alone can miss collisions.** If a UAV moves
from one side of a thin obstacle to the other in one time step, neither
endpoint may lie inside the obstacle. Use a swept sphere/segment test,
smaller substeps, or another continuous-collision approach to handle
this tunneling problem.

Define collision semantics explicitly. For example:

-   Is touching a building face considered a collision?
-   Does a collision terminate the episode, damage the UAV, or merely
    stop it?
-   Is a near miss penalized even when no collision occurs?
-   What happens when two UAVs propose intersecting paths in the same
    time step?
-   Does the system roll back the move, stop both UAVs, or apply a
    deterministic separation rule?

For deterministic simulation, resolve simultaneous conflicts in a
predictable way. Do not let vector iteration order accidentally decide
which UAV survives a conflict.

Collision detection and collision avoidance are different:

-   **Detection** determines whether a proposed or actual trajectory
    intersects an obstacle.
-   **Avoidance** chooses actions that prevent the intersection.

The RL policy can learn avoidance, but a separate safety layer should
reject or modify obviously unsafe commands. The simulator's collision
system remains the final authority.

### 4.6 Battery and charging

Battery must affect feasible assignments and UAV behavior.

For a first model, use battery percentage from 0 to 100 and subtract
energy based on elapsed time, movement, and optional payload. A basic
model could be:

\[ E\_{`\text{used}`{=tex}} = c\_{`\text{idle}`{=tex}}`\Delta `{=tex}t +
c\_{`\text{distance}`{=tex}}`\lVert `{=tex}`\Delta `{=tex}p`\rVert `{=tex}+
c\_{`\text{climb}`{=tex}}`\max`{=tex}(0,`\Delta `{=tex}z) +
c\_{`\text{payload}`{=tex}}m`\lVert `{=tex}`\Delta `{=tex}p`\rVert`{=tex}
\]

The coefficients are simulation parameters that should be calibrated for
stable behavior, not claims about a real drone.

Convert energy consumed to battery percentage using the configured
battery capacity. Clamp battery to valid bounds.

A UAV should not accept a delivery merely because it has enough battery
to reach the destination. The feasibility check should estimate the
energy for:

-   Travel to pickup.
-   Pickup-to-customer travel.
-   Required reserve.
-   A route to a reachable charging station if charging is required
    after the delivery.

Use a conservative reserve. If no charging station is reachable, the
coordinator should reject or defer the assignment rather than knowingly
dispatching a UAV into an unrecoverable state.

Charging behavior:

1.  The coordinator or fleet manager marks a UAV as needing charge.
2.  The UAV receives a charging-station target.
3.  The navigation policy flies there.
4.  The simulator confirms that the UAV is within the docking radius and
    can safely dock.
5.  The battery increases according to the charging model.
6.  The UAV becomes eligible for new assignments when its charge
    threshold is met.

### 4.7 Delivery request generator

The request generator runs continuously during the final simulation. It
creates requests according to a configurable demand model.

A request can contain:

``` cpp
struct DeliveryRequest {
    int id;
    Vec3 pickup;
    Vec3 destination;
    double creationTime;
    double deadline;
    double payload;
    RequestStatus status;
    int assignedUavId;
};
```

A request lifecycle might be:

`Pending -> Assigned -> PickingUp -> InTransit -> Delivered`

Other valid transitions include:

-   `Pending -> Expired` if it times out before assignment.
-   `Assigned -> Pending` if an assignment is canceled and the request
    is requeued.
-   `InTransit -> Failed` if the system defines a failure condition.
-   `Pending -> Rejected` if the request is invalid or outside the
    service region.

Keep these transitions explicit. Avoid setting arbitrary status fields
from many unrelated parts of the code.

For the first demand model, use a Poisson arrival process with rate
(`\lambda`{=tex}) requests per simulated time unit. Inter-arrival times
can be sampled from an exponential distribution:

\[ `\Delta `{=tex}t = -`\frac{\ln(U)}{\lambda}`{=tex}, `\qquad `{=tex}U
`\sim `{=tex}`\operatorname{Uniform}`{=tex}(0,1) \]

Use a seeded random-number generator. This allows the same request
stream to be replayed when comparing policies.

Make the request rate, payload range, pickup/customer distributions, and
deadlines configurable. Start with one fixed request rate; add peak
hours and spatial demand hotspots later.

### 4.8 Fleet manager and UAV state machine

The fleet manager tracks whether each UAV can accept work.

A useful state machine is:

-   `Idle`: available for an assignment.
-   `Assigned`: a request has been allocated but pickup travel has not
    started.
-   `FlyingToPickup`: travelling to the store.
-   `PickingUp`: pickup operation is in progress.
-   `FlyingToDestination`: carrying the delivery.
-   `ReturningToCharge`: travelling to a charging station.
-   `Charging`: battery is being replenished.
-   `Emergency`: the UAV has entered a defined safety/failure condition.
-   `Offline`: the UAV is not available.

Not every implementation needs every state immediately. Begin with the
smallest state set that supports the request lifecycle, then add states
deliberately.

Define allowed transitions in one place. For example, a UAV should not
become `Idle` while it still owns an active request. Similarly, it
should not become available for a second assignment while already
carrying a delivery.

### 4.9 Central coordinator

The coordinator is responsible for fleet-level decisions. It should not
directly move the drones. It assigns work and sets goals; each UAV
controller handles navigation.

At each dispatch event, the coordinator:

1.  Reads pending requests and current fleet states.
2.  Builds the feasible request/UAV pairs.
3.  Estimates travel time, energy use, risk, workload, and deadline
    slack.
4.  Chooses a feasible assignment or waits if no assignment is suitable.
5.  Reserves the request and UAV so another dispatch event cannot assign
    them twice.
6.  Publishes the assigned sequence of targets.
7.  Revisits the decision when new requests arrive, a UAV becomes
    available, or a material state change occurs.

A basic heuristic coordinator is essential even if the final objective
is RL. It provides a working baseline and makes it possible to test the
simulator before training.

One possible assignment cost is:

\[ C(i,j) = w_d D(i,j) + w_t T(i,j) + w_e E(i,j) + w_r R(i,j) + w_q
Q(i,j) \]

where (i) is a request and (j) is a UAV, (D) is estimated travel
distance, (T) is estimated completion time, (E) is energy cost, (R) is
risk, and (Q) represents queueing, lateness, or workload. The weights
are configurable.

Only feasible pairs should be considered. The coordinator should not
choose a low-cost but impossible assignment.

#### Start with heuristics, then train the coordinator

Do not start by training the coordinator and the navigation policy
simultaneously. That makes failures hard to diagnose.

First build a coordinator that uses a clear heuristic. After the
simulator, navigation policy, and request lifecycle work, train a
coordinator policy and compare it against the heuristic.

A coordinator action can be a discrete choice among feasible request/UAV
pairs plus a `WAIT` action. For larger fleets and request queues, this
action space may become large; use candidate filtering or a more
structured assignment policy rather than enumerating every possible
combination indefinitely.

If feasible actions vary per state, use action masking with a compatible
algorithm. For example, MaskablePPO from `sb3-contrib` is one possible
starting point. Verify the chosen library's API and masking semantics
for the version installed.

### 4.10 UAV navigation policy

Each UAV uses the same policy weights initially, with its own
observation and state. This is parameter sharing, not necessarily a
separate neural network for every UAV.

The policy's job is to navigate toward the current target while
respecting constraints. It should not decide which customer to serve;
that is the coordinator's job.

A possible observation vector includes:

-   Current position and velocity.
-   Target position relative to the UAV.
-   Target distance and bearing.
-   Battery fraction.
-   Payload fraction.
-   Maximum speed or other relevant limits.
-   Nearby building geometry or a local occupancy representation.
-   Relative positions and velocities of nearby UAVs.
-   Whether the UAV is carrying a delivery or returning to charge.

The exact observation representation is a key design decision. Avoid
passing the entire city as an unstructured list with an unbounded size.
Start with a fixed-size representation, such as a local grid,
nearest-(k) obstacle features, or a compact set of relative obstacle
measurements. Document the feature order and units.

Possible actions:

-   Desired 3D velocity ((v_x,v_y,v_z)).
-   Desired acceleration ((a_x,a_y,a_z)).
-   A discrete movement command such as one of a fixed set of
    directions.

Continuous actions are natural for velocity/acceleration control. PPO
can support continuous actions; SAC is another candidate. Choose the
algorithm based on training stability and the action space, not because
one algorithm is universally best.

A typical reward design may include:

-   Positive reward for reducing distance to the target.
-   Positive reward for reaching the target.
-   Penalty for energy consumption.
-   Large penalty for collision.
-   Penalty for unsafe boundary approaches.
-   Penalty for excessive time or oscillation.
-   Penalty for violating movement constraints, if violations are
    representable.

Do not reward distance reduction alone without terminal success and
safety checks. Otherwise, a policy can learn to approach a target
recklessly or exploit a flaw in the reward.

### 4.11 Safety layer

A safety layer checks the policy's proposed command before it reaches
the physics engine.

Examples of checks:

-   Clamp speed and acceleration.
-   Reject non-finite values such as NaN or infinity.
-   Check whether the next short segment intersects a building or
    another UAV's predicted safety envelope.
-   Prevent movement outside the permitted world region.
-   Enforce emergency behavior at critically low battery.
-   Apply a fallback stop or safe steering command when a proposed
    action is invalid.

The safety layer should be deterministic and tested independently. It
must not silently hide all failures during evaluation. Log when it
intervenes so you can measure how often the learned policy relies on it.

## 5. Reinforcement-learning environment design

### 5.1 Recommended structure

Keep the C++ simulator independent of Python and Gymnasium. Expose a
small, stable API using pybind11.

Conceptually, the C++ library should provide methods such as:

``` cpp
class Simulator {
public:
    void reset(uint64_t seed);
    Observation getObservation(int uavId) const;
    StepResult step(int uavId, const Action& action, double dt);
    WorldSnapshot getSnapshot() const;
};
```

These are illustrative interfaces, not a complete implementation. In
practice, the simulator may need a fleet-level step that advances all
UAVs together so drone-drone collisions are evaluated consistently.

The Python wrapper calls this library and converts C++ observations and
results into NumPy arrays and Gymnasium return values.

### 5.2 Gymnasium contract

A Gymnasium environment typically has this shape:

``` python
class AeroFleetNavEnv(gymnasium.Env):
    def __init__(self, config):
        self.sim = create_cpp_simulator(config)
        self.observation_space = ...
        self.action_space = ...

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        observation = self.sim.reset_and_get_observation(seed)
        info = {}
        return observation, info

    def step(self, action):
        result = self.sim.step(action)
        observation = result.observation
        reward = result.reward
        terminated = result.goal_reached or result.collision
        truncated = result.time_limit_reached
        info = result.metrics
        return observation, reward, terminated, truncated, info
```

Adapt the wrapper to the exact API of your C++ bindings. The important
requirements are consistency with Gymnasium's current API and a clear
separation between environment mechanics and learning algorithm.

Use `terminated=True` when the task reaches a terminal state, such as
goal reached or a collision that ends the episode. Use `truncated=True`
when an external limit ends the episode, such as a time limit. If both
can happen in one step, define and test the precedence semantics.

### 5.3 Reset and scenario generation

Each training episode should start from a valid scenario. A reset
should:

1.  Seed the random generator.
2.  Generate or load the city geometry.
3.  Place stores, destinations, charging stations, and UAVs.
4.  Validate that initial positions are legal and collision-free.
5.  Select a target and initial battery/payload state.
6.  Initialize episode counters and metrics.
7.  Return the initial observation.

Randomize scenarios enough to support generalization, but do not make
the first training task unnecessarily complex. Start with fixed simple
maps and then introduce variation in stages.

### 5.4 Curriculum learning

A sensible progression is:

1.  Open space, one UAV, one target.
2.  Add boundaries and speed/acceleration constraints.
3.  Add static buildings.
4.  Add narrow passages and varied target positions.
5.  Add battery and energy costs.
6.  Add multiple UAVs and dynamic collision avoidance.
7.  Add pickup/delivery state transitions and changing targets.
8.  Evaluate on maps and demand streams not seen during training.

The policy should learn each added challenge before the environment
becomes significantly more difficult.

### 5.5 Environment validation

Gymnasium's environment checker is useful for checking the API, but it
does **not** prove that collision detection or physics is correct.

Use multiple layers of validation:

-   Gymnasium checker for API and observation/action spaces.
-   C++ unit tests for geometry and collision primitives.
-   Scenario tests for the complete step function.
-   Random-policy rollouts to expose crashes and invalid states.
-   Seed reproducibility tests.
-   Cross-language differential tests comparing the same actions in
    training and deployment.

## 6. Collision correctness: how to establish confidence

Collision correctness must be tested explicitly. Do not infer
correctness just because the rendered drone appears to avoid buildings.

### 6.1 Geometry unit tests

Write deterministic tests with known expected results:

1.  UAV far from a building: no collision.
2.  UAV overlapping the building volume: collision.
3.  UAV touching the boundary: result matches the documented touching
    convention.
4.  UAV crosses a thin obstacle in one large step: collision is detected
    by swept collision or substepping.
5.  UAV flies above a building: result follows the building's 3D height
    and UAV size.
6.  Two UAVs cross paths: drone-drone conflict is detected.
7.  UAV exits world bounds: the defined boundary rule activates.
8.  Invalid or non-finite positions: safely rejected or handled
    according to policy.

### 6.2 World-generation validation

After every generated scenario, validate:

-   Building dimensions and locations are legal.
-   Initial UAVs do not overlap obstacles.
-   Locations are not embedded in buildings.
-   Required targets are reachable when the scenario claims they are
    reachable.
-   Charging locations are usable.
-   No invalid NaN or infinity values exist.

Generation should not simply retry forever if it cannot find a valid
placement. Set a retry limit and return a clear generation error if
constraints cannot be satisfied.

### 6.3 Differential testing

The strongest protection against Python/C++ divergence is to use the
same C++ simulator in both places. Still test the wrapper and deployment
path:

1.  Fix a scenario seed.
2.  Reset the simulator.
3.  Record the initial state and observation.
4.  Apply a fixed sequence of actions.
5.  Record positions, velocities, battery, collision flags, target
    status, and terminal signals after every step.
6.  Replay the same sequence through the deployment interface.
7.  Compare results within documented numerical tolerances.

If the Python wrapper and C++ runtime both call the same simulator
library, the physics should agree. Differences can still arise from
observation conversion, action scaling, timestep configuration,
floating-point settings, or policy normalization, so test those
separately.

## 7. Training and deploying the navigation policy

### 7.1 Training pipeline

A practical first pipeline:

1.  Implement and test the simulator.
2.  Expose it through pybind11.
3.  Create `AeroFleetNavEnv`.
4.  Verify reset, step, observation shape, action shape, and reward
    behavior.
5.  Run a random policy to confirm the episode completes safely.
6.  Implement a hand-coded waypoint controller as a baseline.
7.  Train PPO or SAC on the initial navigation task.
8.  Evaluate across multiple seeds and held-out maps.
9.  Save the best policy and its configuration.
10. Export the neural-network actor to ONNX.
11. Compare ONNX inference against Python inference on fixed
    observations.
12. Load the ONNX actor from C++ using ONNX Runtime.
13. Test complete closed-loop flight in the C++ simulator.

### 7.2 ONNX deployment

ONNX is a portable representation of a trained neural-network graph.
ONNX Runtime provides C++ APIs for loading and executing compatible ONNX
models.

The general runtime flow is:

1.  Read the current UAV observation from the C++ simulator.
2.  Apply the exact observation preprocessing used during training.
3.  Run the ONNX model.
4.  Convert the output into the expected action format.
5.  Apply action scaling/clamping and the safety layer.
6.  Pass the action into the C++ simulator.
7.  Update physics and collisions.
8.  Repeat at the policy's configured control frequency.

The model must match the training pipeline. Pay particular attention to:

-   Observation feature order.
-   Coordinate system and units.
-   Normalization means and scales.
-   Action bounds and scaling.
-   Deterministic versus stochastic action selection.
-   Network architecture and model input/output names.
-   Policy version and configuration.

If a `VecNormalize`-style wrapper was used in training, export or
reproduce its observation normalization statistics. A neural network
receiving differently scaled inputs may behave badly even though ONNX
inference itself is correct.

A deployment bundle should contain the actor model and a manifest, for
example:

``` text
models/
  navigation_actor.onnx
  navigation_manifest.json
  observation_schema.json
  normalization.json
  action_schema.json
  training_config.json
```

The files and schema are a recommended packaging convention. Define the
contents and version them together.

### 7.3 Alternative deployment choices

-   **ONNX Runtime C++:** recommended for the final C++ application when
    the trained model can be exported compatibly.
-   **LibTorch C++:** useful if the chosen model and workflow are
    naturally supported by the PyTorch C++ frontend, but deployment can
    be heavier.
-   **Python process with IPC or sockets:** often easiest for an early
    prototype, but adds process management, serialization, latency, and
    failure modes.

Avoid embedding a full Python interpreter in the final C++ application
unless there is a specific reason. A clean ONNX inference boundary is
easier to reason about and distribute.

## 8. Training the central coordinator

Train the coordinator only after the simulator, delivery lifecycle, and
baseline navigation behavior are working.

### 8.1 Why it is a different RL problem

Navigation is a frequent, local control problem. The coordinator is a
slower, fleet-level scheduling problem. They have different
observations, actions, reward horizons, and update frequencies.

The navigation agent asks: "How should I move toward this target
safely?"

The coordinator asks: "Which request should be assigned to which UAV,
given fleet status, battery, demand, and deadlines?"

### 8.2 Coordinator observation

A coordinator observation may include:

-   Each UAV's mode, position, battery, payload capacity, and estimated
    availability.
-   Each pending request's pickup, destination, age, payload, and
    deadline slack.
-   Distances or estimated travel times for candidate request/UAV pairs.
-   Charging station status.
-   Current queue size and recent fleet-level throughput.
-   Relevant weather or restriction features if later modeled.

For a fixed-size observation, choose a maximum number of candidate
requests and UAVs, or represent them with a structured model. Do not
silently truncate requests without a documented selection rule.

### 8.3 Coordinator actions

A simple action can be one of:

-   Assign candidate request `i` to candidate UAV `j`.
-   Send a selected UAV to charge, if this is modeled as a coordinator
    action.
-   `WAIT`, meaning do not make a new assignment at this decision step.

Only include actions that are feasible. Use an action mask or candidate
list to prevent choices such as assigning a request to a UAV already
carrying a delivery or sending a drone on a route that violates battery
reserve.

### 8.4 Coordinator reward

The coordinator should be rewarded for useful fleet outcomes, not just
for making assignments.

Possible reward components:

-   Positive reward for successful delivery.
-   Penalty for request waiting time and late completion.
-   Penalty for excessive energy consumption.
-   Penalty for failed or abandoned deliveries.
-   Large penalty for safety incidents.
-   Penalty for dispatching an infeasible assignment.
-   Optional fairness term if one area or class of request is
    systematically neglected.

A conceptual form is:

\[ R = w_d N\_{`\text{delivered}`{=tex}} - w_w
T\_{`\text{waiting}`{=tex}} - w_l N\_{`\text{late}`{=tex}} - w_e
E\_{`\text{used}`{=tex}} - w_f N\_{`\text{failed}`{=tex}} - w_c
N\_{`\text{collisions}`{=tex}} \]

The terms and weights must be designed so the policy cannot exploit a
loophole. For example, if only successful deliveries are rewarded and
waiting is not penalized, the coordinator may ignore difficult requests.
If assignment itself gives a large reward, it may assign too eagerly.

### 8.5 Train coordinator against a stable navigation policy

Initially:

1.  Train the navigation policy.
2.  Freeze its weights.
3.  Train the coordinator using the fixed navigation policy.
4.  Compare with the heuristic coordinator.
5.  Evaluate on held-out demand patterns and maps.

This isolates coordinator improvements from navigation changes. Later,
you can alternate updates if there is a strong reason, but do not begin
with joint learning.

## 9. Request assignment and charging logic

A delivery should be assigned only if it is feasible under the
configured model.

A practical feasibility filter:

1.  UAV is online and in an eligible state.
2.  Payload capacity is sufficient.
3.  Request pickup and destination are valid.
4.  Estimated travel time is acceptable relative to deadline.
5.  Estimated energy for pickup and delivery is within available energy.
6.  The UAV can preserve reserve energy or reach an appropriate charging
    station.
7.  The route is not prohibited by map constraints.

Then score the feasible pairs. If no pair is feasible, leave the request
pending, expire it according to policy, or mark it unserviceable with a
reason. Do not repeatedly assign and cancel the same impossible request
without a backoff or state rule.

Charging can be handled in one of two ways:

-   **Rule-based charging:** a fleet manager sends UAVs to charge when a
    threshold or predicted reserve rule is triggered.
-   **Learned charging decisions:** the coordinator chooses when to send
    a UAV to charge as part of the scheduling policy.

Start with rule-based charging because it makes the system safer and
easier to debug. Make it a learned decision later if it is a meaningful
research objective.

## 10. Visualization with raylib

The visualization is a client of the simulation state, not the
simulation itself.

Render:

-   Buildings with heights that reflect their geometry.
-   Ground plane, world boundary, and optional grid.
-   Stores, customer locations, and charging stations with distinct
    markers.
-   UAV models or simple colored geometric representations.
-   Target lines or planned route segments.
-   Request markers and status indicators.
-   Battery bars, UAV state, active request, and speed.
-   Collision events and safety-layer interventions.
-   A HUD with simulation time, pending requests, delivered count,
    active UAV count, average latency, collision count, and energy
    metrics.

Keep the first renderer simple. Clear spatial separation, correct
coordinates, and readable state are more important than detailed models.

The renderer should receive a snapshot or read-only view of the latest
world state. Avoid having drawing code directly update UAV positions or
delivery states.

If simulation and rendering run on different threads, define ownership
and synchronization carefully. Start with one thread and a fixed-step
update loop unless profiling shows that separation is necessary.

## 11. Evaluation and proof that RL helps

A successful demonstration is not sufficient evidence that RL improved
the system. Evaluate against clear baselines.

### 11.1 Navigation baselines

Compare the learned controller with:

-   Straight-line waypoint steering in open space.
-   A simple obstacle-aware heuristic controller.
-   The trained navigation policy.

Measure:

-   Target reach rate.
-   Collision rate.
-   Mean and percentile time to target.
-   Distance traveled.
-   Energy consumed per trip.
-   Safety-layer intervention frequency.
-   Performance across held-out maps.

### 11.2 Coordinator baselines

Compare the learned coordinator with:

-   First-come, first-served.
-   Nearest feasible UAV.
-   Lowest estimated completion time.
-   The RL coordinator.

Measure:

-   Deliveries completed per simulated hour.
-   Mean and P95 request latency.
-   Deadline success rate.
-   Mean request waiting time.
-   Energy per successful delivery.
-   Failed or expired requests.
-   UAV utilization and charging time.
-   Collision/safety incident rate.

Run each policy on the same seeded demand scenarios. Use multiple seeds
and report averages plus variation; a single favorable run is not
enough.

### 11.3 Avoid misleading metrics

-   Do not report only average delivery time; a few very late requests
    may be hidden by the average.
-   Do not reward a coordinator for assignments that never complete.
-   Do not compare methods on different demand streams.
-   Do not hide safety-layer interventions.
-   Do not evaluate only on training maps.
-   Do not claim real-world drone performance from a simplified
    simulation without a validated physical model.

## 12. Suggested repository layout

``` text
AeroFleet/
├── CMakeLists.txt
├── README.md
├── config/
│   ├── simulation.json
│   ├── city.json
│   ├── training_nav.yaml
│   └── training_coordinator.yaml
├── cpp/
│   ├── core/
│   │   ├── Simulator.hpp
│   │   ├── Simulator.cpp
│   │   ├── WorldState.hpp
│   │   └── SimulationClock.hpp
│   ├── world/
│   │   ├── CityMap.hpp
│   │   ├── WorldGenerator.hpp
│   │   └── WorldGenerator.cpp
│   ├── physics/
│   │   ├── Dynamics.hpp
│   │   ├── CollisionSystem.hpp
│   │   └── CollisionSystem.cpp
│   ├── fleet/
│   │   ├── UAVState.hpp
│   │   ├── FleetManager.hpp
│   │   └── FleetManager.cpp
│   ├── requests/
│   │   ├── DeliveryRequest.hpp
│   │   ├── RequestGenerator.hpp
│   │   └── RequestGenerator.cpp
│   ├── coordinator/
│   │   ├── ICoordinator.hpp
│   │   ├── HeuristicCoordinator.hpp
│   │   └── ONNXCoordinator.hpp
│   ├── control/
│   │   ├── INavigationPolicy.hpp
│   │   ├── ONNXNavigationPolicy.hpp
│   │   └── SafetyLayer.hpp
│   ├── inference/
│   │   └── ONNXModel.hpp
│   ├── visualization/
│   │   ├── Renderer.hpp
│   │   └── Renderer.cpp
│   ├── telemetry/
│   │   ├── Metrics.hpp
│   │   └── Logger.hpp
│   └── main.cpp
├── python/
│   ├── bindings/
│   │   └── simulator_bindings.cpp
│   ├── aero_fleet/
│   │   ├── nav_env.py
│   │   ├── coordinator_env.py
│   │   ├── observations.py
│   │   └── rewards.py
│   ├── train_nav.py
│   ├── train_coordinator.py
│   ├── evaluate_nav.py
│   ├── evaluate_coordinator.py
│   └── export_onnx.py
├── models/
│   ├── navigation/
│   └── coordinator/
├── tests/
│   ├── test_collision.cpp
│   ├── test_world_generation.cpp
│   ├── test_delivery_lifecycle.cpp
│   ├── test_battery.cpp
│   └── test_reproducibility.cpp
└── logs/
```

This is a suggested layout, not a requirement to create every file
immediately. Build only the files needed for the current milestone.

## 13. Implementation roadmap

### Phase 0: Write down the simulation contract

Before implementing features, define:

-   Coordinate system and units.
-   World dimensions.
-   UAV dimensions and movement limits.
-   Fixed physics time step.
-   Collision semantics.
-   Battery model.
-   Request lifecycle.
-   Observation and action schemas.
-   Episode termination rules.
-   Configuration and random seeding.

This prevents different modules from making incompatible assumptions.

### Phase 1: Build the C++ simulator without RL

Implement:

1.  Vector math and basic geometry.
2.  City map and static obstacles.
3.  UAV state and motion integration.
4.  Collision system and unit tests.
5.  Battery updates.
6.  A hand-coded waypoint controller.
7.  A simple single-UAV route test.

Do not start RL until a UAV can reach a target in the C++ simulator and
collision tests pass.

### Phase 2: Add deliveries and a fleet

Implement:

1.  Store/customer/charging locations.
2.  Delivery request structure and lifecycle.
3.  Request generator with deterministic seeding.
4.  Fleet state machine.
5.  Heuristic coordinator.
6.  Battery-aware feasibility checks.
7.  End-to-end pickup-to-delivery-to-charge flow.

At this stage, use the heuristic navigation controller and heuristic
coordinator. The simulation should run continuously and complete
deliveries without requiring an RL model.

### Phase 3: Build the live visualization

Render the map, buildings, UAVs, targets, requests, charging stations,
and HUD. Verify that the displayed positions exactly match the
simulator's state.

### Phase 4: Expose the simulator to Python

Add pybind11 bindings. Keep the binding API small and stable. Create a
Gymnasium navigation environment that calls the C++ simulator. Run the
Gymnasium checker and C++ tests.

### Phase 5: Train navigation

Start with one UAV and one target. Train PPO or SAC, compare with the
heuristic controller, and gradually add obstacles, battery, and other
UAVs. Save and evaluate checkpoints.

### Phase 6: Deploy navigation into C++

Export the policy actor to ONNX, load it with ONNX Runtime, and compare
Python and C++ inference on the same observations. Run closed-loop tests
in the C++ simulation.

### Phase 7: Train the coordinator

Freeze the navigation policy. Create a fleet-dispatch Gymnasium
environment, implement action masking or candidate actions as needed,
and train a coordinator. Compare it with the heuristic baseline using
identical seeded demand scenarios.

### Phase 8: Full application integration

Connect the request generator, coordinator, UAV policies, safety layer,
physics, collision system, visualization, and telemetry. Provide a
single command or launch target that starts the final simulation.

### Phase 9: Evaluate and document

Run repeatable benchmarks, save configuration and seeds, report results,
document limitations, and provide instructions for building, training,
exporting, and running the application.

## 14. Final runtime loop

A high-level loop can look like this:

``` cpp
while (!app.shouldClose()) {
    // 1. Advance simulation by fixed physics steps.
    while (simulationClock.hasStep()) {
        // 2. Generate requests whose arrival time has been reached.
        requestGenerator.update(world, simulationClock.now());

        // 3. Dispatch requests at coordinator decision times.
        if (coordinator.shouldDispatch(world, simulationClock.now())) {
            coordinator.assignRequests(world);
        }

        // 4. Build control actions for eligible UAVs.
        auto actions = navigationSystem.computeActions(world);

        // 5. Validate actions before movement.
        safetyLayer.filter(world, actions);

        // 6. Integrate movement and resolve collisions.
        simulator.step(actions, fixedDt);

        // 7. Update pickup/delivery/charging state machines.
        fleetManager.update(world, fixedDt);
        deliveryManager.update(world, fixedDt);

        // 8. Update metrics and logs.
        metrics.update(world, fixedDt);
        simulationClock.advance();
    }

    // 9. Render a read-only view of the current state.
    renderer.draw(world.snapshot(), metrics.snapshot());
}
```

This is an architectural sketch. Exact ordering matters and must be
defined carefully in the real implementation. For example, all UAV
actions for one time step should normally be computed from a consistent
state snapshot, and conflicts should be resolved collectively rather
than allowing one UAV to observe another UAV's already-updated position
while the other still has its old position.

## 15. Important engineering rules

1.  **One source of truth:** the C++ simulator owns physical state.
2.  **Separate decisions from mechanics:** policies propose actions; the
    simulator applies the rules.
3.  **Keep the coordinator and navigator separate:** dispatch chooses
    tasks; navigation moves to the current target.
4.  **Use deterministic seeds:** every important test and benchmark
    should be reproducible.
5.  **Test collisions independently:** visual plausibility is not proof
    of correctness.
6.  **Use a baseline before RL:** always have a working heuristic
    approach for comparison.
7.  **Freeze one policy while training the other:** initially isolate
    learning problems.
8.  **Version model and preprocessing together:** an ONNX file without
    its observation/action schema is not a complete deployable model.
9.  **Log safety interventions and failures:** do not let the safety
    layer conceal policy weaknesses.
10. **Keep rendering non-authoritative:** rendering cannot move the UAV
    or decide whether a delivery succeeded.
11. **Start simple:** one UAV, one target, static buildings, then scale
    up.
12. **Do not overclaim:** simulation results are evidence about the
    configured model, not proof of real-world flight safety.

## 16. What the first working milestone should be

The first milestone should not be a sophisticated neural network or a
beautiful city.

Build a C++ program in which:

-   One UAV starts at a known location.
-   One target is visible.
-   Buildings are represented as simple boxes.
-   A basic controller moves the UAV toward the target.
-   The collision system prevents or flags obstacle intersection.
-   Battery decreases predictably.
-   The simulation runs at a fixed time step.
-   A unit test confirms a known collision and a known non-collision.
-   raylib renders the exact state of the simulator.

Once this is reliable, add requests and the fleet coordinator. Only then
connect Gymnasium and begin training.

## 17. Completion criteria

A credible AeroFleet prototype is complete when all of the following are
true:

-   The city and UAVs are generated from reproducible configuration and
    seeds.
-   The request simulator creates a continuous stream of valid requests.
-   The coordinator assigns feasible deliveries without double-assigning
    UAVs or requests.
-   UAVs complete pickup, delivery, and charging state transitions.
-   Collision detection passes deterministic geometry and movement
    tests.
-   The navigation policy can be trained through Gymnasium using the
    shared C++ simulator.
-   The exported navigation policy runs from C++ inference.
-   Python and C++ inference pipelines agree on fixed test cases within
    defined tolerances.
-   The coordinator policy is trained and evaluated against a heuristic
    baseline, if coordinator RL is included in the target scope.
-   The live raylib visualization shows the real simulator state.
-   Metrics are recorded and reproducible benchmarks show how the
    learned policies compare with baselines.
-   A fresh developer can follow the documentation to build, train,
    export, and run the project.

## 18. Recommended first decisions

Before writing the full implementation, settle these initial choices:

-   **World:** bounded 2D map with 3D UAV altitude and 3D building
    heights.
-   **Physics:** fixed-step, simplified UAV dynamics.
-   **Collision:** sphere/capsule UAV representation, box buildings,
    swept collision or substepping.
-   **Navigation action:** continuous desired velocity or acceleration.
-   **Navigation training:** PPO as a straightforward starting point;
    compare alternatives only after a baseline works.
-   **Coordinator:** heuristic first, then a separate RL policy.
-   **Python/C++ boundary:** pybind11 for training against the C++
    simulator.
-   **Deployment:** ONNX Runtime in C++ if model export is compatible.
-   **Rendering:** raylib reads snapshots and does not mutate simulation
    state.
-   **Evaluation:** seeded scenarios, repeated runs, held-out maps, and
    explicit baseline comparisons.

These choices create a manageable progression from a small, testable
simulation to a full multi-agent delivery system without committing
prematurely to unnecessary complexity.
