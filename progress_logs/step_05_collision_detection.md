# Step 05 Execution Log: Continuous Collision Detection (CCD) & Conflict Resolution

## Overview
- **Step ID**: 05
- **Phase**: Phase II - Authoritative C++ Simulation Core
- **Status**: **COMPLETED**
- **Completion Date**: 2026-10-19

---

## Implementation Details

### 1. Collision Types & Event Structures
- Created `cpp/physics/CollisionSystem.hpp`:
  - `CollisionType` enum:
    - `NONE`
    - `BUILDING` (static obstacle hit)
    - `UAV_UAV` (dynamic mid-air collision between two UAVs)
    - `WORLD_BOUNDARY` (geofence boundary breach)
    - `GROUND` (uncontrolled ground impact)
  - `collisionTypeToString()` serializer for structured telemetry logging.
  - `CollisionEvent` structure:
    - `type`: Category of collision event.
    - `uavIdA`: Primary UAV ID ($0 \le id < N_{\text{fleet}}$).
    - `uavIdB`: Secondary UAV ID (for UAV-UAV pairwise conflicts, with canonical sorting $id_A < id_B$).
    - `buildingId`: ID of the impacted building primitive (for `BUILDING` collisions).
    - `contactPoint`: 3D Cartesian coordinates of impact point.
    - `contactNormal`: 3D normal vector pointing outwards from the obstacle surface.
    - `simulationTime`: Precise timestamp ($s$) when collision occurred.

### 2. Static Building Continuous Collision Detection (CCD)
- Implemented `checkStaticCollision` in `cpp/physics/CollisionSystem.cpp`:
  - Models the swept UAV displacement from previous position $\mathbf{p}_{\text{prev}}$ to current position $\mathbf{p}_{\text{curr}}$ as a 3D `Capsule` primitive with radius $r_{\text{uav}} = 0.5\text{m}$.
  - Broadphase bounding box acceleration: Queries `SpatialGridBroadphase` with an expanded AABB enclosing the entire trajectory $[\min(\mathbf{p}_{\text{prev}}, \mathbf{p}_{\text{curr}}) - r, \max(\mathbf{p}_{\text{prev}}, \mathbf{p}_{\text{curr}}) + r]$.
  - Narrowphase exact evaluation: Invokes `Capsule::intersects(const AABB&)` evaluating endpoint sphere penetration, segment-box interior intersection, and closed-form segment-to-segment distance across all 12 AABB edges.
  - Zero tunneling: Guarantees high-speed UAVs ($30\text{ m/s}$) cannot tunnel through thin walls ($0.2\text{m}$) within a single physics timestep.

### 3. Dynamic Multi-UAV Pairwise Swept Tests
- Implemented `checkDynamicCollision` in `cpp/physics/CollisionSystem.cpp`:
  - Checks mutual swept trajectories between drone pairs $(A, B)$ across the interval $[t, t + \Delta t]$.
  - Calculates the minimum Euclidean distance squared between two moving line segments using `Capsule::segmentSegmentDistanceSquared`.
  - Collision condition: $d^2 \le (r_A + r_B)^2 = (2 r_{\text{uav}})^2$.
  - Canonical ordering: Enforces $id_A < id_B$ to guarantee identical incident reporting regardless of internal vector storage order.

### 4. Continuous World Boundary Enforcement & Conflict Resolution
- Implemented `checkBoundaryCollision` and `evaluateAndResolve` in `cpp/physics/CollisionSystem.cpp`:
  - Evaluates static building collisions, world boundary limits, followed by dynamic pairwise conflicts.
  - Tests whether the swept displacement or terminal bounding sphere breaches the world limits $[x_{\min}, x_{\max}, y_{\min}, y_{\max}, z_{\min}, z_{\max}]$.
  - On collision detection:
    1. Rolls back the UAV position to its previous safe coordinate $\mathbf{p}_{\text{prev}}$.
    2. Zeroes linear velocity $\mathbf{v} = (0, 0, 0)$ and acceleration $\mathbf{a} = (0, 0, 0)$.
    3. Transitions UAV state machine to `UAVMode::EMERGENCY`.
    4. Records immutable `CollisionEvent` in the returned event list.

### 5. Verification Suite & CTest Integration
- Created `tests/test_collision.cpp`:
  - `test_swept_building_collision`: Sweeps a UAV across a $0.2\text{m}$ thin wall at high speed; verifies CCD intercepts the obstacle, logs building ID, rolls back position, and triggers `EMERGENCY` mode.
  - `test_safe_building_clearance`: Flies a UAV parallel to a building with $2.0\text{m}$ clearance ($> r_{\text{uav}}$); asserts 0 false positive collisions.
  - `test_uav_uav_head_on_collision`: Two UAVs flying directly toward each other; verifies detection of mutual collision, canonical ID ordering, rollback, and `EMERGENCY` state for both agents.
  - `test_uav_uav_safe_vertical_separation`: Two UAVs crossing paths with $5.0\text{m}$ vertical altitude separation ($> 2 r_{\text{uav}}$); confirms no conflict detected.
  - `test_deterministic_conflict_resolution`: Tests conflict evaluation under reversed array orders `[A, B]` vs `[B, A]`; confirms identical collision event counts and canonical ID pairs.
  - `test_world_boundary_collision`: Steers a UAV outside world boundary bounds; verifies boundary intercept, rollback, `EMERGENCY` mode, and `WORLD_BOUNDARY` event logging.
- Registered target `test_collision` and `CollisionTests` in `CMakeLists.txt`.

---

## Verification & Test Results

### 1. Build & CTest Verification
```bash
cmake -B build -S . -DCMAKE_PREFIX_PATH=/home/devesh/.local -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```
**Output**:
```text
Test project /mnt/c/Users/deves/Desktop/AeroFleet/build
    Start 1: ConfigTests
1/5 Test #1: ConfigTests ......................   Passed    0.01 sec
    Start 2: MathGeometryTests
2/5 Test #2: MathGeometryTests ................   Passed    0.00 sec
    Start 3: WorldGenerationTests
3/5 Test #3: WorldGenerationTests .............   Passed    0.01 sec
    Start 4: KinematicsTests
4/5 Test #4: KinematicsTests ..................   Passed    0.01 sec
    Start 5: CollisionTests
5/5 Test #5: CollisionTests ...................   Passed    0.01 sec

100% tests passed, 0 tests failed out of 5
```

### 2. Unit Test Details
```bash
./build/test_collision
```
**Output**:
```text
========================================
     AeroFleet Collision Tests          
========================================
[*] Running test_swept_building_collision... PASSED
[*] Running test_safe_building_clearance... PASSED
[*] Running test_uav_uav_head_on_collision... PASSED
[*] Running test_uav_uav_safe_vertical_separation... PASSED
[*] Running test_deterministic_conflict_resolution... PASSED
[*] Running test_world_boundary_collision... PASSED
========================================
Summary: 6 Passed, 0 Failed
========================================
```

---

## Edge Cases Handled

| Scenario | Risk | Solution Applied |
| :--- | :--- | :--- |
| High-Speed Tunneling | Drone traveling at $30\text{ m/s}$ bypasses a thin $0.2\text{m}$ building wall in $\Delta t = 0.02\text{s}$ | Evaluates swept 3D Capsule against AABB volume and all 12 edges via `Capsule::intersects`. |
| Broadphase Query Truncation | Spatial grid query using only destination position $\mathbf{p}_{t+\Delta t}$ misses obstacles along trajectory | Query AABB computed from the bounding union of start and end positions expanded by $r_{\text{uav}}$. |
| Order-Dependent Agent Collisions | Pairwise checking produces different collision counts depending on fleet vector index order | Canonical pair sorting (`std::min(idA, idB)`, `std::max(idA, idB)`) with strict $i < j$ iteration. |
| False Positive Near-Miss | Drones flying vertically separated or parallel to obstacles register false alarm | Exact segment-to-segment Euclidean distance squared compared strictly against $(r_A + r_B)^2$. |

---
**At the time of completion:** Step 05 passed its recorded tests and the next planned item was **Step 06: High-Fidelity Battery, Energy & Charging Station Mechanics**. Current overall status is maintained in [the progress ledger](README.md).

