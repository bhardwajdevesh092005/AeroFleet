# Step 07 Execution Log: Delivery Request Generator & Demand Modeling

## Overview
- **Step ID**: 07
- **Phase**: Phase III - Fleet Lifecycle & Baseline Logistics
- **Status**: **COMPLETED**
- **Completion Date**: 2026-10-19

---

## Implementation Details

### 1. Delivery Request Lifecycle & Data Model
- Created `cpp/requests/DeliveryRequest.hpp`:
  - `DeliveryRequest` authoritative data structure:
    - `id`: Monotonically increasing request identifier.
    - `storeId` & `pickupLocation`: 3D Cartesian coordinates of the originating store hub.
    - `customerId` & `dropoffLocation`: 3D Cartesian coordinates of customer delivery site.
    - `creationTime`: Timestamp ($s$) when the order entered the system.
    - `deadline`: Guaranteed delivery deadline ($s$).
    - `payloadMass`: Transported cargo mass ($kg$).
    - `status`: Atomic lifecycle state (`PENDING`, `ASSIGNED`, `PICKING_UP`, `IN_TRANSIT`, `DELIVERED`, `EXPIRED`, `CANCELLED`, `REJECTED`).
    - `assignedUavId`: Identifier of the assigned UAV ($-1$ if unassigned).
    - `pickupTime` & `deliveryTime`: Actual execution timestamps.
  - Lifecycle state machine validation via `canTransitionTo()` and `transitionTo()`:
    - Enforces valid operational flow: `PENDING -> ASSIGNED -> PICKING_UP -> IN_TRANSIT -> DELIVERED`.
    - Supports task re-queuing (`ASSIGNED / PICKING_UP / IN_TRANSIT -> PENDING`) on emergency aborts.
    - Rejects illegal state jumps (e.g. `PENDING` directly to `DELIVERED`) by throwing `RequestTransitionException`.
    - Treats `DELIVERED`, `EXPIRED`, `CANCELLED`, and `REJECTED` as terminal states.

### 2. Stochastic Poisson Demand Generator
- Created `cpp/requests/RequestGenerator.hpp` and `cpp/requests/RequestGenerator.cpp`:
  - Poisson arrival process implemented via Inverse Transform Sampling of the Exponential distribution:
    $$\Delta t_{\text{inter}} = -\frac{\ln(U)}{\lambda}$$
    where $U \sim \text{Uniform}(0, 1)$ (strictly guarded $U \ge 10^{-9}$) and $\lambda = \frac{\text{arrivalRatePerMin}}{60.0}\text{ req/s}$.
  - Landmark sampling: Uniformly selects store hubs and customer drop locations from authoritative `CityMap`.
  - Dynamic deadline estimation:
    $$t_{\text{deadline}} = t_{\text{now}} + \frac{\|\mathbf{p}_{\text{drop}} - \mathbf{p}_{\text{pick}}\|}{v_{\text{nominal}}} \times \text{slackFactor} + t_{\text{buffer}}$$
  - Seed-deterministic replication using `std::mt19937_64`.
  - Continuous time stepping via `step(currentTime, dt)` queue emission.

### 3. Verification Suite & CTest Integration
- Created `tests/test_requests.cpp`:
  - `test_poisson_goodness_of_fit`: Generates 10,000 inter-arrival samples and verifies the sample mean matches theoretical $1 / \lambda = 5.0\text{s}$ within $<3\%$ error.
  - `test_request_lifecycle_transitions`: Tests forward progression and verifies timestamp recording.
  - `test_illegal_lifecycle_skips`: Asserts direct illegal jumps throw `RequestTransitionException`.
  - `test_request_requeue`: Confirms task aborts reset assigned UAV IDs and restore `PENDING` state.
  - `test_seed_determinism`: Proves identical random seeds produce byte-identical request sequences across multiple runs.
  - `test_deadline_and_expiration`: Validates deadline expiration checks and terminal delivery overrides.
  - `test_payload_mass_bounds`: Asserts all sampled package weights strictly respect configured $[m_{\min}, m_{\max}]$ bounds.
  - `test_generator_step_emissions`: Verifies realistic Poisson request throughput during simulation stepping.
- Registered target `test_requests` and `RequestTests` in `CMakeLists.txt`.

---

## Verification & Test Results

### 1. Build & CTest Verification
```bash
ctest --test-dir build --output-on-failure
```
**Output**:
```text
Test project /mnt/c/Users/deves/Desktop/AeroFleet/build
    Start 1: ConfigTests
1/7 Test #1: ConfigTests ......................   Passed    0.01 sec
    Start 2: MathGeometryTests
2/7 Test #2: MathGeometryTests ................   Passed    0.00 sec
    Start 3: WorldGenerationTests
3/7 Test #3: WorldGenerationTests .............   Passed    0.01 sec
    Start 4: KinematicsTests
4/7 Test #4: KinematicsTests ..................   Passed    0.01 sec
    Start 5: CollisionTests
5/7 Test #5: CollisionTests ...................   Passed    0.01 sec
    Start 6: BatteryTests
6/7 Test #6: BatteryTests .....................   Passed    0.00 sec
    Start 7: RequestTests
7/7 Test #7: RequestTests .....................   Passed    0.01 sec

100% tests passed, 0 tests failed out of 7
Total Test time (real) = 0.16 sec
```

### 2. Unit Test Details
```bash
./build/test_requests
```
**Output**:
```text
========================================
      AeroFleet Request Tests           
========================================
[*] Running test_poisson_goodness_of_fit... PASSED
[*] Running test_request_lifecycle_transitions... PASSED
[*] Running test_illegal_lifecycle_skips... PASSED
[*] Running test_request_requeue... PASSED
[*] Running test_seed_determinism... PASSED
[*] Running test_deadline_and_expiration... PASSED
[*] Running test_payload_mass_bounds... PASSED
[*] Running test_generator_step_emissions... PASSED
========================================
Summary: 8 Passed, 0 Failed
========================================
```

---

## Edge Cases Handled

| Scenario | Risk | Solution Applied |
| :--- | :--- | :--- |
| Division by Zero / Log Zero | $U = 0.0$ sampled from uniform distribution causes $\ln(0) \to -\infty$ | Guarded with `u = std::max(1e-9, m_uniformDist(m_rng));`. |
| Premature Direct Delivery | Corrupted logic marks requests delivered without assignment | Strict transition table validation in `canTransitionTo()` throwing `RequestTransitionException`. |
| Impossible Delivery Deadlines | High payload or long transit leaves inadequate flight time | Slack multiplier ($\text{slackFactor} \ge 2.0$) and fixed buffer time ($\ge 30\text{s}$) guaranteed. |
| Re-queuing Ghost Assignments | Aborted flight leaves previous UAV assigned in pending pool | `assignedUavId` automatically reset to $-1$ whenever transitioning to `PENDING`. |

---
**Status**: Step 07 is completed with 100% test pass rate. Ready for **Step 08: Fleet State Machine & UAV Operational Lifecycle**.

