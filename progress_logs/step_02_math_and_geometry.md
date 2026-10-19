# Step 02 Execution Log: 3D Vector Math, Spatial Primitives & Bounding Volumes

## Overview
- **Step ID**: 02
- **Phase**: Phase I - Foundation & Mathematical Core
- **Status**: **COMPLETED**
- **Completion Date**: 2026-10-18

---

## The 5-Part Breakdown & Implementation Details

### Part 2.1: Mathematical Utilities & Precision Helpers
- **Objective**: Implement clean, constexpr math utility functions for physical calculations and angle conversions.
- **Components Implemented**:
  - `cpp/math/MathUtils.hpp`:
    - Constants: $\pi$, $2\pi$, $\frac{\pi}{2}$, $\epsilon = 10^{-9}$.
    - Generic templated `clamp(val, low, high)` and `lerp(a, b, t)`.
    - Degree-to-radian (`degToRad`) and radian-to-degree (`radToDeg`) conversions.
    - Angle wrap function `wrapAnglePi(rad)` normalizing any angle into $[-\pi, \pi]$.
    - Precision equality helpers `isZero()` and `areEqual()`.

### Part 2.2: Bounding Volume Primitives: Sphere, AABB, Ray & Capsule
- **Objective**: Implement zero-allocation spatial representations in 3D East-North-Up coordinate space.
- **Components Implemented**:
  - `cpp/math/Geometry.hpp`:
    - `Ray`: Origin vector, normalized direction vector, and evaluation method `at(t)`.
    - `Sphere`: Center vector, radius, point containment test, and `Sphere-Sphere` intersection.
    - `AABB`: Minimum and maximum extent vectors, `center()`, `extents()` (half-dimensions), `size()`, point containment, point-AABB closest point projection, Euclidean distance, and bounding box expansion.
    - `Capsule`: Segment endpoints $A$ and $B$, radius $r$, segment length, and point-to-capsule squared distance.

### Part 2.3: Analytical Geometric Intersections & Raycasting
- **Objective**: Implement fast analytical intersection tests for proximity queries, obstacle collision detection, and sensor rangefinding.
- **Components Implemented**:
  - `cpp/math/Geometry.hpp`:
    - `Sphere-AABB` intersection: Compares sphere radius against the Euclidean distance from the sphere center to the closest point on the AABB.
    - `AABB-AABB` intersection: Hyperplane separation test across X, Y, and Z axes.
    - `Ray-AABB` (Slab Method): Computes ray entry/exit parameters $t_{\min}$ and $t_{\max}$ across axis-aligned slabs. Returns hit status, distance, hit coordinates, and surface normal vector. Correctly handles rays parallel to slabs and rays originating inside the box.
    - `Ray-Sphere` intersection: Solves quadratic ray-sphere equation for immediate distance, hit point, and normal.

### Part 2.4: Continuous Collision Detection (CCD) via Capsule-AABB Swept Tests
- **Objective**: Guarantee zero high-speed tunneling through thin obstacle walls by testing swept UAV motion envelopes against building volumes.
- **Components Implemented**:
  - `Capsule::intersects(const AABB& aabb)` in `cpp/math/Geometry.hpp`:
    1. Tests both endpoint spheres ($A$ and $B$) against the AABB.
    2. Tests if the 3D line segment $AB$ passes directly through the AABB volume using 3D slab ray clipping.
    3. Implements the closed-form segment-segment distance algorithm (Ericson RTCD 5.1.9) across all 12 AABB edges to detect any edge grazing within radius envelope $r$.

### Part 2.5: Test Suite, CTest Integration & Verification
- **Objective**: Verify spatial mathematics and collision edge cases with dedicated unit tests.
- **Components Implemented**:
  - `tests/test_math_geometry.cpp`:
    - Sphere primitives and containment.
    - AABB primitives, closest point, and distance calculations.
    - Sphere-AABB penetration and corner tests.
    - Ray-AABB slab raycasting (direct frontal hit, vertical miss, parallel miss, inside origin).
    - Ray-Sphere intersection.
    - High-speed tunneling continuous collision detection test ($0.2\text{m}$ thin wall traversed in 1 step).
    - Math utilities (clamp, lerp, angle wrapping).
  - Updated `CMakeLists.txt` to register target `test_math_geometry` with `add_test(NAME MathGeometryTests ...)`.

---

## Verification & Test Results

### 1. Build Verification
```bash
cmake --build build --parallel
```
**Output**:
```text
[ 25%] Built target AeroFleetCore
[ 37%] Building CXX object CMakeFiles/test_math_geometry.dir/tests/test_math_geometry.cpp.o
[ 62%] Built target test_config
[ 87%] Built target AeroFleet
[100%] Linking CXX executable test_math_geometry
[100%] Built target test_math_geometry
```

### 2. Unit Test Execution
```bash
./build/test_math_geometry
```
**Output**:
```text
========================================
    AeroFleet Math & Geometry Tests     
========================================
[*] Running test_sphere_primitives... PASSED
[*] Running test_aabb_primitives... PASSED
[*] Running test_sphere_aabb_intersection... PASSED
[*] Running test_raycast_aabb... PASSED
[*] Running test_raycast_sphere... PASSED
[*] Running test_capsule_continuous_collision_swept... PASSED
[*] Running test_math_utilities... PASSED
========================================
Summary: 7 Passed, 0 Failed
========================================
```

### 3. CTest Suite Verification (All Tests)
```bash
ctest --test-dir build --output-on-failure
```
**Output**:
```text
Test project /mnt/c/Users/deves/Desktop/AeroFleet/build
    Start 1: ConfigTests
1/2 Test #1: ConfigTests ......................   Passed    0.01 sec
    Start 2: MathGeometryTests
2/2 Test #2: MathGeometryTests ................   Passed    0.01 sec

100% tests passed, 0 tests failed out of 2
```

---

## Edge Cases Handled

| Scenario | Risk | Solution Applied |
| :--- | :--- | :--- |
| High-Speed Tunneling | Drone moving $30\text{ m/s}$ skips across a $0.2\text{m}$ wall in one $\Delta t$ | `Capsule-AABB` continuous swept test evaluates the full trajectory segment against box volume and edges. |
| Parallel Raycast to Box Face | Division by zero ($1.0 / 0.0 \to \infty$) in slab test | Guarded with $|d_i| < \epsilon$: if ray is parallel, verifies origin is strictly bounded within slab extents. |
| Degenerate Capsule Segment | $A = B$ (zero length displacement) | Guarded $AB$ length squared $< \epsilon$: defaults closest point to $A$, reducing capsule to a Sphere test. |

---
**Conclusion**: Step 02 is complete and verified. Moving to **Step 03: Procedural City Map Generator & World Representation** to finish Phase I!

