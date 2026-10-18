#include <iostream>
#include <cassert>
#include <cmath>
#include "math/Vec3.hpp"
#include "math/MathUtils.hpp"
#include "math/Geometry.hpp"

using namespace aerofleet;

static int g_testsPassed = 0;
static int g_testsFailed = 0;

#define TEST_ASSERT(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "[-] FAILED: " << msg << " (" << __FILE__ << ":" << __LINE__ << ")\n"; \
            g_testsFailed++; \
            return; \
        } \
    } while(0)

#define RUN_TEST(fn) \
    do { \
        std::cout << "[*] Running " << #fn << "... "; \
        int initialFailed = g_testsFailed; \
        fn(); \
        if (g_testsFailed == initialFailed) { \
            std::cout << "PASSED\n"; \
            g_testsPassed++; \
        } \
    } while(0)

void test_sphere_primitives() {
    Sphere s1(Vec3(0.0, 0.0, 0.0), 2.0);
    Sphere s2(Vec3(3.0, 0.0, 0.0), 1.5);
    Sphere s3(Vec3(10.0, 0.0, 0.0), 1.0);

    TEST_ASSERT(s1.contains(Vec3(1.0, 1.0, 0.0)), "Sphere contains point inside");
    TEST_ASSERT(!s1.contains(Vec3(2.5, 0.0, 0.0)), "Sphere does not contain point outside");
    TEST_ASSERT(s1.intersects(s2), "Overlapping spheres must intersect");
    TEST_ASSERT(!s1.intersects(s3), "Distant spheres must not intersect");
}

void test_aabb_primitives() {
    AABB box(Vec3(-10.0, -10.0, 0.0), Vec3(10.0, 10.0, 20.0));

    TEST_ASSERT(box.contains(Vec3(0.0, 0.0, 10.0)), "AABB contains central point");
    TEST_ASSERT(!box.contains(Vec3(15.0, 0.0, 10.0)), "AABB does not contain external point");

    Vec3 cp = box.closestPoint(Vec3(15.0, 5.0, 10.0));
    TEST_ASSERT(cp == Vec3(10.0, 5.0, 10.0), "Closest point on AABB face");

    Vec3 cpCorner = box.closestPoint(Vec3(15.0, 15.0, 25.0));
    TEST_ASSERT(cpCorner == Vec3(10.0, 10.0, 20.0), "Closest point on AABB corner");

    double dist = box.distance(Vec3(13.0, 10.0, 20.0));
    TEST_ASSERT(std::abs(dist - 3.0) < 1e-6, "Euclidean distance to AABB face");

    AABB box2(Vec3(5.0, 5.0, 10.0), Vec3(15.0, 15.0, 30.0));
    AABB box3(Vec3(50.0, 50.0, 0.0), Vec3(60.0, 60.0, 10.0));
    TEST_ASSERT(box.intersects(box2), "Overlapping AABBs must intersect");
    TEST_ASSERT(!box.intersects(box3), "Disjoint AABBs must not intersect");
}

void test_sphere_aabb_intersection() {
    AABB building(Vec3(10.0, 10.0, 0.0), Vec3(20.0, 20.0, 30.0));

    Sphere overlapping(Vec3(9.0, 15.0, 10.0), 1.5); // Centers at 9.0, radius 1.5 reaches 10.5
    Sphere outside(Vec3(5.0, 15.0, 10.0), 1.0);     // Centers at 5.0, radius 1.0 reaches 6.0
    Sphere cornerOverlap(Vec3(8.0, 8.0, 0.0), 3.0);  // dist to (10, 10, 0) is sqrt(8) ~ 2.828 < 3.0

    TEST_ASSERT(building.intersects(overlapping), "Sphere penetrating AABB face must intersect");
    TEST_ASSERT(!building.intersects(outside), "Sphere outside AABB must not intersect");
    TEST_ASSERT(building.intersects(cornerOverlap), "Sphere penetrating AABB corner must intersect");
}

void test_raycast_aabb() {
    AABB box(Vec3(-5.0, -5.0, 0.0), Vec3(5.0, 5.0, 10.0));

    // Direct hit from front
    Ray r1(Vec3(-20.0, 0.0, 5.0), Vec3(1.0, 0.0, 0.0));
    RaycastHit h1 = raycastAABB(r1, box);
    TEST_ASSERT(h1.hit, "Direct frontal ray must hit AABB");
    TEST_ASSERT(std::abs(h1.distance - 15.0) < 1e-6, "Raycast hit distance mismatch");
    TEST_ASSERT(h1.point.isApprox(Vec3(-5.0, 0.0, 5.0)), "Raycast hit coordinate mismatch");
    TEST_ASSERT(h1.normal.isApprox(Vec3(-1.0, 0.0, 0.0)), "Raycast normal must point opposite to ray direction");

    // Miss above box
    Ray r2(Vec3(-20.0, 0.0, 15.0), Vec3(1.0, 0.0, 0.0));
    RaycastHit h2 = raycastAABB(r2, box);
    TEST_ASSERT(!h2.hit, "Ray shooting above AABB must miss");

    // Parallel ray outside box
    Ray r3(Vec3(6.0, -20.0, 5.0), Vec3(0.0, 1.0, 0.0));
    RaycastHit h3 = raycastAABB(r3, box);
    TEST_ASSERT(!h3.hit, "Parallel ray outside box must miss without error");

    // Ray starting inside box
    Ray rInside(Vec3(0.0, 0.0, 5.0), Vec3(1.0, 0.0, 0.0));
    RaycastHit hInside = raycastAABB(rInside, box);
    TEST_ASSERT(hInside.hit && hInside.distance == 0.0, "Ray starting inside AABB hits at dist 0");
}

void test_raycast_sphere() {
    Sphere s(Vec3(0.0, 20.0, 0.0), 5.0);

    Ray rHit(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 1.0, 0.0));
    RaycastHit hHit = raycastSphere(rHit, s);
    TEST_ASSERT(hHit.hit, "Direct ray must hit sphere");
    TEST_ASSERT(std::abs(hHit.distance - 15.0) < 1e-6, "Sphere hit distance must be 15 (20 - 5)");

    Ray rMiss(Vec3(0.0, 0.0, 0.0), Vec3(1.0, 0.0, 0.0));
    RaycastHit hMiss = raycastSphere(rMiss, s);
    TEST_ASSERT(!hMiss.hit, "Perpendicular ray must miss sphere");
}

void test_capsule_continuous_collision_swept() {
    // Thin wall building obstacle (thickness 0.2m along X, height 20m)
    AABB wall(Vec3(10.0, -10.0, 0.0), Vec3(10.2, 10.0, 20.0));

    // High-speed swept movement: UAV starts at X=8.0 and ends at X=12.0 in a single timestep!
    // Discretely, neither X=8.0 nor X=12.0 is inside the wall [10.0, 10.2].
    // Continuous swept Capsule MUST catch this tunneling collision!
    Capsule sweptTunneling(Vec3(8.0, 0.0, 10.0), Vec3(12.0, 0.0, 10.0), 0.5);
    TEST_ASSERT(sweptTunneling.intersects(wall), "Swept capsule MUST detect high-speed tunneling through thin obstacle");

    // Clear flight above building (Z=25m, wall height is 20m, radius 0.5m)
    Capsule sweptAbove(Vec3(8.0, 0.0, 25.0), Vec3(12.0, 0.0, 25.0), 0.5);
    TEST_ASSERT(!sweptAbove.intersects(wall), "Swept capsule flying safely above wall must not collide");

    // Flight parallel to wall but within radius margin (wall at Y=10.0, flight at Y=10.3 with radius 0.5)
    Capsule sweptGraze(Vec3(0.0, 10.3, 5.0), Vec3(20.0, 10.3, 5.0), 0.5);
    TEST_ASSERT(sweptGraze.intersects(wall), "Swept capsule grazing wall within radius envelope must collide");

    // Flight parallel to wall safely outside radius margin (flight at Y=11.0 with radius 0.5 -> gap is 0.5m)
    Capsule sweptSafe(Vec3(0.0, 11.0, 5.0), Vec3(20.0, 11.0, 5.0), 0.5);
    TEST_ASSERT(!sweptSafe.intersects(wall), "Swept capsule with clear clearance must not collide");
}

void test_math_utilities() {
    TEST_ASSERT(MathUtils::clamp(15.0, 0.0, 10.0) == 10.0, "MathUtils::clamp upper");
    TEST_ASSERT(MathUtils::clamp(-5.0, 0.0, 10.0) == 0.0, "MathUtils::clamp lower");
    TEST_ASSERT(MathUtils::clamp(5.0, 0.0, 10.0) == 5.0, "MathUtils::clamp inside");

    TEST_ASSERT(std::abs(MathUtils::degToRad(180.0) - MathUtils::PI) < 1e-9, "degToRad 180");
    TEST_ASSERT(std::abs(MathUtils::radToDeg(MathUtils::HALF_PI) - 90.0) < 1e-9, "radToDeg PI/2");

    double angleWrapped = MathUtils::wrapAnglePi(MathUtils::PI + 0.5);
    TEST_ASSERT(angleWrapped < 0.0 && std::abs(angleWrapped - (-MathUtils::PI + 0.5)) < 1e-6, "wrapAnglePi");
}

int main() {
    std::cout << "========================================\n";
    std::cout << "    AeroFleet Math & Geometry Tests     \n";
    std::cout << "========================================\n";

    RUN_TEST(test_sphere_primitives);
    RUN_TEST(test_aabb_primitives);
    RUN_TEST(test_sphere_aabb_intersection);
    RUN_TEST(test_raycast_aabb);
    RUN_TEST(test_raycast_sphere);
    RUN_TEST(test_capsule_continuous_collision_swept);
    RUN_TEST(test_math_utilities);

    std::cout << "========================================\n";
    std::cout << "Summary: " << g_testsPassed << " Passed, " << g_testsFailed << " Failed\n";
    std::cout << "========================================\n";

    return (g_testsFailed == 0) ? 0 : 1;
}

