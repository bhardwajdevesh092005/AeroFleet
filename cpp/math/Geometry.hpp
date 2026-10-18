#pragma once

#include <cmath>
#include <algorithm>
#include <limits>
#include <array>
#include "math/Vec3.hpp"
#include "math/MathUtils.hpp"

namespace aerofleet {

/**
 * @brief 3D Ray with origin and normalized direction.
 */
struct Ray {
    Vec3 origin{0.0, 0.0, 0.0};
    Vec3 direction{0.0, 0.0, 1.0};

    constexpr Ray() noexcept = default;
    Ray(const Vec3& orig, const Vec3& dir) noexcept
        : origin(orig), direction(dir.normalized()) {}

    [[nodiscard]] constexpr Vec3 at(double t) const noexcept {
        return origin + direction * t;
    }
};

/**
 * @brief Raycast intersection result structure.
 */
struct RaycastHit {
    bool hit{false};
    double distance{std::numeric_limits<double>::infinity()};
    Vec3 point{0.0, 0.0, 0.0};
    Vec3 normal{0.0, 0.0, 0.0};
};

/**
 * @brief Bounding Sphere primitive.
 */
struct Sphere {
    Vec3 center{0.0, 0.0, 0.0};
    double radius{0.5};

    constexpr Sphere() noexcept = default;
    constexpr Sphere(const Vec3& c, double r) noexcept : center(c), radius(r) {}

    [[nodiscard]] bool contains(const Vec3& pt) const noexcept {
        return center.distanceSquared(pt) <= (radius * radius);
    }

    [[nodiscard]] bool intersects(const Sphere& other) const noexcept {
        const double rSum = radius + other.radius;
        return center.distanceSquared(other.center) <= (rSum * rSum);
    }
};

/**
 * @brief 3D Axis-Aligned Bounding Box (AABB).
 */
struct AABB {
    Vec3 minPoint{-1.0, -1.0, -1.0};
    Vec3 maxPoint{1.0, 1.0, 1.0};

    constexpr AABB() noexcept = default;
    constexpr AABB(const Vec3& minP, const Vec3& maxP) noexcept : minPoint(minP), maxPoint(maxP) {}

    [[nodiscard]] constexpr Vec3 center() const noexcept {
        return (minPoint + maxPoint) * 0.5;
    }

    [[nodiscard]] constexpr Vec3 extents() const noexcept {
        return (maxPoint - minPoint) * 0.5;
    }

    [[nodiscard]] constexpr Vec3 size() const noexcept {
        return maxPoint - minPoint;
    }

    [[nodiscard]] constexpr bool contains(const Vec3& pt) const noexcept {
        return pt.x >= minPoint.x && pt.x <= maxPoint.x &&
               pt.y >= minPoint.y && pt.y <= maxPoint.y &&
               pt.z >= minPoint.z && pt.z <= maxPoint.z;
    }

    [[nodiscard]] constexpr bool intersects(const AABB& other) const noexcept {
        if (maxPoint.x < other.minPoint.x || minPoint.x > other.maxPoint.x) return false;
        if (maxPoint.y < other.minPoint.y || minPoint.y > other.maxPoint.y) return false;
        if (maxPoint.z < other.minPoint.z || minPoint.z > other.maxPoint.z) return false;
        return true;
    }

    [[nodiscard]] Vec3 closestPoint(const Vec3& pt) const noexcept {
        return Vec3(
            MathUtils::clamp(pt.x, minPoint.x, maxPoint.x),
            MathUtils::clamp(pt.y, minPoint.y, maxPoint.y),
            MathUtils::clamp(pt.z, minPoint.z, maxPoint.z)
        );
    }

    [[nodiscard]] double distanceSquared(const Vec3& pt) const noexcept {
        const Vec3 cp = closestPoint(pt);
        return pt.distanceSquared(cp);
    }

    [[nodiscard]] double distance(const Vec3& pt) const noexcept {
        return std::sqrt(distanceSquared(pt));
    }

    [[nodiscard]] bool intersects(const Sphere& sphere) const noexcept {
        return distanceSquared(sphere.center) <= (sphere.radius * sphere.radius);
    }

    void expand(double margin) noexcept {
        minPoint -= Vec3(margin);
        maxPoint += Vec3(margin);
    }

    [[nodiscard]] constexpr AABB expanded(double margin) const noexcept {
        return AABB(minPoint - Vec3(margin), maxPoint + Vec3(margin));
    }
};

/**
 * @brief Capsule primitive defined by segment AB and radius.
 * Ideal for Continuous Collision Detection (swept sphere movement).
 */
struct Capsule {
    Vec3 a{0.0, 0.0, 0.0};
    Vec3 b{0.0, 0.0, 0.0};
    double radius{0.5};

    constexpr Capsule() noexcept = default;
    constexpr Capsule(const Vec3& p1, const Vec3& p2, double r) noexcept : a(p1), b(p2), radius(r) {}

    [[nodiscard]] constexpr Vec3 segmentVector() const noexcept {
        return b - a;
    }

    [[nodiscard]] double segmentLength() const noexcept {
        return (b - a).length();
    }

    [[nodiscard]] Vec3 closestPointOnSegment(const Vec3& pt) const noexcept {
        const Vec3 ab = b - a;
        const double abLenSq = ab.lengthSquared();
        if (abLenSq < MathUtils::EPSILON) {
            return a;
        }
        const double t = MathUtils::clamp((pt - a).dot(ab) / abLenSq, 0.0, 1.0);
        return a + ab * t;
    }

    [[nodiscard]] double distanceSquaredToPoint(const Vec3& pt) const noexcept {
        const Vec3 cp = closestPointOnSegment(pt);
        return cp.distanceSquared(pt);
    }

    [[nodiscard]] bool contains(const Vec3& pt) const noexcept {
        return distanceSquaredToPoint(pt) <= (radius * radius);
    }

    [[nodiscard]] bool intersects(const Sphere& sphere) const noexcept {
        const double totalRadius = radius + sphere.radius;
        return distanceSquaredToPoint(sphere.center) <= (totalRadius * totalRadius);
    }

    /**
     * @brief Computes squared distance between two 3D line segments.
     * Analytical closed-form algorithm (Ericson RTCD 5.1.9).
     */
    static double segmentSegmentDistanceSquared(
        const Vec3& p1, const Vec3& q1,
        const Vec3& p2, const Vec3& q2
    ) noexcept {
        const Vec3 d1 = q1 - p1;
        const Vec3 d2 = q2 - p2;
        const Vec3 r = p1 - p2;

        const double a = d1.lengthSquared();
        const double e = d2.lengthSquared();
        const double f = d2.dot(r);

        double s = 0.0;
        double t = 0.0;

        if (a <= MathUtils::EPSILON && e <= MathUtils::EPSILON) {
            return r.lengthSquared();
        }
        if (a <= MathUtils::EPSILON) {
            s = 0.0;
            t = MathUtils::clamp(f / e, 0.0, 1.0);
        } else {
            const double c = d1.dot(r);
            if (e <= MathUtils::EPSILON) {
                t = 0.0;
                s = MathUtils::clamp(-c / a, 0.0, 1.0);
            } else {
                const double b = d1.dot(d2);
                const double denom = a * e - b * b;

                if (denom > MathUtils::EPSILON) {
                    s = MathUtils::clamp((b * f - c * e) / denom, 0.0, 1.0);
                } else {
                    s = 0.0;
                }

                t = (b * s + f) / e;

                if (t < 0.0) {
                    t = 0.0;
                    s = MathUtils::clamp(-c / a, 0.0, 1.0);
                } else if (t > 1.0) {
                    t = 1.0;
                    s = MathUtils::clamp((b - c) / a, 0.0, 1.0);
                }
            }
        }

        const Vec3 c1 = p1 + d1 * s;
        const Vec3 c2 = p2 + d2 * t;
        return (c1 - c2).lengthSquared();
    }

    /**
     * @brief Tests if the Capsule intersects an AABB.
     * Evaluates endpoints, segment-box interior intersection, and all 12 AABB edges.
     */
    [[nodiscard]] bool intersects(const AABB& aabb) const noexcept {
        const double rSq = radius * radius;

        // 1. Check if either endpoint sphere overlaps the AABB
        if (aabb.distanceSquared(a) <= rSq) return true;
        if (aabb.distanceSquared(b) <= rSq) return true;

        // 2. Check if segment AB intersects the AABB volume directly
        const Vec3 ab = b - a;
        double tMin = 0.0;
        double tMax = 1.0;

        for (int i = 0; i < 3; ++i) {
            const double pOrigin = (i == 0) ? a.x : ((i == 1) ? a.y : a.z);
            const double pDir = (i == 0) ? ab.x : ((i == 1) ? ab.y : ab.z);
            const double boxMin = (i == 0) ? aabb.minPoint.x : ((i == 1) ? aabb.minPoint.y : aabb.minPoint.z);
            const double boxMax = (i == 0) ? aabb.maxPoint.x : ((i == 1) ? aabb.maxPoint.y : aabb.maxPoint.z);

            if (std::abs(pDir) < MathUtils::EPSILON) {
                if (pOrigin < boxMin || pOrigin > boxMax) {
                    tMin = 2.0; // No overlap along this axis
                    break;
                }
            } else {
                double t1 = (boxMin - pOrigin) / pDir;
                double t2 = (boxMax - pOrigin) / pDir;
                if (t1 > t2) std::swap(t1, t2);

                tMin = std::max(tMin, t1);
                tMax = std::min(tMax, t2);

                if (tMin > tMax) break;
            }
        }

        if (tMin <= tMax) {
            return true; // Segment passes directly through the AABB volume
        }

        // 3. Check distance from segment AB to all 12 edges of the AABB
        const double x0 = aabb.minPoint.x, x1 = aabb.maxPoint.x;
        const double y0 = aabb.minPoint.y, y1 = aabb.maxPoint.y;
        const double z0 = aabb.minPoint.z, z1 = aabb.maxPoint.z;

        // The 12 AABB edges
        const std::array<std::pair<Vec3, Vec3>, 12> edges = {{
            // 4 edges parallel to X axis
            {Vec3(x0, y0, z0), Vec3(x1, y0, z0)},
            {Vec3(x0, y1, z0), Vec3(x1, y1, z0)},
            {Vec3(x0, y0, z1), Vec3(x1, y0, z1)},
            {Vec3(x0, y1, z1), Vec3(x1, y1, z1)},
            // 4 edges parallel to Y axis
            {Vec3(x0, y0, z0), Vec3(x0, y1, z0)},
            {Vec3(x1, y0, z0), Vec3(x1, y1, z0)},
            {Vec3(x0, y0, z1), Vec3(x0, y1, z1)},
            {Vec3(x1, y0, z1), Vec3(x1, y1, z1)},
            // 4 edges parallel to Z axis
            {Vec3(x0, y0, z0), Vec3(x0, y0, z1)},
            {Vec3(x1, y0, z0), Vec3(x1, y0, z1)},
            {Vec3(x0, y1, z0), Vec3(x0, y1, z1)},
            {Vec3(x1, y1, z0), Vec3(x1, y1, z1)}
        }};

        for (const auto& [e0, e1] : edges) {
            if (segmentSegmentDistanceSquared(a, b, e0, e1) <= rSq) {
                return true;
            }
        }

        return false;
    }
};

/**
 * @brief Analytical Ray-AABB intersection using the Slab Method.
 */
inline RaycastHit raycastAABB(const Ray& ray, const AABB& aabb, double maxDist = 1e9) noexcept {
    RaycastHit result;
    double tMin = 0.0;
    double tMax = maxDist;
    Vec3 enterNormal(0.0, 0.0, 0.0);

    for (int i = 0; i < 3; ++i) {
        const double rayOrig = (i == 0) ? ray.origin.x : ((i == 1) ? ray.origin.y : ray.origin.z);
        const double rayDir = (i == 0) ? ray.direction.x : ((i == 1) ? ray.direction.y : ray.direction.z);
        const double boxMin = (i == 0) ? aabb.minPoint.x : ((i == 1) ? aabb.minPoint.y : aabb.minPoint.z);
        const double boxMax = (i == 0) ? aabb.maxPoint.x : ((i == 1) ? aabb.maxPoint.y : aabb.maxPoint.z);

        Vec3 axisNormal(0.0, 0.0, 0.0);
        if (i == 0) axisNormal.x = 1.0;
        else if (i == 1) axisNormal.y = 1.0;
        else axisNormal.z = 1.0;

        if (std::abs(rayDir) < MathUtils::EPSILON) {
            // Ray is parallel to slab; check if origin is outside slab
            if (rayOrig < boxMin || rayOrig > boxMax) {
                return result; // Miss
            }
        } else {
            const double invD = 1.0 / rayDir;
            double t0 = (boxMin - rayOrig) * invD;
            double t1 = (boxMax - rayOrig) * invD;
            Vec3 norm = -axisNormal;

            if (t0 > t1) {
                std::swap(t0, t1);
                norm = axisNormal;
            }

            if (t0 > tMin) {
                tMin = t0;
                enterNormal = norm;
            }
            tMax = std::min(tMax, t1);

            if (tMin > tMax) {
                return result; // Miss
            }
        }
    }

    if (tMin <= maxDist && tMin >= 0.0) {
        result.hit = true;
        result.distance = tMin;
        result.point = ray.at(tMin);
        result.normal = enterNormal;
    } else if (aabb.contains(ray.origin)) {
        // Ray starts inside box
        result.hit = true;
        result.distance = 0.0;
        result.point = ray.origin;
        result.normal = -ray.direction;
    }

    return result;
}

/**
 * @brief Analytical Ray-Sphere intersection.
 */
inline RaycastHit raycastSphere(const Ray& ray, const Sphere& sphere, double maxDist = 1e9) noexcept {
    RaycastHit result;
    const Vec3 m = ray.origin - sphere.center;
    const double b = m.dot(ray.direction);
    const double c = m.lengthSquared() - (sphere.radius * sphere.radius);

    // If ray origin is outside sphere and ray direction points away from sphere
    if (c > 0.0 && b > 0.0) {
        return result;
    }

    const double discr = b * b - c;
    if (discr < 0.0) {
        return result; // Ray misses sphere
    }

    double t = -b - std::sqrt(discr);
    if (t < 0.0) {
        // Ray started inside sphere
        t = 0.0;
    }

    if (t <= maxDist) {
        result.hit = true;
        result.distance = t;
        result.point = ray.at(t);
        result.normal = (result.point - sphere.center).normalized();
    }

    return result;
}

} // namespace aerofleet

