#pragma once

#include <cmath>
#include <iostream>
#include <string>
#include <sstream>

namespace aerofleet {

/**
 * @brief 3D vector representing coordinates, velocities, or forces in East-North-Up space.
 * X = East (meters)
 * Y = North (meters)
 * Z = Up / Altitude (meters)
 */
struct Vec3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};

    constexpr Vec3() noexcept = default;
    constexpr Vec3(double px, double py, double pz) noexcept : x(px), y(py), z(pz) {}
    constexpr explicit Vec3(double scalar) noexcept : x(scalar), y(scalar), z(scalar) {}

    // Basic arithmetic
    constexpr Vec3 operator+(const Vec3& rhs) const noexcept {
        return Vec3(x + rhs.x, y + rhs.y, z + rhs.z);
    }

    constexpr Vec3 operator-(const Vec3& rhs) const noexcept {
        return Vec3(x - rhs.x, y - rhs.y, z - rhs.z);
    }

    constexpr Vec3 operator-() const noexcept {
        return Vec3(-x, -y, -z);
    }

    constexpr Vec3 operator*(double scalar) const noexcept {
        return Vec3(x * scalar, y * scalar, z * scalar);
    }

    constexpr Vec3 operator/(double scalar) const noexcept {
        return Vec3(x / scalar, y / scalar, z / scalar);
    }

    // Compound assignments
    constexpr Vec3& operator+=(const Vec3& rhs) noexcept {
        x += rhs.x; y += rhs.y; z += rhs.z;
        return *this;
    }

    constexpr Vec3& operator-=(const Vec3& rhs) noexcept {
        x -= rhs.x; y -= rhs.y; z -= rhs.z;
        return *this;
    }

    constexpr Vec3& operator*=(double scalar) noexcept {
        x *= scalar; y *= scalar; z *= scalar;
        return *this;
    }

    constexpr Vec3& operator/=(double scalar) noexcept {
        x /= scalar; y /= scalar; z /= scalar;
        return *this;
    }

    // Vector operations
    [[nodiscard]] constexpr double dot(const Vec3& rhs) const noexcept {
        return x * rhs.x + y * rhs.y + z * rhs.z;
    }

    [[nodiscard]] constexpr Vec3 cross(const Vec3& rhs) const noexcept {
        return Vec3(
            y * rhs.z - z * rhs.y,
            z * rhs.x - x * rhs.z,
            x * rhs.y - y * rhs.x
        );
    }

    [[nodiscard]] constexpr double lengthSquared() const noexcept {
        return x * x + y * y + z * z;
    }

    [[nodiscard]] double length() const noexcept {
        return std::sqrt(lengthSquared());
    }

    /**
     * @brief Return normalized unit vector.
     * Guaranteed to never produce NaN: returns (0, 0, 0) if magnitude < epsilon.
     */
    [[nodiscard]] Vec3 normalized(double epsilon = 1e-12) const noexcept {
        const double magSq = lengthSquared();
        if (magSq < epsilon) {
            return Vec3(0.0, 0.0, 0.0);
        }
        const double invMag = 1.0 / std::sqrt(magSq);
        return Vec3(x * invMag, y * invMag, z * invMag);
    }

    [[nodiscard]] double distance(const Vec3& rhs) const noexcept {
        return (*this - rhs).length();
    }

    [[nodiscard]] constexpr double distanceSquared(const Vec3& rhs) const noexcept {
        return (*this - rhs).lengthSquared();
    }

    [[nodiscard]] constexpr Vec3 lerp(const Vec3& target, double t) const noexcept {
        return *this + (target - *this) * t;
    }

    [[nodiscard]] bool isApprox(const Vec3& rhs, double epsilon = 1e-6) const noexcept {
        return std::abs(x - rhs.x) <= epsilon &&
               std::abs(y - rhs.y) <= epsilon &&
               std::abs(z - rhs.z) <= epsilon;
    }

    constexpr bool operator==(const Vec3& rhs) const noexcept {
        return x == rhs.x && y == rhs.y && z == rhs.z;
    }

    constexpr bool operator!=(const Vec3& rhs) const noexcept {
        return !(*this == rhs);
    }

    [[nodiscard]] std::string toString() const {
        std::ostringstream oss;
        oss << "(" << x << ", " << y << ", " << z << ")";
        return oss.str();
    }
};

// Left scalar multiplication
constexpr inline Vec3 operator*(double scalar, const Vec3& v) noexcept {
    return v * scalar;
}

inline std::ostream& operator<<(std::ostream& os, const Vec3& v) {
    return os << v.toString();
}

} // namespace aerofleet

