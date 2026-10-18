#pragma once

#include <cmath>
#include <algorithm>
#include <numbers>

namespace aerofleet {

namespace MathUtils {

    constexpr double PI = 3.14159265358979323846;
    constexpr double TWO_PI = 2.0 * PI;
    constexpr double HALF_PI = 0.5 * PI;
    constexpr double EPSILON = 1e-9;

    template<typename T>
    constexpr T clamp(T val, T low, T high) noexcept {
        return (val < low) ? low : ((val > high) ? high : val);
    }

    template<typename T>
    constexpr T lerp(T a, T b, double t) noexcept {
        return a + static_cast<T>((b - a) * t);
    }

    constexpr double degToRad(double degrees) noexcept {
        return degrees * (PI / 180.0);
    }

    constexpr double radToDeg(double radians) noexcept {
        return radians * (180.0 / PI);
    }

    constexpr double absVal(double val) noexcept {
        return (val < 0.0) ? -val : val;
    }

    constexpr bool isZero(double val, double eps = EPSILON) noexcept {
        return absVal(val) <= eps;
    }

    constexpr bool areEqual(double a, double b, double eps = EPSILON) noexcept {
        return absVal(a - b) <= eps;
    }

    /**
     * @brief Normalizes an angle into the range [-PI, PI].
     */
    inline double wrapAnglePi(double rad) noexcept {
        double angle = std::fmod(rad + PI, TWO_PI);
        if (angle < 0.0) {
            angle += TWO_PI;
        }
        return angle - PI;
    }

} // namespace MathUtils

} // namespace aerofleet

