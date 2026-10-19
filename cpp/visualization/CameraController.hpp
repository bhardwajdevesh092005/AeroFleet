#pragma once

#include <cmath>
#include <string>
#include <algorithm>
#include "math/Vec3.hpp"
#include "math/MathUtils.hpp"
#include "core/WorldState.hpp"
#include <raylib.h>
#if defined(PI)
#undef PI
#endif

namespace aerofleet {

enum class CameraMode : uint8_t {
    ORBIT = 0,    ///< Orbital camera rotating around city center or focus point
    CHASE,        ///< Third-person chase camera tracking selected UAV from behind
    TOP_DOWN,     ///< Orthogonal-style overhead top-down perspective
    FREE          ///< Free-fly camera with WASD and mouse controls
};

inline std::string cameraModeToString(CameraMode mode) {
    switch (mode) {
        case CameraMode::ORBIT: return "ORBIT";
        case CameraMode::CHASE: return "CHASE";
        case CameraMode::TOP_DOWN: return "TOP-DOWN";
        case CameraMode::FREE: return "FREE";
        default: return "UNKNOWN";
    }
}

/**
 * @brief Camera controller managing 3D perspective views, orbital rotation,
 * and UAV chase camera modes.
 */
class CameraController {
public:
    CameraController();
    explicit CameraController(const Vec3& initialTarget, double distance = 350.0);

    /**
     * @brief Computes 3D Cartesian camera eye coordinate from spherical orbit angles.
     */
    [[nodiscard]] static Vec3 computeOrbitPosition(
        const Vec3& target,
        double distance,
        double azimuthRad,
        double elevationRad
    ) noexcept;

    /**
     * @brief Computes third-person chase camera eye coordinate behind a UAV.
     */
    [[nodiscard]] static Vec3 computeChasePosition(
        const Vec3& uavPos,
        double uavYaw,
        double distance,
        double height
    ) noexcept;

    /**
     * @brief Updates camera position and target based on active mode and current world snapshot.
     */
    void update(double dt, const WorldSnapshot& snapshot);

    /**
     * @brief Polls user mouse/keyboard input to pan, rotate, zoom, or cycle modes.
     */
    void handleInput(double dt);

    // Mode and target selection
    void setMode(CameraMode mode) noexcept { m_mode = mode; }
    [[nodiscard]] CameraMode getMode() const noexcept { return m_mode; }
    void cycleMode() noexcept;

    void setChaseTarget(UAVId id) noexcept { m_chaseTargetId = id; }
    [[nodiscard]] UAVId getChaseTarget() const noexcept { return m_chaseTargetId; }
    void cycleChaseTarget(size_t fleetSize) noexcept;

    // Camera parameters
    [[nodiscard]] const Vec3& getPosition() const noexcept { return m_position; }
    [[nodiscard]] const Vec3& getTarget() const noexcept { return m_target; }
    [[nodiscard]] const Vec3& getUp() const noexcept { return m_up; }
    [[nodiscard]] double getFov() const noexcept { return m_fov; }

    void setTarget(const Vec3& target) noexcept { m_target = target; }
    void setDistance(double dist) noexcept { m_distance = MathUtils::clamp(dist, 10.0, 2000.0); }
    [[nodiscard]] double getDistance() const noexcept { return m_distance; }

    [[nodiscard]] double getAzimuth() const noexcept { return m_azimuth; }
    [[nodiscard]] double getElevation() const noexcept { return m_elevation; }

    /**
     * @brief Construct raylib Camera3D representation.
     */
    [[nodiscard]] Camera3D getRaylibCamera() const noexcept;

private:
    CameraMode m_mode{CameraMode::ORBIT};
    Vec3 m_position{200.0, 200.0, 200.0};
    Vec3 m_target{0.0, 0.0, 15.0};
    Vec3 m_up{0.0, 0.0, 1.0}; // Z-Up in AeroFleet coordinate space
    double m_fov{45.0};

    // Orbit parameters
    double m_distance{350.0};
    double m_azimuth{MathUtils::PI * 0.25};     // 45 degrees
    double m_elevation{MathUtils::PI * 0.20};   // ~36 degrees

    // Chase parameters
    UAVId m_chaseTargetId{0};
    double m_chaseDistance{20.0};
    double m_chaseHeight{8.0};

    // Top-down parameters
    double m_topDownAltitude{450.0};
};

} // namespace aerofleet

