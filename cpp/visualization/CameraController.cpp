#include "visualization/CameraController.hpp"

namespace aerofleet {

namespace {

inline Vector3 toRaylibCoords(const Vec3& v) noexcept {
    // AeroFleet ENU: X = East, Y = North, Z = Up
    // Raylib 3D:     X = East, Y = Up,    Z = North
    return Vector3{
        static_cast<float>(v.x),
        static_cast<float>(v.z),
        static_cast<float>(v.y)
    };
}

} // namespace

CameraController::CameraController() {
    m_position = computeOrbitPosition(m_target, m_distance, m_azimuth, m_elevation);
}

CameraController::CameraController(const Vec3& initialTarget, double distance)
    : m_target(initialTarget), m_distance(distance) {
    m_position = computeOrbitPosition(m_target, m_distance, m_azimuth, m_elevation);
}

Vec3 CameraController::computeOrbitPosition(
    const Vec3& target,
    double distance,
    double azimuthRad,
    double elevationRad
) noexcept {
    const double cosEl = std::cos(elevationRad);
    const double sinEl = std::sin(elevationRad);
    const double cosAz = std::cos(azimuthRad);
    const double sinAz = std::sin(azimuthRad);

    return Vec3(
        target.x + distance * cosEl * cosAz,
        target.y + distance * cosEl * sinAz,
        target.z + distance * sinEl
    );
}

Vec3 CameraController::computeChasePosition(
    const Vec3& uavPos,
    double uavYaw,
    double distance,
    double height
) noexcept {
    // uavYaw = 0 points along +X (East), pi/2 points along +Y (North)
    const double behindX = -std::cos(uavYaw) * distance;
    const double behindY = -std::sin(uavYaw) * distance;

    return Vec3(
        uavPos.x + behindX,
        uavPos.y + behindY,
        uavPos.z + height
    );
}

void CameraController::cycleMode() noexcept {
    switch (m_mode) {
        case CameraMode::ORBIT:
            m_mode = CameraMode::CHASE;
            break;
        case CameraMode::CHASE:
            m_mode = CameraMode::TOP_DOWN;
            break;
        case CameraMode::TOP_DOWN:
            m_mode = CameraMode::ORBIT;
            break;
        case CameraMode::FREE:
            m_mode = CameraMode::ORBIT;
            break;
    }
}

void CameraController::cycleChaseTarget(size_t fleetSize) noexcept {
    if (fleetSize == 0) return;
    m_chaseTargetId = static_cast<UAVId>((m_chaseTargetId + 1) % fleetSize);
}

void CameraController::update(double dt, const WorldSnapshot& snapshot) {
    switch (m_mode) {
        case CameraMode::ORBIT: {
            m_position = computeOrbitPosition(m_target, m_distance, m_azimuth, m_elevation);
            m_up = Vec3(0.0, 0.0, 1.0);
            break;
        }
        case CameraMode::CHASE: {
            const auto* uav = snapshot.findUAV(m_chaseTargetId);
            if (uav) {
                const Vec3 idealPos = computeChasePosition(uav->position, uav->yaw, m_chaseDistance, m_chaseHeight);
                const Vec3 idealTarget = uav->position + Vec3(0.0, 0.0, 1.0);

                // Smooth camera dampening
                const double lerpFactor = MathUtils::clamp(dt * 8.0, 0.1, 1.0);
                m_position = m_position.lerp(idealPos, lerpFactor);
                m_target = m_target.lerp(idealTarget, lerpFactor);
                m_up = Vec3(0.0, 0.0, 1.0);
            } else {
                // Target UAV not found; fall back to orbit
                m_position = computeOrbitPosition(m_target, m_distance, m_azimuth, m_elevation);
            }
            break;
        }
        case CameraMode::TOP_DOWN: {
            m_position = Vec3(m_target.x, m_target.y + 0.001, m_target.z + m_topDownAltitude);
            m_up = Vec3(0.0, 1.0, 0.0); // Up points North in top-down view
            break;
        }
        case CameraMode::FREE: {
            // Free mode position and target updated in handleInput
            break;
        }
    }
}

void CameraController::handleInput(double dt) {
    if (!IsWindowReady()) return;

    // 1. Mode cycling hotkeys
    if (IsKeyPressed(KEY_C)) {
        cycleMode();
    }
    if (IsKeyPressed(KEY_TAB)) {
        m_chaseTargetId = (m_chaseTargetId + 1) % 10;
    }

    // 2. Mouse interactions for orbit mode
    if (m_mode == CameraMode::ORBIT) {
        if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT) || IsMouseButtonDown(MOUSE_BUTTON_MIDDLE)) {
            const Vector2 delta = GetMouseDelta();
            m_azimuth -= delta.x * 0.005;
            m_elevation += delta.y * 0.005;

            // Clamp elevation between ~5 degrees and 85 degrees
            m_elevation = MathUtils::clamp(m_elevation, 0.08, MathUtils::PI * 0.48);
            m_azimuth = MathUtils::wrapAnglePi(m_azimuth);
        }

        const float wheel = GetMouseWheelMove();
        if (std::abs(wheel) > 0.001f) {
            m_distance -= wheel * 25.0;
            m_distance = MathUtils::clamp(m_distance, 20.0, 1500.0);
        }

        // Panning with arrow keys or WASD
        const double panSpeed = m_distance * 0.5 * dt;
        if (IsKeyDown(KEY_W) || IsKeyDown(KEY_UP)) {
            m_target.y += panSpeed;
        }
        if (IsKeyDown(KEY_S) || IsKeyDown(KEY_DOWN)) {
            m_target.y -= panSpeed;
        }
        if (IsKeyDown(KEY_A) || IsKeyDown(KEY_LEFT)) {
            m_target.x -= panSpeed;
        }
        if (IsKeyDown(KEY_D) || IsKeyDown(KEY_RIGHT)) {
            m_target.x += panSpeed;
        }
    } else if (m_mode == CameraMode::CHASE) {
        const float wheel = GetMouseWheelMove();
        if (std::abs(wheel) > 0.001f) {
            m_chaseDistance -= wheel * 3.0;
            m_chaseDistance = MathUtils::clamp(m_chaseDistance, 5.0, 60.0);
        }
    } else if (m_mode == CameraMode::TOP_DOWN) {
        const float wheel = GetMouseWheelMove();
        if (std::abs(wheel) > 0.001f) {
            m_topDownAltitude -= wheel * 30.0;
            m_topDownAltitude = MathUtils::clamp(m_topDownAltitude, 50.0, 1500.0);
        }
        const double panSpeed = m_topDownAltitude * 0.5 * dt;
        if (IsKeyDown(KEY_W) || IsKeyDown(KEY_UP)) m_target.y += panSpeed;
        if (IsKeyDown(KEY_S) || IsKeyDown(KEY_DOWN)) m_target.y -= panSpeed;
        if (IsKeyDown(KEY_A) || IsKeyDown(KEY_LEFT)) m_target.x -= panSpeed;
        if (IsKeyDown(KEY_D) || IsKeyDown(KEY_RIGHT)) m_target.x += panSpeed;
    }
}

Camera3D CameraController::getRaylibCamera() const noexcept {
    Camera3D cam;
    cam.position = toRaylibCoords(m_position);
    cam.target = toRaylibCoords(m_target);
    cam.up = toRaylibCoords(m_up);
    cam.fovy = static_cast<float>(m_fov);
    cam.projection = CAMERA_PERSPECTIVE;
    return cam;
}

} // namespace aerofleet

