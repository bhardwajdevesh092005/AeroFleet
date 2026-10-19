#include "visualization/Renderer.hpp"
#include <iomanip>
#include <sstream>

namespace aerofleet {

namespace {

inline Vector3 toRaylib(const Vec3& v) noexcept {
    // AeroFleet ENU (X=East, Y=North, Z=Up) -> Raylib 3D (X=East, Y=Up, Z=North)
    return Vector3{
        static_cast<float>(v.x),
        static_cast<float>(v.z),
        static_cast<float>(v.y)
    };
}

} // namespace

Renderer::Renderer() : m_config(RendererConfig{}) {}

Renderer::Renderer(const RendererConfig& config) : m_config(config) {}

Renderer::~Renderer() {
    close();
}

bool Renderer::init() {
    if (m_initialized) return true;

    SetTraceLogLevel(LOG_WARNING);
    InitWindow(m_config.screenWidth, m_config.screenHeight, m_config.windowTitle.c_str());

    if (!IsWindowReady()) {
        m_initialized = false;
        return false;
    }

    SetTargetFPS(m_config.targetFPS);
    m_initialized = true;
    return true;
}

void Renderer::close() {
    if (m_initialized && IsWindowReady()) {
        CloseWindow();
        m_initialized = false;
    }
}

bool Renderer::shouldClose() const noexcept {
    if (!m_initialized || !IsWindowReady()) return true;
    return WindowShouldClose();
}

bool Renderer::isReady() const noexcept {
    return m_initialized && IsWindowReady();
}

Color Renderer::getBatteryColor(double percent) noexcept {
    if (percent > 60.0) {
        return Color{46, 204, 113, 255}; // Bright Green
    } else if (percent > 25.0) {
        return Color{241, 196, 15, 255}; // Amber Yellow
    } else {
        return Color{231, 76, 60, 255};  // Warning Red
    }
}

Color Renderer::getModeColor(UAVMode mode) noexcept {
    switch (mode) {
        case UAVMode::IDLE:
            return Color{149, 165, 166, 255}; // Cool Silver/Gray
        case UAVMode::ASSIGNED:
            return Color{243, 156, 18, 255};  // Warm Amber
        case UAVMode::FLYING_TO_PICKUP:
            return Color{46, 204, 113, 255};  // Vibrant Green
        case UAVMode::PICKING_UP:
            return Color{26, 188, 156, 255};  // Turquoise
        case UAVMode::FLYING_TO_DESTINATION:
            return Color{52, 152, 219, 255};  // Vivid Dodger Blue
        case UAVMode::DROPPING_OFF:
            return Color{155, 89, 182, 255};  // Amethyst Purple
        case UAVMode::RETURNING_TO_CHARGE:
            return Color{241, 196, 15, 255};  // Golden Yellow
        case UAVMode::CHARGING:
            return Color{230, 126, 34, 255};  // Electric Orange
        case UAVMode::EMERGENCY:
            return Color{231, 76, 60, 255};   // Crimson Red
        case UAVMode::OFFLINE:
            return Color{52, 73, 94, 255};    // Charcoal Navy
        default:
            return WHITE;
    }
}

void Renderer::cycleSelectedUav(size_t fleetSize) noexcept {
    if (fleetSize == 0) return;
    m_config.selectedUavId = static_cast<UAVId>((m_config.selectedUavId + 1) % fleetSize);
}

void Renderer::draw(const WorldSnapshot& snapshot, const CityMap& map, CameraController& camera) {
    if (!m_initialized || !IsWindowReady()) return;

    // Handle user HUD and selection toggles
    if (IsKeyPressed(KEY_H)) toggleHUD();
    if (IsKeyPressed(KEY_T)) toggleTrajectories();
    if (IsKeyPressed(KEY_L)) toggleShadowLines();
    if (IsKeyPressed(KEY_TAB)) cycleSelectedUav(snapshot.uavs.size());

    // Update camera target
    camera.handleInput(GetFrameTime());
    camera.update(GetFrameTime(), snapshot);

    BeginDrawing();
    ClearBackground(Color{18, 22, 28, 255}); // Modern dark slate

    // 3D Scene Rendering
    draw3DScene(snapshot, map, camera);

    // 2D Telemetry & HUD Overlay
    if (m_config.showHUD) {
        drawHUD(snapshot, camera);
    }

    EndDrawing();
}

void Renderer::draw3DScene(const WorldSnapshot& snapshot, const CityMap& map, const CameraController& camera) {
    Camera3D rayCam = camera.getRaylibCamera();
    BeginMode3D(rayCam);

    // 1. City ground plane & spatial grid
    drawCityGroundAndGrid(map);

    // 2. Static buildings
    drawBuildings(map);

    // 3. Landmark beacons (Stores, Customers, Chargers)
    drawLandmarks(map);

    // 4. Flight path trajectories
    if (m_config.showTrajectories) {
        drawTrajectories(snapshot);
    }

    // 5. Active UAV fleet
    drawUAVs(snapshot);

    EndMode3D();
}

void Renderer::drawCityGroundAndGrid(const CityMap& map) {
    const float width = static_cast<float>(map.worldMax.x - map.worldMin.x);
    const float depth = static_cast<float>(map.worldMax.y - map.worldMin.y);
    const float centerX = static_cast<float>((map.worldMin.x + map.worldMax.x) * 0.5);
    const float centerZ = static_cast<float>((map.worldMin.y + map.worldMax.y) * 0.5);

    // Ground plane
    DrawPlane(Vector3{centerX, 0.0f, centerZ}, Vector2{width, depth}, Color{28, 32, 40, 255});

    // 2D Spatial Grid
    if (m_config.showGrid) {
        const int slices = static_cast<int>(std::max(width, depth) / 50.0f);
        DrawGrid(slices, 50.0f);
    }
}

void Renderer::drawBuildings(const CityMap& map) {
    for (const auto& b : map.buildings) {
        const auto& minP = b.bounds.minPoint;
        const auto& maxP = b.bounds.maxPoint;

        const float w = static_cast<float>(maxP.x - minP.x);
        const float h = static_cast<float>(maxP.z - minP.z);
        const float d = static_cast<float>(maxP.y - minP.y);

        const Vector3 center{
            static_cast<float>((minP.x + maxP.x) * 0.5),
            static_cast<float>(h * 0.5f), // Resting on ground plane Y=0
            static_cast<float>((minP.y + maxP.y) * 0.5)
        };

        // Dark architectural skyscraper body
        DrawCube(center, w, h, d, Color{45, 52, 64, 255});
        DrawCubeWires(center, w, h, d, Color{70, 80, 98, 255});

        // Rooftop landing highlight
        DrawCube(Vector3{center.x, h, center.z}, w, 0.2f, d, Color{55, 65, 80, 255});
    }
}

void Renderer::drawLandmarks(const CityMap& map) {
    // 1. Store Hubs (Green Cylinders)
    for (const auto& store : map.stores) {
        Vector3 pos = toRaylib(store.position);
        pos.y = 0.0f; // Ground baseline

        DrawCylinder(pos, 5.0f, 5.0f, 4.0f, 16, Color{46, 204, 113, 220});
        DrawCylinderWires(pos, 5.0f, 5.0f, 4.0f, 16, Color{39, 174, 96, 255});

        // Vertical beacon pillar
        DrawCylinder(Vector3{pos.x, 15.0f, pos.z}, 0.3f, 0.3f, 30.0f, 8, Color{46, 204, 113, 100});
    }

    // 2. Charging Stations (Orange Docks with landing pads)
    for (const auto& station : map.chargingStations) {
        Vector3 pos = toRaylib(station.position);
        pos.y = 0.0f;

        DrawCylinder(pos, 6.0f, 6.0f, 1.2f, 20, Color{230, 126, 34, 230});
        DrawCylinderWires(pos, 6.0f, 6.0f, 1.2f, 20, Color{243, 156, 18, 255});

        // Occupancy status ring
        Color ringColor = station.hasAvailableSlot() ? Color{46, 204, 113, 255} : Color{231, 76, 60, 255};
        DrawCylinderWires(Vector3{pos.x, 1.3f, pos.z}, 6.2f, 6.2f, 0.2f, 20, ringColor);
    }

    // 3. Customer Delivery Sites (Dodger Blue Pillars)
    for (const auto& cust : map.customers) {
        Vector3 pos = toRaylib(cust.position);
        pos.y = 0.0f;

        DrawCube(Vector3{pos.x, 1.5f, pos.z}, 3.0f, 3.0f, 3.0f, Color{52, 152, 219, 220});
        DrawCubeWires(Vector3{pos.x, 1.5f, pos.z}, 3.0f, 3.0f, 3.0f, Color{41, 128, 185, 255});
    }
}

void Renderer::drawTrajectories(const WorldSnapshot& snapshot) {
    for (const auto& uav : snapshot.uavs) {
        if (uav.mode == UAVMode::IDLE || uav.mode == UAVMode::EMERGENCY || uav.mode == UAVMode::OFFLINE) {
            continue;
        }

        Vector3 start = toRaylib(uav.position);
        Vector3 target = toRaylib(uav.targetWaypoint);

        Color trajColor = getModeColor(uav.mode);
        trajColor.a = 180;

        DrawLine3D(start, target, trajColor);

        // Target waypoint sphere beacon
        DrawSphere(target, 0.8f, Fade(trajColor, 0.6f));
        DrawSphereWires(target, 0.8f, 6, 6, trajColor);
    }
}

void Renderer::drawUAVs(const WorldSnapshot& snapshot) {
    for (const auto& uav : snapshot.uavs) {
        Vector3 pos = toRaylib(uav.position);
        Color modeCol = getModeColor(uav.mode);
        bool isSelected = (uav.id == m_config.selectedUavId);

        // 1. Altitude ground projection shadow line
        if (m_config.showShadowLines && pos.y > 0.1f) {
            Vector3 groundPt{pos.x, 0.0f, pos.z};
            DrawLine3D(pos, groundPt, Color{100, 110, 125, 140});
            DrawCircle3D(groundPt, 1.2f, Vector3{1.0f, 0.0f, 0.0f}, 90.0f, Color{0, 0, 0, 90});
        }

        // 2. UAV 3D airframe representation
        const float uavRadius = 1.0f;
        DrawSphere(pos, uavRadius, modeCol);
        DrawSphereWires(pos, uavRadius, 8, 8, isSelected ? YELLOW : Fade(WHITE, 0.4f));

        if (isSelected) {
            // Highlighting selection ring
            DrawCircle3D(pos, 2.0f, Vector3{0.0f, 1.0f, 0.0f}, 0.0f, Color{241, 196, 15, 220});
        }

        // 3. Directional heading indicator
        // uav.yaw: 0 = +X (East), pi/2 = +Y (North/Raylib +Z)
        const float headingLength = 2.4f;
        const Vector3 headingEnd{
            pos.x + static_cast<float>(std::cos(uav.yaw)) * headingLength,
            pos.y,
            pos.z + static_cast<float>(std::sin(uav.yaw)) * headingLength
        };
        DrawLine3D(pos, headingEnd, WHITE);
        DrawSphere(headingEnd, 0.25f, YELLOW);

        // 4. Transported cargo representation
        if (uav.hasPayload()) {
            Vector3 cargoPos{pos.x, pos.y - 1.2f, pos.z};
            DrawCube(cargoPos, 0.8f, 0.8f, 0.8f, Color{230, 126, 34, 255});
            DrawCubeWires(cargoPos, 0.8f, 0.8f, 0.8f, WHITE);
        }
    }
}

void Renderer::drawHUD(const WorldSnapshot& snapshot, const CameraController& camera) {
    drawTopStatusBar(snapshot, camera);
    drawDroneTelemetryPanel(snapshot);
    drawControlsHelp();
}

void Renderer::drawTopStatusBar(const WorldSnapshot& snapshot, const CameraController& camera) {
    const int barHeight = 44;
    DrawRectangle(0, 0, m_config.screenWidth, barHeight, Color{15, 18, 24, 235});
    DrawLine(0, barHeight, m_config.screenWidth, barHeight, Color{45, 55, 72, 255});

    // App Branding & Time
    DrawText("AeroFleet v1.0", 20, 12, 20, Color{46, 204, 113, 255});

    std::ostringstream timeOss;
    timeOss << "Sim Time: " << std::fixed << std::setprecision(1) << snapshot.simulationTime << "s";
    DrawText(timeOss.str().c_str(), 190, 14, 16, RAYWHITE);

    // Delivery throughput statistics
    std::string statsStr = "Delivered: " + std::to_string(snapshot.metrics.totalDelivered) +
                           "  |  Pending: " + std::to_string(snapshot.metrics.totalPending) +
                           "  |  Active: " + std::to_string(snapshot.metrics.totalActive) +
                           "  |  Collisions: " + std::to_string(snapshot.metrics.collisionCount);
    DrawText(statsStr.c_str(), 350, 14, 16, Color{189, 195, 199, 255});

    // Camera Mode & FPS
    std::string camStr = "Cam: " + cameraModeToString(camera.getMode());
    DrawText(camStr.c_str(), m_config.screenWidth - 220, 14, 16, Color{241, 196, 15, 255});

    std::string fpsStr = "FPS: " + std::to_string(GetFPS());
    DrawText(fpsStr.c_str(), m_config.screenWidth - 90, 14, 16, Color{52, 152, 219, 255});
}

void Renderer::drawDroneTelemetryPanel(const WorldSnapshot& snapshot) {
    const auto* uav = snapshot.findUAV(m_config.selectedUavId);
    if (!uav) return;

    const int panelX = 20;
    const int panelY = 60;
    const int panelW = 280;
    const int panelH = 210;

    // Semi-transparent rounded backdrop
    DrawRectangle(panelX, panelY, panelW, panelH, Color{18, 22, 28, 220});
    DrawRectangleLines(panelX, panelY, panelW, panelH, Color{52, 73, 94, 255});

    // Panel Header
    std::string headerStr = "UAV #" + std::to_string(uav->id) + " Telemetry";
    DrawText(headerStr.c_str(), panelX + 15, panelY + 12, 18, RAYWHITE);

    // Mode Tag
    Color modeColor = getModeColor(uav->mode);
    std::string modeStr = uavModeToString(uav->mode);
    DrawText(modeStr.c_str(), panelX + 15, panelY + 38, 15, modeColor);

    // Battery Bar
    DrawText("Battery:", panelX + 15, panelY + 65, 14, LIGHTGRAY);
    std::ostringstream socOss;
    socOss << std::fixed << std::setprecision(1) << uav->batteryPercent << "%";
    DrawText(socOss.str().c_str(), panelX + 215, panelY + 65, 14, RAYWHITE);

    const int barX = panelX + 15;
    const int barY = panelY + 85;
    const int barW = panelW - 30;
    const int barH = 12;
    DrawRectangle(barX, barY, barW, barH, Color{40, 45, 55, 255});
    const int fillW = static_cast<int>(barW * MathUtils::clamp(uav->batteryPercent / 100.0, 0.0, 1.0));
    DrawRectangle(barX, barY, fillW, barH, getBatteryColor(uav->batteryPercent));

    // Flight Telemetry Details
    std::ostringstream speedOss;
    speedOss << "Speed: " << std::fixed << std::setprecision(1) << uav->speed() << " m/s";
    DrawText(speedOss.str().c_str(), panelX + 15, panelY + 110, 14, RAYWHITE);

    std::ostringstream altOss;
    altOss << "Altitude: " << std::fixed << std::setprecision(1) << uav->position.z << " m";
    DrawText(altOss.str().c_str(), panelX + 140, panelY + 110, 14, RAYWHITE);

    std::ostringstream payloadOss;
    payloadOss << "Payload: " << std::fixed << std::setprecision(2) << uav->payloadMass << " kg";
    DrawText(payloadOss.str().c_str(), panelX + 15, panelY + 135, 14, uav->hasPayload() ? Color{230, 126, 34, 255} : GRAY);

    std::string reqStr = "Mission Req: " + (uav->activeRequestId != -1 ? ("#" + std::to_string(uav->activeRequestId)) : "NONE");
    DrawText(reqStr.c_str(), panelX + 15, panelY + 160, 14, LIGHTGRAY);

    std::ostringstream coordOss;
    coordOss << "Pos: (" << static_cast<int>(uav->position.x) << ", "
             << static_cast<int>(uav->position.y) << ", "
             << static_cast<int>(uav->position.z) << ")";
    DrawText(coordOss.str().c_str(), panelX + 15, panelY + 185, 12, DARKGRAY);
}

void Renderer::drawControlsHelp() {
    const int textY = m_config.screenHeight - 26;
    DrawRectangle(0, textY - 6, m_config.screenWidth, 32, Color{15, 18, 24, 210});
    const char* helpText = "[C] Camera Mode  |  [Tab] Cycle Drone  |  [RMB/MMB Drag] Orbit  |  [Scroll] Zoom  |  [WASD] Pan  |  [H] Toggle HUD";
    DrawText(helpText, 20, textY, 13, Color{149, 165, 166, 255});
}

} // namespace aerofleet
