#include "raylib.h"

#include <algorithm>
#include <cmath>
#include <vector>

struct Building {
    float x;
    float z;
    float width;
    float depth;
    float height;
    Color color;
};

static std::vector<Building> GenerateCity() {
    const float centers[] = {-30.0f, -10.0f, 10.0f, 30.0f};
    const Color palette[] = {
        {190, 116, 91, 255},   // terracotta
        {132, 158, 175, 255},  // blue-grey
        {156, 174, 139, 255},  // sage
        {205, 174, 105, 255},  // ochre
        {177, 151, 181, 255},  // muted violet
        {194, 194, 178, 255}   // warm grey
    };

    std::vector<Building> buildings;
    for (int ix = 0; ix < 4; ++ix) {
        for (int iz = 0; iz < 4; ++iz) {
            const float x = centers[ix];
            const float z = centers[iz];

            // Reserve these blocks for the store, delivery pad, and charger.
            const bool reserved =
                (x == -30.0f && z == -10.0f) ||
                (x ==  10.0f && z ==  30.0f) ||
                (x ==  30.0f && z == -30.0f);
            if (reserved) continue;

            const float height = 7.0f + static_cast<float>((ix * 5 + iz * 3) % 6) * 2.4f;
            buildings.push_back({
                x, z, 7.6f, 7.6f, height,
                palette[(ix * 3 + iz * 2) % 6]
            });
        }
    }
    return buildings;
}

static void DrawBuilding(const Building& b) {
    DrawCube({b.x, b.height * 0.5f, b.z}, b.width, b.height, b.depth, b.color);
    DrawCubeWires({b.x, b.height * 0.5f, b.z}, b.width, b.height, b.depth,
                  Color{62, 67, 72, 255});

    // A darker roof cap helps the buildings read clearly from an elevated camera.
    DrawCube({b.x, b.height + 0.12f, b.z}, b.width + 0.2f, 0.24f,
             b.depth + 0.2f, Color{65, 76, 86, 255});

    const Color windowColor{90, 177, 202, 255};
    for (float y = 1.8f; y < b.height - 0.5f; y += 2.25f) {
        // Windows on the near-facing wall (negative Z).
        for (int col = -1; col <= 1; ++col) {
            DrawCube({b.x + col * 2.0f, y, b.z - b.depth * 0.5f - 0.045f},
                     0.9f, 0.72f, 0.09f, windowColor);
        }
        // Windows on the right-facing wall (positive X).
        for (int col = -1; col <= 1; ++col) {
            DrawCube({b.x + b.width * 0.5f + 0.045f, y, b.z + col * 2.0f},
                     0.09f, 0.72f, 0.9f, windowColor);
        }
    }
}

static void DrawRoads() {
    const Color asphalt{58, 63, 70, 255};
    const Color marking{244, 205, 77, 255};

    // East-west roads.
    for (int z = -40; z <= 40; z += 20) {
        DrawCube({0.0f, 0.025f, static_cast<float>(z)}, 100.0f, 0.07f, 5.0f, asphalt);
        for (int x = -46; x <= 46; x += 6) {
            DrawCube({static_cast<float>(x), 0.068f, static_cast<float>(z)},
                     2.0f, 0.02f, 0.12f, marking);
        }
    }

    // North-south roads.
    for (int x = -40; x <= 40; x += 20) {
        DrawCube({static_cast<float>(x), 0.026f, 0.0f}, 5.0f, 0.075f, 100.0f, asphalt);
        for (int z = -46; z <= 46; z += 6) {
            DrawCube({static_cast<float>(x), 0.07f, static_cast<float>(z)},
                     0.12f, 0.02f, 2.0f, marking);
        }
    }
}

static void DrawPad(Vector3 p, Color color) {
    DrawCube({p.x, 0.18f, p.z}, 5.4f, 0.36f, 5.4f, color);
    DrawCubeWires({p.x, 0.18f, p.z}, 5.4f, 0.36f, 5.4f, Color{35, 42, 48, 255});
    // A small central marker makes each pad visible from the overview camera.
    DrawCube({p.x, 0.55f, p.z}, 0.65f, 0.45f, 0.65f, RAYWHITE);
}

static void DrawDrone(Vector3 p, float timeSeconds) {
    // Body and arms.
    DrawCube({p.x, p.y, p.z}, 0.95f, 0.28f, 0.75f, BLUE);
    DrawCubeWires({p.x, p.y, p.z}, 0.95f, 0.28f, 0.75f, DARKBLUE);
    DrawCube({p.x, p.y, p.z}, 2.0f, 0.10f, 0.14f, DARKGRAY);
    DrawCube({p.x, p.y, p.z}, 0.14f, 0.10f, 2.0f, DARKGRAY);

    const float rotorOffset = 0.76f;
    const float rotorY = p.y + 0.14f;
    const float angle = timeSeconds * 13.0f;
    const float bladeX = std::cos(angle) * 0.48f;
    const float bladeZ = std::sin(angle) * 0.48f;

    for (int sx : {-1, 1}) {
        for (int sz : {-1, 1}) {
            const float rx = p.x + sx * rotorOffset;
            const float rz = p.z + sz * rotorOffset;
            DrawSphere({rx, rotorY, rz}, 0.16f, DARKGRAY);
            DrawLine3D({rx - bladeX, rotorY + 0.04f, rz - bladeZ},
                       {rx + bladeX, rotorY + 0.04f, rz + bladeZ}, BLACK);
        }
    }
}

static bool CollidesWithBuilding(Vector3 p, const std::vector<Building>& buildings) {
    constexpr float droneRadius = 0.55f;
    constexpr float droneHalfHeight = 0.20f;

    for (const Building& b : buildings) {
        const bool overlapX = std::fabs(p.x - b.x) < b.width * 0.5f + droneRadius;
        const bool overlapZ = std::fabs(p.z - b.z) < b.depth * 0.5f + droneRadius;
        const bool overlapY = (p.y - droneHalfHeight) < b.height &&
                              (p.y + droneHalfHeight) > 0.0f;
        if (overlapX && overlapZ && overlapY) return true;
    }
    return false;
}

int main() {
    SetTraceLogLevel(LOG_INFO);
    SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_VSYNC_HINT);

    InitWindow(1280, 720, "AeroFleet | C++ raylib 3D City Demo");
    if (!IsWindowReady()) return 1;  // fail loudly instead of hanging

    SetTargetFPS(60);

    const std::vector<Building> buildings = GenerateCity();

    const Vector3 storePos{ -30.0f, 0.0f, -10.0f };
    const Vector3 deliveryPos{ 10.0f, 0.0f, 30.0f };
    const Vector3 chargerPos{ 30.0f, 0.0f, -30.0f };

    Vector3 dronePos{storePos.x, 4.0f, storePos.z};
    bool followCamera = true;

    Camera3D camera{};
    camera.position = {dronePos.x + 22.0f, dronePos.y + 15.0f, dronePos.z + 25.0f};
    camera.target = {dronePos.x, dronePos.y, dronePos.z};
    camera.up = {0.0f, 1.0f, 0.0f};
    camera.fovy = 55.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    while (!WindowShouldClose()) {
        // Clamp dt so a long first frame can't make the drone jump.
        const float dt = std::min(GetFrameTime(), 0.05f);
        const float boost = (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) ? 2.2f : 1.0f;
        const float speed = 10.0f * boost * dt;

        if (IsKeyPressed(KEY_C)) followCamera = !followCamera;

        // World-axis controls: W/S change Z, A/D change X, E/Q change altitude.
        const float moveX = static_cast<float>(IsKeyDown(KEY_D)) -
                            static_cast<float>(IsKeyDown(KEY_A));
        const float moveZ = static_cast<float>(IsKeyDown(KEY_S)) -
                            static_cast<float>(IsKeyDown(KEY_W));
        const float moveY = static_cast<float>(IsKeyDown(KEY_E)) -
                            static_cast<float>(IsKeyDown(KEY_Q));

        Vector3 candidate = dronePos;
        candidate.x += moveX * speed;
        if (candidate.x >= -47.0f && candidate.x <= 47.0f &&
            !CollidesWithBuilding(candidate, buildings)) {
            dronePos.x = candidate.x;
        }

        candidate = dronePos;
        candidate.z += moveZ * speed;
        if (candidate.z >= -47.0f && candidate.z <= 47.0f &&
            !CollidesWithBuilding(candidate, buildings)) {
            dronePos.z = candidate.z;
        }

        candidate = dronePos;
        candidate.y += moveY * speed;
        if (candidate.y >= 1.2f && candidate.y <= 24.0f &&
            !CollidesWithBuilding(candidate, buildings)) {
            dronePos.y = candidate.y;
        }

        if (followCamera) {
            // Fixed diagonal view from behind and above the drone.
            camera.position = {dronePos.x + 22.0f, dronePos.y + 15.0f, dronePos.z + 25.0f};
            camera.target = {dronePos.x, dronePos.y + 0.4f, dronePos.z};
        } else {
            // Overview of the whole small city.
            camera.position = {76.0f, 68.0f, 76.0f};
            camera.target = {0.0f, 2.0f, 0.0f};
        }

        BeginDrawing();
        ClearBackground(Color{135, 195, 232, 255});

        BeginMode3D(camera);
        DrawCube({0.0f, -0.28f, 0.0f}, 100.0f, 0.5f, 100.0f,
                 Color{87, 142, 83, 255});
        DrawRoads();

        // Delivery-location markers: orange store, green customer, blue charger.
        DrawPad(storePos, ORANGE);
        DrawPad(deliveryPos, GREEN);
        DrawPad(chargerPos, SKYBLUE);

        for (const Building& b : buildings) DrawBuilding(b);
        DrawDrone(dronePos, static_cast<float>(GetTime()));
        EndMode3D();

        // Readable HUD in screen space.
        DrawRectangle(12, 12, 570, 154, Fade(BLACK, 0.68f));
        DrawText("AEROFLEET | C++ / RAYLIB CITY DEMO", 25, 22, 21, RAYWHITE);
        DrawText("W/S: forward-back   A/D: left-right   E/Q: up-down", 25, 53, 17, RAYWHITE);
        DrawText("SHIFT: boost   C: toggle diagonal follow / city overview", 25, 78, 17, RAYWHITE);
        DrawText(TextFormat("Drone: X %.1f   Y %.1f   Z %.1f", dronePos.x, dronePos.y, dronePos.z),
                 25, 103, 17, YELLOW);
        DrawText("Orange: store   Green: delivery   Blue: charging station", 25, 130, 17, RAYWHITE);
        DrawFPS(GetScreenWidth() - 90, 15);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}