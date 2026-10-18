#include <iostream>
#include <string>
#include <vector>
#include <raylib.h>
#include "core/Config.hpp"
#include "core/Types.hpp"
#include "math/Vec3.hpp"

using namespace aerofleet;

int main(int argc, char* argv[]) {
    std::string configPath = "config/simulation.json";
    bool headless = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--config" && i + 1 < argc) {
            configPath = argv[++i];
        } else if (arg == "--headless") {
            headless = true;
        }
    }

    std::cout << "========================================\n";
    std::cout << "         AeroFleet Simulation           \n";
    std::cout << "========================================\n";
    std::cout << "[System] Loading configuration from: " << configPath << "\n";

    SimulationConfig config;
    try {
        config = SimulationConfig::loadFromFile(configPath);
        std::cout << "[System] Loaded: " << config.simulation.name
                  << " v" << config.simulation.version << "\n";
        std::cout << "[System] World Volume: "
                  << config.world.xMin << ".." << config.world.xMax << " m (X), "
                  << config.world.yMin << ".." << config.world.yMax << " m (Y), "
                  << config.world.zMin << ".." << config.world.zMax << " m (Z)\n";
        std::cout << "[System] Fleet Size: " << config.fleet.numUavs << " UAVs\n";
        std::cout << "[System] Timesteps: Physics=" << config.simulation.dtPhysics
                  << "s, Policy=" << config.simulation.dtPolicy
                  << "s, Coordinator=" << config.simulation.dtCoordinator << "s\n";
    } catch (const std::exception& ex) {
        std::cerr << "[Error] Configuration initialization failed: " << ex.what() << "\n";
        return 1;
    }

    if (headless) {
        std::cout << "[System] Running in headless mode. Configuration verified successfully.\n";
        return 0;
    }

    // Try starting visualizer with raylib
    const int screenWidth = 1000;
    const int screenHeight = 700;
    SetTraceLogLevel(LOG_WARNING);
    InitWindow(screenWidth, screenHeight, "AeroFleet - Urban UAV Fleet Simulation");

    if (!IsWindowReady()) {
        std::cout << "[System] Headless environment detected (no graphical display available). Exiting cleanly.\n";
        return 0;
    }

    SetTargetFPS(60);

    while (!WindowShouldClose()) {
        BeginDrawing();
        ClearBackground(Color{24, 28, 36, 255}); // Dark slate background

        DrawText("AeroFleet Simulation Engine", 30, 30, 24, RAYWHITE);
        DrawText(("Simulation: " + config.simulation.name).c_str(), 30, 70, 18, LIGHTGRAY);
        DrawText(("Active Fleet: " + std::to_string(config.fleet.numUavs) + " UAVs").c_str(), 30, 100, 18, GREEN);
        DrawText(("Physics Clock: " + std::to_string(config.simulation.dtPhysics) + "s (50 Hz)").c_str(), 30, 130, 18, SKYBLUE);
        DrawText(("World Area: [" + std::to_string(static_cast<int>(config.world.xMax - config.world.xMin)) + "m x " +
                  std::to_string(static_cast<int>(config.world.yMax - config.world.yMin)) + "m]").c_str(), 30, 160, 18, ORANGE);

        DrawRectangle(30, 210, screenWidth - 60, 3, DARKGRAY);

        DrawText("Status: Phase I Step 1 - Build & Configuration Pipeline Active", 30, 230, 16, LIME);
        DrawText("Press ESC or close window to exit.", 30, screenHeight - 40, 14, GRAY);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}

