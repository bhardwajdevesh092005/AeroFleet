#pragma once

#include <string>
#include <vector>
#include "core/WorldState.hpp"
#include "world/CityMap.hpp"
#include "visualization/CameraController.hpp"
#include <raylib.h>
#if defined(PI)
#undef PI
#endif

namespace aerofleet {

struct RendererConfig {
    int screenWidth{1280};
    int screenHeight{800};
    std::string windowTitle{"AeroFleet - Autonomous Urban UAV Fleet Simulator"};
    int targetFPS{60};
    bool showHUD{true};
    bool showTrajectories{true};
    bool showShadowLines{true};
    bool showGrid{true};
    UAVId selectedUavId{0};
};

/**
 * @brief High-performance 3D visualization and HUD engine powered by raylib.
 * Reads immutable WorldSnapshot instances and renders the 3D urban environment,
 * UAV trajectories, landmarks, and 2D telemetry HUD without mutating simulator memory.
 */
class Renderer {
public:
    Renderer();
    explicit Renderer(const RendererConfig& config);
    ~Renderer();

    /**
     * @brief Initializes the raylib window and OpenGL context.
     * @return true if window initialized successfully, false if running in headless environment.
     */
    bool init();

    /**
     * @brief Closes the raylib window and releases graphics context.
     */
    void close();

    /**
     * @brief Checks if user requested window close (ESC key or window 'X').
     */
    [[nodiscard]] bool shouldClose() const noexcept;

    /**
     * @brief Returns true if graphical window is active and ready for drawing.
     */
    [[nodiscard]] bool isReady() const noexcept;

    /**
     * @brief Renders an immutable snapshot of the simulation world.
     * Guaranteed snapshot purity: read-only access to WorldSnapshot and CityMap.
     */
    void draw(const WorldSnapshot& snapshot, const CityMap& map, CameraController& camera);

    // Configuration and selection
    [[nodiscard]] const RendererConfig& getConfig() const noexcept { return m_config; }
    void setConfig(const RendererConfig& config) noexcept { m_config = config; }

    void setSelectedUav(UAVId id) noexcept { m_config.selectedUavId = id; }
    [[nodiscard]] UAVId getSelectedUav() const noexcept { return m_config.selectedUavId; }
    void cycleSelectedUav(size_t fleetSize) noexcept;

    void toggleHUD() noexcept { m_config.showHUD = !m_config.showHUD; }
    void toggleTrajectories() noexcept { m_config.showTrajectories = !m_config.showTrajectories; }
    void toggleShadowLines() noexcept { m_config.showShadowLines = !m_config.showShadowLines; }

    /**
     * @brief Helper returning a color gradient from Green (100%) -> Yellow (50%) -> Red (0%).
     */
    [[nodiscard]] static Color getBatteryColor(double percent) noexcept;

    /**
     * @brief Helper returning distinct color based on UAV operational lifecycle mode.
     */
    [[nodiscard]] static Color getModeColor(UAVMode mode) noexcept;

private:
    RendererConfig m_config;
    bool m_initialized{false};

    void draw3DScene(const WorldSnapshot& snapshot, const CityMap& map, const CameraController& camera);
    void drawCityGroundAndGrid(const CityMap& map);
    void drawBuildings(const CityMap& map);
    void drawLandmarks(const CityMap& map);
    void drawUAVs(const WorldSnapshot& snapshot);
    void drawTrajectories(const WorldSnapshot& snapshot);
    void drawHUD(const WorldSnapshot& snapshot, const CameraController& camera);
    void drawTopStatusBar(const WorldSnapshot& snapshot, const CameraController& camera);
    void drawDroneTelemetryPanel(const WorldSnapshot& snapshot);
    void drawControlsHelp();
};

} // namespace aerofleet

