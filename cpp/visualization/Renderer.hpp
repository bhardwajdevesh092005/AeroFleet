#pragma once

#include <string>
#include <vector>
#include <deque>
#include "core/WorldState.hpp"
#include "world/CityMap.hpp"
#include "visualization/CameraController.hpp"
#include "visualization/VisualTheme.hpp"
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

struct RippleParticle {
    Vector3 pos{0, 0, 0};
    float radius{1.0f};
    float maxRadius{16.0f};
    float alpha{1.0f};
    Color color{46, 204, 113, 255};
};

struct HudToast {
    std::string message;
    Color color{56, 189, 248, 255};
    float remainingDuration{3.5f};
};

struct CachedHudData {
    std::string timeStr;
    std::string drainStr;
    std::string statsStr;
    std::string fpsStr;
    std::string telemetryHeader;
    std::string telemetryMode;
    std::string telemetrySoc;
    std::string telemetrySpeed;
    std::string telemetryAlt;
    std::string telemetryPayload;
    std::string telemetryMission;
    std::string telemetryPos;
    float lastUpdateTime{0.0f};
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

    // Quality Settings (LOW, MEDIUM, HIGH)
    void cycleQuality() noexcept;
    void setQuality(QualitySetting q) noexcept { m_quality = q; }
    [[nodiscard]] QualitySetting getQuality() const noexcept { return m_quality; }

    // Day / Night Theme
    void toggleTheme() noexcept;
    void setThemeMode(VisualThemeMode mode) noexcept;
    [[nodiscard]] VisualThemeMode getThemeMode() const noexcept { return m_themeMode; }
    [[nodiscard]] const ThemeColors& getThemeColors() const noexcept { return m_currentTheme; }

    void setBatteryDrainMultiplier(double mult) noexcept { m_batteryDrainMultiplier = mult; }
    [[nodiscard]] double getBatteryDrainMultiplier() const noexcept { return m_batteryDrainMultiplier; }
    void setScenarioName(const std::string& name) { m_scenarioName = name; }
    [[nodiscard]] const std::string& getScenarioName() const noexcept { return m_scenarioName; }

    void addToast(const std::string& msg, Color col = Color{56, 189, 248, 255}, float duration = 3.5f);
    void captureScreenshot(const std::string& filename = "aerofleet_screenshot.png");

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
    double m_batteryDrainMultiplier{15.0};
    std::string m_scenarioName{"All Scenarios Showcase"};

    QualitySetting m_quality{QualitySetting::HIGH};
    VisualThemeMode m_themeMode{VisualThemeMode::NIGHT};
    ThemeColors m_currentTheme{VisualTheme::getNightTheme()};
    bool m_showControlsHelp{true};

    Font m_fontRegular{};
    Font m_fontBold{};
    bool m_fontsLoaded{false};

    // Static GPU Merged City Models
    Model m_cityModelDay{};
    Model m_cityModelNight{};
    bool m_cityModelsLoaded{false};
    void buildCityModels(const CityMap& map);
    void unloadCityModels();

    // Vegetation and horizon
    bool m_cityFeaturesGenerated{false};
    std::vector<Vector3> m_treePositions;
    std::vector<int> m_treeTypes;
    std::vector<Vector3> m_parkPositions;

    // UAV Hero Visuals & Animation
    float m_rotorAngle{0.0f};
    std::vector<Vector3> m_trailHistory[32];
    float m_lastTrailSampleTime{0.0f};

    // Telemetry & Sparkline History
    std::deque<float> m_batteryHistory[32];
    float m_lastSparklineSampleTime{0.0f};

    // Dynamic Events & Particles
    std::vector<RippleParticle> m_ripples;
    std::vector<HudToast> m_toasts;
    size_t m_lastDeliveredCount{0};

    // 10 Hz HUD Cache
    CachedHudData m_hudCache;

    void updateAnimations(float dt, const WorldSnapshot& snapshot);
    void updateHudCache(const WorldSnapshot& snapshot, const CameraController& camera);

    void drawTextSans(const char* text, float x, float y, float fontSize, Color color, bool bold = false);
    void drawSkyAndAtmosphere(const CameraController& camera);
    void draw3DScene(const WorldSnapshot& snapshot, const CityMap& map, const CameraController& camera);
    void drawExtendedHorizonTerrain(const CityMap& map);
    void drawCityGroundAndGrid(const CityMap& map);
    void drawRoadNetwork(const CityMap& map, const CameraController& camera);
    void drawGreeneryAndParks(const CityMap& map, const CameraController& camera);
    void drawTree(Vector3 pos, float scale = 1.0f, int treeType = 0, bool simplified = false);
    void drawBuildings(const CityMap& map, const CameraController& camera);
    void drawLandmarks(const CityMap& map);
    void drawHeroUAV(const UAVState& uav, bool isSelected, float curTime, float dt, const CameraController& camera);
    void drawUAVs(const WorldSnapshot& snapshot, const CameraController& camera);
    void drawTrajectories(const WorldSnapshot& snapshot);
    void drawParticleRipples();

    // 2D HUD Layout & Overlays
    void drawHUD(const WorldSnapshot& snapshot, const CityMap& map, const CameraController& camera);
    void drawTopStatusBar(const WorldSnapshot& snapshot, const CameraController& camera);
    void drawDroneTelemetryPanel(const WorldSnapshot& snapshot);
    void drawFleetBatteryOverview(const WorldSnapshot& snapshot, CameraController& camera);
    void drawMinimap(const WorldSnapshot& snapshot, const CityMap& map, const CameraController& camera);
    void drawToasts();
    void drawKeycapChips();
    void drawDroneScreenBillboard(const UAVState& uav, const CameraController& camera);
};

} // namespace aerofleet


