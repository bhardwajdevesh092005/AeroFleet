#pragma once

#include <raylib.h>
#include <string>
#include "core/Types.hpp"

namespace aerofleet {

enum class QualitySetting : uint8_t {
    LOW = 0,
    MEDIUM = 1,
    HIGH = 2
};

inline const char* qualitySettingToString(QualitySetting q) noexcept {
    switch (q) {
        case QualitySetting::LOW: return "LOW";
        case QualitySetting::MEDIUM: return "MEDIUM";
        case QualitySetting::HIGH: return "HIGH";
        default: return "MEDIUM";
    }
}

enum class VisualThemeMode : uint8_t {
    DAY = 0,
    NIGHT = 1
};

inline const char* visualThemeModeToString(VisualThemeMode mode) noexcept {
    switch (mode) {
        case VisualThemeMode::DAY: return "DAY";
        case VisualThemeMode::NIGHT: return "NIGHT";
        default: return "NIGHT";
    }
}

struct DronePalette {
    Color delivering{46, 204, 113, 255};      // Vibrant Emerald Green
    Color toPickup{52, 152, 219, 255};        // Radiant Azure Blue
    Color charging{243, 156, 18, 255};        // Bright Golden Amber
    Color lowBatteryRTH{230, 126, 34, 255};   // Safety Orange
    Color emergency{231, 76, 60, 255};        // Danger Crimson Red (pulsing)
    Color heavyCargo{155, 89, 182, 255};      // Regal Royal Purple
    Color idle{149, 165, 166, 255};           // Clean Silver Slate
};

struct ThemeColors {
    // Sky gradient
    Color skyZenith{15, 23, 42, 255};         // Top sky color
    Color skyHorizon{30, 41, 59, 255};        // Horizon sky color
    Color fogColor{25, 33, 48, 255};          // Distance fog color

    // Ground and terrain
    Color terrainBase{20, 24, 33, 255};       // Outskirt terrain
    Color urbanGround{32, 38, 48, 255};       // Central city ground
    Color asphalt{24, 26, 31, 255};           // Road corridors
    Color sidewalk{50, 56, 68, 255};          // Concrete walkways
    Color laneYellow{250, 204, 21, 255};      // Road double yellow lines
    Color laneWhite{241, 245, 249, 255};       // Crosswalk and dashed lane dividers

    // Buildings
    Color buildingBase{42, 48, 60, 255};      // Neutral slate base
    Color buildingTop{65, 75, 92, 255};       // Height gradient top
    Color buildingRoof{28, 32, 40, 255};      // Dark rooftop gravel/tarmac
    Color buildingEdge{70, 85, 110, 180};     // Soft edge silhouette highlight
    Color windowUnlit{35, 42, 54, 255};       // Dark unlit glass
    Color windowLitWarm{254, 240, 138, 240};  // Emissive warm window light
    Color windowLitCool{186, 230, 253, 240};  // Emissive cyan-white office light

    // Greenery and water
    Color parkGrass{34, 90, 50, 255};         // Park lawn
    Color treeCanopy{40, 120, 60, 255};       // Trees
    Color fountainWater{56, 189, 248, 220};   // Fountains and ponds

    // Airspace fence
    Color geofenceBorder{56, 189, 248, 200};  // Glowing border wire
    Color geofenceWall{56, 189, 248, 12};     // Transparent boundary wash

    // UI and HUD
    Color hudGlassBg{15, 23, 42, 225};        // Semi-transparent glass background
    Color hudGlassBorder{71, 85, 105, 180};   // Glass card border
    Color hudHeaderBg{30, 41, 59, 240};       // Pill and chip background
    Color hudTextPrimary{248, 250, 252, 255}; // Crisp white text
    Color hudTextMuted{148, 163, 184, 255};   // Muted label text
    Color hudAccent{56, 189, 248, 255};       // Sky blue accent
};

class VisualTheme {
public:
    static ThemeColors getDayTheme() noexcept {
        ThemeColors c;
        c.skyZenith = Color{200, 225, 245, 255};
        c.skyHorizon = Color{235, 242, 250, 255};
        c.fogColor = Color{220, 232, 244, 255};

        c.terrainBase = Color{205, 212, 218, 255};
        c.urbanGround = Color{218, 224, 230, 255};
        c.asphalt = Color{48, 52, 60, 255};
        c.sidewalk = Color{225, 230, 235, 255};
        c.laneYellow = Color{250, 204, 21, 255};
        c.laneWhite = Color{255, 255, 255, 255};

        c.buildingBase = Color{195, 202, 212, 255};
        c.buildingTop = Color{230, 235, 242, 255};
        c.buildingRoof = Color{70, 75, 85, 255};
        c.buildingEdge = Color{90, 105, 125, 140};
        c.windowUnlit = Color{130, 150, 175, 255};
        c.windowLitWarm = Color{255, 245, 180, 255};
        c.windowLitCool = Color{210, 240, 255, 255};

        c.parkGrass = Color{65, 150, 85, 255};
        c.treeCanopy = Color{45, 130, 65, 255};
        c.fountainWater = Color{56, 189, 248, 220};

        c.geofenceBorder = Color{14, 165, 233, 220};
        c.geofenceWall = Color{14, 165, 233, 14};

        c.hudGlassBg = Color{15, 23, 42, 230};
        c.hudGlassBorder = Color{71, 85, 105, 180};
        c.hudHeaderBg = Color{30, 41, 59, 240};
        c.hudTextPrimary = Color{248, 250, 252, 255};
        c.hudTextMuted = Color{148, 163, 184, 255};
        c.hudAccent = Color{56, 189, 248, 255};
        return c;
    }

    static ThemeColors getNightTheme() noexcept {
        ThemeColors c;
        c.skyZenith = Color{8, 12, 24, 255};      // Midnight deep space navy
        c.skyHorizon = Color{20, 28, 45, 255};     // Atmospheric night twilight
        c.fogColor = Color{16, 22, 35, 255};

        c.terrainBase = Color{14, 18, 26, 255};
        c.urbanGround = Color{22, 27, 36, 255};
        c.asphalt = Color{20, 22, 28, 255};
        c.sidewalk = Color{38, 44, 56, 255};
        c.laneYellow = Color{250, 204, 21, 240};
        c.laneWhite = Color{226, 232, 240, 230};

        c.buildingBase = Color{32, 38, 48, 255};   // Deep graphite dark skyscraper
        c.buildingTop = Color{48, 56, 70, 255};    // Subtle upward ambient reflection
        c.buildingRoof = Color{22, 25, 32, 255};   // Industrial matte roof
        c.buildingEdge = Color{56, 189, 248, 80};  // Neon cyberpunk faint edge glint
        c.windowUnlit = Color{26, 32, 42, 255};    // Sleeping dark window
        c.windowLitWarm = Color{254, 240, 138, 250};// Glowing amber night window
        c.windowLitCool = Color{147, 197, 253, 250};// Glowing fluorescent tech window

        c.parkGrass = Color{24, 60, 36, 255};
        c.treeCanopy = Color{28, 80, 45, 255};
        c.fountainWater = Color{56, 189, 248, 240};

        c.geofenceBorder = Color{56, 189, 248, 240};
        c.geofenceWall = Color{56, 189, 248, 16};

        c.hudGlassBg = Color{10, 15, 28, 235};
        c.hudGlassBorder = Color{56, 189, 248, 90};
        c.hudHeaderBg = Color{20, 30, 48, 240};
        c.hudTextPrimary = Color{248, 250, 252, 255};
        c.hudTextMuted = Color{148, 163, 184, 255};
        c.hudAccent = Color{56, 189, 248, 255};
        return c;
    }
};

} // namespace aerofleet
