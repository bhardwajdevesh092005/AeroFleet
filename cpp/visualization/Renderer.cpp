#include "visualization/Renderer.hpp"
#include <rlgl.h>
#include <iomanip>
#include <sstream>
#include <cstdlib>
#include <cmath>
#include <algorithm>
#include <cstring>

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

inline Color fadeColor(Color c, float alpha) noexcept {
    return Color{
        c.r,
        c.g,
        c.b,
        static_cast<unsigned char>(std::clamp(alpha * 255.0f, 0.0f, 255.0f))
    };
}

inline Color lerpColor(Color a, Color b, float t) noexcept {
    t = std::clamp(t, 0.0f, 1.0f);
    return Color{
        static_cast<unsigned char>(a.r + (b.r - a.r) * t),
        static_cast<unsigned char>(a.g + (b.g - a.g) * t),
        static_cast<unsigned char>(a.b + (b.b - a.b) * t),
        static_cast<unsigned char>(a.a + (b.a - a.a) * t)
    };
}

/**
 * @brief Lightweight mesh builder assembling unindexed static 3D city geometry with vertex colors.
 */
struct CityMeshBuilder {
    std::vector<float> vertices;
    std::vector<float> normals;
    std::vector<float> texcoords;
    std::vector<unsigned char> colors;

    void addVertex(float x, float y, float z, float nx, float ny, float nz, Color c, float u = 0.0f, float v = 0.0f) {
        vertices.push_back(x);
        vertices.push_back(y);
        vertices.push_back(z);
        normals.push_back(nx);
        normals.push_back(ny);
        normals.push_back(nz);
        texcoords.push_back(u);
        texcoords.push_back(v);
        colors.push_back(c.r);
        colors.push_back(c.g);
        colors.push_back(c.b);
        colors.push_back(c.a);
    }

    void addQuad(Vector3 v0, Vector3 v1, Vector3 v2, Vector3 v3, Vector3 norm,
                 Color c0, Color c1, Color c2, Color c3) {
        // Triangle 1: v0 -> v1 -> v2
        addVertex(v0.x, v0.y, v0.z, norm.x, norm.y, norm.z, c0, 0.0f, 0.0f);
        addVertex(v1.x, v1.y, v1.z, norm.x, norm.y, norm.z, c1, 1.0f, 0.0f);
        addVertex(v2.x, v2.y, v2.z, norm.x, norm.y, norm.z, c2, 1.0f, 1.0f);

        // Triangle 2: v0 -> v2 -> v3
        addVertex(v0.x, v0.y, v0.z, norm.x, norm.y, norm.z, c0, 0.0f, 0.0f);
        addVertex(v2.x, v2.y, v2.z, norm.x, norm.y, norm.z, c2, 1.0f, 1.0f);
        addVertex(v3.x, v3.y, v3.z, norm.x, norm.y, norm.z, c3, 0.0f, 1.0f);
    }

    void addQuad(Vector3 v0, Vector3 v1, Vector3 v2, Vector3 v3, Vector3 norm, Color color) {
        addQuad(v0, v1, v2, v3, norm, color, color, color, color);
    }

    void addBox(Vector3 center, float sx, float sy, float sz, Color colorBottom, Color colorTop) {
        const float hx = sx * 0.5f;
        const float hy = sy * 0.5f;
        const float hz = sz * 0.5f;
        const float y0 = center.y - hy;
        const float y1 = center.y + hy;

        Vector3 c000{center.x - hx, y0, center.z - hz};
        Vector3 c100{center.x + hx, y0, center.z - hz};
        Vector3 c101{center.x + hx, y0, center.z + hz};
        Vector3 c001{center.x - hx, y0, center.z + hz};

        Vector3 c010{center.x - hx, y1, center.z - hz};
        Vector3 c110{center.x + hx, y1, center.z - hz};
        Vector3 c111{center.x + hx, y1, center.z + hz};
        Vector3 c011{center.x - hx, y1, center.z + hz};

        // Front (Z+)
        addQuad(c001, c101, c111, c011, Vector3{0, 0, 1}, colorBottom, colorBottom, colorTop, colorTop);
        // Back (Z-)
        addQuad(c100, c000, c010, c110, Vector3{0, 0, -1}, colorBottom, colorBottom, colorTop, colorTop);
        // Left (X-)
        addQuad(c000, c001, c011, c010, Vector3{-1, 0, 0}, colorBottom, colorBottom, colorTop, colorTop);
        // Right (X+)
        addQuad(c101, c100, c110, c111, Vector3{1, 0, 0}, colorBottom, colorBottom, colorTop, colorTop);
        // Top (Y+)
        addQuad(c010, c110, c111, c011, Vector3{0, 1, 0}, colorTop, colorTop, colorTop, colorTop);
        // Bottom (Y-)
        addQuad(c000, c001, c101, c100, Vector3{0, -1, 0}, colorBottom, colorBottom, colorBottom, colorBottom);
    }

    void addBox(Vector3 center, float sx, float sy, float sz, Color color) {
        addBox(center, sx, sy, sz, color, color);
    }

    Mesh buildMesh() {
        Mesh mesh{};
        mesh.vertexCount = static_cast<int>(vertices.size() / 3);
        mesh.triangleCount = mesh.vertexCount / 3;

        mesh.vertices = static_cast<float*>(MemAlloc(vertices.size() * sizeof(float)));
        std::memcpy(mesh.vertices, vertices.data(), vertices.size() * sizeof(float));

        mesh.normals = static_cast<float*>(MemAlloc(normals.size() * sizeof(float)));
        std::memcpy(mesh.normals, normals.data(), normals.size() * sizeof(float));

        mesh.texcoords = static_cast<float*>(MemAlloc(texcoords.size() * sizeof(float)));
        std::memcpy(mesh.texcoords, texcoords.data(), texcoords.size() * sizeof(float));

        mesh.colors = static_cast<unsigned char*>(MemAlloc(colors.size() * sizeof(unsigned char)));
        std::memcpy(mesh.colors, colors.data(), colors.size() * sizeof(unsigned char));

        mesh.indices = nullptr;
        return mesh;
    }
};

} // namespace

Renderer::Renderer() : m_config(RendererConfig{}) {}

Renderer::Renderer(const RendererConfig& config) : m_config(config) {}

Renderer::~Renderer() {
    close();
}

bool Renderer::init() {
    if (m_initialized) return true;

#if defined(__linux__)
    const char* display = std::getenv("DISPLAY");
    const char* wayland = std::getenv("WAYLAND_DISPLAY");
    const bool hasDisplay = (display != nullptr && display[0] != '\0');
    const bool hasWayland = (wayland != nullptr && wayland[0] != '\0');
    if (!hasDisplay && !hasWayland) {
        m_initialized = false;
        return false;
    }
#endif

    SetTraceLogLevel(LOG_WARNING);
    InitWindow(m_config.screenWidth, m_config.screenHeight, m_config.windowTitle.c_str());

    if (!IsWindowReady()) {
        m_initialized = false;
        return false;
    }

    SetTargetFPS(m_config.targetFPS);

    // Load standard clean Sans font
    const char* regularPaths[] = {
        "assets/fonts/DejaVuSans.ttf",
        "../assets/fonts/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
    };
    for (const char* path : regularPaths) {
        if (FileExists(path)) {
            m_fontRegular = LoadFontEx(path, 32, nullptr, 0);
            if (m_fontRegular.texture.id > 0) {
                SetTextureFilter(m_fontRegular.texture, TEXTURE_FILTER_BILINEAR);
                m_fontsLoaded = true;
                break;
            }
        }
    }

    const char* boldPaths[] = {
        "assets/fonts/DejaVuSans-Bold.ttf",
        "../assets/fonts/DejaVuSans-Bold.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
    };
    for (const char* path : boldPaths) {
        if (FileExists(path)) {
            m_fontBold = LoadFontEx(path, 36, nullptr, 0);
            if (m_fontBold.texture.id > 0) {
                SetTextureFilter(m_fontBold.texture, TEXTURE_FILTER_BILINEAR);
                break;
            }
        }
    }

    // Default showcase theme is NIGHT for glorious glowing visuals
    m_themeMode = VisualThemeMode::NIGHT;
    m_currentTheme = VisualTheme::getNightTheme();

    addToast("AeroFleet v1.0 Initialized", Color{56, 189, 248, 255}, 4.0f);
    m_initialized = true;
    return true;
}

void Renderer::unloadCityModels() {
    if (m_cityModelsLoaded) {
        if (m_cityModelDay.meshCount > 0) {
            UnloadModel(m_cityModelDay);
            m_cityModelDay = Model{};
        }
        if (m_cityModelNight.meshCount > 0) {
            UnloadModel(m_cityModelNight);
            m_cityModelNight = Model{};
        }
        m_cityModelsLoaded = false;
    }
}

void Renderer::close() {
    if (m_initialized) {
        unloadCityModels();

        if (m_fontRegular.texture.id > 0) {
            UnloadFont(m_fontRegular);
            m_fontRegular = Font{};
        }
        if (m_fontBold.texture.id > 0) {
            UnloadFont(m_fontBold);
            m_fontBold = Font{};
        }
        m_fontsLoaded = false;
        m_cityFeaturesGenerated = false;
        m_treePositions.clear();
        m_treeTypes.clear();
        m_parkPositions.clear();

        if (IsWindowReady()) {
            CloseWindow();
        }
        m_initialized = false;
    }
}

void Renderer::cycleQuality() noexcept {
    if (m_quality == QualitySetting::LOW) {
        m_quality = QualitySetting::MEDIUM;
        addToast("Graphics Quality: MEDIUM", Color{241, 196, 15, 255});
    } else if (m_quality == QualitySetting::MEDIUM) {
        m_quality = QualitySetting::HIGH;
        addToast("Graphics Quality: HIGH (Full Shading & Trails)", Color{46, 204, 113, 255});
    } else {
        m_quality = QualitySetting::LOW;
        addToast("Graphics Quality: LOW (High-FPS Mode)", Color{148, 163, 184, 255});
    }
}

void Renderer::toggleTheme() noexcept {
    if (m_themeMode == VisualThemeMode::NIGHT) {
        setThemeMode(VisualThemeMode::DAY);
    } else {
        setThemeMode(VisualThemeMode::NIGHT);
    }
}

void Renderer::setThemeMode(VisualThemeMode mode) noexcept {
    m_themeMode = mode;
    if (mode == VisualThemeMode::DAY) {
        m_currentTheme = VisualTheme::getDayTheme();
        addToast("Theme: Clean Daylight", Color{245, 205, 35, 255});
    } else {
        m_currentTheme = VisualTheme::getNightTheme();
        addToast("Theme: Cyberpunk Midnight", Color{56, 189, 248, 255});
    }
}

void Renderer::addToast(const std::string& msg, Color col, float duration) {
    m_toasts.push_back(HudToast{msg, col, duration});
    if (m_toasts.size() > 4) {
        m_toasts.erase(m_toasts.begin());
    }
}

void Renderer::captureScreenshot(const std::string& filename) {
    TakeScreenshot(filename.c_str());
    addToast(std::string("Screenshot saved: ") + filename, Color{46, 204, 113, 255}, 4.0f);
}

void Renderer::drawTextSans(const char* text, float x, float y, float fontSize, Color color, bool bold) {
    if (!text || text[0] == '\0') return;
    Font fontToUse = (bold && m_fontBold.texture.id > 0) ? m_fontBold 
                   : (m_fontRegular.texture.id > 0 ? m_fontRegular : GetFontDefault());
    DrawTextEx(fontToUse, text, Vector2{x, y}, fontSize, 1.0f, color);
}

bool Renderer::shouldClose() const noexcept {
    if (!m_initialized || !IsWindowReady()) return true;
    return WindowShouldClose();
}

bool Renderer::isReady() const noexcept {
    return m_initialized && IsWindowReady();
}

Color Renderer::getBatteryColor(double percent) noexcept {
    if (percent > 50.0) {
        float t = static_cast<float>((percent - 50.0) / 50.0);
        return lerpColor(Color{243, 156, 18, 255}, Color{46, 204, 113, 255}, t);
    } else {
        float t = static_cast<float>(percent / 50.0);
        return lerpColor(Color{231, 76, 60, 255}, Color{243, 156, 18, 255}, t);
    }
}

Color Renderer::getModeColor(UAVMode mode) noexcept {
    switch (mode) {
        case UAVMode::IDLE:
            return Color{149, 165, 166, 255}; // Silver Slate
        case UAVMode::ASSIGNED:
        case UAVMode::FLYING_TO_PICKUP:
        case UAVMode::PICKING_UP:
            return Color{52, 152, 219, 255};  // Azure Blue (TO PICKUP)
        case UAVMode::FLYING_TO_DESTINATION:
            return Color{46, 204, 113, 255};  // Emerald Green (DELIVERING)
        case UAVMode::DROPPING_OFF:
            return Color{155, 89, 182, 255};  // Royal Purple (DROPPING)
        case UAVMode::RETURNING_TO_CHARGE:
            return Color{230, 126, 34, 255};  // Safety Orange (LOW BATT RTH)
        case UAVMode::CHARGING:
            return Color{243, 156, 18, 255};  // Golden Amber (CHARGING)
        case UAVMode::EMERGENCY:
            return Color{231, 76, 60, 255};   // Danger Red (EMERGENCY)
        case UAVMode::OFFLINE:
        default:
            return Color{100, 110, 125, 255};
    }
}

void Renderer::cycleSelectedUav(size_t fleetSize) noexcept {
    if (fleetSize == 0) return;
    m_config.selectedUavId = static_cast<UAVId>((m_config.selectedUavId + 1) % fleetSize);
}

void Renderer::buildCityModels(const CityMap& map) {
    if (!m_initialized || !IsWindowReady()) return;
    unloadCityModels();

    const float minX = static_cast<float>(map.worldMin.x);
    const float minZ = static_cast<float>(map.worldMin.y);
    const float maxX = static_cast<float>(map.worldMax.x);
    const float maxZ = static_cast<float>(map.worldMax.y);
    const float width = maxX - minX;
    const float depth = maxZ - minZ;
    const float centerX = (minX + maxX) * 0.5f;
    const float centerZ = (minZ + maxZ) * 0.5f;

    const int numAvenues = 15;
    const float stepX = width / static_cast<float>(numAvenues - 1);
    const float stepZ = depth / static_cast<float>(numAvenues - 1);
    const float roadWidth = 22.0f;
    const float sidewalkWidth = 3.5f;

    auto buildThemeMesh = [&](const ThemeColors& tc, bool isNight) -> Mesh {
        CityMeshBuilder b;

        // 1. Central Urban Ground Base Plane
        b.addBox(Vector3{centerX, -0.05f, centerZ}, width + 200.0f, 0.1f, depth + 200.0f, tc.urbanGround);

        // 2. Asphalt Avenues & Sidewalk Network
        for (int i = 0; i < numAvenues; ++i) {
            float coordX = minX + static_cast<float>(i) * stepX;
            float coordZ = minZ + static_cast<float>(i) * stepZ;

            // North-South Avenue
            b.addBox(Vector3{coordX, 0.04f, centerZ}, roadWidth, 0.08f, depth, tc.asphalt);
            b.addBox(Vector3{coordX - (roadWidth * 0.5f + sidewalkWidth * 0.5f), 0.06f, centerZ}, sidewalkWidth, 0.12f, depth, tc.sidewalk);
            b.addBox(Vector3{coordX + (roadWidth * 0.5f + sidewalkWidth * 0.5f), 0.06f, centerZ}, sidewalkWidth, 0.12f, depth, tc.sidewalk);

            // East-West Avenue
            b.addBox(Vector3{centerX, 0.045f, coordZ}, width, 0.08f, roadWidth, tc.asphalt);
            b.addBox(Vector3{centerX, 0.065f, coordZ - (roadWidth * 0.5f + sidewalkWidth * 0.5f)}, width, 0.12f, sidewalkWidth, tc.sidewalk);
            b.addBox(Vector3{centerX, 0.065f, coordZ + (roadWidth * 0.5f + sidewalkWidth * 0.5f)}, width, 0.12f, sidewalkWidth, tc.sidewalk);

            // Double Yellow Centerline Strips
            b.addBox(Vector3{coordX - 0.35f, 0.09f, centerZ}, 0.22f, 0.02f, depth, tc.laneYellow);
            b.addBox(Vector3{coordX + 0.35f, 0.09f, centerZ}, 0.22f, 0.02f, depth, tc.laneYellow);
            b.addBox(Vector3{centerX, 0.095f, coordZ - 0.35f}, width, 0.02f, 0.22f, tc.laneYellow);
            b.addBox(Vector3{centerX, 0.095f, coordZ + 0.35f}, width, 0.02f, 0.22f, tc.laneYellow);
        }

        // 3. Intersection Zebra Crosswalks
        for (int i = 0; i < numAvenues; ++i) {
            float ix = minX + static_cast<float>(i) * stepX;
            for (int j = 0; j < numAvenues; ++j) {
                float iz = minZ + static_cast<float>(j) * stepZ;
                for (float stripeX = ix - 7.0f; stripeX <= ix + 7.0f; stripeX += 2.2f) {
                    b.addBox(Vector3{stripeX, 0.095f, iz - 13.0f}, 1.1f, 0.01f, 3.0f, tc.laneWhite);
                    b.addBox(Vector3{stripeX, 0.095f, iz + 13.0f}, 1.1f, 0.01f, 3.0f, tc.laneWhite);
                }
                for (float stripeZ = iz - 7.0f; stripeZ <= iz + 7.0f; stripeZ += 2.2f) {
                    b.addBox(Vector3{ix - 13.0f, 0.095f, stripeZ}, 3.0f, 0.01f, 1.1f, tc.laneWhite);
                    b.addBox(Vector3{ix + 13.0f, 0.095f, stripeZ}, 3.0f, 0.01f, 1.1f, tc.laneWhite);
                }
            }
        }

        // 4. Municipal Parks and Water Basins
        for (const auto& park : m_parkPositions) {
            b.addBox(Vector3{park.x, 0.05f, park.z}, 90.0f, 0.1f, 90.0f, tc.parkGrass);
            b.addBox(Vector3{park.x, 0.08f, park.z}, 85.0f, 0.05f, 8.0f, tc.sidewalk);
            b.addBox(Vector3{park.x, 0.08f, park.z}, 8.0f, 0.05f, 85.0f, tc.sidewalk);
            b.addBox(Vector3{park.x, 0.12f, park.z}, 24.0f, 0.15f, 24.0f, tc.fountainWater);
        }

        // 5. Buildings (All 191 Skyscraper Towers)
        for (size_t idx = 0; idx < map.buildings.size(); ++idx) {
            const auto& bld = map.buildings[idx];
            const auto& minP = bld.bounds.minPoint;
            const auto& maxP = bld.bounds.maxPoint;

            const float w = static_cast<float>(maxP.x - minP.x);
            const float h = static_cast<float>(maxP.z - minP.z);
            const float d = static_cast<float>(maxP.y - minP.y);

            const Vector3 bCenter{
                static_cast<float>((minP.x + maxP.x) * 0.5),
                static_cast<float>(h * 0.5f),
                static_cast<float>((minP.y + maxP.y) * 0.5)
            };

            // Architectural Tint variation
            const int archStyle = static_cast<int>((idx * 13 + static_cast<size_t>(std::abs(minP.x))) % 3);
            Color baseC = tc.buildingBase;
            Color topC = tc.buildingTop;
            if (archStyle == 1) {
                // Cool Steel tint
                baseC = Color{static_cast<unsigned char>(baseC.r * 0.9f), static_cast<unsigned char>(baseC.g * 0.95f), static_cast<unsigned char>(std::min(255.0f, baseC.b * 1.1f)), 255};
                topC = Color{static_cast<unsigned char>(topC.r * 0.95f), static_cast<unsigned char>(topC.g * 1.0f), static_cast<unsigned char>(std::min(255.0f, topC.b * 1.15f)), 255};
            } else if (archStyle == 2) {
                // Warm Sandstone tint
                baseC = Color{static_cast<unsigned char>(std::min(255.0f, baseC.r * 1.08f)), static_cast<unsigned char>(std::min(255.0f, baseC.g * 1.04f)), static_cast<unsigned char>(baseC.b * 0.95f), 255};
                topC = Color{static_cast<unsigned char>(std::min(255.0f, topC.r * 1.1f)), static_cast<unsigned char>(std::min(255.0f, topC.g * 1.06f)), static_cast<unsigned char>(topC.b * 0.98f), 255};
            }

            const bool hasSetback = (h >= 80.0f);
            const float mainH = hasSetback ? (h * 0.76f) : h;
            const float setbackH = hasSetback ? (h * 0.24f) : 0.0f;
            const float setW = w * 0.74f;
            const float setD = d * 0.74f;

            // Main Shaft with Height Gradient
            Vector3 mainCenter{bCenter.x, mainH * 0.5f, bCenter.z};
            b.addBox(mainCenter, w, mainH, d, baseC, topC);

            // Windows & Floor Ribbon Patterns
            const int numBands = (mainH >= 110.0f) ? 5 : (mainH >= 60.0f ? 4 : 3);
            for (int band = 1; band <= numBands; ++band) {
                const float bandY = mainH * (static_cast<float>(band) / static_cast<float>(numBands + 1));
                const float bandH = 2.4f;

                if (isNight) {
                    // Night Window Rows: Small lit windows scattered across facades
                    const int winCountX = std::max(2, static_cast<int>(w / 4.5f));
                    const int winCountZ = std::max(2, static_cast<int>(d / 4.5f));

                    // Facade Z+ (Front) & Z- (Back)
                    for (int wx = 0; wx < winCountX; ++wx) {
                        float fx = bCenter.x - w * 0.45f + (static_cast<float>(wx) + 0.5f) * (w * 0.9f / winCountX);
                        int hashFront = static_cast<int>(idx * 7919 + band * 31 + wx * 17) % 100;
                        Color winCol = (hashFront < 20) ? tc.windowLitWarm : (hashFront < 30 ? tc.windowLitCool : tc.windowUnlit);
                        b.addBox(Vector3{fx, bandY, bCenter.z + d * 0.505f}, 2.0f, bandH * 0.8f, 0.05f, winCol);

                        int hashBack = static_cast<int>(idx * 5449 + band * 43 + wx * 19) % 100;
                        Color winBack = (hashBack < 20) ? tc.windowLitWarm : (hashBack < 30 ? tc.windowLitCool : tc.windowUnlit);
                        b.addBox(Vector3{fx, bandY, bCenter.z - d * 0.505f}, 2.0f, bandH * 0.8f, 0.05f, winBack);
                    }
                    // Facade X+ (Right) & X- (Left)
                    for (int wz = 0; wz < winCountZ; ++wz) {
                        float fz = bCenter.z - d * 0.45f + (static_cast<float>(wz) + 0.5f) * (d * 0.9f / winCountZ);
                        int hashRight = static_cast<int>(idx * 3187 + band * 61 + wz * 13) % 100;
                        Color winCol = (hashRight < 20) ? tc.windowLitWarm : (hashRight < 30 ? tc.windowLitCool : tc.windowUnlit);
                        b.addBox(Vector3{bCenter.x + w * 0.505f, bandY, fz}, 0.05f, bandH * 0.8f, 2.0f, winCol);

                        int hashLeft = static_cast<int>(idx * 9473 + band * 73 + wz * 23) % 100;
                        Color winLeft = (hashLeft < 20) ? tc.windowLitWarm : (hashLeft < 30 ? tc.windowLitCool : tc.windowUnlit);
                        b.addBox(Vector3{bCenter.x - w * 0.505f, bandY, fz}, 0.05f, bandH * 0.8f, 2.0f, winLeft);
                    }
                } else {
                    // Day Theme: Clean tinted glass ribbon bands
                    b.addBox(Vector3{bCenter.x, bandY, bCenter.z}, w + 0.15f, bandH, d + 0.15f, tc.windowUnlit);
                }
            }

            // Stepped Setback Upper Tower
            if (hasSetback) {
                Vector3 setPos{bCenter.x, mainH + setbackH * 0.5f, bCenter.z};
                b.addBox(setPos, setW, setbackH, setD, baseC, topC);
                b.addBox(Vector3{bCenter.x, mainH + setbackH * 0.5f, bCenter.z}, setW + 0.15f, 2.2f, setD + 0.15f, tc.windowUnlit);
            }

            // Rooftop Terrace & Mechanical Penthouse
            const float topY = h;
            const float topW = hasSetback ? setW : w;
            const float topD = hasSetback ? setD : d;

            b.addBox(Vector3{bCenter.x, topY + 0.15f, bCenter.z}, topW, 0.30f, topD, tc.buildingRoof);

            // Mechanical Penthouse
            const float pentW = topW * 0.38f;
            const float pentD = topD * 0.38f;
            const float pentH = 4.2f;
            Vector3 pentPos{bCenter.x, topY + 0.30f + pentH * 0.5f, bCenter.z};
            b.addBox(pentPos, pentW, pentH, pentD, isNight ? Color{45, 52, 65, 255} : Color{195, 202, 210, 255});

            // Rooftop Helipads on High-Rises (h >= 70m)
            if (h >= 70.0f) {
                const float padRadius = std::min(11.0f, std::min(topW, topD) * 0.40f);
                b.addBox(Vector3{bCenter.x, topY + 0.35f, bCenter.z}, padRadius * 2.0f, 0.04f, padRadius * 2.0f, Color{45, 50, 58, 255});
                b.addBox(Vector3{bCenter.x, topY + 0.38f, bCenter.z}, padRadius * 1.8f, 0.04f, padRadius * 1.8f, tc.laneYellow);
                b.addBox(Vector3{bCenter.x, topY + 0.40f, bCenter.z}, padRadius * 1.6f, 0.04f, padRadius * 1.6f, Color{45, 50, 58, 255});

                // Painted White 'H' Helipad Marker
                const float hArm = padRadius * 0.70f;
                b.addBox(Vector3{bCenter.x - padRadius * 0.32f, topY + 0.44f, bCenter.z}, 1.2f, 0.06f, hArm, WHITE);
                b.addBox(Vector3{bCenter.x + padRadius * 0.32f, topY + 0.44f, bCenter.z}, 1.2f, 0.06f, hArm, WHITE);
                b.addBox(Vector3{bCenter.x, topY + 0.44f, bCenter.z}, padRadius * 0.64f, 0.06f, 1.2f, WHITE);
            }

            // Antenna Spire atop Penthouse
            if (h >= 70.0f) {
                const float spireH = 12.0f;
                b.addBox(Vector3{bCenter.x, topY + pentH + spireH * 0.5f + 0.3f, bCenter.z}, 0.5f, spireH, 0.5f, Color{120, 130, 145, 255});
            }

            // Ground Floor Entrance Canopy
            b.addBox(Vector3{bCenter.x, 2.2f, bCenter.z + d * 0.5f + 1.2f}, w * 0.45f, 0.3f, 2.5f, Color{45, 55, 70, 255});
        }

        return b.buildMesh();
    };

    Mesh dayMesh = buildThemeMesh(VisualTheme::getDayTheme(), false);
    UploadMesh(&dayMesh, false);
    m_cityModelDay = LoadModelFromMesh(dayMesh);

    Mesh nightMesh = buildThemeMesh(VisualTheme::getNightTheme(), true);
    UploadMesh(&nightMesh, false);
    m_cityModelNight = LoadModelFromMesh(nightMesh);

    m_cityModelsLoaded = true;
}

void Renderer::updateAnimations(float dt, const WorldSnapshot& snapshot) {
    const float curTime = static_cast<float>(GetTime());

    // 1. Spinning Rotors Animation
    m_rotorAngle += dt * 42.0f;
    if (m_rotorAngle > 360.0f) m_rotorAngle -= 360.0f;

    // 2. Sample UAV Motion Trails every ~0.10s (last 30 samples = 3.0s history)
    if (curTime - m_lastTrailSampleTime >= 0.10f) {
        m_lastTrailSampleTime = curTime;
        for (const auto& u : snapshot.uavs) {
            if (u.id < 32) {
                Vector3 pos = toRaylib(u.position);
                m_trailHistory[u.id].push_back(pos);
                if (m_trailHistory[u.id].size() > 30) {
                    m_trailHistory[u.id].erase(m_trailHistory[u.id].begin());
                }
            }
        }
    }

    // 3. Sample Battery Sparkline History every ~1.0s (last 50 samples = 50s telemetry curve)
    if (curTime - m_lastSparklineSampleTime >= 1.0f) {
        m_lastSparklineSampleTime = curTime;
        for (const auto& u : snapshot.uavs) {
            if (u.id < 32) {
                m_batteryHistory[u.id].push_back(static_cast<float>(u.batteryPercent));
                if (m_batteryHistory[u.id].size() > 50) {
                    m_batteryHistory[u.id].pop_front();
                }
            }
        }
    }

    // 4. Delivery Completed Particle Ripples & Toast trigger
    if (snapshot.metrics.totalDelivered > m_lastDeliveredCount) {
        m_lastDeliveredCount = snapshot.metrics.totalDelivered;

        // Spawn delivery completed ripple at selected UAV or center
        const auto* selUav = snapshot.findUAV(m_config.selectedUavId);
        Vector3 dropPos = selUav ? toRaylib(selUav->position) : Vector3{0, 0, 0};
        dropPos.y = 0.5f;

        m_ripples.push_back(RippleParticle{dropPos, 1.0f, 22.0f, 1.0f, Color{46, 204, 113, 255}});
        std::string toastMsg = "+1 Package Delivered! (Total: " + std::to_string(m_lastDeliveredCount) + ")";
        addToast(toastMsg, Color{46, 204, 113, 255}, 4.0f);
    }

    // 5. Update Particle Ripples
    for (auto it = m_ripples.begin(); it != m_ripples.end();) {
        it->radius += dt * 14.0f;
        it->alpha -= dt * 0.9f;
        if (it->alpha <= 0.0f || it->radius >= it->maxRadius) {
            it = m_ripples.erase(it);
        } else {
            ++it;
        }
    }

    // 6. Update HUD Toasts
    for (auto it = m_toasts.begin(); it != m_toasts.end();) {
        it->remainingDuration -= dt;
        if (it->remainingDuration <= 0.0f) {
            it = m_toasts.erase(it);
        } else {
            ++it;
        }
    }
}

void Renderer::updateHudCache(const WorldSnapshot& snapshot, const CameraController& camera) {
    (void)camera;
    const float curTime = static_cast<float>(GetTime());
    if (curTime - m_hudCache.lastUpdateTime < 0.10f) return; // 10 Hz throttle
    m_hudCache.lastUpdateTime = curTime;

    std::ostringstream timeOss;
    timeOss << "Time: " << std::fixed << std::setprecision(1) << snapshot.simulationTime << "s";
    m_hudCache.timeStr = timeOss.str();

    std::ostringstream drainOss;
    drainOss << "⚡ Drain: " << std::fixed << std::setprecision(0) << m_batteryDrainMultiplier << "x";
    m_hudCache.drainStr = drainOss.str();

    m_hudCache.statsStr = "Del: " + std::to_string(snapshot.metrics.totalDelivered) +
                          " | Act: " + std::to_string(snapshot.metrics.totalActive) +
                          " | Coll: " + std::to_string(snapshot.metrics.collisionCount);

    m_hudCache.fpsStr = "FPS: " + std::to_string(GetFPS());

    const auto* uav = snapshot.findUAV(m_config.selectedUavId);
    if (uav) {
        m_hudCache.telemetryHeader = "UAV #" + std::to_string(uav->id) + " TELEMETRY";
        m_hudCache.telemetryMode = uavModeToString(uav->mode);

        std::ostringstream socOss;
        socOss << std::fixed << std::setprecision(1) << uav->batteryPercent << "%";
        m_hudCache.telemetrySoc = socOss.str();

        std::ostringstream speedOss;
        speedOss << "Speed: " << std::fixed << std::setprecision(1) << uav->speed() << " m/s";
        m_hudCache.telemetrySpeed = speedOss.str();

        std::ostringstream altOss;
        altOss << "Altitude: " << std::fixed << std::setprecision(1) << uav->position.z << " m";
        m_hudCache.telemetryAlt = altOss.str();

        std::ostringstream payOss;
        payOss << "Payload: " << std::fixed << std::setprecision(2) << uav->payloadMass << " kg";
        m_hudCache.telemetryPayload = payOss.str();

        m_hudCache.telemetryMission = "Mission: " + (uav->activeRequestId != -1 ? ("#" + std::to_string(uav->activeRequestId)) : "STANDBY");

        std::ostringstream posOss;
        posOss << "Pos: (" << static_cast<int>(uav->position.x) << ", "
               << static_cast<int>(uav->position.y) << ", "
               << static_cast<int>(uav->position.z) << ")";
        m_hudCache.telemetryPos = posOss.str();
    }
}

void Renderer::draw(const WorldSnapshot& snapshot, const CityMap& map, CameraController& camera) {
    if (!m_initialized || !IsWindowReady()) return;

    const float dt = GetFrameTime();

    // Hotkey Controls
    if (IsKeyPressed(KEY_H)) toggleHUD();
    if (IsKeyPressed(KEY_T)) toggleTheme();
    if (IsKeyPressed(KEY_Q)) cycleQuality();
    if (IsKeyPressed(KEY_P)) captureScreenshot();
    if (IsKeyPressed(KEY_SLASH)) m_showControlsHelp = !m_showControlsHelp;
    if (IsKeyPressed(KEY_TAB)) {
        cycleSelectedUav(snapshot.uavs.size());
        camera.setChaseTarget(m_config.selectedUavId);
    }

    // Build GPU city models on first frame with map data
    if (!m_cityModelsLoaded && !map.buildings.empty()) {
        buildCityModels(map);
    }

    // Update animations, ripples, particle systems & 10 Hz HUD cache
    updateAnimations(dt, snapshot);
    updateHudCache(snapshot, camera);

    BeginDrawing();

    // Clear color and depth buffers completely for current frame
    ClearBackground(m_currentTheme.skyHorizon);

    // 1. Vertical Gradient Atmospheric Sky
    drawSkyAndAtmosphere(camera);

    // 2. 3D Scene Rendering
    draw3DScene(snapshot, map, camera);

    // 3. 2D Telemetry & HUD Overlay
    if (m_config.showHUD) {
        drawHUD(snapshot, map, camera);
    }

    EndDrawing();
}

void Renderer::drawSkyAndAtmosphere(const CameraController& camera) {
    (void)camera;
    // Vertical 2D gradient sky dome (Zenith to Horizon) across full window dimensions
    const int screenW = GetScreenWidth();
    const int screenH = GetScreenHeight();
    DrawRectangleGradientV(0, 0, screenW, screenH,
                           m_currentTheme.skyZenith, m_currentTheme.skyHorizon);
}

void Renderer::draw3DScene(const WorldSnapshot& snapshot, const CityMap& map, const CameraController& camera) {
    Camera3D rayCam = camera.getRaylibCamera();

    // Camera clipping plane: 1.0m to 25,000m ensures zero horizon clipping
    rlSetClipPlanes(1.0, 25000.0);
    BeginMode3D(rayCam);

    // Disable backface culling to ensure geometry is never clipped regardless of viewing angle
    rlDisableBackfaceCulling();

    // 1. Regional terrain basin & outer ring expressway
    drawExtendedHorizonTerrain(map);

    // 2. High-Performance Static Merged City Model (1 Draw Call!)
    if (m_cityModelsLoaded && m_cityModelDay.meshCount > 0 && m_cityModelNight.meshCount > 0) {
        Model activeModel = (m_themeMode == VisualThemeMode::DAY) ? m_cityModelDay : m_cityModelNight;
        DrawModel(activeModel, Vector3{0, 0, 0}, 1.0f, WHITE);
    } else {
        // Fallback immediate mode if model not yet uploaded
        drawCityGroundAndGrid(map);
        drawBuildings(map, camera);
    }

    // 3. Municipal Parks, trees & boulevards (Distance LOD)
    if (m_quality != QualitySetting::LOW) {
        drawGreeneryAndParks(map, camera);
    }

    // 4. Landmark Beacons (Stores, Customers, Charging Stations)
    drawLandmarks(map);

    // 5. Flight Path Trajectories & Routes
    if (m_config.showTrajectories) {
        drawTrajectories(snapshot);
    }

    // 6. Delivery Particle Ripples
    drawParticleRipples();

    // 7. Active Hero UAV Fleet
    drawUAVs(snapshot, camera);

    EndMode3D();
}

void Renderer::drawExtendedHorizonTerrain(const CityMap& map) {
    const float centerX = static_cast<float>((map.worldMin.x + map.worldMax.x) * 0.5);
    const float centerZ = static_cast<float>((map.worldMin.y + map.worldMax.y) * 0.5);

    // Grand Regional Basin Terrain Plane (24,000m x 24,000m)
    DrawPlane(Vector3{centerX, -0.30f, centerZ}, Vector2{24000.0f, 24000.0f}, m_currentTheme.terrainBase);

    // Outer Expressway Ring Corridor
    const float ringRadius = 1500.0f;
    DrawCube(Vector3{centerX, -0.05f, centerZ - ringRadius}, ringRadius * 2.0f, 0.1f, 28.0f, m_currentTheme.asphalt);
    DrawCube(Vector3{centerX, -0.05f, centerZ + ringRadius}, ringRadius * 2.0f, 0.1f, 28.0f, m_currentTheme.asphalt);
    DrawCube(Vector3{centerX - ringRadius, -0.05f, centerZ}, 28.0f, 0.1f, ringRadius * 2.0f, m_currentTheme.asphalt);
    DrawCube(Vector3{centerX + ringRadius, -0.05f, centerZ}, 28.0f, 0.1f, ringRadius * 2.0f, m_currentTheme.asphalt);
}

void Renderer::drawCityGroundAndGrid(const CityMap& map) {
    const float minX = static_cast<float>(map.worldMin.x);
    const float minZ = static_cast<float>(map.worldMin.y);
    const float maxX = static_cast<float>(map.worldMax.x);
    const float maxZ = static_cast<float>(map.worldMax.y);
    const float width = maxX - minX;
    const float depth = maxZ - minZ;
    const float centerX = (minX + maxX) * 0.5f;
    const float centerZ = (minZ + maxZ) * 0.5f;

    DrawPlane(Vector3{centerX, 0.0f, centerZ}, Vector2{width, depth}, m_currentTheme.urbanGround);

    // Glowing Geofence Perimeter Wire
    Vector3 c1{minX, 0.1f, minZ};
    Vector3 c2{maxX, 0.1f, minZ};
    Vector3 c3{maxX, 0.1f, maxZ};
    Vector3 c4{minX, 0.1f, maxZ};
    DrawLine3D(c1, c2, m_currentTheme.geofenceBorder);
    DrawLine3D(c2, c3, m_currentTheme.geofenceBorder);
    DrawLine3D(c3, c4, m_currentTheme.geofenceBorder);
    DrawLine3D(c4, c1, m_currentTheme.geofenceBorder);
}

void Renderer::drawTree(Vector3 pos, float scale, int treeType, bool simplified) {
    (void)treeType;
    if (simplified) {
        DrawCylinder(Vector3{pos.x, 3.5f * scale, pos.z}, 0.1f, 1.6f * scale, 7.0f * scale, 5, m_currentTheme.treeCanopy);
        return;
    }
    const float trunkR = 0.35f * scale;
    const float trunkH = 3.8f * scale;
    DrawCylinder(pos, trunkR, trunkR * 1.2f, trunkH, 6, Color{78, 50, 30, 255});

    const Vector3 canopyPos{pos.x, pos.y + trunkH * 0.9f, pos.z};
    DrawSphere(canopyPos, 2.3f * scale, m_currentTheme.treeCanopy);
}

void Renderer::drawGreeneryAndParks(const CityMap& map, const CameraController& camera) {
    const Vector3 camPos = toRaylib(camera.getPosition());

    if (!m_cityFeaturesGenerated) {
        m_parkPositions.push_back(Vector3{0.0f, 0.0f, 0.0f});
        m_parkPositions.push_back(Vector3{-280.0f, 0.0f, 280.0f});
        m_parkPositions.push_back(Vector3{280.0f, 0.0f, -280.0f});

        const float minX = static_cast<float>(map.worldMin.x);
        const float maxX = static_cast<float>(map.worldMax.x);
        const float minZ = static_cast<float>(map.worldMin.y);
        const float maxZ = static_cast<float>(map.worldMax.y);
        const float width = maxX - minX;
        const int numAvenues = 15;
        const float step = width / static_cast<float>(numAvenues - 1);

        for (int i = 0; i < numAvenues; ++i) {
            float ax = minX + static_cast<float>(i) * step;
            for (float z = minZ + 40.0f; z <= maxZ - 40.0f; z += 55.0f) {
                for (float side : {-14.0f, 14.0f}) {
                    float tx = ax + side;
                    float tz = z;
                    Vec3 treeEnu(tx, tz, 4.0);
                    if (!map.isLocationClear(treeEnu, 5.0)) continue;
                    m_treePositions.push_back(Vector3{tx, 0.0f, tz});
                    m_treeTypes.push_back(static_cast<int>(std::abs(tx * 3.0f + tz * 7.0f)) % 3);
                }
            }
        }
        m_cityFeaturesGenerated = true;
    }

    // Render Boulevard Avenue Trees with Distance LOD
    for (size_t i = 0; i < m_treePositions.size(); ++i) {
        float d2 = (camPos.x - m_treePositions[i].x) * (camPos.x - m_treePositions[i].x) +
                   (camPos.z - m_treePositions[i].z) * (camPos.z - m_treePositions[i].z);
        if (d2 > 2400.0f * 2400.0f) continue;
        bool simplified = (d2 > 950.0f * 950.0f || m_quality == QualitySetting::LOW);
        int tType = (i < m_treeTypes.size()) ? m_treeTypes[i] : 0;
        drawTree(m_treePositions[i], 1.0f, tType, simplified);
    }
}

void Renderer::drawBuildings(const CityMap& map, const CameraController& camera) {
    (void)camera;
    for (const auto& b : map.buildings) {
        const auto& minP = b.bounds.minPoint;
        const auto& maxP = b.bounds.maxPoint;
        const float w = static_cast<float>(maxP.x - minP.x);
        const float h = static_cast<float>(maxP.z - minP.z);
        const float d = static_cast<float>(maxP.y - minP.y);
        const Vector3 center{
            static_cast<float>((minP.x + maxP.x) * 0.5),
            static_cast<float>(h * 0.5f),
            static_cast<float>((minP.y + maxP.y) * 0.5)
        };
        DrawCube(center, w, h, d, m_currentTheme.buildingBase);
    }
}

void Renderer::drawLandmarks(const CityMap& map) {
    const float curTime = static_cast<float>(GetTime());

    // 1. Store Pickup Hubs: Pulsing Emerald Ground Ring + Skyward Light Column
    for (const auto& store : map.stores) {
        Vector3 pos = toRaylib(store.position);
        pos.y = 0.0f;

        DrawCylinder(pos, 5.5f, 5.5f, 1.2f, 16, Color{46, 204, 113, 220});
        DrawCylinderWires(pos, 5.5f, 5.5f, 1.2f, 16, Color{39, 174, 96, 255});

        // Pulsing holographic ring
        float ringRadius = 5.0f + 2.5f * std::sin(curTime * 3.0f);
        DrawCircle3D(Vector3{pos.x, 0.4f, pos.z}, ringRadius, Vector3{1.0f, 0.0f, 0.0f}, 90.0f, Color{46, 204, 113, 180});

        // Skyward Beacon Light Beam Column
        if (m_quality != QualitySetting::LOW) {
            DrawCylinder(Vector3{pos.x, 30.0f, pos.z}, 0.5f, 0.5f, 60.0f, 8, Color{46, 204, 113, 110});
        }
    }

    // 2. Charging Stations: Power Rings & Docking Halos
    for (const auto& station : map.chargingStations) {
        Vector3 pos = toRaylib(station.position);
        pos.y = 0.0f;

        DrawCylinder(pos, 6.0f, 6.0f, 1.2f, 20, Color{243, 156, 18, 220});
        DrawCylinderWires(pos, 6.0f, 6.0f, 1.2f, 20, Color{230, 126, 34, 255});

        // Docking status ring
        Color ringColor = station.hasAvailableSlot() ? Color{46, 204, 113, 240} : Color{231, 76, 60, 240};
        DrawCylinderWires(Vector3{pos.x, 1.3f, pos.z}, 6.2f, 6.2f, 0.2f, 20, ringColor);

        // Power energy core pillar
        if (m_quality != QualitySetting::LOW) {
            DrawCylinder(Vector3{pos.x, 12.0f, pos.z}, 0.4f, 0.4f, 24.0f, 8, Color{243, 156, 18, 120});
        }
    }

    // 3. Customer Delivery Sites: Pulsing Target Rings & Beacon Column
    for (const auto& cust : map.customers) {
        Vector3 pos = toRaylib(cust.position);
        pos.y = 0.0f;

        DrawCube(Vector3{pos.x, 1.0f, pos.z}, 3.0f, 2.0f, 3.0f, Color{155, 89, 182, 220});
        DrawCubeWires(Vector3{pos.x, 1.0f, pos.z}, 3.0f, 2.0f, 3.0f, Color{142, 68, 173, 255});

        // Pulsing delivery target circles
        float pulse = 3.0f + 1.8f * std::sin(curTime * 4.0f + static_cast<float>(cust.id));
        DrawCircle3D(Vector3{pos.x, 0.2f, pos.z}, pulse, Vector3{1.0f, 0.0f, 0.0f}, 90.0f, Color{52, 152, 219, 180});

        if (m_quality != QualitySetting::LOW) {
            DrawCylinder(Vector3{pos.x, 25.0f, pos.z}, 0.4f, 0.4f, 50.0f, 8, Color{155, 89, 182, 100});
        }
    }
}

void Renderer::drawParticleRipples() {
    for (const auto& r : m_ripples) {
        Color ringCol = fadeColor(r.color, r.alpha);
        DrawCircle3D(r.pos, r.radius, Vector3{1.0f, 0.0f, 0.0f}, 90.0f, ringCol);
        DrawCircle3D(r.pos, r.radius * 0.7f, Vector3{1.0f, 0.0f, 0.0f}, 90.0f, fadeColor(WHITE, r.alpha * 0.8f));
    }
}

void Renderer::drawTrajectories(const WorldSnapshot& snapshot) {
    for (const auto& uav : snapshot.uavs) {
        if (uav.mode == UAVMode::IDLE || uav.mode == UAVMode::OFFLINE) continue;

        Vector3 start = toRaylib(uav.position);
        Vector3 target = toRaylib(uav.targetWaypoint);
        Color trajColor = getModeColor(uav.mode);
        const bool isSelected = (uav.id == m_config.selectedUavId);

        // Dashed Planned Route to destination waypoint
        if (isSelected) {
            DrawLine3D(start, target, trajColor);
            DrawSphere(target, 1.2f, fadeColor(trajColor, 0.7f));
            DrawSphereWires(target, 1.2f, 8, 8, WHITE);
        } else if (m_quality != QualitySetting::LOW) {
            DrawLine3D(start, target, fadeColor(trajColor, 0.40f));
            DrawSphere(target, 0.6f, fadeColor(trajColor, 0.35f));
        }
    }
}

void Renderer::drawHeroUAV(const UAVState& uav, bool isSelected, float curTime, float dt, const CameraController& camera) {
    (void)dt;
    (void)camera;
    Vector3 pos = toRaylib(uav.position);
    Color modeCol = getModeColor(uav.mode);

    // Dynamic Hover Bobbing
    const float bob = std::sin(curTime * 4.0f + uav.id * 1.3f) * 0.35f;
    pos.y += bob;

    // 1. Altitude Ground Projection Line & Circular Ground Shadow
    if (m_config.showShadowLines && pos.y > 0.1f) {
        Vector3 groundPt{pos.x, 0.0f, pos.z};
        DrawLine3D(pos, groundPt, isSelected ? Color{250, 204, 21, 160} : Color{70, 85, 105, 120});

        float shadowRadius = std::max(1.2f, 3.5f - (pos.y * 0.03f));
        DrawCircle3D(groundPt, shadowRadius, Vector3{1.0f, 0.0f, 0.0f}, 90.0f, Color{0, 0, 0, 140});
    }

    // 2. Fading Motion Ribbon Trail (last 30 samples)
    if (m_quality != QualitySetting::LOW && uav.id < 32) {
        const auto& trail = m_trailHistory[uav.id];
        if (trail.size() >= 2) {
            for (size_t s = 0; s < trail.size() - 1; ++s) {
                float alphaFrac = static_cast<float>(s) / static_cast<float>(trail.size());
                Color segCol = fadeColor(modeCol, alphaFrac * 0.75f);
                DrawLine3D(trail[s], trail[s + 1], segCol);
            }
        }
    }

    // 3. Quadcopter 3.5x Scaled Model
    const float armLength = 3.2f;
    const float rotorRadius = 1.35f;
    const float yaw = static_cast<float>(uav.yaw);

    // Emissive Underside Light / Halo
    float haloRadius = 3.5f;
    Color haloColor = fadeColor(modeCol, 0.45f);
    if (uav.mode == UAVMode::EMERGENCY) {
        float pulse = 0.5f + 0.5f * std::sin(curTime * 10.0f);
        haloColor = fadeColor(RED, pulse * 0.85f);
    }
    DrawCircle3D(Vector3{pos.x, pos.y - 0.2f, pos.z}, haloRadius, Vector3{1.0f, 0.0f, 0.0f}, 90.0f, haloColor);

    // 4 Carbon-Fiber Motor Arms & Spinning Rotor Discs
    for (int armIdx = 0; armIdx < 4; ++armIdx) {
        const float angle = yaw + armIdx * (3.14159265f / 2.0f) + (3.14159265f / 4.0f);
        const Vector3 armEnd{
            pos.x + std::cos(angle) * armLength,
            pos.y,
            pos.z + std::sin(angle) * armLength
        };

        // Carbon structural arm
        DrawLine3D(pos, armEnd, Color{45, 50, 60, 255});

        // Motor pod hub
        DrawSphere(armEnd, 0.35f, Color{30, 35, 42, 255});

        // Spinning Propeller Rotor Disc
        DrawCircle3D(armEnd, rotorRadius, Vector3{0.0f, 1.0f, 0.0f}, 0.0f, fadeColor(modeCol, 0.30f));

        // High-Speed Spinning Carbon Blades
        const float bAngle = m_rotorAngle * (armIdx % 2 == 0 ? 1.0f : -1.0f);
        const Vector3 blade1{
            armEnd.x + std::cos(bAngle) * rotorRadius,
            armEnd.y,
            armEnd.z + std::sin(bAngle) * rotorRadius
        };
        const Vector3 blade2{
            armEnd.x - std::cos(bAngle) * rotorRadius,
            armEnd.y,
            armEnd.z - std::sin(bAngle) * rotorRadius
        };
        DrawLine3D(blade1, blade2, Color{220, 225, 235, 200});
    }

    // Central Aerodynamic Fuselage Body (Scaled ~3.5x)
    const float bodyRadius = 1.6f;
    DrawSphere(pos, bodyRadius, modeCol);
    DrawSphereWires(pos, bodyRadius, 8, 8, isSelected ? YELLOW : fadeColor(WHITE, 0.6f));

    // Top Electronics Dome & Canopy
    DrawSphere(Vector3{pos.x, pos.y + 0.6f, pos.z}, bodyRadius * 0.65f, Color{25, 30, 38, 255});

    // Forward Directional Heading Nose Beacon
    const float headingLength = 4.2f;
    const Vector3 headingEnd{
        pos.x + std::cos(yaw) * headingLength,
        pos.y,
        pos.z + std::sin(yaw) * headingLength
    };
    DrawLine3D(pos, headingEnd, WHITE);
    DrawSphere(headingEnd, 0.45f, Color{56, 189, 248, 255});

    // 4. Selected Drone Reticle
    if (isSelected) {
        float rotReticle = curTime * 2.0f;
        DrawCircle3D(pos, 4.8f, Vector3{0.0f, 1.0f, 0.0f}, 0.0f, Color{245, 205, 35, 220});
        for (int a = 0; a < 4; ++a) {
            float ang = rotReticle + static_cast<float>(a) * (3.14159265f * 0.5f);
            Vector3 retP{pos.x + std::cos(ang) * 5.2f, pos.y, pos.z + std::sin(ang) * 5.2f};
            DrawSphere(retP, 0.35f, Color{245, 205, 35, 255});
        }
    }

    // 5. Cargo Container Box Payload
    if (uav.hasPayload()) {
        Vector3 cargoPos{pos.x, pos.y - 2.2f, pos.z};
        Color crateColor = (uav.payloadMass >= 2.0) ? Color{155, 89, 182, 255} : Color{243, 156, 18, 255};
        DrawCube(cargoPos, 1.6f, 1.4f, 1.6f, crateColor);
        DrawCubeWires(cargoPos, 1.6f, 1.4f, 1.6f, WHITE);

        // 4 Suspension rigging cables
        DrawLine3D(pos, Vector3{cargoPos.x - 0.7f, cargoPos.y + 0.7f, cargoPos.z - 0.7f}, Color{200, 210, 220, 255});
        DrawLine3D(pos, Vector3{cargoPos.x + 0.7f, cargoPos.y + 0.7f, cargoPos.z - 0.7f}, Color{200, 210, 220, 255});
        DrawLine3D(pos, Vector3{cargoPos.x + 0.7f, cargoPos.y + 0.7f, cargoPos.z + 0.7f}, Color{200, 210, 220, 255});
        DrawLine3D(pos, Vector3{cargoPos.x - 0.7f, cargoPos.y + 0.7f, cargoPos.z + 0.7f}, Color{200, 210, 220, 255});
    }

    // Docking laser tether beam
    if (uav.mode == UAVMode::PICKING_UP || uav.mode == UAVMode::DROPPING_OFF) {
        Vector3 groundTarget = toRaylib(uav.targetWaypoint);
        groundTarget.y = 0.0f;
        DrawLine3D(pos, groundTarget, Color{46, 204, 113, 220});
    } else if (uav.mode == UAVMode::CHARGING) {
        Vector3 dockFloor{pos.x, 0.0f, pos.z};
        DrawLine3D(pos, dockFloor, Color{243, 156, 18, 230});
    }
}

void Renderer::drawUAVs(const WorldSnapshot& snapshot, const CameraController& camera) {
    const float curTime = static_cast<float>(GetTime());
    const float dt = GetFrameTime();

    for (const auto& uav : snapshot.uavs) {
        bool isSelected = (uav.id == m_config.selectedUavId);
        drawHeroUAV(uav, isSelected, curTime, dt, camera);
    }
}

void Renderer::drawDroneScreenBillboard(const UAVState& uav, const CameraController& camera) {
    Vector3 pos = toRaylib(uav.position);
    pos.y += 3.8f; // Billboard above drone

    const Vector3 camPos = toRaylib(camera.getPosition());
    float dist = std::sqrt((camPos.x - pos.x) * (camPos.x - pos.x) +
                           (camPos.y - pos.y) * (camPos.y - pos.y) +
                           (camPos.z - pos.z) * (camPos.z - pos.z));
    if (dist > 1800.0f) return; // Far distance culling

    const int screenW = GetScreenWidth();
    const int screenH = GetScreenHeight();
    Vector2 screenPos = GetWorldToScreen(pos, camera.getRaylibCamera());
    if (screenPos.x < 10 || screenPos.x > screenW - 10 ||
        screenPos.y < 50 || screenPos.y > screenH - 30) {
        return;
    }

    float alpha = std::clamp(1.0f - (dist - 900.0f) / 900.0f, 0.2f, 1.0f);
    Color badgeBg = fadeColor(Color{15, 23, 42, 230}, alpha);
    Color textCol = fadeColor(WHITE, alpha);
    Color batCol = fadeColor(getBatteryColor(uav.batteryPercent), alpha);

    std::string tag = "#" + std::to_string(uav.id) + " · " + std::to_string(static_cast<int>(uav.batteryPercent)) + "%";
    const int tagW = 74;
    const int tagH = 20;

    DrawRectangleRounded(Rectangle{screenPos.x - tagW * 0.5f, screenPos.y - tagH * 0.5f, static_cast<float>(tagW), static_cast<float>(tagH)}, 0.4f, 4, badgeBg);
    DrawRectangleRoundedLines(Rectangle{screenPos.x - tagW * 0.5f, screenPos.y - tagH * 0.5f, static_cast<float>(tagW), static_cast<float>(tagH)}, 0.4f, 4, fadeColor(Color{56, 189, 248, 180}, alpha));

    // Mini battery strip indicator
    DrawRectangle(static_cast<int>(screenPos.x - tagW * 0.5f + 4), static_cast<int>(screenPos.y + tagH * 0.5f - 3), static_cast<int>((tagW - 8) * (uav.batteryPercent / 100.0)), 2, batCol);

    drawTextSans(tag.c_str(), screenPos.x - tagW * 0.5f + 6, screenPos.y - tagH * 0.5f + 3, 11, textCol, true);
}

void Renderer::drawHUD(const WorldSnapshot& snapshot, const CityMap& map, const CameraController& camera) {
    drawTopStatusBar(snapshot, camera);
    drawDroneTelemetryPanel(snapshot);
    drawFleetBatteryOverview(snapshot, const_cast<CameraController&>(camera));
    drawMinimap(snapshot, map, camera);
    drawToasts();

    // Floating 2D Screen-space labels above UAVs
    if (m_quality != QualitySetting::LOW) {
        for (const auto& u : snapshot.uavs) {
            drawDroneScreenBillboard(u, camera);
        }
    }

    if (m_showControlsHelp) {
        drawKeycapChips();
    }
}

void Renderer::drawTopStatusBar(const WorldSnapshot& snapshot, const CameraController& camera) {
    (void)camera;
    const int screenW = GetScreenWidth();
    const int barH = 46;
    DrawRectangle(0, 0, screenW, barH, Color{10, 15, 26, 240});
    DrawLine(0, barH, screenW, barH, Color{56, 189, 248, 80});

    // Zone 1: Branding Logo & Version
    drawTextSans("AEROFLEET", 20, 12, 22, Color{56, 189, 248, 255}, true);
    drawTextSans("v1.0", 152, 16, 12, Color{148, 163, 184, 255}, false);

    // Zone 2: Scenario Pill Badge
    if (!m_scenarioName.empty()) {
        std::string scenStr = m_scenarioName;
        DrawRectangleRounded(Rectangle{205, 10, 180, 26}, 0.5f, 4, Color{30, 41, 59, 240});
        DrawRectangleRoundedLines(Rectangle{205, 10, 180, 26}, 0.5f, 4, Color{56, 189, 248, 140});
        drawTextSans(scenStr.c_str(), 218, 16, 12, Color{56, 189, 248, 255}, true);
    }

    // Zone 3: KPI Metrics Chips (Fixed-width layout, zero collision!)
    int chipX = 400;
    auto drawKpiChip = [&](const std::string& label, Color c, int w = 95) {
        DrawRectangleRounded(Rectangle{static_cast<float>(chipX), 10.0f, static_cast<float>(w), 26.0f}, 0.4f, 4, Color{20, 28, 42, 230});
        DrawRectangleRoundedLines(Rectangle{static_cast<float>(chipX), 10.0f, static_cast<float>(w), 26.0f}, 0.4f, 4, Color{71, 85, 105, 140});
        drawTextSans(label.c_str(), chipX + 8, 16, 12, c, true);
        chipX += w + 8;
    };

    drawKpiChip(m_hudCache.timeStr, WHITE, 90);
    drawKpiChip("Del: " + std::to_string(snapshot.metrics.totalDelivered), Color{46, 204, 113, 255}, 80);
    drawKpiChip("Act: " + std::to_string(snapshot.metrics.totalActive), Color{52, 152, 219, 255}, 75);
    drawKpiChip("Coll: " + std::to_string(snapshot.metrics.collisionCount), snapshot.metrics.collisionCount > 0 ? RED : Color{148, 163, 184, 255}, 80);
    drawKpiChip(m_hudCache.drainStr, Color{245, 205, 35, 255}, 90);

    // Zone 4: Right Controls Status (Theme, Quality, FPS)
    int rightX = screenW - 260;
    std::string themeTag = (m_themeMode == VisualThemeMode::NIGHT) ? "🌙 NIGHT" : "☀️ DAY";
    DrawRectangleRounded(Rectangle{static_cast<float>(rightX), 10, 80, 26}, 0.4f, 4, Color{30, 41, 59, 230});
    drawTextSans(themeTag.c_str(), rightX + 8, 16, 11, Color{245, 205, 35, 255}, true);

    std::string qTag = qualitySettingToString(m_quality);
    DrawRectangleRounded(Rectangle{static_cast<float>(rightX + 88), 10, 72, 26}, 0.4f, 4, Color{30, 41, 59, 230});
    drawTextSans(qTag.c_str(), rightX + 96, 16, 11, Color{46, 204, 113, 255}, true);

    drawTextSans(m_hudCache.fpsStr.c_str(), rightX + 172, 16, 13, Color{56, 189, 248, 255}, true);
}

void Renderer::drawDroneTelemetryPanel(const WorldSnapshot& snapshot) {
    const auto* uav = snapshot.findUAV(m_config.selectedUavId);
    if (!uav) return;

    const int panelX = 20;
    const int panelY = 60;
    const int panelW = 290;
    const int panelH = 260;

    // Semi-transparent rounded glass card
    DrawRectangleRounded(Rectangle{static_cast<float>(panelX), static_cast<float>(panelY), static_cast<float>(panelW), static_cast<float>(panelH)}, 0.08f, 6, Color{10, 16, 30, 230});
    DrawRectangleRoundedLines(Rectangle{static_cast<float>(panelX), static_cast<float>(panelY), static_cast<float>(panelW), static_cast<float>(panelH)}, 0.08f, 6, Color{56, 189, 248, 120});

    // Header
    drawTextSans(m_hudCache.telemetryHeader.c_str(), panelX + 14, panelY + 12, 16, WHITE, true);

    // State Pill Badge
    Color modeCol = getModeColor(uav->mode);
    DrawRectangleRounded(Rectangle{static_cast<float>(panelX + 14), static_cast<float>(panelY + 36), 140, 20}, 0.5f, 4, fadeColor(modeCol, 0.25f));
    DrawRectangleRoundedLines(Rectangle{static_cast<float>(panelX + 14), static_cast<float>(panelY + 36), 140, 20}, 0.5f, 4, modeCol);
    drawTextSans(m_hudCache.telemetryMode.c_str(), panelX + 22, panelY + 39, 12, modeCol, true);

    // Large Gradient Battery Bar
    drawTextSans("BATTERY SOC:", panelX + 14, panelY + 66, 12, Color{148, 163, 184, 255}, false);
    drawTextSans(m_hudCache.telemetrySoc.c_str(), panelX + 220, panelY + 64, 14, getBatteryColor(uav->batteryPercent), true);

    const int barX = panelX + 14;
    const int barY = panelY + 84;
    const int barW = panelW - 28;
    const int barH = 14;
    DrawRectangle(barX, barY, barW, barH, Color{30, 41, 59, 255});
    const int fillW = static_cast<int>(barW * MathUtils::clamp(uav->batteryPercent / 100.0, 0.0, 1.0));
    DrawRectangle(barX, barY, fillW, barH, getBatteryColor(uav->batteryPercent));

    // Battery Sparkline Curve (Historical telemetry)
    if (uav->id < 32 && m_batteryHistory[uav->id].size() >= 2) {
        const auto& hist = m_batteryHistory[uav->id];
        const int sparkX = panelX + 14;
        const int sparkY = panelY + 104;
        const int sparkW = panelW - 28;
        const int sparkH = 26;

        DrawRectangle(sparkX, sparkY, sparkW, sparkH, Color{15, 23, 42, 200});
        for (size_t i = 0; i < hist.size() - 1; ++i) {
            float x1 = sparkX + (static_cast<float>(i) / hist.size()) * sparkW;
            float y1 = sparkY + sparkH - (hist[i] / 100.0f) * sparkH;
            float x2 = sparkX + (static_cast<float>(i + 1) / hist.size()) * sparkW;
            float y2 = sparkY + sparkH - (hist[i + 1] / 100.0f) * sparkH;
            DrawLine(static_cast<int>(x1), static_cast<int>(y1), static_cast<int>(x2), static_cast<int>(y2), Color{56, 189, 248, 220});
        }
    }

    // 3-Column Metric Grid
    const int gridY = panelY + 140;
    drawTextSans(m_hudCache.telemetrySpeed.c_str(), panelX + 14, gridY, 13, WHITE);
    drawTextSans(m_hudCache.telemetryAlt.c_str(), panelX + 150, gridY, 13, WHITE);

    drawTextSans(m_hudCache.telemetryPayload.c_str(), panelX + 14, gridY + 24, 13, uav->hasPayload() ? Color{245, 205, 35, 255} : Color{148, 163, 184, 255});
    drawTextSans(m_hudCache.telemetryMission.c_str(), panelX + 150, gridY + 24, 13, Color{56, 189, 248, 255});

    // Position Readout (High contrast)
    drawTextSans(m_hudCache.telemetryPos.c_str(), panelX + 14, gridY + 54, 12, Color{226, 232, 240, 255}, true);
}

void Renderer::drawFleetBatteryOverview(const WorldSnapshot& snapshot, CameraController& camera) {
    if (snapshot.uavs.empty()) return;

    const int screenW = GetScreenWidth();
    const int panelW = 260;
    const int itemH = 21;
    const int panelH = 36 + static_cast<int>(snapshot.uavs.size()) * itemH;
    const int panelX = screenW - panelW - 20;
    const int panelY = 60;

    DrawRectangleRounded(Rectangle{static_cast<float>(panelX), static_cast<float>(panelY), static_cast<float>(panelW), static_cast<float>(panelH)}, 0.08f, 6, Color{10, 16, 30, 230});
    DrawRectangleRoundedLines(Rectangle{static_cast<float>(panelX), static_cast<float>(panelY), static_cast<float>(panelW), static_cast<float>(panelH)}, 0.08f, 6, Color{56, 189, 248, 120});

    drawTextSans("FLEET BATTERY MONITOR", panelX + 14, panelY + 10, 14, Color{56, 189, 248, 255}, true);

    Vector2 mousePos = GetMousePosition();
    bool mouseClicked = IsMouseButtonPressed(MOUSE_BUTTON_LEFT);

    for (size_t i = 0; i < snapshot.uavs.size(); ++i) {
        const auto& u = snapshot.uavs[i];
        const int itemY = panelY + 32 + static_cast<int>(i) * itemH;
        const bool isSel = (u.id == m_config.selectedUavId);

        // Clickable row bounds
        Rectangle rowRect{static_cast<float>(panelX + 4), static_cast<float>(itemY - 1), static_cast<float>(panelW - 8), static_cast<float>(itemH - 2)};
        bool isHovered = CheckCollisionPointRec(mousePos, rowRect);

        if (isHovered && mouseClicked) {
            m_config.selectedUavId = u.id;
            camera.setChaseTarget(u.id);
            camera.setMode(CameraMode::CHASE);
            addToast("Chasing UAV #" + std::to_string(u.id), Color{56, 189, 248, 255});
        }

        if (isSel) {
            DrawRectangleRounded(rowRect, 0.3f, 4, Color{245, 205, 35, 60});
            DrawRectangleRoundedLines(rowRect, 0.3f, 4, Color{245, 205, 35, 200});
        } else if (isHovered) {
            DrawRectangleRounded(rowRect, 0.3f, 4, Color{56, 189, 248, 35});
        }

        // State indicator dot
        Color modeCol = getModeColor(u.mode);
        DrawCircle(panelX + 14, itemY + 8, 4.0f, modeCol);

        // UAV Label
        std::string label = "#" + std::to_string(u.id);
        drawTextSans(label.c_str(), panelX + 24, itemY + 2, 12, isSel ? YELLOW : WHITE, isSel);

        // Battery bar
        const int bX = panelX + 54;
        const int bW = 55;
        const int bH = 8;
        DrawRectangle(bX, itemY + 5, bW, bH, Color{30, 41, 59, 255});
        const int fillW = static_cast<int>(bW * MathUtils::clamp(u.batteryPercent / 100.0, 0.0, 1.0));
        DrawRectangle(bX, itemY + 5, fillW, bH, getBatteryColor(u.batteryPercent));

        // Battery % text
        std::ostringstream ss;
        ss << static_cast<int>(u.batteryPercent) << "%";
        drawTextSans(ss.str().c_str(), panelX + 116, itemY + 2, 11, getBatteryColor(u.batteryPercent), true);

        // State Pill
        std::string tag = uavModeToString(u.mode);
        drawTextSans(tag.c_str(), panelX + 155, itemY + 2, 11, modeCol);
    }
}

void Renderer::drawMinimap(const WorldSnapshot& snapshot, const CityMap& map, const CameraController& camera) {
    const int screenW = GetScreenWidth();
    const int screenH = GetScreenHeight();
    const int mapSize = 190;
    const int mapX = screenW - mapSize - 20;
    const int mapY = screenH - mapSize - 44;

    DrawRectangleRounded(Rectangle{static_cast<float>(mapX), static_cast<float>(mapY), static_cast<float>(mapSize), static_cast<float>(mapSize)}, 0.08f, 6, Color{10, 16, 30, 235});
    DrawRectangleRoundedLines(Rectangle{static_cast<float>(mapX), static_cast<float>(mapY), static_cast<float>(mapSize), static_cast<float>(mapSize)}, 0.08f, 6, Color{56, 189, 248, 140});

    drawTextSans("RADAR MINIMAP", mapX + 10, mapY + 6, 11, Color{56, 189, 248, 255}, true);

    const float cityMinX = static_cast<float>(map.worldMin.x);
    const float cityMaxX = static_cast<float>(map.worldMax.x);
    const float cityMinY = static_cast<float>(map.worldMin.y);
    const float cityMaxY = static_cast<float>(map.worldMax.y);
    const float cityW = cityMaxX - cityMinX;
    const float cityH = cityMaxY - cityMinY;

    auto toRadar = [&](float wx, float wy) -> Vector2 {
        float normX = (wx - cityMinX) / cityW;
        float normY = (wy - cityMinY) / cityH;
        return Vector2{
            mapX + 14.0f + normX * (mapSize - 28.0f),
            mapY + mapSize - 14.0f - normY * (mapSize - 28.0f)
        };
    };

    // Concentric range rings
    const Vector2 radarCenter{mapX + mapSize * 0.5f, mapY + mapSize * 0.5f};
    DrawCircleLines(static_cast<int>(radarCenter.x), static_cast<int>(radarCenter.y), 35.0f, Color{56, 189, 248, 40});
    DrawCircleLines(static_cast<int>(radarCenter.x), static_cast<int>(radarCenter.y), 70.0f, Color{56, 189, 248, 40});

    // Stores (Green circles)
    for (const auto& s : map.stores) {
        Vector2 pt = toRadar(static_cast<float>(s.position.x), static_cast<float>(s.position.y));
        DrawCircle(static_cast<int>(pt.x), static_cast<int>(pt.y), 3.0f, Color{46, 204, 113, 220});
    }

    // Chargers (Amber squares)
    for (const auto& c : map.chargingStations) {
        Vector2 pt = toRadar(static_cast<float>(c.position.x), static_cast<float>(c.position.y));
        DrawRectangle(static_cast<int>(pt.x - 2), static_cast<int>(pt.y - 2), 5, 5, Color{243, 156, 18, 220});
    }

    // Customer drop-offs (Purple dots)
    for (const auto& cust : map.customers) {
        Vector2 pt = toRadar(static_cast<float>(cust.position.x), static_cast<float>(cust.position.y));
        DrawCircle(static_cast<int>(pt.x), static_cast<int>(pt.y), 2.5f, Color{155, 89, 182, 200});
    }

    // Drones (Colored dots with heading vectors)
    for (const auto& u : snapshot.uavs) {
        Vector2 pt = toRadar(static_cast<float>(u.position.x), static_cast<float>(u.position.y));
        Color modeCol = getModeColor(u.mode);
        DrawCircle(static_cast<int>(pt.x), static_cast<int>(pt.y), 4.0f, modeCol);

        // Heading directional arrow
        float hx = pt.x + std::cos(u.yaw) * 7.0f;
        float hy = pt.y - std::sin(u.yaw) * 7.0f;
        DrawLine(static_cast<int>(pt.x), static_cast<int>(pt.y), static_cast<int>(hx), static_cast<int>(hy), WHITE);

        // Selected drone reticle
        if (u.id == m_config.selectedUavId) {
            DrawCircleLines(static_cast<int>(pt.x), static_cast<int>(pt.y), 7.0f, YELLOW);
        }
    }

    // Camera gaze position and frustum outline cone
    const Vec3& cPos = camera.getPosition();
    const Vec3& cTar = camera.getTarget();
    Vector2 camPt = toRadar(static_cast<float>(cPos.x), static_cast<float>(cPos.y));
    float gazeAngle = std::atan2(static_cast<float>(cTar.y - cPos.y), static_cast<float>(cTar.x - cPos.x));
    float fovHalf = 0.35f;
    float coneLen = 22.0f;
    Vector2 leftPt{camPt.x + std::cos(gazeAngle - fovHalf) * coneLen, camPt.y - std::sin(gazeAngle - fovHalf) * coneLen};
    Vector2 rightPt{camPt.x + std::cos(gazeAngle + fovHalf) * coneLen, camPt.y - std::sin(gazeAngle + fovHalf) * coneLen};

    DrawLine(static_cast<int>(camPt.x), static_cast<int>(camPt.y), static_cast<int>(leftPt.x), static_cast<int>(leftPt.y), Color{56, 189, 248, 160});
    DrawLine(static_cast<int>(camPt.x), static_cast<int>(camPt.y), static_cast<int>(rightPt.x), static_cast<int>(rightPt.y), Color{56, 189, 248, 160});
    DrawLine(static_cast<int>(leftPt.x), static_cast<int>(leftPt.y), static_cast<int>(rightPt.x), static_cast<int>(rightPt.y), Color{56, 189, 248, 120});
    DrawCircle(static_cast<int>(camPt.x), static_cast<int>(camPt.y), 3.0f, Color{56, 189, 248, 255});
}

void Renderer::drawToasts() {
    const int screenW = GetScreenWidth();
    int toastY = 56;
    for (const auto& t : m_toasts) {
        float alpha = std::clamp(t.remainingDuration / 0.8f, 0.0f, 1.0f);
        int toastW = 340;
        int toastH = 28;
        int toastX = (screenW - toastW) / 2;

        DrawRectangleRounded(Rectangle{static_cast<float>(toastX), static_cast<float>(toastY), static_cast<float>(toastW), static_cast<float>(toastH)}, 0.4f, 4, fadeColor(Color{15, 23, 42, 235}, alpha));
        DrawRectangleRoundedLines(Rectangle{static_cast<float>(toastX), static_cast<float>(toastY), static_cast<float>(toastW), static_cast<float>(toastH)}, 0.4f, 4, fadeColor(t.color, alpha));

        drawTextSans(t.message.c_str(), toastX + 14, toastY + 6, 13, fadeColor(t.color, alpha), true);
        toastY += toastH + 6;
    }
}

void Renderer::drawKeycapChips() {
    const int screenW = GetScreenWidth();
    const int screenH = GetScreenHeight();
    const int barH = 34;
    const int barY = screenH - barH;
    DrawRectangle(0, barY, screenW, barH, Color{10, 15, 26, 235});
    DrawLine(0, barY, screenW, barH, Color{56, 189, 248, 60});

    const char* chips[] = {
        "[1-9] Chase UAV",
        "[0] Orbit",
        "[C] Cam Mode",
        "[O] Auto-Orbit",
        "[T] Theme",
        "[Q] Quality",
        "[B] Drain Mult",
        "[D] Low Batt (18%)",
        "[F] Recharge (100%)",
        "[K] Drain (0%)",
        "[H] Hide HUD",
        "[P] Photo"
    };

    int chipX = 14;
    for (const char* chip : chips) {
        drawTextSans(chip, chipX, barY + 10, 12, Color{226, 232, 240, 255}, false);
        chipX += static_cast<int>(std::strlen(chip)) * 7 + 16;
    }
}

} // namespace aerofleet
