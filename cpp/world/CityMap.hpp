#pragma once

#include <string>
#include <vector>
#include <unordered_set>
#include <cmath>
#include <memory>
#include "core/Types.hpp"
#include "math/Vec3.hpp"
#include "math/Geometry.hpp"

namespace aerofleet {

struct Building {
    int32_t id{0};
    AABB bounds;
    double height{20.0};

    Building() = default;
    Building(int32_t bId, const AABB& bBounds)
        : id(bId), bounds(bBounds), height(bBounds.maxPoint.z - bBounds.minPoint.z) {}
};

struct StoreLocation {
    int32_t id{0};
    std::string name{"Store Hub"};
    Vec3 position{0.0, 0.0, 0.0};
};

struct CustomerLocation {
    int32_t id{0};
    std::string name{"Customer Site"};
    Vec3 position{0.0, 0.0, 0.0};
};

struct ChargingStation {
    int32_t id{0};
    std::string name{"Charging Dock"};
    Vec3 position{0.0, 0.0, 0.0};
    int32_t capacity{4};
    int32_t occupiedCount{0};

    [[nodiscard]] bool hasAvailableSlot() const noexcept {
        return occupiedCount < capacity;
    }
};

/**
 * @brief 2D Uniform Spatial Hash Grid Broadphase for high-speed static obstacle queries.
 */
class SpatialGridBroadphase {
public:
    SpatialGridBroadphase() = default;

    void initialize(const Vec3& minP, const Vec3& maxP, double cellSize, const std::vector<Building>& buildings) {
        m_xMin = minP.x;
        m_yMin = minP.y;
        m_cellSize = (cellSize > 1.0) ? cellSize : 50.0;

        m_cols = static_cast<int32_t>(std::ceil((maxP.x - minP.x) / m_cellSize));
        m_rows = static_cast<int32_t>(std::ceil((maxP.y - minP.y) / m_cellSize));
        m_cols = std::max(1, m_cols);
        m_rows = std::max(1, m_rows);

        m_grid.clear();
        m_grid.resize(static_cast<size_t>(m_cols * m_rows));

        for (size_t bIdx = 0; bIdx < buildings.size(); ++bIdx) {
            const auto& b = buildings[bIdx];
            int32_t colMin = getCol(b.bounds.minPoint.x);
            int32_t colMax = getCol(b.bounds.maxPoint.x);
            int32_t rowMin = getRow(b.bounds.minPoint.y);
            int32_t rowMax = getRow(b.bounds.maxPoint.y);

            for (int32_t r = rowMin; r <= rowMax; ++r) {
                for (int32_t c = colMin; c <= colMax; ++c) {
                    size_t idx = getCellIndex(c, r);
                    m_grid[idx].push_back(static_cast<int32_t>(bIdx));
                }
            }
        }
    }

    [[nodiscard]] std::vector<int32_t> queryAABB(const AABB& box) const {
        std::vector<int32_t> results;
        if (m_grid.empty()) return results;

        int32_t colMin = getCol(box.minPoint.x);
        int32_t colMax = getCol(box.maxPoint.x);
        int32_t rowMin = getRow(box.minPoint.y);
        int32_t rowMax = getRow(box.maxPoint.y);

        std::unordered_set<int32_t> seen;
        for (int32_t r = rowMin; r <= rowMax; ++r) {
            for (int32_t c = colMin; c <= colMax; ++c) {
                size_t cellIdx = getCellIndex(c, r);
                for (int32_t bIdx : m_grid[cellIdx]) {
                    if (seen.insert(bIdx).second) {
                        results.push_back(bIdx);
                    }
                }
            }
        }
        return results;
    }

    [[nodiscard]] std::vector<int32_t> querySphere(const Vec3& center, double radius) const {
        AABB queryBox(center - Vec3(radius), center + Vec3(radius));
        return queryAABB(queryBox);
    }

private:
    double m_xMin{-500.0};
    double m_yMin{-500.0};
    double m_cellSize{50.0};
    int32_t m_cols{20};
    int32_t m_rows{20};
    std::vector<std::vector<int32_t>> m_grid;

    [[nodiscard]] int32_t getCol(double x) const noexcept {
        int32_t col = static_cast<int32_t>((x - m_xMin) / m_cellSize);
        return MathUtils::clamp(col, 0, m_cols - 1);
    }

    [[nodiscard]] int32_t getRow(double y) const noexcept {
        int32_t row = static_cast<int32_t>((y - m_yMin) / m_cellSize);
        return MathUtils::clamp(row, 0, m_rows - 1);
    }

    [[nodiscard]] size_t getCellIndex(int32_t col, int32_t row) const noexcept {
        return static_cast<size_t>(row * m_cols + col);
    }
};

/**
 * @brief Authoritative representation of city environment, geometry, and landmarks.
 */
class CityMap {
public:
    CityMap() = default;

    std::string name{"AeroFleet City"};
    Vec3 worldMin{-500.0, -500.0, 0.0};
    Vec3 worldMax{500.0, 500.0, 150.0};

    std::vector<Building> buildings;
    std::vector<StoreLocation> stores;
    std::vector<CustomerLocation> customers;
    std::vector<ChargingStation> chargingStations;

    SpatialGridBroadphase broadphase;

    void rebuildBroadphase(double cellSize = 50.0) {
        broadphase.initialize(worldMin, worldMax, cellSize, buildings);
    }

    /**
     * @brief Checks if a coordinate is clear of all building geometry by at least clearanceMargin.
     */
    [[nodiscard]] bool isLocationClear(const Vec3& pt, double clearanceMargin = 1.0) const {
        if (pt.x < worldMin.x + clearanceMargin || pt.x > worldMax.x - clearanceMargin ||
            pt.y < worldMin.y + clearanceMargin || pt.y > worldMax.y - clearanceMargin ||
            pt.z < worldMin.z || pt.z > worldMax.z - clearanceMargin) {
            return false;
        }

        const auto candidateIndices = broadphase.querySphere(pt, clearanceMargin);
        for (int32_t idx : candidateIndices) {
            if (buildings[static_cast<size_t>(idx)].bounds.distanceSquared(pt) <= (clearanceMargin * clearanceMargin)) {
                return false;
            }
        }
        return true;
    }

    /**
     * @brief Continuous swept capsule collision test against all static buildings in the city.
     */
    [[nodiscard]] bool checkSweptCollision(const Capsule& capsule) const {
        AABB queryBounds(
            Vec3(std::min(capsule.a.x, capsule.b.x) - capsule.radius,
                 std::min(capsule.a.y, capsule.b.y) - capsule.radius,
                 std::min(capsule.a.z, capsule.b.z) - capsule.radius),
            Vec3(std::max(capsule.a.x, capsule.b.x) + capsule.radius,
                 std::max(capsule.a.y, capsule.b.y) + capsule.radius,
                 std::max(capsule.a.z, capsule.b.z) + capsule.radius)
        );

        const auto candidates = broadphase.queryAABB(queryBounds);
        for (int32_t idx : candidates) {
            if (capsule.intersects(buildings[static_cast<size_t>(idx)].bounds)) {
                return true;
            }
        }
        return false;
    }

    /**
     * @brief Raycasts against all buildings in the city, returning the closest hit.
     */
    [[nodiscard]] RaycastHit raycast(const Ray& ray, double maxDist = 1000.0) const {
        RaycastHit closestHit;
        closestHit.hit = false;
        closestHit.distance = maxDist;

        // Bounding box along ray path
        const Vec3 endPt = ray.at(maxDist);
        AABB rayBox(
            Vec3(std::min(ray.origin.x, endPt.x), std::min(ray.origin.y, endPt.y), std::min(ray.origin.z, endPt.z)),
            Vec3(std::max(ray.origin.x, endPt.x), std::max(ray.origin.y, endPt.y), std::max(ray.origin.z, endPt.z))
        );

        const auto candidates = broadphase.queryAABB(rayBox);
        for (int32_t idx : candidates) {
            RaycastHit hit = raycastAABB(ray, buildings[static_cast<size_t>(idx)].bounds, closestHit.distance);
            if (hit.hit && hit.distance < closestHit.distance) {
                closestHit = hit;
            }
        }

        return closestHit;
    }
};

} // namespace aerofleet

