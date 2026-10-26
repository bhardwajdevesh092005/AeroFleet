#include "world/WorldGenerator.hpp"
#include <iostream>
#include <algorithm>
#include <stdexcept>

namespace aerofleet {

CityMap WorldGenerator::generate(
    const WorldSettings& settings,
    uint64_t seed,
    const GenerationParameters& params
) {
    CityMap map;
    map.name = "Procedural Urban Map (Seed: " + std::to_string(seed) + ")";
    map.worldMin = Vec3(settings.xMin, settings.yMin, settings.zMin);
    map.worldMax = Vec3(settings.xMax, settings.yMax, settings.zMax);

    std::mt19937_64 rng(seed);

    const double totalWidth = settings.xMax - settings.xMin;
    const double totalHeight = settings.yMax - settings.yMin;

    // Dynamically adapt grid subdivisions if world domain is smaller than standard metropolitan scale
    int32_t blocksX = params.gridBlocksX;
    int32_t blocksY = params.gridBlocksY;
    double streetW = params.streetWidth;

    while (blocksX > 2 && (totalWidth - (blocksX + 1) * streetW) / blocksX < 15.0) {
        if (streetW > 12.0) {
            streetW *= 0.75;
        } else {
            blocksX--;
        }
    }
    while (blocksY > 2 && (totalHeight - (blocksY + 1) * streetW) / blocksY < 15.0) {
        blocksY--;
    }

    const double blockWidth = (totalWidth - (blocksX + 1) * streetW) / blocksX;
    const double blockHeight = (totalHeight - (blocksY + 1) * streetW) / blocksY;

    // Setback distribution: 6m to 16m for pedestrian walkways, trees, and corner plazas
    std::uniform_real_distribution<double> setbackDist(std::min(4.0, blockWidth * 0.05), std::min(16.0, blockWidth * 0.22));

    const double maxRadius = std::sqrt((totalWidth * 0.5) * (totalWidth * 0.5) + (totalHeight * 0.5) * (totalHeight * 0.5));
    const double effectiveCeiling = (settings.zMax > settings.zMin + 20.0) ? (settings.zMax - 15.0) : settings.zMax;

    int32_t buildingIdCounter = 1;

    // Generate grid blocks with buildings
    for (int32_t bx = 0; bx < blocksX; ++bx) {
        double blockXMin = settings.xMin + streetW + bx * (blockWidth + streetW);
        double blockXMax = blockXMin + blockWidth;

        for (int32_t by = 0; by < blocksY; ++by) {
            double blockYMin = settings.yMin + streetW + by * (blockHeight + streetW);
            double blockYMax = blockYMin + blockHeight;

            // 1. Central Open Plaza (contains coordinate (0, 0) for safe spawn & initial waypoint)
            if (bx == blocksX / 2 && by == blocksY / 2) {
                continue;
            }
            // 2. Municipal Parks in four quadrants for city greenery
            if (blocksX >= 8 && blocksY >= 8) {
                if ((bx == blocksX / 4 && by == blocksY / 4) ||
                    (bx == 3 * blocksX / 4 && by == blocksY / 4) ||
                    (bx == blocksX / 4 && by == 3 * blocksY / 4) ||
                    (bx == 3 * blocksX / 4 && by == 3 * blocksY / 4)) {
                    continue;
                }
            }

            const double setback = setbackDist(rng);
            const double bMinX = blockXMin + setback;
            const double bMaxX = blockXMax - setback;
            const double bMinY = blockYMin + setback;
            const double bMaxY = blockYMax - setback;

            // Center-to-Perimeter height gradient:
            // Downtown CBD center has soaring skyscrapers up to 185m, tapering to 40-75m at the perimeter
            const double blockCenterX = (blockXMin + blockXMax) * 0.5;
            const double blockCenterY = (blockYMin + blockYMax) * 0.5;
            const double distFromCenter = std::sqrt(blockCenterX * blockCenterX + blockCenterY * blockCenterY);
            const double distNorm = std::min(1.0, distFromCenter / (maxRadius * 0.82));

            const double targetMaxH = std::min(effectiveCeiling, params.maxBuildingHeight * (1.0 - distNorm * 0.58));
            const double targetMinH = std::max(params.minBuildingHeight, targetMaxH * 0.55);
            std::uniform_real_distribution<double> heightDist(targetMinH, targetMaxH);
            const double bHeight = heightDist(rng);

            if (bMaxX > bMinX + 5.0 && bMaxY > bMinY + 5.0) {
                AABB bounds(Vec3(bMinX, bMinY, settings.zMin), Vec3(bMaxX, bMaxY, settings.zMin + bHeight));
                map.buildings.emplace_back(buildingIdCounter++, bounds);
            }
        }
    }

    // Build spatial hash broadphase
    map.rebuildBroadphase(40.0);

    // Generate Store Locations along open street corridors
    int32_t storeIdCounter = 1;
    const double margin = std::min(50.0, totalWidth * 0.15);
    std::uniform_real_distribution<double> xCoordDist(settings.xMin + margin, settings.xMax - margin);
    std::uniform_real_distribution<double> yCoordDist(settings.yMin + margin, settings.yMax - margin);

    for (int32_t s = 0; s < params.numStores; ++s) {
        Vec3 storePos;
        bool placed = false;
        for (int retry = 0; retry < 500; ++retry) {
            storePos = Vec3(xCoordDist(rng), yCoordDist(rng), settings.zMin);
            if (map.isLocationClear(storePos, 8.0)) {
                placed = true;
                break;
            }
        }
        if (!placed) {
            storePos = Vec3(0.0, 0.0, settings.zMin); // Fallback to central plaza
        }
        map.stores.push_back(StoreLocation{
            storeIdCounter,
            "Store Hub " + std::to_string(storeIdCounter),
            storePos
        });
        storeIdCounter++;
    }

    // Generate Charging Stations adjacent to stores or in clear spaces
    int32_t stationIdCounter = 1;
    for (int32_t c = 0; c < params.numChargingStations; ++c) {
        Vec3 stationPos;
        bool placed = false;
        if (c < static_cast<int32_t>(map.stores.size())) {
            // Place near corresponding store hub
            const Vec3& storePos = map.stores[static_cast<size_t>(c)].position;
            const Vec3 offset(18.0, 0.0, 0.0);
            if (map.isLocationClear(storePos + offset, 4.0)) {
                stationPos = storePos + offset;
                placed = true;
            }
        }
        if (!placed) {
            // Distribute stations across the 4 city sectors
            const double quadSignX = (c % 2 == 0) ? 1.0 : -1.0;
            const double quadSignY = ((c / 2) % 2 == 0) ? 1.0 : -1.0;
            std::uniform_real_distribution<double> sectorX(margin * 0.5, settings.xMax - margin);
            std::uniform_real_distribution<double> sectorY(margin * 0.5, settings.yMax - margin);
            for (int retry = 0; retry < 500; ++retry) {
                stationPos = Vec3(quadSignX * sectorX(rng), quadSignY * sectorY(rng), settings.zMin);
                if (map.isLocationClear(stationPos, 5.0)) {
                    placed = true;
                    break;
                }
            }
        }
        if (!placed) {
            stationPos = Vec3(40.0 * (c - 5), 40.0 * (c - 5), settings.zMin);
        }
        map.chargingStations.push_back(ChargingStation{
            stationIdCounter,
            "Charging Dock " + std::to_string(stationIdCounter),
            stationPos,
            8, // 8 UAV capacity per station
            0  // 0 occupied initially
        });
        stationIdCounter++;
    }

    // Generate Customer Locations
    int32_t custIdCounter = 1;
    for (int32_t cu = 0; cu < params.numCustomers; ++cu) {
        Vec3 custPos;
        bool placed = false;
        for (int retry = 0; retry < 500; ++retry) {
            custPos = Vec3(xCoordDist(rng), yCoordDist(rng), settings.zMin);
            if (map.isLocationClear(custPos, 4.0)) {
                placed = true;
                break;
            }
        }
        if (!placed) {
            custPos = Vec3(10.0 * cu, -10.0 * cu, settings.zMin);
        }
        map.customers.push_back(CustomerLocation{
            custIdCounter,
            "Customer Site " + std::to_string(custIdCounter),
            custPos
        });
        custIdCounter++;
    }

    return map;
}

CityMap WorldGenerator::loadFromJsonFile(
    const std::string& filepath,
    const WorldSettings& settings
) {
    JsonValue root = JsonValue::fromFile(filepath);
    CityMap map;
    map.worldMin = Vec3(settings.xMin, settings.yMin, settings.zMin);
    map.worldMax = Vec3(settings.xMax, settings.yMax, settings.zMax);

    if (root.contains("city")) {
        const auto& c = root["city"];
        map.name = c.getOrDefault<std::string>("name", "Urban Map");
    }

    if (root.contains("stores") && root["stores"].isArray()) {
        const auto& storesArr = root["stores"].asArray();
        for (const auto& s : storesArr) {
            StoreLocation store;
            store.id = static_cast<int32_t>(s.getOrDefault<int64_t>("id", 0));
            store.name = s.getOrDefault<std::string>("name", "Store");
            store.position = Vec3(
                s.getOrDefault<double>("x", 0.0),
                s.getOrDefault<double>("y", 0.0),
                s.getOrDefault<double>("z", 0.0)
            );
            map.stores.push_back(store);
        }
    }

    if (root.contains("charging_stations") && root["charging_stations"].isArray()) {
        const auto& chargeArr = root["charging_stations"].asArray();
        for (const auto& cs : chargeArr) {
            ChargingStation station;
            station.id = static_cast<int32_t>(cs.getOrDefault<int64_t>("id", 0));
            station.name = cs.getOrDefault<std::string>("name", "Dock");
            station.position = Vec3(
                cs.getOrDefault<double>("x", 0.0),
                cs.getOrDefault<double>("y", 0.0),
                cs.getOrDefault<double>("z", 0.0)
            );
            station.capacity = static_cast<int32_t>(cs.getOrDefault<int64_t>("capacity", 4));
            station.occupiedCount = 0;
            map.chargingStations.push_back(station);
        }
    }

    map.rebuildBroadphase(50.0);
    return map;
}

} // namespace aerofleet

