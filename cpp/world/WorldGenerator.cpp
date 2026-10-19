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
    std::uniform_real_distribution<double> heightDist(params.minBuildingHeight, params.maxBuildingHeight);
    std::uniform_real_distribution<double> setbackDist(2.0, 6.0);

    const double totalWidth = settings.xMax - settings.xMin;
    const double totalHeight = settings.yMax - settings.yMin;

    const double blockWidth = (totalWidth - (params.gridBlocksX + 1) * params.streetWidth) / params.gridBlocksX;
    const double blockHeight = (totalHeight - (params.gridBlocksY + 1) * params.streetWidth) / params.gridBlocksY;

    int32_t buildingIdCounter = 1;

    // Generate grid blocks with buildings
    for (int32_t bx = 0; bx < params.gridBlocksX; ++bx) {
        double blockXMin = settings.xMin + params.streetWidth + bx * (blockWidth + params.streetWidth);
        double blockXMax = blockXMin + blockWidth;

        for (int32_t by = 0; by < params.gridBlocksY; ++by) {
            double blockYMin = settings.yMin + params.streetWidth + by * (blockHeight + params.streetWidth);
            double blockYMax = blockYMin + blockHeight;

            // Leave central plaza or occasional park empty (e.g. 1 out of 8 blocks)
            if (bx == params.gridBlocksX / 2 && by == params.gridBlocksY / 2) {
                continue; // Central Open Plaza
            }

            const double setback = setbackDist(rng);
            const double bMinX = blockXMin + setback;
            const double bMaxX = blockXMax - setback;
            const double bMinY = blockYMin + setback;
            const double bMaxY = blockYMax - setback;
            const double bHeight = heightDist(rng);

            if (bMaxX > bMinX + 5.0 && bMaxY > bMinY + 5.0) {
                AABB bounds(Vec3(bMinX, bMinY, settings.zMin), Vec3(bMaxX, bMaxY, settings.zMin + bHeight));
                map.buildings.emplace_back(buildingIdCounter++, bounds);
            }
        }
    }

    // Build spatial hash broadphase
    map.rebuildBroadphase(50.0);

    // Generate Store Locations along open street corridors
    int32_t storeIdCounter = 1;
    std::uniform_real_distribution<double> xCoordDist(settings.xMin + 50.0, settings.xMax - 50.0);
    std::uniform_real_distribution<double> yCoordDist(settings.yMin + 50.0, settings.yMax - 50.0);

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
            const Vec3 offset(15.0, 0.0, 0.0);
            if (map.isLocationClear(storePos + offset, 4.0)) {
                stationPos = storePos + offset;
                placed = true;
            }
        }
        if (!placed) {
            for (int retry = 0; retry < 500; ++retry) {
                stationPos = Vec3(xCoordDist(rng), yCoordDist(rng), settings.zMin);
                if (map.isLocationClear(stationPos, 5.0)) {
                    placed = true;
                    break;
                }
            }
        }
        if (!placed) {
            stationPos = Vec3(20.0 * c, 20.0 * c, settings.zMin);
        }
        map.chargingStations.push_back(ChargingStation{
            stationIdCounter,
            "Charging Dock " + std::to_string(stationIdCounter),
            stationPos,
            4, // 4 UAV capacity
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

