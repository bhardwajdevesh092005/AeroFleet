#pragma once

#include <string>
#include <random>
#include "world/CityMap.hpp"
#include "core/Config.hpp"

namespace aerofleet {

struct GenerationParameters {
    int32_t gridBlocksX{14};
    int32_t gridBlocksY{14};
    double streetWidth{24.0};
    double minBuildingHeight{35.0};
    double maxBuildingHeight{185.0};
    int32_t numStores{6};
    int32_t numCustomers{30};
    int32_t numChargingStations{12};
};

class WorldGenerator {
public:
    static CityMap generate(
        const WorldSettings& settings,
        uint64_t seed = 42,
        const GenerationParameters& params = GenerationParameters{}
    );

    static CityMap loadFromJsonFile(
        const std::string& filepath,
        const WorldSettings& settings
    );
};

} // namespace aerofleet

