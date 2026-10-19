#pragma once

#include <string>
#include <random>
#include "world/CityMap.hpp"
#include "core/Config.hpp"

namespace aerofleet {

struct GenerationParameters {
    int32_t gridBlocksX{6};
    int32_t gridBlocksY{6};
    double streetWidth{25.0};
    double minBuildingHeight{15.0};
    double maxBuildingHeight{80.0};
    int32_t numStores{4};
    int32_t numCustomers{20};
    int32_t numChargingStations{4};
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

