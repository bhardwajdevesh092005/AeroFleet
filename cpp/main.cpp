#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <iomanip>
#include <algorithm>

#include "core/Config.hpp"
#include "core/Types.hpp"
#include "core/WorldState.hpp"
#include "core/SimulationClock.hpp"
#include "math/Vec3.hpp"
#include "world/WorldGenerator.hpp"
#include "fleet/FleetManager.hpp"
#include "requests/RequestGenerator.hpp"
#include "control/WaypointController.hpp"
#include "coordinator/HeuristicCoordinator.hpp"
#include "physics/CollisionSystem.hpp"
#include "physics/Kinematics.hpp"
#include "visualization/Renderer.hpp"
#include "visualization/CameraController.hpp"
#include "telemetry/Metrics.hpp"
#include "telemetry/Logger.hpp"
#include "control/ONNXNavigationPolicy.hpp"
#include "control/SafetyLayer.hpp"
#include "coordinator/ONNXCoordinatorPolicy.hpp"
#include <fstream>
#include <filesystem>
#include <memory>

using namespace aerofleet;

namespace {

std::string resolveConfigPath(const std::string& path) {
    std::ifstream f(path);
    if (f.good()) return path;

    std::string fallback1 = "../" + path;
    std::ifstream f1(fallback1);
    if (f1.good()) return fallback1;

    std::string fallback2 = "../../" + path;
    std::ifstream f2(fallback2);
    if (f2.good()) return fallback2;

    return path;
}

void printHelp(const char* prog) {
    std::cout << "========================================\n"
              << "      AeroFleet Simulation Engine       \n"
              << "========================================\n"
              << "Usage: " << prog << " [options]\n\n"
              << "Options:\n"
              << "  --config <path>       Path to simulation JSON config (default: config/simulation.json)\n"
              << "  --headless            Run in headless mode without raylib GUI window\n"
              << "  --benchmark           Run high-speed benchmark and print detailed performance metrics\n"
              << "  --steps <N>           Run for N fixed physics steps then exit (0 = unlimited)\n"
              << "  --speed <factor>      Time scale factor in visual mode (e.g. 1.0 = real-time, 5.0 = 5x)\n"
              << "  --seed <S>            Override PRNG seed\n"
              << "  --output <dir>        Directory to output metrics and logs (default: logs)\n"
              << "  --onnx-nav            Enable embedded ONNX deep RL navigation policy\n"
              << "  --nav-policy <path>   Path to ONNX navigation model (default: models/navigation/nav_policy.onnx)\n"
              << "  --norm-path <path>    Path to normalization stats (default: models/navigation/normalization.json)\n"
              << "  --no-safety           Disable deterministic Safety Layer obstacle filter\n"
              << "  --log-events          Enable event ring-buffer logging to simulation_events.jsonl\n"
              << "  --no-metrics          Disable saving metrics to JSON/CSV\n"
              << "  --scenario <all|standard> Run All-Scenarios Showcase (default in visual) or standard run\n"
              << "  --battery-drain <mult>    Accelerate battery discharge rate (default in visual: 15.0x)\n"
              << "  --fast-battery            Shortcut for --battery-drain 25.0\n"
              << "  --showcase                Explicitly activate All-Scenarios Showcase\n"
              << "  --help, -h            Show this help message and exit\n\n";
}

void setupShowcaseScenarios(
    FleetManager& fleet,
    std::vector<DeliveryRequest>& pendingRequests,
    CityMap& map,
    double currentTime
) {
    if (fleet.size() < 10) return;

    (void)pendingRequests;
    for (auto& cs : map.chargingStations) {
        cs.occupiedCount = 0;
    }

    auto makeClearPos = [&](Vec3 p) {
        while (!map.isLocationClear(p, 3.0) && p.z < 195.0) {
            p.z += 10.0;
        }
        if (!map.isLocationClear(p, 3.0)) {
            p = Vec3(0.0, 0.0, 35.0); // Fall back to central plaza which is guaranteed open
        }
        return p;
    };

    // UAV 0: Available at Central Base, 100% battery (IDLE)
    {
        auto& u = fleet.getUAV(0);
        u.mode = UAVMode::IDLE;
        u.batteryPercent = 100.0;
        u.payloadMass = 0.0;
        u.activeRequestId = -1;
        if (!map.chargingStations.empty()) {
            u.position = map.chargingStations[0].position;
        }
        u.velocity = Vec3(0, 0, 0);
        u.targetWaypoint = u.position;
    }

    // UAV 1: Flying to pickup at Store 0, 78% battery
    if (!map.stores.empty()) {
        auto& u = fleet.getUAV(1);
        u.mode = UAVMode::FLYING_TO_PICKUP;
        u.batteryPercent = 78.0;
        u.payloadMass = 0.0;
        const Vec3 storePos = map.stores[0].position;
        u.position = makeClearPos(Vec3(storePos.x * 0.4, storePos.y * 0.4, 25.0));
        u.targetWaypoint = storePos;
        u.velocity = (storePos - u.position).normalized() * 12.0;

        DeliveryRequest req;
        req.id = 101;
        req.storeId = map.stores[0].id;
        req.pickupLocation = storePos;
        req.dropoffLocation = makeClearPos(Vec3(160.0, 120.0, 15.0));
        req.payloadMass = 1.0;
        req.creationTime = currentTime;
        req.deadline = currentTime + 300.0;
        req.status = RequestStatus::ASSIGNED;
        req.assignedUavId = 1;
        auto& ctx = fleet.getContext(1);
        ctx.activeRequest = req;
        ctx.assignedRequestId = 101;
        u.activeRequestId = 101;
    }

    // UAV 2: Delivering package en-route across skyline, 62% battery
    {
        auto& u = fleet.getUAV(2);
        u.mode = UAVMode::FLYING_TO_DESTINATION;
        u.batteryPercent = 62.0;
        u.payloadMass = 1.25;
        u.position = makeClearPos(Vec3(-90.0, 130.0, 38.0));
        const Vec3 dropoff = makeClearPos(Vec3(-240.0, 210.0, 15.0));
        u.targetWaypoint = dropoff;
        u.velocity = (dropoff - u.position).normalized() * 14.0;

        DeliveryRequest req;
        req.id = 102;
        req.storeId = map.stores.empty() ? 0 : map.stores[0].id;
        req.pickupLocation = Vec3(0, 0, 15.0);
        req.dropoffLocation = dropoff;
        req.payloadMass = 1.25;
        req.creationTime = currentTime - 20.0;
        req.deadline = currentTime + 280.0;
        req.status = RequestStatus::IN_TRANSIT;
        req.assignedUavId = 2;
        auto& ctx = fleet.getContext(2);
        ctx.activeRequest = req;
        ctx.assignedRequestId = 102;
        u.activeRequestId = 102;
    }

    // UAV 3: Dropping off payload with active cargo tether, 52% battery
    {
        auto& u = fleet.getUAV(3);
        u.mode = UAVMode::DROPPING_OFF;
        u.batteryPercent = 52.0;
        u.payloadMass = 0.85;
        const Vec3 dropLoc(130.0, -170.0, 16.0);
        u.position = makeClearPos(dropLoc);
        u.targetWaypoint = u.position;
        u.velocity = Vec3(0, 0, 0);

        DeliveryRequest req;
        req.id = 103;
        req.storeId = 0;
        req.pickupLocation = Vec3(0, 0, 15);
        req.dropoffLocation = u.position;
        req.payloadMass = 0.85;
        req.creationTime = currentTime - 40.0;
        req.deadline = currentTime + 260.0;
        req.status = RequestStatus::IN_TRANSIT;
        req.assignedUavId = 3;
        auto& ctx = fleet.getContext(3);
        ctx.activeRequest = req;
        ctx.assignedRequestId = 103;
        ctx.dwellTimer = 1.0;
        u.activeRequestId = 103;
    }

    // UAV 4: Returning to charge demonstration, 55% battery
    {
        auto& u = fleet.getUAV(4);
        u.mode = UAVMode::RETURNING_TO_CHARGE;
        u.batteryPercent = 55.0;
        u.payloadMass = 0.0;
        u.activeRequestId = -1;
        u.position = makeClearPos(Vec3(210.0, -190.0, 32.0));
        const Vec3 targetStation = (!map.chargingStations.empty()) ? map.chargingStations[0].position : Vec3(0, 0, 0);
        u.targetWaypoint = Vec3(targetStation.x, targetStation.y, 35.0);
        u.velocity = (u.targetWaypoint - u.position).normalized() * 14.0;
        auto& ctx = fleet.getContext(4);
        ctx.targetChargerId = (!map.chargingStations.empty()) ? map.chargingStations[0].id : 0;
    }

    // UAV 5: DOCKED & ACTIVELY CHARGING! 35.0% battery
    if (map.chargingStations.size() > 1) {
        auto& u = fleet.getUAV(5);
        u.mode = UAVMode::CHARGING;
        u.batteryPercent = 35.0;
        u.payloadMass = 0.0;
        u.activeRequestId = -1;
        u.position = map.chargingStations[1].position;
        u.targetWaypoint = u.position;
        u.velocity = Vec3(0, 0, 0);
        map.chargingStations[1].occupiedCount++;
        auto& ctx = fleet.getContext(5);
        ctx.targetChargerId = map.chargingStations[1].id;
    }

    // UAV 6: En-route to charge, 65% battery
    {
        auto& u = fleet.getUAV(6);
        u.mode = UAVMode::RETURNING_TO_CHARGE;
        u.batteryPercent = 65.0;
        u.payloadMass = 0.0;
        u.activeRequestId = -1;
        u.position = makeClearPos(Vec3(-180.0, -150.0, 26.0));
        const Vec3 targetStation = (map.chargingStations.size() > 2) ? map.chargingStations[2].position : Vec3(0, 0, 0);
        u.targetWaypoint = Vec3(targetStation.x, targetStation.y, 30.0);
        u.velocity = (u.targetWaypoint - u.position).normalized() * 15.0;
        auto& ctx = fleet.getContext(6);
        ctx.targetChargerId = (map.chargingStations.size() > 2) ? map.chargingStations[2].id : 0;
    }

    // UAV 7: HEAVY PAYLOAD COURIER (2.95 kg cargo, 74% battery)
    {
        auto& u = fleet.getUAV(7);
        u.mode = UAVMode::FLYING_TO_DESTINATION;
        u.batteryPercent = 74.0;
        u.payloadMass = 2.95;
        u.position = makeClearPos(Vec3(-110.0, -60.0, 36.0));
        const Vec3 dropoff = makeClearPos(Vec3(210.0, 200.0, 20.0));
        u.targetWaypoint = dropoff;
        u.velocity = (dropoff - u.position).normalized() * 11.0;

        DeliveryRequest req;
        req.id = 107;
        req.storeId = 0;
        req.pickupLocation = Vec3(0, 0, 15);
        req.dropoffLocation = dropoff;
        req.payloadMass = 2.95;
        req.creationTime = currentTime - 10.0;
        req.deadline = currentTime + 350.0;
        req.status = RequestStatus::IN_TRANSIT;
        req.assignedUavId = 7;
        auto& ctx = fleet.getContext(7);
        ctx.activeRequest = req;
        ctx.assignedRequestId = 107;
        u.activeRequestId = 107;
    }

    // UAV 8 & 9: TRAFFIC SEPARATION & OBSTACLE CLEARANCE DEMO
    {
        const Vec3 dropoff8 = makeClearPos(Vec3(55.0, 0.0, 32.0));
        auto& u8 = fleet.getUAV(8);
        u8.mode = UAVMode::FLYING_TO_DESTINATION;
        u8.batteryPercent = 85.0;
        u8.payloadMass = 0.5;
        u8.position = makeClearPos(Vec3(-45.0, 0.0, 32.0));
        u8.targetWaypoint = dropoff8;
        u8.velocity = Vec3(12.0, 0.0, 0.0);

        DeliveryRequest req8;
        req8.id = 108;
        req8.storeId = 0;
        req8.pickupLocation = Vec3(0, 0, 15);
        req8.dropoffLocation = dropoff8;
        req8.payloadMass = 0.5;
        req8.creationTime = currentTime - 15.0;
        req8.deadline = currentTime + 300.0;
        req8.status = RequestStatus::IN_TRANSIT;
        req8.assignedUavId = 8;
        auto& ctx8 = fleet.getContext(8);
        ctx8.activeRequest = req8;
        ctx8.assignedRequestId = 108;
        u8.activeRequestId = 108;

        const Vec3 dropoff9 = makeClearPos(Vec3(0.0, 55.0, 32.0));
        auto& u9 = fleet.getUAV(9);
        u9.mode = UAVMode::FLYING_TO_DESTINATION;
        u9.batteryPercent = 82.0;
        u9.payloadMass = 0.5;
        u9.position = makeClearPos(Vec3(0.0, -45.0, 32.0));
        u9.targetWaypoint = dropoff9;
        u9.velocity = Vec3(0.0, 12.0, 0.0);

        DeliveryRequest req9;
        req9.id = 109;
        req9.storeId = 0;
        req9.pickupLocation = Vec3(0, 0, 15);
        req9.dropoffLocation = dropoff9;
        req9.payloadMass = 0.5;
        req9.creationTime = currentTime - 15.0;
        req9.deadline = currentTime + 300.0;
        req9.status = RequestStatus::IN_TRANSIT;
        req9.assignedUavId = 9;
        auto& ctx9 = fleet.getContext(9);
        ctx9.activeRequest = req9;
        ctx9.assignedRequestId = 109;
        u9.activeRequestId = 109;
    }
}

} // namespace

int main(int argc, char* argv[]) {
    std::string configPath = "config/simulation.json";
    std::string outputDir = "logs";
    bool headless = false;
    bool benchmark = false;
    uint64_t maxSteps = 0; // 0 = unlimited
    double timeScale = 1.0;
    uint64_t seedOverride = 0;
    bool hasSeedOverride = false;
    bool logEvents = false;
    bool exportMetrics = true;

    bool useOnnxNav = false;
    std::string navPolicyPath = "models/navigation/nav_policy.onnx";
    std::string normPath = "models/navigation/normalization.json";
    bool useSafety = true;

    bool useOnnxCoord = true;
    std::string coordPolicyPath = "models/coordinator/coord_policy.onnx";

    std::string scenario = "standard";
    double batteryDrain = -1.0;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            printHelp(argv[0]);
            return 0;
        } else if (arg == "--config" && i + 1 < argc) {
            configPath = argv[++i];
        } else if (arg == "--headless") {
            headless = true;
        } else if (arg == "--benchmark") {
            benchmark = true;
        } else if (arg == "--steps" && i + 1 < argc) {
            maxSteps = std::stoull(argv[++i]);
        } else if (arg == "--speed" && i + 1 < argc) {
            timeScale = std::stod(argv[++i]);
        } else if (arg == "--seed" && i + 1 < argc) {
            seedOverride = std::stoull(argv[++i]);
            hasSeedOverride = true;
        } else if (arg == "--output" && i + 1 < argc) {
            outputDir = argv[++i];
        } else if (arg == "--onnx-nav") {
            useOnnxNav = true;
        } else if (arg == "--nav-policy" && i + 1 < argc) {
            navPolicyPath = argv[++i];
            useOnnxNav = true;
        } else if (arg == "--norm-path" && i + 1 < argc) {
            normPath = argv[++i];
        } else if (arg == "--onnx-coord") {
            useOnnxCoord = true;
        } else if (arg == "--heuristic-coord") {
            useOnnxCoord = false;
        } else if (arg == "--coord-policy" && i + 1 < argc) {
            coordPolicyPath = argv[++i];
            useOnnxCoord = true;
        } else if (arg == "--no-safety") {
            useSafety = false;
        } else if (arg == "--log-events") {
            logEvents = true;
        } else if (arg == "--no-metrics") {
            exportMetrics = false;
        } else if (arg == "--scenario" && i + 1 < argc) {
            scenario = argv[++i];
        } else if ((arg == "--battery-drain" || arg == "--battery-multiplier") && i + 1 < argc) {
            batteryDrain = std::stod(argv[++i]);
        } else if (arg == "--fast-battery") {
            batteryDrain = 25.0;
        } else if (arg == "--showcase") {
            scenario = "all";
        } else if (arg == "--standard") {
            scenario = "standard";
        }
    }

    configPath = resolveConfigPath(configPath);

    std::cout << "========================================\n";
    std::cout << "      AeroFleet Simulation Engine       \n";
    std::cout << "========================================\n";
    std::cout << "[System] Configuration: " << configPath << "\n";

    SimulationConfig config;
    try {
        config = SimulationConfig::loadFromFile(configPath);
        if (hasSeedOverride) {
            config.simulation.randomSeed = seedOverride;
        }
        std::cout << "[System] Loaded: " << config.simulation.name << " v" << config.simulation.version << "\n";
        std::cout << "[System] World Volume: "
                  << config.world.xMin << ".." << config.world.xMax << " m (X), "
                  << config.world.yMin << ".." << config.world.yMax << " m (Y), "
                  << config.world.zMin << ".." << config.world.zMax << " m (Z)\n";
        std::cout << "[System] Fleet Size: " << config.fleet.numUavs << " UAVs\n";
        std::cout << "[System] Seed: " << config.simulation.randomSeed << "\n";
    } catch (const std::exception& ex) {
        std::cerr << "[Error] Configuration initialization failed: " << ex.what() << "\n";
        return 1;
    }

    // 1. Generate procedural city environment
    CityMap map = WorldGenerator::generate(config.world, config.simulation.randomSeed);
    std::cout << "[System] City Map Generated: " << map.buildings.size() << " buildings, "
              << map.stores.size() << " stores, " << map.chargingStations.size() << " charging docks.\n";

    // 2. Initialize simulation components
    FleetManager fleet(config.fleet, config.energy, map, config.world.defaultSpawnAltitude);
    RequestGenerator requestGen(config.requests, map, config.fleet.maxHorizontalSpeed,
                                config.simulation.randomSeed,
                                config.world.zMin + config.world.defaultSpawnAltitude);

    WaypointControllerConfig navConfig;
    navConfig.maxHorizontalSpeed = config.fleet.maxHorizontalSpeed;
    navConfig.maxVerticalSpeed = config.fleet.maxVerticalSpeed;
    WaypointController navigator(navConfig);

    std::shared_ptr<ONNXNavigationPolicy> navPolicy;
    if (useOnnxNav) {
        std::string resolvedModel = resolveConfigPath(navPolicyPath);
        std::string resolvedNorm = resolveConfigPath(normPath);
        SafetyLayerConfig safetyCfg;
        if (!useSafety) {
            safetyCfg.safetyMargin = 0.0;
            safetyCfg.lookaheadTime = 0.0;
        }
        try {
            navPolicy = std::make_shared<ONNXNavigationPolicy>(resolvedModel, resolvedNorm, safetyCfg);
            std::cout << "[System] Navigation Engine: Trained ONNX Actor (" << resolvedModel << ")\n";
            std::cout << "[System] Safety Layer: " << (useSafety ? "ENABLED" : "DISABLED") << "\n";
        } catch (const std::exception& ex) {
            std::cerr << "[Warning] Failed to initialize ONNX Navigation Policy: " << ex.what() << "\n";
            std::cerr << "          Falling back to Heuristic APF Navigation.\n";
        }
    }

    std::shared_ptr<ICoordinator> coordinator;
    if (useOnnxCoord) {
        std::string resolvedCoordModel = resolveConfigPath(coordPolicyPath);
        if (std::filesystem::exists(resolvedCoordModel)) {
            try {
                coordinator = std::make_shared<ONNXCoordinatorPolicy>(resolvedCoordModel);
                std::cout << "[System] Coordinator Engine: Deep RL MaskablePPO Policy (" << resolvedCoordModel << ")\n";
            } catch (const std::exception& ex) {
                std::cerr << "[Warning] Failed to initialize ONNX Coordinator: " << ex.what() << "\n";
                std::cerr << "          Falling back to Heuristic Greedy Coordinator.\n";
                coordinator = std::make_shared<HeuristicCoordinator>();
            }
        } else {
            std::cout << "[System] Coordinator Engine: Baseline Heuristic Greedy (Model not found at " << resolvedCoordModel << ")\n";
            coordinator = std::make_shared<HeuristicCoordinator>();
        }
    } else {
        std::cout << "[System] Coordinator Engine: Baseline Heuristic Greedy\n";
        coordinator = std::make_shared<HeuristicCoordinator>();
    }

    CollisionSystem collisions(config.fleet.uavRadius);
    KinematicsIntegrator integrator(KinematicsConfig::fromConfig(config.fleet, config.world));
    SimulationClock clock(config.simulation.dtPhysics, timeScale);

    std::vector<DeliveryRequest> pendingRequests;
    std::vector<DeliveryRequest> deliveredRequests;
    std::vector<DeliveryRequest> requeuedRequests;
    std::vector<CollisionEvent> allCollisionEvents;

    double lastCoordinatorDispatchTime = -10.0;
    size_t totalCreated = 0;

    MetricsTracker metrics;
    EventLogger logger(131072); // High-speed 128k event ring buffer

    // 3. Initialize visualizer (unless headless requested)
    RendererConfig rendererConfig;
    rendererConfig.targetFPS = 30;
    Renderer renderer(rendererConfig);
    CameraController camera(Vec3(0.0, 0.0, 20.0), 650.0);

    bool visualMode = !headless;
    if (visualMode) {
        if (!renderer.init()) {
            std::cout << "[System] Graphical display not available; switching to headless execution.\n";
            visualMode = false;
        } else {
            std::cout << "[System] 3D raylib renderer initialized successfully.\n";
        }
    } else {
        std::cout << "[System] Running in headless mode.\n";
    }

    if (batteryDrain <= 0.0) {
        batteryDrain = 1.0;
    }
    fleet.setBatteryDischargeMultiplier(batteryDrain);
    renderer.setBatteryDrainMultiplier(batteryDrain);
    renderer.setScenarioName(scenario == "all" ? "All Scenarios Showcase" : "Standard Run");

    if (scenario == "all") {
        setupShowcaseScenarios(fleet, pendingRequests, map, 0.0);
        std::cout << "[System] Scenario Initialized: All 10 Fleet Scenarios Active concurrently.\n";
        std::cout << "[System] Accelerated Battery Drain: " << batteryDrain << "x\n";
    }

    // Previous positions vector for continuous collision detection
    std::vector<Vec3> prevPositions(fleet.size());
    for (size_t i = 0; i < fleet.size(); ++i) {
        prevPositions[i] = fleet.getUAV(i).position;
    }

    WorldSnapshot prevSnapshot;
    WorldSnapshot currSnapshot;

    auto wallClockStart = std::chrono::steady_clock::now();
    auto lastFrameTime = wallClockStart;

    std::cout << "[System] Simulation running...\n";

    // Master Simulation Loop
    while (true) {
        if (visualMode && renderer.shouldClose()) {
            break;
        }
        if (maxSteps > 0 && clock.getStepCount() >= maxSteps) {
            break;
        }

        // Measure real elapsed frame time
        auto currentFrameTime = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = currentFrameTime - lastFrameTime;
        lastFrameTime = currentFrameTime;

        if (visualMode) {
            clock.update(elapsed.count());
        } else {
            // In headless mode, step as fast as possible without real-world throttling
            clock.advance();
        }

        // Integrate fixed physics steps
        while (!visualMode || clock.hasStep()) {
            const double dt = clock.getDtPhysics();
            const double currentTime = clock.now();

            // Record previous positions for CCD
            for (size_t i = 0; i < fleet.size(); ++i) {
                prevPositions[i] = fleet.getUAV(i).position;
            }

            // 1. Generate stochastic Poisson customer delivery requests
            auto newRequests = requestGen.step(currentTime, dt);
            if (!newRequests.empty()) {
                totalCreated += newRequests.size();
                for (const auto& req : newRequests) {
                    metrics.recordRequestCreated(req);
                    if (logEvents) {
                        logger.log(currentTime, SimulationEventType::REQUEST_CREATED,
                                   req.id, req.storeId, req.payloadMass);
                    }
                }
                pendingRequests.insert(pendingRequests.end(), newRequests.begin(), newRequests.end());
            }

            // Expire requests while they are still waiting. Assigned requests
            // leave this queue immediately after dispatch below.
            for (auto it = pendingRequests.begin(); it != pendingRequests.end();) {
                const bool timedOut = currentTime > it->creationTime + config.requests.requestTimeoutSeconds;
                if (it->status == RequestStatus::PENDING && (it->isExpired(currentTime) || timedOut)) {
                    it->transitionTo(RequestStatus::EXPIRED, currentTime);
                    metrics.recordRequestExpired(*it, currentTime);
                    if (logEvents) logger.log(currentTime, SimulationEventType::REQUEST_EXPIRED, it->id);
                    it = pendingRequests.erase(it);
                } else {
                    ++it;
                }
            }

            // 2. Dispatch coordinator at coordinator rate (e.g. 1 Hz)
            if (currentTime - lastCoordinatorDispatchTime >= config.simulation.dtCoordinator) {
                if (coordinator) {
                    coordinator->dispatch(fleet, pendingRequests, map, currentTime);
                }
                for (const auto& req : pendingRequests) {
                    if (req.status == RequestStatus::ASSIGNED && logEvents) {
                        logger.log(currentTime, SimulationEventType::REQUEST_ASSIGNED,
                                   req.id, req.assignedUavId);
                    }
                }
                pendingRequests.erase(
                    std::remove_if(pendingRequests.begin(), pendingRequests.end(),
                                   [](const DeliveryRequest& req) { return req.status != RequestStatus::PENDING; }),
                    pendingRequests.end());
                lastCoordinatorDispatchTime = currentTime;
            }

            // 3. Step fleet lifecycle (dwell times, state changes, charging)
            const size_t prevDeliveredCount = deliveredRequests.size();
            fleet.step(dt, currentTime, map, deliveredRequests, requeuedRequests);
            for (size_t d = prevDeliveredCount; d < deliveredRequests.size(); ++d) {
                metrics.recordRequestDelivered(deliveredRequests[d], currentTime);
                if (logEvents) {
                    logger.log(currentTime, SimulationEventType::REQUEST_DELIVERED,
                               deliveredRequests[d].id, deliveredRequests[d].assignedUavId,
                               deliveredRequests[d].deliveryTime - deliveredRequests[d].creationTime);
                }
            }
            if (!requeuedRequests.empty()) {
                for (const auto& req : requeuedRequests) {
                    if (logEvents) {
                        logger.log(currentTime, SimulationEventType::REQUEST_REQUEUED, req.id, req.assignedUavId);
                    }
                }
                pendingRequests.insert(pendingRequests.end(), requeuedRequests.begin(), requeuedRequests.end());
                requeuedRequests.clear();
            }

            // 4. Compute navigation actions and integrate rigid kinematics
            double totalPowerWatts = 0.0;
            const std::vector<UAVState> trafficSnapshot = fleet.getAllUAVs();
            for (auto& uav : fleet.getAllUAVs()) {
                VelocityCommand action;
                if (uav.mode == UAVMode::FLYING_TO_PICKUP ||
                    uav.mode == UAVMode::FLYING_TO_DESTINATION ||
                    uav.mode == UAVMode::RETURNING_TO_CHARGE) {
                    if (navPolicy) {
                        action = navPolicy->computeAction(uav, map, dt);
                        action = navigator.applyTrafficAvoidance(uav, action, trafficSnapshot);
                    } else {
                        action = navigator.computeActionWithTraffic(uav, map, trafficSnapshot, dt);
                    }
                }
                integrator.stepVelocityCommand(uav, action, dt);
                if (uav.mode != UAVMode::IDLE && uav.mode != UAVMode::CHARGING && !uav.isGrounded) {
                    fleet.getBatteryModel().stepDischarge(uav, dt);
                    totalPowerWatts += fleet.getBatteryModel().computePowerWatts(uav.velocity, uav.payloadMass);
                }
            }
            metrics.recordStep(fleet.getAllUAVs(), dt, totalPowerWatts, currentTime);

            // 5. Continuous collision detection and deterministic rollback
            auto newCollisions = collisions.evaluateAndResolve(fleet.getAllUAVs(), prevPositions, map, currentTime);
            if (!newCollisions.empty()) {
                for (const auto& c : newCollisions) {
                    metrics.recordCollision(c);
                    if (logEvents) {
                        logger.log(currentTime,
                                   c.type == CollisionType::BUILDING ? SimulationEventType::COLLISION_BUILDING : SimulationEventType::COLLISION_UAV,
                                   c.uavIdA,
                                   c.type == CollisionType::BUILDING ? c.buildingId : c.uavIdB);
                    }

                    const auto abortCrashedAssignment = [&](UAVId id) {
                        DeliveryRequest requeued;
                        if (fleet.abortAssignment(id, requeued)) {
                            requeuedRequests.push_back(requeued);
                            if (logEvents) logger.log(currentTime, SimulationEventType::REQUEST_REQUEUED,
                                                      requeued.id, id);
                        }
                        if (id >= 0 && static_cast<size_t>(id) < fleet.size()) {
                            fleet.getUAV(static_cast<size_t>(id)).mode = UAVMode::EMERGENCY;
                        }
                    };
                    abortCrashedAssignment(c.uavIdA);
                    if (c.type == CollisionType::UAV_UAV) abortCrashedAssignment(c.uavIdB);
                }
                if (!requeuedRequests.empty()) {
                    pendingRequests.insert(pendingRequests.end(), requeuedRequests.begin(), requeuedRequests.end());
                    requeuedRequests.clear();
                }
                allCollisionEvents.insert(allCollisionEvents.end(), newCollisions.begin(), newCollisions.end());
            }

            if (visualMode) {
                clock.advance();
            } else {
                break; // One step completed in headless iteration
            }
        }

        // Build snapshot
        prevSnapshot = currSnapshot;
        currSnapshot.simulationTime = clock.now();
        currSnapshot.wallTime = std::chrono::duration<double>(std::chrono::steady_clock::now() - wallClockStart).count();
        currSnapshot.stepCount = clock.getStepCount();
        currSnapshot.uavs = fleet.getAllUAVs();
        currSnapshot.chargingStations = map.chargingStations;
        currSnapshot.activeRequests = pendingRequests;
        currSnapshot.recentEvents = allCollisionEvents;

        currSnapshot.metrics.totalCreated = totalCreated;
        currSnapshot.metrics.totalDelivered = deliveredRequests.size();
        currSnapshot.metrics.totalPending = pendingRequests.size();
        currSnapshot.metrics.totalActive = fleet.countInMode(UAVMode::FLYING_TO_PICKUP) +
                                           fleet.countInMode(UAVMode::PICKING_UP) +
                                           fleet.countInMode(UAVMode::FLYING_TO_DESTINATION) +
                                           fleet.countInMode(UAVMode::DROPPING_OFF);
        currSnapshot.metrics.collisionCount = allCollisionEvents.size();
        currSnapshot.metrics.deliverySuccessRate = metrics.getDeadlineSuccessRate();
        currSnapshot.metrics.averageDeliveryDuration = metrics.getMeanLatency();

        // Render frame with sub-step interpolation
        if (visualMode) {
            camera.handleInput(elapsed.count());
            camera.update(elapsed.count(), currSnapshot);
            if (IsKeyPressed(KEY_C)) {
                camera.cycleMode();
            }
            if (IsKeyPressed(KEY_O)) {
                camera.toggleCinematicAutoOrbit();
                renderer.addToast(camera.isCinematicAutoOrbit() ? "Cinematic Auto-Orbit: ON" : "Cinematic Auto-Orbit: OFF", Color{245, 205, 35, 255});
            }
            if (IsKeyPressed(KEY_B)) {
                double currentMult = fleet.getBatteryDischargeMultiplier();
                double nextMult = 1.0;
                if (currentMult < 4.0) nextMult = 5.0;
                else if (currentMult < 12.0) nextMult = 15.0;
                else if (currentMult < 25.0) nextMult = 30.0;
                else if (currentMult < 45.0) nextMult = 50.0;
                else nextMult = 1.0;
                fleet.setBatteryDischargeMultiplier(nextMult);
                renderer.setBatteryDrainMultiplier(nextMult);
                renderer.addToast("Battery Drain: " + std::to_string(static_cast<int>(nextMult)) + "x", Color{245, 205, 35, 255});
                std::cout << "[Simulator] Battery discharge multiplier set to " << nextMult << "x\n";
            }
            if (IsKeyPressed(KEY_D)) {
                UAVId sel = renderer.getSelectedUav();
                if (sel >= 0 && static_cast<size_t>(sel) < fleet.size()) {
                    fleet.setUAVBatteryPercent(sel, 18.0);
                    renderer.addToast("UAV #" + std::to_string(sel) + " Low Battery (18%) RTH", Color{231, 76, 60, 255});
                    std::cout << "[Simulator] UAV #" << sel << " battery forced to 18.0% (Low-Battery RTH Triggered!)\n";
                }
            }
            if (IsKeyPressed(KEY_F)) {
                UAVId sel = renderer.getSelectedUav();
                if (sel >= 0 && static_cast<size_t>(sel) < fleet.size()) {
                    fleet.setUAVBatteryPercent(sel, 100.0);
                    auto& u = fleet.getUAV(sel);
                    u.mode = UAVMode::IDLE;
                    u.velocity = Vec3(0, 0, 0);
                    u.acceleration = Vec3(0, 0, 0);
                    auto& ctx = fleet.getContext(sel);
                    if (ctx.targetChargerId != -1) {
                        for (auto& s : map.chargingStations) {
                            if (s.id == ctx.targetChargerId) {
                                if (s.occupiedCount > 0) s.occupiedCount--;
                            }
                        }
                        ctx.targetChargerId = -1;
                    }
                    renderer.addToast("UAV #" + std::to_string(sel) + " 100% Recharged (Mode: IDLE)", Color{46, 204, 113, 255});
                    std::cout << "[Simulator] UAV #" << sel << " battery recharged to 100.0% (Mode: IDLE)\n";
                }
            }
            if (IsKeyPressed(KEY_K)) {
                UAVId sel = renderer.getSelectedUav();
                if (sel >= 0 && static_cast<size_t>(sel) < fleet.size()) {
                    fleet.setUAVBatteryPercent(sel, 0.0);
                    renderer.addToast("UAV #" + std::to_string(sel) + " Battery Depleted (0%)", Color{231, 76, 60, 255});
                    std::cout << "[Simulator] UAV #" << sel << " battery depleted to 0.0% (Emergency shutdown!)\n";
                }
            }
            if (IsKeyPressed(KEY_S)) {
                setupShowcaseScenarios(fleet, pendingRequests, map, clock.now());
                renderer.addToast("Reset Fleet to Showcase", Color{56, 189, 248, 255});
                std::cout << "[Simulator] Reset fleet to All-Scenarios Showcase.\n";
            }
            for (int k = KEY_ONE; k <= KEY_NINE; ++k) {
                if (IsKeyPressed(k)) {
                    UAVId target = static_cast<UAVId>(k - KEY_ONE);
                    if (static_cast<size_t>(target) < fleet.size()) {
                        renderer.setSelectedUav(target);
                        camera.setChaseTarget(target);
                        camera.setMode(aerofleet::CameraMode::CHASE);
                        renderer.addToast("Chasing UAV #" + std::to_string(target + 1), Color{56, 189, 248, 255});
                    }
                }
            }
            if (IsKeyPressed(KEY_ZERO)) {
                if (fleet.size() >= 10) {
                    UAVId target = 9; // UAV #10 (0-indexed 9)
                    renderer.setSelectedUav(target);
                    camera.setChaseTarget(target);
                    camera.setMode(aerofleet::CameraMode::CHASE);
                    renderer.addToast("Chasing UAV #10", Color{56, 189, 248, 255});
                } else {
                    camera.setMode(aerofleet::CameraMode::ORBIT);
                    camera.setTarget(Vec3(0.0, 0.0, 15.0));
                    renderer.addToast("Camera: ORBIT (Downtown Center)", Color{56, 189, 248, 255});
                }
            }
            if (IsKeyPressed(KEY_O)) {
                camera.setMode(aerofleet::CameraMode::ORBIT);
                camera.setTarget(Vec3(0.0, 0.0, 15.0));
                renderer.addToast("Camera: ORBIT (Downtown Center)", Color{56, 189, 248, 255});
            }
            if (IsKeyPressed(KEY_TAB) || IsKeyPressed(KEY_N)) {
                if (fleet.size() > 0) {
                    UAVId curr = renderer.getSelectedUav();
                    UAVId next = static_cast<UAVId>((curr + 1) % fleet.size());
                    renderer.setSelectedUav(next);
                    camera.setChaseTarget(next);
                    camera.setMode(aerofleet::CameraMode::CHASE);
                    renderer.addToast("Chasing UAV #" + std::to_string(next + 1), Color{56, 189, 248, 255});
                }
            }
            if (IsKeyPressed(KEY_P)) {
                if (fleet.size() > 0) {
                    UAVId curr = renderer.getSelectedUav();
                    UAVId prev = static_cast<UAVId>((curr - 1 + fleet.size()) % fleet.size());
                    renderer.setSelectedUav(prev);
                    camera.setChaseTarget(prev);
                    camera.setMode(aerofleet::CameraMode::CHASE);
                    renderer.addToast("Chasing UAV #" + std::to_string(prev + 1), Color{56, 189, 248, 255});
                }
            }
            WorldSnapshot renderSnapshot = WorldSnapshot::interpolate(prevSnapshot, currSnapshot, clock.getAlpha());
            renderer.draw(renderSnapshot, map, camera);
        }
    }

    auto totalElapsedWall = std::chrono::duration<double>(std::chrono::steady_clock::now() - wallClockStart).count();

    metrics.finalize(totalElapsedWall);
    metrics.printSummary(std::cout);

    if (benchmark) {
        double stepsPerSec = totalElapsedWall > 0.0 ? (static_cast<double>(clock.getStepCount()) / totalElapsedWall) : 0.0;
        double realTimeMult = totalElapsedWall > 0.0 ? (clock.now() / totalElapsedWall) : 0.0;
        std::cout << "\n========================================\n";
        std::cout << "       AeroFleet Benchmark Report        \n";
        std::cout << "========================================\n";
        std::cout << "  Execution Mode:        " << (headless ? "Headless" : "Graphical") << "\n";
        std::cout << "  Navigation Engine:     " << (navPolicy ? "Trained ONNX Actor + Safety Layer" : "Baseline Heuristic APF") << "\n";
        std::cout << "  Safety Layer:          " << (useSafety ? "Enabled" : "Disabled") << "\n";
        std::cout << "  Total Steps Executed:  " << clock.getStepCount() << "\n";
        std::cout << "  Simulation Duration:   " << std::fixed << std::setprecision(2) << clock.now() << " s ("
                  << (clock.now() / 3600.0) << " sim-hours)\n";
        std::cout << "  Wall Execution Time:   " << std::setprecision(4) << totalElapsedWall << " s\n";
        std::cout << "  Step Throughput:       " << std::setprecision(2) << stepsPerSec << " steps/s\n";
        std::cout << "  Real-Time Multiplier:  " << std::setprecision(2) << realTimeMult << "x real-time\n";
        std::cout << "  Total Requests:        Created=" << totalCreated
                  << ", Delivered=" << deliveredRequests.size()
                  << ", Expired=" << metrics.getExpiredCount() << "\n";
        std::cout << "  On-Time Success Rate:  " << std::setprecision(2) << metrics.getDeadlineSuccessRate() << "%\n";
        std::cout << "  Mean Latency:          " << std::setprecision(2) << metrics.getMeanLatency() << " s\n";
        std::cout << "  Total Collisions:      " << allCollisionEvents.size() << " (0 allowed)\n";
        std::cout << "========================================\n\n";
    }

    if (exportMetrics && !outputDir.empty()) {
        std::error_code ec;
        std::filesystem::create_directories(outputDir, ec);
        std::string jsonPath = outputDir + "/metrics.json";
        std::string csvPath = outputDir + "/metrics.csv";
        if (metrics.saveToJsonFile(jsonPath)) {
            std::cout << "[System] Performance metrics saved: " << jsonPath << "\n";
        }
        if (metrics.saveToCsvFile(csvPath)) {
            std::cout << "[System] Performance metrics saved: " << csvPath << "\n";
        }
    }

    if (logEvents && !outputDir.empty()) {
        std::error_code ec;
        std::filesystem::create_directories(outputDir, ec);
        std::string eventsPath = outputDir + "/simulation_events.jsonl";
        if (logger.flushToFile(eventsPath)) {
            std::cout << "[System] Telemetry events logged: " << eventsPath
                      << " (" << logger.totalLogged() << " events)\n";
        }
    }

    renderer.close();
    return 0;
}
