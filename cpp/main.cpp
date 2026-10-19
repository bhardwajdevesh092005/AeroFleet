#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <iomanip>

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
#include <fstream>

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

} // namespace

int main(int argc, char* argv[]) {
    std::string configPath = "config/simulation.json";
    bool headless = false;
    uint64_t maxSteps = 0; // 0 = unlimited
    double timeScale = 1.0;
    uint64_t seedOverride = 0;
    bool hasSeedOverride = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--config" && i + 1 < argc) {
            configPath = argv[++i];
        } else if (arg == "--headless") {
            headless = true;
        } else if (arg == "--steps" && i + 1 < argc) {
            maxSteps = std::stoull(argv[++i]);
        } else if (arg == "--speed" && i + 1 < argc) {
            timeScale = std::stod(argv[++i]);
        } else if (arg == "--seed" && i + 1 < argc) {
            seedOverride = std::stoull(argv[++i]);
            hasSeedOverride = true;
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
    RequestGenerator requestGen(config.requests, map, config.fleet.maxHorizontalSpeed, config.simulation.randomSeed);

    WaypointControllerConfig navConfig;
    navConfig.maxHorizontalSpeed = config.fleet.maxHorizontalSpeed;
    navConfig.maxVerticalSpeed = config.fleet.maxVerticalSpeed;
    WaypointController navigator(navConfig);

    HeuristicCoordinator coordinator;
    CollisionSystem collisions(config.fleet.uavRadius);
    KinematicsIntegrator integrator(KinematicsConfig::fromConfig(config.fleet, config.world));
    SimulationClock clock(config.simulation.dtPhysics, timeScale);

    std::vector<DeliveryRequest> pendingRequests;
    std::vector<DeliveryRequest> deliveredRequests;
    std::vector<DeliveryRequest> requeuedRequests;
    std::vector<CollisionEvent> allCollisionEvents;

    double lastCoordinatorDispatchTime = -10.0;
    size_t totalCreated = 0;

    // 3. Initialize visualizer (unless headless requested)
    RendererConfig rendererConfig;
    rendererConfig.targetFPS = 60;
    Renderer renderer(rendererConfig);
    CameraController camera(Vec3(0.0, 0.0, 15.0), 350.0);

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
                pendingRequests.insert(pendingRequests.end(), newRequests.begin(), newRequests.end());
            }

            // 2. Dispatch coordinator at coordinator rate (e.g. 1 Hz)
            if (currentTime - lastCoordinatorDispatchTime >= config.simulation.dtCoordinator) {
                coordinator.dispatch(fleet, pendingRequests, map, currentTime);
                lastCoordinatorDispatchTime = currentTime;
            }

            // 3. Step fleet lifecycle (dwell times, state changes, charging)
            fleet.step(dt, currentTime, map, deliveredRequests, requeuedRequests);
            if (!requeuedRequests.empty()) {
                pendingRequests.insert(pendingRequests.end(), requeuedRequests.begin(), requeuedRequests.end());
                requeuedRequests.clear();
            }

            // 4. Compute navigation actions and integrate rigid kinematics
            for (auto& uav : fleet.getAllUAVs()) {
                VelocityCommand action;
                if (uav.mode == UAVMode::FLYING_TO_PICKUP ||
                    uav.mode == UAVMode::FLYING_TO_DESTINATION ||
                    uav.mode == UAVMode::RETURNING_TO_CHARGE) {
                    action = navigator.computeAction(uav, map, dt);
                }
                integrator.stepVelocityCommand(uav, action, dt);
                fleet.getBatteryModel().stepDischarge(uav, dt);
            }

            // 5. Continuous collision detection and deterministic rollback
            auto newCollisions = collisions.evaluateAndResolve(fleet.getAllUAVs(), prevPositions, map, currentTime);
            if (!newCollisions.empty()) {
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

        // Render frame with sub-step interpolation
        if (visualMode) {
            WorldSnapshot renderSnapshot = WorldSnapshot::interpolate(prevSnapshot, currSnapshot, clock.getAlpha());
            renderer.draw(renderSnapshot, map, camera);
        }
    }

    auto totalElapsedWall = std::chrono::duration<double>(std::chrono::steady_clock::now() - wallClockStart).count();

    std::cout << "\n========================================\n";
    std::cout << "         Simulation Summary             \n";
    std::cout << "========================================\n";
    std::cout << "[Report] Simulated Time: " << clock.now() << "s\n";
    std::cout << "[Report] Real Wall Time: " << totalElapsedWall << "s\n";
    std::cout << "[Report] Total Steps:    " << clock.getStepCount() << "\n";
    if (totalElapsedWall > 0.0) {
        std::cout << "[Report] Speed Multiplier: " << (clock.now() / totalElapsedWall) << "x real-time\n";
    }
    std::cout << "[Report] Requests Created:   " << totalCreated << "\n";
    std::cout << "[Report] Requests Delivered: " << deliveredRequests.size() << "\n";
    std::cout << "[Report] Requests Pending:   " << pendingRequests.size() << "\n";
    std::cout << "[Report] Collision Incidents:" << allCollisionEvents.size() << "\n";
    std::cout << "========================================\n";

    renderer.close();
    return 0;
}
