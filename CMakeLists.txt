cmake_minimum_required(VERSION 3.20)
project(AeroFleet VERSION 0.1.0 LANGUAGES CXX C)

# Enforce standard C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Export compile commands for language servers
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Optimization and compiler warnings
if(MSVC)
    add_compile_options(/W4 /WX- /permissive-)
else()
    add_compile_options(
        -Wall
        -Wextra
        -Wpedantic
        -Wshadow
        -Wnon-virtual-dtor
        -Wold-style-cast
        -Wcast-align
        -Wunused
        -Woverloaded-virtual
    )
endif()

# Find raylib (using CMake config package or PkgConfig fallback)
find_package(raylib QUIET)
if(NOT raylib_FOUND)
    find_package(PkgConfig QUIET)
    if(PKG_CONFIG_FOUND)
        pkg_check_modules(RAYLIB QUIET raylib)
        if(RAYLIB_FOUND)
            add_library(raylib INTERFACE IMPORTED)
            target_include_directories(raylib INTERFACE ${RAYLIB_INCLUDE_DIRS})
            target_link_libraries(raylib INTERFACE ${RAYLIB_LIBRARIES})
            target_link_directories(raylib INTERFACE ${RAYLIB_LIBRARY_DIRS})
            set(raylib_FOUND TRUE)
        endif()
    endif()
endif()

# Global include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/cpp)

# Core simulation static library
add_library(AeroFleetCore STATIC
    cpp/core/Config.cpp
    cpp/world/WorldGenerator.cpp
    cpp/physics/Kinematics.cpp
    cpp/physics/CollisionSystem.cpp
    cpp/physics/BatteryModel.cpp
    cpp/requests/RequestGenerator.cpp
    cpp/fleet/FleetManager.cpp
    cpp/control/WaypointController.cpp
    cpp/coordinator/HeuristicCoordinator.cpp
)

target_include_directories(AeroFleetCore PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/cpp
)

# Main simulation executable (built when raylib is available)
if(raylib_FOUND)
    add_executable(AeroFleet
        cpp/main.cpp
    )

    target_link_libraries(AeroFleet PRIVATE
        AeroFleetCore
        raylib
    )
else()
    message(STATUS "raylib not found; skipping AeroFleet GUI executable. AeroFleetCore and all tests are enabled.")
endif()

# Enable testing harness
enable_testing()

# Unit tests: Configuration
add_executable(test_config
    tests/test_config.cpp
)
target_link_libraries(test_config PRIVATE AeroFleetCore)
add_test(NAME ConfigTests COMMAND test_config)
set_tests_properties(ConfigTests PROPERTIES WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")

# Unit tests: Math and Spatial Geometry
add_executable(test_math_geometry
    tests/test_math_geometry.cpp
)
target_link_libraries(test_math_geometry PRIVATE AeroFleetCore)
add_test(NAME MathGeometryTests COMMAND test_math_geometry)
set_tests_properties(MathGeometryTests PROPERTIES WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")

# Unit tests: World Generation and Spatial Broadphase
add_executable(test_world_generation
    tests/test_world_generation.cpp
)
target_link_libraries(test_world_generation PRIVATE AeroFleetCore)
add_test(NAME WorldGenerationTests COMMAND test_world_generation)
set_tests_properties(WorldGenerationTests PROPERTIES WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")

# Unit tests: UAV Rigid Kinematics & Motion Integration
add_executable(test_kinematics
    tests/test_kinematics.cpp
)
target_link_libraries(test_kinematics PRIVATE AeroFleetCore)
add_test(NAME KinematicsTests COMMAND test_kinematics)
set_tests_properties(KinematicsTests PROPERTIES WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")

# Unit tests: Continuous Collision Detection & Conflict Resolution
add_executable(test_collision
    tests/test_collision.cpp
)
target_link_libraries(test_collision PRIVATE AeroFleetCore)
add_test(NAME CollisionTests COMMAND test_collision)
set_tests_properties(CollisionTests PROPERTIES WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")

# Unit tests: High-Fidelity Battery, Energy & Charging Station Mechanics
add_executable(test_battery
    tests/test_battery.cpp
)
target_link_libraries(test_battery PRIVATE AeroFleetCore)
add_test(NAME BatteryTests COMMAND test_battery)
set_tests_properties(BatteryTests PROPERTIES WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")

# Unit tests: Delivery Request Generator & Demand Modeling
add_executable(test_requests
    tests/test_requests.cpp
)
target_link_libraries(test_requests PRIVATE AeroFleetCore)
add_test(NAME RequestTests COMMAND test_requests)
set_tests_properties(RequestTests PROPERTIES WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")

# Unit tests: Fleet State Machine & UAV Operational Lifecycle
add_executable(test_fleet
    tests/test_fleet.cpp
)
target_link_libraries(test_fleet PRIVATE AeroFleetCore)
add_test(NAME FleetTests COMMAND test_fleet)
set_tests_properties(FleetTests PROPERTIES WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")

# Unit tests: Deterministic heuristic waypoint navigation
add_executable(test_heuristic_nav
    tests/test_heuristic_nav.cpp
)
target_link_libraries(test_heuristic_nav PRIVATE AeroFleetCore)
add_test(NAME HeuristicNavigationTests COMMAND test_heuristic_nav)
set_tests_properties(HeuristicNavigationTests PROPERTIES WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")

# Unit tests: Heuristic fleet coordinator
add_executable(test_heuristic_coordinator
    tests/test_heuristic_coordinator.cpp
)
target_link_libraries(test_heuristic_coordinator PRIVATE AeroFleetCore)
add_test(NAME HeuristicCoordinatorTests COMMAND test_heuristic_coordinator)
set_tests_properties(HeuristicCoordinatorTests PROPERTIES WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")

# Cross-module integration tests: request generation through delivery execution
add_executable(test_integration
    tests/test_integration.cpp
)
target_link_libraries(test_integration PRIVATE AeroFleetCore)
add_test(NAME IntegrationTests COMMAND test_integration)
set_tests_properties(IntegrationTests PROPERTIES WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
