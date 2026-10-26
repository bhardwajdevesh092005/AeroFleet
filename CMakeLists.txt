cmake_minimum_required(VERSION 3.20)
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
project(AeroFleet VERSION 0.1.0 LANGUAGES CXX C)

# Enforce standard C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

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

# Find raylib (using CMake config package, PkgConfig, or FetchContent fallback)
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

if(NOT raylib_FOUND)
    message(STATUS "raylib not found on system; fetching raylib 5.0 via FetchContent...")
    include(FetchContent)
    set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(BUILD_GAMES OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(
        raylib
        GIT_REPOSITORY https://github.com/raysan5/raylib.git
        GIT_TAG 5.5
        GIT_SHALLOW TRUE
    )
    FetchContent_MakeAvailable(raylib)
    set(raylib_FOUND TRUE)
endif()

# ONNX Runtime dependency
set(ONNXRUNTIME_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party/onnxruntime")
if(EXISTS "${ONNXRUNTIME_ROOT}/include/onnxruntime_cxx_api.h")
    set(ONNXRUNTIME_INCLUDE_DIR "${ONNXRUNTIME_ROOT}/include")
    set(ONNXRUNTIME_LIBRARY "${ONNXRUNTIME_ROOT}/lib/libonnxruntime.so")
    set(ONNXRUNTIME_FOUND TRUE)
endif()

# Global include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/cpp)

# Core simulation static library
add_library(AeroFleetCore STATIC
    cpp/core/Config.cpp
    cpp/core/Simulator.cpp
    cpp/world/WorldGenerator.cpp
    cpp/physics/Kinematics.cpp
    cpp/physics/CollisionSystem.cpp
    cpp/physics/BatteryModel.cpp
    cpp/requests/RequestGenerator.cpp
    cpp/fleet/FleetManager.cpp
    cpp/control/WaypointController.cpp
    cpp/control/SafetyLayer.cpp
    cpp/control/ONNXNavigationPolicy.cpp
    cpp/inference/ONNXModel.cpp
    cpp/coordinator/HeuristicCoordinator.cpp
    cpp/visualization/CameraController.cpp
    cpp/visualization/Renderer.cpp
    cpp/telemetry/Metrics.cpp
)

target_include_directories(AeroFleetCore PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/cpp
    ${ONNXRUNTIME_INCLUDE_DIR}
)

target_link_libraries(AeroFleetCore PUBLIC
    raylib
    ${ONNXRUNTIME_LIBRARY}
)

set_target_properties(AeroFleetCore PROPERTIES
    BUILD_RPATH "${ONNXRUNTIME_ROOT}/lib"
    INSTALL_RPATH "${ONNXRUNTIME_ROOT}/lib"
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

# Unit and integration tests: 3D Visualization, Camera Math & Frame-Rate Decoupling
add_executable(test_visualization
    tests/test_visualization.cpp
)
target_link_libraries(test_visualization PRIVATE AeroFleetCore raylib)
add_test(NAME VisualizationTests COMMAND test_visualization)
set_tests_properties(VisualizationTests PROPERTIES WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")

# Unit and integration tests: High-Speed Simulation Runner, Telemetry, Seed Reproducibility & Benchmarking
add_executable(test_reproducibility
    tests/test_reproducibility.cpp
)
target_link_libraries(test_reproducibility PRIVATE AeroFleetCore)
add_test(NAME ReproducibilityTests COMMAND test_reproducibility)
set_tests_properties(ReproducibilityTests PROPERTIES WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")

# Unit and integration tests: Unified Simulator Engine, Observation Extraction & Stepping
add_executable(test_simulator
    tests/test_simulator.cpp
)
target_link_libraries(test_simulator PRIVATE AeroFleetCore)
add_test(NAME SimulatorTests COMMAND test_simulator)
set_tests_properties(SimulatorTests PROPERTIES WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")

# Unit and integration tests: Embedded C++ ONNX Runtime Engine & Navigation Safety Layer
add_executable(test_onnx_runtime
    tests/test_onnx_runtime.cpp
)
target_link_libraries(test_onnx_runtime PRIVATE AeroFleetCore)
set_target_properties(test_onnx_runtime PROPERTIES
    BUILD_RPATH "${ONNXRUNTIME_ROOT}/lib"
)
add_test(NAME ONNXRuntimeTests COMMAND test_onnx_runtime)
set_tests_properties(ONNXRuntimeTests PROPERTIES
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    ENVIRONMENT "LD_LIBRARY_PATH=${ONNXRUNTIME_ROOT}/lib:$ENV{LD_LIBRARY_PATH}"
)

# Battery Stress Tests
add_executable(test_battery_stress tests/test_battery_stress.cpp)
target_link_libraries(test_battery_stress PRIVATE AeroFleetCore)
add_test(NAME BatteryStressTests COMMAND test_battery_stress)
set_tests_properties(BatteryStressTests PROPERTIES
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
)

# Optional Python pybind11 module. Prefer installed CMake packages, then use
# the hermetic Python/pybind11 development files kept under third_party/.  The
# latter matters on course/lab machines where the Python interpreter exists
# but its development package is not installed system-wide.
find_package(Python3 COMPONENTS Interpreter Development.Module QUIET)
find_package(pybind11 CONFIG QUIET)

set(AEROFLEET_PYTHON_BINDINGS_AVAILABLE FALSE)
set(AEROFLEET_PYTHON_EXECUTABLE "${Python3_EXECUTABLE}")
if(Python3_FOUND AND pybind11_FOUND)
    pybind11_add_module(aerofleet_cpp MODULE
        python/bindings/aerofleet_bindings.cpp
    )
    target_link_libraries(aerofleet_cpp PRIVATE AeroFleetCore)
    set_target_properties(aerofleet_cpp PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/python"
        BUILD_RPATH "${ONNXRUNTIME_ROOT}/lib"
        INSTALL_RPATH "${ONNXRUNTIME_ROOT}/lib"
    )
    set(AEROFLEET_PYTHON_BINDINGS_AVAILABLE TRUE)
else()
    set(PYBIND11_INC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/usr/include")
    set(AEROFLEET_VENDORED_PYTHON_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party/usr")
    file(GLOB AEROFLEET_VENDORED_PYTHON_INCLUDE_DIRS
        "${AEROFLEET_VENDORED_PYTHON_ROOT}/include/python3.*"
    )
    if(NOT AEROFLEET_PYTHON_EXECUTABLE)
        find_program(AEROFLEET_PYTHON_EXECUTABLE NAMES python3 python)
    endif()
    if(EXISTS "${PYBIND11_INC_DIR}/pybind11/pybind11.h"
       AND AEROFLEET_VENDORED_PYTHON_INCLUDE_DIRS
       AND AEROFLEET_PYTHON_EXECUTABLE)
        execute_process(
            COMMAND "${AEROFLEET_PYTHON_EXECUTABLE}" -c
                    "import sysconfig; print(sysconfig.get_config_var('EXT_SUFFIX') or '')"
            OUTPUT_VARIABLE AEROFLEET_PYTHON_EXTENSION_SUFFIX
            OUTPUT_STRIP_TRAILING_WHITESPACE
        )
        if(NOT AEROFLEET_PYTHON_EXTENSION_SUFFIX)
            set(AEROFLEET_PYTHON_EXTENSION_SUFFIX ".so")
        endif()
        add_library(aerofleet_cpp MODULE python/bindings/aerofleet_bindings.cpp)
        target_include_directories(aerofleet_cpp SYSTEM PRIVATE
            "${PYBIND11_INC_DIR}" ${AEROFLEET_VENDORED_PYTHON_INCLUDE_DIRS}
        )
        if(WIN32)
            find_package(Python3 COMPONENTS Development.Module QUIET)
            if(TARGET Python3::Module)
                target_link_libraries(aerofleet_cpp PRIVATE AeroFleetCore Python3::Module)
            else()
                target_link_libraries(aerofleet_cpp PRIVATE AeroFleetCore)
            endif()
        else()
            # On UNIX/Linux, a MODULE library must not link libpython directly
            target_link_libraries(aerofleet_cpp PRIVATE AeroFleetCore)
        endif()
        set_target_properties(aerofleet_cpp PROPERTIES
            PREFIX ""
            SUFFIX "${AEROFLEET_PYTHON_EXTENSION_SUFFIX}"
            LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/python"
            BUILD_RPATH "${ONNXRUNTIME_ROOT}/lib"
            INSTALL_RPATH "${ONNXRUNTIME_ROOT}/lib"
        )
        set(AEROFLEET_PYTHON_BINDINGS_AVAILABLE TRUE)
    endif()
endif()

if(AEROFLEET_PYTHON_BINDINGS_AVAILABLE)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(aerofleet_cpp PRIVATE -fvisibility=hidden)
    endif()

    set(AEROFLEET_TEST_PYTHONPATH "${CMAKE_CURRENT_SOURCE_DIR}/python")
    set(AEROFLEET_VENDORED_DIST_PKG "${CMAKE_CURRENT_SOURCE_DIR}/third_party/usr/lib/python3/dist-packages")
    if(EXISTS "${AEROFLEET_VENDORED_DIST_PKG}")
        if(WIN32)
            set(AEROFLEET_TEST_PYTHONPATH "${AEROFLEET_TEST_PYTHONPATH};${AEROFLEET_VENDORED_DIST_PKG}")
        else()
            set(AEROFLEET_TEST_PYTHONPATH "${AEROFLEET_TEST_PYTHONPATH}:${AEROFLEET_VENDORED_DIST_PKG}")
        endif()
    endif()

    set(AEROFLEET_TEST_ENV
        "PYTHONPATH=${AEROFLEET_TEST_PYTHONPATH}"
    )
    if(NOT WIN32)
        set(AEROFLEET_VENDORED_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/usr/lib/x86_64-linux-gnu")
        if(EXISTS "${AEROFLEET_VENDORED_LIB_DIR}")
            list(APPEND AEROFLEET_TEST_ENV
                "LD_LIBRARY_PATH=${AEROFLEET_VENDORED_LIB_DIR}/lapack:${AEROFLEET_VENDORED_LIB_DIR}/blas:${AEROFLEET_VENDORED_LIB_DIR}:$ENV{LD_LIBRARY_PATH}"
            )
        endif()
    endif()

    add_test(NAME PythonBindingTests
        COMMAND ${AEROFLEET_PYTHON_EXECUTABLE} tests/test_bindings.py
    )
    set_tests_properties(PythonBindingTests PROPERTIES
        WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
        ENVIRONMENT "${AEROFLEET_TEST_ENV}"
    )

    execute_process(
        COMMAND ${AEROFLEET_PYTHON_EXECUTABLE} -c
                "try:\n    import gymnasium, numpy, pytest\nexcept Exception:\n    import sys; sys.path.insert(0, '${AEROFLEET_VENDORED_DIST_PKG}'); import gymnasium, numpy, pytest"
        RESULT_VARIABLE AEROFLEET_RL_PYTHON_DEPS_RESULT
        OUTPUT_QUIET
        ERROR_QUIET
    )
    if(AEROFLEET_RL_PYTHON_DEPS_RESULT EQUAL 0)
        add_test(NAME GymnasiumNavigationTests
            COMMAND ${AEROFLEET_PYTHON_EXECUTABLE} -m pytest -q tests/test_nav_env.py
        )
        set_tests_properties(GymnasiumNavigationTests PROPERTIES
            WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
            ENVIRONMENT "${AEROFLEET_TEST_ENV}"
        )

        add_test(NAME DifferentialPhysicsTests
            COMMAND ${AEROFLEET_PYTHON_EXECUTABLE} -m pytest -q tests/test_differential_physics.py
        )
        set_tests_properties(DifferentialPhysicsTests PROPERTIES
            WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
            ENVIRONMENT "${AEROFLEET_TEST_ENV}"
        )

        add_test(NAME ONNXExportTests
            COMMAND ${AEROFLEET_PYTHON_EXECUTABLE} -m pytest -q tests/test_onnx_export.py
        )
        set_tests_properties(ONNXExportTests PROPERTIES
            WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
            ENVIRONMENT "${AEROFLEET_TEST_ENV}"
        )

        add_test(NAME CoordinatorEnvironmentTests
            COMMAND ${AEROFLEET_PYTHON_EXECUTABLE} -m pytest -q tests/test_coordinator_env.py
        )
        set_tests_properties(CoordinatorEnvironmentTests PROPERTIES
            WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
            ENVIRONMENT "${AEROFLEET_TEST_ENV}"
        )
    else()
        message(STATUS "Gymnasium/NumPy/pytest not found; skipping Gymnasium navigation integration tests.")
    endif()
else()
    message(STATUS "pybind11/Python development files not found; skipping aerofleet_cpp and Python binding tests.")
endif()


