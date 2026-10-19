cmake_minimum_required(VERSION 3.20)
project(OpenRoadDynamics VERSION 0.3.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(ORD_BUILD_TESTS "Build OpenRoad Dynamics tests" ON)
option(ORD_BUILD_TOOLS "Build benchmark tools" ON)
option(ORD_WARNINGS_AS_ERRORS "Treat warnings as errors" OFF)

if(MSVC)
  set(ORD_WARNINGS /W4 /permissive-)
  if(ORD_WARNINGS_AS_ERRORS)
    list(APPEND ORD_WARNINGS /WX)
  endif()
else()
  set(ORD_WARNINGS -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wno-sign-conversion)
  if(ORD_WARNINGS_AS_ERRORS)
    list(APPEND ORD_WARNINGS -Werror)
  endif()
endif()

# ---- Engine-independent physics core (no engine headers allowed) ----
add_library(ord_core
  src/core/src/tire.cpp
  src/core/src/drivetrain.cpp
  src/core/src/ground.cpp
  src/core/src/ground_cache.cpp
  src/core/src/assists.cpp
  src/core/src/vehicle_def.cpp
  src/core/src/vehicle.cpp
  src/core/src/telemetry.cpp
  src/core/src/debug_draw.cpp
  src/core/src/traffic.cpp
  src/core/src/lod_manager.cpp
  src/core/src/tire_generator.cpp
  src/core/src/single_track.cpp
  src/core/src/two_wheeler.cpp
  src/core/src/crash.cpp
)
add_library(ord::core ALIAS ord_core)
target_include_directories(ord_core PUBLIC src/core/include)
target_compile_options(ord_core PRIVATE ${ORD_WARNINGS})

# ---- Reference vehicle definitions (fictional, for benchmarks) ----
add_library(ord_vehicles src/vehicles/reference_vehicles.cpp)
add_library(ord::vehicles ALIAS ord_vehicles)
target_include_directories(ord_vehicles PUBLIC src/vehicles)
target_link_libraries(ord_vehicles PUBLIC ord_core)
target_compile_options(ord_vehicles PRIVATE ${ORD_WARNINGS})

# ---- Benchmark scenarios (shared by tests and the ordbench tool) ----
add_library(ord_scenarios src/scenarios/scenarios.cpp)
target_include_directories(ord_scenarios PUBLIC src/scenarios)
target_link_libraries(ord_scenarios PUBLIC ord_core ord_vehicles)
target_compile_options(ord_scenarios PRIVATE ${ORD_WARNINGS})

if(ORD_BUILD_TOOLS)
  add_executable(ordbench tools/ordbench/main.cpp)
  target_link_libraries(ordbench PRIVATE ord_scenarios)
  target_compile_options(ordbench PRIVATE ${ORD_WARNINGS})
endif()

if(ORD_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
