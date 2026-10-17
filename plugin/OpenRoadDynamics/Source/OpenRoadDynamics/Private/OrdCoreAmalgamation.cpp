// OpenRoad Dynamics — Unreal Engine adapter.
// SPDX-License-Identifier: MIT
//
// Compiles the engine-independent physics core (src/core/src/*.cpp) into the
// OpenRoadDynamics binary, unchanged, as one translation unit.
//
// Rules for this file:
//  * Include NO engine header here (the module uses no PCH and no unity build,
//    so nothing from Unreal precedes these sources).
//  * Keep the list identical to src/core/src. `python3 tools/package_plugin.py
//    --check` fails when a core source is missing from this list.
//  * The include root ("src/...") is published by the OpenRoadCore module
//    (packaged copy or repository path, see OpenRoadCore.Build.cs).
//
// The amalgamation was verified headless to compile as a single translation unit
// with GCC and Clang (-std=c++20 -fno-exceptions -fno-rtti -Wall -Wextra
// -Wshadow-all -Werror); it has not been compiled by UnrealBuildTool.

#include "src/assists.cpp"
#include "src/debug_draw.cpp"
#include "src/drivetrain.cpp"
#include "src/ground.cpp"
#include "src/ground_cache.cpp"
#include "src/lod_manager.cpp"
#include "src/single_track.cpp"
#include "src/telemetry.cpp"
#include "src/tire.cpp"
#include "src/tire_generator.cpp"
#include "src/traffic.cpp"
#include "src/vehicle.cpp"
#include "src/vehicle_def.cpp"
