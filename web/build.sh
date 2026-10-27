#!/bin/sh
# OpenRoad Dynamics — build the browser proving ground's WebAssembly module.
# SPDX-License-Identifier: MIT
# Requires the Emscripten SDK (em++ on PATH, e.g. `source emsdk_env.sh`).
# Usage: web/build.sh [output-dir]   (default: web/dist)
set -eu
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/.." && pwd)
OUT=${1:-"$HERE/dist"}
mkdir -p "$OUT"
em++ -std=c++20 -O3 -fno-exceptions -fno-rtti -Wall -Wextra \
  -I"$REPO/src/core/include" -I"$REPO/src/vehicles" \
  "$HERE/ord_web.cpp" "$REPO"/src/core/src/*.cpp "$REPO/src/vehicles/reference_vehicles.cpp" "$REPO/src/vehicles/production_vehicles.cpp" \
  -sMODULARIZE=1 -sEXPORT_ES6=1 -sEXPORT_NAME=createOrdModule -sENVIRONMENT=web,node \
  -sALLOW_MEMORY_GROWTH=0 -sINITIAL_MEMORY=128MB -sSTACK_SIZE=1MB -sFILESYSTEM=0 \
  -sEXPORTED_FUNCTIONS=_ord_create,_ord_reset,_ord_set_preset,_ord_set_aids,_ord_set_input,_ord_advance,_ord_state,_ord_state_header,_ord_state_per_wheel,_ord_ground_height,_ord_bump_lane_y,_ord_proving_bump_y,_ord_proving_split_y,_ord_set_world,_ord_city_heights,_ord_city_classes,_ord_city_colliders,_ord_city_commit,_ord_set_drift_assist,_ord_set_tuning,_ord_bike_create,_ord_bike_remove,_ord_bike_set_assists,_ord_bike_set_input,_ord_bike_advance,_ord_bike_state,_ord_bike_state_size,_ord_set_occupants,_ord_occupant_state,_ord_crash_lane_y,_ord_crash_barrier_x,_ord_crash_pole_x,_ord_roll_kerb_y,_ord_roll_ramp_lane_y,_ord_smash_lane_y,_ord_obstacle_count,_ord_obstacles,_ord_break_events,_ord_break_buffer,_ord_reset_obstacles,_ord_text_buffer,_ord_text,_ord_text_length,_ord_apply_setup,_ord_clear_setup,_ord_setup_json,_ord_set_tyre_warmers,_ord_set_wind,_ord_garage_state,_ord_garage_state_size,_ord_step,_ord_set_weather,_ord_puddles,_ord_puddle_buffer,_ord_test_areas,_ord_snap_push,_ord_snap_count,_ord_snap_restore,_ord_snap_clear,_ord_shadow_start,_ord_shadow_stop,_ord_shadow_state,_ord_launch,_ord_hit_events,_ord_hit_buffer,_ord_crash_structure,_ord_vehicle_info,_ord_loop_profile,_ord_loop_profile_count,_ord_set_damage,_ord_damage_mode,_ord_repair,_ord_damage_state,_ord_cage_count,_ord_cage_nodes,_ord_cage_beam_count,_ord_cage_beams,_ord_telemetry,_ord_aero_info,_ord_tunnel \
  -sEXPORTED_RUNTIME_METHODS=HEAPF64,HEAPF32,HEAP16,HEAPU8 \
  -o "$OUT/ord.mjs"
ls -l "$OUT"
