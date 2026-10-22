#!/bin/sh
# OpenRoad Dynamics — build the browser module with wasi-sdk instead of Emscripten.
# SPDX-License-Identifier: MIT
# Produces the same ord.mjs + ord.wasm interface as web/build.sh (Emscripten): ord.mjs here is a
# small hand-written loader (web/ord_wasi_loader.mjs) instead of Emscripten's glue.
# Usage: WASI_SDK=/path/to/wasi-sdk web/build_wasi.sh [output-dir]   (default: web/dist)
set -eu
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/.." && pwd)
OUT=${1:-"$HERE/dist"}
: "${WASI_SDK:?set WASI_SDK to the wasi-sdk directory}"
mkdir -p "$OUT"
EXPORTS=""
for f in ord_create ord_reset ord_set_preset ord_set_aids ord_set_input ord_advance ord_state ord_state_header ord_state_per_wheel ord_ground_height ord_bump_lane_y ord_proving_bump_y ord_proving_split_y ord_set_world ord_city_heights ord_city_classes ord_city_colliders ord_city_commit ord_set_drift_assist ord_set_tuning ord_bike_create ord_bike_remove ord_bike_set_assists ord_bike_set_input ord_bike_advance ord_bike_state ord_bike_state_size ord_set_occupants ord_occupant_state ord_crash_lane_y ord_crash_barrier_x ord_crash_pole_x ord_roll_kerb_y ord_roll_ramp_lane_y ord_smash_lane_y ord_obstacle_count ord_obstacles ord_break_events ord_break_buffer ord_reset_obstacles ord_text_buffer ord_text ord_text_length ord_apply_setup ord_clear_setup ord_setup_json ord_set_tyre_warmers ord_set_wind ord_garage_state ord_garage_state_size ord_step ord_set_weather ord_puddles ord_puddle_buffer ord_test_areas ord_snap_push ord_snap_count ord_snap_restore ord_snap_clear ord_shadow_start ord_shadow_stop ord_shadow_state ord_launch ord_hit_events ord_hit_buffer ord_crash_structure ord_vehicle_info ord_loop_profile ord_loop_profile_count; do
  EXPORTS="$EXPORTS -Wl,--export=$f"
done
"$WASI_SDK/bin/clang++" --target=wasm32-wasip1 --sysroot="$WASI_SDK/share/wasi-sysroot" \
  -std=c++20 -O3 -fno-exceptions -fno-rtti -Wall -Wextra -mexec-model=reactor \
  -I"$REPO/src/core/include" -I"$REPO/src/vehicles" \
  "$HERE/ord_web.cpp" "$REPO"/src/core/src/*.cpp "$REPO/src/vehicles/reference_vehicles.cpp" "$REPO/src/vehicles/production_vehicles.cpp" \
  $EXPORTS -Wl,--initial-memory=134217728 -Wl,--max-memory=134217728 -Wl,--strip-all \
  -o "$OUT/ord.wasm"
cp "$HERE/ord_wasi_loader.mjs" "$OUT/ord.mjs"
ls -l "$OUT"
