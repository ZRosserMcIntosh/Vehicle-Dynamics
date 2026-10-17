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
  "$HERE/ord_web.cpp" "$REPO"/src/core/src/*.cpp "$REPO/src/vehicles/reference_vehicles.cpp" \
  -sMODULARIZE=1 -sEXPORT_ES6=1 -sEXPORT_NAME=createOrdModule -sENVIRONMENT=web,node \
  -sALLOW_MEMORY_GROWTH=0 -sINITIAL_MEMORY=64MB -sFILESYSTEM=0 \
  -sEXPORTED_FUNCTIONS=_ord_create,_ord_reset,_ord_set_preset,_ord_set_aids,_ord_set_input,_ord_advance,_ord_state,_ord_state_header,_ord_state_per_wheel,_ord_ground_height,_ord_bump_lane_y,_ord_proving_bump_y,_ord_proving_split_y,_ord_set_world,_ord_city_heights,_ord_city_classes,_ord_city_colliders,_ord_city_commit,_ord_set_drift_assist \
  -sEXPORTED_RUNTIME_METHODS=HEAPF64,HEAPF32,HEAP16,HEAPU8 \
  -o "$OUT/ord.mjs"
ls -l "$OUT"
