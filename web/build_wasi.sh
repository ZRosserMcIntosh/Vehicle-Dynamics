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
for f in ord_create ord_reset ord_set_preset ord_set_aids ord_set_input ord_advance ord_state ord_state_header ord_state_per_wheel ord_ground_height ord_bump_lane_y ord_proving_bump_y ord_proving_split_y ord_set_world ord_city_heights ord_city_classes ord_city_colliders ord_city_commit ord_set_drift_assist; do
  EXPORTS="$EXPORTS -Wl,--export=$f"
done
"$WASI_SDK/bin/clang++" --target=wasm32-wasip1 --sysroot="$WASI_SDK/share/wasi-sysroot" \
  -std=c++20 -O3 -fno-exceptions -fno-rtti -Wall -Wextra -mexec-model=reactor \
  -I"$REPO/src/core/include" -I"$REPO/src/vehicles" \
  "$HERE/ord_web.cpp" "$REPO"/src/core/src/*.cpp "$REPO/src/vehicles/reference_vehicles.cpp" \
  $EXPORTS -Wl,--initial-memory=67108864 -Wl,--max-memory=67108864 -Wl,--strip-all \
  -o "$OUT/ord.wasm"
cp "$HERE/ord_wasi_loader.mjs" "$OUT/ord.mjs"
ls -l "$OUT"
