#!/bin/sh
# OpenRoad Dynamics — build the many-vehicle host module (gsp_web.cpp) with wasi-sdk.
# SPDX-License-Identifier: MIT
# Produces gsp.wasm + gsp.mjs (loader: web/gsp_loader.mjs) for Georgia State Pursuit
# (zrossermcintosh.com/gsp), where the page's Rapier world hosts every car's chassis.
# Usage: WASI_SDK=/path/to/wasi-sdk web/build_gsp_wasi.sh [output-dir]   (default: web/dist)
set -eu
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/.." && pwd)
OUT=${1:-"$HERE/dist"}
: "${WASI_SDK:?set WASI_SDK to the wasi-sdk directory}"
mkdir -p "$OUT"
EXPORTS=""
for f in $(grep -o 'GSP_EXPORT [^(]*(' "$HERE/gsp_web.cpp" | sed -E 's/.* \*?([a-z_0-9]+)\($/\1/'); do
  EXPORTS="$EXPORTS -Wl,--export=$f"
done
"$WASI_SDK/bin/clang++" --target=wasm32-wasip1 --sysroot="$WASI_SDK/share/wasi-sysroot" \
  -std=c++20 -O3 -fno-exceptions -fno-rtti -Wall -Wextra -mexec-model=reactor \
  -I"$REPO/src/core/include" \
  "$HERE/gsp_web.cpp" "$REPO"/src/core/src/*.cpp \
  $EXPORTS -Wl,-z,stack-size=1048576 -Wl,--initial-memory=67108864 -Wl,--max-memory=268435456 -Wl,--strip-all \
  -o "$OUT/gsp.wasm"
cp "$HERE/gsp_loader.mjs" "$OUT/gsp.mjs"
ls -l "$OUT/gsp.wasm" "$OUT/gsp.mjs"
