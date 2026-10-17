# Browser build (WebAssembly)

`ord_web.cpp` is a thin C ABI over `ord::Vehicle` for the browser test track at
<https://zrossermcintosh.com/sao-paulo/driving/>. It compiles the unmodified core with
Emscripten and runs it at a fixed 1/360 s step. The page only renders and reads input.

```sh
source /path/to/emsdk/emsdk_env.sh
web/build.sh                  # -> web/dist/ord.mjs + ord.wasm (~110 KB)
node web/smoke_test.mjs       # 0-100 km/h per preset, steady circle, ground function
```

Without Emscripten, `WASI_SDK=/path/to/wasi-sdk web/build_wasi.sh` builds the same module with
[wasi-sdk](https://github.com/WebAssembly/wasi-sdk) (tested with wasi-sdk 25); `ord_wasi_loader.mjs`
becomes `ord.mjs` and exposes the identical `_ord_*` / `HEAPF64` interface.

Cars (`ord_create` index): 0 Sports RWD, 1 Hatch FWD, 2 Saloon AWD, 3 Hypercar AWD, 4 Race car,
5 Rosser Stella V8 (flat-plane V8, 9,000 rpm; the São Paulo World game's own car).

The proving ground (bump lane heights, wet and rough patches) is defined in `ord_web.cpp`
(including the default-start proving ground west of the drag strip: graded bump course, split-mu lane,
dry R30 and wet R25 skidpads; `ord_proving_bump_y` / `ord_proving_split_y` give the lane positions)
and exported (`ord_ground_height`), so the page draws exactly the surface the physics sees.

## State buffer

`ord_state()` returns a `double*` that `ord_advance()` refreshes: a 36-value header, then 16 values per wheel.

| Header | Meaning | Wheel | Meaning |
|---|---|---|---|
| 0–2 | body origin x, y, z (m, ISO 8855 world) | 0–1 | hardpoint x, y (body) |
| 3–6 | orientation quaternion w, x, y, z | 2 | wheel-centre z (body) |
| 7–10 | speed, lateral velocity, yaw rate, sideslip | 3 | radius |
| 11–13 | body acceleration x, y, z (m/s²) | 4 | steer angle (rad) |
| 14–15 | engine rpm, gear | 5 | wheel speed (rad/s) |
| 16–17 | steer angle, throttle after TCS | 6 | load (N) |
| 18–20 | TCS cut, countersteer angle, ESC yaw error | 7–8 | slip ratio, slip angle |
| 21–23 | time, wheel count, faulted | 9–10 | Fx, Fy (N) |
| 24–26 | shifting, wheel-0 radius, clutch | 11–13 | contact, locked, ABS active |
| 27–30 | steer/throttle/brake input, ground queries | 14–15 | ESC brake torque, surface id |
| 31 | largest impact speed this frame (m/s, city world) | | |
| 32–35 | drift assist active, target sideslip (rad), throttle cut, steering change (rad) | | |

Performance in the browser is not comparable to the native benchmarks and is not used for any claim.
