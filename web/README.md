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

`ord_state()` returns a `double*` that `ord_advance()` refreshes: a 36-value header, then 17 values per wheel (`ord_state_per_wheel()`).

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
| 32–35 | drift assist active, target sideslip (rad), throttle cut, steering change (rad) | 16 | grip used (0 = none, ~1 = at the limit) |

`ord_set_tuning(steerLock, diffType, preload, powerRamp, coastRamp)` sets the steering lock (rad,
0 = stock) and the rear differential (−1 stock, 0 open, 1 locked, 2 clutch LSD) now and whenever
the car is re-created.

## Two-wheelers

`ord_bike_create(kind, x, y, yaw, speed)` (0 bicycle, 1 e-scooter, 2 sport motorcycle),
`ord_bike_set_assists(mode, balance, abs, tcs, wheelieControl, wheeliePitch)` (mode 0 = lean, 1 =
direct bar torque), `ord_bike_set_input(steer, throttle, frontBrake, rearBrake)` (steer + = left),
`ord_bike_advance(dt)` (fixed 1/500 s steps), `ord_bike_remove()`. `ord_bike_state()` returns
`ord_bike_state_size()` (66) doubles:

| Index | Meaning |
|---|---|
| 0–3 | kind, fallen, time, speed (m/s) |
| 4–8 | lean, steer, pitch, lean target (rad; lean/steer + = right), bar torque (N·m) |
| 9–12 | drive, front brake, rear brake, balance-aid torques (N·m) |
| 13–15 | ABS, TCS, wheelie control active |
| 16–21 | grip used rear/front, load rear/front (N), camber rear/front (rad) |
| 22–25 | heading (+ left), rear contact point x, y, z |
| 26–28 | rear/front wheel radius, wheelbase |
| 29–31 | crash impact speed (city walls), steer and throttle input |
| 32–59 | 4 bodies (rear wheel, rear frame + rider, front frame, front wheel): position x, y, z, quaternion w, x, y, z |
| 60–65 | wheel spin rear/front (rad/s), crown radius rear/front, total mass, steer-axis tilt |

Performance in the browser is not comparable to the native benchmarks and is not used for any claim.

## Crash dummies

`ord_set_occupants(belted, frontAirbag, sideAirbag)` sets both front seats (driver on the left).
`ord_occupant_state(seat)` (0 driver, 1 passenger, 2 the bike rider's last crash) returns 24
doubles: torso dx, dy and head dx, dy (m, car frame), current g, event active, events so far, the
last finished report (valid, zone 1 front / 2 rear / 3 left / 4 right, delta-V m/s, car pulse
peak g, torso peak g, head peak g, chest 3 ms g, HIC15, torso and head travel m, intrusion m, hard
contact, serious head injury risk, life-threatening injury risk, Joksch delta-V statistic, verdict
0 walks away … 4 critical) and the seat y. The test track has a crash-test lane
(`ord_crash_lane_y`, `ord_crash_barrier_x`, `ord_crash_pole_x`): a rigid barrier and a 30 cm pole.
See [crash-survivability.md](../docs/architecture/crash-survivability.md).
