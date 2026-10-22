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
5 Rosser Stella V8 (flat-plane V8, 9,000 rpm; the São Paulo World game's own car), and approximations of real
cars from published figures (`src/vehicles/production_vehicles.hpp`): 6 Porsche 911 Carrera S, 7 911 Carrera 4S,
8 Mercedes-AMG S 63 4MATIC+, 9 Mercedes-Benz S 600 Guard, 10 Formula 1 car (2026 regulations).

The proving ground (bump lane heights, wet and rough patches) is defined in `ord_web.cpp`
(including the default-start proving ground west of the drag strip: graded bump course, split-mu lane,
dry R30 and wet R25 skidpads; `ord_proving_bump_y` / `ord_proving_split_y` give the lane positions)
and exported (`ord_ground_height`), so the page draws exactly the surface the physics sees.

## State buffer

`ord_state()` returns a `double*` that `ord_advance()` refreshes: a 39-value header, then 17 values per wheel (`ord_state_per_wheel()`).

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
| 36–38 | upright (body z · world z: 1 upright, −1 on the roof), body hull points touching the ground, their total normal force (N) | | |

`ord_set_tuning(steerLock, diffType, preload, powerRamp, coastRamp)` sets the steering lock (rad,
0 = stock) and the rear differential (−1 stock, 0 open, 1 locked, 2 clutch LSD) now and whenever
the car is re-created.

## Garage

Setups go in and out as JSON (fields and units in
[`ord/setup.hpp`](../src/core/include/ord/setup.hpp)). Write UTF-8 into
`ord_text_buffer(n)` and call `ord_apply_setup()`. Only the fields present change. It returns
1, or 0 with the reason in `ord_text()` / `ord_text_length()`. The car is rebuilt where it is,
at the speed it had. `ord_setup_json(stock)` puts the current setup (or the stock one) into
`ord_text()`. `ord_clear_setup()` restores the stock car. `ord_set_tyre_warmers(celsius)`
(< 0 = ambient) sets the tread temperature now and on every re-creation.
`ord_set_wind(wx, wy)` sets a steady wind in m/s, as the direction the air moves.

`ord_garage_state()` returns `ord_garage_state_size()` doubles:
- 0–11: airspeed, airflow yaw, front and rear downforce, drag, wing downforce, floor clearance
  front and rear, wing stalled, floor stalled, ambient °C, air density.
- 12–13: front and rear roll-centre height above the ground (m), from the suspension linkage.
- Then, from 14, 4 per wheel: camber (rad, negative = top in), tread °C, optimal °C, grip factor from
  temperature.

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

## Breakable walls

North of the crash lane, three run-up lanes (`ord_smash_lane_y(k)`: 0 glass storefront, 1
hollow-block wall, 2 brick wall) end in a wall at the barrier's x, each built from three 2.5 m
sections. A section has a fracture energy (representative estimates: 1.5 kJ for a 10 mm pane,
25 kJ for an unreinforced block wall 2 m high, 120 kJ for a double-wythe brick wall). A hit whose
energy along the wall normal (with the effective mass at the contact) exceeds what is left of it
breaks the section: the car keeps the rest of its energy and passes through, and the occupants see
the milder speed change as their crash pulse. A hit that falls short is a rigid collision, and
half its energy stays as damage, so repeated ramming breaks a wall. `ord_obstacles()` lists every
segment for drawing (8 doubles: endpoints, z range, material, broken); `ord_break_events()` returns
how many sections broke since the last call, with 8 doubles each in `ord_break_buffer()` (segment,
material, hit x, y, wall normal x, y, closing speed, fraction of the energy the car kept).
`ord_reset_obstacles()` rebuilds them, as `ord_reset()` does. Debris is the page's job.

## Weather and test areas

`ord_set_weather(rain)` (0 dry .. 1 downpour) wets every surface: asphalt keeps ~75% of its grip
and carries a 1.2 mm water film, cobblestones lose 45% and paint 55%, soil turns to mud, and
puddles 6–20 mm deep form (one candidate per 22 m cell, ~30% of them, on road classes only in the
city). A tyre in a puddle can hydroplane and the water drags at it
([water.md](../docs/architecture/water.md)). `ord_puddles(x, y, radius)` returns how many puddles
lie near a point; `ord_puddle_buffer()` holds 4 doubles each (x, y, half length along x, depth;
the half width along y is 0.6 of the length).

`ord_test_areas()` returns 35 doubles describing the areas south and north of the proving ground:
the moose lane's centre (0), the aquaplaning lane (1..5: y, 8 mm pool x0/x1, 12 mm left-side
puddle x0/x1), the surfaces lane (6..10: y, cobblestones x0/x1, zebra stripes x0/x1) and the stunt
park (11..34: two kicker lanes, their kickers and landing ramps, the half-pipe, and the loop:
x, y, bottom radius, sideways shift over its length, half width, approach heading).

## Stunt park

Two kickers (1.5 m and 3 m: a curved run-in, then straight to the lip, so the car leaves without
a pitch rate) with landing ramps whose front faces are walls; a half-pipe with walls rising to
50° and concrete parapets along its decks; and a loop. The loop is not part of the height field:
the ground the core sees is wrapped (`DemoGround`) so rays near the loop also test a ribbon on the
inside of a general cylinder whose profile is tabulated by integrating its curvature: a 25 m
clothoid lead-in (the load comes on over ~1 s instead of as a step that would bounce the car off
its bump stops), the loop proper with ρ(θ) = R_b + (R_t − R_b)(1 − cos θ)/2 (22 m at the bottom,
9 m at the top, 36 m tall), and a mirror-image lead-out. The ribbon shifts 9 m sideways over its
length so the way in and the way out pass side by side; driven in at `heading`, a car follows it
(a straight line on the unrolled cylinder). `ord_loop_profile()` / `ord_loop_profile_count()`
give the samples (x relative to the loop, z, slope angle, arc length) for drawing.

Near the stunt park the height field is ray-marched in steps of a quarter of the height gap and
bisected (a fixed-point solve diverges on the steep faces). Every car in the demo makes the loop
with lane keeping once it carries the speed (the 911 from ~80 km/h on full throttle, the 4.2 t
S 600 Guard from ~125); `web/smoke_test.mjs` drives it.

## Rewind, shadow car, crash lab

- `ord_snap_push()` saves the whole simulation (car, shadow car, dummies, walls, latched input,
  step remainder) into a ring of 240; `ord_snap_restore(k)` restores the k-th newest and forgets the
  newer ones; `ord_snap_count()`, `ord_snap_clear()`. The step is deterministic, so a snapshot
  plus the same frame times and inputs reproduces the state exactly (checked in the smoke test).
- `ord_shadow_start(carIndex, stock)` starts a second car where the car is, moving as it moves:
  carIndex −1 for the same model, stock 1 to ignore the garage setup. It takes the same inputs
  every step, collides with nothing and leaves the dummies alone. `ord_shadow_state()` has the
  state-buffer layout; `ord_shadow_stop()`.
- `ord_launch(x, y, yaw, speed, lateral)` places the car already moving (a test sled), with fresh
  dummies; `ord_step(n)` runs exactly n fixed steps. `ord_hit_events()` returns the hits harder
  than 1 m/s since the last call, 7 doubles each in `ord_hit_buffer()` (contact x, y, z, wall
  normal x, y, closing speed, kind: 0 wall, 1 post, 2 a breakable wall that broke).
  `ord_crash_structure()` gives the crush zones (front, rear, side natural frequency; their crush
  limits; cabin half width) and `ord_vehicle_info()` the CoM, mass and inertia.
- Memory is a fixed 128 MB (snapshots are ~66 kB each).
