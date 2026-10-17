# Proving ground: a drivable car in an empty UE 5.4 project

> **Not compiled, not run.** The demo pawn (`AOpenRoadDemoVehicle`), the level generator (`Content/Python/build_proving_ground.py`) and the automation tests were written against the UE 5.4 API without an engine. The parts that run without Unreal are checked headless: `Extras/HeadlessCheck/run.sh` covers the physics-thread wrapper, and a mocked-`unreal` smoke run covers the generator's own logic. Expect to fix small API mismatches the first time; the [troubleshooting](#troubleshooting) table lists the likely ones.

What you get:
- A primitive RWD sports car: a box chassis, cube body and cylinder wheels animated from the simulation.
- Keyboard and gamepad driving with an automatic gearbox.
- Runtime presets FUN/SPORT/SIMULATION/DRIFT and ABS/TCS/ESC toggles.
- Suspension that reacts to curbs and bumps, and Chaos collisions with cones and the ground.
- Telemetry CSV capture, an on-screen readout, and two scripted parity manoeuvres.

## 1. Project and plugin

1. Create a project: Epic Games Launcher → Unreal Engine 5.4 (or later) → **Games → Blank**, select **C++**, starter content off. A C++ toolchain is needed, as for any C++ project: Visual Studio 2022 on Windows, Xcode on macOS, or the bundled clang on Linux.
2. From this repository, package the core into the plugin:
   ```sh
   python3 tools/package_plugin.py
   python3 tools/package_plugin.py --check    # "ok: ... packaged core is up to date"
   ```
3. Copy `plugin/OpenRoadDynamics` to `<Project>/Plugins/OpenRoadDynamics`. `Extras/` is optional and is not built by UBT.

## 2. Plugins and project settings

Close the editor and edit `<Project>.uproject`. Add the following to `"Plugins"`, or enable the same plugins in Edit → Plugins:

```json
"Plugins": [
  { "Name": "OpenRoadDynamics", "Enabled": true },
  { "Name": "EnhancedInput", "Enabled": true },
  { "Name": "PythonScriptPlugin", "Enabled": true }
]
```

`PythonScriptPlugin` is "Python Editor Script Plugin" in the UI. It is editor-only and only the level generator needs it.

Add to `<Project>/Config/DefaultEngine.ini`. This is configuration **A** of the plugin README: a 240 Hz async fixed step, one vehicle substep, and the most faithful parity.

```ini
[/Script/Engine.PhysicsSettings]
bTickPhysicsAsync=True
AsyncFixedTimeStepSize=0.004166667
bSubstepping=False
```

UI equivalent: Project Settings → Engine → Physics → Framerate → **Tick Physics Async** on, **Async Fixed Time Step Size** 0.004166667, Substepping off. Configuration B (`AsyncFixedTimeStepSize=0.016666667`, 4 vehicle substeps) also works. Its trajectories drift about 2% from A over a hard steering run; see `docs/AUDIT.md` item 17.

Enhanced Input is the default in new 5.4 projects. If your `Config/DefaultInput.ini` says otherwise, set:

```ini
[/Script/Engine.InputSettings]
DefaultPlayerInputClass=/Script/EnhancedInput.EnhancedPlayerInput
DefaultInputComponentClass=/Script/EnhancedInput.EnhancedInputComponent
```

## 3. Build

1. Right-click `<Project>.uproject` → **Generate Visual Studio project files** (on Linux: `GenerateProjectFiles.sh`).
2. Build **Development Editor** for your platform, or double-click the `.uproject` and accept the rebuild prompt.
3. Errors in `OrdChaosBridge.cpp` are expected first-build API mismatches. See [troubleshooting](#troubleshooting).

## 4. Generate the level

1. Open the project. Then open **Window → Output Log**, and switch the command box mode (bottom left) to **Python**.
2. Run:
   ```python
   import build_proving_ground; build_proving_ground.build()
   ```
   Alternatively, in **Cmd** mode run `py "<Project>/Plugins/OpenRoadDynamics/Content/Python/build_proving_ground.py"`, or use **Tools → Execute Python Script…** and pick that file.
3. The script:
   - creates `/Game/OpenRoad/ProvingGround` and opens it;
   - creates the physical materials `PM_OpenRoad_Wet` and `PM_OpenRoad_Curb`, and the surface table `DA_OpenRoad_ProvingGroundSurfaces` (wet: friction scale 0.75, wet speed falloff 0.004; curb: roughness 0.6);
   - places about 260 basic-shape actors, a PlayerStart and an auto-possessed `OpenRoadDemoVehicle`;
   - sets World Settings → GameMode Override = `OpenRoadDemoGameMode` and Global Gravity Z = −980.665 (the core's g);
   - saves the level.

   To rebuild, delete `/Game/OpenRoad/ProvingGround` first.

Layout (UE cm; X forward, start line at X = 0):

| Section | Where | Geometry |
|---|---|---|
| Spawn | X = −30 m, Y = 0 | Heading +X |
| Acceleration/braking strip | X 0 → 1200 m, Y = 0 | Boards every 100 m, start and finish lines |
| Wet braking lane | X 0 → 400 m, Y = +40 m | 12 m wide, 2 mm proud of the ground |
| Slalom | X 100 → 262 m, Y = +100 m | 10 physics cones at 18 m spacing; cones ignore the Visibility channel, so the tires do not ride over them |
| Curb lane | Y = +200 m | 8 cm and 12 cm curbs (0.5 m long, 0.6 m wide, 2 cm half-height chamfer steps) under the left wheels at X = 100 m and 150 m, and under the right wheels at 250 m; 8 cm × 0.5 m sin² speed bump at 200 m; rumble (8 cm every 1 m) at 300–320 m; long kerbs, 0.5 m wide, 8 cm at 400–440 m and 12 cm at 460–500 m, 2.5 m left of the lane centre |
| Skidpad | Centre X = 300 m, Y = −150 m | R = 40 m path markers, inner/outer rings at ±3 m |

The geometry mirrors `src/scenarios`: curb 8 cm × 0.5 m, bump 8 cm × 0.5 m sin², skidpad R40, wet = the `ordbench` wet case.

## 5. Play

Press **Play** (Selected Viewport), then click into the viewport.

| Action | Keyboard | Gamepad |
|---|---|---|
| Throttle | W / Up | Right trigger |
| Brake | S / Down | Left trigger |
| Steer | A D / Left Right | Left stick X |
| Handbrake | Space | B (face right) |
| Preset FUN / SPORT / SIMULATION / DRIFT | 1 / 2 / 3 / 4 | D-pad left / up / right / — |
| ABS on/off | B | X (face left) |
| TCS off → sport → full | T | Y (face top) |
| ESC off → sport → full | E | D-pad down |
| Shift up / down (manual mode) | R / F | Right / left shoulder |
| Automatic ↔ manual gearbox | M | — |
| Debug draw (contacts, suspension, tire forces) | F1 or G | View |
| Telemetry start/stop | F2 or C | Menu |
| Scripted 0-100 km/h, then full brake | Z | — |
| Scripted step steer at 80 km/h | X | — |

The gearbox stays automatic in every preset (the SIMULATION preset would otherwise be manual). Toggle with M. In PIE the editor may take F1/F2 as viewport hotkeys; use G and C instead.

The on-screen readout (Development builds) shows:
- speed, gear, rpm and driver inputs;
- preset, assists and their activity;
- per-wheel load, slip ratio, slip angle, F/Fz (friction used) and compression;
- ground-cache health (horizon, cache age, misses);
- coupling diagnostics: host-sync error, velocity-reference check, CoM mismatch, mass, principal-frame angle.

**Telemetry** goes to `<Project>/Saved/OpenRoad/OpenRoadDemoVehicle_<yyyyMMdd_HHmmss>.csv`, one row per physics step, with the same columns as `ordbench --csv-dir`.
- Positions and lateral quantities in the CSV are in the core's ISO frame: Y left, metres.
- The ring buffer keeps the last `TelemetryCaptureSeconds` (30 s) of the capture.

## 6. Parity validation against the headless benchmarks

Reference: `./build/ordbench` (Markdown report), or `./build/ordbench --csv-dir out/`, which writes `accel_sports_sim.csv`, `brake_sports_sim.csv` and `skidpad_sports_sim.csv` for the SIMULATION preset.

The demo car uses the asset defaults, which match `ord::vehicles::referenceSportsRwd()`, with the origin moved to the box centre. Use configuration A, press **3** (SIMULATION), and drive on the strip (dry).

| Benchmark | How in UE | Record from the CSV | Target vs headless |
|---|---|---|---|
| 0–100 km/h | Press **Z** on the strip. Telemetry starts from rest, at full throttle with auto shift. | First `time` where `speed_kmh` ≥ 100, minus the start time; distance from `x` | < 1% |
| 100–0 km/h | Continues automatically after 100 km/h (full brake). SIMULATION has ABS off, like the benchmark table; press B before Z for the ABS-on row of the `ordbench` wet/dry table. | `time` and `x` from the brake application to `speed_kmh` < 0.5; peak `ax` | < 1% (distance) |
| Skidpad R40 | Drive the R = 40 m ring counter-clockwise; raise speed slowly until the car cannot hold the line. Record with F2. | Steady `ay / 9.80665` (max lateral g), `steer_angle`, `sideslip_rad` | < 1–2% (manual driving) |
| Step steer 80 km/h | Press **X** on the strip. It holds 80 km/h for 2 s, then steers 0.08 to the left for 4 s. | Yaw-rate 10–90% rise time, overshoot, steady `yaw_rate` | < 1% |

Also record, from the readout or the Output Log line "coupling check":
- **host-sync error**, expected < 0.1 cm at 240 Hz;
- **V-check CoM vs origin**: CoM ≈ 0 means Chaos V is the CoM velocity;
- **CoM off** < 1 cm, **mass** = 1450 kg, **rotMass** ≈ 0°;
- **ground misses**: 0 on the strip, and only during hitches.

If a metric is off by more than 1%, check those values first: they cover README unverified items 5, 6, 8 and 10.

## 7. Automation tests (unverified)

Session Frontend → Automation → filter **OpenRoad**. Or headless:

```
UnrealEditor-Cmd <Project>.uproject -ExecCmds="Automation RunTests OpenRoad; Quit" -unattended -nullrhi
```

The tests check frame/unit conversions, the default definition → core mapping and the assist mapping. They need no world.

## Troubleshooting

Each symptom points to the plugin README list of unverified assumptions (U#).

| Symptom | Likely cause (U#) | Fix |
|---|---|---|
| Compile error on `TSimCallbackObject<…, ESimCallbackOptions::Presimulate>` | Template signature (U1) | Drop the third argument if your version defaults to Presimulate; check `Chaos/SimCallbackObject.h` |
| Compile error on `CreateAndRegisterSimCallbackObject_External` | Template parameters (U2) | Match the declaration in `PhysicsSolverBase.h`; keep the call in `OrdChaosBridge.cpp` |
| Deprecation warnings or errors on `X()`/`R()`/`V()`/`W()` | Accessor renames (U5) | Build with `ORD_CHAOS_USE_GET_ACCESSORS=1` (add `PublicDefinitions.Add("ORD_CHAOS_USE_GET_ACCESSORS=1")` to `OpenRoadDynamics.Build.cs`) |
| Compile error on `RotationOfMass()` or `AddForce(F, false)` | Handle API (U6, U8) | Remove the diagnostic line, or the second argument |
| "VEHICLE INACTIVE" in the readout | Body not simulating, no definition, or no Chaos solver | Read the `LogOpenRoad` lines in the Output Log |
| Car sinks, flies away or jitters; host-sync error grows to cm | Force/torque frame, units or per-step clearing (U8); CoM semantics (U5) | Compare V-check CoM vs origin; apply a known force on an isolated body and check Δv = F·dt/m per step |
| "Chaos mass properties differ" warning | COMNudge or inertia not applied (U10), or asymmetric collision (U6) | Check the BeginPlay "body:" log line; `rotMass` should be ≈ 0 |
| Wheels drop through curbs, or ground misses grow | Ground traces blocked or ignored (Visibility channel), or long hitches (§11 of `docs/AUDIT.md`) | Make sure curbs and road block Visibility, and that cones/props ignore it; check horizon vs cache age in the readout |
| No input response | Not an Enhanced Input component (see §2), or the viewport has no focus | Log line "input component is not a UEnhancedInputComponent"; click into the viewport |
| Two cars at the start | Auto-possess failed and the game mode spawned a second pawn | Delete the placed pawn; the game mode spawns one at the PlayerStart |
| Python: `AttributeError: module 'unreal' has no attribute 'OpenRoadDemoVehicle'` | Plugin module not loaded / not built | Build the C++ project first; check Edit → Plugins |
| Python: errors on a property name (`world_size`, `real_time_capture`, …) | Editor Python names differ between versions | Cosmetic steps only log a warning; for others, check the class in the Python API docs (`help(unreal.<Class>)`) |
| Crash when ending PIE or destroying the car | Callback/proxy lifetime (U2, U12) | Report with a callstack; run with `-stompmalloc`; see `docs/AUDIT.md` items 1–2 |
| `dt changed` warning | Async physics off (U11) | §2 project settings |
