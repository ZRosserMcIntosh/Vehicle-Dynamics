# Architecture overview

Status: v0.2 (core: unsprung mass, multi-point contact, ground prefetch, host integration, LOD tiers; UE plugin written but not yet compiled). This document describes what exists now
(✅), what is designed but not built (🧭), and open questions (❓).

## Layers

```
┌──────────────────────────────────────────────────────────────────────┐
│ Game (Brazil open world)        UI, cameras, audio, AI driving, net  │
├──────────────────────────────────────────────────────────────────────┤
│ 🟡 UE5 adapter plugin  plugin/OpenRoadDynamics (written, not compiled) │
│   UOpenRoadVehicleComponent · Chaos async callback · GroundQuery via  │
│   Chaos scene queries · PhysicalMaterial → SurfaceProperties ·        │
│   force application to Chaos body · replication · debug overlay       │
├──────────────────────────────────────────────────────────────────────┤
│ ✅ ord_core  (C++20, no engine headers, no allocation in step)        │
│   Vehicle (LOD0) ─ tire · suspension · drivetrain solver · aero       │
│   AssistSystem ─ input shaping · ABS · TCS · ESC · transmission       │
│   Telemetry · FixedStepper · GroundQuery interface                    │
│   ✅ GroundPrefetchCache · ChassisMode::External · stepSubstepped     │
│   ✅ LOD1 (Vehicle lod1 fidelity) · LOD2 TrafficSystem · LodManager    │
│   ✅ TelemetryRecorder · debug draw (+SVG) · tire parameter generator  │
│   🧭 state snapshot/restore for netcode                                │
├──────────────────────────────────────────────────────────────────────┤
│ ✅ ord_scenarios + ordbench + ord_tests  (headless, deterministic)    │
└──────────────────────────────────────────────────────────────────────┘
```

Rule: **`src/core` never includes an engine header.** The host supplies:
- a `GroundQuery` (raycast → hit point, normal, `SurfaceProperties`, surface id),
- a fixed `dt`,
- `DriverInput`.

The core returns state, forces and telemetry.

## Conventions

- **ISO 8855 vehicle axes**: X forward, Y left, Z up, right-handed. SI units, radians.
- Unreal is left-handed (X forward, **Y right**, Z up) and uses **centimetres**. The adapter converts at the boundary: `y_ord = -y_ue`, `len_ord = len_ue / 100`. Handedness flips the sign of yaw rates and lateral quantities. The adapter will have a dedicated unit test.
- Double precision everywhere in the core (`ord::Real = double`).

## One fixed step (`Vehicle::step`) ✅

1. **Assists** turn `DriverInput` into `ActuatorCommands`, using sensor values from the previous step.
2. **Contact sampling**: 1, 5, 8 or 9 samples per wheel on a flat-footprint patch, queried through `GroundQuery`. The query is normally the `GroundPrefetchCache`, so it reaches no engine. A procedural surface micro-texture is added where the material declares one.
3. **Suspension**: an implicit per-corner unsprung-mass DOF (tire vertical spring and damper per sample, suspension spring and damper, ARB, bump stops). LOD1 uses a massless wheel instead.
4. **Tire slip and drivetrain**: lateral transient slip, then one implicit drivetrain/wheel/longitudinal-slip solve ([drivetrain-solver.md](drivetrain-solver.md)).
5. **Forces**: combined-slip Magic Formula forces at the patch, tire normal force minus the wheel's relative inertia, aero, and gravity (unless the host applies it) → `ChassisForces`.
6. **Chassis integration**: `ChassisMode::Internal` integrates in the core. `ChassisMode::External` hands the forces to the host (Chaos), which integrates and writes the state back with `setChassisState()`.
7. **Telemetry** frame (full or minimal, per `VehicleFidelity`).

`stepSubstepped(hostDt, k)` runs *k* internal steps per host step and returns impulse-equivalent forces. The world can stay at 60 Hz while the car runs at 240+ Hz ([ADR-0003](../decisions/ground-prefetch.md)).

## Timestep ✅

- Fixed `dt` per vehicle. Rate is decoupled from the world rate through `stepSubstepped`.
- Smooth-road results converge by 120 Hz. With the 5-point patch, sharp impacts (curbs, potholes) are within ~7% at 240 Hz and ~1% at 360 Hz of a 1000 Hz reference.
- **Default: 60 Hz world × 4 substeps (240 Hz) for LOD0**; 360 Hz with 8 samples as a quality option; LOD1 120 Hz; LOD2 30 Hz. See [performance-and-risks.md](performance-and-risks.md).

## Threading 🧭

- One `Vehicle` is single-threaded and owns all its state. Different vehicles share no mutable state, so a pool of vehicles parallelises trivially (task per vehicle, or per batch of LOD1 cars).
- In UE the step runs inside the Chaos async physics callback (`ISimCallbackObject` / async physics tick) on the physics thread. Inputs come in and outputs go out through double-buffered plain structs. **No `UObject` access from the physics thread.** KinetiForge's unsynchronised approach is the specific failure mode to avoid ([kinetiforge-analysis.md](../research/kinetiforge-analysis.md) §9).
- ✅ **No scene queries on the physics thread.** The game thread rebuilds a `GroundPrefetchCache` per frame and hands it over immutable (double-buffered). The physics thread only reads it ([ADR-0003](../decisions/ground-prefetch.md)).

## Large worlds 🧭

- UE5 Large World Coordinates use doubles for positions. The core also uses doubles, so absolute positions across a ~1,000 km map (São Paulo ↔ Florianópolis) keep sub-millimetre precision (2^-52 × 10^6 m ≈ 2×10^-10 m).
- Even so, the core only ever needs *local* geometry (hardpoints, contact points relative to the CoM). The adapter may rebase each vehicle to a local origin per step to keep floats safe in any float-precision path (e.g. Chaos internals, if they use single precision).
- Physics is independent of streaming: the core needs only `GroundQuery`. If a ground tile isn't loaded, the adapter must supply a fallback (last known plane or a low-res heightfield) and flag it in telemetry. Wheels never fall through the world because a tile is late.

## Determinism and networking 🧭

- ✅ The core is deterministic for the same build, inputs and `dt` (test: `vehicle: simulation is bit-for-bit deterministic`). Cross-platform bit-equality is not promised; floating-point flags, compilers and `libm` differ.
- 🧭 Planned model: **server-authoritative with client prediction** for the local player's car. `VehicleState` (body + wheel spin + slip states + drivetrain + assist state) will be serialisable for snapshot/restore and replay. Remote cars run LOD1 with interpolation and extrapolation from snapshots.

## Data-driven vehicles ✅/🧭

- ✅ `VehicleDefinition` is plain data with `validate()`, which rejects non-finite or out-of-range values (vehicle files are untrusted input, see [SECURITY.md](../../SECURITY.md)).
- 🧭 JSON/TOML vehicle files with schema validation. In UE, a `UDataAsset` wrapper converts to `VehicleDefinition`.

## Module status

| Module | v0.2 | Next |
|---|---|---|
| Tire | Reduced MF + load sensitivity + combined slip + relaxation; parameter generator from tire size/category | Camber, MF 6.x import, thermal/pressure/wear, wet model |
| Contact | Flat-footprint multi-point patch (1/5/8/9), procedural surface texture | Sweep fallback for large obstacles |
| Suspension | Unsprung-mass DOF (implicit), spring/damper/ARB/bump stops; massless mode for LOD1; double-wishbone and MacPherson linkage kinematics (camber/toe curves, roll centres, anti-dive, motion ratio) | Solid axles, bushing compliance |
| Drivetrain | Engine curve, governor, limiter, clutch, gearbox, open/locked/LSD diffs, AWD split | Turbo, EV/hybrid, torque converter, viscous couplings |
| Aero | Drag, axle downforce | Ride-height sensitivity, active aero, slipstream |
| Assists | Steering shaping, ABS, TCS, ESC, countersteer, auto/manual shifting | Brake assist, launch, FFB, speed estimator |
| Ground | `GroundPrefetchCache` (adaptive strips, extrapolation on miss), `CountingGroundQuery` | Strip reuse across frames, multi-level roads |
| Host integration | `ChassisMode::External`, `stepSubstepped`, `host_frame.hpp` | Chaos validation in UE |
| LOD | LOD1 (`VehicleFidelity::lod1`), LOD2 `TrafficSystem` (IDM), `LodManager`, `LaneFollowDriver` | Lane changes, junctions, collisions promoting cars, parallel stepping |
| Tooling | `TelemetryRecorder`, debug draw + `SvgDebugSink`, `ordbench` | CI report diff |
| UE adapter | Written (`plugin/OpenRoadDynamics`), **not compiled** | Compile on UE 5.4–5.6, parity validation |
