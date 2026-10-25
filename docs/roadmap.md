# Engineering roadmap

Prioritised by "what unlocks measurable driving quality soonest". Each item has acceptance criteria tied to benchmarks in [benchmark-plan.md](benchmarks/benchmark-plan.md). The GitHub issues mirror this list.

## M0 — Foundation (this session) ✅

- [x] Research: Forza public physics, Forza telemetry, KinetiForge audit, alternatives & licensing
- [x] ADRs: physics foundation; one simulation, many experiences
- [x] Engine-independent core v0.1: MF tire + transient slip, implicit drivetrain solver, suspension, aero, assists, presets, telemetry
- [x] Scenario library, 35 tests, `ordbench` report, CI

## v0.2 — Priorities 1–5 and lightness work (this session)

- [x] Unsprung mass DOF (M2.1), multi-point contact patch + curb/pothole/cobblestone benchmarks (M2.2), fixed-rate decision (M2.5)
- [x] Ground prefetch cache, vehicle-local substepping, procedural surface texture (ADR-0003)
- [x] Host integration mode (`ChassisMode::External`) + UE frame conversions, parity tests
- [x] LOD1 + LOD2 traffic + `LodManager`, transition and scale benchmarks (M5.1)
- [x] Telemetry recorder, engine-agnostic debug draw + SVG
- [x] Tire parameter generator (licensing risk)
- [~] UE5 plugin: source written (M1.1–M1.5), **not compiled yet**, see `plugin/OpenRoadDynamics/README.md`

## v0.3 — Main-branch integration and engineering hardening (this session)

- [x] Foundation merged into `main` (PR #11); CI on Linux GCC/Clang, sanitizers, macOS, Windows
- [x] Numerical audit of the 60 Hz × substep host coupling (like-for-like comparison, sub-mm host pose error)
- [x] Geometric envelope probes for sharp edges (curb convergence across rates) (PR #12)
- [x] Robustness: range validation, static-support check, runtime fault guard, ABS-off hidden-aid fix, linear steering-mapping crossfade on preset switch, race-downforce and hypercar reference cars, fuzz/long-duration/determinism tests, golden baseline (PR #13)
- [x] Zero steady-state allocation proven by test; honest headless-benchmark scope documented
- [x] Versioning and release policy ([RELEASING.md](RELEASING.md))
- [~] Plugin static audit and proving-ground preparation: **still not compiled against Unreal**

## After v0.3 — Drift, crashes, two-wheelers and the tuning garage

- [x] Drift assist: steering sets the angle, throttle holds it, centred steering straightens without a swing back; donuts at full lock from a crawl; auto clutch opens with the handbrake ([driving-presets.md](architecture/driving-presets.md))
- [x] Browser build (`web/`): WebAssembly bindings and a CI smoke test for the online proving ground
- [x] Rideable bicycle, e-scooter and sport motorcycle on the single-track model (M7)
- [x] Crash survivability: crash pulses, belted/airbag occupants, chest g, HIC15, injury risk ([crash-survivability.md](architecture/crash-survivability.md))
- [x] Tuning garage: alignment and camber geometry, two-stage dampers, tyre compounds with pressure and temperature, brake balance, gears, aero with floor and wing stall, wind and altitude, JSON setups ([tuning-garage.md](architecture/tuning-garage.md))
- [ ] Test bench: scripted manoeuvres, side-by-side setup comparison, parameter sweeps
- [x] Suspension kinematics: double-wishbone and MacPherson linkages from pivot geometry, camber/toe curves, roll centres and jacking, anti-dive/anti-squat, motion ratio ([suspension-kinematics.md](architecture/suspension-kinematics.md))
- [ ] Solid axles and bushing compliance on top of the linkage kinematics

## M1 — Unreal Engine 5 adapter (next: compile and validate)

1. **Plugin skeleton** `plugin/OpenRoadDynamics` (Runtime + Editor modules). `UOpenRoadVehicleComponent` owns an `ord::Vehicle`. Build on UE 5.4+. *Accept:* compiles on Win64 with UE 5.4/5.5/5.6. Car drives on the proving-ground level.
2. **Chaos async integration**: core steps inside the async physics callback (`stepSubstepped`, 60 Hz world × 4). Ground comes from a per-frame `GroundPrefetchCache` built on the game thread (no physics-thread scene queries). Forces and torque go to the Chaos body (`ChassisMode::External`). *Accept:* headless-vs-UE benchmark divergence < 1% on B1/B2/B4/B5. No `UObject` access on the physics thread.
3. **Coordinate/unit adapter** (cm, left-handed): conversion maths done and unit-tested in `ord/host_frame.hpp`; UE wrappers pending compile.
4. **Physical Material → SurfaceProperties** data asset.
5. **Debug overlay**: per-wheel forces, slip, loads, assist activity. Telemetry CSV capture.
6. **Proving-ground level** mirroring `TestTrackGround`.
7. **Chaos Vehicles A/B baseline** car in the same level.

## M2 — Contact & suspension fidelity

1. **Unsprung mass DOF** per corner (wheel hop, harshness). *Accept:* B20, B22 natural frequencies within 3% of analytic.
2. **Multi-point contact patch** (5 → 8 samples, perimeter + centre, weighted plane fit). *Accept:* B19 curb load spikes converge with sample count. Rate × samples matrix published.
3. **Kinematic suspension** ✅: double-wishbone and MacPherson linkages solved from their own geometry (no third-party solver needed); solid axle still open. *Accepted:* 3-D solve matches an independent 2-D four-bar to 1e-5, roll centre matches the graphical construction.
4. Anti-dive / anti-squat / roll-centre load paths ✅ (they fall out of the linkage path; tested).
5. **Fixed-rate decision**: 120/240/360 Hz on rough-road benchmarks → default for LOD0.

## M3 — Tire depth

1. Camber thrust (done; camber peak-grip model in the garage); full MF 6.x coefficient support (`.tir`-style import, our own parameter sets only).
2. Wet model: water-film depth input, speed-dependent μ, hydroplaning onset (B21). *Done (first step):* standing water with Horne's hydroplaning speed, tread depth, displacement drag and spin-down ([water.md](architecture/water.md)); a measured partial-hydroplaning curve and tread-pattern effects remain.
3. Loose surfaces (gravel/dirt/grass curve shapes, rolling resistance).
4. Thermal (surface/carcass), pressure and wear. Off by default outside SIMULATION. No refuelling gameplay. *Done (first step):* compounds, pressure and a one-node tread temperature in the tuning garage ([tuning-garage.md](architecture/tuning-garage.md)); carcass node, wear and pressure rise with heat remain.
5. Self-aligning torque (MF Mz) → **force feedback** pipeline.

## M4 — Drivetrain & powertrains

Turbo boost/lag, supercharger, EV motor envelopes, hybrid, torque converter, viscous couplings, torque vectoring, engine stall/start, manual-clutch realism.

## M5 — Scale & multiplayer

1. LOD1 model + LOD2 kinematic traffic + transitions (B24, B25).
2. State snapshot/restore. Server-authoritative prediction and reconciliation prototype.
3. Parallel vehicle stepping, SIMD-friendly batch for LOD2.

## M6 — Validation & experience

1. Real-vehicle comparison with an open parameter set (e.g. Chrono BMW E90 data).
2. Forza Data Out capture tool (our own recordings) + comparison report (B4 lateral-g curve, B2).
3. Playtest programme; FUN-mode tuning; drift (B17).
4. Assist refinements: brake assist, launch control, reference-speed estimator.

## M7 — Two-wheelers

Bicycles, e-scooters/e-bikes and motorcycles at car-level fidelity. Plan, results and limitations: [two-wheelers.md](architecture/two-wheelers.md).

1. [x] **Single-track multibody prototype** (`ord/single_track.hpp`): four rigid bodies, revolute joints, Whipple rolling contact + basic tyre mode. *Accept:* weave/capsize speeds of the Meijaard et al. (2007) benchmark bicycle within 1e-3 m/s. Achieved: ~5e-12 m/s, matrices and eigenvalues to ~1e-12.
2. [x] Camber thrust/twisting torque (shared with M3.1; cars pass camber 0 for now) and toroidal tyre contact on arbitrary ground. *Accept:* Tyre-mode benchmark bicycle self-stable in a band near the benchmark's. Achieved: ~4.8–6.5 m/s (benchmark 4.29–6.02).
3. Virtual rider (target lean → steer torque), FUN/SPORT/SIMULATION presets acting only through actuators, low-speed balance and foot-down. Bicycle reference vehicles. *(Bicycle total ~2.5 months.)* **Partly done:** `TwoWheeler` with Lean/Direct rider, balance moment, brakes, ABS/TCS/wheelie control and three ridable vehicles (bicycle, e-scooter, sport motorcycle); still to do: leg contact for foot-down, rider body lean, presets, pedalling model.
4. E-scooter and e-bike: hub motor, battery sag, pedal assist, human pedalling model with derailleur gears. *(+3–4 weeks.)*
5. Motorcycles: fork and swingarm (chain anti-squat), CVT and manual powertrains, wheelie/stoppie, low-side/high-side detection, weave/wobble validation (Sharp 1971, Cossalter). *(+2–3 months.)*
6. UE: rider skeleton/animation, ragdoll handoff, camera. *(+1–2+ months.)*

## Next five engineering priorities

0. Garage test bench (scripted manoeuvres, setup comparison, sweeps)
1. Compile the UE5 plugin on UE 5.4–5.6 and resolve the API assumptions recorded in its audit (M1.1–M1.3)
2. Build the proving-ground level from the documented steps and drive the first car in Unreal; record headless-vs-Chaos parity (< 1% target on B1/B2/B4/B5) and real ray costs (M1.2, M1.6)
3. Tire cleat-test calibration to settle the 5- vs 8-sample contact layout sensitivity (~30% on sharp curbs) and add continuous effective-road-profile enveloping (M2.2)
4. ~~Wet model: water-film depth, speed-dependent grip, aquaplaning (M3.2)~~ first step done ([water.md](architecture/water.md))
5. Solid axles, bushing compliance and tilted contact samples on top of the linkage kinematics (M2.3)

## Damage

Done in [architecture/damage.md](architecture/damage.md): (b) mechanical damage and (c) a node/beam
crash structure (XPBD beams with plastic yield, hardening and tearing) solved with the rigid body,
which replaces the faked crush when it is on. Surveyed in
[research/vehicle-damage-survey.md](research/vehicle-damage-survey.md).
Next: car-to-car contact (two structures), detachable panels and bumpers as their own bodies,
glass in the greenhouse, a finer lattice per car from its mesh.
