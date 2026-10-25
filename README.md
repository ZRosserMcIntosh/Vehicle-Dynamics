# OpenRoad Dynamics

**An open-source vehicle physics and driving system for Unreal Engine 5, built on an engine-independent C++ core.**

One physically grounded simulation. Configurable assists make it accessible,
and turning them off makes it unforgiving. The aim: Forza Horizon-level
enjoyment on a gamepad and Forza Motorsport-level consequences with every aid
off, in a giant open world.

> **Status: v0.3 (October 2026).** The engine-independent core builds and
> passes 171 automated tests in CI on Linux (GCC, Clang, ASan/UBSan), macOS
> (Apple Clang), Windows (MSVC) and WebAssembly. It includes unsprung mass, a
> multi-point tire contact patch with geometric envelope probes for sharp
> edges, a per-frame ground-query cache, host (Chaos) integration with
> vehicle-local substepping, three fidelity tiers (player / nearby AI /
> traffic), parameter validation plus a runtime fault guard, a
> zero-allocation steady state (tested), and a golden benchmark baseline that
> CI checks. On top of that: a **tuning garage** (alignment, two-stage
> dampers, tyre compounds with pressure and temperature, aerodynamics with
> floor and wing stall), a **drift assist** that also lets you spin donuts,
> **crash survivability** with crash-test dummies, and rideable
> **two-wheelers**.
>
> **Try it:** the same core, compiled to WebAssembly, drives the
> [browser proving ground](https://zrossermcintosh.com/sao-paulo/driving/)
> (garage on <kbd>U</kbd>); [how it works](https://zrossermcintosh.com/sao-paulo/physics/).
>
> **The Unreal Engine 5 plugin source is written but has not been compiled
> yet**, so no car has been driven in Unreal. There is no Unreal Engine in
> the development environment. Treat the plugin as a draft until it builds on
> UE 5.4+ ([plugin audit](plugin/OpenRoadDynamics/README.md)).
> Nothing here is affiliated with or endorsed by Microsoft, Turn 10 Studios,
> Playground Games or Epic Games. "Forza" is used only to describe publicly
> documented design goals; we don't claim equivalence with any game.

The first user is a larger open-world game set in southeastern and southern
Brazil (São Paulo, Rio de Janeiro, Curitiba, Florianópolis and the highways
connecting them). The system is designed as a reusable library and plugin for
any developer.

## Principles

1. **One simulation, many experiences.** FUN, SPORT, SIMULATION and DRIFT are bundles of individually configurable aids. They never change tire grip or any other physics ([ADR-0002](docs/decisions/one-simulation-many-experiences.md)).
2. **Physics you can measure.** Every handling claim maps to a reproducible benchmark ([plan](docs/benchmarks/benchmark-plan.md), [current results](docs/benchmarks/baseline-results.md)).
3. **Engine-independent core, thin engine adapter.** `src/core` has no engine headers, is deterministic, and doesn't allocate in the step ([ADR-0001](docs/decisions/physics-foundation.md)).
4. **Clean provenance.** MIT, implemented from published mathematics. No engine source, no game data, no leaked material ([THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)).

## What exists today

| Area | Implemented | Verified by |
|---|---|---|
| Tire | Reduced Pacejka Magic Formula: load sensitivity, combined slip, relaxation-length transient slip, standstill stiffness, **camber thrust** and a camber peak-grip model (best ~2° into the turn), wet grip, **hydroplaning in standing water** (Horne's NASA rule from the inflation pressure, the tread depth, displacement drag, spin-down; [details](docs/architecture/water.md)), loose-surface grip, a one-node **tread temperature** with a grip window, inflation pressure. **Parameter generator** from public tire size/category | `tests/test_tire.cpp`, `tests/test_garage.cpp`, `tests/test_water.cpp` |
| Contact & suspension | **Multi-point contact patch** (1/5/8/9 samples), **implicit unsprung-mass DOF** per corner, springs, **two-stage (slow/fast) dampers**, ARB, bump stops, wheel camber from static camber, camber gain, caster and body roll, **procedural surface micro-texture** for paving | `tests/test_contact.cpp` (frequencies match the analytic quarter car to <0.1%) |
| Suspension kinematics | **Double-wishbone and MacPherson linkages** solved from their pivot geometry at construction: camber and toe curves (bump steer, roll steer), the wheel's path, **roll centres and jacking**, **anti-dive / anti-squat**, the spring's **motion ratio**; the ride height moves the operating point along the curves. The Stella and the real cars use them; the reference cars keep sliders so the baseline stands ([details](docs/architecture/suspension-kinematics.md)) | `tests/test_kinematics.cpp` (3-D solve vs. an independent 2-D four-bar to 1e-5; roll centre vs. the graphical construction; a +95 mm roll centre cuts roll 16 %; anti-dive cuts dive 37 % at the same stopping distance) |
| Drivetrain | Engine curve, governor, limiter, clutch, gearbox, open/locked/LSD diffs, AWD split, all solved implicitly with stick/slip friction | `tests/test_drivetrain.cpp` |
| Assists | Speed-sensitive gamepad steering (full countersteer reachable in a slide), ABS, brake-based TCS, ESC, countersteer aid, **drift assist** (steering sets the drift angle and throttle holds it; countersteer and throttle trim only; full lock and throttle from a crawl spins **donuts**), auto/manual shifting, an auto clutch that opens with the handbrake like a drifter's foot. Real actuators only; every intervention reported ([presets and drift assist](docs/architecture/driving-presets.md)) | `tests/test_assists.cpp` |
| Lightness | **Ground prefetch cache** (per-frame adaptive strips; no scene queries from substeps or the physics thread), **vehicle-local substepping** inside a 60 Hz world, **host integration mode** for Chaos | `tests/test_contact.cpp`, `tests/test_host.cpp` |
| Many vehicles | LOD1 nearby AI (`Vehicle` at low fidelity + lane-following driver), LOD2 kinematic traffic (IDM), `LodManager` with seamless promotion | `tests/test_lod.cpp` |
| Robustness | Range validation of every definition field (incl. static-support and unsprung-mass sanity), `faulted()` runtime guard against non-finite state, fuzzed definitions, reversals, zero speed/reverse, contact loss, long-duration drift, determinism | `tests/test_robustness.cpp`, `tests/test_alloc.cpp` |
| Vehicle range | FWD hatch, RWD sports, AWD saloon, 750 kg high-downforce race car (~1.7 g at 100 km/h → ~2.5 g at 250 km/h), ~476 km/h AWD hypercar, the Rosser Stella V8 (9,000 rpm, 7-speed DCT, 0–100 km/h in 3.8 s) | `tests/test_robustness.cpp` |
| Real-car approximations | Porsche 911 Carrera S and 4S (992.2), Mercedes-AMG S 63 4MATIC+ and S 600 Guard (W222, VR9 armoured, 4.2 t), a 2026-regulation Formula 1 car, from published specifications (estimates marked), with electronic top-speed limiters ([details and limits](docs/architecture/production-cars.md)) | `tests/test_production.cpp` (0–100 km/h and top speed against the published figures) |
| Regression guard | 13 headline metrics in `benchmarks/baseline.csv` with per-metric tolerances; changing them needs an explicit, reviewed regeneration | `tests/test_baseline.cpp` |
| Tooling | Telemetry ring buffer + CSV, engine-agnostic debug draw (+SVG), `ordbench` report | `tools/ordbench` |
| Rollover | A body collision hull (box, greenhouse, tyre shoulders) contacts the ground with friction once the car tilts or leaves the ground, so cars tip, trip over kerbs, land on their side or roof and slide; on steep ground (bankings, walls, a loop) the tyres keep carrying the car ([details](docs/architecture/rollover.md)) | `tests/test_rollover.cpp` (rests on its roof; tipping angles by car; kerb trip vs. flat ground; never touches in normal driving) |
| Damage | Off by default. A **node/beam crash structure** (~110 nodes, elastic-plastic beams that yield, harden, bottom out and tear) solved with XPBD with the rigid body in the solve: the crush, the crash pulse and the intrusion come out of the structure, calibrated to the car's crash structure (56 km/h full-width: 0.51 m, 27 g, 92 ms). Hosts pass walls and posts through `ObstacleQuery`. **Mechanical damage** from it: suspension alignment, wheels torn off, pinch flats and cuts, steering offset and lost lock, radiator → overheating → seized engine, drag and wing loss; repair ([details](docs/architecture/damage.md)) | `tests/test_damage.cpp` (crush/pulse/energy by speed; pole; roof; symmetry; pull after an offset hit; overheating; flat tyre; three wheels; repair; determinism; fuzz), `tests/test_alloc.cpp` |
| Crash survivability | Each impact's delta-V becomes a crumple-zone crash pulse; a torso + head occupant per seat (belt, airbags, door, wheel/dash) gives chest 3 ms g, HIC15 and injury risk from NHTSA's published curves, plus the Joksch delta-V statistic. A game estimate, not a validated crash simulation ([details](docs/architecture/crash-survivability.md)) | `tests/test_crash.cpp` (56 km/h belted frontal in the NCAP range; orderings for speed, pulse, belts, airbags, side) |
| Tuning garage | `VehicleSetup` in setup-sheet units: camber / toe / caster / camber gain (ground camber from geometry and body roll), springs, ride height, two-stage dampers, ARBs, **tyre compounds** (off-road … slick, rain) with **pressure and tread temperature**, brake balance, diff and gears, **aero** (ride-height-sensitive floor with stall, rear wing with a real stall, wind and yaw, altitude via the ISA, ported from Aviation Dynamics), JSON save/load ([details](docs/architecture/tuning-garage.md)) | `tests/test_garage.cpp` (compound ladders dry/wet/gravel, camber vs braking, brake-balance spin, wing stall, floor stall, crosswind) |
| Two-wheelers (prototype) | **Benchmark bicycle**: nonlinear 4-body single-track multibody model (Whipple rolling contact or tyres with **camber thrust and a rounded crown** on any ground). **Rideable** bicycle, e-scooter and sport motorcycle: virtual rider (counter-steer emerges), brakes, ABS, traction and wheelie control. No suspension or gearbox yet ([plan and limits](docs/architecture/two-wheelers.md)) | `tests/test_single_track.cpp` (Meijaard et al. 2007 weave/capsize speeds reproduced to ~1e-11 m/s), `tests/test_two_wheeler.cpp` |
| Browser build | `web/`: C ABI over the core compiled to WebAssembly (Emscripten or wasi-sdk), with the proving ground, city heightfield and colliders, crash dummies, two-wheelers and garage setups (JSON) exposed to JavaScript; weather and puddles, a moose lane, an aquaplaning lane, cobblestones and paint, a stunt park with a clothoid loop, exact rewind snapshots, a shadow car on the same inputs and crash-lab launches ([web/README.md](web/README.md)) | `web/smoke_test.mjs` (CI) |
| Unreal Engine 5 | Plugin `plugin/OpenRoadDynamics`: component, Chaos sim callback, game-thread ground prefetch, debug overlay. **Not compiled yet** | — |

Selected results for the fictional 300 kW RWD reference car ([full report](docs/benchmarks/baseline-results.md)):

| | FUN | SPORT | SIMULATION |
|---|---|---|---|
| 0–100 km/h | 5.1 s | 4.6 s | 5.6 s (wheelspin, no TCS) |
| 100–0 km/h | 39.0 m (ABS) | 39.0 m (ABS) | 47.7 m (locked wheels) |
| Skidpad R40 | 0.91 g | 1.00 g | 1.00 g |

And what the garage changes on the same car ([tuning garage](docs/architecture/tuning-garage.md)):

| | Off-road | Street | Slick | Rain |
|---|---|---|---|---|
| Skidpad, dry (warm tyres) | 0.76 g | 0.83 g | 0.94 g | 0.79 g |
| Skidpad, deep water | 0.40 g | 0.47 g | 0.22 g | 0.56 g |
| Skidpad, gravel | 0.62 g | 0.50 g | 0.37 g | 0.50 g |

Cold slicks give 0.74 g. A rear wing at 200 km/h makes 1,394 N at 24° and stalls past that (920 N at 30°). More negative camber costs braking (39.0 m at 0°, 42.6 m at −6°).

**How light is it?** Headless core only, single-threaded on a 2.1 GHz cloud Xeon.
This measures physics, ground-cache management and the analytic test ground. It does
**not** include Unreal Engine scene queries, Chaos, rendering, animation, audio or game
logic, so it is not a claim about how many vehicles a game can run
([what these numbers do and don't mean](docs/architecture/performance-and-risks.md)):

| Scene | CPU per 60 fps frame | Engine ground rays per frame |
|---|---|---|
| Player car: 60 Hz world × 4 substeps, 5-point contact + envelope probes, ground cache | ~0.07 ms | ~24 |
| + 16 nearby AI cars (LOD1) + 300 traffic cars (LOD2) | ~0.44 ms total | ~108 total |
| + 1000 traffic cars instead of 300 | ~0.71 ms total | ~150 total |

On curbs and potholes, the cached 60 Hz × 4 configuration is within ~5–8% of a 2000 Hz reference (rate-fair, 60 Hz-window metrics), at ~4× fewer rays than direct 240 Hz querying ([ADR-0003](docs/decisions/ground-prefetch.md), [risks & budget](docs/architecture/performance-and-risks.md)).

These are fictional cars tested against plausibility envelopes, not real-world validation.

## Build

Requires CMake ≥ 3.20 and a C++20 compiler.

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build
ctest --test-dir build --output-on-failure      # 151 test cases
./build/ordbench                                # Markdown benchmark report
./build/ordbench --csv-dir /tmp/traces          # per-step telemetry CSV for plotting
./build/ordbench --write-baseline               # regenerate benchmarks/baseline.csv (reviewed changes only)
```

### Minimal use of the core

```cpp
#include "ord/presets.hpp"
#include "ord/vehicle.hpp"
#include "reference_vehicles.hpp"

ord::Vehicle car(ord::vehicles::referenceSportsRwd(),
                 ord::makePreset(ord::DrivingPreset::Sport));
ord::FlatGround ground;               // hosts implement ord::GroundQuery
car.reset({0, 0, 0.6}, 0.0, ground);

ord::DriverInput in;
in.throttle = 1.0;
for (int i = 0; i < 360 * 5; ++i)
    car.step(1.0 / 360.0, in, ground);   // fixed timestep
double kmh = ord::msToKmh(car.telemetry().speed);
```

### Inside a game engine (the light path)

```cpp
// Once per render frame, on the thread that owns the world:
requests.clear();
car.groundPrefetchRequests(requests);              // one strip per wheel
cache.rebuild(engineGround, requests.data(), requests.size());

// In the engine's 60 Hz physics step (host integrates the chassis):
car.setChassisMode(ord::ChassisMode::External, /*hostAppliesGravity=*/true);
car.setChassisState(bodyStateFromEngine);          // see ord/host_frame.hpp for UE units
car.stepSubstepped(1.0 / 60.0, 4, input, cache);   // tires/suspension at 240 Hz, no engine queries
applyToEngineBody(car.chassisForces());            // force + torque about the CoM
```

## Repository layout

```
src/core/          engine-independent physics core (ord::)
plugin/            Unreal Engine 5 plugin (draft, not yet compiled)
src/vehicles/      fictional reference vehicle definitions
src/scenarios/     reproducible benchmark manoeuvres
tools/ordbench/    benchmark report generator
web/               WebAssembly bindings and smoke test for the browser demo
tests/             unit, regression and benchmark tests (no dependencies)
docs/research/     Forza public research, telemetry, KinetiForge audit, alternatives, tire parameters
docs/architecture/ architecture, drivetrain solver, presets & drift assist, open world & LOD,
                   tuning garage, suspension kinematics, crash survivability, rollover, two-wheelers,
                   performance & risks
docs/benchmarks/   benchmark plan, telemetry channels, baseline results
docs/decisions/    architecture decision records
docs/roadmap.md    milestones and priorities
```

## Research

| Document | Contents |
|---|---|
| [forza-physics.md](docs/research/forza-physics.md) | What Turn 10 / Playground have publicly said about tires (8 contact points at 360 Hz in FM 2023), suspension, drivetrain, assists, with evidence class and confidence |
| [forza-handling-philosophy.md](docs/research/forza-handling-philosophy.md) | How Forza makes a sim-based model accessible; implications for our presets |
| [forza-telemetry.md](docs/research/forza-telemetry.md) | Official Data Out format (Sled/Dash), what lawful self-recorded telemetry can and can't validate |
| [kinetiforge-analysis.md](docs/research/kinetiforge-analysis.md) | Source-level audit of the MIT KinetiForge UE5 plugin: capability matrix, bugs, reusable parts |
| [alternatives.md](docs/research/alternatives.md) | Chaos Vehicles, PhysX 5, Jolt, Chrono, Bullet, GPL simulators: verified licenses and reuse options |

Research caveat: the sandbox used for this research couldn't load several
official pages directly. Findings marked "(snippet)" or "(extract)" come from
search-engine excerpts and should be checked against the live pages before
being quoted.

## Roadmap

Next: a test bench for setups (scripted manoeuvres, side-by-side setup
comparison, parameter sweeps) → compliance and solid axles on top of the linkage
kinematics → compile and validate the UE5 plugin (Chaos parity < 1%, real ray costs) → proving-ground
level and Chaos Vehicles A/B → multiplayer. See [docs/roadmap.md](docs/roadmap.md).

## Versioning

Pre-1.0 [semantic versioning](docs/RELEASING.md): minor versions may change the API and the
handling. Every change to the golden baseline is called out in the release notes.

## Contributing & license

See [CONTRIBUTING.md](CONTRIBUTING.md) and [SECURITY.md](SECURITY.md).
Licensed under the [MIT License](LICENSE).
