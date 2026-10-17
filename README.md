# OpenRoad Dynamics

**An open-source vehicle physics and driving system for Unreal Engine 5, built on an engine-independent C++ core.**

One physically grounded simulation. Configurable assists make it accessible,
and turning them off makes it unforgiving. The aim: Forza Horizon-level
enjoyment on a gamepad and Forza Motorsport-level consequences with every aid
off, in a giant open world.

> **Status: v0.3 (October 2026).** The engine-independent core builds and
> passes 78 automated tests in CI on Linux (GCC, Clang, ASan/UBSan), macOS
> (Apple Clang) and Windows (MSVC). It includes unsprung mass, a multi-point
> tire contact patch with geometric envelope probes for sharp edges, a
> per-frame ground-query cache, host (Chaos) integration with vehicle-local
> substepping, three fidelity tiers (player / nearby AI / traffic), parameter
> validation plus a runtime fault guard, a zero-allocation steady state
> (tested), and a golden benchmark baseline that CI checks.
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
| Tire | Reduced Pacejka Magic Formula: load sensitivity, combined slip, relaxation-length transient slip, standstill stiffness. **Parameter generator** from public tire size/category | `tests/test_tire.cpp` |
| Contact & suspension | **Multi-point contact patch** (1/5/8/9 samples), **implicit unsprung-mass DOF** per corner, springs/dampers/ARB/bump stops, **procedural surface micro-texture** for paving | `tests/test_contact.cpp` (frequencies match the analytic quarter car to <0.1%) |
| Drivetrain | Engine curve, governor, limiter, clutch, gearbox, open/locked/LSD diffs, AWD split, all solved implicitly with stick/slip friction | `tests/test_drivetrain.cpp` |
| Assists | Speed-sensitive gamepad steering (full countersteer reachable in a slide), ABS, brake-based TCS, ESC, countersteer aid, **drift assist** (steering sets the drift angle; countersteer and throttle trim only), auto/manual shifting. Real actuators only; every intervention reported | `tests/test_assists.cpp` |
| Lightness | **Ground prefetch cache** (per-frame adaptive strips; no scene queries from substeps or the physics thread), **vehicle-local substepping** inside a 60 Hz world, **host integration mode** for Chaos | `tests/test_contact.cpp`, `tests/test_host.cpp` |
| Many vehicles | LOD1 nearby AI (`Vehicle` at low fidelity + lane-following driver), LOD2 kinematic traffic (IDM), `LodManager` with seamless promotion | `tests/test_lod.cpp` |
| Robustness | Range validation of every definition field (incl. static-support and unsprung-mass sanity), `faulted()` runtime guard against non-finite state, fuzzed definitions, reversals, zero speed/reverse, contact loss, long-duration drift, determinism | `tests/test_robustness.cpp`, `tests/test_alloc.cpp` |
| Vehicle range | FWD hatch, RWD sports, AWD saloon, 750 kg high-downforce race car (~1.7 g at 100 km/h → ~2.5 g at 250 km/h), ~476 km/h AWD hypercar | `tests/test_robustness.cpp` |
| Regression guard | 13 headline metrics in `benchmarks/baseline.csv` with per-metric tolerances; changing them needs an explicit, reviewed regeneration | `tests/test_baseline.cpp` |
| Tooling | Telemetry ring buffer + CSV, engine-agnostic debug draw (+SVG), `ordbench` report | `tools/ordbench` |
| Unreal Engine 5 | Plugin `plugin/OpenRoadDynamics`: component, Chaos sim callback, game-thread ground prefetch, debug overlay. **Not compiled yet** | — |

Selected results for the fictional 300 kW RWD reference car ([full report](docs/benchmarks/baseline-results.md)):

| | FUN | SPORT | SIMULATION |
|---|---|---|---|
| 0–100 km/h | 5.2 s | 4.7 s | 6.3 s (wheelspin, no TCS) |
| 100–0 km/h | 39.6 m (ABS) | 39.6 m (ABS) | 48.2 m (locked wheels) |
| Skidpad R40 | 0.96 g | 1.02 g | 1.00 g |

**How light is it?** Headless core only, single-threaded on a 2.1 GHz cloud Xeon.
This measures physics, ground-cache management and the analytic test ground. It does
**not** include Unreal Engine scene queries, Chaos, rendering, animation, audio or game
logic, so it is not a claim about how many vehicles a game can run
([what these numbers do and don't mean](docs/architecture/performance-and-risks.md)):

| Scene | CPU per 60 fps frame | Engine ground rays per frame |
|---|---|---|
| Player car: 60 Hz world × 4 substeps, 5-point contact + envelope probes, ground cache | ~0.06 ms | ~24 |
| + 16 nearby AI cars (LOD1) + 300 traffic cars (LOD2) | ~0.37 ms total | ~108 total |
| + 1000 traffic cars instead of 300 | ~0.69 ms total | ~150 total |

On curbs and potholes, the cached 60 Hz × 4 configuration is within ~5–8% of a 2000 Hz reference (rate-fair, 60 Hz-window metrics), at ~4× fewer rays than direct 240 Hz querying ([ADR-0003](docs/decisions/ground-prefetch.md), [risks & budget](docs/architecture/performance-and-risks.md)).

These are fictional cars tested against plausibility envelopes, not real-world validation.

## Build

Requires CMake ≥ 3.20 and a C++20 compiler.

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build
ctest --test-dir build --output-on-failure      # 78 test cases
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
tests/             unit, regression and benchmark tests (no dependencies)
docs/research/     Forza public research, telemetry, KinetiForge audit, alternatives, tire parameters
docs/architecture/ architecture, drivetrain solver, presets, open world & LOD
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

Next: compile and validate the UE5 plugin (Chaos parity < 1%, real ray costs)
→ proving-ground level and Chaos Vehicles A/B → wet roads → kinematic
suspension → multiplayer. See [docs/roadmap.md](docs/roadmap.md).

## Versioning

Pre-1.0 [semantic versioning](docs/RELEASING.md): minor versions may change the API and the
handling. Every change to the golden baseline is called out in the release notes.

## Contributing & license

See [CONTRIBUTING.md](CONTRIBUTING.md) and [SECURITY.md](SECURITY.md).
Licensed under the [MIT License](LICENSE).
