# Driving presets and configurable assists

Presets are **named bundles of `AssistConfig` values**. They never touch the
physical car. Source: [`src/core/include/ord/presets.hpp`](../../src/core/include/ord/presets.hpp),
[`assists.hpp`](../../src/core/include/ord/assists.hpp). Rationale:
[ADR-0002](../decisions/one-simulation-many-experiences.md).

## Pipeline

```
DriverInput (device)  ──►  AssistSystem  ──►  ActuatorCommands  ──►  Vehicle physics
 steer/throttle/brake       input shaping         steer angle             (identical for
 clutch/shift/handbrake     steering limit        throttle                 every preset)
                            countersteer aid      per-wheel brake torque
                            transmission          clutch, gear
                            ABS / TCS / ESC
                                 │
                                 └──► AssistReport (every intervention, per step)
```

## Parameter reference

| Parameter | Kind | Meaning | Real-car equivalent? |
|---|---|---|---|
| `steeringMode` | input | `Direct`: input → road-wheel angle, linear. `SpeedSensitive`: full stick = angle for the tightest sustainable radius + target front slip angle | Direct: yes. SpeedSensitive: **no** (gamepad aid) |
| `steeringExponent` | input | Input shaping `|x|^e` (finer control near centre) | no |
| `steerRate`, `steerReturnRate` | input | Max steering input change per second (towards / back to centre) | no (thumbstick smoothing) |
| `targetFrontSlipAngle` | input | Slip angle allowed at the SpeedSensitive limit (≈ tire peak) | no |
| `throttleRate` | input | Max throttle change per second | partly (drive-by-wire maps) |
| `abs`, `absTargetSlip` | electronic | Per-wheel brake-pressure modulation holding slip near the target | **yes** |
| `tractionControl` | electronic | `Off`/`Sport`/`Full`: engine-torque cut on driven-wheel slip (Sport allows more slip) | **yes** |
| `stabilityControl` | electronic | `Off`/`Sport`/`Full`: yaw-rate and sideslip control by braking single wheels plus torque cut | **yes** |
| `countersteerAssist` | aid | 0..1 blend of front wheels toward the direction of travel when the rear slides | **no** (labelled) |
| `driftAssist`, `driftAngleHalf`, `driftAngleMax` | aid | 0..1. In a slide, steering says where to go: into the turn asks for drift angle, centred straightens the car out; the aid countersteers and trims (never adds) throttle | **no** (labelled) |
| `shiftMode`, `autoClutch`, `launchRpm` | transmission | Automatic vs manual shifting; automatic clutch vs pedal | yes (automatic / DCT / manual) |

## The presets

| Setting | FUN | SPORT | SIMULATION | DRIFT |
|---|---|---|---|---|
| Steering, gamepad | SpeedSensitive, exp 1.4, rate 2.5/s (return 4/s), slip target 0.10 rad | SpeedSensitive, exp 1.2, rate 4/s (return 6/s), slip target 0.14 rad | Direct, linear, unfiltered | SpeedSensitive, exp 1.2, rate 5/s (return 8/s), slip target 0.14 rad |
| Steering, wheel | Direct (always) | Direct (always) | Direct (always) | Direct (always) |
| Throttle smoothing | 3 /s | 6 /s | none | 6 /s |
| ABS | on | on | **off** | on |
| Traction control | Full | Sport | **off** | **off** |
| Stability control | Full | **off** | **off** | **off** |
| Countersteer aid (gamepad) | 0.6 | 0.25 | **0** | 0 (the drift assist does it) |
| Drift assist | 0 | 0 | **0** | **1** (19° at half steering, up to 34° or the steering lock; centred exits) |
| Shifting | automatic | automatic | manual (sequential), auto clutch | automatic (holds gear while sliding) |
| Manual clutch | — | — | opt-in (`autoClutch = false`) | — |

Each row is independently overridable, e.g. "SIMULATION + ABS":

```cpp
AssistConfig cfg = makePreset(DrivingPreset::Simulation, InputDevice::SteeringWheel);
cfg.abs = true;   // keep ABS, everything else stays off
```

### Design intent per preset

- **FUN** (default open world): forgiving and believable. Limits what the driver *asks* for (steering range, throttle rate). It never adds grip. Slides are caught by ESC and the countersteer aid, not by fake yaw damping. Cars stay distinct because limits come from their physics.
- **SPORT** (Horizon-inspired sweet spot): realistic traction, more direct steering, no ESC, light countersteer help on a gamepad. Drift-capable: TCS Sport allows 15% slip.
- **SIMULATION**: no aids, no filtering. The driver owns every input. ABS/TCS/ESC exist only if deliberately enabled and then behave like the real systems.
- **DRIFT**: SPORT's steering with traction and stability control off and the drift assist on. Throw the car in (handbrake, lift-off or power), then steer where you want to go: into the turn holds a drift (more steering, more angle and a tighter line), centred straightens the car out once it points where you want. Keep the throttle in to sustain it (less throttle, less angle); the assist only trims throttle when the car over-rotates or nears the rev limiter. Full lock and throttle from parking-lot speed spins donuts: the assist steps aside.

## Drift assist (an aid with no real-car equivalent)

Why it exists: measured on the reference sports car, a slide grows from catchable to a spin in about the reaction time of a player (0.15 s), and past roughly the steering-lock angle (~32-35° of sideslip on these cars) even full lock cannot stop the rotation. That is real physics; on a gamepad it makes drifting a guessing game.

What it does (`AssistSystem::applyDriftSteer` / `applyDriftThrottle`):
- **Engages** once the rear axle slides (> 8° rear slip angle with yaw), blending in over 0.15 s; lets go when the rear grips again.
- **Steering says where to go** (v2): into the turn asks for drift angle, `driftAngleHalf` at half steering up to `driftAngleMax` at full (never beyond the steering lock, the largest angle the front wheels can still catch; `driftAngleMax` defaults to 1 rad so the lock is what limits it: ~32° on the stock reference cars, ~52° with a 0.9 rad drift lock set through `Vehicle::setSteeringLock`); centred or against the turn asks for 0, so the car straightens out. A bigger angle settles slower and tighter, so steering harder also tightens the line. (v1 held 20° with the steering centred; measured with keyboard-style input, a player who let go once the car pointed down the new road kept rotating ~80° past it.)
- **Exits stay in control**: once engaged, the assist keeps steering until the car is straight and settled, even as the angle swings through zero; it straightens at most 0.55 rad/s so the yaw momentum is gone by then. Letting go mid-exit used to let the rear swing ~9° the other way.
- **Catching is a cascade**: the angle error sets how fast the drift angle should change; the wheels are steered from the front axle's direction of travel by the error in that rate, plus a slow integral trim. Controlling the rate is what catches the yaw momentum of an aggressive entry.
- **Throttle is part of the request** (v3): the angle asked for scales from half (no throttle) to all of it (80 % throttle and up), as in a real drift where power holds the angle and easing off lets the car come back. Before, the assist held the same angle whatever the throttle did, so the pedal only changed the speed.
- **Throttle trim**: only removed, never added, and never more than 60 %: when the angle predicted 0.25 s ahead overshoots the target while the car is still rotating away, and over the last 4 % of the rev range (soft limiter). With no power at all the rear tyres grip again and the car snaps back the other way, which is what the earlier 100 % cut did after a handbrake entry. No trim builds up while the handbrake holds the clutch open (it would starve the clutch kick). Keeping a drift alive is still the player's job.
- **Donuts** (v3): near full lock with the throttle in, asked for below ~30 km/h while the car is not already drifting, means "keep rotating". The assist steps aside (no countersteer, no trim) and reports `donut`. Easing off the lock or the throttle hands the slide back, and the assist catches it and straightens the car. The decision is latched when the request starts, so a drift that slows down at full lock stays a drift. Before, full lock read as "hold ~32°": the assist countersteered against the stick and the car ran a wide 60 km/h circle instead of a donut.
- **Handbrake entries**: while the handbrake is pulled the assist hands back 30 % of the steering (catching the flick at full countersteer swung the nose back once the clutch kick spun the rear up). The worst dip below the requested angle after a handbrake entry fell from ~14° to ~7°.
- **Reports** `driftActive`, `driftTargetSideslip`, `driftSteerAngle` and `driftThrottleCut` every step.

It changes nothing physical: tires, mass, inertia and grip are identical, and with `driftAssist = 0` it is fully gone (tested: the same entry with constant full lock spins).

Measured with steady inputs after a handbrake entry (10 s, see [baseline-results.md](../benchmarks/baseline-results.md)): the reference sports car and the Rosser Stella V8 hold 17° / 24° / 31.5° within about 1° for steering 0.5 / 0.75 / 1 at full throttle, and the smaller angles 60 % throttle asks for (15° / 21° / 28°) just as closely, with no spins; holding full steering and full throttle ends the 10 s at 50-58 km/h. Centred steering straightens the car within about 0.7 s, without a swing to the other side. With keyboard-style input (digital keys, the page's steering ramp), releasing the turn once the car travels in the wanted direction ends ~12° past it, every time. Two of these values are in the golden baseline.

### Related fixes for everyone who drifts without the assist

- **Countersteer reach** (SPORT/FUN steering): the speed-sensitive range protects the front tires, so it is now measured from where the front axle is travelling. In a slide, the range on the countersteer side widens to reach that direction. Before, at 60 km/h in a 26° slide, full opposite stick gave only 16° of countersteer where 23° was needed.
- **Gearbox**: the automatic box no longer upshifts while the rear slides under power (it already held the gear for straight-line wheelspin), which used to drop the revs and kill the drift. Exception (v3): once a drift has the engine on the limiter and the next gear would still be in the power band, it shifts clutch-to-clutch so more throttle means something again; before, a long drift sat on the limiter in 1st at 75 km/h. It never shifts while the handbrake holds the clutch open.
- **Handbrake and the auto clutch** (v3): a driver puts the clutch in while pulling the handbrake, or the engine just keeps driving the rear wheels it is trying to lock. The auto clutch now does the same whenever the handbrake acts on a driven wheel, and lets the clutch back out over 0.15 s on release (with the revs up, the clutch kick). Before, handbrake plus throttle did nothing at all: the rear wheels never stopped turning. Reported as `handbrakeClutch`. Front-driven cars keep their clutch in.

## Measured differences (v0.1, reference sports RWD, 360 Hz)

From [baseline-results.md](../benchmarks/baseline-results.md):

| | FUN | SPORT | SIMULATION |
|---|---|---|---|
| 0-100 km/h | 5.2 s (TCS Full cuts throttle) | 4.7 s | 6.2 s (heavy wheelspin, peak slip ≈ 3.8) |
| 100-0 km/h | 39.7 m (ABS) | 39.7 m (ABS) | 48.2 m (locked wheels) |
| Skidpad R40 | 0.95 g | 1.01 g | 0.98 g |

Assists can't change tire grip, but they *can* change how well the car uses it.
On the RWD car, FUN (0.95 g) sits below SIMULATION (0.98 g). The SPORT/SIMULATION
difference comes from TCS preventing power oversteer during the speed-holding
throttle inputs. On the understeering FWD hatch, FUN reaches 0.89 g vs 0.85 g:
ESC brakes the inside rear wheel, and the extra yaw moment lets the rear axle
contribute more. Real ESC and brake-based torque vectoring do the same, so this is
physically legitimate. It is tracked so that aid tuning never turns into a
hidden handicap for SIMULATION players.

## Switching presets while driving

`AssistSystem::setConfig()` can be called at any time.
- Physical aids (ABS, TCS, ESC) switch immediately, with no residual effect.
- If the steering *mapping* changes (direct vs speed-sensitive, exponent, slip target, drift assist on/off), the road-wheel angle crossfades from the old mapping's output to the new one over 0.3 s. Without the crossfade, switching FUN → SIMULATION mid-corner made the front wheels jump by several degrees.

Covered by regression tests.

## Planned (roadmap)

- Braking assist (brake-pressure ramp), launch control, anti-stall, auto-blip.
- Optional "stability aid" (yaw-damping torque) as a separate, labelled, default-off aid if playtests demand it (ADR-0002).
- Force-feedback pipeline from the tire aligning torque (`TireForces::mz`), with wheel-specific settings.
- Per-device response curves and dead zones exposed to the player.
- Reference speed estimator from wheel speeds and an IMU, replacing the current ground-truth reference speed used by ABS/TCS.
