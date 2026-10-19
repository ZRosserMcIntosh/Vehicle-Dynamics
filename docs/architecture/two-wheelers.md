# Two-wheelers: bicycles, e-scooters and motorcycles

Status: **rideable prototype.** The validated multibody bicycle now has
leaning tyres (camber thrust, twisting torque, a rounded crown) on arbitrary
ground, and a rider, powertrain, brakes and aids on top: a bicycle, an
e-scooter and a sport motorcycle can be ridden, in the browser test track
too. Suspension, gearboxes, crash physics and the UE adapter are still to do
(see limitations).
Source: [`single_track.hpp`](../../src/core/include/ord/single_track.hpp),
[`single_track.cpp`](../../src/core/src/single_track.cpp),
[`two_wheeler.hpp`](../../src/core/include/ord/two_wheeler.hpp),
[`two_wheeler.cpp`](../../src/core/src/two_wheeler.cpp), tests in
[`tests/test_single_track.cpp`](../../tests/test_single_track.cpp) and
[`tests/test_two_wheeler.cpp`](../../tests/test_two_wheeler.cpp).
Roadmap: [M7](../roadmap.md#m7--two-wheelers).

The game is set in southeastern and southern Brazil, where bicycles, shared
e-scooters and 125–160 cc commuter and delivery ("motoboy") motorcycles are a
large part of real traffic. Two-wheelers should reach the same standard as the
cars: one physical simulation, and presets that only change aids, never
physics or grip ([ADR-0002](../decisions/one-simulation-many-experiences.md)).

## Why a two-wheeler is not a narrow car

A car is statically stable: four contact patches hold it up, and the driver
mostly asks for yaw. A two-wheeler is statically unstable and is balanced
*dynamically*. Lean, steer, gyroscopic and trail effects are coupled, and the
rider steers mostly with torque, not angle. The first move into a right turn
is a brief steer to the left (counter-steering). The car core's assumptions
don't hold here: the chassis is no longer one rigid body on suspension
corners, steer is a degree of freedom rather than a prescribed angle, and
lateral tyre force comes largely from camber, not slip. So two-wheelers get
a multibody core of their own and reuse the car infrastructure around it.

## What is reused and what is new

| Area | Reused from the car core | New for two-wheelers |
|---|---|---|
| Maths, determinism, fixed step, zero-allocation step | `ord/math.hpp`, `ord/linear_solve.hpp` (dense solve), test harness, allocation audit | — |
| Chassis | — | **Single-track multibody** (rear wheel, rear frame + rider, front frame, front wheel; revolute joints; steer as a DOF) — done |
| Tyre | `MagicFormulaTire` slip forces, relaxation-length transient slip, surface properties | **Camber thrust**, camber twisting torque, **toroidal (crowned) contact** with the contact point moving across the tread as the bike leans. Camber thrust is also car roadmap M3.1, so both share it |
| Ground | `GroundQuery`, ground prefetch cache, surface micro-texture | Contact-point search on a crowned tyre over a sloped/rough surface |
| Suspension | Spring/damper/bump-stop formulas, unsprung-mass ideas | Telescopic fork (sliding joint along the steer axis), swingarm (revolute joint) with chain-force anti-squat |
| Powertrain | Engine curve, clutch, gearbox, implicit drivetrain solver | Chain final drive, CVT, hub motor + battery, human pedalling model |
| Assists | Assist architecture: aids act only through actuators and report every intervention | Virtual rider (balance), two-wheeler ABS/TC/wheelie control |
| Telemetry, debug draw, benchmarks | Recorder, CSV, SVG debug draw, `ordbench`, golden baseline | Two-wheeler channels (lean, steer torque, weave/wobble metrics) |

## The model that exists today

Four rigid bodies in maximal coordinates (24 DOF), each with mass, full
inertia tensor, position + quaternion and world-frame velocities. Gyroscopic
terms (ω × Iω) are included. Three revolute joints: rear axle, steer axis
tilted back by λ, front axle. Contact modes:

- **NoSlip** (Whipple/Carvallo): the material velocity of each contact point
  is zero, giving 3 velocity constraints per wheel. The vertical one is the
  derivative of "contact height = 0", which is also enforced by position
  projection. 21 constraints on 24 DOF leave the 3 velocity DOF of the
  Whipple bicycle (forward speed, lean rate, steer rate). This mode is for
  validation.
- **Tyre**: normal spring-damper plus `MagicFormulaTire` longitudinal and
  lateral forces with relaxation-length slip, rolling resistance as an axle
  moment, and the lean response of a real tyre:
  - **Camber thrust** (motorcycle Magic Formula form: camber adds to the
    argument of the sine, so camber and slip share one friction limit) and a
    **camber twisting torque** that steers the wheel into the lean.
  - **Toroidal contact**: the tyre has a crown radius ρ; the contact point is
    centre + (R − ρ)·d − ρ·n, so it rolls around the crown as the bike leans.
  - **Arbitrary ground**: one vertical ray per wheel per step gives that wheel
    a local plane (point, normal, surface), so the bike rides over bumps,
    slopes and the browser's city heightfield.
  The car model keeps its flat footprint. When this was added cars passed camber 0, so their
  behaviour and benchmark numbers were unchanged; since the tuning garage they pass their real
  camber, and the car camber model is enabled by `applySetup()` (stock definitions keep camber
  stiffness 0, so the baseline is unchanged).

Dynamics: Newton–Euler equations with constraints at acceleration level
(`(J M⁻¹ Jᵀ) λ = −J̇v − J M⁻¹ F`), classical RK4, then mass-weighted position
projection onto the holonomic constraints and velocity projection onto all
constraints. Inputs: steer torque, rear drive torque, and an external roll
couple (a disturbance, also used to extract the mass matrix). The step uses
only fixed-size arrays and does not allocate (tested). It costs about 25 µs
per step on the development machine, so it is far too slow to use as-is for
real-time traffic.

The model is derived from rigid bodies, joints and rolling contact. It
**does not use** Meijaard's closed-form linear matrices, which is what makes
the comparison below an independent check.

## Validation results (Meijaard et al. 2007 benchmark bicycle)

Benchmark parameters are from Table 1 of the paper, converted from the paper's
axes (x forward, y right, z down) to ord's (x forward, y left, z up): z
coordinates and the products of inertia I_xz change sign. The linearised
dynamics were extracted from the nonlinear model by central finite
differences (h = 10⁻⁶) of its constrained accelerations, taken around upright
straight running in the states [φ, δ, φ̇, δ̇]. The mass matrix comes from the
responses to unit lean and steer torques.

| Quantity | ord nonlinear model | Published | Error |
|---|---|---|---|
| Weave speed | 4.292382536339 m/s | 4.292382536341111 m/s | −2.2·10⁻¹² m/s |
| Capsize speed | 6.024262015383 m/s | 6.024262015388372 m/s | −5.4·10⁻¹² m/s |
| M, K0, K2, C1 (all 16 entries) | — | eq. (A 1)–(A 3) values | ≤ 4.4·10⁻¹³ of each matrix's largest entry |
| Eigenvalues at v = 0, 2, 4, 5, 6, 8 m/s | see test output | from the published matrices | ≤ 2.3·10⁻¹² (relative, \|λ\| ≥ 1 scale) |
| Static eigenvalues (v = 0) | ±3.131643248, ±5.530943718 | ±3.13164324790656, ±5.53094371765393 | < 10⁻¹¹ |

Eigenvalues of the nonlinear model at 5 m/s, for example, are −14.078389693,
−0.775341882 ± 4.464867714i and −0.322866429. The target was 10⁻³ m/s on the
critical speeds and 10⁻³ relative on the eigenvalues. The model reaches
finite-difference precision, which is what you'd expect if the multibody
equations and the benchmark describe the same mechanics. The tests assert
10⁻⁸ (speeds) and 10⁻⁸/10⁻⁹ (eigenvalues/matrices), leaving roughly three
orders of magnitude of margin for other compilers.

A note on the reference data: the published matrices give a first static
eigenvalue of 3.13164324790656. A value of 3.13164324079320 has also been
quoted for it. That value disagrees with the matrices from the 9th digit and
looks like a transcription slip. The test uses the matrix-consistent value,
and the test file says so.

Time-domain checks (nonlinear, NoSlip, RK4 at 0.5 ms):

- 5 m/s (inside the self-stable band 4.29–6.02 m/s): a 0.05 rad lean first
  overshoots to about 0.075 rad, then decays to about 0.003 rad (6% of the
  initial lean) at 10 s.
- 3 m/s (below the weave speed): a 0.02 rad lean grows as an oscillating
  weave and the bicycle falls (|φ| > 1 rad) after about 5 s.
- Energy with no drive and no damping: |ΔE| ≤ 1.5·10⁻¹⁰ J over 10 s
  (E ≈ 1600–2000 J). Holonomic constraint error stays around 10⁻¹² m.
- Counter-steering: a right steer torque at 5 m/s steers right and leans the
  bicycle left first.
- Tyre mode, knife-edge tyres (camber, crown and trail off): as cornering
  stiffness rises from 15 to 80 (per unit load, per rad), the maximum
  deviation of the lean history from NoSlip falls from 6.5·10⁻³ to
  1.2·10⁻³ rad over 4 s (initial lean 0.05 rad), so the model converges
  toward the Whipple constraint.
- Tyre mode, default (leaning) bicycle tyres: the bicycle is self-stable in a
  band of about 4.8–6.5 m/s (NoSlip: 4.29–6.02). At 5.5 m/s a 0.05 rad lean
  decays to 0.003 rad; at 3 m/s it falls. Tyre compliance, the crown and the
  twisting torque shift both critical speeds a little; on its own the crown
  moves the band down and a 10 mm trail without camber terms destroys it,
  which is why trail stayed 0 until camber existed. The loads carry the weight
  within 1% and rolling deceleration is within 10% of C_rr·W / m_eff.
- Camber thrust carries the cornering force: in the same steady lean the
  tyres need about 1/20 of the slip angle per radian of lean that slip-only
  tyres need (0.001 vs 0.03 rad).
- Ground: the same run on a plane 1.5 m higher and rotated 40° is the same
  run (lean differs < 10⁻⁶ rad); a 4 cm bump at 6 m/s is ridden over.

Test runtime: about 3.9 s in Release. Debug+ASan builds run shortened
versions (about 20 s).

## Rideable two-wheelers (`TwoWheeler`)

`TwoWheeler` wraps the Tyre-mode model with what a game needs. Everything
acts through what a real rider has: handlebar torque, throttle or pedal
torque, two brake levers and, at walking pace, a foot / body roll moment. No
aid changes grip or any force model.

- **Rider, Lean mode** (default): the steer input asks for a lean angle,
  limited to what the bike can hold (65% of tyre grip, and the steering lock
  at that speed), eased in and out at up to 1.2 rad/s like a rider would. An outer loop picks a steer angle from the point-mass lean
  dynamics φ̈ = (g/h)φ − v²/(h w)·δ, an inner "arm" loop tracks it with a
  stiff, damped torque that relaxes at speed (a stiff grip at speed feeds the
  weave mode), and at speed a direct lean-error torque plus a slow integral
  do most of the work. The arms also keep the bars within twice the steer
  angle the tyres can use at that speed. **Counter-steering emerges**: to turn left at 20 m/s
  the first bar torque and steer angle go right.
- **Rider, Direct mode**: the input is handlebar torque. Realistic and hard;
  meant for mouse steering.
- **Powertrain**: power-limited wheel torque (min(T, P/ω)), optional top-speed
  limiter, front or rear drive, aerodynamic drag. No clutch or gearbox yet.
- **Brakes**: front and rear lever torques with a dry-friction clamp, so a
  stopped bike holds without the jitter a stiff viscous brake has at game
  step sizes.
- **Aids** (each optional, all acting on throttle, brakes or the bars):
  - low-speed balance (foot down, fades out between 1.5 and 4 m/s; off = a
    stopped bike falls over);
  - ABS: slip control toward ~8% slip, plus **cornering ABS** that caps brake
    torque to the friction left over from cornering at the current lean (as
    IMU-based ABS on modern bikes does);
  - **cornering traction control**: slip control whose target shrinks with
    lean, plus the same friction-circle torque ceiling, computed from
    low-passed wheel loads (feeding back the instantaneous load pumped the
    pitch mode);
  - wheelie control: keeps the front loaded (cuts drive as the front load
    falls below ~45% of static, before it lifts, since steering needs a loaded
    front tyre) and eases the front brake as the rear unloads (stoppie
    control). With a larger pitch setting it becomes a wheelie assist that
    holds the front up at that angle (pitch predicted 0.3 s ahead).
- **Vehicles**: the benchmark bicycle with a pedalling rider (60 N·m /
  350 W), a stand-up e-scooter (8.5" wheels, 500 W hub motor, 25 km/h
  limiter) and a ~180 kg sport motorcycle with a 75 kg rider (80 kW, steering
  damper, 0.08 m tyre relaxation length: with 0.2 m its weave, coupled with
  the rider loop, ran away under aggressive input at 25–35 m/s). The scooter
  and motorcycle are representative, not measured.

Results (`tests/test_two_wheeler.cpp`, 2 ms step, the browser's rate):

| Check | Result |
|---|---|
| Lean to 80% input: time to 90% of the target | bicycle 1.3 s (6 m/s), e-scooter 0.9 s (6 m/s), motorcycle 1.2–1.7 s (10–35 m/s); overshoot ≤ ~11% (25% for the e-scooter at walking pace) |
| Back upright after release | 0.9–1.5 s |
| Counter-steer at 20 m/s (left turn) | bars first go right (positive torque and steer), then lean ≈ −0.44 rad |
| Full throttle, motorcycle from 5 m/s, no aids | front lifts, loops over backwards (crash) |
| Wheelie assist at 0.4 rad | holds 0.40–0.48 rad pitch for 3.5 s |
| ABS stop from 20 m/s, both brakes | 2.2 s (0.92 g) |
| Front brake 0.6 at 11 m/s, no aids | over the bars; with stoppie control: pitch > −0.25 rad |
| E-scooter full throttle | 24.5 km/h (limiter) |
| Bicycle up a 6% grade | climbs, contact follows the ground within 5 cm |
| Determinism | identical runs are bit-identical |
| Rough random riding, Fun aids, 12 × 30 s each (full-lock flicks, throttle stabs, brake grabs; motorcycle under ~110 km/h) | bicycle 0, e-scooter 1, motorcycle 0 crashes |

Native cost is about 19 µs per 2 ms step (motorcycle), about 1% of a core at
500 Hz.

## Plan

### 1. Single-track base and bicycle (~2.5 months)

- [x] Nonlinear multibody single-track, NoSlip + basic Tyre contact, Meijaard benchmark validation.
- [x] Camber thrust and twisting torque in `MagicFormulaTire` (shared with cars, which pass their real camber; the garage enables the car camber model), **toroidal contact** (contact point at centre + (R − ρ)·d − ρ·n), local ground planes per wheel.
- [x] First virtual rider (Lean and Direct modes), low-speed balance moment, brakes, power-limited drive, ABS/TCS/wheelie control (`TwoWheeler`). Still to do from this list: a real foot-down leg contact, rider body lean, the human pedalling model, presets.
- Performance: a reduced-coordinate formulation or a tailored sparse solve (21×21 dense today) to reach tens of µs per *frame* per player bike, plus a cheaper LOD model for traffic riders.
- **Rider control ("virtual rider")**: the rider's input is a target lean (or target path curvature). A controller turns it into steer torque, using the linear model at the current speed for gains (counter-steer emerges on its own). The rider can't create forces that aren't there. Balance aids act **only through actuators**: steer torque, rider lean (upper-body mass shift as a joint), and later brake/throttle. The model offers no "hidden hand" holding the bike up.
- **Presets** follow ADR-0002: they never change grip or physics.
  - FUN: target-lean input with strong virtual-rider gains, automatic foot-down at very low speed, auto-recovery from small slides through steer torque, wide lean limits only where the tyres can deliver them.
  - SPORT: lighter assistance, faster lean response, foot-down still automatic.
  - SIMULATION: the stick (or a wheel/handlebar device) maps to steer torque and rider lean directly. No balance aid beyond what a real rider's body does.
- **Low-speed balance and foot-down**: below ~1.5–2 m/s even a good rider uses steer and body movement; at a standstill the foot comes down. Model this as a rider leg that contacts the ground (an unilateral constraint) and is triggered by the rider controller. It isn't a constraint that bolts the bike upright.
- **Human pedalling power model**: crank torque as a function of cadence (force–velocity relation), a sustainable power–duration curve (fatigue), and derailleur gears as a discrete ratio set with shift time. The rider controller picks cadence and gear. Pedal-stroke torque ripple is optional.
- **Reference vehicles**: road bike, city bike (upright rider, heavier), both with measured or literature geometry where a redistributable data set exists, otherwise fictional and plausible like the cars.

### 2. E-scooter and e-bike (+3–4 weeks)

- Kick-scooter geometry: small wheels (8–10″), near-zero or negative trail, low head angle, standing rider (higher CoM, big rider/vehicle mass ratio). Self-stability is weak or absent, so the virtual rider matters more. These vehicles are a good stress test of the controller.
- **Hub motor + battery**: motor torque/speed envelope, controller current limit, **battery sag** (open-circuit voltage vs state of charge, internal resistance → voltage drops under load → less top speed and torque), regenerative braking, legal speed limiting (25 km/h e-bikes, local scooter rules) as a controller feature, not a physics change.
- E-bike: pedal-assist modes (torque or cadence sensor) on top of the human pedalling model.

### 3. Motorcycles (+2–3 months physics/controls)

- Telescopic fork (sliding joint along the steer axis, spring/damper, dive), swingarm with **chain-drive anti-squat**: the chain force line versus the swingarm pivot sets squat or lift under power. Rear shock linkage.
- Toroidal tyres with camber thrust are mandatory here (lean angles reach 45–55°). Tyre relaxation and frame/fork compliance are needed for **wobble** (≈ 6–10 Hz steer oscillation) and **weave** at speed.
- Powertrains: single-cylinder 125–160 cc commuter/"motoboy" engines with a wet multi-plate clutch and sequential gearbox; **CVT** for scooters (centrifugal clutch, variator ratio map); sport-bike inline-four. All reuse the implicit drivetrain solver with a chain or belt final drive.
- **Wheelie and stoppie**: they emerge from the multibody model (a wheel leaves the ground) once contact is unilateral. Wheelie control and stoppie/rear-lift mitigation are electronic aids acting on throttle and brake pressure, like the car TCS/ABS.
- **Low-side / high-side detection**: low-side = front or rear lateral saturation with lean growing. High-side = rear slide followed by sudden grip recovery that throws the rider (large roll rate with the rear tyre recovering grip). Detection runs on the physical state, and its output is an event for the game. It never edits forces.
- **Ragdoll handoff**: when the rider's grip/seat limits are exceeded, hand the rider body (pose + velocities of the rear-frame rider part) to the engine ragdoll, and continue the bike as a separate rigid multibody (or a simplified rigid body once it's lying down). Needs a frame/handlebar-to-ground contact model (today the simulation stops making sense once a wheel lies flat).
- Reference vehicles: 125–160 cc commuter ("motoboy"), automatic scooter (CVT), sport bike.
- Validation: Sharp 1971 (weave/wobble mode structure, capsize), Cossalter, *Motorcycle Dynamics* (2nd ed., 2006) for steady-turning and mode data, plus published modal analyses of specific motorcycles where parameter sets are public.

### 4. Unreal Engine integration (+1–2+ months, overlapping)

- Rider skeleton driven by the physical state: lean, steer (hands on bars), pedalling cadence, knee-out/lean-in from the rider-lean actuator, foot-down. IK to pegs/pedals/handlebar.
- Ragdoll handoff as above; blending back to the animated rider after recovery.
- Camera: lean-aware chase camera (horizon partly follows lean), helmet/first-person camera with head stabilisation.
- Same adapter pattern as the cars: `stepSubstepped`, ground prefetch strips per wheel, `ChassisMode::External`-style coupling (will need a multibody variant: the engine can host the rear frame as one rigid body, with the core owning the joints and wheels).

### Recommended order and effort

bicycle → e-scooter/e-bike → motorcycle. Each step reuses the previous model
and adds one hard thing: rolling multibody and rider control, then weak
self-stability and electric drive, then high-speed tyres, suspension and
crashes.

| Step | Rough effort |
|---|---|
| Single-track base + bicycle (incl. this validation, camber/toroidal tyre, virtual rider, foot-down, two reference bikes) | ~2.5 months |
| E-scooter + e-bike | +3–4 weeks |
| Motorcycles (physics + controls) | +2–3 months |
| UE rider work (skeleton, animation, ragdoll, camera) | +1–2+ months |

## Limitations of the current prototype

- No UE adapter yet; the browser track is the only place to ride.
- Rigid frames, rigid rider (no body lean or hang-off), **no suspension**: the motorcycle's vertical tyre stiffness stands in for tyre + suspension in series (~3 cm sag), so there is no fork dive or squat geometry. The steering damper on the motorcycle is there because the rigid model wobbles from ~20 m/s without one.
- No frame, pedal, foot or handlebar contact: a crash is detected (lean past ~66°, pitch past ~69°, or a wall in the city) and the ride stops. The low-speed balance aid is a roll moment, not yet a leg touching the ground.
- No clutch or gearbox (power-limited torque instead), no battery model, no pedalling cadence.
- Camber and crown parameters are plausible, not fitted to measured tyres. The bicycle's self-stable band (4.8–6.5 m/s) is close to, not equal to, the knife-edge benchmark's.
- Above ~110 km/h, full-lock flicks at full throttle can still put the motorcycle into a weave it doesn't recover from, even in Fun.
- The cornering ABS / traction ceilings and the wheelie/stoppie controls are modelled on what modern bike electronics do, not on any manufacturer's logic.
- Slip states are updated once per step, before the RK4 stages (operator split), so very stiff tyres need small steps (0.2 ms for the stiff-tyre test). The game tyres run at 2 ms.
- Lean, steer and steer torque use the benchmark sign convention (positive = right). `TwoWheelerInput::steer` uses the car convention (positive = left).

## References

- J.P. Meijaard, J.M. Papadopoulos, A. Ruina, A.L. Schwab, "Linearized dynamics equations for the balance and steer of a bicycle: a benchmark and review", *Proc. R. Soc. A* 463:1955–1982, 2007.
- J.D.G. Kooijman, A.L. Schwab, J.P. Meijaard, "Experimental validation of a model of an uncontrolled bicycle", *Multibody System Dynamics* 19:115–132, 2008.
- F.J.W. Whipple, "The stability of the motion of a bicycle", *Quarterly Journal of Pure and Applied Mathematics* 30:312–348, 1899.
- R.S. Sharp, "The stability and control of motorcycles", *Journal of Mechanical Engineering Science* 13(5):316–329, 1971.
- V. Cossalter, *Motorcycle Dynamics*, 2nd ed., 2006.
- D.J.N. Limebeer, R.S. Sharp, "Bicycles, motorcycles, and models", *IEEE Control Systems Magazine* 26(5):34–61, 2006.
- H.B. Pacejka, *Tire and Vehicle Dynamics*, 3rd ed., 2012 (ch. 7 transient slip, ch. 11 motorcycle tyres and camber).
- D. Baraff, "Linear-time dynamics using Lagrange multipliers", SIGGRAPH 1996; E. Hairer, C. Lubich, G. Wanner, *Geometric Numerical Integration*, 2nd ed., 2006 (projection methods).
