# Suspension kinematics

Source: [`kinematics.hpp`](../../src/core/include/ord/kinematics.hpp),
[`kinematics.cpp`](../../src/core/src/kinematics.cpp), the wheel loop in
[`vehicle.cpp`](../../src/core/src/vehicle.cpp), tests in
[`tests/test_kinematics.cpp`](../../tests/test_kinematics.cpp).

Before this, every wheel slid straight up and down the body's z axis. Camber gain was a
number typed into the definition, toe was constant, and the only thing a lateral tyre force
could do to the suspension was nothing: the slider is perpendicular to it. Roll centres,
anti-dive and anti-squat did not exist.

Now a wheel can be guided by a real linkage, and those effects come out of its geometry.

## What a linkage is

`LinkageDef` on a wheel's `SuspensionDef` describes one of:

| Type | Points |
|---|---|
| `Slider` (default) | none: the old behaviour, with `camberGain` and `staticToe` as constants |
| `DoubleWishbone` | lower arm (two inboard pivots, outboard ball joint), upper arm (likewise), tie rod (rack end, steering-arm end), spring (body mount, lower-arm mount) |
| `MacPherson` | lower arm, strut top mount on the body, tie rod, spring on the strut |

Points are in the body frame, relative to the wheel centre at the design position (the car as
drawn, at static ride height), for the left wheel of the axle. The right wheel mirrors y.
`makeDoubleWishbone()` and `makeMacPherson()` build common layouts from arm lengths, the
upright height, the kingpin offset, a side-view axis inclination (anti-dive) and where the
spring sits on the arm.

## What the engine does with it

**Once, at construction.** The mechanism is solved by Newton's method over the wheel's travel
(41 samples): for each wheel-centre height, the lower arm's angle, the upper arm's angle (or
the strut's extension) and the upright's rotation about its kingpin axis that keep every joint
and the tie rod at their lengths. Each sample stores where the wheel centre has scrubbed to,
the camber, the toe, the spring length, and their derivatives. A geometry that cannot reach the
travel, or folds over, makes the definition invalid with a reason, like any other bad
parameter. The physics step only interpolates the table; nothing allocates.

**Every step**, the wheel's single degree of freedom (its compression, solved implicitly as
before) now moves along the linkage's path instead of straight up:

- The wheel centre's direction per metre of compression, \(\mathbf d = (\partial x/\partial s,\
  \partial y/\partial s, 1)\), replaces the body's up axis in the wheel equation: generalised
  mass \(m|\mathbf d|^2\), gravity and the body's acceleration projected on \(\mathbf d\).
- The contact patch has its own path, \(\mathbf d_p = \mathbf d + \boldsymbol\omega\times(\mathbf
  p-\mathbf c)\): as the upright tilts along the camber curve the bottom of the tyre swings
  sideways by the tyre radius per radian. Tyre forces at the patch do work along \(\mathbf d_p\).
  **This is where the handling effects come from:**
  - a cornering force along a path with a sideways component jacks the suspension: on a car
    with its roll centre above the ground the outside wheel is pushed toward droop and the
    body is held up through the links, so it rolls less for the same springs and bars;
  - a braking force along a path that leans forward in bump pushes the front wheels toward
    droop: anti-dive. The same geometry on a rear axle is pro-squat, so rear axles use the
    opposite inclination for anti-squat.
- The spring's motion ratio \(\rho(s)\) (coil travel per wheel travel, normalised to 1 at the
  design position) scales the wheel rate by \(\rho^2\) and the damping by \(\rho^2\). `springRate`
  remains the wheel rate at design height; the geometry makes it rise or fall through the
  travel. The spring force is \(k\,\rho(s)\,(S(s)-S_{\text{droop}})\) with \(S=\int\rho\,ds\).
- Camber is `staticCamber` plus the curve's change from the design pose. Toe is `staticToe`
  plus the curve's change (bump steer, and with roll, roll steer). `camberGain` is ignored
  for a real linkage.
- The garage's ride height moves the operating point along the curves (`rideOffset`):
  lowering a car by 20 mm puts its wheels 20 mm further into bump at rest, and the camber
  that comes with it, until the alignment is corrected.

The front-view roll-centre height is reported per axle in telemetry (`rollCentreHeight`),
from the patch's path: \(h = t/2\,(\partial y_c/\partial s - r\,\partial\gamma/\partial s)\),
which has no singularity when the camber does not change.

## Checks (`tests/test_kinematics.cpp`)

| Check | Result |
|---|---|
| 3-D wishbone solve against an independent 2-D four-bar (circle intersection, bisection) | camber and scrub agree to 1.4·10⁻⁵ over ±80 mm |
| MacPherson solve against the 2-D construction | agree to 2·10⁻⁵ |
| Parallel equal arms | camber change < 10⁻⁷; a 30 mm raised rack gives bump steer of opposite sense in bump and droop |
| Roll centre from the path vs. the graphical construction (arm lines → instant centre → n-line) | 120.006 vs 120.000 mm; 55.90 vs 55.89 mm; 94.94 vs 94.94 mm |
| Steady 0.59 g turn, same springs and bars, roll centre at the ground vs. +95 mm | roll gradient 2.54 → 2.12 deg/g; lateral load transfer unchanged within 8 % |
| Braking from 80 km/h at 45 % pedal, level pivots vs. 0.30 rad anti-dive | dive 1.69° → 1.06°; stopping distance 30.46 vs 30.46 m |
| Lowered 20 mm in the garage | camber at rest −0.01° → −0.59°, the curve's value at 20 mm |
| Kerb strike at 60 km/h, 240 vs 1000 Hz | heave 13.1 vs 13.0 mm |
| 40 randomly perturbed linkages (±4 cm on every point) | all valid, none faults |

The reference cars (`referenceSportsRwd`, hatch, saloon, hypercar, race) keep sliders, so the
golden baseline is unchanged and stays a regression guard. The Rosser Stella V8 and the five
real-car approximations use linkages:

| Car | Front | Rear | Roll centres (front / rear) |
|---|---|---|---|
| Rosser Stella V8 | double wishbone, anti-dive | double wishbone, anti-squat | 112 / 103 mm |
| Porsche 911 Carrera S / 4S | MacPherson strut (published layout) | multilink, as a double wishbone | 57 / 86 mm |
| Mercedes-AMG S 63 | four-link, as a double wishbone | multilink, as a double wishbone | 84 / 87 mm |
| Mercedes-Benz S 600 Guard | as the S 63 | as the S 63 | 77 / 81 mm |
| Formula 1 (2026) | long parallel wishbones, pushrod (spring fraction 0.9) | likewise | 74 / 73 mm |

The layouts are the published types; the point coordinates are representative, not from any
drawing.

## Limits

- One degree of freedom per wheel: the linkage is rigid and its joints have no compliance.
  Bushings, compliance steer and the small longitudinal compliance of real arms are not
  modelled.
- The traction work term uses the previous step's tyre forces (one step lagged, like the body
  acceleration the wheel sees). For sliders it is not applied: their path is the body axis,
  the term is a few per cent of the brake force under pitch, and leaving it out keeps the
  golden baseline bit-identical and the 60 Hz-host rate check as it was.
- Anti-roll bars still couple the two wheels through the partner's previous-step compression.
- No solid axles, trailing arms or multilink geometries as such; a multilink axle is
  approximated by the double wishbone with the same camber and roll-centre behaviour.
- The camber curve rotates the upright; it does not yet tilt the contact samples, so the tyre
  still presses on a flat footprint.
