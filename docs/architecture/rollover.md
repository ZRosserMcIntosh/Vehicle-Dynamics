# Rollover

Source: [`vehicle.cpp`](../../src/core/src/vehicle.cpp) (hull build in the constructor, body
contact in `computeStep`), [`BodyDef`](../../src/core/include/ord/vehicle_def.hpp), tests in
[`tests/test_rollover.cpp`](../../tests/test_rollover.cpp).

Before this, only the tyres touched the ground. A car tipped onto its side was held up by tyre
contacts found "straight down" its own tilted axis, so it could never land on its side or roof.

## Body hull

Each car gets a collision hull of up to 21 points in body coordinates:

- the box corners at the floor and at the belt line, plus the middle of each side (12);
- a narrower, shorter roof (the greenhouse: 80% of the width, 30–65% of the length) with
  four corners and a centre point (5);
- each tyre's outer shoulder (one per wheel): what a car pivots on as it tips.

`BodyDef` sets the overhangs, half-width, height, floor height and friction. Every field left at
0 is derived from the wheels: 0.9 m overhangs, half-width = track/2 + 0.12 m, the floor 13 cm
above the static ground, the roof 1.15 m above the floor. Body friction defaults to 0.5 (metal
and glass sliding on asphalt) and is scaled by the surface.

## Contact

The hull is only checked when the car is tilted more than ~31° (`up.z < 0.86`) or has no wheel
on the ground. Normal driving does not query it and costs nothing extra. Each point below the
ground gets a stiff, damped normal force plus sliding friction. The stiffness and damping are
capped by the effective mass that point sees (mass and inertia about the CoM), so the explicit
integration stays stable for any definition.

Past ~53° of tilt (`up.z < 0.6`) a tyre's support fades out, and it is gone by ~70°. Its contact
ray along the car's own −z then points well off to the side and would prop the car back up. The
tilt is measured against the vertical, not the local ground normal, because kerb edges have
steep local normals under an upright car.

Telemetry: `upright` (body z-axis · world z, so −1 is on the roof), `bodyContacts` and
`bodyContactForce`.

## What it does (tests)

| Case | Result |
|---|---|
| Released on its roof | settles with up = −1.00; ~5 hull points carry its weight |
| Released tilted (static tipping) | sports car stays up at 55° and falls at 75°; S 600 Guard (tall, 4.2 t) falls at 65°; F1 car stays up at 65° |
| Sliding sideways into a 25 cm kerb | a bump at 15 km/h, over the roof at 45 km/h |
| Sliding sideways on flat asphalt at 80 km/h | no roll: tyre grip alone can't trip a car this low |
| Normal driving | the hull never touches |

## In the browser demo

- The proving ground has a rollover area (`ord_roll_ramp_lane_y`, `ord_roll_kerb_y`):
  - a one-sided ramp that lifts the left wheels 0.9 m over 10 m and ends in a drop (a
    corkscrew): the S 600 Guard goes over from ~70 km/h, the sports car from ~90 km/h, and the
    F1 car not even at 120 km/h (checked in `web/smoke_test.mjs`);
  - a 25 cm trip kerb to slide sideways into.
- Wall and post hits are 3D impulses through the full inverse inertia, applied at the CoM height
  clamped to the obstacle's height. A low obstacle is hit below the CoM and can trip the car.
- The state buffer carries `upright`, the hull contact count and the hull force (header 36–38).

## Limits

- Rigid body: no crush of the roof or pillars, and nothing breaks off. Crash survivability does
  not score rollovers.
- The hull is coarse (21 points). An edge can sink between points on a sharp ridge.
- Tyre sidewall forces are not modelled. Between ~53° and ~70° the tyre's support fades out
  linearly rather than following a real sidewall.
