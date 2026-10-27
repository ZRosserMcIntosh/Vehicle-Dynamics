# Standing water and hydroplaning

Source: [`tire.hpp`](../../src/core/include/ord/tire.hpp) (`hydroplaningSpeed`, `hydroplaneLift`,
`waterDrag`), [`tire.cpp`](../../src/core/src/tire.cpp), the wheel loop in
[`vehicle.cpp`](../../src/core/src/vehicle.cpp), tests in
[`tests/test_water.cpp`](../../tests/test_water.cpp).

A wet road already had less grip (`frictionScale`, `wetness` for the compound's wet grip, and a
crude speed falloff). What it lacked was water deep enough to stand on: puddles, ruts, a flooded
underpass. `SurfaceProperties::waterDepth` (m) is that, and three things follow from it.

## Hydroplaning

The water wedge in front of the contact patch carries a dynamic pressure ½ρv²C_L. When it reaches
the tyre's inflation pressure p the tyre lifts onto the water:

  v_p = √(2p / (ρ_w C_L)),  C_L = 0.643

which is Horne's NASA rule v_p = 6.35 √p[kPa] km/h (Horne & Dreher, NASA TN D-2056, 1963) for
water deeper than the tread can carry away: 94 km/h at 220 kPa, 110 km/h at 300 kPa.

How much of the patch rides on water, 0..1:

  lift = depth(h) · smoothstep((v / v_p − 0.5) / 0.5)
  depth(h) = clamp((h − h₀) / (h₁ − h₀)),  h₀ = 0.5 mm + 0.1 d,  h₁ = 1 mm + 0.4 d

with d the tread depth. Nothing below half of v_p, all of it at v_p; nothing in water the grooves
clear (a fresh 8 mm tread clears 1.3 mm, a worn 1.6 mm tread 0.66 mm, a slick 0.5 mm). The tyre's
grip is scaled by 1 − 0.95 · lift (a water film keeps ~5%).

`TireParams::inflationPressure` (kPa, default 220, set by the garage's pressure) and
`treadDepth` (m, default 7 mm; the tyre generator gives 8 mm road tyres and 0 for slicks; the
garage compounds go from 12 mm off-road to 10 mm rain tyres, 2 mm semi-slick, 0 slick).

## Displacement drag

Pushing the water aside costs a force on the tyre:

  F = ½ ρ_w C_D w h v²,  C_D = 0.75

w the tyre width (2 × patchHalfWidth), h the depth: the method aircraft certification uses for
contaminated runways (EASA CS-25 AMC 25.1591). It acts on the car at the contact patch against the
rolling direction whatever the tyre's grip, so water under one side pulls the car toward it.

## Spin-down

On water the lifted share of the load acts at the front of the patch, where the wedge is, not
under the axle: a torque −lift · F_z · a (a = half the patch length) on the wheel's spin. A
hydroplaning wheel that nothing drives spins down, the effect NASA's tests used to detect
hydroplaning.

## Checks

| Check | Result |
|---|---|
| v_p against Horne's rule at 150, 220, 300 kPa | within 0.5% |
| Lift needs both depth past the tread and speed past v_p/2; a worn tyre lifts first | as specified |
| Steering into 10 mm of water (reference sports car) | 50 km/h: 0.36 vs 0.37 rad/s dry; 130 km/h: 0.007 vs 0.34 rad/s; front tyres 33 m/s short of the road speed (spun down) |
| A 20 m puddle under one side at 100 km/h, hands off | 4.1° toward the water, mirror-symmetric |

The golden baseline is unchanged: every default is zero water.

## In the browser demo

`ord_set_weather(rain)` (0..1): wet asphalt keeps ~75% of its grip and carries a 1.2 mm film;
cobblestones lose 45% and painted lines 55% (both notoriously slick); soil turns to mud; puddles
6–20 mm deep form on ~30% of 22 m cells (on roads only, in the city). An aquaplaning lane has an
8 mm pool across it and a 12 mm puddle under the left wheels. See [web/README.md](../../web/README.md).

## Limits

- The lift ramps with speed rather than following a measured partial-hydroplaning curve; tread
  pattern, tyre width and load only enter through the tread depth and the drag.
- The water film is uniform over a surface; ruts and drainage are not modelled.
- Spray, and water thrown onto the car, are drawn by the demo but cost nothing.
