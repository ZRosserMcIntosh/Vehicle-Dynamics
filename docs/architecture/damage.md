# Damage: crash structure and mechanical damage

Source: [`damage.hpp`](../../src/core/include/ord/damage.hpp), [`damage.cpp`](../../src/core/src/damage.cpp),
the hooks in [`vehicle.cpp`](../../src/core/src/vehicle.cpp) (`enableDamage`, `stepDamage`, the wheel loop),
tests in [`tests/test_damage.cpp`](../../tests/test_damage.cpp) and [`tests/test_alloc.cpp`](../../tests/test_alloc.cpp).

Off by default. `Vehicle::enableDamage(DamageConfig)` builds the structure; a car without it behaves
exactly as before (rigid body, [body hull](rollover.md)), and the golden baselines are untouched.
With it on and nothing near, driving is bit-identical and makes no extra ground queries (tested).

## 1. The crash structure

**Lattice.** Built from the same box as the body hull (overhangs, track, floor, belt line, roof and
greenhouse) and the car's [`CrashStructure`](crash-survivability.md):

- stations along the car: three cells in the front crush zone, ~0.6 m cells along the passenger
  cell, three cells in the rear zone. The zones reach at least 0.3 m behind the front axle
  (the firewall) and 0.25 m ahead of the rear axle, so a hard hit moves the wheels' mounts;
- four columns across: the outer skin at ±half width and the cell wall at ±(half width − side crush);
- two levels (floor, belt) and roof nodes over the greenhouse;
- ~110 nodes and ~380 beams for a car; fixed arrays (128 / 640), nothing allocates.

The **passenger cell is the rigid body**: its nodes (the inner columns between the firewall and
the rear bulkhead) are anchored to it. Everything else is a free node with a share of
`nodeMassFraction` (5%) of the mass. Tyre-shoulder points are extra anchored contact nodes, as in
the hull, and go away with their wheel.

**Beams** are elastic-plastic. Each has an elastic stiffness (1% strain at its full strength), a
yield force and linear hardening. Past yield, the rest length moves so the beam is left carrying
its yield force (plastic flow); crushed to 15% of its length it is solid (bottomed out); stretched
to 145% it tears. The XPBD damping term (Macklin et al. 2016, eq. 26) gives each beam ~30% of
critical damping for an eighth of the car's mass, so a car resting on the structure does not rock.

**Strengths from the CrashStructure.** A zone of crush depth d and natural frequency ω behaves like
a spring k = mω² whose force reaches k·d at full crush. The 8 axial members of a zone (rails)
take half of that (`kZoneShare`, the diagonals and cross members carry the rest as the zone
folds), each yielding at 15% of its share and hardening linearly to it. Side beams follow the
side zone the same way (sills 1.5× stronger); the roof's pillars yield at `roofStrength` (3) × the
car's weight in total, FMVSS 216's requirement, and double over 25 cm.

**Solver.** XPBD (Macklin, Müller, Chentanez, "XPBD: position-based simulation of compliant
constrained dynamics", 2016) with the rigid body taking part through its generalised inverse mass
at each anchored point, 1/m + (r×n)·I⁻¹(r×n) (Müller et al., "Detailed rigid body simulation with
extended position based dynamics", 2020). Each vehicle step, after the chassis is integrated:

1. contacts are detected where the nodes will be at the end of the step (ground by ray, the host's
   obstacles by `ObstacleQuery::point`, posts and wall corners poking between nodes by
   `ObstacleQuery::edge` on the skin edges) and kept as planes;
2. the body restarts from its start-of-step pose with the step's velocity change from its own
   forces (tyres, aero, gravity) applied a share per substep;
3. 16 substeps × 2 iterations ("Small steps in physics simulation", Macklin et al. 2019: substeps
   converge a stiff structure where iterations do not), each: predict, solve beams and contacts,
   update velocities, then plasticity from each beam's multiplier (its force is −λ/h², not its
   leftover strain, which a finite solve overstates);
4. the body leaves with the corrected pose and velocity; the corrections are part of the step's
   acceleration (the occupants feel the real pulse).

Gauss-Seidel in a fixed order leans a symmetric crash to one side, so the sweeps alternate
direction and beams and contacts are stored in **mirror pairs**: each pair is solved left-first,
then right-first. A symmetric frontal now leaves the steering centred and the two front wheels
with mirrored toe (tested).

**The cell past the zones.** A cell node that reaches an obstacle means the zone in front of it
is used up. Past its crush limit the structure stiffens: those contacts get a compliance of a zone
at 2.5× the front frequency, shared by the cell's front nodes, loading along it and unloading along
a line 20× stiffer (crushed metal does not push back). How far the cell went in is the intrusion.

**Overlaps that are already there.** A node can start a substep inside the ground: the
structure woke up with the car already sunk in (a car tilting into a concave curve, where the
overhangs sink below the surface by about x_f x_r / 2ρ, as on the loop), or a node was dragged in.
Removing that in one substep (1/5760 s) would turn centimetres into a launch, so an existing
overlap is pushed out at no more than 2 m/s, while the penetration a substep's own motion makes is
corrected in full (nothing tunnels). Before this, the loop with damage on threw the car 985 m up.

**Thin walls.** A wall with no thickness pushes a node toward the side it came from: the node's
own position at the start of the step, a quarter of the way in toward the car's centre (not the
centre itself, which is already past the wall when the car is half through a hole in it).

**Sleep.** The structure is solved only when the host reports an obstacle near (`anyNear`, a
broadphase), the car is tilted past ~30° or airborne (when the body hull would be active), or a
wheel is gone. Otherwise the free nodes ride with the body at their deformed shape and nothing is
queried. Awake with nothing touching and the structure settled (driving past a wall, airborne),
it is not solved either: the nodes ride with the body (6 s beside a wall: one solve). Solving
costs ~0.4-0.7 ms per step in WebAssembly, only while something touches. With the structure on,
its nodes replace the hull as the car's collision with the ground.

**Calibration** (reference saloon, 1,800 kg, ω = 32 rad/s front, full-width rigid wall, coasting;
`tests/test_damage.cpp` prints the table):

| Speed | Crush | Peak | Pulse | Structure's share of the energy |
| --- | --- | --- | --- | --- |
| 15 km/h | 0.08 m | 11 g | 58 ms | ~100% |
| 30 km/h | 0.22 m | 16 g | 75 ms | ~100% |
| 56 km/h | 0.51 m | 27 g | 92 ms | 100% |
| 80 km/h | 0.91 m | 39 g | 119 ms | 89% (the rest in the cell's crush) |

The linear-spring estimate at 56 km/h is δv/ω = 0.49 m over π/ω = 98 ms; published full-width
56 km/h tests show 0.5-0.7 m and 25-40 g. Rebound stays under 2 m/s (restitution ~0.1). The
hatch, sports car, 911, S 600 Guard and Stella all crush 0.50 ± 0.01 m at 56 km/h because their
CrashStructures are the default: give a car its own frequencies and depths and the structure
follows. A 254 mm pole at 32 km/h (Euro NCAP) pushes the door 0.35 m with 5 cm of intrusion;
upside down from 0.3-2 m the roof crushes 0.16-0.27 m and the car comes to rest carrying its weight.

## 2. Mechanical damage

Read from the deformed lattice (a Gaussian-weighted average of node displacements around a point,
anchored nodes counting as zero) whenever its shape changes, and from the wheels' loads each step:

| Part | Cause | Effect |
| --- | --- | --- |
| Suspension mounts | lattice displacement at the hardpoint and ±0.3 m fore/aft, 0.3 m below | the wheel moves (`mountOffset`), toe (front/rear difference), camber (top/bottom difference) |
| Wheel | mount moved > 0.35 m, or a load > 30× its static share | torn off: no contact, no force; the corner rests on the structure |
| Suspension arm | load 15-30× static (a kerb strike) | bent: toe-out added |
| Tyre | deflection past 85 mm at speed (pinch flat on the rim), or > 60 kN on an obstacle at the wheel (cut) | deflates over 4 s: radius −18%, vertical stiffness −85%, grip −45%, rolling resistance ×11 |
| Steering rack | lattice displacement at the front axle, floor height | sideways: steering offset (the car pulls); pushed back: lock lost (down to 25%) |
| Radiator | crush 12 cm behind the bumper | coolant leaks (all of it in 45 s when destroyed) |
| Engine | coolant gone: temperature rises with load (thermostat 90 °C); crushed engine bay | power falls from 115 °C to half at 130 °C; seizes past 135 °C (dead, drags); crushed bay costs up to 70% of the torque |
| Aero | crush front / sides / roof; rear crush > 25 cm | drag up to ×1.75; the rear wing stops working |

`Vehicle::repairDamage()` puts everything back: shape, tyres, steering, engine (tested to crash the
same way again).

Tested: an offset hit bends that corner and the car no longer drives straight; a destroyed
radiator costs power after ~30 s of full throttle and seizes the engine after ~40 s; a 20 cm kerb
at 70 km/h pinch-flats the left tyres and the car pulls left; a 90 km/h offset hit tears the front
wheels off and the car still drives on three; random crashes (cars, speeds, yaw, spin, posts and
walls) never fault; a copy taken mid-crash continues bit-identically (rewind).

## For hosts

- Implement `ObstacleQuery` for your static world (walls, barriers, posts) and pass it with
  `setObstacles`. `point()` returns the deepest penetration of a point, with thin walls pushing
  toward the side of `ref` (the car's centre of mass); `edge()` handles posts and wall corners
  that fit between nodes.
- `DamageModel::impact(i)` lists what was hit each step with its impulse and the work done
  (breakable walls, sounds, particles).
- Draw the body from `DamageModel::node(i)`: `rest` (as built) and `local` (as deformed), body
  frame. Skin the mesh to the nodes (each vertex to its nearest few rest positions).
- The structure is stepped with `ChassisMode::Internal` only.

## Per car

`BodyDef::structureStrength` scales every beam (the S 600 Guard: 1.8, it crushes 0.36 m at
56 km/h where the S 63 crushes 0.51 m) and `BodyDef::runFlatTyres` keeps a punctured tyre at a
quarter flat (the Guard's PAX run-flats).

## Limits

Rollover with the structure on is not the hull's rollover: its nodes are a different shape
(no belt-line points at the ends, roof nodes on the cell's width), so marginal cases end
differently (released at 70 degrees, the hull ends on its side, the structure on its roof; a
35 km/h kerb trip rolls further). The rollover tests run with damage off.

A coarse lattice (~110 nodes), one-dimensional beams, no panels, welds or material model, a rigid
passenger cell (its crush is a contact compliance, not geometry), no car-to-car contact (obstacles
are static), and parts damaged by simple rules from the lattice and the loads. A game damage model
on a physical method, calibrated against the crash structure, not a finite-element crash
simulation. No third-party code: the methods are from the cited papers.
