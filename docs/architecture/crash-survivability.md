# Crash survivability

Status: **first version.** Source: [`crash.hpp`](../../src/core/include/ord/crash.hpp),
[`crash.cpp`](../../src/core/src/crash.cpp), tests in
[`tests/test_crash.cpp`](../../tests/test_crash.cpp). In the browser demo, two crash test
dummies ride in the front seats and a report follows every impact.

> "Speed has never killed anyone. Suddenly becoming stationary, that's what gets you."

The speed change of a crash (**delta-V**) fixes how much the occupant must slow down. **How
suddenly** it happens (the crash pulse) and **what the occupant hits** on the way decide the
injuries. The model follows that chain.

## 1. The crash pulse

A game collision solver changes a car's velocity within one step (an impulse), which says
nothing about how long the stop took. So each impact's delta-V is turned into the pulse a
crumple zone would produce: the response of a linear crush spring, a half sine of duration
T = π/ω and area delta-V (peak delta-V·ω/2). ω is set per zone:

| Zone | ω (rad/s) | Pulse | Crush space |
|---|---|---|---|
| Front | 32 | 98 ms | 0.9 m |
| Rear | 28 | 112 ms | 0.7 m |
| Side | 45 | 70 ms | 0.3 m |

The crush the pulse needs is delta-V·π/(2ω). When that exceeds the zone's crush space, the
structure goes solid: the pulse gets harsher, and the excess becomes **intrusion** (the dash comes
back, or the door comes in). Because a side has a third of a front's crush space, the same delta-V
is harsher from the side, which is why side impacts hurt at lower speeds.

## 2. The occupants

Each seat holds a torso and a head, moving in the car's x and y relative to the seat, stepped at
0.1 ms:

- **Torso**: a seat-belt spring after 3 cm of slack, the airbag from 15 cm forward, and the seat
  back behind. Sideways: the door on the near side, the console or the other occupant on the far
  side, a weak lateral hold from the belt and, unbelted, the knee bolster.
- **Head**: on a neck spring (~2.5 Hz). It reaches the head airbag or the curtain airbag, or,
  past the free space, the steering wheel, dash, pillar or window. That is "hard contact": stiff
  and lightly damped.
- Normal driving accelerations feed in as well, so the dummies sway in corners. A hard landing or
  bump over ~12 g opens an event even without an impact.

## 3. Metrics and risk

These are the measures and published curves NHTSA and NCAP use:

| Metric | Definition | Reference |
|---|---|---|
| HIC15 | max over windows ≤ 15 ms of (t2 − t1)·(mean head acceleration in g)^2.5; limit 700 | FMVSS 208 |
| Chest 3 ms clip | highest torso acceleration held for a contiguous 3 ms (FMVSS 208 counts cumulative time above a level, which reads higher on a spiky trace); limit 60 g | FMVSS 208 |
| Serious head injury (AIS3+) | Φ((ln HIC15 − 7.45231) / 0.73998) | Eppinger et al., NHTSA 1999 |
| Life-threatening injury (AIS4+) | head 1/(1+e^(5.02−0.00351·HIC)), chest 1/(1+e^(5.55−0.0693·A3ms)), joined as 1 − (1−Ph)(1−Pc) | NCAP before 2011 |
| Delta-V fatality statistic | (delta-V / 71 mph)^4 | Joksch 1993 |

The Joksch figure is a population statistic over real crashes. It is shown next to the model's
own numbers as an independent cross-check, not combined with them.

The verdict, from the AIS4+ risk (and the AIS3+ head risk for the mildest bands):

| AIS4+ risk | Verdict |
|---|---|
| < 3% | walks away |
| < 8% | bruised |
| < 25% | injured |
| < 60% | seriously injured |
| 60% or more | critical |

## Results (`tests/test_crash.cpp`)

Rigid barrier, delta-V including 10% rebound; belted, front and curtain airbags unless stated.

| Case | delta-V | Chest 3 ms | Head | HIC15 | AIS4+ risk | Verdict |
|---|---|---|---|---|---|---|
| 10 km/h frontal | 11 km/h | 6 g | 8 g | 2 | 1.3% | walks away |
| 30 km/h frontal | 33 km/h | 20 g | 30 g | 69 | 2.3% | walks away |
| **56 km/h frontal** (the NCAP test speed) | 62 km/h | 41 g | 47 g | 216 | 7.5% | bruised |
| 80 km/h frontal | 88 km/h | 66 g | 138 g | 1505 | 68% | critical |
| 110 km/h frontal | 121 km/h | 249 g | 350 g | 17,000 | 100% | critical |
| 56 km/h frontal, unbelted, no airbag | 62 km/h | 48 g | 211 g (hits the wheel) | 4738 | 100% | critical |
| 30 km/h side, near side | 30 km/h | 28 g | 37 g | 110 | 3.6% | bruised |
| 30 km/h side, near side, no airbags | 30 km/h | 52 g | 99 g (hits the door) | 703 | 19% | injured |
| 30 km/h side, far side | 30 km/h | 11 g | 11 g | 6 | 1.5% | walks away |
| Helmeted rider into a wall, 30 km/h | 30 km/h | 52 g | 70 g | 449 | 15% | injured |
| Helmeted rider into a wall, 60 km/h | 60 km/h | 133 g | 153 g | 3198 | 100% | critical |

**It's the sudden stop.** At the same 40 km/h, a crush pulse lasting 157, 98 or 52 ms gives a
chest of 19, 31 or 45 g and a HIC of 60, 119 or 192.

**Calibration target.** The 56 km/h belted frontal lands where NCAP reports modern cars: chest
30–50 g, HIC 100–500. The tests pin these orderings:
- speed;
- pulse harshness;
- belted vs. unbelted;
- near vs. far side;
- curtain airbag vs. none;
- car vs. rider.

## In the browser demo

- **Cars**: every collision's delta-V, in the car's frame, goes to both front-seat dummies. The
  driver sits on the left, since Brazil drives on the right.
- **Settings**: seat belts and airbags can be switched off.
- **Crash-test lane**: the test track has a rigid barrier and a 30 cm pole.
- **Bikes**: on a crash the rider gets a report too:
  - into a wall at the bike's speed;
  - over the bars or looped: head and shoulders first;
  - low-side: dropped from the seat.
  The cyclist rides without a helmet; the e-scooter and motorcycle riders wear one.

## Limits

- A game estimate built from published relations, **not a validated crash simulation.**
- Two point masses per occupant. No neck, femur or tibia metrics. No rollover or vertical
  (spinal) injury. The head's mass doesn't load the torso.
- Structure stiffness is per zone, not per vehicle design. The car's own collision response is
  still the demo's rigid impulse; only the occupants see the crush pulse.
- Unbelted with an airbag comes out close to belted. The lumped torso can't submarine or slide
  out of position the way a real unbelted occupant does.
- Rider falls use one representative impact. The slide that follows decelerates at ~0.5 g and
  isn't treated as injury: road rash isn't modelled.

## References

- NHTSA, FMVSS No. 208 *Occupant crash protection* (HIC15 ≤ 700, chest 3 ms ≤ 60 g).
- R. Eppinger et al. (NHTSA), *Development of improved injury criteria for the assessment of advanced automotive restraint systems II*, 1999; head AIS3+ risk Φ((ln HIC − 7.45231)/0.73998).
- NHTSA NCAP (pre-2011) injury-risk functions: head 1/(1+e^(5.02−0.00351·HIC)), chest 1/(1+e^(5.55−0.0693·A)).
- H.C. Joksch, "Velocity change and fatality risk in a crash: a rule of thumb", *Accident Analysis & Prevention* 25(1):103–104, 1993.
- M. Huang, *Vehicle Crash Mechanics*, CRC Press, 2002 (crash pulses, spring-mass occupant models).
