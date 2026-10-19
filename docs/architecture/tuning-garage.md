# Tuning garage

Status: **first version.** Source: [`setup.hpp`](../../src/core/include/ord/setup.hpp),
[`setup.cpp`](../../src/core/src/setup.cpp), [`aero.hpp`](../../src/core/include/ord/aero.hpp),
tests in [`tests/test_garage.cpp`](../../tests/test_garage.cpp).

A `VehicleSetup` holds everything a tuning menu adjusts, in setup-sheet units (degrees, N/mm,
mm, bar, percent). `applySetup(stock, setup)` turns it into a new `VehicleDefinition`. The
physics stays the same; only the parts change. `setupToJson` / `setupFromJson` save and load
setups.

```cpp
VehicleSetup s = stockSetup(def);
s.front.camberDeg = -2.5;
s.front.compound = s.rear.compound = TyreCompound::SemiSlick;
s.brakeBalance = 0.62;
s.rearWing = true; s.wingAngleDeg = 12;
s.altitudeM = 760;                       // São Paulo
Vehicle car(applySetup(def, s), assists);
```

All the new models are **off in the reference vehicles** unless a setup is applied, so
`benchmarks/baseline.csv` is unchanged.

## What a setup changes

| Group | Settings | Model |
|---|---|---|
| Alignment | camber, toe, caster (front), camber gain per cm of bump | Each wheel's ground camber comes from its geometry: static camber, camber gain × travel, caster × steer, plus body roll (through the chassis orientation). Telemetry reports it per wheel. |
| Springs / dampers | spring N/mm, ride height mm, bump and rebound (slow and fast), ARB N/mm | Two-stage dampers: the slow coefficient applies up to the knee speed (0.1 m/s), the fast one beyond. The implicit unsprung solve linearizes them. Ride height moves the hardpoints. |
| Tyres | compound, pressure (bar) | See below. |
| Brakes | balance (front share), pressure (× stock torque) | Splits the total torque between axles. |
| Drivetrain | diff type (open, locked, clutch LSD) with preload and ramps, final drive, gear ratios, steering lock | Existing drivetrain model. |
| Aero | front / rear downforce scale, rear wing and its angle | See below. |
| Environment | altitude, air temperature, turbocharged | Air density from the ISA, and the SAE J1349 power correction for naturally aspirated engines. |

## Tyres

**Camber.** Lateral peak grip is highest when the tyre leans about 2° into the turn. It falls
by 1 − k·(γ − 2°)². Longitudinal grip is best at zero camber. Camber thrust (the motorcycle
Magic Formula term) adds a lateral force. So negative camber trades braking for cornering, as
on a real car:

| Static camber (front / rear) | 0 / 0 | −2 / −1.4 | −4 / −2.8 | −6 / −4.2 |
|---|---|---|---|---|
| 100–0 km/h (ABS) | 39.0 m | 39.5 m | 40.9 m | 42.6 m |

**Compounds** scale peak μ, cornering stiffness, curve shape, and the response to wet and loose
surfaces. They also set the temperature window, rolling resistance and relaxation length.
Warm tyres, sports RWD reference car, 40 m skidpad:

| Compound | Dry | 100–0 km/h | Wet (deep water) | Gravel |
|---|---|---|---|---|
| Off-road | 0.76 g | 49.0 m | 0.40 g | **0.62 g** |
| Street | 0.83 g | 39.0 m | 0.47 g | 0.50 g |
| Slick | **0.94 g** | **28.9 m** | 0.22 g | 0.37 g |
| Rain | 0.79 g | 39.1 m | **0.56 g** | 0.50 g |

All-season, sport and semi-slick sit between these.

**Temperature.** A one-node tread model:
- Heating: a fraction of the sliding power, plus rolling hysteresis.
- Cooling: toward ambient, faster with speed.
- Grip: 1 − loss·((T − T_opt)/window)², floored at 75% cold and 60% overheated.

Slicks (window centred at 95 °C) give 0.74 g from a cold start and 0.94 g warm. Cruising for
two minutes at 100 km/h keeps them near 27 °C. A minute of hard cornering takes the outside
front past 55 °C. Tyre warmers (`setTyreTemperatures`) start them hot.

**Pressure** relative to 2.2 bar:
- Higher pressure stiffens the carcass: cornering stiffness, vertical rate, shorter
  relaxation length.
- Peak grip falls either side of nominal (0.824 / 0.832 / 0.822 g at 1.6 / 2.2 / 2.8 bar).

## Aerodynamics

Ported from [Aviation Dynamics](https://github.com/ZRosserMcIntosh/Aviation-Dynamics) (MIT,
same author) and reduced to what a car needs:
- The US Standard Atmosphere (1976) for density.
- A full-range wing section polar: attached lift, a smooth stall, Viterna–Corrigan post-stall.
- The Helmbold 3-D lift slope.
- Induced drag C_L²/(π e AR).

Car-specific use follows Katz, *Race Car Aerodynamics* (1995).

- **Airspeed** is relative to the air, so wind (`Vehicle::setWind`) changes drag and
  downforce. Body downforce falls with yaw (cos²β). A side-force area pushes the car
  downwind.
- **Floor / body downforce** grows as the floor runs lower, (h_ref / h)^0.6. Below
  `floorStallHeight` the diffuser flow separates within about 15 mm, dropping the floor to
  40%. At 180 km/h: 1457 N stock, 1690 N lowered 40 mm, 686 N once the floor stalls.
- **Rear wing** (1.4 × 0.28 m when fitted by the setup). Its angle of attack is the set angle
  minus the local flow angle in body axes, so pitch and heave change it. At 200 km/h:

| Wing angle | 0° | 6° | 12° | 18° | 24° | 30° |
|---|---|---|---|---|---|---|
| Wing downforce | 211 N | 533 N | 855 N | 1174 N | 1394 N | 920 N (stalled) |
| Total drag | 1337 N | 1360 N | 1405 N | 1472 N | 1533 N | 1550 N |

- **Altitude.** ISA pressure with the actual temperature sets the air density. For a
  naturally aspirated engine, the SAE J1349 factor (p/99 kPa)·√(298 K/T) scales torque
  relative to sea level at 15 °C. A turbocharged car keeps its power. At 2500 m the wing
  makes over 20% less downforce and the engine over 20% less torque.

Telemetry: `TelemetryFrame::aero` gives airspeed, airflow yaw, front/rear downforce, drag,
wing downforce, floor clearance front/rear, and wing and floor stall flags. Per wheel:
`camber` and `temperature`.

## Limits

- Camber gain is linear in travel. Real suspension kinematics (hardpoints, roll centres,
  bump steer) come next ([roadmap](../roadmap.md) M2.3).
- The tyre thermal model has one node: no surface vs carcass, no wear, and pressure does not
  rise with temperature.
- Compound and aero numbers are representative, not data for any real tyre or wing. The tests
  pin the **orderings** a setup tool must get right, not absolute values.
- Wing and floor are quasi-steady. There is no wake interaction between them and the body.
