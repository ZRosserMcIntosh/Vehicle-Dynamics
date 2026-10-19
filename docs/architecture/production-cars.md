# Real-car approximations

Source: [`production_vehicles.hpp`](../../src/vehicles/production_vehicles.hpp),
[`production_vehicles.cpp`](../../src/vehicles/production_vehicles.cpp), tests in
[`tests/test_production.cpp`](../../tests/test_production.cpp).

The reference vehicles are fictional. These five cars approximate real ones. Every value is
either **published** (manufacturer technical data, press releases, the FIA regulations) or an
**engineering estimate**, and the source file marks which is which. The estimates (weight
split, inertia, springs and dampers, tyre coefficients, aero lift, F1 gear ratios and aero)
were tuned so that each car reproduces its published acceleration and top speed. No
manufacturer CAD, telemetry or other proprietary data is used.

## What each car is built from

| Car | Published inputs | Main estimates |
|---|---|---|
| Porsche 911 Carrera S (992.2) | 1,553 kg (US curb); wheelbase 2,450 mm, tracks 1,601 / 1,555 mm; 353 kW at 6,500 rpm, 530 Nm at 2,300–5,000 rpm, 7,500 rpm; 8-speed PDK ratios, final drive 3.12; 245/35 R20 + 305/30 R21; Cd 0.30 × 2.07 m² | 38/62 weight split, springs, anti-roll bars (set toward the tested ~1 g skidpad), aero drag including cooling |
| Porsche 911 Carrera 4S (992.2) | as above, 1,585 kg | Traction Management modelled as a 25/75 split with a slipping centre clutch |
| Mercedes-AMG S 63 4MATIC+ (W222) | 2,180 kg (US curb); wheelbase 3,165 mm, tracks 1,643 mm; 450 kW at 5,500–6,000 rpm, 900 Nm at 2,750–4,500 rpm; MCT 9-speed ratios, final drive 2.82; 255/40 R20 + 285/35 R20; 250 km/h limit | 53/47 split, 30/70 variable AWD, Cd·A 0.66 m², limiter rpm |
| Mercedes-Benz S 600 Guard (W222) | ~4.16 t, VR9 armour; 390 kW, 830 Nm from 1,900 rpm; 7G-TRONIC; RWD; Michelin PAX run-flats; 210 km/h limit | gear ratios (standard 7G-TRONIC), final drive, tyre size, very stiff springs |
| Formula 1 car (2026 regulations) | 768 kg minimum incl. driver; wheelbase ≤ 3.4 m, width 1.9 m; 1.6 L V6 turbo + 350 kW MGU-K; 8 gears; 18-inch tyres about 280/705 mm front and 375/710 mm rear | combined ~750 kW torque curve, gear ratios, Cl·A 4.2 m² and Cd·A 1.25 m² (corner mode) |

## How they compare (Sport preset, flat dry road)

| Car | 0–100 km/h | Published | Top speed | Published |
|---|---|---|---|---|
| 911 Carrera S | 3.8 s | 3.5 s (3.3 s Sport Chrono) | 316 km/h | 308 km/h |
| 911 Carrera 4S | 3.3 s | 3.5 s (3.3 s) | 315 km/h | 308 km/h |
| AMG S 63 4MATIC+ | 3.2 s | 3.5 s | 248 km/h (limited) | 250 km/h |
| S 600 Guard | 6.7 s | not published | 209 km/h (limited) | 210 km/h |
| F1 (2026) | 2.6 s | ~2.6 s (typical) | 312 km/h | ~340 km/h in low-drag mode |

## Limits

- Not modelled: launch control, turbo lag, the 911's optional rear-axle steering, adaptive
  dampers and air suspension levels, active aero (the F1 car is always in corner mode), and
  hybrid energy limits (the F1 car can deploy its full 350 kW indefinitely).
- The S 600 Guard's 0–100 km/h time is not published. Armoured relatives quote 7.9–8.3 s, so
  its estimated drivetrain may be optimistic.
- Trademarks: Porsche, 911, Carrera, PDK, Mercedes-Benz, Mercedes-AMG, S 63, S 600, GUARD,
  Formula 1 and F1 belong to their owners. The names here only identify which published figures
  were used. A commercial game would need licences, or renamed and rebadged cars.
