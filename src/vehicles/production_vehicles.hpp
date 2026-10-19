// OpenRoad Dynamics — vehicle definitions approximating real production and racing cars.
// SPDX-License-Identifier: MIT
//
// Unlike reference_vehicles.hpp (fictional cars), these approximate real cars from their
// manufacturers' *published* specifications: mass, dimensions, engine output and torque band,
// gear ratios, tyre sizes, top speed. Everything not published (weight distribution, inertia,
// spring and damper rates, tyre coefficients, aero lift, F1 gear ratios and aero) is an
// engineering estimate and marked as such in the .cpp. They contain no manufacturer CAD,
// telemetry or other proprietary data, and are not endorsed by any manufacturer.
//
// Trademarks: Porsche, 911, Carrera, PDK, Mercedes-Benz, Mercedes-AMG, S 63, S 600 and GUARD,
// Formula 1 and F1 are trademarks of their owners. The names identify which published figures
// were used; a commercial game would need licences (or renamed, rebadged cars).
#pragma once

#include "ord/vehicle_def.hpp"

namespace ord::vehicles {

// Porsche 911 Carrera S (992.2, 2025): 3.0 L twin-turbo flat-six, 353 kW / 530 Nm,
// 8-speed PDK, RWD with an electronic rear differential lock, 245/35 R20 + 305/30 R21.
VehicleDefinition porsche911CarreraS();

// Porsche 911 Carrera 4S (992.2): as the Carrera S with Porsche Traction Management
// (multi-plate clutch to the front axle, rear-biased).
VehicleDefinition porsche911Carrera4S();

// Mercedes-AMG S 63 4MATIC+ long (W222 facelift, 2018-2020): 4.0 L twin-turbo V8,
// 450 kW / 900 Nm, AMG SPEEDSHIFT MCT 9-speed, variable AWD, limited to 250 km/h.
VehicleDefinition mercedesAmgS63();

// Mercedes-Benz S 600 Guard (W222, 2015): VR9 armoured, 6.0 L twin-turbo V12, 390 kW / 830 Nm,
// 7G-TRONIC, RWD, ~4.16 t, run-flat tyres, limited to 210 km/h.
VehicleDefinition mercedesS600Guard();

// A generic Formula 1 car to the 2026 technical regulations: 768 kg minimum (plus fuel),
// 3.4 m wheelbase, 1.6 L V6 turbo + 350 kW MGU-K, 8 gears, 18-inch slicks. No team.
VehicleDefinition formula1Car2026();

}  // namespace ord::vehicles
