// OpenRoad Dynamics — reference vehicle definitions.
// SPDX-License-Identifier: MIT
//
// Generic, fictional vehicles with plausible parameters for benchmarking.
// They are NOT models of any real car and contain no manufacturer or game
// data. Values are round numbers in typical ranges from textbook sources
// (e.g. Milliken & Milliken, "Race Car Vehicle Dynamics", 1995; Gillespie,
// "Fundamentals of Vehicle Dynamics", 1992).
#pragma once

#include "ord/vehicle_def.hpp"

namespace ord::vehicles {

// ~1450 kg, ~300 kW turbocharged RWD sports coupe with a clutch-pack LSD.
VehicleDefinition referenceSportsRwd();

// ~1250 kg, ~110 kW FWD hatchback with an open differential.
VehicleDefinition referenceHatchFwd();

// ~1600 kg, ~280 kW AWD saloon, 40/60 open centre split, rear LSD.
VehicleDefinition referenceSaloonAwd();

// ~750 kg, ~450 kW open-wheel race car with very high downforce
// (Cl*A 3.6 m^2 total) on slick tires. Fictional.
VehicleDefinition referenceRaceDownforce();

// ~1500 kg, ~1100 kW AWD hypercar geared for ~450 km/h. Fictional.
VehicleDefinition referenceHypercarAwd();

// Rosser Stella V8: ~1480 kg mid-engine RWD, flat-plane V8, ~430 kW, 9,000 rpm (game tune),
// 7-speed dual clutch. The game's own fictional car (São Paulo World).
VehicleDefinition rosserStellaV8();
// The six-speed gated manual version (index 11 in the web build).
VehicleDefinition rosserStellaV8Manuale();

}  // namespace ord::vehicles
