// OpenRoad Dynamics — engine-independent vehicle physics core.
// SPDX-License-Identifier: MIT
//
// Aerodynamic building blocks shared with the Aviation Dynamics core
// (github.com/ZRosserMcIntosh/Aviation-Dynamics, MIT, same author), adapted
// to cars: the standard atmosphere for air density (São Paulo sits at
// ~760 m, where the air is ~8% thinner than at sea level), a full-range wing
// section polar (attached lift, a smooth stall, Viterna-Corrigan post-stall)
// for rear wings and the Helmbold 3-D lift slope.
//
// References (as in Aviation Dynamics):
//  - U.S. Standard Atmosphere 1976 (troposphere: T = 288.15 - 0.0065 h,
//    p = p0 (T/T0)^(g0/(R L))).
//  - Helmbold equation, USAF DATCOM 4.1.3.2.
//  - Viterna & Corrigan (1982), NASA CP-2230 (post-stall).
//  - Katz, "Race Car Aerodynamics" (1995) for the car-specific use: inverted
//    wings, ride-height sensitive floors and their stall.
#pragma once

#include "ord/math.hpp"

namespace ord {

// Dry air density (kg/m^3) at a geometric altitude (m) in the ISA
// troposphere, with the actual air temperature (deg C) and the sea-level
// pressure (Pa, QNH).
Real airDensity(Real altitude, Real temperatureC, Real seaLevelPressure = 101325.0);
// ISA static pressure (Pa) at a geometric altitude (m) in the troposphere.
Real airPressure(Real altitude, Real seaLevelPressure = 101325.0);
// SAE J1349-style power factor of a naturally aspirated engine relative to
// the 99 kPa / 25 C reference: (p / 99 kPa) * sqrt(298 K / T).
Real enginePowerFactor(Real altitude, Real temperatureC);

// Wing section polar (2-D), for any angle of attack.
struct WingSectionPolar {
    Real liftSlope = 2 * kPi * 0.95;  // per rad
    Real zeroLiftAlpha = -0.07;       // rad (cambered: lifts at 0)
    Real clMax = 1.9;                 // high-lift race-wing section (single element)
    Real clMin = -0.8;
    Real cdMin = 0.012;
    Real cdQuadratic = 0.008;
    Real clAtCdMin = 0.5;
    Real stallWidth = 0.08;           // rad
    Real postStallClDrop = 0.3;
    Real flatPlateCd = 1.6;
};
struct WingSectionResult {
    Real cl = 0, cd = 0;
    bool stalled = false;
};
WingSectionResult evaluateWingSection(const WingSectionPolar& p, Real alpha, Real liftSlope3D);

// Helmbold/DATCOM 3-D lift-curve slope for aspect ratio AR (unswept).
Real liftCurveSlope3D(Real aspectRatio, Real a2d = 2 * kPi * 0.95);

}  // namespace ord
