// OpenRoad Dynamics — engine-independent vehicle physics core.
// SPDX-License-Identifier: MIT
//
// Data-driven vehicle definition. A VehicleDefinition fully describes the
// *physical* car. Driver assists and input interpretation live elsewhere
// (assists.hpp) and can never modify these values: presets change how the
// driver's intent is turned into actuator commands, not the car itself.
#pragma once

#include <string>
#include <vector>

#include "ord/aero.hpp"
#include "ord/drivetrain.hpp"
#include "ord/kinematics.hpp"
#include "ord/math.hpp"
#include "ord/tire.hpp"

namespace ord {

struct SuspensionDef {
    // Wheel-centre position at full bump (maximum compression), body frame,
    // relative to the body origin. The wheel travels along -Z_body from here.
    Vec3 hardpoint;
    Real travel = 0.20;            // m, full droop to full bump
    Real springRate = 35000.0;     // N/m at the wheel
    Real springPreload = 0.0;      // N at full droop
    Real bumpDamping = 3000.0;     // N*s/m, compression
    Real reboundDamping = 4500.0;  // N*s/m, extension
    // Two-stage dampers (as on adjustable coilovers): the coefficients above
    // apply up to damperKneeSpeed; past it the fast coefficients (0 = same).
    Real bumpDampingFast = 0.0;    // N*s/m
    Real reboundDampingFast = 0.0; // N*s/m
    Real damperKneeSpeed = 0.1;    // m/s
    Real bumpStopRate = 300000.0;  // N/m beyond full bump
    // The linkage that guides the wheel (kinematics.hpp). The default slider
    // travels straight along body z and uses camberGain / staticToe as
    // constants; a double wishbone or MacPherson strut derives the camber and
    // toe curves, the wheel's path (roll centre, anti-dive) and the spring's
    // motion ratio from its geometry. springRate stays the wheel rate at the
    // design position; the motion ratio makes it rise or fall through the travel.
    LinkageDef linkage;
};

struct WheelDef {
    SuspensionDef suspension;
    TireParams tire;
    Real maxSteerAngle = 0.0;      // rad at full steering lock (0 = not steered)
    Real staticToe = 0.0;          // rad, + = toe-in on the left wheel convention handled per side
    // Camber (rad), the usual sign on both sides: negative = top of the wheel
    // leaning in toward the car's centre line. Static value at ride height,
    // plus camberGain per metre of compression away from ride height (a
    // double wishbone typically gains negative camber in bump: -0.3..-1
    // rad/m). Body roll adds to it through the geometry.
    Real staticCamber = 0.0;
    Real camberGain = 0.0;         // rad per m of compression
    // Caster (rad, steered wheels): steering tilts the outside wheel's top
    // in by about sin(caster) * sin(steer), the inside wheel's out.
    Real casterAngle = 0.0;
    Real maxBrakeTorque = 2000.0;  // N*m at 100% pedal
    Real handbrakeTorque = 0.0;    // N*m at 100% handbrake
    Real unsprungMass = 40.0;      // kg: wheel, tire, hub, brake, half the links. Part of VehicleDefinition::mass.
    bool isLeft = true;
};

struct AxleDef {
    int leftWheel = 0, rightWheel = 1;
    Real antiRollBarRate = 0.0;    // N/m at the wheel (per metre of travel difference)
};

struct AeroDef {
    Real dragArea = 0.70;          // Cd*A, m^2
    Real frontDownforceArea = 0.0; // Cl*A at the front axle, m^2 (positive = downforce)
    Real rearDownforceArea = 0.0;  // Cl*A at the rear axle
    Real airDensity = 1.225;       // kg/m^3 (sea level, 15 C)
    Real ambientTemperature = 25.0;  // deg C, for the tyres' thermal model

    // Ride-height sensitive body and floor downforce (0 = insensitive). The
    // front/rear downforce areas apply at a floor clearance of rideHeightRef
    // and scale as (rideHeightRef / h)^rideHeightExponent as the car runs
    // lower, until the floor stalls below floorStallHeight: the flow under
    // the car chokes and that axle's downforce falls away (Katz 1995).
    // Clearance at each axle comes from the suspension: floor at body-frame
    // z = floorZ, over flat ground.
    Real rideHeightRef = 0.0;      // m
    Real rideHeightExponent = 0.6;
    Real floorStallHeight = 0.0;   // m
    Real floorZ = 0.0;             // m, body frame

    // Yaw (the car slides or there is crosswind): downforce drops as
    // cos^2(yaw) and a side force pushes the car downwind.
    Real sideForceArea = 0.0;      // m^2, F = q * sideForceArea * sin(yaw)

    // Rear wing: an inverted finite wing with a full-range section polar
    // (stalls past its best angle: downforce falls, drag climbs).
    struct Wing {
        Real span = 0.0, chord = 0.0;  // m (0 = no wing)
        Real angle = 0.0;              // rad, angle of attack toward downforce at zero pitch
        Vec3 position{};               // aerodynamic centre, body frame
        Real efficiency = 0.85;        // span efficiency for induced drag
        WingSectionPolar polar;
    } rearWing;
};

// The body as a collision hull against the ground: only touches when the car is tilted (a
// rollover, a car on its side or roof) or fully airborne (landing a jump). Sizes are relative to
// the wheel layout; 0 = a default for a typical car.
struct BodyDef {
    Real overhangFront = 0.0;  // m ahead of the front axle (0 = 0.9)
    Real overhangRear = 0.0;   // m behind the rear axle (0 = 0.9)
    Real halfWidth = 0.0;      // m (0 = widest track / 2 + 0.12)
    Real height = 0.0;         // m, floor to roof (0 = 1.15)
    Real floorZ = 0.0;         // m, floor height relative to the body origin (0 = from the hardpoints)
    // m added to a derived floor (floorZ = 0): the garage's ride-height change, so raising the
    // car on its suspension raises its floor too (the derived floor otherwise follows the
    // hardpoints the garage moves and would keep the same clearance).
    Real clearanceOffset = 0.0;
    Real friction = 0.5;       // sliding friction of the body on the road (scaled by the surface)
};

struct VehicleDefinition {
    std::string name = "unnamed";
    Real mass = 1400.0;            // kg, total including unsprung masses
    Vec3 centerOfMass{0, 0, 0.0};  // body frame
    Vec3 inertia{500, 2200, 2400}; // kg*m^2 principal moments about the CoM (roll, pitch, yaw)
    std::vector<WheelDef> wheels;  // ordered; axles reference them
    std::vector<AxleDef> axles;    // front to rear
    Real steeringAckermann = 0.6;  // 0 = parallel, 1 = full Ackermann
    Real topSpeedLimit = 0.0;      // m/s, electronic limiter: throttle fades over the last 2% (0 = none)
    DrivetrainDef drivetrain;
    AeroDef aero;
    BodyDef body;

    // Returns an empty string when valid, else a human-readable reason.
    std::string validate() const;

    Real wheelbase() const;
    Real totalUnsprungMass() const;
};

}  // namespace ord
