// OpenRoad Dynamics — suspension linkage kinematics.
// SPDX-License-Identifier: MIT
//
// A wheel's single suspension degree of freedom (its compression) is driven
// by a real linkage: a double wishbone (short-long-arm) or a MacPherson strut,
// with a tie rod for the steering and a coil spring/damper somewhere on it.
// The mechanism is solved once, at construction, over the wheel's travel; the
// result is a table of what the linkage does as the wheel moves:
//   - where the wheel centre goes (it scrubs sideways and fore/aft),
//   - how the wheel tilts (camber curve) and steers (bump steer, roll steer),
//   - how far the spring compresses per metre of wheel travel (motion ratio,
//     so the wheel rate rises or falls through the travel).
// The physics step only interpolates the table. The path's slope is the
// generalised direction of the wheel's DOF: tyre forces along it do work on
// the suspension, which is where roll-centre jacking, anti-dive and
// anti-squat come from, with no separate "anti" parameters.
//
// Geometry is given for the LEFT wheel of an axle, in the body frame, relative
// to the wheel centre at its design position (the car as drawn, at static
// ride height). The right wheel mirrors y. All points in metres.
#pragma once

#include <array>
#include <string>

#include "ord/math.hpp"

namespace ord {

enum class LinkageType {
    Slider,          // straight travel along body z (no kinematics: camberGain / toe constants)
    DoubleWishbone,  // upper and lower A-arms on body pivots, ball joints on the upright
    MacPherson,      // lower A-arm plus a strut that pivots at its top mount and telescopes
};

struct LinkageDef {
    LinkageType type = LinkageType::Slider;
    // Lower arm: inboard pivot axis through two points, outboard ball joint.
    Vec3 lowerPivotFront{0.18, -0.40, -0.16}, lowerPivotRear{-0.18, -0.40, -0.16};
    Vec3 lowerBallJoint{0.0, -0.03, -0.13};
    // Double wishbone: upper arm likewise (shorter arm = negative camber in bump).
    Vec3 upperPivotFront{0.12, -0.30, 0.21}, upperPivotRear{-0.12, -0.30, 0.21};
    Vec3 upperBallJoint{0.0, -0.08, 0.17};
    // MacPherson: the strut's upper mount on the body. The strut's axis runs from
    // here through the lower ball joint and is the steering axis.
    Vec3 strutTop{0.0, -0.10, 0.55};
    // Tie rod: inner end on the rack (body), outer end on the steering arm (upright).
    Vec3 tieRodInner{-0.13, -0.38, -0.10}, tieRodOuter{-0.13, -0.05, -0.08};
    // Spring/damper: upper mount on the body; lower mount on the lower arm
    // (wishbone) or on the strut, which moves with the upright (MacPherson).
    Vec3 springBody{0.0, -0.25, 0.30}, springArm{0.0, -0.20, -0.14};
    // Static ride position relative to the design position, metres of wheel
    // travel, positive toward bump. The garage's ride height sets it: lowering
    // the car by 20 mm puts the wheels 20 mm further into bump at rest, and the
    // camber and toe curves move with it.
    Real rideOffset = 0.0;
};

// What the linkage does at one wheel travel t (metres from the design position,
// positive toward bump). Camber and toe in the usual sign for either side:
// negative camber = top of the wheel in, positive toe = toe-in.
struct LinkagePose {
    Real lon = 0, lat = 0;      // wheel-centre offset from the design position, body x and y
    Real dLon = 0, dLat = 0;    // d(lon)/dt, d(lat)/dt: the DOF's direction is (dLon, dLat, 1)
    Real camber = 0, toe = 0;   // rad, change from the design pose
    Real dCamber = 0;           // rad/m
    Real springLength = 0;      // m
    Real motionRatio = 1;       // d(springLength)/dt (negative: the spring shortens in bump)
};

class LinkageTable {
public:
    static constexpr int kSamples = 41;

    // Solves the mechanism over travel tMin..tMax. Returns false, with a
    // reason, when the geometry cannot reach that travel or is degenerate.
    bool build(const LinkageDef& def, Real tMin, Real tMax, std::string* error = nullptr);
    bool valid() const { return valid_; }
    Real tMin() const { return tMin_; }
    Real tMax() const { return tMax_; }

    // Interpolates the table (clamped at the ends).
    LinkagePose at(Real t) const;
    // Motion ratio at the design position (normalises the spring rate).
    Real designMotionRatio() const { return mrDesign_; }
    // Integral of the normalised motion ratio: spring deflection in "wheel
    // metres" so that the wheel-rate spring force is k * rho(t) * (S(t) - S(t0)).
    Real springDeflection(Real t) const;

private:
    struct Row { Real t, lon, lat, camber, toe, springLength, deflection, dLon, dLat, dCamber, mr; };
    bool solveAt(const LinkageDef& def, Real t, Real& thetaLower, Real& thetaUpper, Real& phi, Row& row,
                 std::string* error) const;
    bool valid_ = false;
    Real tMin_ = 0, tMax_ = 0, mrDesign_ = 1;
    std::array<Row, kSamples> rows_{};
};

// Builds common linkages from a few descriptive numbers (all metres / radians).
//   armLength: lower arm length; upperArmLength: shorter for negative camber in bump;
//   uprightHeight: ball-joint separation; kingpin: lateral offset of the upper
//   ball joint inboard of the lower (kingpin inclination); antiDive: side-view
//   inclination of the pivot axes in radians. Positive leans the wheel's path
//   forward as it compresses: anti-dive on a front axle. On a rear axle the
//   same geometry is pro-squat, so pass a negative value for anti-squat.
LinkageDef makeDoubleWishbone(Real armLength, Real upperArmLength, Real uprightHeight, Real kingpinOffset,
                              Real antiDive = 0.0, Real springFraction = 0.55);
LinkageDef makeMacPherson(Real armLength, Real strutHeight, Real strutTopInboard, Real antiDive = 0.0);

// Front-view roll-centre height above the ground for one wheel of an axle, in
// metres, from the DOF direction and the camber rate at its current travel.
// halfTrack = lateral distance from the centreline to the contact patch,
// contactBelow = wheel centre height above the contact patch.
Real rollCentreHeight(const LinkagePose& pose, Real halfTrack, Real contactBelow);

}  // namespace ord
