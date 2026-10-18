// OpenRoad Dynamics — engine-independent vehicle physics core.
// SPDX-License-Identifier: MIT
#include "ord/two_wheeler.hpp"

#include <algorithm>

namespace ord {

namespace {

Real smoothStep01(Real x) {
    x = clamp(x, 0.0, 1.0);
    return x * x * (3 - 2 * x);
}

// Thin-ring-ish wheel: axial = k m r^2, diametral = half of it.
SingleTrackWheelDef wheel(Real radius, Real mass, Real k) {
    const Real axial = k * mass * radius * radius;
    return {radius, mass, 0.5 * axial, axial};
}

Mat3 inertia(Real ixx, Real iyy, Real izz, Real ixz) {
    Mat3 m = Mat3::diagonal({ixx, iyy, izz});
    m.m[0][2] = m.m[2][0] = ixz;
    return m;
}

TireParams motorcycleTyre(Real radius, Real nominalLoad, Real crown) {
    // Representative sport-touring radial (plausible values in the ranges of
    // Pacejka 2012 ch. 11 and Cossalter 2006 ch. 2, not a measured tyre).
    TireParams t;
    t.radius = radius;
    t.nominalLoad = nominalLoad;
    t.muX = 1.25;
    t.muXLoadSensitivity = -0.06;
    t.muY = 1.25;
    t.muYLoadSensitivity = -0.06;
    t.longStiffnessPerLoad = 16.0;
    t.shapeX = 1.45;  // keeps ~75% of peak grip when spinning or locked
    t.cornerStiffness = 11.0;
    t.cornerStiffnessLoad = 2.0;
    t.camberStiffness = 1.0;
    t.camberShape = 0.6;
    t.camberTwist = 0.02;
    t.crownRadius = crown;
    // Short lateral relaxation: with 0.2 m the rigid model's weave, coupled
    // with the rider loop, ran away under aggressive input at 25-35 m/s.
    t.relaxationLengthX = 0.08;
    t.relaxationLengthY = 0.08;
    t.lowSpeedDampingPerLoad = 1.0;
    t.lowSpeedDampingFadeSpeed = 1.5;
    t.rollingResistance = 0.015;
    t.pneumaticTrail = 0.02;
    t.trailCollapseAngle = 0.2;
    // No suspension DOF yet: the vertical "tyre" stands in for tyre and
    // suspension in series (~3 cm static sag, damping ratio ~0.4).
    t.verticalStiffness = 4.0e4;
    t.verticalDamping = 1900.0;
    return t;
}

TireParams scooterTyre(Real radius, Real nominalLoad) {
    // 8.5" pneumatic scooter tyre: small, round, less grip than a bicycle tyre.
    TireParams t;
    t.radius = radius;
    t.nominalLoad = nominalLoad;
    t.muX = 0.9;
    t.muXLoadSensitivity = -0.05;
    t.muY = 0.9;
    t.muYLoadSensitivity = -0.05;
    t.longStiffnessPerLoad = 14.0;
    t.cornerStiffness = 10.0;
    t.cornerStiffnessLoad = 2.0;
    t.camberStiffness = 0.7;
    t.camberShape = 0.6;
    t.camberTwist = 0.008;
    t.crownRadius = 0.025;
    t.relaxationLengthX = 0.04;
    t.relaxationLengthY = 0.06;
    t.lowSpeedDampingPerLoad = 1.0;
    t.lowSpeedDampingFadeSpeed = 1.5;
    t.rollingResistance = 0.012;
    t.pneumaticTrail = 0.004;
    t.trailCollapseAngle = 0.2;
    t.verticalStiffness = 8.0e4;
    t.verticalDamping = 400.0;
    return t;
}

}  // namespace

TwoWheelerDef benchmarkBicycle() {
    TwoWheelerDef d;
    d.name = "bicycle";
    d.frame = meijaardBenchmark();
    d.powertrain = {};  // pedalling: 60 N*m / 350 W at the rear wheel, rim brakes
    d.powertrain.dragArea = 0.5;
    d.rider = {};
    d.rider.maxSteerTorque = 15;
    d.rider.leanTorqueGain = 20;
    d.rider.leanRateTorqueGain = 4;
    d.rider.leanIntegralGain = 30;
    d.steerLock = 1.2;
    return d;
}

TwoWheelerDef electricScooter() {
    TwoWheelerDef d;
    d.name = "e-scooter";
    SingleTrackDef& f = d.frame;
    f.wheelbase = 0.85;
    f.trail = 0.035;
    f.steerAxisTilt = 0.26;  // ~15 deg
    f.rearWheel = wheel(0.11, 3.5, 0.6);   // hub motor
    f.frontWheel = wheel(0.11, 1.5, 0.8);
    // Deck + standing rider (75 kg), CoM between the feet, about 1 m up.
    f.rearFrame.mass = 87.0;
    f.rearFrame.centreOfMass = {0.42, 0.0, 1.0};
    f.rearFrame.inertia = inertia(12.0, 12.5, 2.0, -0.6);
    // Stem + handlebar.
    f.frontFrame.mass = 3.5;
    f.frontFrame.centreOfMass = {0.80, 0.0, 0.55};
    f.frontFrame.inertia = inertia(0.22, 0.22, 0.02, 0.01);
    f.rearTyre = scooterTyre(0.11, 500.0);
    f.frontTyre = scooterTyre(0.11, 420.0);
    d.powertrain.maxWheelTorque = 25;
    d.powertrain.maxPower = 500;
    d.powertrain.topSpeed = 25.0 / 3.6;
    d.powertrain.dragArea = 0.55;
    d.powertrain.frontBrakeTorque = 30;
    d.powertrain.rearBrakeTorque = 25;
    d.rider.maxLean = 0.6;
    d.rider.maxSteerTorque = 12;
    d.rider.leanTorqueGain = 20;
    d.rider.leanRateTorqueGain = 4;
    d.rider.leanIntegralGain = 30;
    d.rider.steerStiffness = 40;
    d.rider.steerDamping = 2.5;
    d.steerLock = 0.9;
    return d;
}

TwoWheelerDef sportMotorcycle() {
    TwoWheelerDef d;
    d.name = "motorcycle";
    SingleTrackDef& f = d.frame;
    f.wheelbase = 1.40;
    f.trail = 0.10;
    f.steerAxisTilt = 0.42;  // ~24 deg rake
    f.rearWheel = wheel(0.32, 14.0, 0.7);
    f.frontWheel = wheel(0.30, 11.0, 0.7);
    // Frame, engine, tank and a 75 kg rider tucked in: ~50/50 static split.
    f.rearFrame.mass = 230.0;
    f.rearFrame.centreOfMass = {0.62, 0.0, 0.68};
    f.rearFrame.inertia = inertia(24.0, 48.0, 32.0, -4.0);
    // Fork, triple clamps, bars (steer inertia ~0.6 kg m^2 with the wheel).
    f.frontFrame.mass = 18.0;
    f.frontFrame.centreOfMass = {1.22, 0.0, 0.62};
    f.frontFrame.inertia = inertia(1.3, 1.2, 0.35, 0.2);
    f.rearTyre = motorcycleTyre(0.32, 1350.0, 0.08);
    f.frontTyre = motorcycleTyre(0.30, 1250.0, 0.06);
    d.powertrain.maxWheelTorque = 1100;  // enough to lift the front in the first gears
    d.powertrain.maxPower = 80000;
    d.powertrain.dragArea = 0.35;
    d.powertrain.frontBrakeTorque = 1600;
    d.powertrain.rearBrakeTorque = 450;
    d.rider.maxLean = 0.8;
    d.rider.maxSteerTorque = 70;
    d.rider.steerStiffness = 900;
    d.rider.steerDamping = 30;
    d.rider.leanFrequency = 3.5;
    d.rider.leanTorqueGain = 150;
    d.rider.leanRateTorqueGain = 25;
    d.rider.leanIntegralGain = 400;
    d.steerLock = 0.55;
    d.steerDamper = 20;
    return d;
}

TwoWheeler::TwoWheeler(const TwoWheelerDef& def, const TwoWheelerAssists& assists)
    : def_(def), assists_(assists), st_(def.frame, SingleTrackContact::Tyre) {
    const SingleTrackDef& f = def_.frame;
    const Real mR = f.rearWheel.mass, mB = f.rearFrame.mass, mH = f.frontFrame.mass, mF = f.frontWheel.mass;
    const Real m = mR + mB + mH + mF;
    comHeight_ = (mR * f.rearWheel.radius + mB * f.rearFrame.centreOfMass.z + mH * f.frontFrame.centreOfMass.z +
                  mF * f.frontWheel.radius) / m;
    // Front assembly inertia about the steer axis (fork + bars, and the wheel's
    // diametral inertia), each with its parallel-axis term.
    {
        const Real lam = f.steerAxisTilt;
        const Vec3 u{-std::sin(lam), 0, std::cos(lam)};
        const Vec3 groundPt{f.wheelbase + f.trail, 0, 0};
        auto aboutAxis = [&](const Mat3& I, Real mass, const Vec3& c) {
            const Vec3 r = c - groundPt;
            const Vec3 perp = r - u * dot(r, u);
            return dot(u, I * u) + mass * dot(perp, perp);
        };
        const Real idw = f.frontWheel.inertiaDiametral;
        steerInertia_ = aboutAxis(f.frontFrame.inertia, mH, f.frontFrame.centreOfMass) +
                        aboutAxis(Mat3::diagonal({idw, f.frontWheel.inertiaAxial, idw}), mF, {f.wheelbase, 0, f.frontWheel.radius});
        steerInertia_ = std::max(steerInertia_, 1e-3);
    }
    rearStatic_ = m * f.gravity;  // minus the front share, below
    frontStatic_ = m * f.gravity *
                   (mR * 0 + mB * f.rearFrame.centreOfMass.x + mH * f.frontFrame.centreOfMass.x + mF * f.wheelbase) /
                   (m * f.wheelbase);
    rearStatic_ -= frontStatic_;
    rollInertia_ = f.rearFrame.inertia.m[0][0] + mB * f.rearFrame.centreOfMass.z * f.rearFrame.centreOfMass.z +
                   f.frontFrame.inertia.m[0][0] + mH * f.frontFrame.centreOfMass.z * f.frontFrame.centreOfMass.z;
    reset(0, 0, 0, 0);
}

bool TwoWheeler::reset(Real x, Real y, Real yaw, Real speed, const GroundQuery* ground) {
    tel_ = {};
    absScale_[0] = absScale_[1] = 1;
    driveScale_ = 1;
    leanI_ = leanTarget_ = 0;
    tcsScale_ = 1;
    prevPitch_ = pitchRate_ = 0;
    if (!st_.reset(speed)) return false;
    Real z = 0;
    if (ground) {
        const GroundHit h = ground->raycast({x, y, 1000.0}, {0, 0, -1}, 2000.0);
        if (h.hit) z = h.point.z;
    }
    st_.place({x, y, z}, yaw);
    return true;
}

void TwoWheeler::step(Real dt, const TwoWheelerInput& in, const GroundQuery* ground) {
    if (tel_.fallen || st_.faulted() || !(dt > 0)) return;
    const SingleTrackDef& f = def_.frame;
    const RiderParams& rp = def_.rider;
    const TwoWheelerPowertrain& pt = def_.powertrain;
    const Real g = f.gravity;

    const Real v = st_.forwardSpeed();
    const Real phi = st_.leanAngle(), phid = st_.leanRate();
    const Real delta = st_.steerAngle(), deltad = st_.steerRate();
    const Real steerIn = clamp(in.steer, -1.0, 1.0);

    // ---- handlebar torque
    // Rolling factor: how much the lean dynamics are steerable (0 at a stop).
    const Real roll = smoothStep01((std::abs(v) - 1.0) / 3.0);
    Real steerTorque = 0;
    Real target = 0;
    if (assists_.mode == RiderMode::Direct) {
        // + input (left) = torque toward left steer (benchmark negative).
        steerTorque = -steerIn * rp.maxSteerTorque;
    } else {
        // Lean asked for: full lean only once rolling; at walking pace the
        // input steers the bars directly instead.
        const Real vs = std::max(std::abs(v), 1.0);
        // A lean the bike can actually hold: within grip (65% of the weaker
        // tyre's friction) and within the steering lock at this speed
        // (balanced turn: tan(phi) = v^2 / (g R), R ~ w / (delta cos(lambda))).
        const Real mu = std::min(f.rearTyre.muY, f.frontTyre.muY) * std::max(st_.surfaceFriction(), 0.05);
        const Real leanGrip = std::atan(0.65 * mu);
        const Real leanLock = std::atan(vs * vs * 0.65 * def_.steerLock * std::cos(f.steerAxisTilt) / (g * f.wheelbase));
        const Real maxLean = std::min({rp.maxLean, leanGrip, leanLock});
        // A rider eases in and out of a lean: the target slews at a limited
        // rate, and a large lean error asks for a bounded lean acceleration
        // (letting go of the steering at 35 deg must not snap the bars over).
        const Real asked = -steerIn * maxLean * roll;
        const Real slew = rp.maxLeanRate * dt;
        leanTarget_ = clamp(asked, leanTarget_ - slew, leanTarget_ + slew);
        target = leanTarget_;
        const Real wn = rp.leanFrequency, z = rp.leanDamping;
        const Real cosL = std::cos(f.steerAxisTilt);
        // Outer loop: ground steer angle for the commanded lean acceleration
        // phi'' = -wn^2 (phi - target) - 2 z wn phi'. Uses g tan(phi) so the
        // steady part is the kinematic steer of a balanced turn.
        const Real need = g * std::tan(clamp(phi, -1.2, 1.2)) +
                          comHeight_ * (wn * wn * clamp(phi - target, -0.25, 0.25) + 2 * z * wn * phid);
        Real deltaStar = f.wheelbase * need / (vs * vs * cosL);
        // Walking pace: bars follow the input (the foot / balance moment holds the bike up).
        deltaStar = roll * deltaStar + (1 - roll) * (-steerIn * 0.7 * def_.steerLock);
        deltaStar = clamp(deltaStar, -0.9 * def_.steerLock, 0.9 * def_.steerLock);
        // Arms relax with speed: a stiff grip on the bars at speed feeds the
        // weave mode (riders are told to hold the bars loosely).
        const Real grip = 1.0 - (1.0 - rp.highSpeedArmFraction) * smoothStep01((std::abs(v) - rp.armRelaxSpeed) / rp.armRelaxSpeed);
        steerTorque = grip * (rp.steerStiffness * (deltaStar - delta) - rp.steerDamping * deltad);
        // At speed the steer angle needed is tiny and the front end's own
        // (gyroscopic, trail) torques dominate it: lean is then controlled by
        // handlebar torque directly. Torque toward right steer first leans the
        // bike left (counter-steer), so: T = Kp (phi - target) + Kd phi'.
        steerTorque += roll * (rp.leanTorqueGain * clamp(phi - target, -0.2, 0.2) + rp.leanRateTorqueGain * phid);
        // Arms also keep the bars inside what the tyres can use at this
        // speed: twice the kinematic steer of a turn at full grip (plus a
        // little). A bigger angle at speed only saturates the front tyre.
        const Real deltaGrip = 2.0 * f.wheelbase * mu * g / (vs * vs * cosL) + 0.03;
        const Real excess = std::abs(delta) - deltaGrip;
        if (excess > 0) steerTorque -= std::copysign(rp.steerStiffness * excess, delta) + (delta * deltad > 0 ? rp.steerDamping * deltad : 0.0);
        // Slow integral on the lean error removes the steady-state offset of
        // the small-angle model (tyre, gyroscopic and crown effects).
        const Real iMax = 0.4 * rp.maxSteerTorque;
        // Integrates only while the lean is settling (no wind-up in transients).
        const Real settle = 1.0 - smoothStep01(std::abs(phid) / 0.3);
        leanI_ = clamp(leanI_ + rp.leanIntegralGain * roll * settle * (phi - target) * dt, -iMax, iMax);
        if (roll < 0.5) leanI_ *= std::max(0.0, 1.0 - dt / 0.3);
        steerTorque += leanI_;
        steerTorque = clamp(steerTorque, -rp.maxSteerTorque, rp.maxSteerTorque);
    }
    // Steering lock: an end stop as stiff as the step allows. It acts on
    // the steer DOF explicitly (once per step), so its stiffness and damping
    // are bounded by the front end's steer inertia: k dt^2 / I <= 0.2 and
    // c dt / I <= 0.4 keep it stable at any step size.
    const Real over = std::abs(delta) - def_.steerLock;
    Real lockTorque = 0;
    if (over > 0) {
        const Real kLock = 0.2 * steerInertia_ / (dt * dt), cLock = 0.4 * steerInertia_ / dt;
        lockTorque = -std::copysign(1.0, delta) * kLock * over - (delta * deltad > 0 ? cLock * deltad : 0.0);
    }

    // ---- low-speed balance (foot down / body roll)
    Real balance = 0;
    const Real fade = 1.0 - smoothStep01((std::abs(v) - 1.5) / 2.5);
    if (assists_.balanceAssist > 0 && fade > 0) {
        Real m = 0;
        for (int i = 0; i < SingleTrack::kBodies; ++i) m += st_.body(i).mass;
        const Real kb = 2.0 * m * g * comHeight_;  // twice the toppling stiffness
        const Real cb = 2.0 * 0.8 * std::sqrt(kb * rollInertia_);
        balance = -(kb * (phi - target) + cb * phid) * fade * clamp(assists_.balanceAssist, 0.0, 1.0);
    }

    // Wheel loads as an ECU estimates them: low-passed (~0.3 s). Feeding the
    // instantaneous load back into the torque limits pumps the pitch mode.
    for (int w = 0; w < 2; ++w) loadF_[w] += (st_.normalLoad(w) - loadF_[w]) * std::min(1.0, dt / 0.3);

    // ---- powertrain and brakes
    const int driven = pt.frontDrive ? 1 : 0;
    const Real omega = std::max(std::abs(st_.wheelSpin(driven)), 1e-3);
    Real drive = clamp(in.throttle, 0.0, 1.0) * std::min(pt.maxWheelTorque, pt.maxPower / omega);
    if (pt.topSpeed > 0) drive *= 1.0 - smoothStep01((v - 0.95 * pt.topSpeed) / (0.05 * pt.topSpeed));
    if (v < -0.5) drive = 0;

    // Assists on the drive: wheel spin (TCS) and front lift (wheelie control).
    tel_.tcsActive = tel_.wheelieActive = false;
    Real driveCut = 1.0, driveCeiling = 1e30;
    if (assists_.tractionControl) {
        // Slip control toward ~10% drive slip; slow enough (tens of ms) not to
        // pump the pitch mode the way an on/off cut does.
        // Cornering traction control (as on modern bikes, from the lean
        // angle an IMU measures): leaned over, part of the friction circle is
        // taken by cornering, so both the slip target and the torque ceiling
        // shrink with lean.
        const Real sl = std::abs(std::sin(phi));
        tcsScale_ = clamp(tcsScale_ + dt * 12.0 * (0.10 - 0.07 * sl - st_.tyreSlip(driven).kappa), 0.05, 1.0);
        driveCut = std::min(driveCut, tcsScale_);
        const Real muD = (driven ? f.frontTyre.muX : f.rearTyre.muX) * std::max(st_.surfaceFriction(), 0.05);
        const Real lat = std::min(1.0, std::tan(std::min(std::abs(phi), 1.2)) / muD);
        const Real rD = driven ? f.frontWheel.radius : f.rearWheel.radius;
        const Real ceiling = 0.7 * rD * muD * loadF_[driven] * std::sqrt(std::max(0.0, 1.0 - lat * lat));
        driveCeiling = ceiling;
        tel_.tcsActive = driveCut < 0.98 || drive > ceiling;
    } else {
        tcsScale_ = 1;
    }
    const Real pitch = st_.pitchAngle();
    pitchRate_ += ((pitch - prevPitch_) / dt - pitchRate_) * std::min(1.0, dt / 0.02);
    prevPitch_ = pitch;
    const Real wp = std::max(assists_.wheeliePitch, 0.0);
    const Real pitchAhead = pitch + 0.3 * pitchRate_;  // where the front will be in ~0.3 s
    if (assists_.wheelieControl) {
        if (wp < 0.1) {
            // Keep the front down: steering needs a loaded front tyre, so cut
            // drive as the front load falls below ~45% of its static value
            // (well before the wheel actually lifts).
            const Real unload = st_.normalLoad(1) / std::max(frontStatic_, 1.0);
            if (unload < 0.45 || pitchAhead > wp) {
                driveCut = std::min(driveCut, std::min(clamp((unload - 0.15) / 0.3, 0.0, 1.0),
                                                       clamp(1.0 - (pitchAhead - wp) / 0.08, 0.0, 1.0)));
                tel_.wheelieActive = true;
            }
        } else if (pitchAhead > wp) {
            // Wheelie assist: hold the front up at about wp.
            driveCut = std::min(driveCut, clamp(1.0 - (pitchAhead - wp) / 0.08, 0.0, 1.0));
            tel_.wheelieActive = true;
        }
    }
    // Smooth the cut (a real ECU ramps torque, ~50 ms). The lean ceiling is
    // already smooth (filtered load, lean) and applies directly.
    driveScale_ += (driveCut - driveScale_) * std::min(1.0, dt / 0.05);
    drive = std::min(drive * driveScale_, driveCeiling);

    Real brake[2] = {clamp(in.rearBrake, 0.0, 1.0) * pt.rearBrakeTorque,
                     clamp(in.frontBrake, 0.0, 1.0) * pt.frontBrakeTorque};
    if (assists_.wheelieControl) {
        // Stoppie control: ease the front brake as the rear unloads (before
        // it lifts), and harder once the rear is in the air.
        const Real rearUnload = st_.normalLoad(0) / std::max(rearStatic_, 1.0);
        Real ease = clamp((rearUnload - 0.1) / 0.3, 0.15, 1.0);
        if (pitch < -0.03) ease = std::min(ease, clamp(1.0 + (pitch + 0.03) / 0.05, 0.15, 1.0));
        if (ease < 1) tel_.wheelieActive = true;
        brake[1] *= ease;
    }
    tel_.absActive = false;
    for (int w = 0; w < 2; ++w) {
        if (assists_.abs && brake[w] > 0 && std::abs(v) > 2.0) {
            // Slip control toward the peak of the braking force (~8% slip):
            // the lever pressure the rider asks for is the ceiling.
            const Real kappa = st_.tyreSlip(w).kappa;
            absScale_[w] = clamp(absScale_[w] + dt * 60.0 * (kappa + 0.08), 0.05, 1.0);
            if (absScale_[w] < 0.999) tel_.absActive = true;
        } else {
            absScale_[w] = 1;
        }
        brake[w] *= absScale_[w];
        if (assists_.abs && brake[w] > 0) {
            // Cornering ABS: leaned over, cornering already uses part of the
            // friction circle, so the brake torque is capped to what is left.
            const Real muB = (w ? f.frontTyre.muX : f.rearTyre.muX) * std::max(st_.surfaceFriction(), 0.05);
            const Real lat = std::min(1.0, std::tan(std::min(std::abs(phi), 1.2)) / muB);
            const Real rB = w ? f.frontWheel.radius : f.rearWheel.radius;
            const Real ceiling = 0.6 * rB * muB * loadF_[w] * std::sqrt(std::max(0.0, 1.0 - lat * lat));
            if (brake[w] > ceiling && std::abs(phi) > 0.1) {
                brake[w] = ceiling;
                tel_.absActive = true;
            }
        }
    }
    // Brake torque opposes wheel spin, never more than stops the wheel within
    // half a step (a dry-friction clamp that holds at rest; a plain viscous
    // ramp stiff enough to hold would be unstable at game step sizes).
    auto brakeTorque = [&](int w) {
        const Real spin = st_.wheelSpin(w);
        const Real inertia = w == 0 ? f.rearWheel.inertiaAxial : f.frontWheel.inertiaAxial;
        return -std::copysign(std::min(brake[w], 0.5 * inertia * std::abs(spin) / dt), spin);
    };

    // Aerodynamic drag on the rider + frame (at the CoM height, as a force pair
    // is not available here: applied through the rear wheel as an equivalent
    // torque, which ignores the small pitch moment of drag).
    const Real drag = 0.5 * 1.2 * pt.dragArea * v * std::abs(v);

    SingleTrackInput si;
    si.steerTorque = steerTorque + lockTorque - def_.steerDamper * deltad;
    si.driveTorque = (driven == 0 ? drive : 0) + brakeTorque(0) - drag * f.rearWheel.radius;
    si.frontWheelTorque = (driven == 1 ? drive : 0) + brakeTorque(1);
    si.rollTorque = balance;
    st_.step(dt, si, ground);

    tel_.speed = st_.forwardSpeed();
    tel_.lean = st_.leanAngle();
    tel_.steer = st_.steerAngle();
    tel_.pitch = st_.pitchAngle();
    tel_.leanTarget = target;
    tel_.steerTorque = steerTorque;
    tel_.driveTorque = drive;
    tel_.rearBrake = brake[0];
    tel_.frontBrake = brake[1];
    tel_.balanceTorque = balance;
    if (st_.faulted() || std::abs(tel_.lean) > rp.fallLean || std::abs(tel_.pitch) > 1.2) tel_.fallen = true;
}

}  // namespace ord
