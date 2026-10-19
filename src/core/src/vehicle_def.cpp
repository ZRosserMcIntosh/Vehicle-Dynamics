// OpenRoad Dynamics — engine-independent vehicle physics core.
// SPDX-License-Identifier: MIT
#include "ord/vehicle_def.hpp"

#include <cmath>

namespace ord {

namespace {
bool inRange(Real v, Real lo, Real hi) { return std::isfinite(v) && v >= lo && v <= hi; }
}  // namespace

// Ranges are deliberately wide (karts to heavy trucks, eco tires to slicks) but
// reject values that can only be unit mistakes or corrupted data. Vehicle files
// are untrusted input (SECURITY.md); a definition that passes here must
// simulate without NaN/Inf (fuzz-tested in tests/test_robustness.cpp).
std::string VehicleDefinition::validate() const {
    if (!inRange(mass, 50, 100000)) return "mass must be within 50..100000 kg";
    if (!inRange(inertia.x, 1, 1e7) || !inRange(inertia.y, 1, 1e7) || !inRange(inertia.z, 1, 1e7))
        return "inertia components must be within 1..1e7 kg*m^2";
    if (!inRange(centerOfMass.x, -10, 10) || !inRange(centerOfMass.y, -10, 10) || !inRange(centerOfMass.z, -10, 10))
        return "centerOfMass must be finite and within 10 m of the body origin";
    if (!inRange(steeringAckermann, 0, 1)) return "steeringAckermann must be within 0..1";
    if (!inRange(topSpeedLimit, 0, 200)) return "topSpeedLimit must be within 0..200 m/s (0 = none)";
    if (!inRange(body.overhangFront, 0, 5) || !inRange(body.overhangRear, 0, 5) || !inRange(body.halfWidth, 0, 3) ||
        !inRange(body.height, 0, 5) || !inRange(body.floorZ, -3, 3) || !inRange(body.friction, 0, 3))
        return "body hull sizes must be within 0..5 m (floorZ -3..3 m) and friction 0..3";
    if (wheels.empty() || wheels.size() > kMaxWheels) return "wheel count must be 1..kMaxWheels";
    if (axles.empty() || axles.size() > kMaxAxles) return "axle count must be 1..kMaxAxles";
    for (std::size_t i = 0; i < wheels.size(); ++i) {
        const WheelDef& w = wheels[i];
        const TireParams& t = w.tire;
        const SuspensionDef& sd = w.suspension;
        const std::string id = "wheel " + std::to_string(i) + ": ";
        if (!inRange(t.radius, 0.1, 1.5)) return id + "tire radius must be within 0.1..1.5 m";
        if (!inRange(t.nominalLoad, 100, 1e6)) return id + "tire nominalLoad must be within 100..1e6 N";
        if (!inRange(t.wheelInertia, 0.05, 500)) return id + "wheelInertia must be within 0.05..500 kg*m^2";
        if (!inRange(t.muX, 0.05, 4) || !inRange(t.muY, 0.05, 4)) return id + "tire mu must be within 0.05..4";
        if (!inRange(t.muXLoadSensitivity, -1, 1) || !inRange(t.muYLoadSensitivity, -1, 1))
            return id + "tire load sensitivity must be within -1..1";
        if (!inRange(t.shapeX, 0.3, 3) || !inRange(t.shapeY, 0.3, 3)) return id + "Magic Formula shape C must be within 0.3..3";
        if (!inRange(t.curvatureX, -10, 1) || !inRange(t.curvatureY, -10, 1))
            return id + "Magic Formula curvature E must be within -10..1";
        if (!inRange(t.longStiffnessPerLoad, 0.5, 200) || !inRange(t.cornerStiffness, 0.5, 200) ||
            !inRange(t.cornerStiffnessLoad, 0.1, 10))
            return id + "tire stiffness parameters out of range";
        if (!inRange(t.rBx1, 0, 100) || !inRange(t.rBx2, -100, 100) || !inRange(t.rCx1, 0.1, 3) ||
            !inRange(t.rBy1, 0, 100) || !inRange(t.rBy2, -100, 100) || !inRange(t.rCy1, 0.1, 3))
            return id + "combined-slip coefficients out of range";
        if (!inRange(t.relaxationLengthX, 0.01, 5) || !inRange(t.relaxationLengthY, 0.01, 5))
            return id + "relaxation lengths must be within 0.01..5 m";
        if (!inRange(t.lowSpeedDampingPerLoad, 0, 100) || !inRange(t.lowSpeedDampingFadeSpeed, 0.1, 20))
            return id + "low-speed damping parameters out of range";
        if (!inRange(t.rollingResistance, 0, 0.2) || !inRange(t.pneumaticTrail, 0, 0.3) ||
            !inRange(t.trailCollapseAngle, 0.01, 1.5))
            return id + "rolling resistance / trail out of range";
        if (!inRange(t.verticalStiffness, 1e4, 1e8)) return id + "tire verticalStiffness must be within 1e4..1e8 N/m";
        if (!inRange(t.verticalDamping, 0, 1e6)) return id + "tire verticalDamping must be within 0..1e6 N*s/m";
        if (!inRange(t.patchHalfLength, 0, 0.6 * t.radius) || !inRange(t.patchHalfWidth, 0, 1.0))
            return id + "patch half extents must be >= 0, length <= 0.6 r, width <= 1 m";
        if (!inRange(sd.travel, 0.005, 2)) return id + "suspension travel must be within 0.005..2 m";
        if (!inRange(sd.springRate, 100, 1e8)) return id + "springRate must be within 100..1e8 N/m";
        if (!inRange(sd.springPreload, 0, 1e6)) return id + "springPreload must be within 0..1e6 N";
        if (!inRange(sd.bumpDamping, 0, 1e7) || !inRange(sd.reboundDamping, 0, 1e7)) return id + "damping must be within 0..1e7 N*s/m";
        if (!inRange(sd.bumpStopRate, 0, 1e9)) return id + "bumpStopRate must be within 0..1e9 N/m";
        if (!inRange(sd.hardpoint.x, -20, 20) || !inRange(sd.hardpoint.y, -20, 20) || !inRange(sd.hardpoint.z, -20, 20))
            return id + "hardpoint must be finite and within 20 m";
        if (!inRange(w.maxSteerAngle, 0, 1.2)) return id + "maxSteerAngle must be within 0..1.2 rad";
        if (!inRange(w.staticToe, -0.2, 0.2)) return id + "staticToe must be within -0.2..0.2 rad";
        if (!inRange(w.staticCamber, -0.3, 0.2)) return id + "staticCamber must be within -0.3..0.2 rad";
        if (!inRange(w.camberGain, -5, 5)) return id + "camberGain must be within -5..5 rad/m";
        if (!inRange(w.casterAngle, -0.2, 0.4)) return id + "casterAngle must be within -0.2..0.4 rad";
        if (!inRange(sd.bumpDampingFast, 0, 1e6) || !inRange(sd.reboundDampingFast, 0, 1e6))
            return id + "fast damping must be within 0..1e6 N*s/m";
        if (!inRange(sd.damperKneeSpeed, 0.001, 10)) return id + "damperKneeSpeed must be within 0.001..10 m/s";
        if (!inRange(w.tire.thermalCapacity, 0, 1e7) || !inRange(w.tire.temperatureWindow, 1, 500))
            return id + "tyre thermal parameters out of range";
        if (!inRange(w.tire.wetGrip, 0, 3) || !inRange(w.tire.looseGrip, 0, 3) || !inRange(w.tire.aquaplaneSensitivity, 0, 20))
            return id + "tyre compound factors out of range";
        if (!inRange(w.maxBrakeTorque, 0, 1e6) || !inRange(w.handbrakeTorque, 0, 1e6)) return id + "brake torque must be within 0..1e6 N*m";
        if (!inRange(w.unsprungMass, 0, 10000)) return id + "unsprungMass must be within 0..10000 kg";
    }
    for (std::size_t a = 0; a < axles.size(); ++a) {
        const AxleDef& ax = axles[a];
        if (ax.leftWheel < 0 || ax.rightWheel < 0 || ax.leftWheel >= static_cast<int>(wheels.size()) ||
            ax.rightWheel >= static_cast<int>(wheels.size()) || ax.leftWheel == ax.rightWheel)
            return "axle " + std::to_string(a) + ": invalid wheel indices";
        if (!inRange(ax.antiRollBarRate, 0, 1e8)) return "axle " + std::to_string(a) + ": antiRollBarRate must be within 0..1e8 N/m";
    }
    if (!inRange(aero.rideHeightRef, 0, 2) || !inRange(aero.floorStallHeight, 0, 1) || !inRange(aero.rideHeightExponent, 0, 3))
        return "aero: ride-height parameters out of range";
    if (!inRange(aero.rearWing.span, 0, 4) || !inRange(aero.rearWing.chord, 0, 2) || !inRange(aero.rearWing.angle, -0.5, 0.8))
        return "aero: rear wing out of range";
    if (!inRange(aero.sideForceArea, 0, 20) || !inRange(aero.ambientTemperature, -60, 70)) return "aero: out of range";
    const DrivetrainDef& dt = drivetrain;
    const EngineDef& e = dt.engine;
    if (e.torqueCurve.size() < 2) return "engine torque curve needs >= 2 points";
    for (std::size_t i = 0; i < e.torqueCurve.size(); ++i) {
        if (!inRange(e.torqueCurve[i].rpm, 0, 50000) || !inRange(e.torqueCurve[i].torque, -1e5, 1e5))
            return "torque curve points must be finite (rpm 0..50000, torque within 1e5 N*m)";
        if (i > 0 && e.torqueCurve[i].rpm <= e.torqueCurve[i - 1].rpm) return "torque curve rpm must increase";
    }
    if (!inRange(e.limiterRpm, 500, 50000) || !inRange(e.idleRpm, 0, e.limiterRpm)) return "idle/limiter rpm out of range";
    if (!inRange(e.inertia, 0.001, 100) || !inRange(dt.gearbox.inputInertia, 0.001, 10)) return "drivetrain inertia out of range";
    if (!inRange(e.frictionTorque, 0, 1e4) || !inRange(e.frictionPerRadS, 0, 100)) return "engine friction out of range";
    if (dt.gearbox.forwardRatios.empty()) return "gearbox needs at least one forward ratio";
    for (Real r : dt.gearbox.forwardRatios)
        if (!inRange(r, 0.05, 50)) return "gear ratios must be within 0.05..50";
    if (!inRange(dt.gearbox.reverseRatio, 0.05, 50)) return "reverse ratio must be within 0.05..50";
    if (!inRange(dt.gearbox.efficiency, 0.1, 1) || !inRange(dt.gearbox.shiftTime, 0, 5)) return "gearbox efficiency/shift time out of range";
    if (!inRange(dt.clutch.maxTorque, 1, 1e6)) return "clutch maxTorque must be within 1..1e6 N*m";
    if (dt.drivenAxles.empty()) return "at least one driven axle required";
    Real share = 0;
    for (const auto& da : dt.drivenAxles) {
        if (da.axleIndex >= axles.size()) return "driven axle index out of range";
        if (!inRange(da.torqueShare, 0, 1)) return "torque shares must be within 0..1";
        if (!inRange(da.differential.finalDrive, 0.1, 50)) return "final drive must be within 0.1..50";
        if (!inRange(da.differential.preload, 0, 1e6) || !inRange(da.differential.powerRamp, 0, 10) ||
            !inRange(da.differential.coastRamp, 0, 10))
            return "differential locking parameters out of range";
        share += da.torqueShare;
    }
    if (std::abs(share - 1.0) > 1e-6) return "driven axle torque shares must sum to 1";
    if (!inRange(dt.centerPreload, 0, 1e6) || !inRange(dt.centerRamp, 0, 10)) return "centre coupling parameters out of range";
    if (!inRange(aero.dragArea, 0, 20) || !inRange(aero.frontDownforceArea, -20, 50) ||
        !inRange(aero.rearDownforceArea, -20, 50) || !inRange(aero.airDensity, 0.1, 3))
        return "aero parameters out of range";
    if (totalUnsprungMass() >= 0.5 * mass) return "unsprung masses must be well below half the total mass";
    // A wheel heavier than its corner's share of the car would leave a negative
    // sprung load there (non-physical; real cars ~10-12%, heavy trucks ~25%).
    for (const WheelDef& w : wheels)
        if (w.unsprungMass > 0.5 * mass / static_cast<Real>(wheels.size()))
            return "each wheel's unsprungMass must be <= half of its equal share of the total mass";
    // Each corner must be able to hold up its share of the car within its travel
    // (spring over full travel plus half the travel into the bump stop). Uses an
    // equal share with a 1.6x margin for front/rear weight distribution.
    const Real cornerLoad = 1.6 * mass * kGravity / static_cast<Real>(wheels.size());
    for (std::size_t i = 0; i < wheels.size(); ++i) {
        const SuspensionDef& sd = wheels[i].suspension;
        const Real capacity = sd.springPreload + sd.springRate * sd.travel + sd.bumpStopRate * 0.5 * sd.travel;
        if (capacity < cornerLoad)
            return "wheel " + std::to_string(i) + ": suspension cannot support the static load (spring/bump stop/travel too small for the mass)";
    }
    return {};
}

Real VehicleDefinition::totalUnsprungMass() const {
    Real m = 0;
    for (const WheelDef& w : wheels) m += w.unsprungMass;
    return m;
}

Real VehicleDefinition::wheelbase() const {
    if (axles.size() < 2) return 1.0;
    const Real xf = wheels[axles.front().leftWheel].suspension.hardpoint.x;
    const Real xr = wheels[axles.back().leftWheel].suspension.hardpoint.x;
    return std::abs(xf - xr);
}

}  // namespace ord
