// OpenRoad Dynamics — engine-independent vehicle physics core.
// SPDX-License-Identifier: MIT
#include "ord/vehicle.hpp"

#include "ord/linear_solve.hpp"

#include <algorithm>

namespace ord {

namespace {

constexpr Real kRayStartOffset = 0.15;  // m above the full-bump hardpoint, so deep penetrations still hit
constexpr Real kRayExtra = 0.05;        // m below full droop + radius (tire deflection margin)

struct Offset {
    Real s, l;
};

// Contact sample layout in the patch (longitudinal s, lateral l), in units of
// the half extents. "Flat footprint": every sample sits on the plane of the
// deformed patch, so on flat ground all layouts give the same load.
int sampleLayout(int n, std::array<Offset, kMaxContactSamples>& out) {
    switch (n) {
        case 5:
            out[0] = {0, 0}; out[1] = {1, 0}; out[2] = {-1, 0}; out[3] = {0, 1}; out[4] = {0, -1};
            return 5;
        case 8:
            out[0] = {1, 1}; out[1] = {1, 0}; out[2] = {1, -1}; out[3] = {0, 1};
            out[4] = {0, -1}; out[5] = {-1, 1}; out[6] = {-1, 0}; out[7] = {-1, -1};
            return 8;
        case 9:
            out[0] = {0, 0}; out[1] = {1, 1}; out[2] = {1, 0}; out[3] = {1, -1}; out[4] = {0, 1};
            out[5] = {0, -1}; out[6] = {-1, 1}; out[7] = {-1, 0}; out[8] = {-1, -1};
            return 9;
        default:
            out[0] = {0, 0};
            return 1;
    }
}

// Two-stage damper: force at velocity v (+ = compression), and its slope.
Real damperForce(const SuspensionDef& sd, Real v, Real& slope) {
    const Real cs = v > 0 ? sd.bumpDamping : sd.reboundDamping;
    const Real cfRaw = v > 0 ? sd.bumpDampingFast : sd.reboundDampingFast;
    const Real cf = cfRaw > 0 ? cfRaw : cs;
    const Real vk = std::max(sd.damperKneeSpeed, 1e-3);
    if (std::abs(v) <= vk) {
        slope = cs;
        return cs * v;
    }
    slope = cf;
    return std::copysign(cs * vk + cf * (std::abs(v) - vk), v);
}

int sanitizeSamples(int n) { return (n == 5 || n == 8 || n == 9) ? n : 1; }

}  // namespace

Vehicle::Vehicle(const VehicleDefinition& def, const AssistConfig& assists, const VehicleFidelity& fidelity)
    : def_(def), fidelity_(fidelity), assists_(assists) {
    fidelity_.contactSamples = sanitizeSamples(fidelity_.contactSamples);
    error_ = def_.validate();
    valid_ = error_.empty();
    if (!valid_) return;

    for (const WheelDef& w : def_.wheels) tires_.emplace_back(w.tire);

    std::array<std::array<int, 2>, kMaxAxles> axleWheels{};
    for (std::size_t a = 0; a < def_.axles.size(); ++a) {
        const auto l = static_cast<std::size_t>(def_.axles[a].leftWheel);
        const auto r = static_cast<std::size_t>(def_.axles[a].rightWheel);
        axleWheels[a] = {def_.axles[a].leftWheel, def_.axles[a].rightWheel};
        front_[l] = front_[r] = a == 0;
        axleOf_[l] = axleOf_[r] = static_cast<int>(a);
    }
    for (const auto& da : def_.drivetrain.drivenAxles) {
        driven_[static_cast<std::size_t>(def_.axles[da.axleIndex].leftWheel)] = true;
        driven_[static_cast<std::size_t>(def_.axles[da.axleIndex].rightWheel)] = true;
    }
    drivetrain_ = Drivetrain(def_.drivetrain, def_.wheels.size(), axleWheels);

    for (const WheelDef& wd : def_.wheels) maxSteerRef_ = std::max(maxSteerRef_, wd.maxSteerAngle);
    inertiaBody_ = def_.inertia;
    invInertiaBody_ = Mat3::diagonal({1.0 / def_.inertia.x, 1.0 / def_.inertia.y, 1.0 / def_.inertia.z});
    telem_.wheelCount = def_.wheels.size();
    {
        // Compression at static ride height (camber gain reference).
        std::array<Real, kMaxWheels> load{};
        staticWheelLoads(load);
        for (std::size_t w = 0; w < def_.wheels.size(); ++w) {
            const SuspensionDef& sd = def_.wheels[w].suspension;
            const Real sprung = load[w] - def_.wheels[w].unsprungMass * kGravity;
            rideCompression_[w] = clamp((sprung - sd.springPreload) / std::max(sd.springRate, 1.0), 0.0, sd.travel);
        }
        setTyreTemperatures(def_.aero.ambientTemperature);
        // Linkage tables: solve each real linkage over its travel (plus a margin
        // for the bump stops). A geometry that cannot reach the travel makes the
        // definition invalid, like any other bad parameter.
        for (std::size_t w = 0; w < def_.wheels.size(); ++w) {
            const SuspensionDef& sd = def_.wheels[w].suspension;
            if (sd.linkage.type == LinkageType::Slider) continue;
            kinT0_[w] = -rideCompression_[w] + sd.linkage.rideOffset;
            std::string err;
            if (!linkage_[w].build(sd.linkage, kinT0_[w] - 0.03, sd.travel + kinT0_[w] + 0.03, &err)) {
                error_ = "wheel " + std::to_string(w) + ": " + err;
                valid_ = false;
                return;
            }
            kinDroopS_[w] = linkage_[w].springDeflection(kinT0_[w]);
        }
    }
    {
        // Body collision hull: a box from the axles' overhangs, the track and a floor-to-roof
        // height, with a narrower, shorter roof (the greenhouse) and points at the sills and the
        // belt line. 17 points: enough to land on a corner, a side, the roof or the floor.
        const BodyDef& b = def_.body;
        Real xf = -1e9, xr = 1e9, yMax = 0, groundZ = 1e9;
        for (std::size_t w = 0; w < def_.wheels.size(); ++w) {
            const WheelDef& wd = def_.wheels[w];
            xf = std::max(xf, wd.suspension.hardpoint.x);
            xr = std::min(xr, wd.suspension.hardpoint.x);
            yMax = std::max(yMax, std::abs(wd.suspension.hardpoint.y));
            // ground under this wheel at static ride height
            groundZ = std::min(groundZ, wd.suspension.hardpoint.z - (wd.suspension.travel - rideCompression_[w]) - wd.tire.radius);
        }
        const Real front = xf + (b.overhangFront > 0 ? b.overhangFront : 0.9);
        const Real rear = xr - (b.overhangRear > 0 ? b.overhangRear : 0.9);
        const Real hw = b.halfWidth > 0 ? b.halfWidth : yMax + 0.12;
        const Real floor = b.floorZ != 0 ? b.floorZ : groundZ + 0.13 + b.clearanceOffset;  // 13 cm ground clearance stock
        const Real roof = floor + (b.height > 0 ? b.height : 1.15);
        const Real belt = floor + 0.55 * (roof - floor);
        const Real rr = rear + 0.30 * (front - rear), rf = rear + 0.65 * (front - rear);  // roof from 30% to 65% of the length
        int k = 0;
        for (Real x : {front, rear})
            for (Real y : {hw, -hw}) {
                hull_[static_cast<std::size_t>(k++)] = {x, y, floor};
                hull_[static_cast<std::size_t>(k++)] = {x, y, belt};
            }
        for (Real y : {hw, -hw}) {
            hull_[static_cast<std::size_t>(k++)] = {0.5 * (front + rear), y, floor};
            hull_[static_cast<std::size_t>(k++)] = {0.5 * (front + rear), y, belt};
        }
        for (Real x : {rr, rf})
            for (Real y : {0.8 * hw, -0.8 * hw}) hull_[static_cast<std::size_t>(k++)] = {x, y, roof};
        hull_[static_cast<std::size_t>(k++)] = {0.5 * (rr + rf), 0, roof};
        // The tyres' outer shoulders: what a car tipping over pivots on (the tyre's own support
        // fades out as it tilts past ~53 degrees, see computeStep).
        for (std::size_t w = 0; w < def_.wheels.size() && k < kHullPoints; ++w) {
            const WheelDef& wd = def_.wheels[w];
            const Vec3 hp = wd.suspension.hardpoint;
            const Real wcz = hp.z - (wd.suspension.travel - rideCompression_[w]);
            const Real side = hp.y >= 0 ? 1.0 : -1.0;
            hull_[static_cast<std::size_t>(k++)] = {hp.x, hp.y + side * 0.12, wcz - 0.7 * wd.tire.radius};
            shoulder_[w] = hull_[static_cast<std::size_t>(k - 1)];
        }
        hullCount_ = k;
        cageBox_ = {front, rear, hw, floor, roof, belt, rr, rf};
    }
    setFidelity(fidelity_);
}

void Vehicle::setTyreTemperatures(Real celsius) {
    for (auto& ws : wheels_) ws.temperature = celsius;
}

bool Vehicle::enableDamage(const DamageConfig& cfg) {
    if (!valid_) return false;
    const std::size_t nw = def_.wheels.size();
    std::array<Vec3, kMaxWheels> mounts{};
    std::array<Real, kMaxWheels> load{};
    staticWheelLoads(load);
    for (std::size_t w = 0; w < nw; ++w) {
        mounts[w] = def_.wheels[w].suspension.hardpoint;
        intactTire_[w] = def_.wheels[w].tire;
        appliedDeflation_[w] = 0;
    }
    damage_.build(cfg, cageBox_, def_.mass, nw, mounts, shoulder_, load, def_.aero.ambientTemperature);
    damage_.snapToBody(body_);
    drivetrain_.setEngineDamage(1, false);
    return true;
}

void Vehicle::disableDamage() {
    repairDamage();
    damage_.disable();
}

void Vehicle::repairDamage() {
    if (!damage_.enabled()) return;
    damage_.repair(def_.aero.ambientTemperature);
    for (std::size_t w = 0; w < def_.wheels.size(); ++w) {
        def_.wheels[w].tire = intactTire_[w];
        tires_[w] = MagicFormulaTire(intactTire_[w]);
        appliedDeflation_[w] = 0;
    }
    drivetrain_.setEngineDamage(1, false);
    damage_.snapToBody(body_);
}

// A punctured tyre going flat: it rides on the rim (radius down ~18%), its vertical stiffness
// and grip collapse and it drags. Applied as the tyre's parameters, rebuilt when it changes.
void Vehicle::applyTyreDamage() {
    const DamageState& ds = damage_.state();
    for (std::size_t w = 0; w < def_.wheels.size(); ++w) {
        const Real d = ds.wheels[w].deflation;
        if (std::abs(d - appliedDeflation_[w]) < 0.01 && !(d >= 1 && appliedDeflation_[w] < 1)) continue;
        TireParams p = intactTire_[w];
        p.radius -= d * 0.18 * p.radius;
        p.verticalStiffness *= 1 - 0.85 * d;
        p.muX *= 1 - 0.45 * d;
        p.muY *= 1 - 0.45 * d;
        p.rollingResistance *= 1 + 10 * d;
        def_.wheels[w].tire = p;
        tires_[w] = MagicFormulaTire(p);
        appliedDeflation_[w] = d;
    }
}

void Vehicle::setFidelity(const VehicleFidelity& f) {
    fidelity_ = f;
    fidelity_.contactSamples = sanitizeSamples(f.contactSamples);
    for (std::size_t w = 0; w < def_.wheels.size(); ++w) {
        wheels_[w].prevPen.fill(0);
        wheels_[w].compressionRate = 0;
    }
}

Real Vehicle::yawRate() const { return body_.orientation.inverseRotate(body_.angularVelocity).z; }

Real Vehicle::wheelSteerAngle(std::size_t w, Real ref) const {
    const WheelDef& wd = def_.wheels[w];
    // Static toe plus the linkage's bump steer at the current travel.
    const Real toe = wd.staticToe + (linkage_[w].valid() ? linkagePose(w, wheels_[w].compression).toe : 0.0);
    if (maxSteerRef_ <= 0 || wd.maxSteerAngle <= 0) return wd.isLeft ? -toe : toe;
    Real delta = ref * (wd.maxSteerAngle / maxSteerRef_);
    if (std::abs(delta) > 1e-6 && front_[w] && def_.steeringAckermann > 0 && def_.axles.size() >= 2) {
        const Real L = def_.wheelbase();
        const Real halfTrack = std::abs(wd.suspension.hardpoint.y);
        const Real R = L / std::tan(std::abs(delta));
        const bool inner = (delta > 0) == wd.isLeft;
        const Real ideal = std::atan(L / std::max(R + (inner ? -halfTrack : halfTrack), 0.1));
        delta = sign(delta) * lerp(std::abs(delta), ideal, def_.steeringAckermann);
    }
    // Toe-in convention: positive static toe points both wheels toward the centreline.
    return delta + (wd.isLeft ? -toe : toe);
}

LinkagePose Vehicle::linkagePose(std::size_t w, Real compression) const {
    return linkage_[w].valid() ? linkage_[w].at(compression + kinT0_[w]) : LinkagePose{};
}

Real Vehicle::springRatio(std::size_t w, Real compression) const {
    if (!linkage_[w].valid()) return 1.0;
    return linkage_[w].at(compression + kinT0_[w]).motionRatio / linkage_[w].designMotionRatio();
}

Real Vehicle::springForceAt(std::size_t w, Real compression) const {
    const SuspensionDef& sd = def_.wheels[w].suspension;
    if (!linkage_[w].valid()) return sd.springPreload + sd.springRate * compression;
    // Wheel rate k at the design position; the coil's deflection is the integral
    // of the motion ratio, and its force comes back to the wheel through the
    // ratio again: F = k * rho(x) * (S(x) - S(droop)).
    const Real t = compression + kinT0_[w];
    const Real rho = linkage_[w].at(t).motionRatio / linkage_[w].designMotionRatio();
    return sd.springPreload + sd.springRate * rho * (linkage_[w].springDeflection(t) - kinDroopS_[w]);
}

VehicleSensors Vehicle::buildSensors(Real dt) const {
    VehicleSensors s;
    s.dt = dt;
    const Vec3 vb = bodyVelocity();
    s.speed = vb.x;
    s.lateralVelocity = vb.y;
    s.yawRate = yawRate();
    s.wheelbase = def_.wheelbase();
    s.cgToFrontAxle = def_.wheels[def_.axles.front().leftWheel].suspension.hardpoint.x - def_.centerOfMass.x;
    s.maxSteerAngle = maxSteerRef_;
    s.wheelCount = def_.wheels.size();
    Real muSum = 0;
    int muCount = 0;
    for (std::size_t w = 0; w < s.wheelCount; ++w) {
        const WheelState& ws = wheels_[w];
        const WheelDef& wd = def_.wheels[w];
        s.wheelOmega[w] = drivetrain_.wheelOmega(w);
        s.wheelRadius[w] = wd.tire.radius;
        s.wheelGroundSpeed[w] = ws.contactVx;
        s.wheelDriven[w] = driven_[w];
        s.wheelIsFront[w] = front_[w];
        s.wheelIsLeft[w] = wd.isLeft;
        s.wheelInContact[w] = ws.contact;
        s.maxBrakeTorque[w] = wd.maxBrakeTorque;
        s.handbrakeTorque[w] = wd.handbrakeTorque;
        if (ws.contact) {
            muSum += wd.tire.muY * ws.hit.surface.frictionScale;
            ++muCount;
        }
    }
    s.frictionEstimate = muCount ? muSum / muCount : 1.0;
    s.engineRpm = radSToRpm(drivetrain_.engineOmega());
    s.idleRpm = def_.drivetrain.engine.idleRpm;
    s.limiterRpm = def_.drivetrain.engine.limiterRpm;
    s.gear = assists_.currentGear();
    s.forwardGears = drivetrain_.forwardGearCount();
    s.shiftTime = def_.drivetrain.gearbox.shiftTime;
    for (int g = 1; g <= s.forwardGears && g < static_cast<int>(s.gearRpm.size()); ++g)
        s.gearRpm[g] = radSToRpm(drivetrain_.engineOmegaForGear(g));
    return s;
}

void Vehicle::setRollingSpeed(Real v) {
    body_.velocity = body_.orientation.rotate({v, 0, 0});
    body_.angularVelocity = {};
    for (std::size_t w = 0; w < def_.wheels.size(); ++w) {
        drivetrain_.setWheelOmega(w, v / def_.wheels[w].tire.radius);
        wheels_[w].slip = {};
        wheels_[w].contactVx = v;
    }
    const Real idle = rpmToRadS(def_.drivetrain.engine.idleRpm);
    drivetrain_.setEngineOmega(std::max(idle, drivetrain_.engineOmegaForGear(assists_.currentGear())));
    prevVelocity_ = body_.velocity;
    prevAngularVelocity_ = body_.angularVelocity;
    accel_ = angAccel_ = {};
}

void Vehicle::selectGearForSpeed(Real v) {
    const DrivetrainDef& dt = def_.drivetrain;
    const auto& da = dt.drivenAxles.front();
    const Real r = def_.wheels[static_cast<std::size_t>(def_.axles[da.axleIndex].leftWheel)].tire.radius;
    const Real wheelOmega = std::abs(v) / r;
    int gear = 1;
    for (int g = 1; g <= static_cast<int>(dt.gearbox.forwardRatios.size()); ++g) {
        const Real rpm = radSToRpm(wheelOmega * dt.gearbox.forwardRatios[static_cast<std::size_t>(g - 1)] *
                                   da.differential.finalDrive);
        if (rpm >= 0.35 * dt.engine.limiterRpm) gear = g;
        else break;
    }
    // Never pick a gear that would put the engine over ~90% of the limiter.
    while (gear > 1 &&
           radSToRpm(wheelOmega * dt.gearbox.forwardRatios[static_cast<std::size_t>(gear - 1)] * da.differential.finalDrive) >
               0.9 * dt.engine.limiterRpm)
        --gear;
    assists_.setGear(gear);
}

void Vehicle::staticWheelLoads(std::array<Real, kMaxWheels>& load) const {
    const Real W = def_.mass * kGravity;
    load.fill(0);
    std::array<Real, kMaxAxles> axleLoad{};
    const std::size_t na = def_.axles.size();
    auto axleX = [&](std::size_t a) {
        return 0.5 * (def_.wheels[def_.axles[a].leftWheel].suspension.hardpoint.x +
                      def_.wheels[def_.axles[a].rightWheel].suspension.hardpoint.x);
    };
    if (na >= 2) {
        // Lever rule between the first and last axle; intermediate axles share equally (approximation).
        const Real xf = axleX(0), xr = axleX(na - 1);
        const Real mid = na > 2 ? W * static_cast<Real>(na - 2) / static_cast<Real>(na) : 0.0;
        const Real rem = W - mid;
        axleLoad[0] = rem * (def_.centerOfMass.x - xr) / (xf - xr);
        axleLoad[na - 1] = rem - axleLoad[0];
        for (std::size_t a = 1; a + 1 < na; ++a) axleLoad[a] = mid / static_cast<Real>(na - 2);
    } else {
        axleLoad[0] = W;
    }
    for (std::size_t a = 0; a < na; ++a) {
        const auto l = static_cast<std::size_t>(def_.axles[a].leftWheel);
        const auto r = static_cast<std::size_t>(def_.axles[a].rightWheel);
        const Real yl = def_.wheels[l].suspension.hardpoint.y, yr = def_.wheels[r].suspension.hardpoint.y;
        const Real fl = std::abs(yl - yr) > 1e-6 ? (def_.centerOfMass.y - yr) / (yl - yr) : 0.5;
        load[l] = axleLoad[a] * fl;
        load[r] = axleLoad[a] * (1 - fl);
    }
}

void Vehicle::placeOnGround(Real x, Real y, Real yaw, const GroundQuery& ground, Real vx, Real vy, Real yawRate) {
    if (!valid_) return;
    faulted_ = false;
    havePrediction_ = false;
    forces_ = {};
    const std::size_t nw = def_.wheels.size();
    std::array<Real, kMaxWheels> load{};
    staticWheelLoads(load);
    const Quat qYaw = Quat::fromAxisAngle({0, 0, 1}, yaw);
    const Real zHint = body_.position.z;

    // Required hardpoint height above ground per wheel in static equilibrium.
    DenseSystem<3> ls;
    ls.reset(3);
    std::array<Real, kMaxWheels> xs{}, pens{};
    int hits = 0;
    Real groundSum = 0;
    std::array<Real, kMaxWheels> groundZ{};
    std::array<bool, kMaxWheels> groundHit{};
    for (std::size_t w = 0; w < nw; ++w) {
        const WheelDef& wd = def_.wheels[w];
        const Vec3 hpXY = qYaw.rotate({wd.suspension.hardpoint.x, wd.suspension.hardpoint.y, 0});
        const GroundHit h = ground.raycast({x + hpXY.x, y + hpXY.y, zHint + 5.0}, {0, 0, -1}, 25.0);
        ++groundQueries_;
        groundHit[w] = h.hit;
        groundZ[w] = h.point.z;
        if (h.hit) { groundSum += h.point.z; ++hits; }
    }
    const Real groundAvg = hits ? groundSum / hits : zHint - 0.6;
    for (std::size_t w = 0; w < nw; ++w) {
        const WheelDef& wd = def_.wheels[w];
        const SuspensionDef& sd = wd.suspension;
        const bool unsprung = usesUnsprung(w);
        const Real fs = load[w] - (unsprung ? wd.unsprungMass * kGravity : 0.0);
        xs[w] = clamp((fs - sd.springPreload) / sd.springRate, 0.0, sd.travel);
        if (linkage_[w].valid())  // the motion ratio makes the wheel rate vary: a few Newton steps
            for (int it = 0; it < 6; ++it) {
                const Real k = sd.springRate * springRatio(w, xs[w]) * springRatio(w, xs[w]);
                xs[w] = clamp(xs[w] + (fs - springForceAt(w, xs[w])) / std::max(k, 1.0), 0.0, sd.travel);
            }
        pens[w] = unsprung ? load[w] / wd.tire.verticalStiffness : 0.0;
        const Real hpAbove = wd.tire.radius - pens[w] + (sd.travel - xs[w]);
        const Real zReq = (groundHit[w] ? groundZ[w] : groundAvg) + hpAbove - sd.hardpoint.z;
        // zReq = z0 - hp.x * pitch + hp.y * roll (small angles)
        const Real row[3] = {1.0, -sd.hardpoint.x, sd.hardpoint.y};
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) ls.a(i, j) += row[i] * row[j];
            ls.b(i) += row[i] * zReq;
        }
    }
    Real z0 = groundAvg + 0.5, pitch = 0, roll = 0;
    if (nw >= 3 && ls.solve()) {
        z0 = ls.x(0);
        pitch = clamp(ls.x(1), -0.5, 0.5);
        roll = clamp(ls.x(2), -0.5, 0.5);
    }
    body_.orientation = qYaw * Quat::fromAxisAngle({0, 1, 0}, pitch) * Quat::fromAxisAngle({1, 0, 0}, roll);
    body_.orientation.normalize();
    body_.position = {x, y, z0};

    for (std::size_t w = 0; w < nw; ++w) {
        WheelState& ws = wheels_[w];
        ws = WheelState{};
        ws.temperature = def_.aero.ambientTemperature;
        ws.compression = ws.prevCompression = xs[w];
        ws.prevPen.fill(pens[w]);
    }
    setRollingSpeed(vx);
    const Vec3 up = body_.orientation.rotate({0, 0, 1});
    body_.velocity = body_.orientation.rotate({vx, vy, 0});
    body_.angularVelocity = up * yawRate;
    prevVelocity_ = body_.velocity;
    prevAngularVelocity_ = body_.angularVelocity;
    accel_ = angAccel_ = {};
    if (damage_.enabled()) damage_.snapToBody(body_);
}

void Vehicle::reset(const Vec3& position, Real yaw, const GroundQuery& ground) {
    if (!valid_) return;
    body_ = RigidBodyState{};
    body_.position = position;  // z is the search hint
    assists_.setGear(1);
    placeOnGround(position.x, position.y, yaw, ground);
    DriverInput hold;
    hold.brake = 1.0;
    const Real dt = 1.0 / 240.0;
    for (int i = 0; i < 120; ++i) step(dt, hold, ground);
    body_.velocity = {};
    body_.angularVelocity = {};
    setRollingSpeed(0);
    time_ = 0;
}

void Vehicle::setChassisState(const RigidBodyState& s) {
    if (havePrediction_) {
        // Compare the host's integrated chassis with our own substep prediction.
        const Vec3 comP = predicted_.position + predicted_.orientation.rotate(def_.centerOfMass);
        const Vec3 comH = s.position + s.orientation.rotate(def_.centerOfMass);
        const Quat dq = s.orientation * predicted_.orientation.conjugate();
        hostSync_.positionError = length(comH - comP);
        hostSync_.angleError = 2.0 * std::acos(clamp(std::abs(dq.w), 0.0, 1.0));
        hostSync_.maxPositionError = std::max(hostSync_.maxPositionError, hostSync_.positionError);
        hostSync_.maxAngleError = std::max(hostSync_.maxAngleError, hostSync_.angleError);
        ++hostSync_.syncs;
        // Unsprung wheels have their own inertia: when the host's body pose or
        // velocity differs from the one the substeps advanced, the *wheel*
        // stays where it was and the suspension absorbs the difference (instead
        // of the tire penetration jumping by the pose error every host step).
        // Large differences (teleports, respawns) are not carried over.
        const Vec3 upH = s.orientation.rotate({0, 0, 1});
        const Vec3 comHP = comH, comPP = comP;
        for (std::size_t w = 0; w < def_.wheels.size(); ++w) {
            if (!usesUnsprung(w)) continue;
            const Vec3 hpLocal = def_.wheels[w].suspension.hardpoint;
            const Vec3 hpP = predicted_.position + predicted_.orientation.rotate(hpLocal);
            const Vec3 hpH = s.position + s.orientation.rotate(hpLocal);
            const Real dx = dot(hpP - hpH, upH);
            if (std::abs(dx) > kMaxCarriedPoseError) continue;
            const Vec3 vP = predicted_.velocity + cross(predicted_.angularVelocity, hpP - comPP);
            const Vec3 vH = s.velocity + cross(s.angularVelocity, hpH - comHP);
            wheels_[w].compression += dx;
            wheels_[w].compressionRate += dot(vP - vH, upH);
        }
        havePrediction_ = false;
    }
    updateAccelEstimate(s.velocity, s.angularVelocity, lastDt_);
    body_ = s;
}

void Vehicle::updateAccelEstimate(const Vec3& vNew, const Vec3& wNew, Real dt) {
    accel_ = (vNew - prevVelocity_) / dt;
    angAccel_ = (wNew - prevAngularVelocity_) / dt;
    prevVelocity_ = vNew;
    prevAngularVelocity_ = wNew;
}

void Vehicle::groundPrefetchRequests(std::vector<GroundStripRequest>& out) const {
    const Quat& q = body_.orientation;
    const Vec3 up = q.rotate({0, 0, 1});
    const Vec3 com = centerOfMassWorld();
    for (std::size_t w = 0; w < def_.wheels.size(); ++w) {
        const WheelDef& wd = def_.wheels[w];
        const WheelState& ws = wheels_[w];
        const Vec3 hp = body_.position + q.rotate(wd.suspension.hardpoint);
        GroundStripRequest r;
        r.center = hp - up * (wd.suspension.travel - ws.compression + wd.tire.radius);
        r.velocity = body_.velocity + cross(body_.angularVelocity, r.center - com);
        r.heading = q.rotate({std::cos(ws.steer), std::sin(ws.steer), 0});
        // Cover the envelope probes too (they reach 0.7 r ahead of / behind the patch).
        const bool probes = fidelity_.contactSamples > 1 && fidelity_.envelopeProbes > 0 && usesUnsprung(w);
        r.halfLength = probes ? std::max(wd.tire.patchHalfLength, 0.7 * wd.tire.radius + 0.02) : wd.tire.patchHalfLength;
        r.halfWidth = wd.tire.patchHalfWidth;
        out.push_back(r);
    }
}

int Vehicle::sampleContact(std::size_t w, const Vec3& hp, const Vec3& up, const Vec3& heading,
                           const GroundQuery& ground, std::array<Sample, kMaxContactSamples>& out, int& patchCount) {
    const WheelDef& wd = def_.wheels[w];
    std::array<Offset, kMaxContactSamples> layout{};
    patchCount = sampleLayout(fidelity_.contactSamples, layout);
    int n = patchCount;
    // Envelope probes at fractions of the radius ahead of and behind the patch.
    static constexpr Real kProbeFraction[kMaxEnvelopeProbes] = {0.40, 0.55, 0.70};
    const int probes = (patchCount > 1 && usesUnsprung(w)) ? std::clamp(fidelity_.envelopeProbes, 0, kMaxEnvelopeProbes) : 0;
    for (int side = -1; side <= 1; side += 2)
        for (int j = 0; j < probes; ++j) {
            const Real sOff = std::max(kProbeFraction[j] * wd.tire.radius, wd.tire.patchHalfLength + 0.01);
            layout[static_cast<std::size_t>(n++)] = {side * sOff / std::max(wd.tire.patchHalfLength, 1e-6), 0.0};
        }
    const Vec3 lateral = cross(up, heading);
    const Real maxDist = kRayStartOffset + wd.suspension.travel + wd.tire.radius + kRayExtra;
    for (int k = 0; k < n; ++k) {
        const Real s = layout[k].s * wd.tire.patchHalfLength;
        const Real l = layout[k].l * wd.tire.patchHalfWidth;
        out[k].probe = k >= patchCount;
        const Vec3 origin = hp + heading * s + lateral * l + up * kRayStartOffset;
        const GroundHit h = ground.raycast(origin, -up, maxDist);
        ++groundQueries_;
        Sample& o = out[k];
        o.hit = h.hit;
        o.s = s;
        if (h.hit) {
            o.d = h.distance - kRayStartOffset;
            o.point = h.point;
            o.normal = h.normal;
            const Real tex = surfaceTextureHeight(h.surface, h.point.x, h.point.y);
            if (tex != 0) {
                o.d -= tex / std::max(dot(-up, Vec3{0, 0, -1}), 0.2);
                o.point.z += tex;
            }
            if (k == 0 || !wheels_[w].hit.hit) wheels_[w].hit = h;
        }
    }
    return n;
}

bool Vehicle::stateFinite() const {
    if (!isFinite(body_.position) || !isFinite(body_.velocity) || !isFinite(body_.angularVelocity)) return false;
    if (!std::isfinite(body_.orientation.w) || !std::isfinite(body_.orientation.x) ||
        !std::isfinite(body_.orientation.y) || !std::isfinite(body_.orientation.z))
        return false;
    if (!isFinite(forces_.force) || !isFinite(forces_.torque)) return false;
    for (std::size_t w = 0; w < def_.wheels.size(); ++w)
        if (!std::isfinite(wheels_[w].compression) || !std::isfinite(wheels_[w].omega)) return false;
    return std::isfinite(drivetrain_.engineOmega());
}

void Vehicle::step(Real dt, const DriverInput& input, const GroundQuery& ground) {
    if (!valid_ || faulted_ || !(dt > 0) || !std::isfinite(dt)) return;
    const RigidBodyState before = body_;
    computeStep(dt, input, ground);
    if (chassisMode_ == ChassisMode::Internal) integrateChassis(dt);
    if (damage_.enabled() && chassisMode_ == ChassisMode::Internal) stepDamage(dt, before, ground);
    if (!stateFinite()) {
        body_ = before;
        forces_ = {};
        faulted_ = true;
        return;
    }
    time_ += dt;
    lastDt_ = dt;
}

void Vehicle::stepDamage(Real dt, const RigidBodyState& before, const GroundQuery& ground) {
    if (damage_.config().structure) {
        DamageModel::BodyParams bp;
        bp.mass = def_.mass;
        bp.invInertia = {1.0 / def_.inertia.x, 1.0 / def_.inertia.y, 1.0 / def_.inertia.z};
        bp.com = def_.centerOfMass;
        bp.bodyFriction = def_.body.friction;
        const Vec3 v0 = body_.velocity, w0 = body_.angularVelocity;
        if (damage_.stepStructure(dt, body_, before, bp, ground, obstacles_, cageGroundGate_, groundQueries_)) {
            // the structure's contact and crush corrections are part of this step's acceleration
            accel_ += (body_.velocity - v0) / dt;
            angAccel_ += (body_.angularVelocity - w0) / dt;
            prevVelocity_ = body_.velocity;
            prevAngularVelocity_ = body_.angularVelocity;
            refreshBodyTelemetry(dt);
            if (fidelity_.telemetry) telem_.accelBody = body_.orientation.inverseRotate(accel_);
        }
        bodyContacts_ = damage_.state().nodeContacts;
        bodyContactForce_ = damage_.contactForce();
        telem_.bodyContacts = bodyContacts_;
        telem_.bodyContactForce = bodyContactForce_;
        telem_.groundQueries = groundQueries_;
    }
    if (damage_.config().mechanical) {
        std::array<Real, kMaxWheels> load{}, speed{};
        for (std::size_t w = 0; w < def_.wheels.size(); ++w) {
            load[w] = wheels_[w].load;
            speed[w] = wheels_[w].contactVx;
        }
        const bool running = drivetrain_.engineOmega() > rpmToRadS(300.0);
        damage_.stepMechanical(dt, load, tyreDeflection_, speed, lastCmd_.throttle, running, def_.aero.ambientTemperature);
        drivetrain_.setEngineDamage(damage_.state().torqueScale, damage_.state().engineSeized);
        applyTyreDamage();
    }
}

void Vehicle::stepSubstepped(Real hostDt, int substeps, const DriverInput& input, const GroundQuery& ground) {
    if (!valid_ || faulted_ || !(hostDt > 0) || !std::isfinite(hostDt)) return;
    substeps = std::max(substeps, 1);
    const Real h = hostDt / substeps;
    if (chassisMode_ == ChassisMode::Internal) {
        DriverInput in = input;
        for (int k = 0; k < substeps; ++k) {
            step(h, in, ground);
            in.shiftUp = in.shiftDown = false;  // edge-triggered inputs apply once
        }
        return;
    }
    // External: integrate a private copy of the chassis across the substeps,
    // then hand the host the average force/torque (same impulse over hostDt).
    const RigidBodyState start = body_;
    const Vec3 gravityForce = hostGravity_ ? Vec3{0, 0, -kGravity * def_.mass} : Vec3{};
    Vec3 impulse{}, angularImpulse{};
    DriverInput in = input;
    for (int k = 0; k < substeps; ++k) {
        computeStep(h, in, ground);
        impulse += forces_.force * h;
        angularImpulse += forces_.torque * h;
        integrateChassis(h, gravityForce);
        time_ += h;
        in.shiftUp = in.shiftDown = false;  // edge-triggered inputs apply once
    }
    forces_ = {impulse / hostDt, angularImpulse / hostDt};
    if (!stateFinite()) {
        body_ = start;
        forces_ = {};
        faulted_ = true;
        havePrediction_ = false;
        return;
    }
    predicted_ = body_;
    havePrediction_ = true;
    body_ = start;  // the host owns the chassis; it calls setChassisState() next
    prevVelocity_ = start.velocity;
    prevAngularVelocity_ = start.angularVelocity;
    lastDt_ = hostDt;
}

void Vehicle::computeStep(Real dt, const DriverInput& input, const GroundQuery& ground) {
    const std::size_t nw = def_.wheels.size();
    const ActuatorCommands cmd = assists_.process(input, buildSensors(dt));
    lastCmd_ = cmd;

    const Quat& q = body_.orientation;
    const Vec3 up = q.rotate({0, 0, 1});
    const Vec3 comWorld = centerOfMassWorld();
    const Vec3 gravity{0, 0, -kGravity};

    // ---- 1. Contact sampling + suspension (unsprung DOF or massless) ----
    std::array<Sample, kMaxContactSamples> samples{};
    std::array<Real, kMaxWheels> prevComp{};
    for (std::size_t w = 0; w < nw; ++w) prevComp[w] = wheels_[w].compression;

    for (std::size_t w = 0; w < nw; ++w) {
        WheelState& ws = wheels_[w];
        const WheelDef& wd = def_.wheels[w];
        const SuspensionDef& sd = wd.suspension;
        const Real r = wd.tire.radius;
        const Real side = wd.isLeft ? 1.0 : -1.0;
        // The linkage's pose at the start-of-step travel: where the wheel centre
        // has scrubbed to, and the direction it moves per metre of compression
        // (the generalised direction of this wheel's DOF). Sliders: no scrub, +up.
        const LinkagePose pose = linkagePose(w, ws.compression);
        const bool dmg = damage_.enabled() && damage_.config().mechanical;
        const WheelDamage* wdmg = dmg ? &damage_.state().wheels[w] : nullptr;
        const Vec3 mount = wdmg ? wdmg->mountOffset : Vec3{};
        const Vec3 hp = body_.position + q.rotate(sd.hardpoint + mount + Vec3{pose.lon, side * pose.lat, 0});
        const Vec3 dof = q.rotate({pose.dLon, side * pose.dLat, 1.0});
        const Real dof2 = dot(dof, dof);
        ws.dofDir = dof;
        // The contact patch's own path: the upright also tilts as it rises (the
        // camber curve), which swings the bottom of the tyre sideways by r per
        // radian. Forces at the patch do their work along this direction; it is
        // what the front-view roll centre encodes.
        const Vec3 dofPatch = q.rotate({pose.dLon, side * (pose.dLat - r * pose.dCamber), 1.0});
        {
            Real ref = cmd.steerAngle;
            if (dmg && wd.maxSteerAngle > 0) ref = ref * damage_.state().steeringLock + damage_.state().steeringOffset;
            ws.steer = wheelSteerAngle(w, ref);
            if (wdmg) ws.steer += wdmg->toe + (wd.isLeft ? 1.0 : -1.0) * wdmg->bend;  // bends go toe-out
        }
        const bool detached = wdmg && wdmg->detached;
        const Vec3 headingWorld = q.rotate({std::cos(ws.steer), std::sin(ws.steer), 0});
        ws.hit = GroundHit{};
        int patchCount = 1;
        if (detached) samples[0] = Sample{};
        const int n = detached ? 0 : sampleContact(w, hp, up, headingWorld, ground, samples, patchCount);
        const Real weight = 1.0 / patchCount;
        // A car tilted far over (past ~53 degrees) rests on the tyre's sidewall,
        // not on a point found straight "down" its own axis, which by then lies well off to the
        // side and would prop the car back up. The tyre's support fades out by ~70 degrees and
        // the body hull's sills take over (rollover). 1 in any normal driving.
        // Measured against the vertical (kerb edges have steep local normals under an upright car)
        // or, on steep ground, against the ground this wheel found, whichever is better aligned:
        // a car on a banking, a wall or inside a loop is on its wheels, not tipping over.
        Real align = up.z;
        {
            // only ground within the wheel's reach: tilted far over, the ray down the car's own
            // axis finds ground well off to the side, where the tyre is not
            const Real reach = wd.suspension.travel + wd.tire.radius + 0.05;
            Vec3 ns{};
            for (int k = 0; k < n; ++k) {
                const Sample& sk = samples[static_cast<std::size_t>(k)];
                if (sk.hit && !sk.probe && sk.d < reach) ns += sk.normal;
            }
            const Real l = length(ns);
            // on near-level ground the two measures agree; only steep ground (banking, wall, loop)
            // can make the car's own ground the better reference
            if (l > 1e-9 && ns.z / l < 0.9) align = std::max(align, dot(up, ns) / l);
        }
        const Real tc = clamp((align - 0.34) / 0.26, 0.0, 1.0);
        const Real tilt = tc * tc * (3 - 2 * tc);

        // ARB partner (explicit coupling to the other wheel's previous compression).
        const AxleDef& ax = def_.axles[static_cast<std::size_t>(axleOf_[w])];
        const auto partner = static_cast<std::size_t>(static_cast<int>(w) == ax.leftWheel ? ax.rightWheel : ax.leftWheel);
        const Real karb = ax.antiRollBarRate;
        const Real xo = prevComp[partner];

        ws.prevCompression = ws.compression;
        Vec3 patch{}, nrm{};
        Real patchW = 0;
        ws.tireNormalForce = {};
        ws.relativeAccel = 0;
        ContactDebug& dbg = debug_[w];
        dbg.samples = n;

        if (usesUnsprung(w)) {
            const Real m = wd.unsprungMass;
            const Real kt = wd.tire.verticalStiffness, ct = wd.tire.verticalDamping;
            const Real x0 = ws.compression, xd0 = ws.compressionRate;
            const Vec3 rhp = hp - comWorld;
            const Vec3 aHp = accel_ + cross(angAccel_, rhp) + cross(body_.angularVelocity, cross(body_.angularVelocity, rhp));

            // Tire: one spring per contact sample, flat-footprint penetration.
            Real P0 = 0, Pprev = 0, Wact = 0;
            std::array<Real, kMaxContactSamples> pen0{}, ru{};
            std::array<Vec3, kMaxContactSamples> radial{};
            const Vec3 wc0 = hp - up * (sd.travel - x0);
            for (int k = 0; k < n; ++k) {
                Real slope = 1.0;  // d(penetration)/d(compression), negated
                if (!samples[k].hit) {
                    pen0[k] = -1.0;
                } else if (samples[k].probe) {
                    // Radial penetration of the ground point into the tire circle.
                    const Vec3 d = wc0 - samples[k].point;
                    const Real dist = length(d);
                    pen0[k] = r - dist;
                    radial[k] = dist > 1e-9 ? d / dist : up;
                    slope = std::max(0.0, dot(radial[k], dofPatch));
                } else {
                    pen0[k] = (sd.travel - x0) + r - samples[k].d;  // flat footprint
                    // Patch samples push along their ground normal (tilted on curb and
                    // pothole edges, which produces the longitudinal impact force).
                    radial[k] = samples[k].normal;
                }
                if (pen0[k] <= 0) continue;
                ru[k] = std::max(0.0, dot(radial[k], dofPatch)) * tilt;  // work the sample does on the DOF
                P0 += weight * pen0[k] * ru[k];
                Pprev += weight * ws.prevPen[k] * ru[k];
                Wact += weight * ru[k] * slope;
            }
            // Suspension spring/damper/ARB/bump stops, linearised at x0. A linkage's
            // motion ratio scales the wheel rate (rho^2) and the damping (rho^2).
            const Real rho = springRatio(w, x0);
            Real c = 0;
            Real fd0 = damperForce(sd, xd0, c) * rho * rho;  // linearised two-stage damper: fd ~ fd0 + c (xd - xd0)
            c *= rho * rho;
            Real As = springForceAt(w, x0) + karb * (x0 - xo) + (fd0 - c * xd0);
            Real Ks = sd.springRate * rho * rho + karb;
            if (x0 > sd.travel) { As += sd.bumpStopRate * (x0 - sd.travel); Ks += sd.bumpStopRate; }
            if (x0 < 0) { As += sd.bumpStopRate * x0; Ks += sd.bumpStopRate; }
            const Real Bs = Ks * dt + c;
            Real At = kt * P0 + ct * (P0 - Pprev) / dt;
            Real Bt = (kt * dt + ct) * Wact;
            // The wheel's generalised mass along its path, gravity and the body's
            // acceleration projected on it, and the work last step's tyre
            // traction does on it: on a real linkage the path has sideways and
            // fore/aft components, so cornering force jacks the suspension
            // (roll centre) and braking force stiffens it (anti-dive).
            const Vec3 traction = ws.longDir * ws.forces.fx + ws.latDir * ws.forces.fy;
            const Real mg = m * dof2;
            const Real rhs0 = mg * xd0 / dt - As + m * dot(gravity, dof) - m * dot(aHp, dof) + (linkage_[w].valid() ? dot(traction, dofPatch) : 0.0);
            Real xd1 = (rhs0 + At) / (mg / dt + Bt + Bs);
            if (At - Bt * xd1 < 0) {  // tire would pull: it leaves the ground this step
                At = Bt = 0;
                xd1 = rhs0 / (mg / dt + Bs);
            }
            const Real x1 = x0 + dt * xd1;
            ws.compression = x1;
            ws.compressionRate = xd1;
            ws.relativeAccel = (xd1 - xd0) / dt;
            ws.springForce = As + Bs * xd1;

            // Final per-sample radial forces at x1.
            const Vec3 wc1 = hp - up * (sd.travel - x1);
            Vec3 ft{};
            for (int k = 0; k < n; ++k) {
                Real pen = -1.0;
                Vec3 dir = samples[k].normal;
                if (samples[k].hit && samples[k].probe) {
                    const Vec3 d = wc1 - samples[k].point;
                    const Real dist = length(d);
                    pen = r - dist;
                    dir = dist > 1e-9 ? d / dist : up;
                } else if (samples[k].hit) {
                    pen = (sd.travel - x1) + r - samples[k].d;
                }
                Real fk = 0;
                if (pen > 0 && Bt > 0) fk = tilt * std::max(0.0, weight * (kt * pen + ct * (pen - ws.prevPen[k]) / dt));
                ws.prevPen[k] = pen > 0 ? pen : 0.0;
                dbg.point[k] = samples[k].hit ? samples[k].point : wc1 - up * r;
                dbg.penetration[k] = pen;
                if (fk <= 0) continue;
                ft += dir * fk;
                patch += samples[k].point * fk;
                nrm += dir * fk;
                patchW += fk;
            }
            ws.contact = patchW > 0;
            if (ws.contact) {
                ws.normal = normalized(nrm);
                ws.patchPoint = patch / patchW;
                ws.load = std::max(0.0, dot(ft, ws.normal));
                ws.tireNormalForce = ft;
            }
            dbg.wheelCenter = wc1;
            ws.wheelCenter = wc1;
        } else {
            // Massless wheel: compression follows the highest sample (rigid tire).
            Real best = -1e9;
            for (int k = 0; k < patchCount; ++k)
                if (samples[k].hit) best = std::max(best, sd.travel + r - samples[k].d);
            ws.contact = best >= 0;
            const Real x = ws.contact ? best : 0.0;
            ws.compression = x;
            Real f = springForceAt(w, std::min(x, sd.travel));
            if (x > sd.travel) f += sd.bumpStopRate * (x - sd.travel);
            const Real xdot = (x - ws.prevCompression) / dt;
            ws.compressionRate = xdot;
            Real slope = 0;
            const Real rho = springRatio(w, x);
            f += damperForce(sd, xdot, slope) * rho * rho;
            f += karb * (x - xo);
            ws.springForce = f;
            ws.load = ws.contact ? std::max(0.0, f) * tilt : 0.0;
            for (int k = 0; k < n; ++k) {
                dbg.point[k] = samples[k].point;
                dbg.penetration[k] = samples[k].hit ? sd.travel + r - samples[k].d - x : -1.0;
                if (!samples[k].hit || sd.travel + r - samples[k].d < best - 0.005) continue;
                patch += samples[k].point;
                nrm += samples[k].normal;
                patchW += 1;
            }
            if (ws.contact && patchW > 0) {
                ws.normal = normalized(nrm);
                ws.patchPoint = patch / patchW;
            }
            dbg.wheelCenter = hp - up * (sd.travel - x);
            ws.wheelCenter = dbg.wheelCenter;
        }
        if (detached) {  // the wheel is gone: nothing reaches the body through this corner
            ws.contact = false;
            ws.tireNormalForce = {};
            ws.relativeAccel = 0;
            ws.springForce = 0;
        }
        {
            Real defl = 0;
            for (int k = 0; k < n; ++k)
                if (!samples[static_cast<std::size_t>(k)].probe) defl = std::max(defl, ws.prevPen[static_cast<std::size_t>(k)]);
            tyreDeflection_[w] = usesUnsprung(w) ? defl : 0.0;
        }
        if (!ws.contact) {
            ws.load = 0;
            ws.normal = up;
            ws.patchPoint = hp - up * (sd.travel - ws.compression + r);
        }

        const Vec3 nn = ws.normal;
        ws.longDir = normalized(headingWorld - nn * dot(nn, headingWorld));
        ws.latDir = cross(nn, ws.longDir);
        const Vec3 vContact = body_.velocity + cross(body_.angularVelocity, ws.patchPoint - comWorld);
        ws.contactVx = dot(vContact, ws.longDir);
        ws.contactVy = dot(vContact, ws.latDir);
        {
            // Wheel inclination to the road: static camber + gain with
            // compression + caster with steer, through the body's roll.
            const Real camberKin = linkage_[w].valid() ? pose.camber : wd.camberGain * (ws.compression - rideCompression_[w]);
            Real g = side * (wd.staticCamber + camberKin) + (wdmg ? wdmg->camber : 0.0);
            if (wd.maxSteerAngle > 0) g += std::sin(wd.casterAngle) * std::sin(ws.steer);
            const Real cs = std::cos(ws.steer), sn = std::sin(ws.steer);
            const Vec3 axle = q.rotate({-sn * std::cos(g), cs * std::cos(g), -std::sin(g)});
            ws.camber = -std::asin(clamp(dot(axle, nn), -1.0, 1.0));
        }
        dbg.hardpoint = hp;
        dbg.patchCenter = ws.patchPoint;
        dbg.normal = nn;
        dbg.longDir = ws.longDir;
        dbg.latDir = ws.latDir;
        dbg.contact = ws.contact;
    }

    // ---- 2. Lateral transient slip, linearised longitudinal force ----
    std::array<WheelSolveInput, kMaxWheels> solveIn{};
    std::array<WheelSolveOutput, kMaxWheels> solveOut{};
    for (std::size_t w = 0; w < nw; ++w) {
        WheelState& ws = wheels_[w];
        const WheelDef& wd = def_.wheels[w];
        const MagicFormulaTire& tire = tires_[w];
        const Real r = wd.tire.radius;
        WheelSolveInput& si = solveIn[w];
        si.radius = r;
        si.inertia = wd.tire.wheelInertia;
        si.relaxationLength = wd.tire.relaxationLengthX;
        si.kappa0 = ws.slip.kappa;
        si.brakeCapacity = cmd.brakeTorque[w];
        const Real omega = drivetrain_.wheelOmega(w);
        if (ws.load > 0) {
            tire.integrateLateralSlip(ws.slip, ws.contactVx, ws.contactVy, dt);
            const Real speed = std::hypot(ws.contactVx, ws.contactVy);
            const TireForces f0 = tire.evaluate(ws.load, ws.slip.kappa, ws.slip.tanAlpha, ws.hit.surface, speed, ws.camber,
                                                wd.tire.thermalCapacity > 0 ? tire.temperatureGrip(ws.temperature) : 1.0);
            si.contactVx = ws.contactVx;
            si.fx0 = f0.fx;
            si.dFxdKappa = std::max(0.0, f0.dFxdKappa);
            const Real fade = std::max(0.0, 1.0 - speed / wd.tire.lowSpeedDampingFadeSpeed);
            si.slipDamping = wd.tire.lowSpeedDampingPerLoad * ws.load * fade;
            // Rolling resistance, plus, when the tyre is riding on water, the lifted share of the
            // load acting at the front of the patch (the water wedge): that spins the wheel down,
            // as measured in NASA's hydroplaning tests.
            const Real lifted = ws.hit.surface.waterDepth > 0 ? hydroplaneLift(wd.tire, ws.hit.surface.waterDepth, speed) : 0.0;
            si.externalTorque = -(wd.tire.rollingResistance * ws.hit.surface.rollingResistanceScale * ws.load * r +
                                  lifted * ws.load * wd.tire.patchHalfLength) *
                                std::tanh(omega * r / 0.5);
        } else {
            ws.slip.tanAlpha = 0;
            si.contactVx = omega * r;  // airborne: slip relaxes toward zero
        }
    }

    // ---- 3. Implicit drivetrain + wheel spin + longitudinal slip ----
    DrivetrainCommand dc;
    dc.throttle = cmd.throttle;
    if (def_.topSpeedLimit > 0) {
        // electronic top-speed limiter (e.g. 250 km/h): throttle fades over the last 2 %
        const Real over = (std::abs(forwardSpeed()) - 0.98 * def_.topSpeedLimit) / (0.02 * def_.topSpeedLimit);
        dc.throttle *= 1.0 - clamp(over, 0.0, 1.0);
    }
    dc.clutchEngagement = cmd.clutchEngagement;
    dc.gear = cmd.gear;
    drivetrain_.step(dt, dc, solveIn.data(), solveOut.data());

    // ---- 4. Chassis forces (non-gravitational first) ----
    Vec3 force{}, torque{};
    for (std::size_t w = 0; w < nw; ++w) {
        WheelState& ws = wheels_[w];
        const WheelDef& wd = def_.wheels[w];
        ws.omega = solveOut[w].omega;
        ws.slip.kappa = clamp(solveOut[w].kappa, -MagicFormulaTire::kSlipStateLimit, MagicFormulaTire::kSlipStateLimit);
        const bool unsprung = usesUnsprung(w);
        if (unsprung) {
            // Exact for a chassis integrated with the *total* mass M (so a host
            // rigid body can simply use M): along the wheel's path
            //   M a_body = F_tire + M g - m_u * xdd * dofDir
            // i.e. the body receives the tire normal force (at the patch) minus
            // the wheel's inertia relative to the body (at the wheel centre).
            const Vec3 fi = ws.dofDir * (-wd.unsprungMass * ws.relativeAccel);
            force += ws.tireNormalForce + fi;
            torque += cross(ws.patchPoint - comWorld, ws.tireNormalForce) + cross(ws.wheelCenter - comWorld, fi);
        }
        if (ws.load <= 0) {
            ws.forces = {};
            continue;
        }
        const Real speed = std::hypot(ws.contactVx, ws.contactVy);
        const bool thermal = wd.tire.thermalCapacity > 0;
        ws.forces = tires_[w].evaluate(ws.load, ws.slip.kappa, ws.slip.tanAlpha, ws.hit.surface, speed, ws.camber,
                                       thermal ? tires_[w].temperatureGrip(ws.temperature) : 1.0);
        ws.forces.fx = solveOut[w].fx;  // consistent with the implicit wheel solve
        ws.forces.fy -= solveIn[w].slipDamping * ws.contactVy;
        if (thermal) {
            // One-node tread temperature: sliding power (and a little rolling
            // hysteresis) heats it, air cools it, faster with speed.
            const Real vSlipX = ws.omega * wd.tire.radius - ws.contactVx;
            const Real slidePower = std::abs(ws.forces.fx * vSlipX) + std::abs(ws.forces.fy * ws.contactVy);
            const Real rolling = wd.tire.rollingResistance * ws.load * std::abs(ws.contactVx) * 0.5;
            const Real cool = (wd.tire.coolingBase + wd.tire.coolingPerSpeed * speed) * (ws.temperature - def_.aero.ambientTemperature);
            ws.temperature += dt * (wd.tire.heatFraction * slidePower + rolling - cool) / wd.tire.thermalCapacity;
            ws.temperature = clamp(ws.temperature, -30.0, 250.0);
        }
        const Vec3 n = ws.normal;
        Vec3 fc = ws.longDir * ws.forces.fx + ws.latDir * ws.forces.fy;
        if (!unsprung) fc += n * ws.load;
        // Standing water pushes on the tyre itself, not through its grip: a drag at the patch
        // against the rolling direction (one-sided water pulls the car toward it).
        if (ws.load > 0 && ws.hit.surface.waterDepth > 0)
            fc -= ws.longDir * (waterDrag(wd.tire, ws.hit.surface.waterDepth, ws.contactVx) * std::tanh(ws.contactVx / 0.5));
        force += fc;
        torque += cross(ws.patchPoint - comWorld, fc) + n * ws.forces.mz;
        debug_[w].fz = ws.load;
        debug_[w].fx = ws.forces.fx;
        debug_[w].fy = ws.forces.fy;
    }

    // Aerodynamics: relative to the air (wind), drag at the CoM, body/floor
    // downforce at each axle (ride-height sensitive when modelled), side force
    // in yaw, rear wing at its own position.
    {
        const AeroDef& aero = def_.aero;
        AeroTelemetry& at = aeroTel_;
        at = {};
        const Vec3 vAir = body_.velocity - wind_;
        const Real vmag = length(vAir);
        at.airspeed = vmag;
        const bool aeroDmg = damage_.enabled() && damage_.config().mechanical;
        const Real dragArea = aero.dragArea * (aeroDmg ? damage_.state().dragScale : 1.0);
        force += vAir * (-0.5 * aero.airDensity * dragArea * vmag);
        at.drag = 0.5 * aero.airDensity * dragArea * vmag * vmag;
        const Vec3 vb = q.inverseRotate(vAir);
        const Real yaw = (vb.x > 0.5) ? std::atan2(vb.y, vb.x) : 0.0;
        at.yaw = yaw;
        const Real qDyn = 0.5 * aero.airDensity * std::max(0.0, vb.x) * std::max(0.0, vb.x);
        const Real yawFactor = std::cos(yaw) * std::cos(yaw);
        if (aero.sideForceArea > 0 && vmag > 0.5) {
            const Vec3 fs = q.rotate({0, -0.5 * aero.airDensity * vmag * vmag * aero.sideForceArea * std::sin(yaw), 0});
            force += fs;
        }
        if (def_.axles.size() >= 2) {
            auto axlePoint = [&](const AxleDef& ax) {
                const Vec3 hp = (def_.wheels[ax.leftWheel].suspension.hardpoint + def_.wheels[ax.rightWheel].suspension.hardpoint) * 0.5;
                return body_.position + q.rotate(hp);
            };
            // Floor clearance at an axle from its suspension state (flat ground).
            auto clearance = [&](const AxleDef& ax) {
                Real c = 0;
                for (int wi : {ax.leftWheel, ax.rightWheel}) {
                    const WheelDef& wd = def_.wheels[static_cast<std::size_t>(wi)];
                    const WheelState& ws = wheels_[static_cast<std::size_t>(wi)];
                    c += aero.floorZ - wd.suspension.hardpoint.z + wd.suspension.travel - ws.compression + wd.tire.radius;
                }
                return 0.5 * c;
            };
            auto heightFactor = [&](Real h, bool& stalled) {
                if (aero.rideHeightRef <= 0) return 1.0;
                const Real hh = std::max(h, 0.005);
                Real f = std::pow(aero.rideHeightRef / hh, aero.rideHeightExponent);
                if (aero.floorStallHeight > 0 && hh < aero.floorStallHeight) {
                    // choke: the diffuser flow separates within ~15 mm below the
                    // stall height, taking the floor to 40% of its downforce
                    const Real t = clamp((aero.floorStallHeight - hh) / 0.015, 0.0, 1.0);
                    f *= 1 - 0.6 * t * t * (3 - 2 * t);
                    stalled = true;
                }
                return std::min(f, 2.5);
            };
            const AxleDef& fa = def_.axles.front();
            const AxleDef& ra = def_.axles.back();
            at.clearanceFront = clearance(fa);
            at.clearanceRear = clearance(ra);
            bool stF = false, stR = false;
            const Real dfF = qDyn * aero.frontDownforceArea * yawFactor * heightFactor(at.clearanceFront, stF);
            const Real dfR = qDyn * aero.rearDownforceArea * yawFactor * heightFactor(at.clearanceRear, stR);
            at.floorStalled = stF || stR;
            if (dfF != 0 || dfR != 0) {
                const Vec3 ff = -up * dfF, fr = -up * dfR;
                force += ff + fr;
                torque += cross(axlePoint(fa) - comWorld, ff) + cross(axlePoint(ra) - comWorld, fr);
            }
            at.downforceFront = dfF;
            at.downforceRear = dfR;
        }
        const auto& wing = aero.rearWing;
        if (wing.span > 0 && wing.chord > 0 && qDyn > 0) {
            // Angle of attack toward downforce: the set angle, less the flow
            // coming from below the chord line in body axes (that includes the
            // car's nose-up pitch: nose down = more wing).
            const Real flowDown = std::atan2(-vb.z, std::max(vb.x, 0.1));
            const Real ar = wing.span / wing.chord;
            const WingSectionResult sec = evaluateWingSection(wing.polar, wing.angle - flowDown, liftCurveSlope3D(ar, wing.polar.liftSlope));
            const Real S = wing.span * wing.chord;
            const Real cdi = sec.cl * sec.cl / (kPi * wing.efficiency * ar);
            const Real down = qDyn * S * sec.cl * yawFactor * (aeroDmg ? damage_.state().wingScale : 1.0);
            const Real dragW = qDyn * S * (sec.cd + cdi);
            const Vec3 pw = body_.position + q.rotate(wing.position);
            const Vec3 fw = -up * down + (vmag > 0.1 ? vAir * (-dragW / vmag) : Vec3{});
            force += fw;
            torque += cross(pw - comWorld, fw);
            at.wingDownforce = down;
            at.downforceRear += down;
            at.drag += dragW;
            at.wingStalled = sec.stalled;
        }
    }

    // ---- Body against the ground (rollover, a car on its side or roof, a landing) ----
    // Only when the car is tilted past ~30 degrees or has no wheel on the ground, so normal
    // driving costs nothing and queries nothing. Each hull point below the ground gets a stiff,
    // damped normal force and sliding friction (metal and glass on the road).
    bodyContacts_ = 0;
    bodyContactForce_ = 0;
    {
        bool anyWheel = false;
        for (std::size_t w = 0; w < nw; ++w) anyWheel = anyWheel || wheels_[w].contact;
        cageGroundGate_ = up.z < 0.86 || !anyWheel;
        // with the crash structure on, its nodes are the hull (solved after the integration)
        const bool cage = damage_.enabled() && damage_.config().structure && chassisMode_ == ChassisMode::Internal;
        if (cageGroundGate_ && !cage) {
            const Real kPt = def_.mass * 600.0;                            // N/m per point
            const Real cPt = 2.0 * 0.6 * std::sqrt(kPt * def_.mass / 6.0);  // ~0.6 of critical, ~6 points in contact
            const Mat3 R = q.toMatrix();
            const Mat3 invI = R * invInertiaBody_ * R.transposed();
            for (int hk = 0; hk < hullCount_; ++hk) {
                const Vec3& local = hull_[static_cast<std::size_t>(hk)];
                const Vec3 p = body_.position + q.rotate(local);
                GroundHit h = ground.raycast({p.x, p.y, p.z + 1.5}, {0, 0, -1}, 3.0);
                Real pen = h.hit ? dot(h.point - p, h.normal) : -1.0;
                // Steeply tilted (on its side, on its roof, on a wall, a banking or a loop): a vertical
                // probe runs along a steep surface without finding it, so also probe along the body's
                // own down axis and take the deeper of the two. On a plane both give the same depth.
                if (up.z < 0.5) {
                    const GroundHit h2 = ground.raycast(p + up * 1.5, -up, 3.0);
                    const Real pen2 = h2.hit ? dot(h2.point - p, h2.normal) : -1.0;
                    if (pen2 > pen) { h = h2; pen = pen2; }
                }
                if (pen <= 0) continue;
                const Vec3 r = p - comWorld;
                const Vec3 v = body_.velocity + cross(body_.angularVelocity, r);
                const Real vn = dot(v, h.normal);
                // Explicit contact: keep it stable for the effective mass this point sees (mass and
                // inertia about the CoM), whatever the definition. Never binds for a normal car.
                const Vec3 rn = cross(r, h.normal);
                const Real mEff = 1.0 / (1.0 / def_.mass + dot(rn, invI * rn));
                const Real k = std::min(kPt, mEff * (0.5 / dt) * (0.5 / dt));
                const Real c = std::min(cPt, mEff * 0.8 / dt);
                const Real fn = std::max(0.0, k * std::min(pen, 0.25) - c * vn);
                if (fn <= 0) continue;
                const Vec3 vt = v - h.normal * vn;
                const Real mu = def_.body.friction * h.surface.frictionScale;
                const Vec3 f = h.normal * fn - vt * (mu * fn / (length(vt) + 0.3));
                force += f;
                torque += cross(r, f);
                ++bodyContacts_;
                bodyContactForce_ += fn;
            }
        }
    }

    // Gravity acts on the total mass at the CoM. The chassis is integrated with the total mass
    // (so a host rigid body can simply use VehicleDefinition::mass); each unsprung wheel's own
    // acceleration along the suspension axis is accounted for by its inertial relief force above.
    if (chassisMode_ == ChassisMode::Internal || !hostGravity_) force += gravity * def_.mass;
    forces_ = {force, torque};

    if (fidelity_.telemetry) fillTelemetry(input, cmd, solveOut);
}

void Vehicle::integrateChassis(Real dt, const Vec3& extraForce) {
    const Quat q = body_.orientation;
    const Vec3 comWorld = centerOfMassWorld();
    body_.velocity += (forces_.force + extraForce) * (dt / def_.mass);
    const Mat3 R = q.toMatrix();
    const Mat3 invIWorld = R * invInertiaBody_ * R.transposed();
    const Mat3 iWorld = R * Mat3::diagonal(inertiaBody_) * R.transposed();
    const Vec3 w0 = body_.angularVelocity;
    body_.angularVelocity += invIWorld * (forces_.torque - cross(w0, iWorld * w0)) * dt;

    const Vec3 newCom = comWorld + body_.velocity * dt;
    const Vec3 wv = body_.angularVelocity;
    const Quat qn = Quat{0, wv.x, wv.y, wv.z} * q;
    body_.orientation = {q.w + 0.5 * dt * qn.w, q.x + 0.5 * dt * qn.x, q.y + 0.5 * dt * qn.y, q.z + 0.5 * dt * qn.z};
    body_.orientation.normalize();
    body_.position = newCom - body_.orientation.rotate(def_.centerOfMass);
    updateAccelEstimate(body_.velocity, body_.angularVelocity, dt);
    refreshBodyTelemetry(dt);
}

void Vehicle::refreshBodyTelemetry(Real dt) {
    if (fidelity_.telemetry) {
        telem_.position = body_.position;
        const Vec3 vb = bodyVelocity();
        telem_.speed = vb.x;
        telem_.lateralVelocity = vb.y;
        telem_.sideslip = std::abs(vb.x) > 0.5 ? std::atan2(vb.y, std::abs(vb.x)) : 0.0;
        telem_.yawRate = yawRate();
        telem_.accelBody = body_.orientation.inverseRotate(accel_);
        const Mat3 R1 = body_.orientation.toMatrix();
        const Mat3 i1 = R1 * Mat3::diagonal(inertiaBody_) * R1.transposed();
        telem_.kineticEnergy = 0.5 * def_.mass * dot(body_.velocity, body_.velocity) +
                               0.5 * dot(body_.angularVelocity, i1 * body_.angularVelocity);
        const Vec3 fwd = body_.orientation.rotate({1, 0, 0});
        const Vec3 left = body_.orientation.rotate({0, 1, 0});
        telem_.pitchAngle = -std::asin(clamp(fwd.z, -1.0, 1.0));
        telem_.rollAngle = std::asin(clamp(left.z, -1.0, 1.0));
        telem_.upright = body_.orientation.rotate({0, 0, 1}).z;
    } else {
        // Minimal channels are always kept current (LOD1 / AI use them).
        const Vec3 vb = bodyVelocity();
        telem_.position = body_.position;
        telem_.speed = vb.x;
        telem_.lateralVelocity = vb.y;
        telem_.yawRate = yawRate();
        telem_.upright = body_.orientation.rotate({0, 0, 1}).z;
    }
    telem_.time = time_ + dt;
}

void Vehicle::fillTelemetry(const DriverInput& input, const ActuatorCommands& cmd,
                            const std::array<WheelSolveOutput, kMaxWheels>& solveOut) {
    TelemetryFrame& t = telem_;
    const std::size_t nw = def_.wheels.size();
    t.time = time_;
    t.position = body_.position;
    const Vec3 vb = bodyVelocity();
    t.speed = vb.x;
    t.lateralVelocity = vb.y;
    t.sideslip = std::abs(vb.x) > 0.5 ? std::atan2(vb.y, std::abs(vb.x)) : 0.0;
    t.yawRate = yawRate();
    t.accelBody = body_.orientation.inverseRotate(accel_);
    t.steerInput = input.steer;
    t.throttleInput = input.throttle;
    t.brakeInput = input.brake;
    t.steerAngle = cmd.steerAngle;
    t.throttleOut = cmd.throttle;
    t.gear = cmd.gear;
    t.clutchEngagement = cmd.clutchEngagement;
    t.drivetrain = drivetrain_.telemetry();
    t.assists = assists_.report();
    t.wheelCount = nw;
    t.aero = aeroTel_;
    t.bodyContacts = bodyContacts_;
    t.bodyContactForce = bodyContactForce_;
    t.groundQueries = groundQueries_;
    for (std::size_t w = 0; w < nw; ++w) {
        const WheelState& ws = wheels_[w];
        WheelTelemetry& wt = t.wheels[w];
        wt.contact = ws.contact;
        wt.compression = ws.compression;
        wt.compressionRate = ws.compressionRate;
        wt.load = ws.load;
        wt.omega = ws.omega;
        wt.slipRatio = ws.slip.kappa;
        wt.slipAngle = std::atan(ws.slip.tanAlpha);
        wt.fx = ws.forces.fx;
        wt.fy = ws.forces.fy;
        {
            const Real cx = ws.forces.muX * ws.load, cy = ws.forces.muY * ws.load;
            const Real ux = cx > 1e-9 ? ws.forces.fx / cx : 0, uy = cy > 1e-9 ? ws.forces.fy / cy : 0;
            wt.gripUsed = ws.contact ? std::sqrt(ux * ux + uy * uy) : 0;
        }
        wt.camber = def_.wheels[w].isLeft ? ws.camber : -ws.camber;
        wt.temperature = ws.temperature;
        wt.steerAngle = ws.steer;
        wt.brakeTorque = cmd.brakeTorque[w];
        wt.brakeLocked = solveOut[w].brakeLocked;
        wt.surfaceId = ws.hit.surfaceId;
    }
    for (std::size_t a = 0; a < def_.axles.size(); ++a) {
        Real h = 0;
        for (int wi : {def_.axles[a].leftWheel, def_.axles[a].rightWheel}) {
            const auto w = static_cast<std::size_t>(wi);
            const WheelDef& wd = def_.wheels[w];
            LinkagePose p = linkagePose(w, wheels_[w].compression);
            if (!linkage_[w].valid()) p.dCamber = wd.camberGain;  // a slider with camber gain rotates the upright too
            h += 0.5 * rollCentreHeight(p, std::abs(wd.suspension.hardpoint.y + (wd.isLeft ? 1 : -1) * p.lat), wd.tire.radius);
        }
        t.rollCentreHeight[a] = h;
    }
}

bool Vehicle::setSteeringLock(Real maxAngle) {
    if (!(maxAngle > 0.05 && maxAngle <= 1.2) || maxSteerRef_ <= 0) return false;
    const Real k = maxAngle / maxSteerRef_;
    for (WheelDef& wd : def_.wheels) wd.maxSteerAngle *= k;  // keeps the Ackermann split
    maxSteerRef_ = maxAngle;
    return true;
}

bool Vehicle::setDifferential(std::size_t drivenAxle, const DifferentialDef& d) {
    if (drivenAxle >= def_.drivetrain.drivenAxles.size()) return false;
    if (!(d.preload >= 0 && d.powerRamp >= 0 && d.powerRamp <= 1 && d.coastRamp >= 0 && d.coastRamp <= 1)) return false;
    DifferentialDef v = d;
    v.finalDrive = def_.drivetrain.drivenAxles[drivenAxle].differential.finalDrive;  // gearing is not a diff setting
    def_.drivetrain.drivenAxles[drivenAxle].differential = v;
    drivetrain_.setDifferential(drivenAxle, v);
    return true;
}

}  // namespace ord
