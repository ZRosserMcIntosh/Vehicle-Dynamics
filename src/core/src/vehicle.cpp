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
    setFidelity(fidelity_);
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
    if (maxSteerRef_ <= 0 || wd.maxSteerAngle <= 0) return wd.isLeft ? -wd.staticToe : wd.staticToe;
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
    return delta + (wd.isLeft ? -wd.staticToe : wd.staticToe);
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
    if (!stateFinite()) {
        body_ = before;
        forces_ = {};
        faulted_ = true;
        return;
    }
    time_ += dt;
    lastDt_ = dt;
}

void Vehicle::stepSubstepped(Real hostDt, int substeps, const DriverInput& input, const GroundQuery& ground) {
    if (!valid_ || faulted_ || !(hostDt > 0) || !std::isfinite(hostDt)) return;
    substeps = std::max(substeps, 1);
    const Real h = hostDt / substeps;
    if (chassisMode_ == ChassisMode::Internal) {
        for (int k = 0; k < substeps; ++k) step(h, input, ground);
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
    const Real gU = dot(gravity, up);

    // ---- 1. Contact sampling + suspension (unsprung DOF or massless) ----
    std::array<Sample, kMaxContactSamples> samples{};
    std::array<Real, kMaxWheels> prevComp{};
    for (std::size_t w = 0; w < nw; ++w) prevComp[w] = wheels_[w].compression;

    for (std::size_t w = 0; w < nw; ++w) {
        WheelState& ws = wheels_[w];
        const WheelDef& wd = def_.wheels[w];
        const SuspensionDef& sd = wd.suspension;
        const Real r = wd.tire.radius;
        const Vec3 hp = body_.position + q.rotate(sd.hardpoint);
        ws.steer = wheelSteerAngle(w, cmd.steerAngle);
        const Vec3 headingWorld = q.rotate({std::cos(ws.steer), std::sin(ws.steer), 0});
        ws.hit = GroundHit{};
        int patchCount = 1;
        const int n = sampleContact(w, hp, up, headingWorld, ground, samples, patchCount);
        const Real weight = 1.0 / patchCount;

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
                    slope = std::max(0.0, dot(radial[k], up));
                } else {
                    pen0[k] = (sd.travel - x0) + r - samples[k].d;  // flat footprint
                    // Patch samples push along their ground normal (tilted on curb and
                    // pothole edges, which produces the longitudinal impact force).
                    radial[k] = samples[k].normal;
                }
                if (pen0[k] <= 0) continue;
                ru[k] = std::max(0.0, dot(radial[k], up));
                P0 += weight * pen0[k] * ru[k];
                Pprev += weight * ws.prevPen[k] * ru[k];
                Wact += weight * ru[k] * slope;
            }
            // Suspension spring/damper/ARB/bump stops, linearised at x0.
            const Real c = xd0 > 0 ? sd.bumpDamping : sd.reboundDamping;
            Real As = sd.springPreload + sd.springRate * x0 + karb * (x0 - xo);
            Real Ks = sd.springRate + karb;
            if (x0 > sd.travel) { As += sd.bumpStopRate * (x0 - sd.travel); Ks += sd.bumpStopRate; }
            if (x0 < 0) { As += sd.bumpStopRate * x0; Ks += sd.bumpStopRate; }
            const Real Bs = Ks * dt + c;
            Real At = kt * P0 + ct * (P0 - Pprev) / dt;
            Real Bt = (kt * dt + ct) * Wact;
            const Real rhs0 = m * xd0 / dt - As + m * gU - m * dot(aHp, up);
            Real xd1 = (rhs0 + At) / (m / dt + Bt + Bs);
            if (At - Bt * xd1 < 0) {  // tire would pull: it leaves the ground this step
                At = Bt = 0;
                xd1 = rhs0 / (m / dt + Bs);
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
                if (pen > 0 && Bt > 0) fk = std::max(0.0, weight * (kt * pen + ct * (pen - ws.prevPen[k]) / dt));
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
        } else {
            // Massless wheel: compression follows the highest sample (rigid tire).
            Real best = -1e9;
            for (int k = 0; k < patchCount; ++k)
                if (samples[k].hit) best = std::max(best, sd.travel + r - samples[k].d);
            ws.contact = best >= 0;
            const Real x = ws.contact ? best : 0.0;
            ws.compression = x;
            Real f = sd.springPreload + sd.springRate * std::min(x, sd.travel);
            if (x > sd.travel) f += sd.bumpStopRate * (x - sd.travel);
            const Real xdot = (x - ws.prevCompression) / dt;
            ws.compressionRate = xdot;
            f += (xdot > 0 ? sd.bumpDamping : sd.reboundDamping) * xdot;
            f += karb * (x - xo);
            ws.springForce = f;
            ws.load = ws.contact ? std::max(0.0, f) : 0.0;
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
            const TireForces f0 = tire.evaluate(ws.load, ws.slip.kappa, ws.slip.tanAlpha, ws.hit.surface, speed);
            si.contactVx = ws.contactVx;
            si.fx0 = f0.fx;
            si.dFxdKappa = std::max(0.0, f0.dFxdKappa);
            const Real fade = std::max(0.0, 1.0 - speed / wd.tire.lowSpeedDampingFadeSpeed);
            si.slipDamping = wd.tire.lowSpeedDampingPerLoad * ws.load * fade;
            si.externalTorque = -wd.tire.rollingResistance * ws.hit.surface.rollingResistanceScale * ws.load * r *
                                std::tanh(omega * r / 0.5);
        } else {
            ws.slip.tanAlpha = 0;
            si.contactVx = omega * r;  // airborne: slip relaxes toward zero
        }
    }

    // ---- 3. Implicit drivetrain + wheel spin + longitudinal slip ----
    DrivetrainCommand dc;
    dc.throttle = cmd.throttle;
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
        const Vec3 hp = body_.position + q.rotate(wd.suspension.hardpoint);
        const bool unsprung = usesUnsprung(w);
        if (unsprung) {
            // Exact for a chassis integrated with the *total* mass M (so a host
            // rigid body can simply use M): along the suspension axis
            //   M a_body = F_tire + M g_u - m_u * xdd
            // i.e. the body receives the tire normal force (at the patch) minus
            // the wheel's inertia relative to the body (at the hardpoint).
            const Vec3 fi = up * (-wd.unsprungMass * ws.relativeAccel);
            force += ws.tireNormalForce + fi;
            torque += cross(ws.patchPoint - comWorld, ws.tireNormalForce) + cross(hp - comWorld, fi);
        }
        if (ws.load <= 0) {
            ws.forces = {};
            continue;
        }
        const Real speed = std::hypot(ws.contactVx, ws.contactVy);
        ws.forces = tires_[w].evaluate(ws.load, ws.slip.kappa, ws.slip.tanAlpha, ws.hit.surface, speed);
        ws.forces.fx = solveOut[w].fx;  // consistent with the implicit wheel solve
        ws.forces.fy -= solveIn[w].slipDamping * ws.contactVy;
        const Vec3 n = ws.normal;
        Vec3 fc = ws.longDir * ws.forces.fx + ws.latDir * ws.forces.fy;
        if (!unsprung) fc += n * ws.load;
        force += fc;
        torque += cross(ws.patchPoint - comWorld, fc) + n * ws.forces.mz;
        debug_[w].fz = ws.load;
        debug_[w].fx = ws.forces.fx;
        debug_[w].fy = ws.forces.fy;
    }

    // Aerodynamics (still air): drag at the CoM, downforce at each axle.
    const Real vmag = length(body_.velocity);
    const AeroDef& aero = def_.aero;
    force += body_.velocity * (-0.5 * aero.airDensity * aero.dragArea * vmag);
    const Real vxBody = bodyVelocity().x;
    const Real qDyn = 0.5 * aero.airDensity * vxBody * vxBody;
    if (def_.axles.size() >= 2 && (aero.frontDownforceArea != 0 || aero.rearDownforceArea != 0)) {
        const auto axlePoint = [&](const AxleDef& ax) {
            const Vec3 hp = (def_.wheels[ax.leftWheel].suspension.hardpoint + def_.wheels[ax.rightWheel].suspension.hardpoint) * 0.5;
            return body_.position + q.rotate(hp);
        };
        const Vec3 ff = -up * (qDyn * aero.frontDownforceArea);
        const Vec3 fr = -up * (qDyn * aero.rearDownforceArea);
        force += ff + fr;
        torque += cross(axlePoint(def_.axles.front()) - comWorld, ff) + cross(axlePoint(def_.axles.back()) - comWorld, fr);
    }

    // With unsprung DOFs the body-vertical forces accelerate only the sprung
    // mass. The chassis is integrated with the total mass (so the host rigid
    // body can simply have mass = VehicleDefinition::mass), so scale the
    // body-Z component of the non-gravitational force by M / M_sprung.
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
    } else {
        // Minimal channels are always kept current (LOD1 / AI use them).
        const Vec3 vb = bodyVelocity();
        telem_.position = body_.position;
        telem_.speed = vb.x;
        telem_.lateralVelocity = vb.y;
        telem_.yawRate = yawRate();
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
        wt.steerAngle = ws.steer;
        wt.brakeTorque = cmd.brakeTorque[w];
        wt.brakeLocked = solveOut[w].brakeLocked;
        wt.surfaceId = ws.hit.surfaceId;
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
