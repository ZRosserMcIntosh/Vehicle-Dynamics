// OpenRoad Dynamics — engine-independent vehicle physics core.
// SPDX-License-Identifier: MIT
//
// Nonlinear rigid multibody single-track model. See ord/single_track.hpp for
// the model description and references.
#include "ord/single_track.hpp"

#include "ord/linear_solve.hpp"

namespace ord {

namespace {

constexpr Vec3 kEx{1, 0, 0};
constexpr Vec3 kEy{0, 1, 0};
constexpr Vec3 kEz{0, 0, 1};

Real comp(const Vec3& v, int k) {
    return k == 0 ? v.x : (k == 1 ? v.y : v.z);
}
Vec3 axisVec(int k) {
    return k == 0 ? kEx : (k == 1 ? kEy : kEz);
}

Mat3 inverse(const Mat3& a) {
    const auto& m = a.m;
    const Real c00 = m[1][1] * m[2][2] - m[1][2] * m[2][1];
    const Real c01 = m[1][2] * m[2][0] - m[1][0] * m[2][2];
    const Real c02 = m[1][0] * m[2][1] - m[1][1] * m[2][0];
    const Real det = m[0][0] * c00 + m[0][1] * c01 + m[0][2] * c02;
    Mat3 r;
    if (std::abs(det) < 1e-300) return Mat3::diagonal({0, 0, 0});
    const Real id = 1.0 / det;
    r.m[0][0] = c00 * id;
    r.m[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * id;
    r.m[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * id;
    r.m[1][0] = c01 * id;
    r.m[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * id;
    r.m[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * id;
    r.m[2][0] = c02 * id;
    r.m[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * id;
    r.m[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * id;
    return r;
}

Mat3 wheelInertia(const SingleTrackWheelDef& w) {
    return Mat3::diagonal({w.inertiaDiametral, w.inertiaAxial, w.inertiaDiametral});
}

// Quaternion time derivative for a world-frame angular velocity.
Quat quatRate(const Quat& q, const Vec3& w) {
    const Quat wq{0, w.x, w.y, w.z};
    Quat d = wq * q;
    d.w *= 0.5;
    d.x *= 0.5;
    d.y *= 0.5;
    d.z *= 0.5;
    return d;
}

// Thin-disc contact geometry: unit vector from the wheel centre toward the
// lowest point of the rim (in the wheel plane). Returns |g|, the sine of the
// angle between the axle and the vertical; ~0 means the wheel lies flat.
Real contactDirection(const Quat& q, Vec3& d, Vec3& axle) {
    axle = q.rotate(kEy);
    const Vec3 g = Vec3{0, 0, -1} + axle * axle.z;
    const Real gn = length(g);
    d = gn > 1e-9 ? g / gn : Vec3{0, 0, -1};
    return gn;
}

TireParams bicycleTyre(Real radius, Real nominalLoad) {
    TireParams t;
    t.radius = radius;
    t.nominalLoad = nominalLoad;
    t.muX = 1.0;
    t.muXLoadSensitivity = -0.05;
    t.muY = 1.0;
    t.muYLoadSensitivity = -0.05;
    t.longStiffnessPerLoad = 15.0;
    t.cornerStiffness = 14.0;
    t.cornerStiffnessLoad = 2.0;
    t.relaxationLengthX = 0.05;
    t.relaxationLengthY = 0.10;
    t.lowSpeedDampingPerLoad = 1.0;
    t.lowSpeedDampingFadeSpeed = 1.5;
    t.rollingResistance = 0.005;
    // Pneumatic trail is 0 on purpose until camber thrust and camber twisting
    // torque exist (roadmap M3.1). On its own, an aligning torque of
    // trail * Fy (no counteracting camber terms) resists the steer-into-the-
    // lean motion the bicycle balances with: with 10 mm trail the benchmark
    // bicycle in Tyre mode capsizes at 5 m/s, where the NoSlip model and real
    // bicycles are self-stable.
    t.pneumaticTrail = 0.0;
    t.trailCollapseAngle = 0.2;
    t.verticalStiffness = 1.0e5;
    t.verticalDamping = 400.0;
    return t;
}

}  // namespace

SingleTrackDef meijaardBenchmark() {
    // Meijaard et al. 2007, Table 1. Paper axes: x forward, y right, z down,
    // origin at the rear contact point. ord axes: (x, -y, -z). Under that
    // rotation positions map (x, z) -> (x, -z) and the products of inertia
    // Ixz flip sign (Ixy, Iyz are zero for the benchmark).
    SingleTrackDef d;
    d.wheelbase = 1.02;
    d.trail = 0.08;
    d.steerAxisTilt = kPi / 10;
    d.gravity = 9.81;

    d.rearWheel = {0.3, 2.0, 0.0603, 0.12};
    d.frontWheel = {0.35, 3.0, 0.1405, 0.28};

    d.rearFrame.mass = 85.0;
    d.rearFrame.centreOfMass = {0.3, 0.0, 0.9};
    d.rearFrame.inertia = Mat3::diagonal({9.2, 11.0, 2.8});
    d.rearFrame.inertia.m[0][2] = d.rearFrame.inertia.m[2][0] = -2.4;  // paper +2.4 (z down)

    d.frontFrame.mass = 4.0;
    d.frontFrame.centreOfMass = {0.9, 0.0, 0.7};
    d.frontFrame.inertia = Mat3::diagonal({0.05892, 0.06, 0.00708});
    d.frontFrame.inertia.m[0][2] = d.frontFrame.inertia.m[2][0] = 0.00756;  // paper -0.00756

    // Static loads are ~610 N rear / ~310 N front.
    d.rearTyre = bicycleTyre(d.rearWheel.radius, 600.0);
    d.frontTyre = bicycleTyre(d.frontWheel.radius, 300.0);
    return d;
}

SingleTrack::SingleTrack(const SingleTrackDef& def, SingleTrackContact contact)
    : def_(def), contact_(contact), rearTyre_(def.rearTyre), frontTyre_(def.frontTyre) {
    const Real w = def_.wheelbase, c = def_.trail, lam = def_.steerAxisTilt;
    const Real rR = def_.rearWheel.radius, rF = def_.frontWheel.radius;

    body_[kRearWheel].mass = def_.rearWheel.mass;
    body_[kRearWheel].inertiaBody = wheelInertia(def_.rearWheel);
    body_[kRearFrame].mass = def_.rearFrame.mass;
    body_[kRearFrame].inertiaBody = def_.rearFrame.inertia;
    body_[kFrontFrame].mass = def_.frontFrame.mass;
    body_[kFrontFrame].inertiaBody = def_.frontFrame.inertia;
    body_[kFrontWheel].mass = def_.frontWheel.mass;
    body_[kFrontWheel].inertiaBody = wheelInertia(def_.frontWheel);
    for (auto& b : body_) b.invInertiaBody = inverse(b.inertiaBody);

    refPos_[kRearWheel] = {0, 0, rR};
    refPos_[kRearFrame] = def_.rearFrame.centreOfMass;
    refPos_[kFrontFrame] = def_.frontFrame.centreOfMass;
    refPos_[kFrontWheel] = {w, 0, rF};

    // Steer axis: meets the ground a trail c ahead of the front contact point
    // and tilts back by lambda. Anchor = foot of the perpendicular from the
    // front wheel centre (any point on the axis would do).
    steerAxisLocal_ = {-std::sin(lam), 0, std::cos(lam)};
    const Vec3 ground{w + c, 0, 0};
    steerAnchorRef_ = ground + steerAxisLocal_ * dot(refPos_[kFrontWheel] - ground, steerAxisLocal_);

    joints_[0] = {kRearWheel, kRearFrame, {0, 0, 0}, refPos_[kRearWheel] - refPos_[kRearFrame], kEy, kEx, kEz};
    joints_[1] = {kFrontFrame,
                  kRearFrame,
                  steerAnchorRef_ - refPos_[kFrontFrame],
                  steerAnchorRef_ - refPos_[kRearFrame],
                  steerAxisLocal_,
                  Vec3{std::cos(lam), 0, std::sin(lam)},
                  kEy};
    joints_[2] = {kFrontWheel, kFrontFrame, {0, 0, 0}, refPos_[kFrontWheel] - refPos_[kFrontFrame], kEy, kEx, kEz};

    reset(0.0);
}

Real SingleTrack::wheelRadius(int bodyIndex) const {
    return bodyIndex == kRearWheel ? def_.rearWheel.radius : def_.frontWheel.radius;
}

Vec3 SingleTrack::invInertiaWorld(const Kin& k, int i, const Vec3& v) const {
    return k.q[i].rotate(body_[i].invInertiaBody * k.q[i].inverseRotate(v));
}
Vec3 SingleTrack::inertiaWorld(const Kin& k, int i, const Vec3& v) const {
    return k.q[i].rotate(body_[i].inertiaBody * k.q[i].inverseRotate(v));
}

// ---------------------------------------------------------------- constraints

bool SingleTrack::buildRows(const Kin& k, Rows& rows, bool withContact) const {
    rows.n = 0;
    for (const Joint& j : joints_) {
        const int a = j.a, b = j.b;
        const Vec3 ra = k.q[a].rotate(j.anchorA), rb = k.q[b].rotate(j.anchorB);
        const Vec3 gap = (k.x[a] + ra) - (k.x[b] + rb);
        const Vec3 gam = cross(k.w[a], cross(k.w[a], ra)) - cross(k.w[b], cross(k.w[b], rb));
        for (int e = 0; e < 3; ++e) {
            Row& r = rows.r[rows.n++];
            const Vec3 ek = axisVec(e);
            r.a = a;
            r.b = b;
            r.la = ek;
            r.aa = cross(ra, ek);
            r.lb = -ek;
            r.ab = -cross(rb, ek);
            r.gamma = comp(gam, e);
            r.c = comp(gap, e);
            r.holonomic = true;
        }
        const Vec3 u = k.q[a].rotate(j.axisA);
        const Vec3 dw = k.w[a] - k.w[b];
        const Vec3 udot = cross(k.w[a], u);
        for (const Vec3& perpLocal : {j.perpB1, j.perpB2}) {
            const Vec3 p = k.q[b].rotate(perpLocal);
            const Vec3 n = cross(u, p);
            Row& r = rows.r[rows.n++];
            r.a = a;
            r.b = b;
            r.la = {};
            r.aa = n;
            r.lb = {};
            r.ab = -n;
            r.gamma = dot(dw, cross(udot, p) + cross(u, cross(k.w[b], p)));
            r.c = dot(u, p);
            r.holonomic = true;
        }
    }
    if (!withContact) return true;
    for (int i : {int(kRearWheel), int(kFrontWheel)}) {
        Vec3 d, axle;
        const Real gn = contactDirection(k.q[i], d, axle);
        if (gn < 1e-6) return false;  // wheel lying flat: no frame-ground contact model
        const Real rad = wheelRadius(i);
        const Vec3 rd = d * rad;
        const Vec3 adot = cross(k.w[i], axle);
        const Vec3 gdot = axle * adot.z + adot * axle.z;
        const Vec3 ddot = (gdot - d * dot(d, gdot)) / gn;
        const Vec3 gam = cross(k.w[i], ddot * rad);
        for (int e = 0; e < 3; ++e) {
            Row& r = rows.r[rows.n++];
            const Vec3 ek = axisVec(e);
            r.a = i;
            r.b = -1;
            r.la = ek;
            r.aa = cross(rd, ek);
            r.lb = {};
            r.ab = {};
            r.gamma = comp(gam, e);
            // Only "contact height = 0" is holonomic; the horizontal rows are
            // nonholonomic rolling constraints.
            r.holonomic = (e == 2);
            r.c = e == 2 ? k.x[i].z + rd.z : 0.0;
        }
    }
    return true;
}

namespace {

// Solves (J M^-1 J^T) lambda = rhs for the selected rows and returns
// M^-1 J^T lambda per body (linear, angular).
template <class Rows, class InvI>
bool constraintImpulse(const Rows& rows, const int* idx, int n, const Real* rhs, const Real* invMass, const InvI& invI,
                       Vec3* dv, Vec3* dw) {
    constexpr int kMax = 24;
    DenseSystem<kMax> sys;
    sys.reset(std::size_t(n));
    // M^-1 J^T columns per row and slot.
    Vec3 mlA[kMax], maA[kMax], mlB[kMax], maB[kMax];
    for (int i = 0; i < n; ++i) {
        const auto& r = rows.r[idx[i]];
        mlA[i] = r.la * invMass[r.a];
        maA[i] = invI(r.a, r.aa);
        if (r.b >= 0) {
            mlB[i] = r.lb * invMass[r.b];
            maB[i] = invI(r.b, r.ab);
        }
    }
    for (int i = 0; i < n; ++i) {
        const auto& ri = rows.r[idx[i]];
        for (int j = 0; j < n; ++j) {
            const auto& rj = rows.r[idx[j]];
            Real s = 0;
            if (ri.a == rj.a) s += dot(ri.la, mlA[j]) + dot(ri.aa, maA[j]);
            if (rj.b >= 0 && ri.a == rj.b) s += dot(ri.la, mlB[j]) + dot(ri.aa, maB[j]);
            if (ri.b >= 0) {
                if (ri.b == rj.a) s += dot(ri.lb, mlA[j]) + dot(ri.ab, maA[j]);
                if (rj.b >= 0 && ri.b == rj.b) s += dot(ri.lb, mlB[j]) + dot(ri.ab, maB[j]);
            }
            sys.a(std::size_t(i), std::size_t(j)) = s;
        }
        sys.b(std::size_t(i)) = rhs[i];
    }
    if (!sys.solve()) return false;
    for (int b = 0; b < SingleTrack::kBodies; ++b) dv[b] = dw[b] = Vec3{};
    for (int i = 0; i < n; ++i) {
        const auto& r = rows.r[idx[i]];
        const Real l = sys.x(std::size_t(i));
        dv[r.a] += mlA[i] * l;
        dw[r.a] += maA[i] * l;
        if (r.b >= 0) {
            dv[r.b] += mlB[i] * l;
            dw[r.b] += maB[i] * l;
        }
    }
    return true;
}

}  // namespace

// ------------------------------------------------------------------- forces

void SingleTrack::externalForces(const Kin& k, const SingleTrackInput& in, Vec3 f[kBodies], Vec3 t[kBodies],
                                 Real* loadsOut) const {
    for (int i = 0; i < kBodies; ++i) {
        f[i] = {0, 0, -def_.gravity * body_[i].mass};
        t[i] = -cross(k.w[i], inertiaWorld(k, i, k.w[i]));  // gyroscopic
    }
    // Steer torque: action on the front frame, reaction on the rear frame.
    // Positive (benchmark sign) turns the front frame about the downward axis.
    const Vec3 u = k.q[kRearFrame].rotate(steerAxisLocal_);
    t[kFrontFrame] -= u * in.steerTorque;
    t[kRearFrame] += u * in.steerTorque;
    // Drive torque about the rear axle (ord +y = left; forward rolling is +).
    const Vec3 axleR = k.q[kRearWheel].rotate(kEy);
    t[kRearWheel] += axleR * in.driveTorque;
    t[kRearFrame] -= axleR * in.driveTorque;
    // External roll couple about the horizontal heading (+ = lean right).
    const Vec3 bx = k.q[kRearFrame].rotate(kEx);
    t[kRearFrame] += normalized(Vec3{bx.x, bx.y, 0}) * in.rollTorque;

    if (contact_ != SingleTrackContact::Tyre) return;
    for (int wi = 0; wi < 2; ++wi) {
        const int i = wi == 0 ? int(kRearWheel) : int(kFrontWheel);
        const TireParams& tp = wi == 0 ? def_.rearTyre : def_.frontTyre;
        const MagicFormulaTire& tyre = wi == 0 ? rearTyre_ : frontTyre_;
        if (loadsOut) loadsOut[wi] = 0;
        Vec3 d, axle;
        if (contactDirection(k.q[i], d, axle) < 1e-6) continue;
        const Vec3 rd = d * wheelRadius(i);
        const Real pen = -(k.x[i].z + rd.z);
        if (pen <= 0) continue;
        const Vec3 vP = k.v[i] + cross(k.w[i], rd);  // material contact-point velocity
        const Vec3 tdir = normalized(cross(axle, kEz));
        const Vec3 ldir = cross(kEz, tdir);
        const Real vx = dot(k.v[i], tdir);
        const Real fz = std::max(0.0, tp.verticalStiffness * pen - tp.verticalDamping * vP.z);
        if (loadsOut) loadsOut[wi] = fz;
        const TireForces tf = tyre.evaluate(fz, slip_[wi].kappa, slip_[wi].tanAlpha, def_.surface, std::abs(vx));
        Real fx = tf.fx, fy = tf.fy;
        // Low-speed carcass damping (the relaxation ODE alone is undamped at standstill).
        const Real fade = std::max(0.0, 1.0 - std::abs(vx) / tp.lowSpeedDampingFadeSpeed);
        const Real cd = tp.lowSpeedDampingPerLoad * fz * fade;
        fx -= cd * dot(vP, tdir);
        fy -= cd * dot(vP, ldir);
        const Vec3 F = tdir * fx + ldir * fy + kEz * fz;
        f[i] += F;
        t[i] += cross(rd, F) + kEz * tf.mz;
        // Rolling resistance is a moment (normal force shifted ahead of the
        // contact), not a contact force: a free wheel would cancel the latter
        // by spinning up. The tyre's longitudinal slip then supplies the drag.
        const Real rr = tp.rollingResistance * def_.surface.rollingResistanceScale;
        t[i] -= axle * (rr * fz * wheelRadius(i) * clamp(vx / 0.5, -1.0, 1.0));
    }
}

void SingleTrack::updateTyreSlip(Real dt) {
    // Relaxation-length transient slip (Pacejka 2012 ch. 7), implicit in time:
    //   sigma * dkappa/dt + |vx| kappa = -(contact slip velocity, longitudinal)
    //   sigma * dtanA/dt  + |vx| tanA  = -(contact slip velocity, lateral)
    // Updated once per step from the start-of-step velocities (operator split).
    for (int wi = 0; wi < 2; ++wi) {
        const int i = wi == 0 ? int(kRearWheel) : int(kFrontWheel);
        const TireParams& tp = wi == 0 ? def_.rearTyre : def_.frontTyre;
        const MagicFormulaTire& tyre = wi == 0 ? rearTyre_ : frontTyre_;
        TireSlipState& s = slip_[wi];
        Vec3 d, axle;
        const bool flat = contactDirection(k_.q[i], d, axle) < 1e-6;
        const Vec3 rd = d * wheelRadius(i);
        if (flat || -(k_.x[i].z + rd.z) <= 0) {
            s.kappa = s.tanAlpha = 0;  // airborne: carcass relaxes
            continue;
        }
        const Vec3 vP = k_.v[i] + cross(k_.w[i], rd);
        const Vec3 tdir = normalized(cross(axle, kEz));
        const Vec3 ldir = cross(kEz, tdir);
        const Real vx = dot(k_.v[i], tdir);
        const Real sx = tp.relaxationLengthX;
        s.kappa = (s.kappa - dt / sx * dot(vP, tdir)) / (1.0 + dt * std::abs(vx) / sx);
        s.kappa = clamp(s.kappa, -MagicFormulaTire::kSlipStateLimit, MagicFormulaTire::kSlipStateLimit);
        tyre.integrateLateralSlip(s, vx, dot(vP, ldir), dt);
    }
}

// ----------------------------------------------------------------- dynamics

bool SingleTrack::accelerations(const Kin& k, const SingleTrackInput& in, Vec3 a[kBodies], Vec3 al[kBodies]) const {
    Vec3 f[kBodies], t[kBodies];
    externalForces(k, in, f, t, nullptr);
    Real invMass[kBodies];
    for (int i = 0; i < kBodies; ++i) {
        invMass[i] = 1.0 / body_[i].mass;
        a[i] = f[i] * invMass[i];
        al[i] = invInertiaWorld(k, i, t[i]);
    }
    Rows rows;
    if (!buildRows(k, rows, contact_ == SingleTrackContact::NoSlip)) return false;
    int idx[kMaxRows];
    Real rhs[kMaxRows];
    for (int r = 0; r < rows.n; ++r) {
        const Row& row = rows.r[r];
        idx[r] = r;
        Real ja = dot(row.la, a[row.a]) + dot(row.aa, al[row.a]);
        if (row.b >= 0) ja += dot(row.lb, a[row.b]) + dot(row.ab, al[row.b]);
        rhs[r] = -row.gamma - ja;
    }
    Vec3 dv[kBodies], dw[kBodies];
    auto invI = [&](int b, const Vec3& v) { return invInertiaWorld(k, b, v); };
    if (!constraintImpulse(rows, idx, rows.n, rhs, invMass, invI, dv, dw)) return false;
    for (int i = 0; i < kBodies; ++i) {
        a[i] += dv[i];
        al[i] += dw[i];
    }
    return true;
}

bool SingleTrack::projectPositions(Kin& k) const {
    Real invMass[kBodies];
    for (int i = 0; i < kBodies; ++i) invMass[i] = 1.0 / body_[i].mass;
    for (int iter = 0; iter < 4; ++iter) {
        Rows rows;
        if (!buildRows(k, rows, contact_ == SingleTrackContact::NoSlip)) return false;
        int idx[kMaxRows];
        Real rhs[kMaxRows];
        int n = 0;
        Real worst = 0;
        for (int r = 0; r < rows.n; ++r) {
            if (!rows.r[r].holonomic) continue;
            idx[n] = r;
            rhs[n++] = -rows.r[r].c;
            worst = std::max(worst, std::abs(rows.r[r].c));
        }
        if (worst < 1e-14) return true;
        Vec3 dx[kBodies], dth[kBodies];
        auto invI = [&](int b, const Vec3& v) { return invInertiaWorld(k, b, v); };
        if (!constraintImpulse(rows, idx, n, rhs, invMass, invI, dx, dth)) return false;
        for (int i = 0; i < kBodies; ++i) {
            k.x[i] += dx[i];
            const Real ang = length(dth[i]);
            if (ang > 0) {
                k.q[i] = Quat::fromAxisAngle(dth[i], ang) * k.q[i];
                k.q[i].normalize();
            }
        }
    }
    return true;
}

bool SingleTrack::projectVelocities(Kin& k) const {
    Real invMass[kBodies];
    for (int i = 0; i < kBodies; ++i) invMass[i] = 1.0 / body_[i].mass;
    Rows rows;
    if (!buildRows(k, rows, contact_ == SingleTrackContact::NoSlip)) return false;
    int idx[kMaxRows];
    Real rhs[kMaxRows];
    for (int r = 0; r < rows.n; ++r) {
        const Row& row = rows.r[r];
        idx[r] = r;
        Real jv = dot(row.la, k.v[row.a]) + dot(row.aa, k.w[row.a]);
        if (row.b >= 0) jv += dot(row.lb, k.v[row.b]) + dot(row.ab, k.w[row.b]);
        rhs[r] = -jv;
    }
    Vec3 dv[kBodies], dw[kBodies];
    auto invI = [&](int b, const Vec3& v) { return invInertiaWorld(k, b, v); };
    if (!constraintImpulse(rows, idx, rows.n, rhs, invMass, invI, dv, dw)) return false;
    for (int i = 0; i < kBodies; ++i) {
        k.v[i] += dv[i];
        k.w[i] += dw[i];
    }
    return true;
}

void SingleTrack::step(Real dt, const SingleTrackInput& in) {
    if (faulted_ || !(dt > 0)) return;
    if (contact_ == SingleTrackContact::Tyre) updateTyreSlip(dt);

    struct Deriv {
        Vec3 dx[kBodies];
        Quat dq[kBodies];
        Vec3 dv[kBodies];
        Vec3 dw[kBodies];
    };
    auto evaluate = [&](const Kin& s, Deriv& d) {
        for (int i = 0; i < kBodies; ++i) {
            d.dx[i] = s.v[i];
            d.dq[i] = quatRate(s.q[i], s.w[i]);
        }
        return accelerations(s, in, d.dv, d.dw);
    };
    auto advance = [&](const Kin& s, const Deriv& d, Real h, Kin& out) {
        for (int i = 0; i < kBodies; ++i) {
            out.x[i] = s.x[i] + d.dx[i] * h;
            out.q[i] = {s.q[i].w + d.dq[i].w * h, s.q[i].x + d.dq[i].x * h, s.q[i].y + d.dq[i].y * h,
                        s.q[i].z + d.dq[i].z * h};
            out.q[i].normalize();
            out.v[i] = s.v[i] + d.dv[i] * h;
            out.w[i] = s.w[i] + d.dw[i] * h;
        }
    };

    // Classical RK4.
    Deriv k1, k2, k3, k4;
    Kin s;
    bool ok = evaluate(k_, k1);
    advance(k_, k1, 0.5 * dt, s);
    ok = ok && evaluate(s, k2);
    advance(k_, k2, 0.5 * dt, s);
    ok = ok && evaluate(s, k3);
    advance(k_, k3, dt, s);
    ok = ok && evaluate(s, k4);
    if (!ok) {
        faulted_ = true;
        return;
    }
    Kin next;
    const Real h6 = dt / 6.0;
    for (int i = 0; i < kBodies; ++i) {
        next.x[i] = k_.x[i] + (k1.dx[i] + 2.0 * k2.dx[i] + 2.0 * k3.dx[i] + k4.dx[i]) * h6;
        next.v[i] = k_.v[i] + (k1.dv[i] + 2.0 * k2.dv[i] + 2.0 * k3.dv[i] + k4.dv[i]) * h6;
        next.w[i] = k_.w[i] + (k1.dw[i] + 2.0 * k2.dw[i] + 2.0 * k3.dw[i] + k4.dw[i]) * h6;
        const Quat& q = k_.q[i];
        next.q[i] = {q.w + (k1.dq[i].w + 2 * k2.dq[i].w + 2 * k3.dq[i].w + k4.dq[i].w) * h6,
                     q.x + (k1.dq[i].x + 2 * k2.dq[i].x + 2 * k3.dq[i].x + k4.dq[i].x) * h6,
                     q.y + (k1.dq[i].y + 2 * k2.dq[i].y + 2 * k3.dq[i].y + k4.dq[i].y) * h6,
                     q.z + (k1.dq[i].z + 2 * k2.dq[i].z + 2 * k3.dq[i].z + k4.dq[i].z) * h6};
        next.q[i].normalize();
    }
    // Drift removal: positions onto the holonomic manifold, then velocities
    // onto all velocity constraints (mass-weighted, i.e. minimum kinetic-energy change).
    if (!projectPositions(next) || !projectVelocities(next)) {
        faulted_ = true;
        return;
    }
    for (int i = 0; i < kBodies; ++i) {
        if (!isFinite(next.x[i]) || !isFinite(next.v[i]) || !isFinite(next.w[i]) || !std::isfinite(next.q[i].w)) {
            faulted_ = true;
            return;
        }
    }
    k_ = next;
    if (contact_ == SingleTrackContact::Tyre) {
        Vec3 f[kBodies], t[kBodies];
        externalForces(k_, in, f, t, load_);
    }
    time_ += dt;
    for (int i = 0; i < kBodies; ++i) {
        body_[i].position = k_.x[i];
        body_[i].orientation = k_.q[i];
        body_[i].velocity = k_.v[i];
        body_[i].angularVelocity = k_.w[i];
    }
}

// --------------------------------------------------------- state set-up

void SingleTrack::placeBodies(Real lean, Real steer, Real pitch, Real rearSink, Kin& k) const {
    // Rear frame orientation: yaw (0) - lean about x - pitch about the new y,
    // the benchmark's rear-frame angle sequence.
    const Quat qB = Quat::fromAxisAngle(kEx, lean) * Quat::fromAxisAngle(kEy, pitch);
    // Front frame: rotation about the steer axis. Benchmark positive steer is
    // about the downward axis, i.e. negative about ord's upward axis.
    const Quat qH = qB * Quat::fromAxisAngle(steerAxisLocal_, -steer);
    k.q[kRearWheel] = qB;
    k.q[kRearFrame] = qB;
    k.q[kFrontFrame] = qH;
    k.q[kFrontWheel] = qH;
    Vec3 d, axle;
    contactDirection(qB, d, axle);
    // Rear rim point at the origin (minus the tyre sink in Tyre mode).
    const Vec3 xR = d * (-def_.rearWheel.radius) - Vec3{0, 0, rearSink};
    const Vec3 anchor = xR + qB.rotate(steerAnchorRef_ - refPos_[kRearWheel]);
    k.x[kRearWheel] = xR;
    k.x[kRearFrame] = xR + qB.rotate(refPos_[kRearFrame] - refPos_[kRearWheel]);
    k.x[kFrontFrame] = anchor + qH.rotate(refPos_[kFrontFrame] - steerAnchorRef_);
    k.x[kFrontWheel] = anchor + qH.rotate(refPos_[kFrontWheel] - steerAnchorRef_);
    for (int i = 0; i < kBodies; ++i) k.v[i] = k.w[i] = Vec3{};
}

bool SingleTrack::buildConfiguration(Real lean, Real steer, Kin& k) const {
    // Tyre mode starts at the static tyre deflection (no drop onto unloaded
    // tyres); NoSlip has rigid wheels.
    Real sinkR = 0, sinkF = 0;
    if (contact_ == SingleTrackContact::Tyre) {
        Real m = 0, mx = 0;
        for (int i = 0; i < kBodies; ++i) {
            m += body_[i].mass;
            mx += body_[i].mass * refPos_[i].x;
        }
        const Real weight = m * def_.gravity;
        const Real front = weight * (mx / m) / def_.wheelbase;
        sinkR = (weight - front) / def_.rearTyre.verticalStiffness;
        sinkF = front / def_.frontTyre.verticalStiffness;
    }
    // Solve the pitch angle that puts the front wheel on the ground (Newton).
    auto frontHeight = [&](Real pitch) {
        placeBodies(lean, steer, pitch, sinkR, k);
        Vec3 d, axle;
        contactDirection(k.q[kFrontWheel], d, axle);
        return k.x[kFrontWheel].z + d.z * def_.frontWheel.radius + sinkF;
    };
    Real pitch = 0;
    for (int it = 0; it < 50; ++it) {
        const Real f = frontHeight(pitch);
        if (std::abs(f) < 1e-15) break;
        constexpr Real h = 1e-7;
        const Real df = (frontHeight(pitch + h) - frontHeight(pitch - h)) / (2 * h);
        if (std::abs(df) < 1e-12) return false;
        const Real stepP = f / df;
        pitch -= stepP;
        if (std::abs(stepP) < 1e-16) break;
    }
    const Real f = frontHeight(pitch);
    return std::abs(f) < 1e-12 && std::abs(pitch) < 1.0;
}

bool SingleTrack::solveVelocities(Real speed, Real leanRate, Real steerRate, Kin& k) const {
    // 21 rolling/joint constraints + 3 prescribed rates (lean, steer, rear
    // wheel spin) determine all 24 body velocities.
    Rows rows;
    if (!buildRows(k, rows, true) || rows.n != kMaxRows) return false;
    DenseSystem<kDofs> sys;
    sys.reset(kDofs);
    auto put = [&](int row, int bodyIndex, const Vec3& lin, const Vec3& ang) {
        const std::size_t c = std::size_t(6 * bodyIndex), r = std::size_t(row);
        sys.a(r, c + 0) += lin.x;
        sys.a(r, c + 1) += lin.y;
        sys.a(r, c + 2) += lin.z;
        sys.a(r, c + 3) += ang.x;
        sys.a(r, c + 4) += ang.y;
        sys.a(r, c + 5) += ang.z;
    };
    for (int r = 0; r < rows.n; ++r) {
        put(r, rows.r[r].a, rows.r[r].la, rows.r[r].aa);
        if (rows.r[r].b >= 0) put(r, rows.r[r].b, rows.r[r].lb, rows.r[r].ab);
    }
    const Vec3 by = k.q[kRearFrame].rotate(kEy);
    const Real phi = std::asin(clamp(by.z, -1.0, 1.0));
    put(21, kRearFrame, {}, cross(by, kEz));  // d(by.z)/dt = cos(phi) * phidot
    sys.b(21) = std::cos(phi) * leanRate;
    const Vec3 u = k.q[kRearFrame].rotate(steerAxisLocal_);
    put(22, kFrontFrame, {}, -u);  // deltadot = -(wH - wB) . u
    put(22, kRearFrame, {}, u);
    sys.b(22) = steerRate;
    const Vec3 axleR = k.q[kRearWheel].rotate(kEy);
    const Real rR = def_.rearWheel.radius;
    put(23, kRearWheel, {}, axleR * rR);  // v = rR * (wR - wB) . axle
    put(23, kRearFrame, {}, axleR * (-rR));
    sys.b(23) = speed;
    if (!sys.solve()) return false;
    for (int i = 0; i < kBodies; ++i) {
        const std::size_t c = std::size_t(6 * i);
        k.v[i] = {sys.x(c + 0), sys.x(c + 1), sys.x(c + 2)};
        k.w[i] = {sys.x(c + 3), sys.x(c + 4), sys.x(c + 5)};
    }
    return true;
}

bool SingleTrack::reset(Real speed, Real lean, Real steer, Real leanRate, Real steerRate) {
    time_ = 0;
    faulted_ = false;
    slip_[0] = slip_[1] = TireSlipState{};
    load_[0] = load_[1] = 0;
    Kin k;
    if (!buildConfiguration(lean, steer, k) || !solveVelocities(speed, leanRate, steerRate, k)) {
        faulted_ = true;
        return false;
    }
    k_ = k;
    for (int i = 0; i < kBodies; ++i) {
        body_[i].position = k_.x[i];
        body_[i].orientation = k_.q[i];
        body_[i].velocity = k_.v[i];
        body_[i].angularVelocity = k_.w[i];
    }
    return true;
}

// -------------------------------------------------------------- observers

Real SingleTrack::leanOf(const Kin& k) {
    const Vec3 by = k.q[kRearFrame].rotate(kEy);
    return std::asin(clamp(by.z, -1.0, 1.0));
}

Real SingleTrack::steerOf(const Kin& k) const {
    Quat rel = k.q[kRearFrame].conjugate() * k.q[kFrontFrame];
    if (rel.w < 0) rel = {-rel.w, -rel.x, -rel.y, -rel.z};
    const Real s = rel.x * steerAxisLocal_.x + rel.y * steerAxisLocal_.y + rel.z * steerAxisLocal_.z;
    return -2.0 * std::atan2(s, rel.w);
}

Real SingleTrack::leanAngle() const {
    return leanOf(k_);
}
Real SingleTrack::steerAngle() const {
    return steerOf(k_);
}

Real SingleTrack::leanRate() const {
    const Vec3 by = k_.q[kRearFrame].rotate(kEy);
    return cross(k_.w[kRearFrame], by).z / std::cos(leanOf(k_));
}

Real SingleTrack::steerRate() const {
    const Vec3 u = k_.q[kRearFrame].rotate(steerAxisLocal_);
    return -dot(k_.w[kFrontFrame] - k_.w[kRearFrame], u);
}

Real SingleTrack::yawAngle() const {
    const Vec3 by = k_.q[kRearFrame].rotate(kEy);
    return std::atan2(-by.x, by.y);
}

Real SingleTrack::forwardSpeed() const {
    const Real psi = yawAngle();
    return dot(k_.v[kRearWheel], Vec3{std::cos(psi), std::sin(psi), 0});
}

Vec3 SingleTrack::position() const {
    Vec3 d, axle;
    contactDirection(k_.q[kRearWheel], d, axle);
    return k_.x[kRearWheel] + d * def_.rearWheel.radius;
}

Real SingleTrack::totalEnergy() const {
    Real e = 0;
    for (int i = 0; i < kBodies; ++i) {
        e += 0.5 * body_[i].mass * dot(k_.v[i], k_.v[i]);
        e += 0.5 * dot(k_.w[i], inertiaWorld(k_, i, k_.w[i]));
        e += body_[i].mass * def_.gravity * k_.x[i].z;
    }
    if (contact_ == SingleTrackContact::Tyre) {
        for (int wi = 0; wi < 2; ++wi) {
            const int i = wi == 0 ? int(kRearWheel) : int(kFrontWheel);
            Vec3 d, axle;
            contactDirection(k_.q[i], d, axle);
            const Real pen = -(k_.x[i].z + d.z * wheelRadius(i));
            const Real kz = wi == 0 ? def_.rearTyre.verticalStiffness : def_.frontTyre.verticalStiffness;
            if (pen > 0) e += 0.5 * kz * pen * pen;
        }
    }
    return e;
}

Real SingleTrack::constraintError() const {
    Rows rows;
    if (!buildRows(k_, rows, contact_ == SingleTrackContact::NoSlip)) return 1e9;
    Real worst = 0;
    for (int r = 0; r < rows.n; ++r)
        if (rows.r[r].holonomic) worst = std::max(worst, std::abs(rows.r[r].c));
    return worst;
}

bool SingleTrack::leanSteerAccelerations(const SingleTrackInput& in, Real& leanAcc, Real& steerAcc) const {
    Vec3 a[kBodies], al[kBodies];
    if (!accelerations(k_, in, a, al)) return false;
    const Vec3 wB = k_.w[kRearFrame];
    const Vec3 by = k_.q[kRearFrame].rotate(kEy);
    const Real phi = leanOf(k_);
    const Real phid = cross(wB, by).z / std::cos(phi);
    // by.z = sin(phi):  d2/dt2 -> (alpha x by + w x (w x by)).z = cos(phi) phidd - sin(phi) phid^2
    leanAcc = (cross(al[kRearFrame], by).z + cross(wB, cross(wB, by)).z + std::sin(phi) * phid * phid) / std::cos(phi);
    // wH - wB = -deltadot * u (u fixed in B)  ->  deltadd = -(alphaH - alphaB) . u
    const Vec3 u = k_.q[kRearFrame].rotate(steerAxisLocal_);
    steerAcc = -dot(al[kFrontFrame] - al[kRearFrame], u);
    return true;
}

SingleTrackLinearization linearizeSingleTrack(const SingleTrackDef& def, Real speed, Real h) {
    SingleTrackLinearization out;
    out.speed = speed;
    SingleTrack st(def, SingleTrackContact::NoSlip);
    auto acc = [&](const Real x[4], const SingleTrackInput& in, Real& p, Real& d) {
        return st.reset(speed, x[0], x[1], x[2], x[3]) && st.leanSteerAccelerations(in, p, d);
    };
    const SingleTrackInput none{};
    for (int j = 0; j < 4; ++j) {
        Real xp[4] = {0, 0, 0, 0}, xm[4] = {0, 0, 0, 0};
        xp[j] = h;
        xm[j] = -h;
        Real pp, dp, pm, dm;
        if (!acc(xp, none, pp, dp) || !acc(xm, none, pm, dm)) return out;
        Real(*target)[2] = j < 2 ? out.Aq : out.Aqd;
        target[0][j % 2] = (pp - pm) / (2 * h);
        target[1][j % 2] = (dp - dm) / (2 * h);
    }
    // Accelerations are exactly linear in applied torques: unit-torque responses.
    const Real x0[4] = {0, 0, 0, 0};
    Real p0, d0;
    if (!acc(x0, none, p0, d0)) return out;
    SingleTrackInput lean{}, steer{};
    lean.rollTorque = 1.0;
    steer.steerTorque = 1.0;
    Real p, d;
    if (!acc(x0, lean, p, d)) return out;
    out.Minv[0][0] = p - p0;
    out.Minv[1][0] = d - d0;
    if (!acc(x0, steer, p, d)) return out;
    out.Minv[0][1] = p - p0;
    out.Minv[1][1] = d - d0;
    out.ok = true;
    return out;
}

}  // namespace ord
