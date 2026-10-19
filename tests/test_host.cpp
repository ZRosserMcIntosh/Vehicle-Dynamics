// OpenRoad Dynamics — host integration tests (frames, external chassis mode).
// SPDX-License-Identifier: MIT
#include "ord/host_frame.hpp"
#include "ord/presets.hpp"
#include "ord/vehicle.hpp"
#include "ord_test.hpp"
#include "reference_vehicles.hpp"
#include "scenarios.hpp"

using namespace ord;

namespace {
// Quaternion rotation written in plain components, as any engine computes it.
host::UeVec ueRotate(const host::UeQuat& q, const host::UeVec& v) {
    const Quat qq{q.w, q.x, q.y, q.z};
    const Vec3 r = qq.rotate({v.x, v.y, v.z});
    return {r.x, r.y, r.z};
}
host::UeVec ueCross(const host::UeVec& a, const host::UeVec& b) {
    const Vec3 c = cross(Vec3{a.x, a.y, a.z}, Vec3{b.x, b.y, b.z});
    return {c.x, c.y, c.z};
}
void checkVec(const Vec3& a, const Vec3& b, double tol) {
    CHECK_NEAR(a.x, b.x, tol);
    CHECK_NEAR(a.y, b.y, tol);
    CHECK_NEAR(a.z, b.z, tol);
}
}  // namespace

TEST_CASE("host frame: Unreal <-> ISO conversions commute with rotation and cross products") {
    const Quat qo = Quat::fromAxisAngle(normalized(Vec3{0.3, -0.5, 0.8}), 0.9);
    const host::UeQuat qu = host::rotationToUe(qo);
    const host::UeVec vu{120.0, -45.0, 33.0};  // cm
    // Rotate in UE then convert == convert then rotate in ISO.
    checkVec(host::positionFromUe(ueRotate(qu, vu)), qo.rotate(host::positionFromUe(vu)), 1e-12);
    // Round trips.
    const Quat back = host::rotationFromUe(qu);
    CHECK_NEAR(back.w, qo.w, 1e-15);
    CHECK_NEAR(back.y, qo.y, 1e-15);
    checkVec(host::positionFromUe(host::positionToUe({1, 2, 3})), {1, 2, 3}, 1e-12);
    // v = w x r must hold in both frames (axial vs polar handling).
    const host::UeVec wu{0.4, -1.1, 0.7};   // rad/s
    const host::UeVec ru{200.0, 80.0, -15.0};
    const Vec3 vIso = cross(host::angularVelocityFromUe(wu), host::positionFromUe(ru));
    checkVec(host::velocityFromUe(ueCross(wu, ru)), vIso, 1e-12);
    // Torque = r x F in both frames, with unit scaling.
    const Vec3 r{1.2, -0.4, 0.3}, f{100, 250, -40};
    const host::UeVec tu = ueCross(host::positionToUe(r), host::forceToUe(f));
    const host::UeVec tExpect = host::torqueToUe(cross(r, f));
    CHECK_NEAR(tu.x, tExpect.x, 1e-6);
    CHECK_NEAR(tu.y, tExpect.y, 1e-6);
    CHECK_NEAR(tu.z, tExpect.z, 1e-6);
    // Yaw left in ISO (+Z) is a negative UE yaw rate about +Z.
    CHECK(host::angularVelocityToUe({0, 0, 1}).z < 0);
}

namespace {
// A host rigid-body integrator: same scheme as the core (parity) or a different one.
void hostIntegrate(RigidBodyState& s, const ChassisForces& f, const VehicleDefinition& d, Real dt, bool alternative) {
    const Vec3 com = s.position + s.orientation.rotate(d.centerOfMass);
    const Vec3 vOld = s.velocity;
    s.velocity += f.force * (dt / d.mass);
    const Mat3 R = s.orientation.toMatrix();
    const Mat3 inv = R * Mat3::diagonal({1 / d.inertia.x, 1 / d.inertia.y, 1 / d.inertia.z}) * R.transposed();
    const Mat3 I = R * Mat3::diagonal(d.inertia) * R.transposed();
    const Vec3 w0 = s.angularVelocity;
    s.angularVelocity += inv * (f.torque - cross(w0, I * w0)) * dt;
    // Alternative: trapezoidal position update and exponential-map rotation
    // (closer to some engine integrators).
    const Vec3 v = alternative ? (vOld + s.velocity) * 0.5 : s.velocity;
    const Vec3 newCom = com + v * dt;
    const Vec3 w = s.angularVelocity;
    Quat q = s.orientation;
    if (alternative) {
        const Real ang = length(w) * dt;
        if (ang > 1e-12) q = Quat::fromAxisAngle(w, ang) * q;
    } else {
        const Quat qn = Quat{0, w.x, w.y, w.z} * q;
        q = {q.w + 0.5 * dt * qn.w, q.x + 0.5 * dt * qn.x, q.y + 0.5 * dt * qn.y, q.z + 0.5 * dt * qn.z};
    }
    q.normalize();
    s.orientation = q;
    s.position = newCom - q.rotate(d.centerOfMass);
}

struct DriveResult {
    Real speed, yawRate, x, y;
};

DriveResult drive(ChassisMode mode, bool alternative, bool hostGravity, Real hostHz = 240, int substeps = 1) {
    const VehicleDefinition d = vehicles::referenceSportsRwd();
    Vehicle v(d, makePreset(DrivingPreset::Sport));
    FlatGround g;
    v.reset({0, 0, 0.6}, 0, g);
    v.setChassisMode(mode, hostGravity);
    DriverInput in;
    const Real dt = 1.0 / hostHz;
    const int n = static_cast<int>(hostHz * 8);
    for (int i = 0; i < n; ++i) {
        const Real t = i * dt;
        in.throttle = t < 5 ? 0.8 : 0.0;
        in.brake = t >= 6 ? 0.3 : 0.0;
        in.steer = t > 3 ? 0.15 : 0.0;
        v.stepSubstepped(dt, substeps, in, g);
        if (mode == ChassisMode::External) {
            RigidBodyState s = v.state();
            ChassisForces f = v.chassisForces();
            if (hostGravity) f.force += Vec3{0, 0, -kGravity * d.mass};  // what the host's own gravity adds
            hostIntegrate(s, f, d, dt, alternative);
            v.setChassisState(s);
        }
    }
    const auto& s = v.state();
    return {v.forwardSpeed(), v.yawRate(), s.position.x, s.position.y};
}
}  // namespace

TEST_CASE("host: external chassis mode with the same integrator reproduces internal mode exactly") {
    const DriveResult a = drive(ChassisMode::Internal, false, false);
    const DriveResult b = drive(ChassisMode::External, false, true);
    CHECK_NEAR(a.speed, b.speed, 1e-9);
    CHECK_NEAR(a.x, b.x, 1e-9);
    CHECK_NEAR(a.y, b.y, 1e-9);
}

TEST_CASE("host: a different host integrator stays within 1% (Chaos-parity risk)") {
    const DriveResult a = drive(ChassisMode::Internal, false, false);
    const DriveResult b = drive(ChassisMode::External, true, true);
    CHECK_NEAR(b.speed / a.speed, 1.0, 0.01);
    const Real dist = std::hypot(a.x, a.y);
    CHECK(std::hypot(a.x - b.x, a.y - b.y) < 0.01 * dist);
}

TEST_CASE("host: 60 Hz host with 4 vehicle substeps matches a 240 Hz internal run within 1%") {
    const DriveResult ref = drive(ChassisMode::Internal, false, false, 240, 1);
    const DriveResult sub = drive(ChassisMode::External, true, true, 60, 4);
    CHECK_NEAR(sub.speed / ref.speed, 1.0, 0.01);
    CHECK(std::hypot(ref.x - sub.x, ref.y - sub.y) < 0.01 * std::hypot(ref.x, ref.y));
}

TEST_CASE("host: one shift press shifts one gear, however many substeps the frame runs") {
    for (ChassisMode mode : {ChassisMode::Internal, ChassisMode::External}) {
        Vehicle v(vehicles::referenceSportsRwd(), makePreset(DrivingPreset::Simulation));  // manual gearbox
        FlatGround g;
        v.reset({0, 0, 0.6}, 0, g);
        v.setChassisMode(mode, false);
        DriverInput in;
        in.shiftUp = true;
        v.stepSubstepped(1.0 / 60, 4, in, g);
        if (mode == ChassisMode::External) v.setChassisState(v.state());
        const int after = v.telemetry().gear;
        in.shiftUp = false;
        v.stepSubstepped(1.0 / 60, 4, in, g);
        CHECK(after == 2);
        CHECK(v.telemetry().gear == 2);
    }
}
