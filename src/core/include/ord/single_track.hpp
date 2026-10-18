// OpenRoad Dynamics — engine-independent vehicle physics core.
// SPDX-License-Identifier: MIT
//
// Nonlinear rigid multibody single-track (two-wheeler) model: the first step
// toward bicycles, e-scooters and motorcycles. See
// docs/architecture/two-wheelers.md.
//
// Four rigid bodies in maximal coordinates (6 DOF each, world frame):
//   R  rear wheel            (thin disc)
//   B  rear frame + rider    (rigid rider, no hands-free motion)
//   H  front frame           (fork + handlebar)
//   F  front wheel           (thin disc)
// joined by three revolute joints (rear axle R-B, steer axis B-H tilted back by
// lambda, front axle H-F) and touching flat ground (z = 0, z up) at one point
// per wheel. Gyroscopic terms (omega x I omega) are included for every body.
//
// Contact modes:
//   NoSlip  Whipple/Carvallo rolling: the material velocity of each wheel's
//           contact point is zero (3 velocity constraints per wheel; the
//           vertical one is the derivative of the holonomic "contact height
//           = 0" constraint, which is also enforced by position projection).
//           Used for validation against the Meijaard et al. benchmark.
//   Tyre    Normal spring-damper on the rim penetration plus longitudinal and
//           lateral forces from ord::MagicFormulaTire with relaxation-length
//           transient slip, camber thrust and camber twisting torque, on a
//           toroidal (rounded-crown) tyre whose contact point rolls around
//           the crown as the wheel leans. Each wheel sees a local ground plane
//           from one ray per step, so it runs on uneven ground.
//
// Dynamics: constrained Newton-Euler equations at acceleration level,
//   M a = F_ext + J^T lambda,   J a = -Jdot v,
// solved densely each evaluation ((J M^-1 J^T) lambda = -Jdot v - J M^-1 F),
// integrated with classical RK4 (quaternions for orientation), followed by a
// mass-weighted position projection onto the holonomic constraints and a
// velocity projection onto all velocity constraints (removes drift). Fixed-size
// arrays only: the step does not allocate.
//
// The model is derived from first principles (rigid bodies, joints, rolling
// contact). It does NOT use the closed-form linear matrices of Meijaard et al.;
// those are only used by the tests as an independent reference.
//
// References:
//  - J.P. Meijaard, J.M. Papadopoulos, A. Ruina, A.L. Schwab, "Linearized
//    dynamics equations for the balance and steer of a bicycle: a benchmark
//    and review", Proc. R. Soc. A 463:1955-1982, 2007 (benchmark geometry and
//    mass data, Table 1; lean/steer coordinate definitions).
//  - F.J.W. Whipple, "The stability of the motion of a bicycle", Quarterly
//    Journal of Pure and Applied Mathematics 30:312-348, 1899.
//  - D. Baraff, "Linear-time dynamics using Lagrange multipliers", SIGGRAPH
//    1996 (acceleration-level constrained rigid-body dynamics).
//  - E. Hairer, C. Lubich, G. Wanner, "Geometric Numerical Integration", 2nd
//    ed., Springer 2006, ch. IV.4 (projection methods).
//
// Sign conventions. The world is ord's (x forward, y left, z up). Lean and
// steer follow the benchmark literature so results compare directly with
// Meijaard et al.: lean phi > 0 = leaning to the RIGHT, steer delta > 0 =
// steering to the RIGHT (rotation of the front frame about the downward steer
// axis), steer torque > 0 acts toward positive delta. Meijaard's paper axes
// (x forward, y right, z down) map to ord as (x, -y, -z).
#pragma once

#include "ord/ground.hpp"
#include "ord/math.hpp"
#include "ord/tire.hpp"

namespace ord {

enum class SingleTrackContact { NoSlip, Tyre };

// Thin-disc wheel. Inertias about the wheel centre.
struct SingleTrackWheelDef {
    Real radius = 0.3;             // m
    Real mass = 2.0;               // kg
    Real inertiaDiametral = 0.06;  // kg*m^2, about any diameter (Ixx = Izz)
    Real inertiaAxial = 0.12;      // kg*m^2, about the axle (Iyy)
};

// Rigid frame. Centre of mass and inertia (about the CoM) are given in ord
// axes in the upright, straight-ahead reference configuration with the origin
// at the rear-wheel contact point.
struct SingleTrackFrameDef {
    Real mass = 1.0;
    Vec3 centreOfMass{};
    Mat3 inertia{};
};

struct SingleTrackDef {
    Real wheelbase = 1.02;          // m, rear to front contact point (w)
    Real trail = 0.08;              // m, front contact behind the steer-axis ground point (c)
    Real steerAxisTilt = kPi / 10;  // rad, steer axis tilt back from vertical (lambda)
    Real gravity = 9.81;            // m/s^2

    SingleTrackWheelDef rearWheel;
    SingleTrackFrameDef rearFrame;   // incl. rigid rider
    SingleTrackFrameDef frontFrame;  // fork + handlebar
    SingleTrackWheelDef frontWheel;

    // Tyre-mode only. radius / wheelInertia are taken from the wheel defs.
    TireParams rearTyre;
    TireParams frontTyre;
    SurfaceProperties surface;
};

// Meijaard et al. (2007) benchmark bicycle, Table 1, converted to ord axes.
// Tyre-mode parameters are plausible bicycle-tyre values (not part of the
// benchmark, which assumes rolling without slip).
SingleTrackDef meijaardBenchmark();

struct SingleTrackInput {
    Real steerTorque = 0;  // N*m, rider steer torque between rear and front frame (+ toward right steer)
    Real driveTorque = 0;  // N*m, between rear frame and rear wheel (+ drives forward)
    Real frontWheelTorque = 0;  // N*m, between front frame and front wheel (+ forward; brakes, hub motors)
    // N*m, external couple on the rear frame about its horizontal heading axis
    // (+ toward right lean). Not a rider actuator: a disturbance (gust, push)
    // and the generalised lean torque T_phi used for validation.
    Real rollTorque = 0;
};

struct SingleTrackBody {
    Real mass = 1;
    Mat3 inertiaBody{};  // about the CoM, body axes
    Mat3 invInertiaBody{};
    Vec3 position{};         // CoM, world
    Quat orientation{};      // body -> world
    Vec3 velocity{};         // world
    Vec3 angularVelocity{};  // world
};

class SingleTrack {
public:
    enum BodyIndex { kRearWheel = 0, kRearFrame = 1, kFrontFrame = 2, kFrontWheel = 3 };
    static constexpr int kBodies = 4;
    static constexpr int kDofs = 6 * kBodies;  // 24
    static constexpr int kMaxRows = 21;        // 3 x 5 joint rows + 2 x 3 rolling-contact rows

    explicit SingleTrack(const SingleTrackDef& def, SingleTrackContact contact = SingleTrackContact::NoSlip);

    // Upright-ish, yaw 0, rear contact at the origin, rolling at `speed` (m/s,
    // rear-wheel spin relative to the rear frame times its radius) with the
    // given lean/steer perturbation (benchmark signs). Pitch is solved so both
    // wheels touch the ground (Tyre mode: at the static tyre deflection) and
    // the initial velocities are pure rolling. Returns false if no consistent
    // state exists.
    bool reset(Real speed, Real lean = 0, Real steer = 0, Real leanRate = 0, Real steerRate = 0);

    // Rigidly moves the whole state: rotate by `yaw` about the world z axis
    // through the origin, then translate by `offset` (velocities rotate too).
    // Call after reset() to start somewhere other than the origin, e.g. on
    // ground at height h: place({x, y, h}, heading).
    void place(const Vec3& offset, Real yaw);

    // One fixed step. Validation uses 1e-4..5e-4 s. Without a ground query
    // the ground is the plane z = 0 with def().surface.
    void step(Real dt, const SingleTrackInput& in = {});
    // Tyre mode on arbitrary ground: one vertical ray per wheel per step gives
    // that wheel a local ground plane (point, normal, surface) for the step.
    // NoSlip mode ignores `ground` (validation runs on z = 0).
    void step(Real dt, const SingleTrackInput& in, const GroundQuery* ground);

    Real leanAngle() const;     // rad, + right
    Real steerAngle() const;    // rad, + right
    Real leanRate() const;      // rad/s
    Real steerRate() const;     // rad/s
    Real yawAngle() const;      // rad, heading of the rear frame (ord: + left)
    Real forwardSpeed() const;  // m/s, rear-wheel-centre velocity along the heading
    // Wheel spin relative to its frame, rad/s (+ = rolling forward); 0 rear, 1 front.
    Real wheelSpin(int wheel) const;
    // Pitch of the rear frame, rad (+ nose up, e.g. a wheelie), relative to
    // its upright reference attitude.
    Real pitchAngle() const;
    Vec3 position() const;      // rear contact point, world
    // Kinetic + gravitational potential (+ tyre spring energy in Tyre mode), J.
    Real totalEnergy() const;
    // Largest holonomic constraint violation (m or rad); diagnostics.
    Real constraintError() const;
    Real time() const { return time_; }
    bool faulted() const { return faulted_; }
    SingleTrackContact contactMode() const { return contact_; }
    const SingleTrackBody& body(int i) const { return body_[i]; }
    const SingleTrackDef& def() const { return def_; }
    // Tyre mode: last normal load per wheel (0 rear, 1 front), N.
    Real normalLoad(int wheel) const { return tyreOut_[wheel].load; }
    // Tyre mode, per wheel (0 rear, 1 front), from the end of the last step.
    struct TyreOutput {
        Real load = 0;        // N, normal
        Real fx = 0, fy = 0;  // N, contact frame (fy + toward the wheel's left)
        Real camber = 0;      // rad, inclination to the ground normal
        Real gripUsed = 0;    // combined force / available friction, 0..~1
        bool contact = false;
    };
    const TyreOutput& tyreOutput(int wheel) const { return tyreOut_[wheel]; }
    const TireSlipState& tyreSlip(int wheel) const { return slip_[wheel]; }
    // Lower friction scale of the surfaces under the two wheels (last step).
    Real surfaceFriction() const {
        return std::min(plane_[0].surface.frictionScale, plane_[1].surface.frictionScale);
    }
    int surfaceId(int wheel) const { return plane_[wheel].surfaceId; }

    // Lean and steer accelerations of the current state under `in`, from the
    // full constrained nonlinear dynamics (no integration). Used to extract
    // the linearised model numerically.
    bool leanSteerAccelerations(const SingleTrackInput& in, Real& leanAcc, Real& steerAcc) const;

private:
    struct Kin {
        Vec3 x[kBodies];
        Quat q[kBodies];
        Vec3 v[kBodies];
        Vec3 w[kBodies];
    };
    struct Row {
        int a = -1, b = -1;   // body indices (b = -1: ground)
        Vec3 la, aa, lb, ab;  // linear/angular Jacobian blocks
        Real gamma = 0;       // Jdot * v
        Real c = 0;           // holonomic violation (if holonomic)
        bool holonomic = false;
    };
    struct Rows {
        Row r[kMaxRows];
        int n = 0;
    };
    struct Joint {
        int a, b;
        Vec3 anchorA, anchorB;  // body-local anchor offsets from each CoM
        Vec3 axisA;             // body-local joint axis in a
        Vec3 perpB1, perpB2;    // body-local perpendiculars in b
    };

    bool buildRows(const Kin& k, Rows& rows, bool withContact) const;
    void externalForces(const Kin& k, const SingleTrackInput& in, Vec3 f[kBodies], Vec3 t[kBodies],
                        TyreOutput* out) const;
    bool accelerations(const Kin& k, const SingleTrackInput& in, Vec3 a[kBodies], Vec3 al[kBodies]) const;
    bool projectPositions(Kin& k) const;
    bool projectVelocities(Kin& k) const;
    bool buildConfiguration(Real lean, Real steer, Kin& k) const;
    void placeBodies(Real lean, Real steer, Real pitch, Real rearSink, Kin& k) const;
    bool solveVelocities(Real speed, Real leanRate, Real steerRate, Kin& k) const;
    void updateTyreSlip(Real dt);
    struct GroundPlane {
        Vec3 point{};
        Vec3 normal{0, 0, 1};
        SurfaceProperties surface;
        int surfaceId = 0;
        bool valid = true;
    };
    struct TyreContact {
        Vec3 axle, n, r;  // wheel axle (+y), ground normal, centre -> contact point
        Vec3 tdir, ldir;  // contact heading and lateral directions (in the ground plane)
        Real pen = 0;     // tyre penetration along n, m
        Real camber = 0;  // inclination to n, rad (top toward the wheel's +y positive)
    };
    bool tyreContact(const Kin& k, int wheel, TyreContact& c) const;
    void updateGround(const GroundQuery* ground);

    Vec3 invInertiaWorld(const Kin& k, int i, const Vec3& v) const;
    Vec3 inertiaWorld(const Kin& k, int i, const Vec3& v) const;
    Real wheelRadius(int bodyIndex) const;
    Real crown(int bodyIndex) const;  // Tyre-mode crown radius (0 in NoSlip)
    static Real leanOf(const Kin& k);
    Real steerOf(const Kin& k) const;

    SingleTrackDef def_;
    SingleTrackContact contact_;
    Kin k_;                          // simulation state
    SingleTrackBody body_[kBodies];  // mirror of k_ for the accessor
    Joint joints_[3];
    Vec3 refPos_[kBodies];  // reference CoM positions (world, upright)
    Vec3 steerAnchorRef_;   // point on the steer axis (world, upright)
    Vec3 steerAxisLocal_;   // upward steer axis in B/H local axes
    MagicFormulaTire rearTyre_, frontTyre_;
    TireSlipState slip_[2];
    GroundPlane plane_[2];
    TyreOutput tyreOut_[2];
    Real time_ = 0;
    Real refPitch_ = 0;
    bool faulted_ = false;
};

// Numerical linearisation of the nonlinear NoSlip model about upright straight
// running at `speed`, in the benchmark states q = [phi, delta]:
//   qdd = Aq q + Aqd qd + Minv [T_phi, T_delta]
// (central finite differences of leanSteerAccelerations). For the benchmark
// linear model, Aq = -M^-1 (g K0 + v^2 K2), Aqd = -v M^-1 C1.
struct SingleTrackLinearization {
    Real speed = 0;
    Real Aq[2][2] = {{0, 0}, {0, 0}};
    Real Aqd[2][2] = {{0, 0}, {0, 0}};
    Real Minv[2][2] = {{0, 0}, {0, 0}};
    bool ok = false;
};
SingleTrackLinearization linearizeSingleTrack(const SingleTrackDef& def, Real speed, Real step = 1e-6);

}  // namespace ord
