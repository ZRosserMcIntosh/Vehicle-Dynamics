// OpenRoad Dynamics — engine-independent vehicle physics core.
// SPDX-License-Identifier: MIT
//
// Wheeled vehicle: 6-DOF chassis, per-corner suspension with optional
// unsprung-mass DOF and multi-point contact patch, Magic Formula tires with
// transient slip, implicit drivetrain/wheel solve, aero and the assist layer.
// The same class serves LOD0 (player) and LOD1 (nearby AI) through
// VehicleFidelity.
//
// Step order (one fixed step):
//   1. assists turn DriverInput into ActuatorCommands (using last step's sensors)
//   2. contact sampling (1..9 points per wheel) through GroundQuery
//   3. suspension: implicit unsprung-mass solve (or massless algebraic)
//   4. lateral transient slip; implicit drivetrain/wheel/longitudinal-slip solve
//   5. combined-slip tire forces, aero -> ChassisForces
//   6. chassis integration (ChassisMode::Internal) or hand-off to the host
//      rigid body (ChassisMode::External, e.g. Chaos in Unreal)
//   7. telemetry
#pragma once

#include <array>
#include <vector>

#include "ord/assists.hpp"
#include "ord/drivetrain.hpp"
#include "ord/ground.hpp"
#include "ord/ground_cache.hpp"
#include "ord/inputs.hpp"
#include "ord/telemetry.hpp"
#include "ord/tire.hpp"
#include "ord/vehicle_def.hpp"

namespace ord {

inline constexpr int kMaxPatchSamples = 9;
inline constexpr int kMaxEnvelopeProbes = 3;  // per side (ahead / behind the patch)
inline constexpr int kMaxContactSamples = kMaxPatchSamples + 2 * kMaxEnvelopeProbes;

struct RigidBodyState {
    Vec3 position;           // world position of the body origin
    Quat orientation;        // body -> world
    Vec3 velocity;           // world, of the centre of mass
    Vec3 angularVelocity;    // world
};

// Per-instance cost/fidelity switches. LOD0 and LOD1 differ only here.
struct VehicleFidelity {
    bool unsprungMass = true;   // per-corner wheel DOF with tire vertical stiffness (needs WheelDef::unsprungMass > 0)
    int contactSamples = 5;     // 1 (centre), 5 (centre + 4 edges), 8 (perimeter), 9 (3x3)
    bool telemetry = true;      // fill the full TelemetryFrame every step
    // Envelope probes per side: radial samples ahead of / behind the patch
    // against the tire's circular profile. They carry no load on flat ground
    // and pick up steps (curbs, pothole edges) before the patch reaches them,
    // like a real tire's geometric enveloping. Unsprung mode with > 1 patch sample only.
    int envelopeProbes = 3;

    static VehicleFidelity lod0() { return {true, 5, true, 3}; }
    static VehicleFidelity lod1() { return {false, 1, false, 0}; }
};

enum class ChassisMode {
    Internal,  // the core integrates the chassis (headless, servers, tests)
    External,  // the host integrates: read chassisForces(), write setChassisState()
};

// Net force and torque on the chassis for the last step, world frame, torque
// about the centre of mass. Includes gravity unless the host applies it.
struct ChassisForces {
    Vec3 force;
    Vec3 torque;
};

class Vehicle {
public:
    // The definition must be valid (VehicleDefinition::validate() == ""). An
    // invalid definition leaves the vehicle inert (isValid() == false); no
    // exceptions are thrown, so the core builds with exceptions disabled.
    explicit Vehicle(const VehicleDefinition& def, const AssistConfig& assists = {},
                     const VehicleFidelity& fidelity = VehicleFidelity::lod0());

    bool isValid() const { return valid_; }
    const std::string& error() const { return error_; }
    // Numerical fault guard: if a step ever produces a non-finite force or
    // state, the previous chassis state is kept, forces are zeroed and the
    // vehicle stops stepping until reset()/placeOnGround(). Hosts must never
    // receive NaN/Inf (it would poison their rigid-body solver).
    bool faulted() const { return faulted_; }

    // Places the car at rest on the ground under (x, y), heading `yaw`
    // (rad about +Z), in static equilibrium (analytic, then a short settle).
    void reset(const Vec3& position, Real yaw, const GroundQuery& ground);

    // Analytic placement in static equilibrium with the given planar motion
    // (no settling steps): used for spawning and LOD promotion.
    void placeOnGround(Real x, Real y, Real yaw, const GroundQuery& ground, Real forwardSpeed = 0,
                       Real lateralSpeed = 0, Real yawRate = 0);

    void step(Real dt, const DriverInput& input, const GroundQuery& ground);

    // Vehicle-local substepping: advances the car by `hostDt` in `substeps`
    // equal internal steps. In ChassisMode::External the chassis is integrated
    // internally across the substeps and chassisForces() returns the
    // impulse-equivalent average force/torque for the whole host step, so the
    // host world can run at 60-120 Hz while tires and suspension run at
    // 240-480 Hz (docs/decisions/ground-prefetch.md, "Rate decoupling").
    void stepSubstepped(Real hostDt, int substeps, const DriverInput& input, const GroundQuery& ground);

    // ---- Host integration (ChassisMode::External) ----
    void setChassisMode(ChassisMode mode, bool hostAppliesGravity) {
        chassisMode_ = mode;
        hostGravity_ = hostAppliesGravity;
    }
    ChassisMode chassisMode() const { return chassisMode_; }
    // Forces from the last step(). In External mode apply them to the host body
    // for the same dt, integrate, then call setChassisState() before next step().
    const ChassisForces& chassisForces() const { return forces_; }
    void setChassisState(const RigidBodyState& s);

    // Host-coupling diagnostics: difference between the chassis state the host
    // integrated and the one predicted by stepSubstepped() for the same step.
    struct HostSyncStats {
        Real positionError = 0, angleError = 0;        // last sync (m, rad)
        Real maxPositionError = 0, maxAngleError = 0;  // since construction
        std::uint64_t syncs = 0;
    };
    const HostSyncStats& hostSync() const { return hostSync_; }
    // Pose differences larger than this are treated as teleports, not carried by the wheels.
    static constexpr Real kMaxCarriedPoseError = 0.05;  // m

    // ---- Ground prefetch (see GroundPrefetchCache) ----
    // Appends one request per wheel; `out` is reused by the caller.
    void groundPrefetchRequests(std::vector<GroundStripRequest>& out) const;

    // Steady wind, world frame (m/s, the direction the air moves).
    void setWind(const Vec3& w) { wind_ = w; }
    // Tread temperature of every tyre (deg C), e.g. tyre warmers at the start.
    void setTyreTemperatures(Real celsius);

    // ---- Run-time tuning (garage settings; take effect on the next step) ----
    // Steering lock: scales every steered wheel's maxSteerAngle so the largest
    // becomes `maxAngle` (rad, 0.05..1.2). Drift cars run 0.8-1.0 rad.
    bool setSteeringLock(Real maxAngle);
    // Replaces a driven axle's differential (type, preload, ramps). The final
    // drive ratio is kept. Returns false for a bad index or out-of-range values.
    bool setDifferential(std::size_t drivenAxle, const DifferentialDef& d);

    // ---- Accessors ----
    const VehicleDefinition& definition() const { return def_; }
    const VehicleFidelity& fidelity() const { return fidelity_; }
    void setFidelity(const VehicleFidelity& f);
    const RigidBodyState& state() const { return body_; }
    RigidBodyState& mutableState() { return body_; }
    const TelemetryFrame& telemetry() const { return telem_; }
    AssistSystem& assists() { return assists_; }
    const AssistSystem& assists() const { return assists_; }
    const Drivetrain& drivetrain() const { return drivetrain_; }
    Vec3 centerOfMassWorld() const { return body_.position + body_.orientation.rotate(def_.centerOfMass); }
    Vec3 bodyVelocity() const { return body_.orientation.inverseRotate(body_.velocity); }
    Real forwardSpeed() const { return bodyVelocity().x; }
    Real yawRate() const;
    Real time() const { return time_; }
    std::uint64_t groundQueries() const { return groundQueries_; }

    // Sets body speed and consistent wheel spin, slip and engine state
    // (tests and LOD promotion). Picks a sensible gear.
    void setRollingSpeed(Real forwardSpeed);
    // Chooses the highest forward gear that keeps the engine above ~35% of the
    // limiter at this road speed (used before placeOnGround for LOD promotion).
    void selectGearForSpeed(Real forwardSpeed);

    // Per-wheel contact geometry of the last step (debug drawing).
    struct ContactDebug {
        int samples = 0;
        std::array<Vec3, kMaxContactSamples> point{};
        std::array<Real, kMaxContactSamples> penetration{};
        Vec3 patchCenter, normal, wheelCenter, hardpoint;
        Vec3 longDir, latDir;
        Real fz = 0, fx = 0, fy = 0;
        bool contact = false;
    };
    const ContactDebug& contactDebug(std::size_t wheel) const { return debug_[wheel]; }

private:
    struct WheelState {
        // Suspension / unsprung DOF
        Real compression = 0;       // m, 0 = full droop
        Real compressionRate = 0;   // m/s
        Real prevCompression = 0;
        std::array<Real, kMaxContactSamples> prevPen{};
        // Contact
        bool contact = false;
        GroundHit hit;              // representative (surface of the patch)
        Vec3 patchPoint;
        Vec3 normal{0, 0, 1};
        Real load = 0;
        Real springForce = 0;       // suspension force on the body along +up
        Vec3 tireNormalForce;       // unsprung mode: sum of radial sample forces, world
        Real relativeAccel = 0;     // unsprung mode: wheel acceleration relative to the body along dofDir
        Vec3 dofDir{0, 0, 1};       // world direction the wheel centre moves per metre of compression (linkage path)
        Vec3 wheelCenter;           // world
        TireSlipState slip;
        Real omega = 0;
        Vec3 longDir, latDir;
        Real contactVx = 0, contactVy = 0;
        Real steer = 0;
        TireForces forces;
        Real camber = 0;            // rad, inclination to the ground normal (top toward +y positive)
        Real temperature = 25;      // deg C, tread (thermal model)
    };

    struct Sample {
        bool hit = false;
        Real d = 0;            // distance from the sample plane (through the hardpoint) along -up
        Vec3 point, normal;
        Real s = 0;            // longitudinal offset in the patch
        bool probe = false;    // envelope probe (radial penetration against the tire circle)
    };

    void computeStep(Real dt, const DriverInput& input, const GroundQuery& ground);
    void integrateChassis(Real dt, const Vec3& extraForce = {});
    void updateAccelEstimate(const Vec3& vNew, const Vec3& wNew, Real dt);
    VehicleSensors buildSensors(Real dt) const;
    Real wheelSteerAngle(std::size_t w, Real refAngle) const;
    int sampleContact(std::size_t w, const Vec3& hp, const Vec3& up, const Vec3& heading, const GroundQuery& ground,
                      std::array<Sample, kMaxContactSamples>& out, int& patchCount);
    void staticWheelLoads(std::array<Real, kMaxWheels>& load) const;
    // Linkage kinematics (kinematics.hpp): pose at a compression, and the spring
    // force / rate at the wheel through the motion ratio (sliders: k * x).
    LinkagePose linkagePose(std::size_t w, Real compression) const;
    Real springForceAt(std::size_t w, Real compression) const;
    Real springRatio(std::size_t w, Real compression) const;
    bool usesUnsprung(std::size_t w) const { return fidelity_.unsprungMass && def_.wheels[w].unsprungMass > 0; }
    void fillTelemetry(const DriverInput& input, const ActuatorCommands& cmd,
                       const std::array<WheelSolveOutput, kMaxWheels>& solveOut);

    VehicleDefinition def_;
    bool valid_ = false;
    bool faulted_ = false;
    std::string error_;
    bool stateFinite() const;
    VehicleFidelity fidelity_;
    std::vector<MagicFormulaTire> tires_;
    AssistSystem assists_;
    Drivetrain drivetrain_;
    RigidBodyState body_;
    std::array<WheelState, kMaxWheels> wheels_{};
    std::array<ContactDebug, kMaxWheels> debug_{};
    std::array<bool, kMaxWheels> driven_{};
    std::array<bool, kMaxWheels> front_{};
    std::array<int, kMaxWheels> axleOf_{};
    Mat3 invInertiaBody_;
    Vec3 inertiaBody_;
    Real maxSteerRef_ = 0;
    std::array<Real, kMaxWheels> rideCompression_{};  // compression at static ride height
    std::array<LinkageTable, kMaxWheels> linkage_{};  // built once per wheel with a real linkage
    std::array<Real, kMaxWheels> kinT0_{};             // linkage travel = compression + kinT0
    std::array<Real, kMaxWheels> kinDroopS_{};         // spring deflection at full droop (wheel metres)
    Vec3 wind_{};
    AeroTelemetry aeroTel_;
    // Body collision hull (see BodyDef): points in the body frame, built once.
    static constexpr int kHullPoints = 17 + kMaxWheels;  // body box + one tyre shoulder per wheel
    std::array<Vec3, kHullPoints> hull_{};
    int hullCount_ = 0;
    int bodyContacts_ = 0;
    Real bodyContactForce_ = 0;
    Real time_ = 0;
    Real lastDt_ = 1.0 / 240.0;
    ChassisMode chassisMode_ = ChassisMode::Internal;
    bool hostGravity_ = false;
    ChassisForces forces_;
    RigidBodyState predicted_;
    bool havePrediction_ = false;
    HostSyncStats hostSync_;
    Vec3 accel_, angAccel_;          // last chassis accelerations (for the unsprung DOF)
    Vec3 prevVelocity_, prevAngularVelocity_;
    std::uint64_t groundQueries_ = 0;
    TelemetryFrame telem_;
    ActuatorCommands lastCmd_;
};

}  // namespace ord
