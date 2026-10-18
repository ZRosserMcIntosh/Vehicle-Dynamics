// OpenRoad Dynamics — engine-independent vehicle physics core.
// SPDX-License-Identifier: MIT
//
// Drivetrain + wheel-spin solver.
//
// The engine, clutch, gearbox, differentials, brakes and the longitudinal tire
// slip states are solved together as one small, linearly implicit system each
// step (docs/architecture/drivetrain-solver.md):
//
//   * Shafts are angular-velocity degrees of freedom (engine, gearbox input,
//     one per wheel).
//   * Gear meshes and open differentials are *equality rows* J*omega = 0. An
//     open differential's equal torque split falls out of the constraint
//     multiplier automatically.
//   * Clutch, brakes, limited-slip and locked differentials are *friction rows*
//     with a torque capacity, resolved by a small active-set iteration
//     (stick/slip). This gives exact standstill holding and lock-up without the
//     chatter of explicit Coulomb friction.
//   * Each wheel's longitudinal transient slip (relaxation length) is an extra
//     unknown, and the tire force is linearised around the current slip, so the
//     stiff wheel/tire coupling is integrated implicitly and remains stable at
//     120 Hz and at very low speed.
//
// Published references: Pacejka (2012) ch. 7 for transient slip; the
// stick/slip active-set formulation is standard constrained multibody practice
// (e.g. D. Baraff, "Linear-time dynamics using Lagrange multipliers", 1996).
#pragma once

#include <array>
#include <cstddef>
#include <vector>

#include "ord/math.hpp"

namespace ord {

inline constexpr std::size_t kMaxWheels = 8;
inline constexpr std::size_t kMaxAxles = kMaxWheels / 2;

struct TorqueCurvePoint {
    Real rpm;
    Real torque;  // N*m, net wide-open-throttle torque at the crank
};

struct EngineDef {
    std::vector<TorqueCurvePoint> torqueCurve;  // sorted by rpm
    Real idleRpm = 900;
    Real limiterRpm = 7000;   // fuel cut above this
    Real inertia = 0.20;      // kg*m^2, crank + flywheel
    // Internal friction / pumping losses (engine braking): T = a + b*omega [N*m]
    Real frictionTorque = 18.0;
    Real frictionPerRadS = 0.035;
};

enum class DifferentialType { Open, Locked, ClutchLsd };

struct DifferentialDef {
    DifferentialType type = DifferentialType::Open;
    Real finalDrive = 3.9;      // used for axle differentials only
    // ClutchLsd: locking torque capacity = preload + ramp * |input torque|
    Real preload = 80.0;        // N*m
    Real powerRamp = 0.45;      // on drive (acceleration)
    Real coastRamp = 0.25;      // on overrun (engine braking)
};

enum class TransmissionKind { Automatic, Sequential, DualClutch, ManualHPattern };

struct GearboxDef {
    std::vector<Real> forwardRatios{3.6, 2.2, 1.5, 1.15, 0.92, 0.76};
    Real reverseRatio = 3.4;
    Real inputInertia = 0.03;   // kg*m^2, clutch disc + gearbox input
    Real efficiency = 0.93;     // applied to positive drive torque
    Real shiftTime = 0.25;      // s, torque interruption for a shift
    TransmissionKind kind = TransmissionKind::Automatic;
};

struct ClutchDef {
    Real maxTorque = 600.0;     // N*m capacity when fully engaged
};

struct DrivenAxle {
    std::size_t axleIndex = 0;  // index into the vehicle's axle list
    Real torqueShare = 1.0;     // open-centre split; shares must sum to 1
    DifferentialDef differential;
};

enum class CenterCoupling { Open, Locked, ClutchLsd };

struct DrivetrainDef {
    EngineDef engine;
    ClutchDef clutch;
    GearboxDef gearbox;
    std::vector<DrivenAxle> drivenAxles;
    // Only meaningful with exactly two driven axles.
    CenterCoupling center = CenterCoupling::Open;
    Real centerPreload = 100.0, centerRamp = 0.3;
};

// Per-wheel quantities for one solve, computed by the vehicle from the
// contact state at the start of the step.
struct WheelSolveInput {
    Real radius = 0.33;
    Real inertia = 1.2;
    Real contactVx = 0;       // contact-point longitudinal ground velocity, wheel frame
    Real relaxationLength = 0.15;
    Real kappa0 = 0;          // transient slip at step start
    Real fx0 = 0;             // tire force at kappa0
    Real dFxdKappa = 0;       // >= 0 slope used for linearisation
    Real slipDamping = 0;     // N*s/m, low-speed carcass damping on slip velocity
    Real externalTorque = 0;  // N*m, e.g. rolling resistance
    Real brakeCapacity = 0;   // N*m, total friction-brake torque capacity
};

struct WheelSolveOutput {
    Real omega = 0;
    Real kappa = 0;
    Real fx = 0;              // longitudinal force on the chassis (consistent with the solve)
    bool brakeLocked = false;
};

struct DrivetrainCommand {
    Real throttle = 0;        // 0..1, after assists (TCS may reduce it)
    Real clutchEngagement = 1;  // 0..1, after auto-clutch logic
    int gear = 1;             // -1 reverse, 0 neutral, 1..N forward
};

struct DrivetrainTelemetry {
    Real engineRpm = 0;
    Real engineTorque = 0;    // N*m, net crank torque produced this step
    Real clutchSlip = 0;      // rad/s
    bool clutchLocked = false;
    Real driveshaftTorque = 0;  // N*m at gearbox input (constraint multiplier)
    int activeSolverIterations = 0;
    bool solveFailed = false;
};

class Drivetrain {
public:
    Drivetrain() = default;
    Drivetrain(const DrivetrainDef& def, std::size_t wheelCount, const std::array<std::array<int, 2>, kMaxAxles>& axleWheels);

    // Solves one fixed step. `wheels`/`out` have one entry per wheel.
    void step(Real dt, const DrivetrainCommand& cmd, const WheelSolveInput* wheels, WheelSolveOutput* out);

    Real engineOmega() const { return omega_[0]; }
    Real wheelOmega(std::size_t w) const { return omega_[2 + w]; }
    void setWheelOmega(std::size_t w, Real v) { omega_[2 + w] = v; }
    void setEngineOmega(Real v) { omega_[0] = omega_[1] = v; }
    // Engine speed implied by the wheels through the given gear (for shift logic).
    Real engineOmegaForGear(int gear) const;
    Real gearRatio(int gear) const;
    int forwardGearCount() const { return static_cast<int>(def_.gearbox.forwardRatios.size()); }
    const DrivetrainDef& def() const { return def_; }
    void setDifferential(std::size_t drivenAxle, const DifferentialDef& d) { def_.drivenAxles[drivenAxle].differential = d; }
    const DrivetrainTelemetry& telemetry() const { return telem_; }
    Real wotTorque(Real rpm) const;

private:
    enum class FrictionKind { Clutch, Brake, AxleDiff, CenterDiff };
    struct FrictionRow {
        FrictionKind kind;
        std::size_t index;              // wheel, axle, ...
        std::array<Real, 2 + kMaxWheels> j{};
        Real capacity = 0;
        bool locked = true;
        Real slipForce = 0;             // applied generalized force when slipping
        Real lambda = 0;                // last multiplier (locked) or applied force
    };

    Real engineTorque(Real omega, Real throttle) const;

    DrivetrainDef def_;
    std::size_t wheelCount_ = 0;
    std::array<std::array<int, 2>, kMaxAxles> axleWheels_{};
    std::array<Real, 2 + kMaxWheels> omega_{};
    std::array<FrictionRow, 2 + kMaxWheels + kMaxAxles> friction_{};
    std::size_t frictionCount_ = 0;
    Real lastInputTorque_ = 0;
    bool limiterCut_ = false;
    DrivetrainTelemetry telem_;
};

}  // namespace ord
