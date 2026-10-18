// OpenRoad Dynamics — engine-independent vehicle physics core.
// SPDX-License-Identifier: MIT
//
// Rideable two-wheelers: the SingleTrack multibody model (Tyre contact) plus
// what a game needs to ride it: a rider, a powertrain, brakes, a steering
// lock and optional assists. Bicycle, e-scooter and motorcycle definitions
// are provided.
//
// The rider is a controller acting only through what a real rider has:
// handlebar (steer) torque, throttle or pedal torque, two brake levers, and
// at walking pace a foot / body roll moment. It never changes grip, mass or
// any force model (same rule as the car assists, assists.hpp).
//
// Rider modes:
//   Lean    the player's steer input asks for a lean angle; the rider finds
//           the handlebar torque (including the counter-steer that starts
//           every turn) with a two-loop controller: an outer lean loop
//           chooses a steer angle from the point-mass lean dynamics
//             phi'' = (g/h) phi - v^2 / (h w) delta_ground   (small angles),
//           an inner loop tracks it with a stiff, damped arm torque.
//   Direct  the player's steer input IS the handlebar torque. Realistic and
//           hard: to turn right you push the right bar forward (steer left)
//           first. Use with mouse steering.
//
// Limitations (documented, not hidden): no suspension (frames are rigid,
// tyres give the compliance); the rider is rigid with the rear frame (no
// body lean / hang-off); no clutch or gearbox (power-limited wheel torque);
// collisions with the frame are not modelled: a crash is detected when the
// lean passes `fallLean` and the sim stops.
//
// References: Meijaard et al. 2007 (multibody model, single_track.hpp);
// R.S. Sharp, "The stability and control of motorcycles", J. Mech. Eng.
// Sci. 13(5), 1971; V. Cossalter, "Motorcycle Dynamics", 2nd ed., 2006
// (geometry ranges, rider torque levels, wheelie condition a > g b / h).
#pragma once

#include "ord/single_track.hpp"

namespace ord {

struct TwoWheelerPowertrain {
    Real maxWheelTorque = 60;  // N*m at the driven wheel
    Real maxPower = 350;       // W at the driven wheel (torque = min(T, P / omega))
    Real topSpeed = 0;         // m/s, drive fades out above it (0 = none; e-scooter limiter)
    bool frontDrive = false;   // hub motor in the front wheel
    Real dragArea = 0.5;       // m^2, CdA for aerodynamic drag
    Real frontBrakeTorque = 80;  // N*m at full lever
    Real rearBrakeTorque = 60;
};

enum class RiderMode { Lean, Direct };

struct RiderParams {
    Real maxLean = 0.8;          // rad, lean asked for at full steer input (at speed)
    Real maxSteerTorque = 20;    // N*m, strongest handlebar torque the rider gives
    Real steerStiffness = 60;    // N*m/rad, inner (arm) loop
    Real steerDamping = 4;       // N*m*s/rad
    Real leanFrequency = 3.0;    // rad/s, outer loop natural frequency
    Real leanDamping = 0.9;      // outer loop damping ratio
    Real armRelaxSpeed = 8.0;          // m/s, arm (inner loop) stiffness fades from here to twice this
    Real highSpeedArmFraction = 0.15;  // arm stiffness left at speed
    Real leanTorqueGain = 0;     // N*m/rad, direct lean-error torque (counter-steer) at speed
    Real leanRateTorqueGain = 0; // N*m*s/rad
    Real leanIntegralGain = 0;   // N*m/(rad*s)
    Real maxLeanRate = 1.2;      // rad/s, how fast the asked-for lean may change
    Real fallLean = 1.15;        // rad, crash detection
};

struct TwoWheelerAssists {
    RiderMode mode = RiderMode::Lean;
    // Low-speed balance: a foot on the ground / body-roll moment that holds
    // the bike up below walking pace, fading out between 1.5 and 4 m/s.
    // 0..1. With 0 a stopped bike falls over, as a real one does.
    Real balanceAssist = 1.0;
    bool abs = false;              // brake release on lock-up (either wheel)
    bool tractionControl = false;  // drive cut on rear wheel spin
    bool wheelieControl = false;   // drive cut when the front lifts; front brake eased on a stoppie
    // Pitch the wheelie control allows, rad. ~0.04 keeps the front down; a
    // larger value (e.g. 0.5) makes it a wheelie assist that holds the front
    // up at that angle while the throttle stays open.
    Real wheeliePitch = 0.04;
};

struct TwoWheelerDef {
    const char* name = "bicycle";
    SingleTrackDef frame;
    TwoWheelerPowertrain powertrain;
    RiderParams rider;
    Real steerLock = 0.7;  // rad, handlebar stop (benchmark sign, either way)
    // Hydraulic steering damper, N*m*s/rad (hardware, always active). Sport
    // motorcycles fit one against wobble (5-30 N*m*s/rad); the rigid model
    // here wobbles from ~20 m/s without it.
    Real steerDamper = 0;
};

TwoWheelerDef benchmarkBicycle();  // Meijaard benchmark + pedalling rider
TwoWheelerDef electricScooter();   // representative stand-up e-scooter (8.5" wheels, 25 km/h)
TwoWheelerDef sportMotorcycle();   // representative ~180 kg sport motorcycle + rider

struct TwoWheelerInput {
    Real steer = 0;       // -1..1, + = left (ISO, same as the car)
    Real throttle = 0;    // 0..1
    Real frontBrake = 0;  // 0..1
    Real rearBrake = 0;   // 0..1
};

struct TwoWheelerTelemetry {
    Real speed = 0;        // m/s
    Real lean = 0;         // rad, + right (benchmark sign)
    Real steer = 0;        // rad, + right
    Real pitch = 0;        // rad, + nose up (wheelie)
    Real leanTarget = 0;   // rad (Lean mode)
    Real steerTorque = 0;  // N*m, handlebar torque applied by the rider
    Real driveTorque = 0, frontBrake = 0, rearBrake = 0;  // N*m applied
    Real balanceTorque = 0;  // N*m, low-speed foot / body moment
    bool absActive = false, tcsActive = false, wheelieActive = false;
    bool fallen = false;
};

class TwoWheeler {
public:
    explicit TwoWheeler(const TwoWheelerDef& def, const TwoWheelerAssists& assists = {});

    // Upright at `speed`, rear contact at (x, y) on the ground, heading `yaw` (+ left).
    bool reset(Real x, Real y, Real yaw, Real speed, const GroundQuery* ground = nullptr);
    void step(Real dt, const TwoWheelerInput& in, const GroundQuery* ground = nullptr);

    void setAssists(const TwoWheelerAssists& a) { assists_ = a; }
    const TwoWheelerAssists& assists() const { return assists_; }
    const TwoWheelerDef& def() const { return def_; }
    const SingleTrack& model() const { return st_; }
    const TwoWheelerTelemetry& telemetry() const { return tel_; }
    bool fallen() const { return tel_.fallen; }
    // Ends the ride (host-detected crash, e.g. into a wall): the sim stops.
    void crash() { tel_.fallen = true; }

private:
    TwoWheelerDef def_;
    TwoWheelerAssists assists_;
    SingleTrack st_;
    TwoWheelerTelemetry tel_;
    Real comHeight_ = 1;   // m, whole-vehicle CoM height (upright)
    Real rollInertia_ = 1; // kg*m^2 about the ground line
    Real steerInertia_ = 0.1; // kg*m^2, front assembly about the steer axis
    Real frontStatic_ = 1;    // N, static front-wheel load
    Real rearStatic_ = 1;     // N, static rear-wheel load
    Real loadF_[2] = {1, 1};  // N, low-passed wheel loads (rear, front)
    Real absScale_[2] = {1, 1};
    Real driveScale_ = 1;
    Real tcsScale_ = 1;
    Real leanI_ = 0;  // N*m, integral of the lean error (Lean mode)
    Real leanTarget_ = 0;  // rad, slewed lean target
    Real prevPitch_ = 0, pitchRate_ = 0;
};

}  // namespace ord
