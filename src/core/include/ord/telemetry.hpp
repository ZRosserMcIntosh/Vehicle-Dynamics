// OpenRoad Dynamics — engine-independent vehicle physics core.
// SPDX-License-Identifier: MIT
//
// Per-step telemetry. Time levels: position, velocities, attitude and the
// accelerations are the state at the END of the step; wheel loads, slips,
// forces, compressions and the assists' commands are the ones that acted
// DURING it (and produced that state).
// Every physical quantity the benchmarks assert on is
// exported here so tests, tools and the Unreal debug overlay share one source.
// Channel definitions: docs/benchmarks/telemetry-channels.md
#pragma once

#include <array>
#include <cstdint>
#include <ostream>

#include "ord/assists.hpp"
#include "ord/drivetrain.hpp"
#include "ord/math.hpp"

namespace ord {

struct WheelTelemetry {
    bool contact = false;
    Real compression = 0;    // m, 0 = full droop
    Real compressionRate = 0;  // m/s
    Real load = 0;           // N, Fz
    Real omega = 0;          // rad/s
    Real slipRatio = 0;      // transient kappa
    Real slipAngle = 0;      // rad, transient
    Real fx = 0, fy = 0;     // N, tire frame
    Real gripUsed = 0;       // |(fx / (muX Fz), fy / (muY Fz))|: 0 = no force, ~1 = at the limit
    Real camber = 0;         // rad, to the road, usual sign (negative = top leaning toward the car)
    Real temperature = 0;    // deg C, tread (thermal model; ambient otherwise)
    Real steerAngle = 0;     // rad, this wheel
    Real brakeTorque = 0;    // N*m commanded
    bool brakeLocked = false;
    int surfaceId = 0;
};

struct AeroTelemetry {
    Real airspeed = 0;          // m/s, relative to the air (wind included)
    Real yaw = 0;               // rad, airflow yaw angle
    Real downforceFront = 0;    // N (body/floor at the front axle)
    Real downforceRear = 0;     // N (body/floor at the rear axle + wing)
    Real drag = 0;              // N
    Real wingDownforce = 0;     // N
    Real clearanceFront = 0, clearanceRear = 0;  // m, floor to ground (when modelled)
    bool wingStalled = false, floorStalled = false;
};

struct TelemetryFrame {
    Real time = 0;
    Vec3 position;
    Real speed = 0;              // m/s, longitudinal (body)
    Real lateralVelocity = 0;    // m/s (body)
    Real sideslip = 0;           // rad, atan2(vy, vx)
    Real yawRate = 0;            // rad/s
    Real rollAngle = 0, pitchAngle = 0;  // rad (small-angle extraction)
    Real upright = 1;            // body up-axis z: 1 upright, 0 on its side, -1 on its roof
    int bodyContacts = 0;        // body hull points touching the ground (rollover, roof, side)
    Real bodyContactForce = 0;   // N, total normal force on the body hull
    Vec3 accelBody;              // m/s^2, kinematic acceleration of the CoM, body frame
    Real kineticEnergy = 0;      // J
    // Driver vs. actuator
    Real steerInput = 0, throttleInput = 0, brakeInput = 0;
    Real steerAngle = 0, throttleOut = 0;
    int gear = 0;
    Real clutchEngagement = 0;
    DrivetrainTelemetry drivetrain;
    AeroTelemetry aero;
    AssistReport assists;
    std::uint64_t groundQueries = 0;  // cumulative GroundQuery calls by this vehicle
    std::size_t wheelCount = 0;
    std::array<WheelTelemetry, kMaxWheels> wheels{};
    // Front-view roll-centre height above the ground per axle, from the
    // linkage's path and camber rate at the current travel (0 = at the ground).
    std::array<Real, kMaxAxles> rollCentreHeight{};
};

void writeCsvHeader(std::ostream& os, std::size_t wheelCount);
void writeCsvRow(std::ostream& os, const TelemetryFrame& f);

}  // namespace ord
