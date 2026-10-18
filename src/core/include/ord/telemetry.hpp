// OpenRoad Dynamics — engine-independent vehicle physics core.
// SPDX-License-Identifier: MIT
//
// Per-step telemetry. Every physical quantity the benchmarks assert on is
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
    Real steerAngle = 0;     // rad, this wheel
    Real brakeTorque = 0;    // N*m commanded
    bool brakeLocked = false;
    int surfaceId = 0;
};

struct TelemetryFrame {
    Real time = 0;
    Vec3 position;
    Real speed = 0;              // m/s, longitudinal (body)
    Real lateralVelocity = 0;    // m/s (body)
    Real sideslip = 0;           // rad, atan2(vy, vx)
    Real yawRate = 0;            // rad/s
    Real rollAngle = 0, pitchAngle = 0;  // rad (small-angle extraction)
    Vec3 accelBody;              // m/s^2, kinematic acceleration of the CoM, body frame
    Real kineticEnergy = 0;      // J
    // Driver vs. actuator
    Real steerInput = 0, throttleInput = 0, brakeInput = 0;
    Real steerAngle = 0, throttleOut = 0;
    int gear = 0;
    Real clutchEngagement = 0;
    DrivetrainTelemetry drivetrain;
    AssistReport assists;
    std::uint64_t groundQueries = 0;  // cumulative GroundQuery calls by this vehicle
    std::size_t wheelCount = 0;
    std::array<WheelTelemetry, kMaxWheels> wheels{};
};

void writeCsvHeader(std::ostream& os, std::size_t wheelCount);
void writeCsvRow(std::ostream& os, const TelemetryFrame& f);

}  // namespace ord
