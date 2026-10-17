// OpenRoad Dynamics — engine-independent vehicle physics core.
// SPDX-License-Identifier: MIT
//
// Driving presets are named AssistConfig bundles. They do not, and cannot,
// change the VehicleDefinition. See docs/architecture/driving-presets.md.
#pragma once

#include <string_view>

#include "ord/assists.hpp"

namespace ord {

// FUN / SPORT / SIMULATION are the main experiences. DRIFT is SPORT's steering with stability
// control off and the drift assist on. Every preset drives the same physical car.
enum class DrivingPreset { Fun, Sport, Simulation, Drift };

inline AssistConfig makePreset(DrivingPreset p, InputDevice device = InputDevice::Gamepad) {
    AssistConfig c;
    const bool wheel = device == InputDevice::SteeringWheel;
    switch (p) {
        case DrivingPreset::Fun:
            c.steeringMode = wheel ? SteeringMode::Direct : SteeringMode::SpeedSensitive;
            c.steeringExponent = wheel ? 1.0 : 1.4;
            c.steerRate = wheel ? 0.0 : 2.5;
            c.steerReturnRate = wheel ? 0.0 : 4.0;
            c.targetFrontSlipAngle = 0.10;
            c.throttleRate = 3.0;
            c.abs = true;
            c.tractionControl = InterventionLevel::Full;
            c.stabilityControl = InterventionLevel::Full;
            c.countersteerAssist = wheel ? 0.0 : 0.6;
            c.shiftMode = ShiftMode::Automatic;
            c.autoClutch = true;
            break;
        case DrivingPreset::Sport:
            c.steeringMode = wheel ? SteeringMode::Direct : SteeringMode::SpeedSensitive;
            c.steeringExponent = wheel ? 1.0 : 1.2;
            c.steerRate = wheel ? 0.0 : 4.0;
            c.steerReturnRate = wheel ? 0.0 : 6.0;
            c.targetFrontSlipAngle = 0.14;
            c.throttleRate = 6.0;
            c.abs = true;
            c.tractionControl = InterventionLevel::Sport;
            c.stabilityControl = InterventionLevel::Off;
            c.countersteerAssist = wheel ? 0.0 : 0.25;
            c.shiftMode = ShiftMode::Automatic;
            c.autoClutch = true;
            break;
        case DrivingPreset::Drift:
            c.steeringMode = wheel ? SteeringMode::Direct : SteeringMode::SpeedSensitive;
            c.steeringExponent = wheel ? 1.0 : 1.2;
            c.steerRate = wheel ? 0.0 : 5.0;
            c.steerReturnRate = wheel ? 0.0 : 8.0;
            c.targetFrontSlipAngle = 0.14;
            c.throttleRate = 6.0;
            c.abs = true;
            c.tractionControl = InterventionLevel::Off;
            c.stabilityControl = InterventionLevel::Off;
            c.countersteerAssist = 0.0;  // the drift assist does the countersteering
            c.driftAssist = 1.0;
            c.shiftMode = ShiftMode::Automatic;
            c.autoClutch = true;
            break;
        case DrivingPreset::Simulation:
            c.steeringMode = SteeringMode::Direct;
            c.steeringExponent = 1.0;
            c.steerRate = 0.0;
            c.throttleRate = 0.0;
            c.abs = false;
            c.tractionControl = InterventionLevel::Off;
            c.stabilityControl = InterventionLevel::Off;
            c.countersteerAssist = 0.0;
            c.shiftMode = ShiftMode::Manual;
            c.autoClutch = true;  // manual clutch is a separate, explicit opt-in
            break;
    }
    return c;
}

inline std::string_view presetName(DrivingPreset p) {
    switch (p) {
        case DrivingPreset::Fun: return "FUN";
        case DrivingPreset::Sport: return "SPORT";
        case DrivingPreset::Simulation: return "SIMULATION";
        case DrivingPreset::Drift: return "DRIFT";
    }
    return "?";
}

}  // namespace ord
