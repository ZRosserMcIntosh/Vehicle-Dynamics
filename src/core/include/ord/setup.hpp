// OpenRoad Dynamics — engine-independent vehicle physics core.
// SPDX-License-Identifier: MIT
//
// Garage setup: everything a tuning menu adjusts, in the units a setup
// sheet uses (degrees, N/mm, mm, bar, percent), applied to a vehicle
// definition. Same physics before and after: a setup only changes the parts.
//
//   VehicleSetup s = stockSetup(def);       // what the car comes with
//   s.front.camberDeg = -2.5;               // edit
//   VehicleDefinition tuned = applySetup(def, s);
//   std::string json = setupToJson(s);      // save / share; setupFromJson to load
//
// Tyre compounds and pressure change TireParams (grip, stiffness, how grip
// falls off, temperature window, response to wet and loose surfaces); see
// applyCompound / applyPressure for the values and what they stand for.
#pragma once

#include <array>
#include <string>

#include "ord/vehicle_def.hpp"

namespace ord {

enum class TyreCompound { Stock = 0, OffRoad, AllSeason, Street, Sport, SemiSlick, Slick, Rain };
constexpr int kTyreCompounds = 8;
const char* compoundName(TyreCompound c);

// Turns the tyre's stock (road) parameters into `c`, keeping its size and
// load rating. Enables the camber and thermal models. Stock = unchanged.
void applyCompound(TireParams& t, TyreCompound c);
// Inflation pressure (bar) relative to `nominalBar`: higher = stiffer
// carcass (more cornering stiffness, vertical rate, shorter relaxation),
// smaller patch (peak grip falls off either side of nominal).
void applyPressure(TireParams& t, Real pressureBar, Real nominalBar = 2.2);
// The camber model for car tyres (best grip leaning ~2 deg into the turn).
void enableCarCamberModel(TireParams& t);

struct AxleSetup {
    // Alignment
    Real camberDeg = 0;        // negative = top of the wheel toward the car
    Real toeDeg = 0;           // + toe-in (per wheel)
    Real casterDeg = 0;        // steered axle only
    Real camberGainDegPerCm = 0;  // camber change per cm of bump (negative = more negative camber in bump)
    // Springs and dampers
    Real springNmm = 35;       // N/mm at the wheel
    Real rideHeightMm = 0;     // offset from stock (+ = higher)
    Real bumpSlow = 3000, bumpFast = 0;        // N*s/m (fast 0 = same as slow)
    Real reboundSlow = 4500, reboundFast = 0;  // N*s/m
    Real arbNmm = 0;           // anti-roll bar, N/mm at the wheel
    // Tyres
    TyreCompound compound = TyreCompound::Stock;
    Real pressureBar = 2.2;
};

struct VehicleSetup {
    AxleSetup front, rear;
    Real brakeBalance = 0.6;     // share of brake torque on the front axle
    Real brakePressure = 1.0;    // x stock total brake torque
    Real steerLockDeg = 31;
    // Drivetrain (rear-most driven axle's differential)
    int diffType = -1;           // -1 stock, 0 open, 1 locked, 2 clutch LSD
    Real diffPreload = 80, diffPower = 0.45, diffCoast = 0.25;
    Real finalDrive = 3.9;
    std::array<Real, 8> gears{};  // forward ratios (count = gearCount)
    int gearCount = 0;
    // Aero
    Real frontAeroScale = 1.0;   // x stock front downforce (splitter / canards)
    Real rearAeroScale = 1.0;    // x stock rear body downforce
    bool rearWing = false;       // fit a rear wing (1.4 m x 0.28 m) if the car has none
    Real wingAngleDeg = 8;       // its angle of attack toward downforce
    // Environment
    Real altitudeM = 0;          // sets air density with temperatureC (ISA pressure) ...
    Real temperatureC = 15;
    // ... and, for a naturally aspirated engine, its power (SAE J1349 factor
    // relative to the sea-level, 15 C conditions the stock curve is quoted at).
    bool turbocharged = false;   // a turbo holds its power at altitude (ignores the correction)
};

VehicleSetup stockSetup(const VehicleDefinition& d);
VehicleDefinition applySetup(const VehicleDefinition& stock, const VehicleSetup& s);

std::string setupToJson(const VehicleSetup& s);
// Reads the fields present (others keep their value in `out`). Returns an
// empty string on success, else what was wrong.
std::string setupFromJson(const std::string& json, VehicleSetup& out);

}  // namespace ord
