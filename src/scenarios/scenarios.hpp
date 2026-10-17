// OpenRoad Dynamics — reproducible benchmark scenarios.
// SPDX-License-Identifier: MIT
//
// Each scenario is a deterministic, scripted manoeuvre on an analytic test
// track, returning objective metrics. The same code drives the unit tests
// (with tolerances) and the `ordbench` report tool.
// Specification: docs/benchmarks/benchmark-plan.md
#pragma once

#include <iosfwd>
#include <string>
#include <vector>

#include "ord/assists.hpp"
#include "ord/lod_manager.hpp"
#include "ord/ground_cache.hpp"
#include "ord/tire.hpp"
#include "ord/vehicle.hpp"
#include "ord/vehicle_def.hpp"

namespace ord::scenarios {

// Ground-query accounting for one scenario run.
struct RunStats {
    std::uint64_t engineRays = 0;      // queries that reached the world (TestTrackGround)
    std::uint64_t vehicleQueries = 0;  // queries issued by the vehicle (direct or to the cache)
    std::uint64_t cacheMisses = 0;
    std::uint64_t roughStrips = 0;     // strips sampled densely
    std::uint64_t frames = 0;          // render frames emulated (prefetch mode)
    Real simulatedSeconds = 0;
};

struct RunConfig {
    VehicleDefinition vehicle;
    AssistConfig assists;
    VehicleFidelity fidelity = VehicleFidelity::lod0();
    Real hz = 360.0;
    SurfaceProperties surface;   // whole test area
    std::ostream* csv = nullptr; // optional per-step telemetry log
    // Ground prefetch emulation: rebuild a GroundPrefetchCache once per render
    // frame at `frameHz` and let the vehicle query only the cache.
    bool prefetch = false;
    Real frameHz = 60.0;
    GroundPrefetchCache::Settings prefetchSettings;
    RunStats* stats = nullptr;   // optional output
    // Host-engine emulation: when > 0 the vehicle runs in ChassisMode::External
    // and a separate host integrator (different scheme, like an engine's)
    // steps at hostHz; the vehicle substeps hz / hostHz times per host step.
    Real hostHz = 0;
};

struct AccelResult {
    bool reached = false;
    Real time0to100 = 0;      // s
    Real time0to60 = 0;       // s (km/h)
    Real peakDrivenSlip = 0;  // max driven-wheel slip ratio observed
    Real maxTcsCut = 0;
};
AccelResult accelerate0to100(const RunConfig& cfg);

struct BrakeResult {
    bool stopped = false;
    Real distance = 0;        // m from 100 km/h
    Real time = 0;            // s
    Real meanDecelG = 0;
    Real maxAbsSideslip = 0;  // rad, stability indicator
    Real absActiveFraction = 0;
    bool anyWheelLocked = false;
};
BrakeResult brake100to0(const RunConfig& cfg, Real startKmh = 100.0);

struct SkidpadResult {
    Real maxLateralG = 0;     // steady lateral acceleration while holding the radius
    Real speedAtMaxKmh = 0;
    Real radius = 0;
    Real sideslipAtMax = 0;   // rad
};
SkidpadResult skidpad(const RunConfig& cfg, Real radius = 40.0);

struct StepSteerResult {
    Real steadyYawRate = 0;   // rad/s
    Real yawRateRiseTime = 0; // s, 10% -> 90%
    Real overshoot = 0;       // fraction of steady value
    Real steadyLateralG = 0;
};
StepSteerResult stepSteer(const RunConfig& cfg, Real speedKmh = 80.0, Real steerInput = 0.08);

// Sine-with-dwell, after the US FMVSS No. 126 ESC test procedure (49 CFR
// 571.126): 0.7 Hz sine steer with a 500 ms dwell at the second peak, from
// 80 km/h with the throttle released. Pass criteria in the regulation: yaw
// rate 1.0 s after steering completion <= 35% of the first yaw-rate peak,
// and <= 20% at 1.75 s. Here the amplitude is given directly as a road-wheel
// angle instead of the regulation's steering-wheel-angle scaling.
struct SineDwellResult {
    Real peakYawRate = 0;       // rad/s, first peak (magnitude)
    Real ratioAt1s = 0;         // |yaw rate| / peak at 1.0 s after steer completion
    Real ratioAt175s = 0;       // at 1.75 s
    Real maxSideslip = 0;       // rad
    bool spun = false;          // |sideslip| exceeded 45 deg
    bool passesFmvss126Criteria() const { return !spun && ratioAt1s <= 0.35 && ratioAt175s <= 0.20; }
};
SineDwellResult sineWithDwell(const RunConfig& cfg, Real roadWheelAmplitude, Real speedKmh = 80.0);

struct HoldResult {
    Real drift = 0;           // m moved over the hold period
};
HoldResult slopeHold(const RunConfig& cfg, Real gradePercent, Real seconds, bool brakes);

struct BumpResult {
    Real peakCompression = 0; // m
    Real peakVerticalAccelG = 0;
    Real airborneTime = 0;    // s with any wheel off the ground (wheel hop)
    Real allAirborneTime = 0; // s with every wheel off the ground (car jumps)
    Real minWheelLoad = 0;    // N, lowest tire load seen after the bump starts
    Real maxWheelLoad = 0;    // N
    bool finite = true;
};
BumpResult speedBump(const RunConfig& cfg, Real speedKmh, Real bumpHeight = 0.08, Real bumpLength = 0.5);

struct DropResult {
    std::vector<Real> bouncePeaks;  // successive body-height peaks after landing
    bool energyIncreased = false;
    bool finite = true;
};
DropResult dropTest(const RunConfig& cfg, Real dropHeight = 0.25);

struct CoastResult {
    Real speedLossPerSecond = 0;  // km/h per s, with drag and rolling resistance removed
    bool speedEverIncreased = false;
};
CoastResult coastNoLosses(const RunConfig& cfg, Real startKmh = 100.0);

struct PerfResult {
    Real microsecondsPerStep = 0;
};
PerfResult measureStepCost(const RunConfig& cfg, int steps = 20000);

// ---- Contact & suspension fidelity (B19, B20, B22) ----

struct RoughRoadResult {
    bool finite = true;
    Real peakLoad = 0;          // N, max tire load on the affected wheels
    Real staticLoad = 0;        // N, affected wheels' load before the feature
    Real minLoad = 0;           // N
    Real peakFx = 0;            // N, max |longitudinal force| (impact)
    Real rmsVerticalAccelG = 0; // body, over the feature
    Real loadVariation = 0;     // std/mean of the affected tire loads over the feature
    Real wheelAirborneTime = 0; // s, affected wheels off the ground
    // Rate-independent measures (comparable across physics rates):
    Real maxHeave = 0;          // m, peak CoM rise over the feature
    Real rmsFrameAccelG = 0;    // body vertical accel averaged over 60 Hz windows
};
// B19: 8 cm curb (raised strip, 0.5 m wide, 2 cm edge ramps) under the left wheels.
RoughRoadResult curbStrike(const RunConfig& cfg, Real speedKmh, Real height = 0.08);
// B20a: rectangular pothole under the left wheels (2 cm edge ramps).
RoughRoadResult pothole(const RunConfig& cfg, Real speedKmh, Real depth = 0.06, Real length = 0.6);
// B20b: 25 m of cobblestones (paralelepipedo): 2 cm stones on a 0.15 m pitch, rough surface.
// textured = true: flat collision + procedural micro-profile with the same stone shape.
RoughRoadResult cobblestones(const RunConfig& cfg, Real speedKmh, bool textured = false);

struct FrequencyResult {
    Real heaveHz = 0, heaveExpectedHz = 0;
    Real wheelHopHz = 0, wheelHopExpectedHz = 0;
};
// ---- Fidelity tiers (B24, B25) ----

struct ScaleResult {
    int trafficAgents = 0;
    Real simulatedSeconds = 0;
    std::size_t maxLod1 = 0;
    std::uint64_t promotions = 0, demotions = 0;
    // Mean CPU milliseconds per 60 fps frame (single thread, this machine).
    Real lod0MsPerFrame = 0, lod1MsPerFrame = 0, lod2MsPerFrame = 0, totalMsPerFrame = 0;
    // Engine ground rays per frame.
    Real lod0RaysPerFrame = 0, lod1RaysPerFrame = 0, lod2RaysPerFrame = 0;
    Real maxLod1LateralError = 0;   // m, AI lane tracking quality
    bool finite = true;
};
// B25: 1 player car (LOD0, 60 Hz host x 4 substeps, ground cache) overtaking
// `agents` traffic cars on a 4-lane ring road with gentle hills.
ScaleResult trafficScale(int agents = 300, Real seconds = 20.0, std::size_t maxLod1 = 16);

struct TransitionResult {
    Real speedJumpPromote = 0;      // m/s, |LOD1 speed after - LOD2 speed before|
    Real yawRateJumpPromote = 0;    // rad/s
    Real maxVerticalAccelG = 0;     // first 1 s after promotion (no drop / pop)
    Real maxLateralError = 0;       // m from the lane, first 2 s
    Real speedJumpDemote = 0;       // m/s
    Real positionJumpDemote = 0;    // m between LOD1 CoM and LOD2 lane position
};
// B24: promote a cruising traffic car on a curve, drive 2 s, demote.
TransitionResult lodTransition(Real speedKmh = 70.0);

// B22: symmetric undamped car, small heave impulse; measured vs analytic quarter-car modes.
FrequencyResult suspensionFrequencies(const RunConfig& cfg);

// Drift hold: handbrake entry into a left-hand slide at `entryKmh`, then constant inputs (the
// steering held at `steerToward`, + = into the turn, and a fixed throttle) for `holdSeconds`.
// With the drift assist on, measures how well the requested drift angle is held.
struct DriftResult {
    bool entered = false;          // the rear broke loose (drift assist engaged, or sideslip > 15 deg)
    bool spun = false;             // sideslip beyond 80 deg or the car stopped
    Real meanSideslip = 0;         // rad, after settling (2 s), sign as ISO sideslip
    Real sdSideslip = 0;           // rad
    Real meanTarget = 0;           // rad, drift assist target over the same window
    Real maxThrottleAdded = 0;     // max(throttle out - driver throttle): must stay <= 0
    int upshiftsWhileSliding = 0;  // automatic gearbox upshifts with the rear sliding and power on
    Real endSpeedKmh = 0;
};
DriftResult driftHold(const RunConfig& cfg, Real steerToward, Real throttle, Real entryKmh = 50.0, Real holdSeconds = 10.0);

// ---- Golden baseline (benchmarks/baseline.csv) ----
// Key metrics that must not change silently. tests/test_baseline.cpp compares
// them with the committed file; `ordbench --write-baseline FILE` regenerates it
// (a deliberate, reviewable diff).
struct BaselineMetric {
    std::string name;
    Real value;
    Real relTolerance;  // relative tolerance used by the comparison test
};
std::vector<BaselineMetric> computeBaselineMetrics();

}  // namespace ord::scenarios
