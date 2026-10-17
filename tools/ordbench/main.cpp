// OpenRoad Dynamics — benchmark report tool.
// SPDX-License-Identifier: MIT
//
// Usage: ordbench [--csv-dir DIR] [--markdown]
// Runs the benchmark suite for every reference vehicle and preset and prints
// a Markdown report (the source of docs/benchmarks/baseline-results.md).
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

#include "ord/presets.hpp"
#include "reference_vehicles.hpp"
#include "scenarios.hpp"

using namespace ord;
using namespace ord::scenarios;

namespace {

struct Car {
    const char* id;
    VehicleDefinition def;
};

void printVehicleSection(const Car& car) {
    std::cout << "\n## " << car.def.name << "\n\n";
    std::cout << "| Preset | 0-100 km/h (s) | 100-0 km/h (m) | Brake mean decel (g) | Skidpad R40 max lat (g) | Step steer 80 km/h yaw rise (s) | Peak driven slip | Max TCS cut |\n";
    std::cout << "|---|---|---|---|---|---|---|---|\n";
    for (DrivingPreset p : {DrivingPreset::Fun, DrivingPreset::Sport, DrivingPreset::Simulation}) {
        RunConfig cfg;
        cfg.vehicle = car.def;
        cfg.assists = makePreset(p);
        const AccelResult a = accelerate0to100(cfg);
        const BrakeResult b = brake100to0(cfg);
        const SkidpadResult s = skidpad(cfg);
        const StepSteerResult st = stepSteer(cfg);
        std::cout << std::fixed << std::setprecision(2) << "| " << presetName(p) << " | "
                  << (a.reached ? std::to_string(a.time0to100).substr(0, 5) : std::string("n/a")) << " | "
                  << b.distance << " | " << b.meanDecelG << " | " << s.maxLateralG << " | " << st.yawRateRiseTime
                  << " | " << a.peakDrivenSlip << " | " << a.maxTcsCut << " |\n";
    }
}

void printWet(const Car& car) {
    std::cout << "\n### Wet vs dry braking (" << car.def.name << ", ABS on vs off)\n\n";
    std::cout << "| Surface | ABS | 100-0 (m) | Mean decel (g) | Max sideslip (deg) | ABS active % |\n|---|---|---|---|---|---|\n";
    for (int wet = 0; wet < 2; ++wet) {
        for (int abs = 1; abs >= 0; --abs) {
            RunConfig cfg;
            cfg.vehicle = car.def;
            cfg.assists = makePreset(DrivingPreset::Simulation);
            cfg.assists.abs = abs != 0;
            if (wet) {
                cfg.surface.frictionScale = 0.75;
                cfg.surface.wetSpeedFalloff = 0.004;
            }
            const BrakeResult b = brake100to0(cfg);
            std::cout << std::fixed << std::setprecision(2) << "| " << (wet ? "wet (placeholder model)" : "dry") << " | "
                      << (abs ? "on" : "off") << " | " << b.distance << " | " << b.meanDecelG << " | "
                      << b.maxAbsSideslip * 180 / kPi << " | " << b.absActiveFraction * 100 << " |\n";
        }
    }
}

void printTimestep(const Car& car) {
    std::cout << "\n### Timestep sensitivity (" << car.def.name << ", SIMULATION preset)\n\n";
    std::cout << "| Rate (Hz) | 0-100 (s) | 100-0 (m) | Skidpad (g) | Step-steer yaw (rad/s) | Cost (us/step) |\n|---|---|---|---|---|---|\n";
    for (Real hz : {60.0, 120.0, 240.0, 360.0, 480.0, 1000.0}) {
        RunConfig cfg;
        cfg.vehicle = car.def;
        cfg.assists = makePreset(DrivingPreset::Simulation);
        cfg.hz = hz;
        const AccelResult a = accelerate0to100(cfg);
        const BrakeResult b = brake100to0(cfg);
        const SkidpadResult s = skidpad(cfg);
        const StepSteerResult st = stepSteer(cfg);
        const PerfResult perf = measureStepCost(cfg);
        std::cout << std::fixed << std::setprecision(3) << "| " << hz << " | " << a.time0to100 << " | " << b.distance
                  << " | " << s.maxLateralG << " | " << st.steadyYawRate << " | " << perf.microsecondsPerStep << " |\n";
    }
}

void printSineDwell(const Car& car) {
    std::cout << "\n### Sine-with-dwell, FMVSS 126-style (" << car.def.name << ", 80 km/h)\n\n";
    std::cout << "| Road-wheel amplitude (deg) | ESC | Peak yaw (rad/s) | Ratio @1.0 s | Ratio @1.75 s | Max sideslip (deg) | Meets 126 ratios |\n|---|---|---|---|---|---|---|\n";
    for (Real ampDeg : {3.0, 5.0, 7.0, 9.0}) {
        for (InterventionLevel esc : {InterventionLevel::Off, InterventionLevel::Full}) {
            RunConfig cfg;
            cfg.vehicle = car.def;
            cfg.assists = makePreset(DrivingPreset::Simulation);
            cfg.assists.stabilityControl = esc;
            const SineDwellResult r = sineWithDwell(cfg, ampDeg * kPi / 180);
            std::cout << std::fixed << std::setprecision(2) << "| " << ampDeg << " | "
                      << (esc == InterventionLevel::Off ? "off" : "full") << " | " << r.peakYawRate << " | "
                      << r.ratioAt1s << " | " << r.ratioAt175s << " | " << r.maxSideslip * 180 / kPi << " | "
                      << (r.passesFmvss126Criteria() ? "yes" : "NO") << " |\n";
        }
    }
}

void printDrift() {
    std::cout << "\n### Drift assist (DRIFT preset, steady inputs after a handbrake entry at 50 km/h)\n\n";
    std::cout << "Behaviour of an aid with no real-car equivalent: steering held at a fixed position (into the turn = +) "
                 "and fixed throttle for 10 s. Sideslip averaged after 2 s; a held drift has mean = target and a small SD.\n\n";
    std::cout << "| Car | Steering | Throttle | Target (deg) | Mean sideslip (deg) | SD (deg) | Spun | End speed (km/h) |\n"
                 "|---|---|---|---|---|---|---|---|\n";
    for (const VehicleDefinition& d : {vehicles::referenceSportsRwd(), vehicles::rosserStellaV8()}) {
        RunConfig cfg;
        cfg.vehicle = d;
        cfg.assists = makePreset(DrivingPreset::Drift);
        for (Real steer : {0.0, 0.5, 1.0})
            for (Real thr : {0.6, 1.0}) {
                const DriftResult r = driftHold(cfg, steer, thr);
                std::cout << std::fixed << std::setprecision(1) << "| " << d.name << " | " << steer << " | " << thr << " | "
                          << r.meanTarget * 180 / kPi << " | " << r.meanSideslip * 180 / kPi << " | "
                          << r.sdSideslip * 180 / kPi << " | " << (r.spun ? "yes" : "no") << " | " << r.endSpeedKmh << " |\n";
            }
    }
}

void printContact(const Car& car) {
    RunConfig base;
    base.vehicle = car.def;
    base.assists = makePreset(DrivingPreset::Simulation);
    const FrequencyResult fr = suspensionFrequencies(base);
    std::cout << "\n### Suspension modes, unsprung-mass DOF (" << car.def.name << ")\n\n";
    std::cout << "| Mode | Measured (Hz) | Analytic quarter car (Hz) |\n|---|---|---|\n" << std::fixed << std::setprecision(3)
              << "| Body heave | " << fr.heaveHz << " | " << fr.heaveExpectedHz << " |\n"
              << "| Wheel hop | " << fr.wheelHopHz << " | " << fr.wheelHopExpectedHz << " |\n";

    std::cout << "\n### Rough roads: rate x contact samples (" << car.def.name << ", SIMULATION)\n\n";
    std::cout << "Rate-fair metrics: peak CoM heave (m) and body vertical acceleration averaged over 60 Hz windows (g, what a\n"
                 "60 fps camera/player perceives). Cobblestones (texture): RMS per-step body acceleration (g). Rays: engine ground\n"
                 "queries per simulated second (direct = every contact sample and envelope probe, every step).\n\n";
    std::cout << "| Configuration | Curb 8 cm @120 heave | Curb @120 a60 | Curb 8 cm @60 a60 | Pothole 6 cm @60 a60 | Cobbles (texture) @40 | Rays/s (curb) | Rays/s (texture cobbles) |\n|---|---|---|---|---|---|---|---|\n";
    struct Cfg { const char* name; Real hz, hostHz; int samples; bool prefetch; };
    const Cfg cfgs[] = {
        {"cached 2000 Hz, 5 samples (reference)", 2000, 0, 5, true},
        {"direct 240 Hz, 1 sample (no patch)", 240, 0, 1, false},
        {"direct 240 Hz, 5 samples", 240, 0, 5, false},
        {"cached 240 Hz, 5 samples", 240, 0, 5, true},
        {"host 60 Hz x4 substeps, 5 samples, ground cache", 240, 60, 5, true},
        {"host 60 Hz x6 substeps, 8 samples, ground cache", 360, 60, 8, true},
    };
    for (const Cfg& c : cfgs) {
        RunConfig r = base;
        r.hz = c.hz;
        r.hostHz = c.hostHz;
        r.fidelity.contactSamples = c.samples;
        r.prefetch = c.prefetch;
        RunStats sc, st;
        r.stats = &sc;
        const RoughRoadResult curb120 = curbStrike(r, 120);
        r.stats = nullptr;
        const RoughRoadResult curb60 = curbStrike(r, 60);
        const RoughRoadResult hole = pothole(r, 60);
        r.stats = &st;
        const RoughRoadResult tex = cobblestones(r, 40, true);
        std::cout << std::setprecision(4) << "| " << c.name << " | " << curb120.maxHeave << " | " << std::setprecision(3)
                  << curb120.rmsFrameAccelG << " | " << curb60.rmsFrameAccelG << " | " << hole.rmsFrameAccelG << " | "
                  << tex.rmsVerticalAccelG << " | " << std::setprecision(0) << static_cast<Real>(sc.engineRays) / sc.simulatedSeconds
                  << " | " << static_cast<Real>(st.engineRays) / st.simulatedSeconds << " |\n";
    }
}

void printLod() {
    const TransitionResult t = lodTransition(70.0);
    std::cout << "\n### Fidelity tiers\n\n";
    std::cout << "LOD2 -> LOD1 -> LOD2 transition at 70 km/h on a 250 m curve:\n\n";
    std::cout << "| Metric | Value |\n|---|---|\n" << std::fixed << std::setprecision(3)
              << "| Speed jump on promotion (m/s) | " << t.speedJumpPromote << " |\n"
              << "| Yaw-rate jump on promotion (rad/s) | " << t.yawRateJumpPromote << " |\n"
              << "| Max vertical accel, first 1 s (g) | " << t.maxVerticalAccelG << " |\n"
              << "| Max lane error, 2 s (m) | " << t.maxLateralError << " |\n"
              << "| Speed jump on demotion (m/s) | " << t.speedJumpDemote << " |\n"
              << "| Position jump on demotion (m) | " << t.positionJumpDemote << " |\n";
    std::cout << "\nScale: 1 player (LOD0: 60 Hz host x4, ground cache) + up to 16 LOD1 + N LOD2, ring road, 20 s, single thread:\n\n";
    std::cout << "| LOD2 agents | Max LOD1 | Promotions | LOD0 ms/frame | LOD1 ms/frame | LOD2 ms/frame | Total ms/frame | Rays/frame LOD0 / LOD1 / LOD2 |\n|---|---|---|---|---|---|---|---|\n";
    for (int n : {100, 300, 1000}) {
        const ScaleResult r = trafficScale(n, 20.0, 16);
        std::cout << std::setprecision(3) << "| " << n << " | " << r.maxLod1 << " | " << r.promotions << " | "
                  << r.lod0MsPerFrame << " | " << r.lod1MsPerFrame << " | " << r.lod2MsPerFrame << " | "
                  << r.totalMsPerFrame << " | " << std::setprecision(1) << r.lod0RaysPerFrame << " / "
                  << r.lod1RaysPerFrame << " / " << r.lod2RaysPerFrame << " |\n";
    }
}

void printStability(const Car& car) {
    RunConfig cfg;
    cfg.vehicle = car.def;
    cfg.assists = makePreset(DrivingPreset::Simulation);
    const DropResult d = dropTest(cfg);
    const CoastResult c = coastNoLosses(cfg);
    const BumpResult bump = speedBump(cfg, 40.0);
    const HoldResult hold = slopeHold(cfg, 20.0, 10.0, true);
    const HoldResult roll = slopeHold(cfg, 20.0, 3.0, false);
    std::cout << "\n### Stability & conservation (" << car.def.name << ")\n\n";
    std::cout << "| Check | Result |\n|---|---|\n";
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "| Drop 0.25 m: energy ever increased | " << (d.energyIncreased ? "YES" : "no") << " |\n";
    std::cout << "| Drop 0.25 m: bounce peaks (m) | ";
    for (std::size_t i = 0; i < std::min<std::size_t>(d.bouncePeaks.size(), 4); ++i) std::cout << d.bouncePeaks[i] << " ";
    std::cout << "|\n";
    std::cout << "| Coast 100 km/h, no drag/rolling: loss (km/h per s) | " << c.speedLossPerSecond << " |\n";
    std::cout << "| Coast: speed ever increased | " << (c.speedEverIncreased ? "YES" : "no") << " |\n";
    std::cout << "| Speed bump 8 cm @ 40 km/h: peak compression (m) | " << bump.peakCompression << " |\n";
    std::cout << "| Speed bump: peak vertical accel (g) | " << bump.peakVerticalAccelG << " |\n";
    std::cout << "| Speed bump: airborne time (s) | " << bump.airborneTime << " |\n";
    std::cout << "| 20% grade, brakes held 10 s: drift (m) | " << hold.drift << " |\n";
    std::cout << "| 20% grade, neutral, no brakes 3 s: roll-back (m) | " << roll.drift << " |\n";
}

}  // namespace

int main(int argc, char** argv) {
    std::string csvDir;
    std::string only;
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--csv-dir") && i + 1 < argc) csvDir = argv[++i];
        else if (!std::strcmp(argv[i], "--only") && i + 1 < argc) only = argv[++i];
        else if (!std::strcmp(argv[i], "--write-baseline") && i + 1 < argc) {
            std::ofstream out(argv[++i]);
            out << "# OpenRoad Dynamics golden baseline. Regenerate ONLY deliberately with:\n"
                   "#   ./build/ordbench --write-baseline benchmarks/baseline.csv\n"
                   "# and explain every changed value in the pull request.\n"
                   "metric,value,rel_tolerance\n";
            out << std::setprecision(10);
            for (const auto& m : computeBaselineMetrics()) out << m.name << ',' << m.value << ',' << m.relTolerance << '\n';
            std::cout << "baseline written\n";
            return 0;
        }
    }
    const Car cars[] = {{"sports", vehicles::referenceSportsRwd()},
                        {"hatch", vehicles::referenceHatchFwd()},
                        {"awd", vehicles::referenceSaloonAwd()}};

    if (!csvDir.empty()) {
        // Telemetry traces for plotting: one file per scenario for the sports car.
        RunConfig cfg;
        cfg.vehicle = cars[0].def;
        cfg.assists = makePreset(DrivingPreset::Simulation);
        std::ofstream a(csvDir + "/accel_sports_sim.csv"); cfg.csv = &a; accelerate0to100(cfg);
        std::ofstream b(csvDir + "/brake_sports_sim.csv"); cfg.csv = &b; brake100to0(cfg);
        std::ofstream s(csvDir + "/skidpad_sports_sim.csv"); cfg.csv = &s; skidpad(cfg);
        std::cout << "CSV traces written to " << csvDir << "\n";
        return 0;
    }

    std::cout << "# OpenRoad Dynamics benchmark report\n\n";
    std::cout << "Fictional reference vehicles; flat dry asphalt (frictionScale 1.0) unless noted; 360 Hz, LOD0 fidelity "
                 "(unsprung mass, 5-point contact) unless noted. Timings: single thread, 4-core Intel Xeon @ 2.1 GHz "
                 "cloud container (indicative only).\n";
    for (const Car& c : cars) {
        if (!only.empty() && only != c.id) continue;
        printVehicleSection(c);
    }
    if (only.empty() || only == "sports") {
        printWet(cars[0]);
        printSineDwell(cars[0]);
        printDrift();
        printContact(cars[0]);
        printLod();
        printTimestep(cars[0]);
    }
    for (const Car& c : cars) {
        if (!only.empty() && only != c.id) continue;
        printStability(c);
    }
    return 0;
}
