// OpenRoad Dynamics — cars approximating real ones, checked against their published figures.
// SPDX-License-Identifier: MIT
//
// The published values (0-100 km/h, top speed or its electronic limit) are what the estimated
// parameters were tuned to; these tests keep them there. Tolerances are wide enough for the
// estimates (launch technique, turbo lag and weather aren't modelled) and tight enough to catch a
// broken definition.
#include <algorithm>
#include <cstdio>
#include <string>

#include "ord/ground.hpp"
#include "ord/presets.hpp"
#include "ord/vehicle.hpp"
#include "ord_test.hpp"
#include "production_vehicles.hpp"
#include "scenarios.hpp"

using namespace ord;
using namespace ord::scenarios;

namespace {
struct Run {
    Real t100 = -1, vmaxKmh = 0;
};
Run fullThrottle(const VehicleDefinition& d, Real seconds) {
    Vehicle v(d, makePreset(DrivingPreset::Sport));
    FlatGround g;
    v.reset({0, 0, 0.6}, 0, g);
    DriverInput in;
    in.throttle = 1;
    Run r;
    for (int i = 0; i < static_cast<int>(seconds * 360); ++i) {
        v.step(1.0 / 360, in, g);
        const Real kmh = v.forwardSpeed() * 3.6;
        if (r.t100 < 0 && kmh >= 100) r.t100 = (i + 1) / 360.0;
        r.vmaxKmh = std::max(r.vmaxKmh, kmh);
    }
    return r;
}
}  // namespace

TEST_CASE("production cars: definitions are valid") {
    for (const VehicleDefinition& d : {vehicles::porsche911CarreraS(), vehicles::porsche911Carrera4S(), vehicles::mercedesAmgS63(),
                                       vehicles::mercedesS600Guard(), vehicles::formula1Car2026()}) {
        const std::string err = d.validate();
        if (!err.empty()) std::printf("    %s: %s\n", d.name.c_str(), err.c_str());
        CHECK(err.empty());
    }
}

TEST_CASE("production cars: 0-100 km/h near the published figures") {
    struct Case { VehicleDefinition d; Real lo, hi; const char* published; };  // published = the source figure
    for (const Case& c : {Case{vehicles::porsche911CarreraS(), 3.3, 4.0, "3.5 s (3.3 s Sport Chrono)"},
                          Case{vehicles::porsche911Carrera4S(), 3.1, 3.8, "3.5 s (3.3 s Sport Chrono)"},
                          Case{vehicles::mercedesAmgS63(), 3.0, 3.9, "3.5 s"},
                          Case{vehicles::mercedesS600Guard(), 5.5, 10.0, "none; armoured relatives 7.9-8.3 s"},
                          Case{vehicles::formula1Car2026(), 2.2, 3.2, "~2.6 s (typical)"}}) {
        const Run r = fullThrottle(c.d, 12);
        std::printf("    %-34s 0-100 km/h %.2f s (published: %s)\n", c.d.name.c_str(), r.t100, c.published);
        CHECK(r.t100 > c.lo && r.t100 < c.hi);
    }
}

TEST_CASE("production cars: top speed, and the electronic limiters hold") {
    if (!ordtest::kOptimisedBuild) return;  // two simulated minutes per car: Release only
    struct Case { VehicleDefinition d; Real lo, hi; const char* published; };  // published = the source figure
    for (const Case& c : {Case{vehicles::porsche911CarreraS(), 293, 325, "308 km/h"},
                          Case{vehicles::mercedesAmgS63(), 244, 251.5, "250 km/h (limited)"},
                          Case{vehicles::mercedesS600Guard(), 204, 211.5, "210 km/h (limited)"},
                          Case{vehicles::formula1Car2026(), 290, 360, "~340 km/h (low-drag mode, not modelled)"}}) {
        const Run r = fullThrottle(c.d, 120);
        std::printf("    %-34s top speed %.0f km/h (published: %s)\n", c.d.name.c_str(), r.vmaxKmh, c.published);
        CHECK(r.vmaxKmh > c.lo && r.vmaxKmh < c.hi);
    }
}

TEST_CASE("production cars: grip and braking in plausible ranges") {
    struct Case { VehicleDefinition d; Real gLo, gHi, brakeHi; };
    for (const Case& c : {Case{vehicles::porsche911CarreraS(), 0.9, 1.1, 40}, Case{vehicles::mercedesAmgS63(), 0.85, 1.1, 40},
                          Case{vehicles::mercedesS600Guard(), 0.7, 1.0, 48}, Case{vehicles::formula1Car2026(), 1.5, 2.5, 30}}) {
        RunConfig cfg;
        cfg.vehicle = c.d;
        cfg.assists = makePreset(DrivingPreset::Sport);
        const Real g = skidpad(cfg, 40).maxLateralG;
        const BrakeResult br = brake100to0(cfg);
        std::printf("    %-34s skidpad R40 %.2f g, 100-0 km/h %.1f m\n", c.d.name.c_str(), g, br.distance);
        CHECK(g > c.gLo && g < c.gHi);
        CHECK(br.stopped && br.distance < c.brakeHi);
    }
}
