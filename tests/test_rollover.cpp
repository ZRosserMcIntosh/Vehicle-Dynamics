// OpenRoad Dynamics — rollover: the body against the ground.
// SPDX-License-Identifier: MIT
#include <algorithm>
#include <cmath>
#include <cstdio>

#include "ord/ground.hpp"
#include "ord/presets.hpp"
#include "ord/vehicle.hpp"
#include "ord_test.hpp"
#include "production_vehicles.hpp"
#include "reference_vehicles.hpp"

using namespace ord;

namespace {
// Released at rest at a roll angle with its lowest point just above the ground. Returns the final
// "upright" (1 on its wheels, 0 on its side, -1 on its roof).
Real releaseTilted(const VehicleDefinition& d, Real deg) {
    FlatGround g;
    Vehicle v(d, makePreset(DrivingPreset::Simulation));
    v.reset({0, 0, 0.6}, 0, g);
    RigidBodyState s = v.state();
    const Real a = deg * kPi / 180;
    s.orientation = Quat::fromAxisAngle({1, 0, 0}, a) * s.orientation;
    Real lowest = 1e9;
    for (const WheelDef& w : d.wheels) {
        const Vec3 c = s.position + s.orientation.rotate(w.suspension.hardpoint - Vec3{0, 0, w.suspension.travel});
        lowest = std::min(lowest, c.z - w.tire.radius * std::cos(a) - 0.12 * std::sin(a));
    }
    s.position.z += 0.01 - lowest;
    v.setChassisState(s);
    DriverInput in;
    for (int i = 0; i < 360 * 5; ++i) v.step(1.0 / 360, in, g);
    CHECK(!v.faulted());
    return v.telemetry().upright;
}
struct Slide {
    Real minUpright = 1;
    bool faulted = false;
};
// Sliding sideways (toward -y) at `kmh` on `ground`.
Slide slideSideways(const VehicleDefinition& d, const GroundQuery& g, Real kmh) {
    Vehicle v(d, makePreset(DrivingPreset::Simulation));
    v.reset({0, 0, 0.6}, 0, g);
    RigidBodyState s = v.state();
    s.velocity = {0, -kmh / 3.6, 0};
    v.setChassisState(s);
    DriverInput in;
    Slide r;
    for (int i = 0; i < 360 * 5; ++i) {
        v.step(1.0 / 360, in, g);
        r.minUpright = std::min(r.minUpright, v.telemetry().upright);
    }
    r.faulted = v.faulted();
    return r;
}
}  // namespace

TEST_CASE("rollover: a car upside down rests on its roof (the body touches the ground)") {
    // Before: no body collision at all - a car on its roof fell through the road.
    FlatGround g;
    Vehicle v(vehicles::referenceSportsRwd(), makePreset(DrivingPreset::Simulation));
    v.reset({0, 0, 0.6}, 0, g);
    RigidBodyState s = v.state();
    s.orientation = Quat::fromAxisAngle({1, 0, 0}, kPi) * s.orientation;
    s.position.z += 1.4;
    v.setChassisState(s);
    DriverInput in;
    for (int i = 0; i < 360 * 4; ++i) v.step(1.0 / 360, in, g);
    const auto& t = v.telemetry();
    std::printf("    on its roof: upright %.2f, body origin %.2f m up, %d hull points carrying %.0f N (weight %.0f N)\n", t.upright,
                v.state().position.z, t.bodyContacts, t.bodyContactForce, v.definition().mass * kGravity);
    CHECK(!v.faulted());
    CHECK(t.upright < -0.95);
    CHECK(v.state().position.z > 0.4 && v.state().position.z < 1.3);
    CHECK(t.bodyContacts >= 3);
    CHECK_NEAR(t.bodyContactForce, v.definition().mass * kGravity, 0.05 * v.definition().mass * kGravity);
}

TEST_CASE("rollover: tipped past its tipping angle a car falls on its side; below it, back on its wheels") {
    // Static stability: the tipping angle is about atan(half track / CoM height) - higher and
    // narrower cars tip earlier. Released at rest just above the ground.
    struct Case { VehicleDefinition d; Real stays, falls; };
    for (const Case& c : {Case{vehicles::referenceSportsRwd(), 55, 75}, Case{vehicles::mercedesS600Guard(), 50, 70},
                          Case{vehicles::formula1Car2026(), 65, 85}}) {
        const Real low = releaseTilted(c.d, c.stays), high = releaseTilted(c.d, c.falls);
        std::printf("    %-34s released at %2.0f deg: upright %+.2f | at %2.0f deg: upright %+.2f\n", c.d.name.c_str(), c.stays, low,
                    c.falls, high);
        CHECK(low > 0.95);
        CHECK(high < 0.5);
    }
    CHECK(releaseTilted(vehicles::mercedesS600Guard(), 65) < 0.5);  // the tall, heavy one tips first
    CHECK(releaseTilted(vehicles::formula1Car2026(), 65) > 0.95);   // the low, wide one last
}

TEST_CASE("rollover: sliding sideways into a kerb trips the car over; flat ground does not") {
    TestTrackGround kerb([](Real, Real y) { return y < -2.0 ? 0.25 : 0.0; });  // 25 cm step
    FlatGround flat;
    for (const VehicleDefinition& d : {vehicles::referenceSportsRwd(), vehicles::mercedesS600Guard(), vehicles::rosserStellaV8()}) {
        const Slide slow = slideSideways(d, kerb, 15), fast = slideSideways(d, kerb, 45), noKerb = slideSideways(d, flat, 80);
        std::printf("    %-34s into a 25 cm kerb at 15 km/h: min upright %+.2f, at 45 km/h: %+.2f; 80 km/h on flat: %+.2f\n",
                    d.name.c_str(), slow.minUpright, fast.minUpright, noKerb.minUpright);
        CHECK(!slow.faulted && !fast.faulted && !noKerb.faulted);
        CHECK(slow.minUpright > 0.8);     // a bump, not a roll
        CHECK(fast.minUpright < -0.5);    // over the roof
        CHECK(noKerb.minUpright > 0.8);   // tyres slide; a car this low doesn't trip on flat asphalt
    }
}

TEST_CASE("rollover: normal driving never touches the body hull") {
    // The hull is only consulted when the car is tilted or airborne: a hard drive on rough ground
    // never reports body contact (and the golden baseline is unchanged).
    TestTrackGround g([](Real x, Real y) { return 0.04 * std::sin(0.8 * x) * std::cos(0.6 * y); });
    Vehicle v(vehicles::referenceSportsRwd(), makePreset(DrivingPreset::Sport));
    v.reset({0, 0, 0.6}, 0, g);
    DriverInput in;
    int touches = 0;
    for (int i = 0; i < 360 * 8; ++i) {
        in.throttle = 1;
        in.steer = std::sin(i / 360.0 * 1.5);
        v.step(1.0 / 360, in, g);
        touches += v.telemetry().bodyContacts;
    }
    CHECK(touches == 0);
    CHECK(!v.faulted());
}
