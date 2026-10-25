// OpenRoad Dynamics — damage: the node/beam crash structure and mechanical damage.
// SPDX-License-Identifier: MIT
#include <algorithm>
#include <cmath>
#include <cstdio>

#include "ord/damage.hpp"
#include "ord/ground.hpp"
#include "ord/presets.hpp"
#include "ord/vehicle.hpp"
#include "ord_test.hpp"
#include "production_vehicles.hpp"
#include "reference_vehicles.hpp"

using namespace ord;

namespace {

// A rigid wall filling x > x0 (full-width barrier).
struct WallAhead final : ObstacleQuery {
    Real x0 = 0;
    explicit WallAhead(Real x) : x0(x) {}
    bool anyNear(const Vec3& c, Real r) const override { return c.x + r > x0; }
    ObstacleContact point(const Vec3& p, const Vec3&, Real margin) const override {
        ObstacleContact c;
        const Real depth = p.x - x0;
        if (depth > -margin) { c.hit = true; c.depth = depth; c.normal = {-1, 0, 0}; c.friction = 0.3; c.id = 1; }
        return c;
    }
};

// A vertical pole (radius r) at (cx, cy).
struct Pole final : ObstacleQuery {
    Real cx, cy, r;
    Pole(Real x, Real y, Real rad) : cx(x), cy(y), r(rad) {}
    bool anyNear(const Vec3& c, Real rad) const override { return std::hypot(c.x - cx, c.y - cy) < rad + r; }
    ObstacleContact point(const Vec3& p, const Vec3&, Real margin) const override {
        ObstacleContact c;
        const Real dx = p.x - cx, dy = p.y - cy, d = std::hypot(dx, dy);
        if (d < r + margin && d > 1e-9) { c.hit = true; c.depth = r - d; c.normal = {dx / d, dy / d, 0}; c.friction = 0.3; c.id = 2; }
        return c;
    }
    ObstacleContact edge(const Vec3& a, const Vec3& b, Real margin, Real& t) const override {
        ObstacleContact c;
        const Real ex = b.x - a.x, ey = b.y - a.y, l2 = ex * ex + ey * ey;
        if (l2 < 1e-12) return c;
        t = std::clamp(((cx - a.x) * ex + (cy - a.y) * ey) / l2, 0.0, 1.0);
        const Real qx = a.x + ex * t, qy = a.y + ey * t, dx = qx - cx, dy = qy - cy, d = std::hypot(dx, dy);
        if (d < r + margin && d > 1e-9) { c.hit = true; c.depth = r - d; c.normal = {dx / d, dy / d, 0}; c.friction = 0.3; c.id = 2; }
        return c;
    }
};

struct CrashResult {
    Real dv = 0, peakG = 0, pulseMs = 0, rebound = 0, crush = 0, intrusion = 0, work = 0, ke = 0;
    bool faulted = false;
    DamageState state;
};

Real frontOf(const Vehicle& v) {
    Real f = -1e9;
    const DamageModel& d = v.damage();
    for (int i = 0; i < d.nodeCount(); ++i) f = std::max(f, d.node(i).rest.x);
    return f;
}

// Coasting straight into a full-width rigid wall at `kmh`. `v` is left after the crash.
CrashResult frontal(Vehicle& v, Real kmh, const ObstacleQuery* wallOverride = nullptr) {
    const VehicleDefinition& def = v.definition();
    FlatGround g;
    const Real speed = kmh / 3.6;
    v.placeOnGround(0, 0, 0, g, speed);
    WallAhead wall(v.state().position.x + frontOf(v) + 0.3);
    v.setObstacles(wallOverride ? wallOverride : &wall);
    CrashResult r;
    r.ke = 0.5 * def.mass * speed * speed;
    DriverInput in;
    const Real dt = 1.0 / 360;
    Real vPrev = speed, tFirst = -1, tMin = 0, vMin = speed;
    for (int i = 0; i < 360; ++i) {
        v.step(dt, in, g);
        const Real vx = v.state().velocity.x;
        if (tFirst < 0 && v.damageState().nodeContacts > 0) tFirst = i * dt;
        r.peakG = std::max(r.peakG, (vPrev - vx) / dt / kGravity);
        if (vx < vMin) { vMin = vx; tMin = i * dt; }
        vPrev = vx;
    }
    r.dv = speed - vMin;
    r.rebound = std::max(0.0, -vMin);
    r.pulseMs = tFirst >= 0 ? (tMin - tFirst) * 1000 : 0;
    r.state = v.damageState();
    r.crush = r.state.crushFront;
    r.intrusion = r.state.intrusion;
    r.work = r.state.plasticWork;
    r.faulted = v.faulted();
    v.setObstacles(nullptr);
    return r;
}

Vehicle crashCar(const VehicleDefinition& def, const DamageConfig& cfg = {}) {
    FlatGround g;
    Vehicle v(def, makePreset(DrivingPreset::Simulation));
    v.reset({0, 0, 0.6}, 0, g);
    v.enableDamage(cfg);
    return v;
}

CrashResult frontal(const VehicleDefinition& def, Real kmh, const DamageConfig& cfg = {}) {
    Vehicle v = crashCar(def, cfg);
    return frontal(v, kmh);
}

// A wall only on the car's left `share` of its width (offset frontal).
struct OffsetWall final : ObstacleQuery {
    Real x0, yMin;
    OffsetWall(Real x, Real y) : x0(x), yMin(y) {}
    bool anyNear(const Vec3& c, Real r) const override { return c.x + r > x0; }
    ObstacleContact point(const Vec3& p, const Vec3&, Real margin) const override {
        ObstacleContact c;
        const Real depth = p.x - x0;
        if (p.y > yMin && depth > -margin) { c.hit = true; c.depth = depth; c.normal = {-1, 0, 0}; c.friction = 0.3; c.id = 3; }
        return c;
    }
    // the wall's edge (a vertical corner at y = yMin) pokes between the nodes
    ObstacleContact edge(const Vec3& a, const Vec3& b, Real margin, Real& t) const override {
        ObstacleContact c;
        const Real ex = b.x - a.x, ey = b.y - a.y, l2 = ex * ex + ey * ey;
        if (l2 < 1e-12) return c;
        t = std::clamp(((x0 - a.x) * ex + (yMin - a.y) * ey) / l2, 0.0, 1.0);
        const Real qx = a.x + ex * t, qy = a.y + ey * t, dx = qx - x0, dy = qy - yMin, d = std::hypot(dx, dy);
        const Real r = 0.03;
        if (d < r + margin && d > 1e-9 && qx > x0 - 0.5) { c.hit = true; c.depth = r - d; c.normal = {dx / d, dy / d, 0}; c.friction = 0.3; c.id = 3; }
        return c;
    }
};

// Heading change (rad) over `seconds` driving straight (no steering) at `kmh` on flat ground.
Real straightLinePull(Vehicle& v, Real kmh, Real seconds) {
    FlatGround g;
    v.placeOnGround(0, 0, 0, g, kmh / 3.6);
    DriverInput in;
    in.throttle = 0.25;
    for (int i = 0; i < static_cast<int>(seconds * 360); ++i) v.step(1.0 / 360, in, g);
    const Vec3 f = v.state().orientation.rotate({1, 0, 0});
    return std::atan2(f.y, f.x);
}

}  // namespace

TEST_CASE("damage: off by default; on, driving clear of everything is bit-identical and costs nothing") {
    TestTrackGround world([](Real x, Real y) { return 0.03 * std::sin(x / 3.0) + 0.02 * std::cos(y / 4.0); });
    WallAhead farWall(1e6);
    Vehicle a(vehicles::referenceSportsRwd(), makePreset(DrivingPreset::Sport));
    Vehicle b(vehicles::referenceSportsRwd(), makePreset(DrivingPreset::Sport));
    a.reset({0, 0, 0.6}, 0, world);
    b.reset({0, 0, 0.6}, 0, world);
    CHECK(!a.damageEnabled());
    b.enableDamage();
    b.setObstacles(&farWall);
    DriverInput in;
    for (int i = 0; i < 240 * 12; ++i) {
        in.throttle = 0.7;
        in.steer = 0.3 * std::sin(i / 240.0);
        a.step(1.0 / 240, in, world);
        b.step(1.0 / 240, in, world);
    }
    CHECK(a.state().position.x == b.state().position.x && a.state().position.y == b.state().position.y);
    CHECK(a.state().velocity.x == b.state().velocity.x);
    CHECK(a.groundQueries() == b.groundQueries());
    CHECK(!b.damageState().awake);
    CHECK(b.damageState().plasticWork == 0);
}

TEST_CASE("damage: frontal rigid wall - crush, pulse and energy follow the crash structure") {
    // Full-width rigid barrier, coasting. A crush spring of frequency omega stops the car in
    // delta-V / omega (0.49 m at 56 km/h for omega = 32 rad/s) over a half period pi / omega
    // (98 ms); real full-width tests at 56 km/h crush 0.5-0.7 m with a 25-40 g peak.
    Real lastCrush = 0;
    for (Real kmh : {15.0, 30.0, 50.0, 56.0, 64.0, 80.0}) {
        const CrashResult r = frontal(vehicles::referenceSaloonAwd(), kmh);
        const DamageState& d = r.state;
        std::printf("    %3.0f km/h: dV %5.1f km/h  peak %5.1f g  pulse %5.1f ms  crush %.3f m  intrusion %.3f  rebound %.2f m/s  structure took %3.0f%% of the energy\n",
                    kmh, r.dv * 3.6, r.peakG, r.pulseMs, r.crush, r.intrusion, r.rebound, 100 * r.work / r.ke);
        std::printf("          toe FL %+.3f FR %+.3f rad, front mounts back %.3f m, steering lock %.2f, radiator %.2f, engine %.2f, drag x%.2f\n",
                    d.wheels[0].toe, d.wheels[1].toe, -d.wheels[0].mountOffset.x, d.steeringLock, d.radiator, d.engineDamage, d.dragScale);
        CHECK(!r.faulted);
        CHECK(r.crush > lastCrush);  // more speed, more crush
        lastCrush = r.crush;
        CHECK(r.rebound < 3.0);      // crashes are mostly plastic (restitution ~0.1)
        if (kmh <= 64) CHECK(r.work > 0.8 * r.ke && r.work < 1.25 * r.ke);  // the energy ends in the structure
        if (kmh == 56) {
            CHECK(r.crush > 0.4 && r.crush < 0.75);
            CHECK(r.peakG > 18 && r.peakG < 45);
            CHECK(r.pulseMs > 60 && r.pulseMs < 160);
            CHECK(r.intrusion < 0.02);  // the zone took it
        }
        if (kmh == 15) CHECK(r.crush < 0.15 && d.radiator < 1);
        if (kmh >= 30) CHECK(d.radiator >= 1);
        if (kmh == 80) {
            CHECK(r.crush > 0.75);
            CHECK(d.engineDamage > 0.4);
            CHECK(d.steeringLock < 0.8);
        }
    }
}

TEST_CASE("damage: other cars crush in their own zones and stay well-behaved") {
    for (const VehicleDefinition& def : {vehicles::referenceHatchFwd(), vehicles::referenceSportsRwd(), vehicles::porsche911CarreraS(),
                                         vehicles::mercedesS600Guard(), vehicles::rosserStellaV8()}) {
        const CrashResult r = frontal(def, 56);
        std::printf("    %-28s 56 km/h: crush %.3f m, peak %.1f g, pulse %.0f ms, structure %.0f%% of the energy\n", def.name.c_str(), r.crush, r.peakG,
                    r.pulseMs, 100 * r.work / r.ke);
        CHECK(!r.faulted);
        CHECK(r.crush > 0.25 && r.crush < 0.95);
        CHECK(r.peakG > 10 && r.peakG < 70);
        CHECK(r.rebound < 3.0);
    }
}

TEST_CASE("damage: a symmetric crash stays symmetric") {
    const CrashResult r = frontal(vehicles::referenceSaloonAwd(), 56);
    const DamageState& d = r.state;
    CHECK(std::abs(d.steeringOffset) < 0.01);
    CHECK(std::abs(d.wheels[0].toe + d.wheels[1].toe) < 0.02);  // mirrored: toe-out both sides
    CHECK(std::abs(d.wheels[0].mountOffset.x - d.wheels[1].mountOffset.x) < 0.02);
}

TEST_CASE("damage: a side pole is stopped by the structure, crush and intrusion grow with speed") {
    Real lastCrush = 0, lastIntr = -1;
    for (Real kmh : {20.0, 32.0, 50.0}) {
        FlatGround g;
        Vehicle v = crashCar(vehicles::referenceSaloonAwd());
        v.placeOnGround(0, 0, 0, g, 0, kmh / 3.6);
        Real hw = 0;
        for (int i = 0; i < v.damage().nodeCount(); ++i) hw = std::max(hw, v.damage().node(i).rest.y);
        Pole pole(v.state().position.x + 0.2, v.state().position.y + hw + 0.25 + 0.127, 0.127);  // 254 mm pole (Euro NCAP)
        v.setObstacles(&pole);
        DriverInput in;
        for (int i = 0; i < 360; ++i) v.step(1.0 / 360, in, g);
        const DamageState& s = v.damageState();
        std::printf("    pole at %2.0f km/h: door crush %.3f m, intrusion %.3f m\n", kmh, s.crushLeft, s.intrusion);
        CHECK(!v.faulted());
        CHECK(v.state().position.y + hw * 0.5 < pole.cy);  // never through the pole
        CHECK(s.crushLeft > lastCrush);
        CHECK(s.intrusion >= lastIntr);
        lastCrush = s.crushLeft;
        lastIntr = s.intrusion;
    }
    CHECK(lastIntr > 0.05);  // at 50 km/h the door is pushed into the cabin
}

TEST_CASE("damage: the roof holds the car, crushes more from higher, and the car comes to rest") {
    Real last = 0;
    for (Real drop : {0.3, 1.0, 2.0}) {
        FlatGround g;
        Vehicle v = crashCar(vehicles::referenceSaloonAwd());
        RigidBodyState st = v.state();
        st.orientation = Quat::fromAxisAngle({1, 0, 0}, kPi) * st.orientation;
        st.position.z += 1.4 + drop;
        v.setChassisState(st);
        DriverInput in;
        Real maxV = 0;
        for (int i = 0; i < 360 * 4; ++i) {
            v.step(1.0 / 360, in, g);
            if (i > 360 * 3) maxV = std::max(maxV, length(v.state().velocity));
        }
        const auto& t = v.telemetry();
        std::printf("    upside down from %.1f m: roof crush %.3f m, at rest carrying %.0f N (weight %.0f N)\n", drop, v.damageState().crushRoof,
                    t.bodyContactForce, v.definition().mass * kGravity);
        CHECK(!v.faulted());
        CHECK(t.upright < -0.95);
        CHECK(v.damageState().crushRoof > last);
        last = v.damageState().crushRoof;
        CHECK(v.damageState().crushRoof < 0.4);  // the pillars hold (3x the weight before they yield)
        CHECK(maxV < 0.03);
        CHECK_NEAR(t.bodyContactForce, v.definition().mass * kGravity, 0.03 * v.definition().mass * kGravity);
    }
}

TEST_CASE("damage: an offset hit bends that corner and the car pulls") {
    FlatGround g;
    Vehicle intact = crashCar(vehicles::referenceSaloonAwd());
    const Real straight = straightLinePull(intact, 60, 4);
    Vehicle v = crashCar(vehicles::referenceSaloonAwd());
    v.placeOnGround(0, 0, 0, g, 0);
    OffsetWall wall(v.state().position.x + frontOf(v) + 0.3, 0.2);  // the left ~40% of the front
    const CrashResult r = frontal(v, 35, &wall);
    const DamageState& d = r.state;
    const Real pulled = straightLinePull(v, 60, 4);
    std::printf("    offset hit at 35 km/h: left crush %.3f m; toe FL %+.3f FR %+.3f, steering %+.3f rad; heading after 4 s straight %+.2f deg (intact %+.2f)\n",
                d.crushFront, d.wheels[0].toe, d.wheels[1].toe, d.steeringOffset, pulled * 180 / kPi, straight * 180 / kPi);
    CHECK(!r.faulted && !v.faulted());
    CHECK(std::abs(d.wheels[0].toe) > std::abs(d.wheels[1].toe));  // the hit corner is the bent one
    CHECK(std::abs(pulled - straight) > 2.0 * kPi / 180);          // it no longer drives straight
}

TEST_CASE("damage: a destroyed radiator overheats the engine; it loses power, then seizes") {
    Vehicle v = crashCar(vehicles::referenceSaloonAwd());
    frontal(v, 32);
    CHECK(v.damageState().radiator >= 1);
    FlatGround g;
    v.placeOnGround(0, 0, 0, g, 15);
    DriverInput in;
    in.throttle = 1.0;
    Real tDerate = -1, tSeize = -1;
    const Real dt = 1.0 / 120;
    for (int i = 0; i < 120 * 180 && tSeize < 0; ++i) {
        v.step(dt, in, g);
        const DamageState& d = v.damageState();
        if (tDerate < 0 && d.torqueScale < 0.95) tDerate = i * dt;
        if (d.engineSeized) tSeize = i * dt;
    }
    std::printf("    coolant gone, full throttle: power loss after %.0f s, seized after %.0f s\n", tDerate, tSeize);
    CHECK(tDerate > 10 && tDerate < 150);
    CHECK(tSeize > tDerate);
    CHECK(v.drivetrain().seized());
    // a seized engine drives nothing: the car rolls to a stop
    for (int i = 0; i < 120 * 20; ++i) v.step(dt, in, g);
    CHECK(v.forwardSpeed() < 3.0);
}

TEST_CASE("damage: a kerb strike pinches the tyre; it goes flat and the car pulls toward it") {
    // a 20 cm square kerb across the left wheels' path only
    TestTrackGround world([](Real x, Real y) { return (y > 0 && x > 30 && x < 30.6) ? 0.20 : 0.0; });
    Vehicle v(vehicles::referenceSaloonAwd(), makePreset(DrivingPreset::Simulation));
    v.reset({0, 0, 0.6}, 0, world);
    v.enableDamage();
    v.placeOnGround(0, 0, 0, world, 70 / 3.6);
    DriverInput in;
    in.throttle = 0.2;
    for (int i = 0; i < 360 * 2; ++i) v.step(1.0 / 360, in, world);
    const DamageState& d = v.damageState();
    std::printf("    kerb at 70 km/h: punctured FL %d FR %d RL %d RR %d, deflation FL %.2f, toe bend FL %.3f\n", d.wheels[0].punctured,
                d.wheels[1].punctured, d.wheels[2].punctured, d.wheels[3].punctured, d.wheels[0].deflation, d.wheels[0].bend);
    CHECK(d.wheels[0].punctured);
    CHECK(!d.wheels[1].punctured);
    // let it go flat, then drive straight: it pulls toward the flat (left)
    for (int i = 0; i < 360 * 4; ++i) v.step(1.0 / 360, in, world);
    CHECK(v.damageState().wheels[0].deflation >= 1.0);
    CHECK(v.definition().wheels[0].tire.radius < v.definition().wheels[1].tire.radius - 0.03);
    const Real heading = straightLinePull(v, 60, 4);
    std::printf("    flat front-left: heading after 4 s straight %+.2f deg\n", heading * 180 / kPi);
    CHECK(heading > 1.0 * kPi / 180);  // +yaw = toward the left
}

TEST_CASE("damage: a big enough hit tears a wheel off; the car carries on, on three") {
    Vehicle v = crashCar(vehicles::referenceSaloonAwd());
    FlatGround g;
    v.placeOnGround(0, 0, 0, g, 0);
    OffsetWall wall(v.state().position.x + frontOf(v) + 0.3, 0.35);
    frontal(v, 90, &wall);
    const DamageState& d = v.damageState();
    std::printf("    offset hit at 90 km/h: wheels gone %d%d%d%d, left mount moved %.3f m\n", d.wheels[0].detached, d.wheels[1].detached, d.wheels[2].detached,
                d.wheels[3].detached, length(d.wheels[0].mountOffset));
    CHECK(d.wheels[0].detached);
    // three wheels and a corner on the ground: it still drives, slowly, without blowing up
    v.placeOnGround(0, 0, 0, g, 0);
    DriverInput in;
    in.throttle = 0.4;
    for (int i = 0; i < 360 * 5; ++i) v.step(1.0 / 360, in, g);
    CHECK(!v.faulted());
    CHECK(v.damageState().nodeContacts > 0);  // the corner rests on the structure
    CHECK(v.state().position.z > -0.5);
}

TEST_CASE("damage: repair restores the car") {
    Vehicle v = crashCar(vehicles::referenceSaloonAwd());
    const Real r0 = v.definition().wheels[0].tire.radius;
    frontal(v, 64);
    CHECK(v.damageState().crushFront > 0.3);
    v.repairDamage();
    const DamageState& d = v.damageState();
    CHECK(d.crushFront == 0 && d.plasticWork == 0 && d.radiator == 0 && d.torqueScale == 1);
    for (int i = 0; i < v.damage().nodeCount(); ++i) {
        const CageNode& n = v.damage().node(i);
        CHECK(n.local.x == n.rest.x && n.local.y == n.rest.y && n.local.z == n.rest.z);
    }
    CHECK(v.definition().wheels[0].tire.radius == r0);
    CHECK(!v.drivetrain().seized());
    // and it crashes the same way again
    const CrashResult again = frontal(v, 30);
    const CrashResult fresh = frontal(vehicles::referenceSaloonAwd(), 30);
    CHECK_NEAR(again.crush, fresh.crush, 0.02);
}

TEST_CASE("damage: deterministic, and a copy mid-crash continues identically (rewind)") {
    const CrashResult a = frontal(vehicles::referenceSaloonAwd(), 50);
    const CrashResult b = frontal(vehicles::referenceSaloonAwd(), 50);
    CHECK(a.crush == b.crush && a.peakG == b.peakG && a.work == b.work);
    FlatGround g;
    Vehicle v = crashCar(vehicles::referenceSaloonAwd());
    v.placeOnGround(0, 0, 0, g, 50 / 3.6);
    WallAhead wall(v.state().position.x + frontOf(v) + 0.3);
    v.setObstacles(&wall);
    DriverInput in;
    for (int i = 0; i < 20; ++i) v.step(1.0 / 360, in, g);
    Vehicle copy = v;
    for (int i = 0; i < 100; ++i) {
        v.step(1.0 / 360, in, g);
        copy.step(1.0 / 360, in, g);
    }
    CHECK(v.state().position.x == copy.state().position.x);
    CHECK(v.damageState().plasticWork == copy.damageState().plasticWork);
}

TEST_CASE("damage: random crashes never fault (cars, speeds, angles, posts)") {
    const VehicleDefinition defs[] = {vehicles::referenceHatchFwd(), vehicles::referenceSaloonAwd(), vehicles::referenceRaceDownforce(),
                                      vehicles::mercedesS600Guard()};
    unsigned seed = 12345;
    auto rnd = [&]() { seed = seed * 1664525u + 1013904223u; return (seed >> 8) / 16777216.0; };
    const int runs = ordtest::kOptimisedBuild ? 24 : 6;
    int faults = 0;
    for (int k = 0; k < runs; ++k) {
        const VehicleDefinition& def = defs[k % 4];
        FlatGround g;
        Vehicle v = crashCar(def);
        const Real yaw = (rnd() - 0.5) * 2.0, speed = 5 + rnd() * 35, lat = (rnd() - 0.5) * 10, spin = (rnd() - 0.5) * 2;
        v.placeOnGround(0, 0, yaw, g, speed, lat, spin);
        Pole pole(6 + rnd() * 3, (rnd() - 0.5) * 2, 0.1 + rnd() * 0.4);
        WallAhead wall(9 + rnd() * 4);
        const bool usePole = rnd() < 0.5;
        v.setObstacles(usePole ? static_cast<const ObstacleQuery*>(&pole) : &wall);
        DriverInput in;
        in.throttle = rnd();
        in.steer = rnd() - 0.5;
        for (int i = 0; i < 360 * 2; ++i) v.step(1.0 / 360, in, g);
        if (v.faulted() || !isFinite(v.state().position)) ++faults;
    }
    CHECK(faults == 0);
}
