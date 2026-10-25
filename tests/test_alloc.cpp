// OpenRoad Dynamics — heap-allocation audit of the hot paths.
// SPDX-License-Identifier: MIT
//
// Counts global operator new calls. The physics step must not allocate once
// warmed up (allocation on a physics thread is a latency and contention risk).
#include <atomic>
#include <cstdlib>
#include <new>

#include "ord/ground_cache.hpp"
#include "ord/lod_manager.hpp"
#include "ord/presets.hpp"
#include "ord/single_track.hpp"
#include "ord/vehicle.hpp"
#include "ord_test.hpp"
#include "reference_vehicles.hpp"

namespace {
std::atomic<long long> gAllocations{0};
}

void* operator new(std::size_t n) {
    gAllocations.fetch_add(1, std::memory_order_relaxed);
    if (void* p = std::malloc(n ? n : 1)) return p;
    throw std::bad_alloc();
}
void operator delete(void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }

using namespace ord;

TEST_CASE("alloc: Vehicle::step, stepSubstepped and the ground cache do not allocate after warm-up") {
    TestTrackGround world([](Real x, Real y) { return 0.04 * std::sin(x / 2.0) + 0.02 * std::cos(y / 3.0); });
    Vehicle v(vehicles::referenceSportsRwd(), makePreset(DrivingPreset::Fun));
    v.reset({0, 0, 0.6}, 0, world);
    GroundPrefetchCache cache;
    std::vector<GroundStripRequest> req;
    req.reserve(kMaxWheels);
    DriverInput in;
    in.throttle = 0.6;
    in.steer = 0.2;
    auto frame = [&](bool external) {
        req.clear();
        v.groundPrefetchRequests(req);
        cache.rebuild(world, req.data(), req.size());
        if (external) {
            v.stepSubstepped(1.0 / 60, 4, in, cache);
            RigidBodyState s = v.state();
            v.setChassisState(s);
        } else {
            for (int k = 0; k < 4; ++k) v.step(1.0 / 240, in, cache);
        }
    };
    for (int f = 0; f < 120; ++f) frame(false);  // warm-up (strip buffers reach their high-water mark)
    long long before = gAllocations.load();
    for (int f = 0; f < 600; ++f) frame(false);
    CHECK(gAllocations.load() - before == 0);
    v.setChassisMode(ChassisMode::External, true);
    for (int f = 0; f < 60; ++f) frame(true);
    before = gAllocations.load();
    for (int f = 0; f < 300; ++f) frame(true);
    CHECK(gAllocations.load() - before == 0);
}

TEST_CASE("alloc: TrafficSystem and LodManager frames do not allocate after warm-up") {
    FlatGround g;
    std::vector<LanePath> lanes{LanePath::circle({0, 0, 0}, 200.0, 30.0), LanePath::circle({0, 0, 0}, 203.5, 30.0)};
    TrafficSystem traffic(lanes, TrafficSystem::Settings{});
    for (int i = 0; i < 200; ++i) traffic.addAgent(i % 2, i * 6.3, 15.0, 15.0 + (i % 5), i % 2);
    LodSettings ls;
    ls.maxLod1 = 8;
    LodManager lod(traffic, {vehicles::referenceHatchFwd(), vehicles::referenceSportsRwd()}, ls);
    const Vec3 focus{0, -200, 0};
    for (int f = 0; f < 240; ++f) lod.update(1.0 / 60, focus, g);  // warm-up incl. promotions
    const long long before = gAllocations.load();
    for (int f = 0; f < 600; ++f) lod.update(1.0 / 60, focus, g);  // steady state, same focus
    CHECK(gAllocations.load() - before == 0);
}

TEST_CASE("alloc: SingleTrack::step does not allocate (NoSlip and Tyre)") {
    for (SingleTrackContact mode : {SingleTrackContact::NoSlip, SingleTrackContact::Tyre}) {
        SingleTrack bike(meijaardBenchmark(), mode);
        bike.reset(5.0, 0.05);
        SingleTrackInput in;
        in.steerTorque = 0.1;
        bike.step(5e-4, in);
        const long long before = gAllocations.load();
        for (int i = 0; i < 400; ++i) bike.step(5e-4, in);
        CHECK(gAllocations.load() - before == 0);
        CHECK(!bike.faulted());
    }
}

namespace {
struct AllocWall final : ord::ObstacleQuery {
    ord::Real x0 = 0;
    bool anyNear(const ord::Vec3& c, ord::Real r) const override { return c.x + r > x0; }
    ord::ObstacleContact point(const ord::Vec3& p, const ord::Vec3&, ord::Real margin) const override {
        ord::ObstacleContact c;
        if (p.x - x0 > -margin) { c.hit = true; c.depth = p.x - x0; c.normal = {-1, 0, 0}; c.id = 1; }
        return c;
    }
};
}  // namespace

TEST_CASE("alloc: the crash structure does not allocate (driving, crashing, on its roof, repair)") {
    FlatGround g;
    Vehicle v(vehicles::referenceSaloonAwd(), makePreset(DrivingPreset::Sport));
    v.reset({0, 0, 0.6}, 0, g);
    v.enableDamage();
    AllocWall wall;
    wall.x0 = 40;
    v.setObstacles(&wall);
    DriverInput in;
    in.throttle = 1.0;
    for (int i = 0; i < 60; ++i) v.step(1.0 / 360, in, g);  // warm-up
    const long long before = gAllocations.load();
    for (int i = 0; i < 360 * 4; ++i) v.step(1.0 / 360, in, g);  // drives into the wall
    RigidBodyState s = v.state();
    s.orientation = Quat::fromAxisAngle({1, 0, 0}, kPi) * s.orientation;
    s.position.z += 1.6;
    v.setChassisState(s);
    for (int i = 0; i < 360; ++i) v.step(1.0 / 360, in, g);  // lands on its roof
    v.repairDamage();
    for (int i = 0; i < 60; ++i) v.step(1.0 / 360, in, g);
    CHECK(v.damageState().awake || v.damageState().version > 0);
    CHECK(gAllocations.load() - before == 0);
}
