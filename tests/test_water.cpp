// OpenRoad Dynamics — standing water (hydroplaning, displacement drag), tyre
// support on steep ground, and the garage's ride height raising the floor.
// SPDX-License-Identifier: MIT
#include <cmath>
#include <iostream>

#include "ord/ground.hpp"
#include "ord/presets.hpp"
#include "ord/setup.hpp"
#include "ord/vehicle.hpp"
#include "ord_test.hpp"
#include "production_vehicles.hpp"
#include "reference_vehicles.hpp"

using namespace ord;

namespace {

SurfaceProperties water(Real depth) {
    SurfaceProperties s;
    s.frictionScale = 0.75;
    s.wetness = 1.0;
    s.waterDepth = depth;
    return s;
}

// Coasts at `kmh`, then steers `steer` for 0.6 s; returns the yaw rate reached and the front
// tyres' slip speed (rolling speed short of the car's) at the end.
struct SteerResult { Real yawRate, frontSlip; };
SteerResult steerStep(const GroundQuery& g, Real kmh, Real steer) {
    Vehicle v(vehicles::referenceSportsRwd(), makePreset(DrivingPreset::Simulation));
    v.placeOnGround(0, 0, 0, g, kmh / 3.6);
    DriverInput in;
    for (int i = 0; i < 360 / 2; ++i) v.step(1.0 / 360, in, g);
    in.steer = steer;
    for (int i = 0; i < 360 * 6 / 10; ++i) v.step(1.0 / 360, in, g);
    const TelemetryFrame& t = v.telemetry();
    const Real front = 0.5 * (t.wheels[0].omega + t.wheels[1].omega) * v.definition().wheels[0].tire.radius;
    return {std::abs(t.yawRate), t.speed - front};
}

// An inclined plane through the origin with unit normal n (analytic, any steepness).
class PlaneGround final : public GroundQuery {
public:
    explicit PlaneGround(const Vec3& n) : n_(normalized(n)) {}
    GroundHit raycast(const Vec3& o, const Vec3& d, Real maxDistance) const override {
        GroundHit h;
        const Real dn = dot(d, n_);
        if (dn > -1e-9) return h;
        const Real t = -dot(o, n_) / dn;
        if (t < 0 || t > maxDistance) return h;
        h.hit = true; h.distance = t; h.point = o + d * t; h.normal = n_;
        return h;
    }
private:
    Vec3 n_;
};

}  // namespace

TEST_CASE("water: the hydroplaning speed follows Horne's rule v = 6.35 sqrt(p[kPa]) km/h") {
    for (Real p : {150.0, 220.0, 300.0}) {
        TireParams t;
        t.inflationPressure = p;
        CHECK_NEAR(hydroplaningSpeed(t) * 3.6, 6.35 * std::sqrt(p), 0.005 * 6.35 * std::sqrt(p));
    }
}

TEST_CASE("water: lift needs water deeper than the tread clears and speed near v_p; worn tyres lift first") {
    TireParams fresh, worn;
    fresh.treadDepth = 0.008;
    worn.treadDepth = 0.0016;
    const Real vp = hydroplaningSpeed(fresh);
    CHECK(hydroplaneLift(fresh, 0.0, vp) == 0);
    CHECK(hydroplaneLift(fresh, 0.001, vp) == 0);       // a film the grooves carry away
    CHECK(hydroplaneLift(fresh, 0.010, 0.45 * vp) == 0);  // too slow
    CHECK_NEAR(hydroplaneLift(fresh, 0.010, vp), 1.0, 1e-12);
    CHECK(hydroplaneLift(worn, 0.003, 0.9 * vp) > hydroplaneLift(fresh, 0.003, 0.9 * vp) + 0.3);
    // displacement drag, 1/2 rho Cd w h v^2: 10 mm at 100 km/h on a 200 mm tyre ~0.58 kN
    TireParams t;
    t.patchHalfWidth = 0.10;
    CHECK_NEAR(waterDrag(t, 0.010, 100 / 3.6), 0.5 * 1000 * 0.75 * 0.2 * 0.010 * (100 / 3.6) * (100 / 3.6), 1e-9);
    CHECK(waterDrag(t, 0, 30) == 0);
}

TEST_CASE("water: a car steers through 10 mm of water slowly and not at all fast, and its front wheels spin down") {
    TestTrackGround dry;
    TestTrackGround wet;
    wet.addPatch({-50, 2000, -50, 50, water(0.010), 6});
    const SteerResult d50 = steerStep(dry, 50, 0.15), w50 = steerStep(wet, 50, 0.15);
    const SteerResult d130 = steerStep(dry, 130, 0.08), w130 = steerStep(wet, 130, 0.08);
    std::cout << "    yaw rate at 50 km/h: dry " << d50.yawRate << ", water " << w50.yawRate << " rad/s; at 130 km/h: dry "
              << d130.yawRate << ", water " << w130.yawRate << " rad/s; front tyre slip speed at 130 km/h in water "
              << w130.frontSlip << " m/s\n";
    CHECK(w50.yawRate > 0.6 * d50.yawRate);   // well below v_p: the tread clears it
    CHECK(w130.yawRate < 0.2 * d130.yawRate); // past v_p: riding on water
    CHECK(w130.frontSlip > 0.3 * 130 / 3.6);  // the wheels spin down (NASA's indicator)
    CHECK(d130.frontSlip < 0.5);
}

TEST_CASE("water: water under one side pulls the car toward it, the same either way") {
    Real yaw[2];
    for (int side = 0; side < 2; ++side) {
        TestTrackGround g;
        if (side == 0) g.addPatch({20, 40, 0.1, 5, water(0.012), 7});
        else g.addPatch({20, 40, -5, -0.1, water(0.012), 7});
        Vehicle v(vehicles::referenceSportsRwd(), makePreset(DrivingPreset::Simulation));
        v.placeOnGround(0, 0, 0, g, 100 / 3.6);
        DriverInput in;
        for (int i = 0; i < 360 * 3 / 2; ++i) v.step(1.0 / 360, in, g);
        const Vec3 fwd = v.state().orientation.rotate({1, 0, 0});
        yaw[side] = std::atan2(fwd.y, fwd.x);
    }
    std::cout << "    heading after a 20 m puddle at 100 km/h, hands off: water on the left " << yaw[0] * 57.3
              << " deg, on the right " << yaw[1] * 57.3 << " deg\n";
    CHECK(yaw[0] > 0.01);   // toward the left (+y)
    CHECK(yaw[1] < -0.01);
    CHECK_NEAR(yaw[0], -yaw[1], 0.1 * std::abs(yaw[0]));
}

TEST_CASE("steep ground: on a 70 degree face the tyres carry the car, not its body") {
    // Without a reference to the ground it is on, a car tilted 70 degrees from the vertical
    // looks like a car falling over: its tyres would let go and the body would land on the
    // face. On a banking, a wall or inside a loop it is on its wheels.
    const Real a = 70.0 * kPi / 180.0;
    const Quat tilt = Quat::fromAxisAngle({1, 0, 0}, a);
    PlaneGround slope(tilt.rotate({0, 0, 1}));
    FlatGround flat;
    Vehicle v(vehicles::referenceSportsRwd(), makePreset(DrivingPreset::Simulation));
    v.placeOnGround(0, 0, 0, flat);
    RigidBodyState s = v.state();
    s.position = tilt.rotate(s.position);
    s.orientation = tilt * s.orientation;
    v.setChassisState(s);
    DriverInput in;
    in.brake = 1;
    Real loads = 0;
    int body = 0;
    for (int i = 0; i < 90; ++i) {
        v.step(1.0 / 360, in, slope);
        if (i >= 30) {
            Real l = 0;
            for (std::size_t w = 0; w < 4; ++w) l += v.telemetry().wheels[w].load;
            loads += l / 60;
            body = std::max(body, v.telemetry().bodyContacts);
        }
    }
    const Real expected = v.definition().mass * 9.81 * std::cos(a);
    std::cout << "    on a 70 deg face: tyre loads " << loads << " N (m g cos 70 = " << expected << " N), body contacts " << body << "\n";
    CHECK_RANGE(loads, 0.7 * expected, 1.3 * expected);
    CHECK(body == 0);
}

TEST_CASE("garage: raising the car on its suspension raises its floor too") {
    const VehicleDefinition stock = vehicles::porsche911CarreraS();
    VehicleSetup s = stockSetup(stock);
    s.front.rideHeightMm = 40;
    s.rear.rideHeightMm = 60;
    const VehicleDefinition d = applySetup(stock, s);
    CHECK_NEAR(d.body.clearanceOffset, 0.05, 1e-12);
    CHECK(d.validate().empty());
    CHECK(Vehicle(d).isValid());
}
