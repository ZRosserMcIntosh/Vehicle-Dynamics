// OpenRoad Dynamics — garage setup tests: alignment, dampers, tyres, aero.
// SPDX-License-Identifier: MIT
//
// These pin the *direction and rough size* of each setup change, the way a
// tuning sheet describes it. Absolute numbers depend on representative tyre
// and aero data; the orderings are what a setup tool must get right.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "ord/ground.hpp"
#include "ord/presets.hpp"
#include "ord/setup.hpp"
#include "ord/vehicle.hpp"
#include "ord_test.hpp"
#include "reference_vehicles.hpp"
#include "scenarios.hpp"

using namespace ord;
using namespace ord::scenarios;

namespace {

RunConfig garageCfg(const VehicleSetup& s, bool warm = false) {
    RunConfig c;
    const VehicleDefinition stock = vehicles::referenceSportsRwd();
    c.vehicle = applySetup(stock, s);
    c.assists = makePreset(DrivingPreset::Simulation);
    c.assists.shiftMode = ShiftMode::Automatic;
    c.assists.abs = true;
    if (warm) c.tyreTemperature = c.vehicle.wheels[0].tire.optimalTemperature;
    return c;
}

VehicleSetup base() { return stockSetup(vehicles::referenceSportsRwd()); }

}  // namespace

TEST_CASE("garage: stock setup round-trips through JSON and reproduces the stock geometry") {
    VehicleSetup s = base();
    s.front.camberDeg = -2.25;
    s.rear.compound = TyreCompound::Slick;
    s.gears[2] = 1.47;
    s.rearWing = true;
    s.wingAngleDeg = 11;
    s.altitudeM = 760;
    const std::string json = setupToJson(s);
    VehicleSetup back = base();
    CHECK(setupFromJson(json, back).empty());
    CHECK_NEAR(back.front.camberDeg, -2.25, 1e-3);
    CHECK(back.rear.compound == TyreCompound::Slick);
    CHECK_NEAR(back.gears[2], 1.47, 1e-3);
    CHECK(back.rearWing && std::abs(back.wingAngleDeg - 11) < 1e-3);
    CHECK_NEAR(back.altitudeM, 760, 1e-6);
    CHECK(setupToJson(back) == json);
    CHECK(!setupFromJson("nonsense", back).empty());
    CHECK(!setupFromJson("{\"front\": ", back).empty());
    CHECK(!setupFromJson("{\"front\": {\"camberDeg\": -1}", back).empty());

    const VehicleDefinition stock = vehicles::referenceSportsRwd();
    const VehicleDefinition same = applySetup(stock, base());
    CHECK(same.validate().empty());
    for (std::size_t w = 0; w < stock.wheels.size(); ++w) {
        CHECK_NEAR(same.wheels[w].suspension.springRate, stock.wheels[w].suspension.springRate, 1e-6);
        CHECK_NEAR(same.wheels[w].maxBrakeTorque, stock.wheels[w].maxBrakeTorque, 1e-6);
        CHECK_NEAR(same.wheels[w].maxSteerAngle, stock.wheels[w].maxSteerAngle, 1e-9);
        CHECK_NEAR(same.wheels[w].suspension.hardpoint.z, stock.wheels[w].suspension.hardpoint.z, 1e-12);
    }
    CHECK_NEAR(same.aero.airDensity, stock.aero.airDensity, 0.001);
}

TEST_CASE("garage: camber follows the geometry: static, body roll, camber gain and caster") {
    VehicleSetup s = base();
    s.front.camberDeg = -2.0;
    s.front.casterDeg = 7.0;
    s.rear.camberDeg = -1.0;
    s.rear.camberGainDegPerCm = -0.8;
    Vehicle v(applySetup(vehicles::referenceSportsRwd(), s), makePreset(DrivingPreset::Simulation));
    FlatGround g;
    v.reset({0, 0, 0.6}, 0, g);
    DriverInput in;
    for (int i = 0; i < 720; ++i) v.step(1.0 / 360, in, g);
    const auto deg = [](Real r) { return r * 180 / kPi; };
    const auto& w0 = v.telemetry().wheels;
    std::printf("    at rest: FL %.2f FR %.2f RL %.2f RR %.2f deg\n", deg(w0[0].camber), deg(w0[1].camber), deg(w0[2].camber), deg(w0[3].camber));
    CHECK_NEAR(deg(w0[0].camber), -2.0, 0.15);
    CHECK_NEAR(deg(w0[1].camber), -2.0, 0.15);
    CHECK_NEAR(deg(w0[2].camber), -1.0, 0.15);
    // Caster: full left lock at a standstill: the outside (right) front gains
    // negative camber, the inside one positive.
    in.steer = 1.0;
    for (int i = 0; i < 360; ++i) v.step(1.0 / 360, in, g);
    const Real fl = deg(v.telemetry().wheels[0].camber), fr = deg(v.telemetry().wheels[1].camber);
    std::printf("    full left lock: FL %.2f FR %.2f deg\n", fl, fr);
    CHECK(fr < -3.5 && fl > -0.5);
    // Cornering: body roll tips the outside wheels toward positive camber;
    // the rear's camber gain (bump on the outside) claws some of it back.
    RunConfig c = garageCfg(s);
    VehicleSetup noGain = s;
    noGain.rear.camberGainDegPerCm = 0;
    auto outsideRear = [&](const VehicleSetup& set) {
        Vehicle car(applySetup(vehicles::referenceSportsRwd(), set), makePreset(DrivingPreset::Simulation));
        car.reset({0, 0, 0.6}, 0, g);
        car.setRollingSpeed(18.0);
        DriverInput d;
        d.steer = 0.25;
        d.throttle = 0.3;
        for (int i = 0; i < 360 * 3; ++i) car.step(1.0 / 360, d, g);
        return deg(car.telemetry().wheels[3].camber);  // RR = outside in a left turn
    };
    const Real withGain = outsideRear(s), without = outsideRear(noGain);
    std::printf("    left turn, outside rear: %.2f deg (gain -0.8 deg/cm) vs %.2f deg (no gain)\n", withGain, without);
    CHECK(without > -0.5);       // roll has eaten the static -1 deg
    CHECK(withGain < without - 0.5);
}

TEST_CASE("garage: more negative camber costs braking; tyre pressure is best near nominal") {
    double prev = 0;
    for (double cam : {0.0, -2.0, -4.0, -6.0}) {
        VehicleSetup s = base();
        s.front.camberDeg = cam;
        s.rear.camberDeg = 0.7 * cam;
        const BrakeResult br = brake100to0(garageCfg(s));
        std::printf("    camber %4.1f deg: 100-0 km/h %.1f m\n", cam, br.distance);
        CHECK(br.stopped && br.distance >= prev - 0.05);
        prev = br.distance;
    }
    double best = 0, bestP = 0;
    for (double p : {1.6, 2.2, 2.8}) {
        VehicleSetup s = base();
        s.front.pressureBar = s.rear.pressureBar = p;
        const double g = skidpad(garageCfg(s), 40).maxLateralG;
        std::printf("    %.1f bar: skidpad %.3f g\n", p, g);
        if (g > best) { best = g; bestP = p; }
    }
    CHECK_NEAR(bestP, 2.2, 1e-9);
}

TEST_CASE("garage: compounds: dry, wet and gravel ladders with warm tyres") {
    struct Row { TyreCompound c; double dry, wet, gravel, brake; };
    std::vector<Row> rows;
    for (TyreCompound c : {TyreCompound::OffRoad, TyreCompound::Street, TyreCompound::Slick, TyreCompound::Rain}) {
        VehicleSetup s = base();
        s.front.compound = s.rear.compound = c;
        RunConfig dry = garageCfg(s, true);
        RunConfig wet = dry, gravel = dry;
        wet.surface.frictionScale = 0.6;
        wet.surface.wetSpeedFalloff = 0.008;
        wet.surface.wetness = 1;
        gravel.surface.frictionScale = 0.6;
        gravel.surface.looseness = 1;
        rows.push_back({c, skidpad(dry, 40).maxLateralG, skidpad(wet, 40).maxLateralG, skidpad(gravel, 40).maxLateralG,
                        brake100to0(dry).distance});
        const Row& r = rows.back();
        std::printf("    %-9s dry %.2f g (brake %4.1f m) | wet %.2f g | gravel %.2f g\n", compoundName(c), r.dry, r.brake, r.wet, r.gravel);
    }
    const Row &off = rows[0], &street = rows[1], &slick = rows[2], &rain = rows[3];
    CHECK(slick.dry > street.dry + 0.08 && street.dry > off.dry + 0.03);
    CHECK(slick.brake < street.brake - 5 && street.brake < off.brake - 4);
    CHECK(rain.wet > street.wet && street.wet > slick.wet + 0.15);
    CHECK(off.gravel > street.gravel + 0.08 && street.gravel > slick.gravel);
}

TEST_CASE("garage: tyre temperature: slicks need heat; hard cornering warms them; cruising doesn't") {
    VehicleSetup s = base();
    s.front.compound = s.rear.compound = TyreCompound::Slick;
    const double cold = skidpad(garageCfg(s, false), 40).maxLateralG;
    const double warm = skidpad(garageCfg(s, true), 40).maxLateralG;
    std::printf("    slicks on the skidpad: cold start %.3f g, warm %.3f g\n", cold, warm);
    CHECK(warm > cold + 0.05);
    // Cruising at 100 km/h: settles well under the slick's window.
    Vehicle v(applySetup(vehicles::referenceSportsRwd(), s), makePreset(DrivingPreset::Sport));
    FlatGround g;
    v.reset({0, 0, 0.6}, 0, g);
    v.setRollingSpeed(27.8);
    DriverInput in;
    for (int i = 0; i < 360 * 120; ++i) {
        in.throttle = v.forwardSpeed() < 27.8 ? 0.4 : 0.0;
        v.step(1.0 / 360, in, g);
    }
    const Real cruise = v.telemetry().wheels[0].temperature;
    // Then a minute of hard cornering on a circle.
    for (int i = 0; i < 360 * 60; ++i) {
        in.steer = 0.35;
        in.throttle = v.forwardSpeed() < 17 ? 0.7 : 0.1;
        v.step(1.0 / 360, in, g);
    }
    const Real hot = v.telemetry().wheels[1].temperature;  // outside front
    std::printf("    slick tread: cruising 2 min at 100 km/h %.0f C, then a minute of hard cornering %.0f C (window %.0f C)\n", cruise,
                hot, applySetup(vehicles::referenceSportsRwd(), s).wheels[0].tire.optimalTemperature);
    CHECK(cruise > s.temperatureC + 5 && cruise < 60);  // rolling hysteresis alone stays well under the window
    CHECK(hot > cruise + 20);
}

TEST_CASE("garage: brake balance: rear-biased brakes without ABS lock the rears and swap ends") {
    // A full stop with a slight steer input (a real road is never perfectly
    // symmetric): front-biased brakes stay straight, rear-biased ones spin.
    auto run = [](double bal) {
        VehicleSetup s = base();
        s.brakeBalance = bal;
        s.brakePressure = 1.3;
        AssistConfig a = makePreset(DrivingPreset::Simulation);
        a.abs = false;
        Vehicle v(applySetup(vehicles::referenceSportsRwd(), s), a);
        FlatGround g;
        v.reset({0, 0, 0.6}, 0, g);
        v.setRollingSpeed(27.8);
        DriverInput in;
        in.brake = 1;
        in.steer = 0.02;
        Real worst = 0;
        for (int i = 0; i < 360 * 6 && v.forwardSpeed() > 2; ++i) {
            v.step(1.0 / 360, in, g);
            worst = std::max(worst, std::abs(v.telemetry().sideslip));
        }
        return worst;
    };
    const Real front = run(0.68), rear = run(0.30);
    std::printf("    braking from 100 km/h, no ABS: 68%% front: peak sideslip %.1f deg | 30%% front: %.1f deg\n", front * 57.3,
                rear * 57.3);
    CHECK(front < 0.05);
    CHECK(rear > 0.35);
}

TEST_CASE("garage: two-stage dampers: a softer fast bump takes the edge off a sharp bump") {
    VehicleSetup s = base();
    const BumpResult stiff = speedBump(garageCfg(s), 40, 0.08, 0.5);
    s.front.bumpFast = s.rear.bumpFast = 0.35 * s.front.bumpSlow;
    const BumpResult soft = speedBump(garageCfg(s), 40, 0.08, 0.5);
    std::printf("    8 cm bump at 40 km/h: peak body accel %.2f g (linear) vs %.2f g (fast bump 35%%)\n", stiff.peakVerticalAccelG,
                soft.peakVerticalAccelG);
    CHECK(soft.peakVerticalAccelG < stiff.peakVerticalAccelG);
}

TEST_CASE("garage: aero: rear wing downforce ~v^2 up to its stall, drag climbs; altitude thins it all") {
    auto aeroAt = [](const VehicleSetup& s, double kmh) {
        Vehicle v(applySetup(vehicles::referenceSportsRwd(), s), makePreset(DrivingPreset::Sport));
        FlatGround g;
        v.reset({0, 0, 0.6}, 0, g);
        v.setRollingSpeed(kmh / 3.6);
        DriverInput in;
        for (int i = 0; i < 360; ++i) {
            in.throttle = v.forwardSpeed() < kmh / 3.6 ? 0.8 : 0.0;
            v.step(1.0 / 360, in, g);
        }
        return v.telemetry().aero;
    };
    VehicleSetup s = base();
    s.rearWing = true;
    double prevDown = 0, prevDrag = 0, peak = 0;
    bool fell = false;
    for (double a : {0.0, 6.0, 12.0, 18.0, 24.0, 30.0}) {
        s.wingAngleDeg = a;
        const AeroTelemetry t = aeroAt(s, 200);
        std::printf("    wing %2.0f deg at 200 km/h: %5.0f N downforce, %5.0f N total drag%s\n", a, t.wingDownforce, t.drag,
                    t.wingStalled ? " (stalled)" : "");
        if (a > 0) CHECK(t.drag > prevDrag);
        if (t.wingDownforce < prevDown) fell = true;
        peak = std::max(peak, t.wingDownforce);
        prevDown = t.wingDownforce;
        prevDrag = t.drag;
    }
    CHECK(fell);              // past its stall the wing loses downforce
    CHECK_RANGE(peak, 900, 1800);
    s.wingAngleDeg = 12;
    const AeroTelemetry t100 = aeroAt(s, 100), t200 = aeroAt(s, 200);
    CHECK_NEAR(t200.wingDownforce / t100.wingDownforce, 4.0, 0.4);
    VehicleSetup high = s;
    high.altitudeM = 2500;
    high.temperatureC = 15;
    CHECK(aeroAt(high, 200).wingDownforce < 0.8 * t200.wingDownforce);
    CHECK(applySetup(vehicles::referenceSportsRwd(), high).drivetrain.engine.torqueCurve[3].torque <
          0.8 * vehicles::referenceSportsRwd().drivetrain.engine.torqueCurve[3].torque);
}

TEST_CASE("garage: aero: floor downforce rises as the car runs lower, then stalls; yaw and crosswind") {
    VehicleDefinition d = vehicles::referenceSportsRwd();
    d.aero.frontDownforceArea = 0.4;
    d.aero.rearDownforceArea = 0.5;
    d.aero.floorZ = d.centerOfMass.z - 0.3;
    d.aero.sideForceArea = 1.2;
    auto run = [&](double rideMm, double stall, Vec3 wind, double& clearance, double& yawOut, double& lateral) {
        VehicleSetup s = stockSetup(d);
        s.front.rideHeightMm = s.rear.rideHeightMm = rideMm;
        VehicleDefinition t = applySetup(d, s);
        Vehicle probe(t);
        FlatGround g;
        probe.reset({0, 0, 0.6}, 0, g);
        DriverInput in;
        for (int i = 0; i < 360; ++i) probe.step(1.0 / 360, in, g);
        t.aero.rideHeightRef = 0.5 * (probe.telemetry().aero.clearanceFront + probe.telemetry().aero.clearanceRear) - rideMm / 1000.0;
        t.aero.floorStallHeight = stall;
        Vehicle v(t, makePreset(DrivingPreset::Sport));
        v.reset({0, 0, 0.6}, 0, g);
        v.setWind(wind);
        v.setRollingSpeed(50);
        for (int i = 0; i < 360; ++i) {
            in.throttle = v.forwardSpeed() < 50 ? 0.9 : 0.0;
            v.step(1.0 / 360, in, g);
        }
        clearance = 0.5 * (v.telemetry().aero.clearanceFront + v.telemetry().aero.clearanceRear);
        yawOut = v.telemetry().aero.yaw;
        lateral = v.telemetry().lateralVelocity;
        return v.telemetry().aero.downforceFront + v.telemetry().aero.downforceRear;
    };
    double c0, c1, c2, y, lat;
    const double stock = run(0, 0, {}, c0, y, lat);
    const double low = run(-40, 0, {}, c1, y, lat);
    const double tooLow = run(-40, c1 + 0.02, {}, c2, y, lat);
    std::printf("    floor downforce at 180 km/h: stock %.0f N (%.0f mm), 40 mm lower %.0f N (%.0f mm), floor stalled %.0f N\n", stock,
                c0 * 1000, low, c1 * 1000, tooLow);
    CHECK(c1 < c0 - 0.03);
    CHECK(low > 1.1 * stock);
    CHECK(tooLow < 0.8 * low);
    double cw, yawW, latW;
    const double cross = run(0, 0, {0, 12, 0}, cw, yawW, latW);
    std::printf("    12 m/s crosswind from the right: airflow yaw %.1f deg, downforce %.0f N, car drifting %.2f m/s left\n",
                yawW * 57.3, cross, latW);
    CHECK(std::abs(yawW) > 0.15);
    CHECK(cross < stock);
}
