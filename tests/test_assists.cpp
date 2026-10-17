// OpenRoad Dynamics — driver-assist and preset tests.
// SPDX-License-Identifier: MIT
#include "ord/presets.hpp"
#include "ord_test.hpp"
#include "reference_vehicles.hpp"
#include "scenarios.hpp"

using namespace ord;
using namespace ord::scenarios;

TEST_CASE("presets: SIMULATION has every electronic and non-physical aid off") {
    const AssistConfig c = makePreset(DrivingPreset::Simulation);
    CHECK(!c.abs);
    CHECK(c.tractionControl == InterventionLevel::Off);
    CHECK(c.stabilityControl == InterventionLevel::Off);
    CHECK(c.countersteerAssist == 0);
    CHECK(c.steeringMode == SteeringMode::Direct);
    CHECK(c.steerRate == 0);
    CHECK(c.throttleRate == 0);
}

TEST_CASE("presets: aids are individually configurable (SIM steering + ABS, no TCS/ESC)") {
    RunConfig cfg;
    cfg.vehicle = vehicles::referenceSportsRwd();
    cfg.assists = makePreset(DrivingPreset::Simulation);
    cfg.assists.abs = true;
    const BrakeResult withAbs = brake100to0(cfg);
    cfg.assists.abs = false;
    const BrakeResult without = brake100to0(cfg);
    CHECK(withAbs.absActiveFraction > 0.5);
    CHECK(without.absActiveFraction == 0);
    CHECK(withAbs.distance < without.distance);
}

TEST_CASE("assists: with all aids off, the assist layer reports no interventions") {
    RunConfig cfg;
    cfg.vehicle = vehicles::referenceSportsRwd();
    cfg.assists = makePreset(DrivingPreset::Simulation);
    Vehicle v(cfg.vehicle, cfg.assists);
    FlatGround g;
    v.reset({0, 0, 0.6}, 0, g);
    DriverInput in;
    in.throttle = 1;
    bool any = false;
    for (int i = 0; i < 360 * 6; ++i) {
        in.steer = i > 360 * 3 ? 0.5 : 0.0;
        v.step(1.0 / 360, in, g);
        const AssistReport& r = v.telemetry().assists;
        any = any || r.tcsThrottleCut > 0 || r.countersteerAngle != 0;
        for (std::size_t w = 0; w < 4; ++w) any = any || r.absActive[w] || r.escBrakeTorque[w] > 0;
    }
    CHECK(!any);
}

TEST_CASE("assists: traction control limits wheelspin on a low-grip launch") {
    RunConfig cfg;
    cfg.vehicle = vehicles::referenceSportsRwd();
    cfg.surface.frictionScale = 0.5;
    cfg.assists = makePreset(DrivingPreset::Simulation);
    const AccelResult off = accelerate0to100(cfg);
    cfg.assists.tractionControl = InterventionLevel::Full;
    const AccelResult on = accelerate0to100(cfg);
    CHECK(on.peakDrivenSlip < 0.5 * off.peakDrivenSlip);
    CHECK(on.maxTcsCut > 0);
}

TEST_CASE("assists: ESC keeps an RWD car stable in sine-with-dwell where it is unstable without") {
    RunConfig cfg;
    cfg.vehicle = vehicles::referenceSportsRwd();
    cfg.assists = makePreset(DrivingPreset::Simulation);
    const Real amp = 9.0 * kPi / 180;
    const SineDwellResult off = sineWithDwell(cfg, amp);
    cfg.assists.stabilityControl = InterventionLevel::Full;
    const SineDwellResult on = sineWithDwell(cfg, amp);
    CHECK(!off.passesFmvss126Criteria());
    CHECK(on.passesFmvss126Criteria());
    CHECK(on.maxSideslip < 0.5 * off.maxSideslip);
}

TEST_CASE("assists: gamepad speed-sensitive steering limits full-stick angle at speed") {
    Vehicle v(vehicles::referenceSportsRwd(), makePreset(DrivingPreset::Fun, InputDevice::Gamepad));
    FlatGround g;
    v.reset({0, 0, 0.6}, 0, g);
    v.setRollingSpeed(kmhToMs(150));
    DriverInput in;
    in.steer = 1.0;
    in.throttle = 0.3;
    for (int i = 0; i < 360; ++i) v.step(1.0 / 360, in, g);
    const Real limit = v.telemetry().assists.steerLimitAngle;
    CHECK(limit < 0.2 * v.definition().wheels[0].maxSteerAngle + 0.12);
    CHECK(limit > 0.0);
    // A steering wheel in SIMULATION gets the full, unfiltered angle.
    Vehicle w(vehicles::referenceSportsRwd(), makePreset(DrivingPreset::Simulation, InputDevice::SteeringWheel));
    w.reset({0, 0, 0.6}, 0, g);
    w.setRollingSpeed(kmhToMs(150));
    in.steer = 0.1;
    w.step(1.0 / 360, in, g);
    CHECK_NEAR(w.telemetry().steerAngle, 0.1 * w.definition().wheels[0].maxSteerAngle, 1e-9);
}

TEST_CASE("presets: the vehicle definition is identical under every preset") {
    // Presets are AssistConfig only; this guards against future coupling.
    const VehicleDefinition d = vehicles::referenceSportsRwd();
    for (DrivingPreset p : {DrivingPreset::Fun, DrivingPreset::Sport, DrivingPreset::Simulation, DrivingPreset::Drift}) {
        Vehicle v(d, makePreset(p));
        CHECK(v.definition().wheels[0].tire.muY == d.wheels[0].tire.muY);
        CHECK(v.definition().mass == d.mass);
        CHECK(v.definition().inertia.z == d.inertia.z);
    }
}

// ---- Drift assist / DRIFT preset ----
// These are behaviour tests of an aid that has no real-car equivalent (the target is game
// feel, not a physical reference): steady inputs must give a steady, requested drift angle.

TEST_CASE("drift: DRIFT preset holds the requested angle with steady inputs, without spinning") {
    RunConfig cfg;
    cfg.vehicle = vehicles::referenceSportsRwd();
    cfg.assists = makePreset(DrivingPreset::Drift);
    const Real hold = ordtest::kOptimisedBuild ? 10.0 : 5.0;
    const DriftResult neutral = driftHold(cfg, 0.0, 0.8, 50.0, hold);
    const DriftResult wide = driftHold(cfg, 1.0, 0.8, 50.0, hold);
    for (const DriftResult* r : {&neutral, &wide}) {
        CHECK(r->entered);
        CHECK(!r->spun);
        CHECK(std::abs(r->meanSideslip - r->meanTarget) < 0.05);  // within ~3 deg of the request
        CHECK(r->sdSideslip < 0.04);
        CHECK(r->meanSideslip < 0);  // left-hand drift slides out to the right (ISO sideslip < 0)
    }
    // Centred steering = the neutral angle; steering into the turn widens it (up to the lock).
    CHECK_NEAR(neutral.meanTarget, -cfg.assists.driftAngleNeutral, 1e-6);
    CHECK(wide.meanSideslip < neutral.meanSideslip - 0.12);
}

TEST_CASE("drift: holds on the reference cars across throttle and steering positions") {
    if (!ordtest::kOptimisedBuild) return;  // 24 ten-second drifts: Release only
    int held = 0, total = 0, spun = 0;
    for (const VehicleDefinition& d : {vehicles::referenceSportsRwd(), vehicles::rosserStellaV8()}) {
        RunConfig cfg;
        cfg.vehicle = d;
        cfg.assists = makePreset(DrivingPreset::Drift);
        for (Real steer : {0.0, 0.5, 1.0})
            for (Real thr : {0.6, 0.8, 1.0})
                for (Real kmh : {40.0, 58.0}) {
                    const DriftResult r = driftHold(cfg, steer, thr, kmh, 8.0);
                    ++total;
                    spun += r.spun;
                    held += r.entered && !r.spun && std::abs(r.meanSideslip - r.meanTarget) < 0.07 && r.sdSideslip < 0.05;
                }
    }
    CHECK(spun == 0);
    CHECK(held >= total - 2);
}

TEST_CASE("drift: the drift assist only ever removes throttle and reports itself") {
    RunConfig cfg;
    cfg.vehicle = vehicles::referenceSportsRwd();
    cfg.assists = makePreset(DrivingPreset::Drift);
    const DriftResult r = driftHold(cfg, 1.0, 1.0, 50.0, ordtest::kOptimisedBuild ? 8.0 : 4.0);
    CHECK(r.entered);
    CHECK(r.maxThrottleAdded <= 1e-9);
    CHECK(r.meanTarget != 0);  // AssistReport carries the target it is holding
}

TEST_CASE("drift: with the drift assist off the car is not held (the aid is really gone)") {
    RunConfig cfg;
    cfg.vehicle = vehicles::referenceSportsRwd();
    cfg.assists = makePreset(DrivingPreset::Drift);
    cfg.assists.driftAssist = 0.0;
    const DriftResult r = driftHold(cfg, 1.0, 1.0, 50.0, 4.0);
    CHECK(r.meanTarget == 0);
    // Constant full lock and full throttle with no one catching it: it spins.
    CHECK(r.spun);
}

TEST_CASE("drift: the automatic gearbox holds its gear while the car slides under power") {
    RunConfig cfg;
    cfg.vehicle = vehicles::referenceSportsRwd();
    cfg.assists = makePreset(DrivingPreset::Drift);
    const DriftResult r = driftHold(cfg, 0.5, 1.0, 50.0, ordtest::kOptimisedBuild ? 8.0 : 4.0);
    CHECK(r.entered);
    CHECK(r.upshiftsWhileSliding == 0);
}

TEST_CASE("assists: speed-sensitive steering can reach full countersteer in a slide") {
    // The speed-sensitive range protects the front tires; it is measured from where the front
    // axle travels, so in a slide full opposite stick reaches that direction (or the lock).
    AssistSystem a(makePreset(DrivingPreset::Sport));
    VehicleSensors s;
    s.dt = 1.0 / 360;
    s.speed = kmhToMs(60);
    s.wheelbase = 2.55;
    s.cgToFrontAxle = 1.25;
    s.maxSteerAngle = 0.55;
    s.frictionEstimate = 1.1;
    s.yawRate = 0.8;                                      // rotating left...
    s.lateralVelocity = -std::tan(0.45) * s.speed;        // ...sliding out to the right (~26 deg)
    const Real frontTravel = std::atan2(s.lateralVelocity + s.cgToFrontAxle * s.yawRate, s.speed);
    CHECK(frontTravel < -0.2);                            // front axle heading well to the right
    DriverInput in;
    in.steer = -1.0;
    ActuatorCommands cmd;
    for (int i = 0; i < 360; ++i) cmd = a.process(in, s);  // let the rate limiter settle
    CHECK(cmd.steerAngle <= std::max(frontTravel, -s.maxSteerAngle) + 1e-9);
    // Into the turn the normal (tire-protecting) limit still applies.
    in.steer = 1.0;
    for (int i = 0; i < 360; ++i) cmd = a.process(in, s);
    CHECK(cmd.steerAngle <= a.report().steerLimitAngle + 1e-9);
    CHECK(a.report().steerLimitAngle < 0.25);
}
