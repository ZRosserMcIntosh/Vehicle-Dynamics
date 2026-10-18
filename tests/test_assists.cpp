// OpenRoad Dynamics — driver-assist and preset tests.
// SPDX-License-Identifier: MIT
#include <algorithm>
#include <cmath>
#include <cstdio>

#include "ord/ground.hpp"
#include "ord/presets.hpp"
#include "ord/vehicle.hpp"
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
    const DriftResult half = driftHold(cfg, 0.5, 0.8, 50.0, hold);
    const DriftResult full = driftHold(cfg, 1.0, 0.8, 50.0, hold);
    for (const DriftResult* r : {&half, &full}) {
        CHECK(r->entered);
        CHECK(!r->spun);
        CHECK(std::abs(r->meanSideslip - r->meanTarget) < 0.05);  // within ~3 deg of the request
        CHECK(r->sdSideslip < 0.04);
        CHECK(r->meanSideslip < 0);  // left-hand drift slides out to the right (ISO sideslip < 0)
    }
    // Full steering into the turn asks for the largest angle the lock can catch; half asks for less.
    CHECK_NEAR(full.meanTarget, -std::min(cfg.assists.driftAngleMax, cfg.vehicle.wheels[0].maxSteerAngle), 1e-6);
    CHECK(half.meanSideslip > full.meanSideslip + 0.12);
}

TEST_CASE("drift: centring the steering straightens the car out of the drift, without a swing back") {
    // "Steer where you want to go": once the car points where the player wants, letting go of
    // the steering ends the drift. Before, centred held a 20 deg drift and the car kept rotating.
    RunConfig cfg;
    cfg.vehicle = vehicles::referenceSportsRwd();
    cfg.assists = makePreset(DrivingPreset::Drift);
    const DriftResult r = driftHold(cfg, 0.0, 0.8, 50.0, 4.0);
    CHECK(r.entered);
    CHECK(!r.spun);
    CHECK(std::abs(r.meanSideslip) < 0.05);  // straight (window starts 2 s after the entry)
    CHECK(r.maxOppositeSideslip < 0.09);     // no pendulum to the other side (< 5 deg)
}

TEST_CASE("drift: holds on the reference cars across throttle and steering positions") {
    if (!ordtest::kOptimisedBuild) return;  // 24 ten-second drifts: Release only
    int held = 0, total = 0, spun = 0;
    for (const VehicleDefinition& d : {vehicles::referenceSportsRwd(), vehicles::rosserStellaV8()}) {
        RunConfig cfg;
        cfg.vehicle = d;
        cfg.assists = makePreset(DrivingPreset::Drift);
        for (Real steer : {0.5, 0.75, 1.0})
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

TEST_CASE("drift: the automatic gearbox holds its gear in a slide, shifting up only off the limiter") {
    // An upshift that drops the revs straightens the car mid-drift, so the box holds its gear.
    // Held on the limiter, though, more throttle can no longer add angle or speed: from there it
    // shifts clutch-to-clutch (drive continues) and the drift carries on through the shift.
    RunConfig cfg;
    cfg.vehicle = vehicles::referenceSportsRwd();
    cfg.assists = makePreset(DrivingPreset::Drift);
    const DriftResult r = driftHold(cfg, 0.5, 1.0, 50.0, ordtest::kOptimisedBuild ? 8.0 : 4.0);
    CHECK(r.entered && !r.spun);
    CHECK(r.upshiftsWhileSliding <= 2);
    CHECK(std::abs(r.meanSideslip - r.meanTarget) < 0.05);
    CHECK(r.sdSideslip < 0.04);
    // part throttle too: whatever shifts happen, the drift holds through them
    const DriftResult part = driftHold(cfg, 0.5, 0.7, 45.0, ordtest::kOptimisedBuild ? 6.0 : 3.0);
    CHECK(part.entered && !part.spun);
    CHECK(std::abs(part.meanSideslip - part.meanTarget) < 0.05);
}

namespace {
struct Spin {
    Real meanYaw = 0, meanSpeedKmh = 0, maxSpeedKmh = 0, meanDonut = 0;
};
// Full lock and full throttle from a standstill (or rolling at `kmh`) for `seconds`, averaged over the last half.
Spin fullLockFullThrottle(const VehicleDefinition& d, const AssistConfig& a, Real kmh, Real seconds, Vehicle** keep = nullptr) {
    static FlatGround ground;
    auto* v = new Vehicle(d, a);
    v->reset({0, 0, 0.6}, 0, ground);
    if (kmh > 0) v->setRollingSpeed(kmh / 3.6);
    DriverInput in;
    in.steer = 1;
    in.throttle = 1;
    Spin r;
    int n = 0;
    for (int i = 0; i < static_cast<int>(seconds * 360); ++i) {
        v->step(1.0 / 360, in, ground);
        const auto& t = v->telemetry();
        const Real kmhNow = std::hypot(t.speed, t.lateralVelocity) * 3.6;
        if (i > seconds * 180) {
            r.meanYaw += t.yawRate;
            r.meanSpeedKmh += kmhNow;
            r.meanDonut += t.assists.donut;
            r.maxSpeedKmh = std::max(r.maxSpeedKmh, kmhNow);
            ++n;
        }
    }
    r.meanYaw /= n;
    r.meanSpeedKmh /= n;
    r.meanDonut /= n;
    if (keep) *keep = v;
    else delete v;
    return r;
}
}  // namespace

TEST_CASE("drift: full lock and throttle from low speed spins donuts; the assist steps aside") {
    // Before, the drift assist read full lock as "hold ~31 deg" and countersteered against it:
    // the car ran a wide 60 km/h drift circle instead of a donut.
    for (const VehicleDefinition& d : {vehicles::referenceSportsRwd(), vehicles::rosserStellaV8()}) {
        for (Real kmh : {0.0, 20.0}) {
            const Spin drift = fullLockFullThrottle(d, makePreset(DrivingPreset::Drift), kmh, 8.0);
            const Spin sim = fullLockFullThrottle(d, makePreset(DrivingPreset::Simulation), kmh, 8.0);
            std::printf("    %s from %2.0f km/h: DRIFT yaw %.2f rad/s at %.0f km/h (donut %.2f) | SIMULATION yaw %.2f rad/s at %.0f km/h\n",
                        d.name.c_str(), kmh, drift.meanYaw, drift.meanSpeedKmh, drift.meanDonut, sim.meanYaw, sim.meanSpeedKmh);
            CHECK(drift.meanYaw > 1.2);       // a donut: rotating >70 deg/s ...
            CHECK(drift.maxSpeedKmh < 30);    // ... on the spot, not a wide drift circle
            CHECK(drift.meanDonut > 0.9);
            CHECK(sim.meanYaw > 1.2);         // (SIMULATION, no aids, for comparison)
        }
    }
}

TEST_CASE("drift: easing off the lock ends a donut; the assist catches it without a spin") {
    Vehicle* v = nullptr;
    fullLockFullThrottle(vehicles::referenceSportsRwd(), makePreset(DrivingPreset::Drift), 0.0, 6.0, &v);
    FlatGround ground;
    DriverInput in;
    in.steer = 0.2;
    in.throttle = 0.5;
    Real lastBeta = 0;
    bool spun = false;
    for (int i = 0; i < 360 * 4; ++i) {
        v->step(1.0 / 360, in, ground);
        lastBeta = v->telemetry().sideslip;
        spun = spun || std::abs(lastBeta) > 1.4;
    }
    std::printf("    4 s after easing off: sideslip %.1f deg, yaw %.2f rad/s, %.0f km/h\n", lastBeta * 57.3,
                v->telemetry().yawRate, v->telemetry().speed * 3.6);
    CHECK(!spun);
    CHECK(std::abs(lastBeta) < 0.1);
    CHECK(v->telemetry().assists.donut < 0.05);
    delete v;
}

TEST_CASE("drift: the handbrake locks the rear even with the throttle held (auto clutch opens)") {
    // A drifter puts the clutch in while pulling the handbrake. Before, the auto clutch stayed
    // engaged and the engine kept the rear wheels turning: handbrake + throttle did nothing.
    for (DrivingPreset p : {DrivingPreset::Drift, DrivingPreset::Simulation, DrivingPreset::Fun}) {
        Vehicle v(vehicles::rosserStellaV8(), makePreset(p));
        FlatGround ground;
        v.reset({0, 0, 0.6}, 0, ground);
        v.setRollingSpeed(50 / 3.6);
        DriverInput in;
        in.throttle = 1;
        in.steer = 0.3;
        Real minRear = 1e9, clutch = 1, hbClutch = 0;
        int gearAtPull = 0;
        bool shiftedDuring = false;
        for (int i = 0; i < 360; ++i) {
            in.handbrake = i > 36 ? 1 : 0;
            v.step(1.0 / 360, in, ground);
            const auto& t = v.telemetry();
            if (i == 37) gearAtPull = t.gear;
            if (i > 140) {
                minRear = std::min(minRear, std::abs(t.wheels[2].omega));
                clutch = std::min(clutch, t.clutchEngagement);
                hbClutch = std::max(hbClutch, t.assists.handbrakeClutch);
                shiftedDuring = shiftedDuring || t.gear != gearAtPull;
            }
        }
        CHECK(minRear < 0.5);   // rear wheels locked
        CHECK(clutch < 0.01);
        CHECK(hbClutch > 0.99);
        CHECK(!shiftedDuring);  // the free-revving engine does not upshift with the clutch open
    }
    // A front-driven car's handbrake does not act on a driven wheel: its clutch stays in.
    Vehicle fwd(vehicles::referenceHatchFwd(), makePreset(DrivingPreset::Drift));
    FlatGround ground;
    fwd.reset({0, 0, 0.6}, 0, ground);
    fwd.setRollingSpeed(50 / 3.6);
    DriverInput in;
    in.throttle = 1;
    in.handbrake = 1;
    for (int i = 0; i < 180; ++i) fwd.step(1.0 / 360, in, ground);
    CHECK(fwd.telemetry().assists.handbrakeClutch == 0);
}

TEST_CASE("drift: throttle is part of the request: less power, less angle") {
    RunConfig cfg;
    cfg.vehicle = vehicles::rosserStellaV8();
    cfg.assists = makePreset(DrivingPreset::Drift);
    const Real hold = ordtest::kOptimisedBuild ? 6.0 : 3.0;
    const DriftResult full = driftHold(cfg, 0.75, 1.0, 50.0, hold);
    const DriftResult part = driftHold(cfg, 0.75, 0.5, 50.0, hold);
    std::printf("    3/4 steering: throttle 100%% holds %.1f deg, 50%% %.1f deg\n", full.meanSideslip * 57.3, part.meanSideslip * 57.3);
    CHECK(full.entered && part.entered && !full.spun && !part.spun);
    CHECK(std::abs(part.meanTarget) < 0.9 * std::abs(full.meanTarget));
    CHECK(std::abs(part.meanSideslip) < std::abs(full.meanSideslip) - 0.03);
}

TEST_CASE("drift: a handbrake entry settles into the drift without swinging back") {
    // Before: the drift cut took all the throttle during the flick and the assist held full
    // countersteer as the clutch came back in, so the angle collapsed from ~21 to ~3 deg (and
    // the car rotated the other way) before the drift was rebuilt.
    Real worstDrop = 0;
    for (const VehicleDefinition& d : {vehicles::referenceSportsRwd(), vehicles::rosserStellaV8()})
        for (Real steer : {0.4, 0.6, 0.8})
            for (Real kmh : {45.0, 70.0}) {
                Vehicle v(d, makePreset(DrivingPreset::Drift));
                FlatGround ground;
                v.reset({0, 0, 0.6}, 0, ground);
                v.setRollingSpeed(kmh / 3.6);
                DriverInput in;
                Real target = 0, peak = 0, minAfter = 1e9;
                bool pastPeak = false;
                for (int i = 0; i < 360 * 3; ++i) {
                    const Real t = i / 360.0;
                    in.steer = t < 0.2 ? 0 : steer;
                    in.throttle = 1;
                    in.handbrake = (t > 0.3 && t < 0.8) ? 1 : 0;
                    v.step(1.0 / 360, in, ground);
                    const auto& tel = v.telemetry();
                    const Real b = std::abs(tel.sideslip);
                    if (tel.assists.driftTargetSideslip != 0) target = std::abs(tel.assists.driftTargetSideslip);
                    if (t > 0.8 && t < 2.5) {
                        if (b > peak) peak = b;
                        else if (peak - b > 0.02) pastPeak = true;
                        if (pastPeak) minAfter = std::min(minAfter, b);
                    }
                    CHECK(b < 1.4);
                }
                if (minAfter < 1e8) worstDrop = std::max(worstDrop, target - minAfter);
            }
    std::printf("    worst dip below the requested angle after a handbrake entry: %.1f deg\n", worstDrop * 57.3);
    CHECK(worstDrop < 0.14);  // < 8 deg (was ~14; now ~7)
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

TEST_CASE("drift: a longer steering lock (angle kit) holds deeper drifts without spinning") {
    // Stock lock (0.55 rad) caps a catchable drift near 32 deg. A drift car's
    // ~0.9 rad lock lets the same aid hold ~50 deg at full steering.
    RunConfig cfg;
    cfg.vehicle = vehicles::referenceSportsRwd();
    cfg.assists = makePreset(DrivingPreset::Drift);
    const Real hold = ordtest::kOptimisedBuild ? 8.0 : 4.0;
    const DriftResult stock = driftHold(cfg, 1.0, 0.8, 50.0, hold);
    for (auto& w : cfg.vehicle.wheels) w.maxSteerAngle *= 0.9 / 0.55;
    const DriftResult kit = driftHold(cfg, 1.0, 0.8, 50.0, hold);
    std::printf("    full-steer drift: stock lock %.1f deg, 0.9 rad lock %.1f deg (sd %.1f deg)\n",
                stock.meanSideslip * 180 / kPi, kit.meanSideslip * 180 / kPi, kit.sdSideslip * 180 / kPi);
    CHECK(kit.entered && !kit.spun);
    CHECK(kit.meanSideslip < stock.meanSideslip - 0.25);  // >14 deg deeper
    CHECK(std::abs(kit.meanSideslip - kit.meanTarget) < 0.05);
}
