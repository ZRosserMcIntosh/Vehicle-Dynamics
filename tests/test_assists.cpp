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
    in.throttle = 0.5;
    Real lastBeta = 0, donutAfterEase = 1;
    bool spun = false;
    for (int i = 0; i < 360 * 4; ++i) {
        // ease off the lock (a small drift is still asked for), then centre the steering
        in.steer = i < 360 ? 0.2 : 0.0;
        v->step(1.0 / 360, in, ground);
        if (i == 359) donutAfterEase = v->telemetry().assists.donut;
        lastBeta = v->telemetry().sideslip;
        spun = spun || std::abs(lastBeta) > 1.4;
    }
    CHECK(donutAfterEase < 0.05);
    std::printf("    eased off, then centred for 3 s: sideslip %.1f deg, yaw %.2f rad/s, %.0f km/h\n", lastBeta * 57.3,
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

namespace {
// Keyboard-style drift attempt, like the browser demo: digital steering ramped at 3/s toward full
// lock, from 60 km/h in the DRIFT preset. `hb` = handbrake window, `liftUntil` = throttle off
// until then (else pinned).
struct Attempt {
    Real firstTenDegAt = -1;  // s after turning in, |sideslip| > 10 deg
    bool kicked = false, spun = false;
    Real maxRpmClutchOpen = 0;
    int gearAtRelease = 0, minGearAfterRelease = 99;
    Real meanThrottleRatio = 0;  // throttle out / in over the held drift
};
Attempt keyboardAttempt(Real hbFrom, Real hbTo, Real liftUntil) {
    Vehicle v(vehicles::rosserStellaV8(), makePreset(DrivingPreset::Drift));
    FlatGround ground;
    v.reset({0, 0, 0.6}, 0, ground);
    v.setRollingSpeed(60 / 3.6);
    DriverInput in;
    Attempt a;
    Real kb = 0, ratio = 0;
    int n = 0;
    const Real dt = 1.0 / 360, limiter = v.definition().drivetrain.engine.limiterRpm;
    for (int i = 0; i < 360 * 5; ++i) {
        const Real t = i * dt;
        const Real target = t > 0.5 ? 1.0 : 0.0;
        kb += std::clamp(target - kb, -3 * dt, 3 * dt);
        in.steer = kb;
        in.throttle = t < 0.5 ? 0.3 : (t < liftUntil ? 0.0 : 1.0);
        in.handbrake = (t > hbFrom && t < hbTo) ? 1.0 : 0.0;
        v.step(dt, in, ground);
        const auto& tel = v.telemetry();
        a.kicked = a.kicked || tel.assists.clutchKick;
        if (a.firstTenDegAt < 0 && std::abs(tel.sideslip) > 0.1745) a.firstTenDegAt = t - 0.5;
        if (tel.assists.handbrakeClutch > 0.9) a.maxRpmClutchOpen = std::max(a.maxRpmClutchOpen, tel.drivetrain.engineRpm / limiter);
        if (hbTo > 0 && std::abs(t - hbTo) < dt / 2) a.gearAtRelease = tel.gear;
        if (hbTo > 0 && t > hbTo && t < hbTo + 0.6) a.minGearAfterRelease = std::min(a.minGearAfterRelease, tel.gear);
        if (t > 2.5 && in.throttle > 0.5) { ratio += tel.throttleOut / in.throttle; ++n; }
        a.spun = a.spun || std::abs(tel.sideslip) > 1.4;
    }
    a.meanThrottleRatio = n ? ratio / n : 0;
    return a;
}
}  // namespace

TEST_CASE("drift: turning in on the throttle breaks the rear loose (clutch kick), no push wide") {
    // Before: full steering and full throttle at 60 km/h pushed wide on the front tyres for ~0.6 s
    // before the rear let go. The DRIFT assist now kicks the clutch on turn-in, like a drifter.
    const Attempt a = keyboardAttempt(-1, -1, 0);
    std::printf("    keyboard turn-in at 60 km/h: kick %d, 10 deg of sideslip %.2f s after turning in\n", a.kicked, a.firstTenDegAt);
    CHECK(a.kicked);
    CHECK(a.firstTenDegAt > 0 && a.firstTenDegAt < 0.65);
    CHECK(!a.spun);
    // and in the drift the throttle still works: the engine isn't pinned on the limiter with half
    // the pedal taken away (before: ~50% of the throttle cut, stuck in 2nd)
    std::printf("    held drift, full throttle: %.0f%% of the pedal reaches the engine\n", 100 * a.meanThrottleRatio);
    CHECK(a.meanThrottleRatio > 0.75);
}

TEST_CASE("drift: handbrake with the throttle held keeps the revs off the limiter; no downshift on release") {
    // Handbrake + throttle: the clutch is open, so the engine free-revs. It is held just under
    // the limiter instead of bouncing off it.
    const Attempt held = keyboardAttempt(0.7, 1.1, 0);
    std::printf("    clutch open: peak %.0f%% of the limiter\n", 100 * held.maxRpmClutchOpen);
    CHECK(held.maxRpmClutchOpen < 0.93);
    CHECK(!held.spun);
    // Lift, handbrake, then power: the box used to drop a gear right after the release (the revs
    // were low only because the rear had been locked) and the shift's clutch gap killed the drift.
    const Attempt lift = keyboardAttempt(0.7, 1.1, 1.1);
    std::printf("    gear at release %d, lowest in the next 0.6 s %d\n", lift.gearAtRelease, lift.minGearAfterRelease);
    CHECK(lift.minGearAfterRelease >= lift.gearAtRelease);
    CHECK(!lift.spun);
}

TEST_CASE("Stella: a full-throttle 1-2 hands over without a torque spike, and the revs fall quickly") {
    // The oncoming clutch of the dual-clutch box carries what the rear tyres carried before the
    // shift and the ignition retards until the engine is down to second's speed. Snapping it shut
    // (the old behaviour, still the default for other cars) dumped the flywheel into the tyres.
    struct R { Real torquePeak, slipPeak; };
    auto run = [](UpshiftControl u) {
        VehicleDefinition d = vehicles::rosserStellaV8();
        d.drivetrain.gearbox.upshift = u;
        Vehicle v(d, makePreset(DrivingPreset::Sport));
        FlatGround g;
        v.reset({0, 0, 0.6}, 0, g);
        DriverInput in;
        in.throttle = 1;
        R r{0, 0};
        Real t = 0, tShift = -1;
        for (int i = 0; i < 360 * 5; ++i) {
            v.step(1.0 / 360, in, g);
            t += 1.0 / 360;
            const TelemetryFrame& f = v.telemetry();
            if (f.gear >= 2 && tShift < 0) tShift = t;
            // the handover (skipping the one-step input-shaft resync on the ratio change)
            if (tShift >= 0 && t - tShift > 0.01 && t - tShift < 0.3) {
                r.torquePeak = std::max(r.torquePeak, std::abs(f.drivetrain.driveshaftTorque));
                r.slipPeak = std::max(r.slipPeak, 0.5 * (f.wheels[2].slipRatio + f.wheels[3].slipRatio));
            }
        }
        return r;
    };
    const R snap = run(UpshiftControl::Snap), power = run(UpshiftControl::PowerShift);
    std::printf("    1-2 handover: drive torque peak %.0f -> %.0f N*m, rear slip peak %.3f -> %.3f (snap -> power shift)\n", snap.torquePeak, power.torquePeak, snap.slipPeak, power.slipPeak);
    CHECK(power.slipPeak < snap.slipPeak - 0.05);   // the tyres no longer flare up on the handover
    FlatGround g;

    // free rev with the clutch in: it used to take over 2 s to fall from 8,500 to 3,000 rpm
    AssistConfig c = makePreset(DrivingPreset::Simulation);
    c.autoClutch = false;
    Vehicle e(vehicles::rosserStellaV8(), c);
    e.reset({0, 0, 0.6}, 0, g);
    DriverInput r;
    r.clutch = 1;
    r.brake = 1;
    r.throttle = 1;
    for (int i = 0; i < 360 * 3 && e.telemetry().drivetrain.engineRpm < 8500; ++i) e.step(1.0 / 360, r, g);
    r.throttle = 0;
    const Real r0 = e.telemetry().drivetrain.engineRpm;
    Real tf = 0;
    while (e.telemetry().drivetrain.engineRpm > 3000 && tf < 5) { e.step(1.0 / 360, r, g); tf += 1.0 / 360; }
    std::printf("    free rev: %.0f -> 3,000 rpm in %.2f s (%.0f rpm/s)\n", r0, tf, (r0 - 3000) / tf);
    CHECK((r0 - 3000) / tf > 4000);
}

TEST_CASE("Stella Manuale: six gears and a real torque gap on the shift") {
    const VehicleDefinition d = vehicles::rosserStellaV8Manuale();
    CHECK(d.validate().empty());
    CHECK(d.drivetrain.gearbox.forwardRatios.size() == 6);
    Vehicle v(d, makePreset(DrivingPreset::Fun));
    FlatGround g;
    v.reset({0, 0, 0.6}, 0, g);
    DriverInput in;
    in.throttle = 1;
    Real minDuring = 1e9;
    int top = 1;
    for (int i = 0; i < 360 * 8; ++i) {
        v.step(1.0 / 360, in, g);
        const TelemetryFrame& f = v.telemetry();
        top = std::max(top, f.gear);
        if (f.assists.shifting && f.gear >= 2) minDuring = std::min(minDuring, std::abs(f.drivetrain.driveshaftTorque));
    }
    std::printf("    Manuale: reached gear %d, drive torque during a shift down to %.0f N*m\n", top, minDuring);
    CHECK(top >= 3);
    CHECK(minDuring < 30);   // the clutch is down while the lever goes across
}

TEST_CASE("Stella: a slow steering ramp at 200 km/h stays on its line (no lift-off snap)") {
    // the engine friction raised for the rev drop must not turn a lift in a fast corner into a spin
    Vehicle v(vehicles::rosserStellaV8(), makePreset(DrivingPreset::Simulation));
    FlatGround g;
    v.reset({0, 0, 0.6}, 0, g);
    const Real V = 200 / 3.6;
    v.setRollingSpeed(V);
    DriverInput in;
    Real iT = 0, worstBeta = 0, peakAy = 0;
    for (int i = 0; i < 360 * 24; ++i) {
        const TelemetryFrame& t = v.telemetry();
        iT = clamp(iT + (V - t.speed) * 0.002, -0.5, 1.0);
        in.throttle = clamp(0.6 * (V - t.speed) + iT, 0.0, 1.0);
        in.steer = std::min(1.0, 1.25 * i / (360.0 * 24));
        v.step(1.0 / 360, in, g);
        worstBeta = std::max(worstBeta, std::abs(v.telemetry().sideslip));
        peakAy = std::max(peakAy, std::abs(v.telemetry().accelBody.y) / kGravity);
    }
    std::printf("    200 km/h ramp: peak %.2f g, worst sideslip %.1f deg\n", peakAy, worstBeta * 180 / kPi);
    CHECK(worstBeta < 10 * kPi / 180);
    CHECK(peakAy > 0.95 && peakAy < 1.3);
}

TEST_CASE("Stella: stops like a 458 Italia (100-0 km/h ~33 m, 70-0 mph ~143 ft)") {
    // Published: Ferrari 32.5 m, auto motor und sport 33.0-33.3 m; Automobile Magazine 143 ft.
    // The Stella (1,180 kg, Cup-type tyres) with ABS, brakes applied at once.
    auto stop = [](Real kmh) {
        Vehicle v(vehicles::rosserStellaV8(), makePreset(DrivingPreset::Sport));
        FlatGround g;
        v.reset({0, 0, 0.6}, 0, g);
        DriverInput in;
        v.setRollingSpeed(kmh / 3.6);
        for (int i = 0; i < 360; ++i) { in.throttle = 0.15; v.step(1.0 / 360, in, g); }
        v.setRollingSpeed(kmh / 3.6);
        in.throttle = 0;
        in.brake = 1;
        const Vec3 p0 = v.state().position;
        Real t = 0;
        while (v.telemetry().speed > 0.05 && t < 30) { v.step(1.0 / 360, in, g); t += 1.0 / 360; }
        return std::hypot(v.state().position.x - p0.x, v.state().position.y - p0.y);
    };
    const Real a = stop(100), b = stop(112.65);
    std::printf("    100-0 km/h %.1f m, 70-0 mph %.1f m (%.0f ft)\n", a, b, b / 0.3048);
    CHECK(a > 31.5 && a < 36.0);
    CHECK(b / 0.3048 > 133 && b / 0.3048 < 150);
}

TEST_CASE("brakes: the discs heat in a hard stop, cool in the airflow; off by default") {
    FlatGround g;
    DriverInput in;
    Vehicle v(vehicles::rosserStellaV8(), makePreset(DrivingPreset::Sport));
    v.reset({0, 0, 0.6}, 0, g);
    v.setRollingSpeed(200 / 3.6);
    v.step(1.0 / 360, in, g);
    in.brake = 1;
    while (v.telemetry().speed > 60 / 3.6) v.step(1.0 / 360, in, g);
    const Real hot = v.telemetry().wheels[0].brakeTemperature, rear = v.telemetry().wheels[2].brakeTemperature;
    in.brake = 0;
    in.throttle = 0.3;
    for (int i = 0; i < 360 * 60; ++i) v.step(1.0 / 360, in, g);
    const Real cooled = v.telemetry().wheels[0].brakeTemperature;
    std::printf("    200->60 km/h: front disc %.0f C, rear %.0f C; after a minute driving on %.0f C\n", hot, rear, cooled);
    CHECK(hot > 350 && hot < 700);
    CHECK(rear > 150 && rear < hot);
    CHECK(cooled < 200);
    // the reference cars have no brake thermal model: ambient, and the baseline is untouched
    Vehicle r(vehicles::referenceSportsRwd(), makePreset(DrivingPreset::Sport));
    r.reset({0, 0, 0.6}, 0, g);
    r.setRollingSpeed(150 / 3.6);
    in = DriverInput{};
    in.brake = 1;
    for (int i = 0; i < 360 * 3; ++i) r.step(1.0 / 360, in, g);
    CHECK(std::abs(r.telemetry().wheels[0].brakeTemperature - 25.0) < 1e-9);
}
