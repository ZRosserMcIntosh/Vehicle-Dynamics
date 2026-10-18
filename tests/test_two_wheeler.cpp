// OpenRoad Dynamics — rideable two-wheeler tests (rider, powertrain, brakes, assists).
// SPDX-License-Identifier: MIT
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "ord/two_wheeler.hpp"
#include "ord_test.hpp"

using namespace ord;

namespace {

constexpr double kDt = 0.002;  // the browser's step

struct StepSteer {
    double t90 = -1, peak = 0, finalError = 0, recover = -1, yawGain = 0;
    bool fell = false;
};

// Hold `speed` with gentle throttle/brake, steer input `amount` from 1 s to 5 s.
StepSteer stepSteer(const TwoWheelerDef& def, double speed, double amount) {
    TwoWheeler bike(def);
    CHECK(bike.reset(0, 0, 0, speed));
    StepSteer r;
    TwoWheelerInput in;
    double target = 0;
    double prevYaw = bike.model().yawAngle(), yaw = 0;  // unwrapped heading change
    for (int i = 0; i < int(8.0 / kDt); ++i) {
        const double t = i * kDt;
        in.steer = (t > 1 && t < 5) ? amount : 0;
        const double v = bike.telemetry().speed > 0 ? bike.telemetry().speed : speed;
        in.throttle = clamp((speed - v) * 0.5, 0.0, 1.0);
        in.rearBrake = clamp((v - speed) * 0.3, 0.0, 1.0);
        bike.step(kDt, in);
        yaw += std::remainder(bike.model().yawAngle() - prevYaw, 2 * kPi);
        prevYaw = bike.model().yawAngle();
        const TwoWheelerTelemetry& tel = bike.telemetry();
        if (tel.fallen) {
            r.fell = true;
            break;
        }
        if (t > 1 && t < 5) {
            target = tel.leanTarget;
            if (r.t90 < 0 && std::abs(tel.lean) > 0.9 * std::abs(target)) r.t90 = t - 1;
            r.peak = std::max(r.peak, std::abs(tel.lean));
            r.finalError = tel.lean - target;
        }
        if (t > 4.9 && t < 5.0) r.yawGain = yaw;
        if (t > 5 && r.recover < 0 && std::abs(tel.lean) < 0.05) r.recover = t - 5;
    }
    std::printf("    %-10s %4.1f m/s: target %.2f rad, t90 %.2f s, overshoot %3.0f%%, error %+.3f, upright again %.2f s\n",
                def.name, speed, target, r.t90, target != 0 ? 100 * (r.peak / std::abs(target) - 1) : 0.0,
                r.finalError, r.recover);
    return r;
}

}  // namespace

TEST_CASE("two_wheeler: the rider leans each vehicle into the asked turn and back up, 3-35 m/s") {
    struct Case {
        TwoWheelerDef def;
        double speed;
    };
    const Case cases[] = {{benchmarkBicycle(), 3.0},  {benchmarkBicycle(), 6.0},  {benchmarkBicycle(), 10.0},
                          {electricScooter(), 3.0},   {electricScooter(), 6.0},   {sportMotorcycle(), 6.0},
                          {sportMotorcycle(), 10.0},  {sportMotorcycle(), 20.0},  {sportMotorcycle(), 35.0}};
    for (const Case& c : cases) {
        const StepSteer r = stepSteer(c.def, c.speed, 0.8);
        CHECK(!r.fell);
        CHECK(r.t90 > 0 && r.t90 < 2.5);
        CHECK(std::abs(r.finalError) < 0.1);
        CHECK(r.peak < 1.35 * 0.8 * c.def.rider.maxLean + 0.05);
        CHECK(r.recover > 0 && r.recover < 2.5);
        CHECK(r.yawGain > 0.1);  // + steer = left = yaw increases
    }
}

TEST_CASE("two_wheeler: counter-steer emerges: to turn left at speed the bars first go right") {
    TwoWheeler bike(sportMotorcycle());
    CHECK(bike.reset(0, 0, 0, 20.0));
    TwoWheelerInput in;
    in.steer = 0.6;  // left
    in.throttle = 0.05;
    double firstTorque = 0, firstSteer = 0;
    for (int i = 0; i < int(0.15 / kDt); ++i) {
        bike.step(kDt, in);
        if (i == 2) firstTorque = bike.telemetry().steerTorque;
        if (std::abs(bike.telemetry().steer) > std::abs(firstSteer)) firstSteer = bike.telemetry().steer;
    }
    for (int i = 0; i < int(2.0 / kDt); ++i) bike.step(kDt, in);
    std::printf("    first bar torque %+.1f N*m, early steer %+.4f rad, lean after 2 s %+.3f rad\n", firstTorque, firstSteer,
                bike.telemetry().lean);
    CHECK(firstTorque > 0);  // benchmark sign: toward right steer
    CHECK(firstSteer > 0);
    CHECK(bike.telemetry().lean < -0.2);  // leaning left
    CHECK(bike.telemetry().steer < 0.0);  // and steering (slightly) left in the turn
}

TEST_CASE("two_wheeler: balance assist holds a stopped bike up; without it the bike falls; brakes hold") {
    for (const TwoWheelerDef& def : {benchmarkBicycle(), electricScooter(), sportMotorcycle()}) {
        for (double assist : {1.0, 0.0}) {
            TwoWheelerAssists a;
            a.balanceAssist = assist;
            TwoWheeler bike(def, a);
            CHECK(bike.reset(0, 0, 0, 0));
            TwoWheelerInput in;
            in.steer = 0.4;  // a nudge
            for (int i = 0; i < int(0.5 / kDt); ++i) bike.step(kDt, in);
            in = {};
            in.frontBrake = in.rearBrake = 0.3;
            for (int i = 0; i < int(4.0 / kDt) && !bike.fallen(); ++i) bike.step(kDt, in);
            if (assist > 0) {
                CHECK(!bike.fallen());
                CHECK(std::abs(bike.telemetry().lean) < 0.02);
                CHECK(std::abs(bike.telemetry().speed) < 0.01);
            } else {
                CHECK(bike.fallen());
            }
        }
    }
}

TEST_CASE("two_wheeler: wheelies: full throttle loops the sport bike, wheelie assist holds the front up") {
    auto run = [](bool control, double pitchLimit, double& maxPitch, double& heldMin, double& heldMax) {
        TwoWheelerAssists a;
        a.wheelieControl = control;
        a.wheeliePitch = pitchLimit;
        TwoWheeler bike(sportMotorcycle(), a);
        CHECK(bike.reset(0, 0, 0, 5.0));
        TwoWheelerInput in;
        in.throttle = 1.0;
        maxPitch = 0;
        heldMin = 10;
        heldMax = -10;
        for (int i = 0; i < int(5.0 / kDt) && !bike.fallen(); ++i) {
            bike.step(kDt, in);
            maxPitch = std::max(maxPitch, bike.telemetry().pitch);
            if (i * kDt > 1.5) {
                heldMin = std::min(heldMin, bike.telemetry().pitch);
                heldMax = std::max(heldMax, bike.telemetry().pitch);
            }
        }
        return bike.fallen();
    };
    double maxP = 0, lo = 0, hi = 0;
    CHECK(run(false, 0.04, maxP, lo, hi));  // flips over backwards
    CHECK(maxP > 1.0);
    CHECK(!run(true, 0.4, maxP, lo, hi));
    std::printf("    wheelie assist at 0.40 rad: pitch %.2f..%.2f rad from 1.5 s to 5 s (peak %.2f)\n", lo, hi, maxP);
    CHECK(lo > 0.3 && hi < 0.6);
    CHECK(!run(true, 0.04, maxP, lo, hi));
    CHECK(maxP < 0.15);
}

TEST_CASE("two_wheeler: braking: ABS stops the motorcycle near 1 g; an unassisted grab of the front brake endos") {
    {
        TwoWheelerAssists a;
        a.abs = true;
        TwoWheeler bike(sportMotorcycle(), a);
        CHECK(bike.reset(0, 0, 0, 20.0));
        TwoWheelerInput in;
        in.frontBrake = in.rearBrake = 1.0;
        double t = 0;
        while (bike.model().forwardSpeed() > 0.3 && t < 6) {
            bike.step(kDt, in);
            t += kDt;
        }
        const double g = 20.0 / t / 9.81;
        std::printf("    ABS stop from 20 m/s: %.2f s (%.2f g)\n", t, g);
        CHECK(!bike.fallen());
        CHECK(g > 0.8 && g < 1.1);
    }
    for (bool control : {false, true}) {
        TwoWheelerAssists a;
        a.wheelieControl = control;
        TwoWheeler bike(sportMotorcycle(), a);
        CHECK(bike.reset(0, 0, 0, 11.0));
        TwoWheelerInput in;
        in.frontBrake = 0.6;
        double minPitch = 0;
        for (int i = 0; i < int(3.0 / kDt) && !bike.fallen(); ++i) {
            bike.step(kDt, in);
            minPitch = std::min(minPitch, bike.telemetry().pitch);
        }
        std::printf("    front brake 0.6 at 11 m/s, stoppie control %s: min pitch %.2f rad%s\n", control ? "on " : "off",
                    minPitch, bike.fallen() ? " (over the bars)" : "");
        if (control) {
            CHECK(!bike.fallen());
            CHECK(minPitch > -0.25);
        } else {
            CHECK(bike.fallen());
        }
    }
}

TEST_CASE("two_wheeler: e-scooter limiter, pedalling speed, uphill on uneven ground, determinism") {
    {
        TwoWheeler s(electricScooter());
        CHECK(s.reset(0, 0, 0, 0));
        TwoWheelerInput in;
        in.throttle = 1.0;
        for (int i = 0; i < int(20.0 / kDt); ++i) s.step(kDt, in);
        std::printf("    e-scooter full throttle 20 s: %.1f km/h\n", s.telemetry().speed * 3.6);
        CHECK(s.telemetry().speed * 3.6 > 23.0 && s.telemetry().speed * 3.6 < 25.6);
    }
    {
        // 6% grade from x = 5 m: the bicycle climbs it (rear contact rises).
        const TestTrackGround hill([](Real x, Real) { return x > 5 ? 0.06 * (x - 5) : 0.0; });
        TwoWheeler b(benchmarkBicycle());
        CHECK(b.reset(0, 0, 0, 5.0, &hill));
        TwoWheelerInput in;
        in.throttle = 1.0;
        for (int i = 0; i < int(6.0 / kDt); ++i) b.step(kDt, in, &hill);
        const Vec3 p = b.model().position();
        std::printf("    bicycle up a 6%% grade: x %.1f m, z %.2f m (ground %.2f m), %.1f m/s\n", p.x, p.z,
                    0.06 * (p.x - 5), b.telemetry().speed);
        CHECK(!b.fallen());
        CHECK(p.x > 20);
        CHECK_NEAR(p.z, 0.06 * (p.x - 5), 0.05);
    }
    {
        // Same inputs, same result, bit for bit (lockstep multiplayer, replays).
        TwoWheeler a(sportMotorcycle()), b(sportMotorcycle());
        CHECK(a.reset(1, 2, 0.3, 12) && b.reset(1, 2, 0.3, 12));
        TwoWheelerInput in;
        for (int i = 0; i < 1500; ++i) {
            in.steer = std::sin(i * 0.004);
            in.throttle = 0.3;
            a.step(kDt, in);
            b.step(kDt, in);
        }
        CHECK(std::memcmp(&a.model().body(1).position, &b.model().body(1).position, sizeof(Vec3)) == 0);
        CHECK(!a.fallen());
    }
}

TEST_CASE("two_wheeler: Direct mode: the input is handlebar torque (push left bar, go left)") {
    TwoWheelerAssists a;
    a.mode = RiderMode::Direct;
    TwoWheeler bike(sportMotorcycle(), a);
    CHECK(bike.reset(0, 0, 0, 20.0));
    TwoWheelerInput in;
    in.steer = -0.3;  // torque toward right steer -> lean left
    in.throttle = 0.05;
    for (int i = 0; i < int(0.6 / kDt); ++i) bike.step(kDt, in);
    std::printf("    0.6 s of +%.0f N*m bar torque: lean %+.3f rad\n", 0.3 * sportMotorcycle().rider.maxSteerTorque,
                bike.telemetry().lean);
    CHECK_NEAR(bike.telemetry().steerTorque, 0.3 * sportMotorcycle().rider.maxSteerTorque, 1e-9);
    CHECK(bike.telemetry().lean < -0.05);
}

TEST_CASE("two_wheeler: rough random riding with the Fun aids rarely ends in a crash (and never blows up)") {
    // Keyboard-style input: full left / centre / full right held 0.3-1.8 s,
    // throttle on or off, sometimes a brake grab, for 30 s; the motorcycle
    // is kept under ~110 km/h (above that, full-lock flicks at full throttle
    // do crash it). Any state must stay finite, and crashes must be rare.
    const int seeds = ordtest::kOptimisedBuild ? 12 : 3;
    for (int kind = 0; kind < 3; ++kind) {
        int falls = 0;
        for (int seed = 0; seed < seeds; ++seed) {
            const TwoWheelerDef def = kind == 0 ? benchmarkBicycle() : kind == 1 ? electricScooter() : sportMotorcycle();
            TwoWheelerAssists a;
            a.abs = a.tractionControl = a.wheelieControl = true;
            TwoWheeler bike(def, a);
            CHECK(bike.reset(0, 0, 0, 0));
            std::uint64_t rng = 0x9E3779B97F4A7C15ULL * static_cast<std::uint64_t>(seed + 1 + 100 * kind);
            auto uniform = [&] {
                rng ^= rng << 13; rng ^= rng >> 7; rng ^= rng << 17;
                return static_cast<double>(rng >> 11) * (1.0 / 9007199254740992.0);
            };
            TwoWheelerInput in;
            double key = 0, next = 0, held = 0, throttle = 0, brake = 0;
            for (int i = 0; i < int(30.0 / kDt) && !bike.fallen(); ++i) {
                const double t = i * kDt;
                if (t > next) {
                    const double r = uniform();
                    key = r < 0.33 ? -1 : r < 0.66 ? 1 : 0;
                    next = t + 0.3 + 1.5 * uniform();
                    throttle = uniform() < 0.75 ? (kind == 2 ? 0.3 + 0.7 * uniform() : 1.0) : 0.0;
                    brake = throttle == 0 && uniform() < 0.5 ? uniform() : 0.0;
                }
                const double rate = (key == 0 || (key > 0) != (held > 0)) ? 6 : 3;
                held += clamp(key - held, -rate * kDt, rate * kDt);
                in.steer = held;
                in.throttle = (kind == 2 && bike.telemetry().speed > 30) ? 0 : throttle;
                in.frontBrake = brake;
                in.rearBrake = 0.6 * brake;
                bike.step(kDt, in);
                CHECK(std::isfinite(bike.telemetry().speed) && std::abs(bike.telemetry().speed) < 120);
            }
            if (bike.fallen()) ++falls;
        }
        std::printf("    %-10s %d crash(es) in %d rough 30 s rides\n", kind == 0 ? "bicycle" : kind == 1 ? "e-scooter" : "motorcycle",
                    falls, seeds);
        CHECK(falls <= seeds / 6);
    }
}
