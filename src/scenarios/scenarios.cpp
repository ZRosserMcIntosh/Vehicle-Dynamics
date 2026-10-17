// OpenRoad Dynamics — reproducible benchmark scenarios.
// SPDX-License-Identifier: MIT
#include "scenarios.hpp"

#include <chrono>
#include <ostream>

#include "ord/ground.hpp"
#include "ord/presets.hpp"
#include "ord/telemetry.hpp"

namespace ord::scenarios {

namespace {

struct Sim {
    TestTrackGround ground;
    CountingGroundQuery counted;
    GroundPrefetchCache cache;
    Vehicle vehicle;
    Real dt;
    std::ostream* csv;
    bool prefetch;
    Real frameDt;
    Real frameAcc = 0;
    RunStats* stats;
    std::vector<GroundStripRequest> requests;

    int substeps = 1;
    bool host = false;

    Sim(const RunConfig& cfg, TestTrackGround::HeightFn h = nullptr, Real yaw = 0, Vec3 start = {0, 0, 0.6},
        const std::vector<TestTrackGround::Patch>& patches = {})
        : ground(std::move(h), cfg.surface),
          counted(ground),
          cache(cfg.prefetchSettings),
          vehicle(cfg.vehicle, cfg.assists, cfg.fidelity),
          dt(1.0 / cfg.hz),
          csv(cfg.csv),
          prefetch(cfg.prefetch),
          frameDt(1.0 / cfg.frameHz),
          stats(cfg.stats) {
        for (const auto& p : patches) ground.addPatch(p);
        start.z += ground.heightAt(start.x, start.y);
        vehicle.reset(start, yaw, ground);
        if (cfg.hostHz > 0) {
            host = true;
            substeps = std::max(1, static_cast<int>(std::lround(cfg.hz / cfg.hostHz)));
            dt = 1.0 / cfg.hostHz;
            vehicle.setChassisMode(ChassisMode::External, true);
        }
        if (csv) writeCsvHeader(*csv, cfg.vehicle.wheels.size());
        if (stats) *stats = RunStats{};
    }
    // Host rigid-body integrator with a different scheme than the core
    // (trapezoidal position, exponential-map rotation), host-applied gravity.
    void hostIntegrate() {
        const VehicleDefinition& d = vehicle.definition();
        RigidBodyState s = vehicle.state();
        const ChassisForces& f = vehicle.chassisForces();
        const Vec3 com = s.position + s.orientation.rotate(d.centerOfMass);
        const Vec3 vOld = s.velocity;
        s.velocity += (f.force + Vec3{0, 0, -kGravity * d.mass}) * (dt / d.mass);
        const Mat3 R = s.orientation.toMatrix();
        const Mat3 inv = R * Mat3::diagonal({1 / d.inertia.x, 1 / d.inertia.y, 1 / d.inertia.z}) * R.transposed();
        const Mat3 I = R * Mat3::diagonal(d.inertia) * R.transposed();
        const Vec3 w0 = s.angularVelocity;
        s.angularVelocity += inv * (f.torque - cross(w0, I * w0)) * dt;
        const Vec3 newCom = com + (vOld + s.velocity) * (0.5 * dt);
        const Real ang = length(s.angularVelocity) * dt;
        if (ang > 1e-12) s.orientation = Quat::fromAxisAngle(s.angularVelocity, ang) * s.orientation;
        s.orientation.normalize();
        s.position = newCom - s.orientation.rotate(d.centerOfMass);
        vehicle.setChassisState(s);
    }
    const GroundQuery& query() const {
        if (prefetch) return cache;
        return counted;
    }
    void step(const DriverInput& in) {
        if (prefetch) {
            frameAcc -= dt;
            if (frameAcc <= 1e-12) {
                // New render frame: the "game thread" refreshes the strips.
                requests.clear();
                vehicle.groundPrefetchRequests(requests);
                cache.rebuild(counted, requests.data(), requests.size());
                frameAcc += frameDt;
                if (stats) {
                    ++stats->frames;
                    stats->roughStrips += cache.roughStripsLastRebuild();
                }
            }
        }
        const std::uint64_t q0 = vehicle.groundQueries();
        if (host) {
            vehicle.stepSubstepped(dt, substeps, in, query());
            hostIntegrate();
        } else {
            vehicle.step(dt, in, query());
        }
        if (stats) {
            stats->vehicleQueries += vehicle.groundQueries() - q0;
            stats->engineRays = counted.count();
            stats->cacheMisses = cache.misses();
            stats->simulatedSeconds += dt;
        }
        if (csv) writeCsvRow(*csv, vehicle.telemetry());
    }
    Vec3 com() const { return vehicle.centerOfMassWorld(); }
};

Real maxSteerRef(const VehicleDefinition& d) {
    Real m = 0;
    for (const auto& w : d.wheels) m = std::max(m, w.maxSteerAngle);
    return m;
}

// Picks a gear giving a mid-range engine speed and sets the car rolling.
void prepareRolling(Vehicle& v, Real speed) {
    const auto& e = v.definition().drivetrain.engine;
    int best = 1;
    for (int g = 1; g <= v.drivetrain().forwardGearCount(); ++g) {
        v.assists().setGear(g);
        v.setRollingSpeed(speed);
        const Real rpm = radSToRpm(v.drivetrain().engineOmegaForGear(g));
        if (rpm >= 0.35 * e.limiterRpm && rpm <= 0.8 * e.limiterRpm) { best = g; break; }
        if (rpm >= 0.35 * e.limiterRpm) best = g;
    }
    v.assists().setGear(best);
    v.setRollingSpeed(speed);
}

// Proportional speed-holding throttle/brake.
void holdSpeed(DriverInput& in, Real speed, Real target) {
    const Real err = target - speed;
    in.throttle = clamp(0.25 + 0.4 * err, 0.0, 1.0);
    in.brake = err < -1.5 ? clamp(-0.1 * err, 0.0, 1.0) : 0.0;
}

RunConfig withAutoShift(const RunConfig& cfg) {
    RunConfig c = cfg;
    c.assists.shiftMode = ShiftMode::Automatic;  // scripted manoeuvres use automatic shifting
    c.assists.autoClutch = true;
    return c;
}

RunConfig withDirectSteering(const RunConfig& cfg) {
    RunConfig c = withAutoShift(cfg);
    // Path-following benchmarks drive the road-wheel angle directly so that
    // results measure the car, not the input filter.
    c.assists.steeringMode = SteeringMode::Direct;
    c.assists.steerRate = 0;
    c.assists.steeringExponent = 1.0;
    c.assists.countersteerAssist = 0;
    return c;
}

}  // namespace

AccelResult accelerate0to100(const RunConfig& cfgIn) {
    const RunConfig cfg = withAutoShift(cfgIn);
    Sim sim(cfg);
    AccelResult r;
    DriverInput in;
    in.throttle = 1.0;
    Real t = 0;
    while (t < 40.0) {
        sim.step(in);
        t += sim.dt;
        const auto& tel = sim.vehicle.telemetry();
        for (std::size_t w = 0; w < tel.wheelCount; ++w)
            if (tel.wheels[w].contact && tel.speed > 1.0)
                r.peakDrivenSlip = std::max(r.peakDrivenSlip, tel.wheels[w].slipRatio);
        r.maxTcsCut = std::max(r.maxTcsCut, tel.assists.tcsThrottleCut);
        if (r.time0to60 == 0 && tel.speed >= kmhToMs(60)) r.time0to60 = t;
        if (tel.speed >= kmhToMs(100)) {
            r.reached = true;
            r.time0to100 = t;
            break;
        }
    }
    return r;
}

BrakeResult brake100to0(const RunConfig& cfgIn, Real startKmh) {
    const RunConfig cfg = withAutoShift(cfgIn);
    Sim sim(cfg);
    const Real v0 = kmhToMs(startKmh);
    prepareRolling(sim.vehicle, v0);
    BrakeResult r;
    DriverInput in;
    in.brake = 1.0;
    Real t = 0, dist = 0;
    int absSteps = 0, steps = 0;
    const Vec3 p0 = sim.com();
    while (t < 20.0) {
        sim.step(in);
        t += sim.dt;
        ++steps;
        const auto& tel = sim.vehicle.telemetry();
        r.maxAbsSideslip = std::max(r.maxAbsSideslip, std::abs(tel.sideslip));
        bool abs = false;
        for (std::size_t w = 0; w < tel.wheelCount; ++w) {
            abs = abs || tel.assists.absActive[w];
            if (tel.speed > 2.0 && tel.wheels[w].contact && std::abs(tel.wheels[w].omega) < 0.05) r.anyWheelLocked = true;
        }
        absSteps += abs;
        if (tel.speed < 0.05) {
            r.stopped = true;
            break;
        }
    }
    const Vec3 d = sim.com() - p0;
    dist = std::hypot(d.x, d.y);
    r.distance = dist;
    r.time = t;
    r.meanDecelG = dist > 0 ? v0 * v0 / (2 * dist) / kGravity : 0;
    r.absActiveFraction = steps ? static_cast<Real>(absSteps) / steps : 0;
    return r;
}

SkidpadResult skidpad(const RunConfig& cfgIn, Real radius) {
    const RunConfig cfg = withDirectSteering(cfgIn);
    Sim sim(cfg);
    SkidpadResult res;
    res.radius = radius;
    const Real L = cfg.vehicle.wheelbase();
    const Real maxSteer = maxSteerRef(cfg.vehicle);
    const Vec3 c{0, radius, 0};  // counter-clockwise circle (left turn) starting at the origin heading +X
    Real target = 8.0;
    prepareRolling(sim.vehicle, target);
    Real ayFilt = 0, t = 0, offTrack = 0, rErrInt = 0;
    DriverInput in;
    while (t < 180.0) {
        const Vec3 p = sim.com();
        const auto& tel = sim.vehicle.telemetry();
        const Real speed = tel.speed;
        // Pure-pursuit steering toward a point on the circle.
        const Real ld = std::max(6.0, 0.5 * speed);
        const Real th = std::atan2(p.y - c.y, p.x - c.x) + ld / radius;
        const Vec3 tgt{c.x + radius * std::cos(th), c.y + radius * std::sin(th), 0};
        const Vec3 toT = sim.vehicle.state().orientation.inverseRotate(tgt - p);
        const Real eta = std::atan2(toT.y, toT.x);
        // Pure pursuit alone drifts outward as the car understeers; a PI term
        // on the radius error keeps the measurement on the intended circle.
        const Real rNow = std::hypot(p.x - c.x, p.y - c.y) - radius;
        rErrInt = clamp(rErrInt + rNow * sim.dt, -2.0, 2.0);
        const Real delta = std::atan(2.0 * L * std::sin(eta) / ld) + 0.01 * rNow + 0.01 * rErrInt;
        in.steer = clamp(delta / maxSteer, -1.0, 1.0);
        holdSpeed(in, speed, target);
        sim.step(in);
        t += sim.dt;
        target += 0.15 * sim.dt;  // slow ramp: quasi-steady-state

        const Real rErr = std::hypot(p.x - c.x, p.y - c.y) - radius;
        const Real a = std::min(1.0, sim.dt / 0.5);
        ayFilt += a * (tel.accelBody.y - ayFilt);
        if (std::abs(rErr) < 1.5 && t > 5.0 && ayFilt / kGravity > res.maxLateralG) {
            res.maxLateralG = ayFilt / kGravity;
            res.speedAtMaxKmh = msToKmh(speed);
            res.sideslipAtMax = tel.sideslip;
        }
        offTrack = std::abs(rErr) > 3.0 ? offTrack + sim.dt : 0.0;
        if (offTrack > 1.0) break;
    }
    return res;
}

StepSteerResult stepSteer(const RunConfig& cfgIn, Real speedKmh, Real steerInput) {
    const RunConfig cfg = withDirectSteering(cfgIn);
    Sim sim(cfg);
    const Real v0 = kmhToMs(speedKmh);
    prepareRolling(sim.vehicle, v0);
    DriverInput in;
    for (Real t = 0; t < 2.0; t += sim.dt) {
        holdSpeed(in, sim.vehicle.telemetry().speed, v0);
        sim.step(in);
    }
    in.brake = 0;
    in.steer = steerInput;
    std::vector<Real> yaw;
    for (Real t = 0; t < 4.0; t += sim.dt) {
        sim.step(in);
        yaw.push_back(sim.vehicle.telemetry().yawRate);
    }
    StepSteerResult r;
    const std::size_t lastSec = static_cast<std::size_t>(1.0 / sim.dt);
    Real sum = 0;
    for (std::size_t i = yaw.size() - lastSec; i < yaw.size(); ++i) sum += yaw[i];
    r.steadyYawRate = sum / static_cast<Real>(lastSec);
    r.steadyLateralG = sim.vehicle.telemetry().accelBody.y / kGravity;
    Real t10 = -1, t90 = -1, peak = 0;
    for (std::size_t i = 0; i < yaw.size(); ++i) {
        const Real f = yaw[i] / r.steadyYawRate;
        if (t10 < 0 && f >= 0.1) t10 = static_cast<Real>(i) * sim.dt;
        if (t90 < 0 && f >= 0.9) t90 = static_cast<Real>(i) * sim.dt;
        peak = std::max(peak, f);
    }
    r.yawRateRiseTime = (t10 >= 0 && t90 >= 0) ? t90 - t10 : -1;
    r.overshoot = peak - 1.0;
    return r;
}

SineDwellResult sineWithDwell(const RunConfig& cfgIn, Real amp, Real speedKmh) {
    const RunConfig cfg = withDirectSteering(cfgIn);
    Sim sim(cfg);
    const Real v0 = kmhToMs(speedKmh);
    prepareRolling(sim.vehicle, v0 + 1.0);
    const Real maxSteer = maxSteerRef(cfg.vehicle);
    DriverInput in;
    for (Real t = 0; t < 1.0; t += sim.dt) {
        holdSpeed(in, sim.vehicle.telemetry().speed, v0 + 1.0);
        sim.step(in);
    }
    in.throttle = 0;
    in.brake = 0;
    constexpr Real f = 0.7;
    const Real quarter = 0.25 / f;
    const Real tDwellStart = 3 * quarter;   // second (negative) peak
    const Real tDwellEnd = tDwellStart + 0.5;
    const Real tEnd = tDwellEnd + quarter;  // back to zero
    SineDwellResult r;
    Real t = 0;
    Real firstPeak = 0;
    bool firstPeakDone = false;
    while (t < tEnd + 2.0) {
        Real delta;
        if (t < tDwellStart) delta = amp * std::sin(2 * kPi * f * t);
        else if (t < tDwellEnd) delta = -amp;
        else if (t < tEnd) delta = amp * std::sin(2 * kPi * f * (t - tDwellEnd + tDwellStart));
        else delta = 0;
        in.steer = clamp(delta / maxSteer, -1.0, 1.0);
        sim.step(in);
        t += sim.dt;
        const auto& tel = sim.vehicle.telemetry();
        r.maxSideslip = std::max(r.maxSideslip, std::abs(tel.sideslip));
        if (std::abs(tel.sideslip) > kPi / 4) r.spun = true;
        if (!firstPeakDone) {
            if (tel.yawRate > firstPeak) firstPeak = tel.yawRate;
            else if (t > quarter && tel.yawRate < 0.8 * firstPeak) firstPeakDone = true;
        }
        if (t >= tEnd + 1.0 && r.ratioAt1s == 0) r.ratioAt1s = std::abs(tel.yawRate) / std::max(firstPeak, 1e-6);
        if (t >= tEnd + 1.75 && r.ratioAt175s == 0) r.ratioAt175s = std::abs(tel.yawRate) / std::max(firstPeak, 1e-6);
    }
    r.peakYawRate = firstPeak;
    return r;
}

HoldResult slopeHold(const RunConfig& cfgIn, Real gradePercent, Real seconds, bool brakes) {
    const RunConfig cfg = withAutoShift(cfgIn);
    const Real g = gradePercent / 100.0;
    Sim sim(cfg, [g](Real x, Real) { return g * x; });
    DriverInput in;
    in.brake = brakes ? 1.0 : 0.0;
    if (!brakes) sim.vehicle.assists().setGear(0);
    const Vec3 p0 = sim.com();
    for (Real t = 0; t < seconds; t += sim.dt) sim.step(in);
    return {length(sim.com() - p0)};
}

BumpResult speedBump(const RunConfig& cfgIn, Real speedKmh, Real h, Real len) {
    const RunConfig cfg = withAutoShift(cfgIn);
    constexpr Real x0 = 15.0;
    Sim sim(cfg, [h, len](Real x, Real) {
        if (x < x0 || x > x0 + len) return 0.0;
        const Real s = std::sin(kPi * (x - x0) / len);
        return h * s * s;
    });
    const Real v0 = kmhToMs(speedKmh);
    prepareRolling(sim.vehicle, v0);
    BumpResult r;
    r.minWheelLoad = 1e9;
    DriverInput in;
    for (Real t = 0; t < 10.0 && sim.com().x < x0 + 20.0; t += sim.dt) {
        holdSpeed(in, sim.vehicle.telemetry().speed, v0);
        sim.step(in);
        const auto& tel = sim.vehicle.telemetry();
        if (!isFinite(tel.position) || !std::isfinite(tel.speed)) {
            r.finite = false;
            break;
        }
        if (sim.com().x < x0 - 3.0) continue;
        bool air = false, all = true;
        for (std::size_t w = 0; w < tel.wheelCount; ++w) {
            r.peakCompression = std::max(r.peakCompression, tel.wheels[w].compression);
            air = air || !tel.wheels[w].contact;
            all = all && !tel.wheels[w].contact;
            r.maxWheelLoad = std::max(r.maxWheelLoad, tel.wheels[w].load);
            r.minWheelLoad = std::min(r.minWheelLoad, tel.wheels[w].load);
        }
        if (air) r.airborneTime += sim.dt;
        if (all) r.allAirborneTime += sim.dt;
        r.peakVerticalAccelG = std::max(r.peakVerticalAccelG, std::abs(tel.accelBody.z) / kGravity);
    }
    return r;
}

DropResult dropTest(const RunConfig& cfgIn, Real dropHeight) {
    const RunConfig cfg = withAutoShift(cfgIn);
    Sim sim(cfg);
    const VehicleDefinition& d = cfg.vehicle;
    sim.vehicle.mutableState().position.z += dropHeight;
    sim.vehicle.mutableState().velocity = {};
    DriverInput in;
    in.brake = 1.0;

    auto energy = [&]() {
        const auto& tel = sim.vehicle.telemetry();
        Real e = tel.kineticEnergy + d.mass * kGravity * sim.com().z;
        for (std::size_t w = 0; w < d.wheels.size(); ++w) {
            const auto& s = d.wheels[w].suspension;
            const Real x = tel.wheels[w].compression;
            const Real xs = std::min(x, s.travel);
            e += s.springPreload * xs + 0.5 * s.springRate * xs * xs;
            if (x > s.travel) e += 0.5 * s.bumpStopRate * (x - s.travel) * (x - s.travel);
        }
        for (const auto& ax : d.axles) {
            const Real dx = tel.wheels[ax.leftWheel].compression - tel.wheels[ax.rightWheel].compression;
            e += 0.5 * ax.antiRollBarRate * dx * dx;
        }
        return e;
    };

    DropResult r;
    sim.step(in);
    const Real e0 = energy();
    Real prevZ = sim.com().z, prevDz = 0;
    bool landed = false;
    for (Real t = 0; t < 4.0; t += sim.dt) {
        sim.step(in);
        const auto& tel = sim.vehicle.telemetry();
        if (!isFinite(tel.position)) {
            r.finite = false;
            break;
        }
        bool anyContact = false;
        for (std::size_t w = 0; w < tel.wheelCount; ++w) anyContact = anyContact || tel.wheels[w].contact;
        landed = landed || anyContact;
        if (energy() > e0 + 0.005 * std::abs(e0) + 50.0) r.energyIncreased = true;
        const Real z = sim.com().z;
        const Real dz = z - prevZ;
        if (landed && prevDz > 0 && dz <= 0) r.bouncePeaks.push_back(prevZ);
        prevDz = dz;
        prevZ = z;
    }
    return r;
}

CoastResult coastNoLosses(const RunConfig& cfgIn, Real startKmh) {
    RunConfig cfg = withAutoShift(cfgIn);
    cfg.vehicle.aero = {0, 0, 0, cfg.vehicle.aero.airDensity};
    for (auto& w : cfg.vehicle.wheels) w.tire.rollingResistance = 0;
    Sim sim(cfg);
    const Real v0 = kmhToMs(startKmh);
    sim.vehicle.assists().setGear(0);
    sim.vehicle.setRollingSpeed(v0);
    CoastResult r;
    DriverInput in;
    Real vmax = v0;
    const Real T = 10.0;
    for (Real t = 0; t < T; t += sim.dt) {
        sim.step(in);
        const Real v = std::hypot(sim.vehicle.telemetry().speed, sim.vehicle.telemetry().lateralVelocity);
        if (v > v0 + 0.01) r.speedEverIncreased = true;  // against the start speed, not step-to-step
        vmax = std::max(vmax, v);
    }
    r.speedLossPerSecond = msToKmh(v0 - sim.vehicle.telemetry().speed) / T;
    return r;
}

PerfResult measureStepCost(const RunConfig& cfgIn, int steps) {
    const RunConfig cfg = withAutoShift(cfgIn);
    RunConfig quiet = cfg;
    quiet.csv = nullptr;
    Sim sim(quiet);
    prepareRolling(sim.vehicle, 20.0);
    DriverInput in;
    in.throttle = 0.3;
    in.steer = 0.05;
    const auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < steps; ++i) sim.step(in);
    const auto t1 = std::chrono::steady_clock::now();
    PerfResult r;
    r.microsecondsPerStep = std::chrono::duration<Real, std::micro>(t1 - t0).count() / steps;
    return r;
}

}  // namespace ord::scenarios

namespace ord::scenarios {

namespace {

constexpr Real kFeatureX = 15.0;

Real ramp01(Real x, Real a, Real b, Real edge) {
    // 0 outside [a, b], 1 inside, linear edges of width `edge`.
    if (x <= a || x >= b) return 0.0;
    return std::min({1.0, (x - a) / edge, (b - x) / edge});
}

RoughRoadResult roughRun(const RunConfig& cfgIn, Real speedKmh, TestTrackGround::HeightFn h, Real xEnd,
                         const std::vector<TestTrackGround::Patch>& patches, bool leftOnly) {
    const RunConfig cfg = withAutoShift(cfgIn);
    Sim sim(cfg, std::move(h), 0, {0, 0, 0.6}, patches);
    const Real v0 = kmhToMs(speedKmh);
    prepareRolling(sim.vehicle, v0);
    const VehicleDefinition& d = cfg.vehicle;
    RoughRoadResult r;
    r.minLoad = 1e9;
    Real staticSum = 0, accSum = 0, loadSum = 0, loadSq = 0;
    int staticN = 0, accN = 0, loadN = 0;
    DriverInput in;
    Real z0 = sim.com().z, frameT = 0, vzFrame = sim.vehicle.state().velocity.z, frameAcc2 = 0;
    int frames = 0;
    constexpr Real kFrame = 1.0 / 60.0;
    for (Real t = 0; t < 20.0 && sim.com().x < xEnd + 6.0; t += sim.dt) {
        holdSpeed(in, sim.vehicle.telemetry().speed, v0);
        sim.step(in);
        const auto& tel = sim.vehicle.telemetry();
        if (!isFinite(tel.position) || !std::isfinite(tel.speed)) {
            r.finite = false;
            break;
        }
        const Real x = sim.com().x;
        if (x < kFeatureX - 4.0) z0 = sim.com().z;
        frameT += sim.dt;
        if (frameT >= kFrame - 1e-9) {
            const Real vz = sim.vehicle.state().velocity.z;
            if (x > kFeatureX - 3.5 && x < xEnd + 3.5) {
                const Real a = (vz - vzFrame) / frameT;
                frameAcc2 += a * a;
                ++frames;
            }
            vzFrame = vz;
            frameT = 0;
        }
        if (x > kFeatureX - 3.5 && x < xEnd + 3.5) r.maxHeave = std::max(r.maxHeave, sim.com().z - z0);
        const bool before = x > kFeatureX - 9.0 && x < kFeatureX - 4.0;
        const bool during = x > kFeatureX - 3.5 && x < xEnd + 3.5;
        bool airborne = false;
        for (std::size_t w = 0; w < d.wheels.size(); ++w) {
            if (leftOnly && !d.wheels[w].isLeft) continue;
            const Real load = tel.wheels[w].load;
            if (before) { staticSum += load; ++staticN; }
            if (during) {
                r.peakLoad = std::max(r.peakLoad, load);
                r.minLoad = std::min(r.minLoad, load);
                r.peakFx = std::max(r.peakFx, std::abs(tel.wheels[w].fx));
                loadSum += load; loadSq += load * load; ++loadN;
                airborne = airborne || !tel.wheels[w].contact;
            }
        }
        if (during) {
            accSum += tel.accelBody.z * tel.accelBody.z;
            ++accN;
            if (airborne) r.wheelAirborneTime += sim.dt;
        }
    }
    r.rmsFrameAccelG = frames ? std::sqrt(frameAcc2 / frames) / kGravity : 0;
    r.staticLoad = staticN ? staticSum / staticN : 0;
    r.rmsVerticalAccelG = accN ? std::sqrt(accSum / accN) / kGravity : 0;
    if (loadN) {
        const Real mean = loadSum / loadN;
        r.loadVariation = mean > 0 ? std::sqrt(std::max(0.0, loadSq / loadN - mean * mean)) / mean : 0;
    }
    return r;
}

// Hann-windowed DFT magnitude peak search with parabolic refinement.
Real dominantFrequency(const std::vector<Real>& sig, Real fs, Real fLo, Real fHi, Real df) {
    const std::size_t n = sig.size();
    Real mean = 0;
    for (Real v : sig) mean += v;
    mean /= static_cast<Real>(n);
    std::vector<Real> x(n);
    for (std::size_t i = 0; i < n; ++i)
        x[i] = (sig[i] - mean) * (0.5 - 0.5 * std::cos(2 * kPi * static_cast<Real>(i) / static_cast<Real>(n - 1)));
    Real bestF = fLo, bestM = -1, mPrev = 0, mNext = 0;
    std::vector<Real> mags;
    for (Real f = fLo; f <= fHi; f += df) {
        Real re = 0, im = 0;
        const Real w = 2 * kPi * f / fs;
        for (std::size_t i = 0; i < n; ++i) {
            re += x[i] * std::cos(w * static_cast<Real>(i));
            im -= x[i] * std::sin(w * static_cast<Real>(i));
        }
        mags.push_back(re * re + im * im);
    }
    std::size_t k = 0;
    for (std::size_t i = 0; i < mags.size(); ++i)
        if (mags[i] > bestM) { bestM = mags[i]; k = i; }
    bestF = fLo + static_cast<Real>(k) * df;
    if (k > 0 && k + 1 < mags.size()) {
        mPrev = mags[k - 1];
        mNext = mags[k + 1];
        const Real denom = mPrev - 2 * bestM + mNext;
        if (std::abs(denom) > 0) bestF += 0.5 * (mPrev - mNext) / denom * df;
    }
    return bestF;
}

}  // namespace

RoughRoadResult curbStrike(const RunConfig& cfg, Real speedKmh, Real height) {
    const Real x0 = kFeatureX, x1 = kFeatureX + 0.5;
    auto h = [=](Real x, Real y) { return (y > 0.5 && y < 1.1) ? height * ramp01(x, x0, x1, 0.02) : 0.0; };
    std::vector<TestTrackGround::Patch> patches{{x0 - 0.05, x1 + 0.05, 0.5, 1.1, {1.0, 1.0, 0.0, 0.6}, 2}};
    return roughRun(cfg, speedKmh, h, x1, patches, true);
}

RoughRoadResult pothole(const RunConfig& cfg, Real speedKmh, Real depth, Real len) {
    const Real x0 = kFeatureX, x1 = kFeatureX + len;
    auto h = [=](Real x, Real y) { return (y > 0.45 && y < 1.15) ? -depth * ramp01(x, x0, x1, 0.02) : 0.0; };
    // Deliberately *no* roughness tag: potholes in plain asphalt must be found by the cache's residual check.
    return roughRun(cfg, speedKmh, h, x1, {}, true);
}

RoughRoadResult cobblestones(const RunConfig& cfg, Real speedKmh, bool textured) {
    const Real x0 = kFeatureX, x1 = kFeatureX + 25.0;
    constexpr Real A = 0.02, lambda = 0.15;
    if (textured) {
        // Flat collision; the stones come from SurfaceProperties' micro-profile
        // (same shape, no per-stone variation, so results are comparable).
        SurfaceProperties s{0.85, 1.6, 0.0, 0.0, A, lambda, 0.0};
        return roughRun(cfg, speedKmh, nullptr, x1, {{x0, x1, -5, 5, s, 3}}, false);
    }
    auto h = [=](Real x, Real y) {
        if (x < x0 || x > x1) return 0.0;
        return A * 0.25 * (1 - std::cos(2 * kPi * x / lambda)) * (1 - std::cos(2 * kPi * y / lambda));
    };
    std::vector<TestTrackGround::Patch> patches{{x0, x1, -5, 5, {0.85, 1.6, 0.0, 0.8}, 3}};
    return roughRun(cfg, speedKmh, h, x1, patches, false);
}

FrequencyResult suspensionFrequencies(const RunConfig& cfgIn) {
    RunConfig cfg = withAutoShift(cfgIn);
    cfg.hz = 5000;
    VehicleDefinition& d = cfg.vehicle;
    d.centerOfMass = {0, 0, 0};
    d.aero = {0, 0, 0, d.aero.airDensity};
    const WheelDef proto = d.wheels[0];
    for (std::size_t w = 0; w < d.wheels.size(); ++w) {
        WheelDef nw = proto;
        nw.isLeft = d.wheels[w].isLeft;
        nw.suspension.hardpoint = {w < 2 ? 1.3 : -1.3, nw.isLeft ? 0.8 : -0.8, proto.suspension.hardpoint.z};
        nw.suspension.bumpDamping = nw.suspension.reboundDamping = 0;
        nw.tire.verticalDamping = 0;
        nw.maxSteerAngle = w < 2 ? proto.maxSteerAngle : 0;
        d.wheels[w] = nw;
    }
    for (auto& ax : d.axles) ax.antiRollBarRate = 0;
    Sim sim(cfg);
    sim.vehicle.mutableState().velocity.z += 0.05;  // heave impulse
    DriverInput in;
    in.brake = 1.0;
    std::vector<Real> z, comp;
    const int total = static_cast<int>(16.0 * cfg.hz);
    for (int i = 0; i < total; ++i) {
        sim.step(in);
        if (i % 20 == 0) z.push_back(sim.com().z);                                  // 250 Hz
        if (i % 5 == 0) comp.push_back(sim.vehicle.telemetry().wheels[0].compression);  // 1000 Hz
    }
    FrequencyResult r;
    r.heaveHz = dominantFrequency(z, cfg.hz / 20, 0.3, 5.0, 0.002);
    r.wheelHopHz = dominantFrequency(comp, cfg.hz / 5, 5.0, 30.0, 0.005);
    // Analytic 2-DOF quarter car: m_s m_u w^4 - (k_s m_u + (k_s + k_t) m_s) w^2 + k_s k_t = 0
    const Real mu = proto.unsprungMass;
    const Real ms = (d.mass - 4 * mu) / 4;
    const Real ks = proto.suspension.springRate, kt = proto.tire.verticalStiffness;
    const Real a = ms * mu, b = -(ks * mu + (ks + kt) * ms), c = ks * kt;
    const Real disc = std::sqrt(b * b - 4 * a * c);
    r.heaveExpectedHz = std::sqrt((-b - disc) / (2 * a)) / (2 * kPi);
    r.wheelHopExpectedHz = std::sqrt((-b + disc) / (2 * a)) / (2 * kPi);
    return r;
}

}  // namespace ord::scenarios

#include <chrono>

#include "reference_vehicles.hpp"

namespace ord::scenarios {

ScaleResult trafficScale(int agentCount, Real seconds, std::size_t maxLod1) {
    using Clock = std::chrono::steady_clock;
    ScaleResult res;
    res.trafficAgents = agentCount;
    // World: gentle hills; ring road of 4 lanes (3 for traffic, outer for the player).
    TestTrackGround world([](Real x, Real y) { return 1.5 * std::sin(x / 60.0) * std::cos(y / 80.0); });
    std::vector<LanePath> lanes;
    for (int i = 0; i < 4; ++i) lanes.push_back(LanePath::circle({0, 0, 0}, 400.0 + 3.5 * i, 33.0, 512));
    TrafficSystem traffic(lanes, TrafficSystem::Settings{});
    std::uint64_t seed = 12345;
    auto rnd = [&]() {
        seed = seed * 6364136223846793005ULL + 1442695040888963407ULL;
        return static_cast<Real>(seed >> 11) * (1.0 / 9007199254740992.0);
    };
    const int perLane = (agentCount + 2) / 3;
    for (int i = 0; i < agentCount; ++i) {
        const int lane = i % 3;
        const Real L = lanes[static_cast<std::size_t>(lane)].length();
        const Real s = (static_cast<Real>(i / 3) + 0.3 * rnd()) * L / perLane;
        const Real vd = 14.0 + 10.0 * rnd();
        traffic.addAgent(lane, s, vd, vd, i % 3);
    }
    LodSettings ls;
    ls.maxLod1 = maxLod1;
    LodManager lod(traffic, {vehicles::referenceHatchFwd(), vehicles::referenceSportsRwd(), vehicles::referenceSaloonAwd()}, ls);

    // Player: LOD0, Chaos-like host at 60 Hz with 4 substeps and a ground cache.
    Vehicle player(vehicles::referenceSportsRwd(), makePreset(DrivingPreset::Sport));
    const LanePath& pl = lanes[3];
    const Vec3 p0 = pl.pointAt(0);
    player.reset({p0.x, p0.y, p0.z + 2.0}, 0.0, world);
    player.selectGearForSpeed(30.0);
    player.placeOnGround(p0.x, p0.y, 0.0, world, 30.0);
    player.setChassisMode(ChassisMode::External, false);
    LaneFollowDriver pilot;
    pilot.reset(0);
    GroundPrefetchCache playerCache;
    std::vector<GroundStripRequest> req;
    CountingGroundQuery playerWorld(world), lodWorld(world);

    const Real frame = 1.0 / 60.0;
    const int frames = static_cast<int>(seconds / frame);
    Real lod0Seconds = 0;
    for (int f = 0; f < frames; ++f) {
        const auto t0 = Clock::now();
        req.clear();
        player.groundPrefetchRequests(req);
        playerCache.rebuild(playerWorld, req.data(), req.size());
        const DriverInput in = pilot.drive(player, pl, 32.0, frame);
        player.stepSubstepped(frame, 4, in, playerCache);
        // Host integration (same scheme as the core here; parity is tested elsewhere).
        RigidBodyState s = player.state();
        const ChassisForces& cf = player.chassisForces();
        const VehicleDefinition& d = player.definition();
        const Vec3 com = s.position + s.orientation.rotate(d.centerOfMass);
        s.velocity += cf.force * (frame / d.mass);
        const Mat3 R = s.orientation.toMatrix();
        const Mat3 inv = R * Mat3::diagonal({1 / d.inertia.x, 1 / d.inertia.y, 1 / d.inertia.z}) * R.transposed();
        const Mat3 I = R * Mat3::diagonal(d.inertia) * R.transposed();
        s.angularVelocity += inv * (cf.torque - cross(s.angularVelocity, I * s.angularVelocity)) * frame;
        const Real ang = length(s.angularVelocity) * frame;
        if (ang > 1e-12) s.orientation = Quat::fromAxisAngle(s.angularVelocity, ang) * s.orientation;
        s.orientation.normalize();
        s.position = com + s.velocity * frame - s.orientation.rotate(d.centerOfMass);
        player.setChassisState(s);
        const auto t1 = Clock::now();
        lod0Seconds += std::chrono::duration<Real>(t1 - t0).count();

        lod.update(frame, player.centerOfMassWorld(), lodWorld);
        res.maxLod1 = std::max(res.maxLod1, lod.stats().lod1Active);
        for (const auto& c : lod.lod1())
            if (c.used) res.maxLod1LateralError = std::max(res.maxLod1LateralError, std::abs(c.driver.lateralError()));
        if (!isFinite(player.state().position)) { res.finite = false; break; }
    }
    for (const auto& c : lod.lod1())
        if (c.vehicle && !isFinite(c.vehicle->state().position)) res.finite = false;
    const Real nf = frames;
    res.simulatedSeconds = frames * frame;
    res.promotions = lod.stats().promotions;
    res.demotions = lod.stats().demotions;
    res.lod0MsPerFrame = 1000.0 * lod0Seconds / nf;
    res.lod1MsPerFrame = 1000.0 * lod.stats().lod1Seconds / nf;
    res.lod2MsPerFrame = 1000.0 * lod.stats().lod2Seconds / nf;
    res.totalMsPerFrame = res.lod0MsPerFrame + res.lod1MsPerFrame + res.lod2MsPerFrame;
    res.lod0RaysPerFrame = static_cast<Real>(playerWorld.count()) / nf;
    res.lod2RaysPerFrame = static_cast<Real>(traffic.groundQueries()) / nf;
    res.lod1RaysPerFrame = (static_cast<Real>(lodWorld.count()) - static_cast<Real>(traffic.groundQueries())) / nf;
    return res;
}

TransitionResult lodTransition(Real speedKmh) {
    TransitionResult r;
    FlatGround world;
    const Real v = kmhToMs(speedKmh);
    std::vector<LanePath> lanes{LanePath::circle({0, 0, 0}, 250.0, 40.0)};
    TrafficSystem traffic(lanes, TrafficSystem::Settings{});
    traffic.addAgent(0, 100.0, v, v, 1);
    LodSettings ls;
    ls.maxLod1 = 1;
    ls.promoteRadius = 1e6;  // promotion is triggered manually below
    ls.demoteRadius = 2e6;
    LodManager lod(traffic, {vehicles::referenceHatchFwd(), vehicles::referenceSportsRwd()}, ls);
    // Cruise as LOD2 for 2 s so the agent is in steady state.
    for (int i = 0; i < 120; ++i) traffic.step(1.0 / 60.0, &world);
    const auto before = traffic.agents()[0];
    lod.promote(0, world);
    const Vehicle& car = *lod.lod1()[0].vehicle;
    r.speedJumpPromote = std::abs(car.forwardSpeed() - before.speed);
    r.yawRateJumpPromote = std::abs(car.yawRate() - before.yawRate);
    Real t = 0;
    Vec3 vPrev = car.state().velocity;
    for (int f = 0; f < 120; ++f) {
        lod.update(1.0 / 60.0, car.centerOfMassWorld(), world);
        t += 1.0 / 60.0;
        const Vec3 vNow = car.state().velocity;
        if (t <= 1.0) r.maxVerticalAccelG = std::max(r.maxVerticalAccelG, std::abs((vNow.z - vPrev.z) * 60.0) / kGravity);
        vPrev = vNow;
        r.maxLateralError = std::max(r.maxLateralError, std::abs(lod.lod1()[0].driver.lateralError()));
    }
    const Real vBefore = car.forwardSpeed();
    const Vec3 comBefore = car.centerOfMassWorld();
    lod.demote(0);
    const auto& a = traffic.agents()[0];
    r.speedJumpDemote = std::abs(a.speed - vBefore);
    const Vec3 lanePos = lanes[0].pointAt(a.s);
    r.positionJumpDemote = std::hypot(lanePos.x - comBefore.x, lanePos.y - comBefore.y);
    return r;
}

}  // namespace ord::scenarios

namespace ord::scenarios {

DriftResult driftHold(const RunConfig& cfg, Real steerToward, Real throttle, Real entryKmh, Real holdSeconds) {
    Sim sim(cfg);
    prepareRolling(sim.vehicle, kmhToMs(entryKmh));
    DriftResult r;
    Real t = 0, tEntered = 0;
    Real sum = 0, sum2 = 0, sumT = 0;
    int n = 0, lastGear = sim.vehicle.telemetry().gear;
    while (t < 4.0 + holdSeconds) {
        const auto& tel = sim.vehicle.telemetry();
        DriverInput in;
        if (!r.entered && (tel.assists.driftActive || std::abs(tel.sideslip) > 0.26)) {
            r.entered = true;
            tEntered = t;
        }
        if (!r.entered) {
            if (t > 4.0) break;  // never broke loose
            in.steer = 0.8;
            in.throttle = 0.9;
            in.handbrake = (t > 0.1 && t < 0.4) ? 1.0 : 0.0;
        } else {
            if (t > tEntered + holdSeconds) break;
            in.steer = steerToward;  // left-hand drift: + steers into the turn
            in.throttle = throttle;
            r.maxThrottleAdded = std::max(r.maxThrottleAdded, tel.throttleOut - tel.throttleInput);
            const bool sliding = std::abs(tel.sideslip) > 0.15;
            if (sliding && tel.gear > lastGear && throttle > 0.3) ++r.upshiftsWhileSliding;
            if (t > tEntered + 2.0) {
                sum += tel.sideslip;
                sum2 += tel.sideslip * tel.sideslip;
                sumT += tel.assists.driftTargetSideslip;
                ++n;
            }
            if (std::abs(tel.sideslip) > 1.4 || tel.speed < 1.5) {
                r.spun = true;
                break;
            }
        }
        lastGear = tel.gear;
        sim.step(in);
        t += sim.dt;
    }
    if (n) {
        r.meanSideslip = sum / n;
        r.sdSideslip = std::sqrt(std::max(0.0, sum2 / n - r.meanSideslip * r.meanSideslip));
        r.meanTarget = sumT / n;
    }
    r.endSpeedKmh = msToKmh(sim.vehicle.telemetry().speed);
    return r;
}

std::vector<BaselineMetric> computeBaselineMetrics() {
    std::vector<BaselineMetric> m;
    RunConfig sim;
    sim.vehicle = vehicles::referenceSportsRwd();
    sim.assists = makePreset(DrivingPreset::Simulation);
    sim.hz = 240;
    RunConfig sport = sim;
    sport.assists = makePreset(DrivingPreset::Sport);
    m.push_back({"sports_sport_0_100_s", accelerate0to100(sport).time0to100, 0.01});
    RunConfig absCfg = sim;
    absCfg.assists.abs = true;
    m.push_back({"sports_abs_100_0_m", brake100to0(absCfg).distance, 0.01});
    m.push_back({"sports_locked_100_0_m", brake100to0(sim).distance, 0.01});
    m.push_back({"sports_skidpad_r40_g", skidpad(sim).maxLateralG, 0.01});
    m.push_back({"sports_stepsteer_80_yaw_rads", stepSteer(sim).steadyYawRate, 0.01});
    const SineDwellResult sd = sineWithDwell(sim, 9.0 * kPi / 180);
    m.push_back({"sports_sinedwell9_max_sideslip_rad", sd.maxSideslip, 0.03});
    RunConfig cached = sim;
    cached.prefetch = true;
    m.push_back({"sports_curb120_cached_max_heave_m", curbStrike(cached, 120).maxHeave, 0.03});
    m.push_back({"sports_cobble_texture40_rms_g", cobblestones(cached, 40, true).rmsVerticalAccelG, 0.02});
    m.push_back({"sports_pothole60_rms_frame_g", pothole(sim, 60).rmsFrameAccelG, 0.02});
    const FrequencyResult fr = suspensionFrequencies(sim);
    m.push_back({"sports_heave_hz", fr.heaveHz, 0.005});
    m.push_back({"sports_wheelhop_hz", fr.wheelHopHz, 0.005});
    RunConfig hatch = sim;
    hatch.vehicle = vehicles::referenceHatchFwd();
    m.push_back({"hatch_sim_0_100_s", accelerate0to100(hatch).time0to100, 0.01});
    m.push_back({"hatch_skidpad_r40_g", skidpad(hatch).maxLateralG, 0.01});
    RunConfig drift = sim;
    drift.assists = makePreset(DrivingPreset::Drift);
    drift.hz = 360;
    m.push_back({"sports_drift_neutral_sideslip_deg", driftHold(drift, 0.0, 0.8).meanSideslip * 180 / kPi, 0.05});
    m.push_back({"sports_drift_full_sideslip_deg", driftHold(drift, 1.0, 0.8).meanSideslip * 180 / kPi, 0.05});
    return m;
}

}  // namespace ord::scenarios
