// OpenRoad Dynamics — suspension linkage kinematics tests.
// SPDX-License-Identifier: MIT
#include <cmath>
#include <random>

#include "ord/ground.hpp"
#include "ord/kinematics.hpp"
#include "ord/presets.hpp"
#include "ord/setup.hpp"
#include "ord/vehicle.hpp"
#include "ord_test.hpp"
#include "reference_vehicles.hpp"
#include "production_vehicles.hpp"
#include "scenarios.hpp"

using namespace ord;
using namespace ord::scenarios;

namespace {

// A planar (front-view) geometry: pivot axes along x, ball joints at x = 0, and
// a tie rod that is the lower arm shifted along x, so the steer about the
// kingpin stays exactly zero. The 3-D mechanism then reduces to a 2-D four-bar
// that can be solved independently by circle intersection.
LinkageDef planarWishbone(Real yl, Real zl, Real yu, Real zu, Real zLbj, Real zUbj, Real yUbj) {
    LinkageDef d;
    d.type = LinkageType::DoubleWishbone;
    d.lowerPivotFront = {0.2, yl, zl};
    d.lowerPivotRear = {-0.2, yl, zl};
    d.lowerBallJoint = {0, -0.02, zLbj};
    d.upperPivotFront = {0.15, yu, zu};
    d.upperPivotRear = {-0.15, yu, zu};
    d.upperBallJoint = {0, yUbj, zUbj};
    d.tieRodInner = {-0.1, yl, zl};
    d.tieRodOuter = d.lowerBallJoint + Vec3{-0.1, 0, 0};
    d.springArm = {0, -0.22, zLbj + 0.01};
    d.springBody = {0, -0.25, zLbj + 0.45};
    return d;
}

struct P2 { Real y, z; };
P2 rot2(P2 v, Real a) { return {v.y * std::cos(a) - v.z * std::sin(a), v.y * std::sin(a) + v.z * std::cos(a)}; }

// Independent 2-D four-bar: wheel centre height, camber and lateral position for
// a lower-arm angle, by circle intersection; then bisection on the angle.
struct Planar2D {
    P2 pl, pu, lbj0, ubj0;
    Real rl, ru, d;
    bool strut = false;  // MacPherson: pu is the strut top, the upright axis passes through it
    Planar2D(const LinkageDef& l) {
        pl = {l.lowerPivotFront.y, l.lowerPivotFront.z};
        lbj0 = {l.lowerBallJoint.y, l.lowerBallJoint.z};
        strut = l.type == LinkageType::MacPherson;
        pu = strut ? P2{l.strutTop.y, l.strutTop.z} : P2{l.upperPivotFront.y, l.upperPivotFront.z};
        ubj0 = strut ? pu : P2{l.upperBallJoint.y, l.upperBallJoint.z};
        rl = std::hypot(lbj0.y - pl.y, lbj0.z - pl.z);
        ru = std::hypot(ubj0.y - pu.y, ubj0.z - pu.z);
        d = std::hypot(ubj0.y - lbj0.y, ubj0.z - lbj0.z);
    }
    // Returns wheel centre (y, z) and camber for a lower-arm angle.
    bool pose(Real theta, P2& wc, Real& camber, Real& lat) const {
        const Real a0 = std::atan2(lbj0.z - pl.z, lbj0.y - pl.y);
        const P2 lbj{pl.y + rl * std::cos(a0 + theta), pl.z + rl * std::sin(a0 + theta)};
        P2 ubj = pu;
        if (!strut) {
            // Circle (pu, ru) meets circle (lbj, d): pick the root nearest the design UBJ.
            const Real dx = lbj.y - pu.y, dz = lbj.z - pu.z, D = std::hypot(dx, dz);
            const Real a = (ru * ru - d * d + D * D) / (2 * D);
            const Real h2 = ru * ru - a * a;
            if (h2 < 0) return false;
            const Real h = std::sqrt(h2);
            const P2 m{pu.y + a * dx / D, pu.z + a * dz / D};
            const P2 c1{m.y + h * dz / D, m.z - h * dx / D}, c2{m.y - h * dz / D, m.z + h * dx / D};
            ubj = std::hypot(c1.y - ubj0.y, c1.z - ubj0.z) < std::hypot(c2.y - ubj0.y, c2.z - ubj0.z) ? c1 : c2;
        }
        const Real psi0 = std::atan2(ubj0.z - lbj0.z, ubj0.y - lbj0.y), psi = std::atan2(ubj.z - lbj.z, ubj.y - lbj.y);
        const Real rot = psi - psi0;
        const P2 r = rot2({0 - lbj0.y, 0 - lbj0.z}, rot);
        wc = {lbj.y + r.y, lbj.z + r.z};
        camber = -rot;  // axle (1,0) rotated by rot has z = sin(rot); camber = -asin(z)
        lat = wc.y;
        return true;
    }
    bool atHeight(Real t, Real& camber, Real& lat) const {
        Real lo = -0.8, hi = 0.8;
        for (int i = 0; i < 80; ++i) {
            const Real mid = 0.5 * (lo + hi);
            P2 wc; Real c, l;
            if (!pose(mid, wc, c, l)) return false;
            (wc.z < t ? lo : hi) = mid;
        }
        P2 wc;
        return pose(0.5 * (lo + hi), wc, camber, lat);
    }
    // Front-view roll centre by the graphical construction: arm lines meet at the
    // instant centre; the line from the contact patch through it crosses the
    // centreline (y = -halfTrack) at the roll centre. Height above the ground.
    Real rollCentre(Real halfTrack, Real r) const {
        // Lines: lower through (pl, lbj0), upper through (pu, ubj0). For a strut the
        // "upper line" is perpendicular to the strut axis at its top.
        P2 a1 = pl, d1{lbj0.y - pl.y, lbj0.z - pl.z};
        P2 a2 = pu, d2 = strut ? P2{-(ubj0.z - lbj0.z), ubj0.y - lbj0.y} : P2{ubj0.y - pu.y, ubj0.z - pu.z};
        const Real den = d1.y * d2.z - d1.z * d2.y;
        const Real s = ((a2.y - a1.y) * d2.z - (a2.z - a1.z) * d2.y) / den;
        const P2 ic{a1.y + s * d1.y, a1.z + s * d1.z};
        const P2 patch{0, -r};
        return (-halfTrack - patch.y) * (ic.z - patch.z) / (ic.y - patch.y);
    }
};

VehicleDefinition withLinkage(VehicleDefinition d, const LinkageDef& front, const LinkageDef& rear) {
    for (WheelDef& w : d.wheels) {
        w.suspension.linkage = w.suspension.hardpoint.x > 0 ? front : rear;
        w.camberGain = 0;
    }
    return d;
}

struct Steady { Real roll, latG, transferFront, transferRear; };
// Holds a steer input at ~constant speed and averages the roll, lateral g and
// the lateral load transfer over the last two seconds.
Steady steadyTurn(const VehicleDefinition& d, Real steer, Real speed) {
    Vehicle v(d, makePreset(DrivingPreset::Simulation));
    FlatGround g;
    v.reset({0, 0, 0.6}, 0, g);
    v.setRollingSpeed(speed);
    DriverInput in;
    Steady s{};
    int n = 0;
    const Real dt = 1.0 / 360;
    for (int i = 0; i < 360 * 10; ++i) {
        const TelemetryFrame& t = v.telemetry();
        in.throttle = clamp(0.3 + 0.5 * (speed - t.speed), 0.0, 1.0);
        in.steer = i > 360 ? steer : 0.0;
        v.step(dt, in, g);
        if (i >= 360 * 8) {
            s.roll += t.rollAngle;
            s.latG += t.accelBody.y / kGravity;
            s.transferFront += t.wheels[1].load - t.wheels[0].load;
            s.transferRear += t.wheels[3].load - t.wheels[2].load;
            ++n;
        }
    }
    s.roll /= n; s.latG /= n; s.transferFront /= n; s.transferRear /= n;
    return s;
}

struct Dive { Real maxPitch, distance; };
Dive brakeDive(const VehicleDefinition& d, Real pedal) {
    Vehicle v(d, makePreset(DrivingPreset::Simulation));
    FlatGround g;
    v.reset({0, 0, 0.6}, 0, g);
    v.setRollingSpeed(80 / 3.6);
    DriverInput in;
    Dive r{0, 0};
    const Real dt = 1.0 / 360;
    for (int i = 0; i < 360 * 2; ++i) { in.throttle = 0.25; v.step(dt, in, g); }  // settle at speed
    in.throttle = 0;
    const Real x0 = v.state().position.x;
    for (int i = 0; i < 360 * 8; ++i) {
        in.brake = pedal;
        v.step(dt, in, g);
        r.maxPitch = std::max(r.maxPitch, v.telemetry().pitchAngle);
        if (v.telemetry().speed < 0.3) break;
    }
    r.distance = v.state().position.x - x0;
    return r;
}

}  // namespace

TEST_CASE("kinematics: the 3-D wishbone solve matches an independent 2-D four-bar") {
    const LinkageDef l = planarWishbone(-0.42, -0.12, -0.32, 0.13, -0.14, 0.16, -0.07);
    LinkageTable t;
    std::string err;
    CHECK(t.build(l, -0.09, 0.09, &err));
    const Planar2D ref(l);
    Real worst = 0;
    for (Real tr = -0.08; tr <= 0.0801; tr += 0.01) {
        Real c2 = 0, lat2 = 0;
        CHECK(ref.atHeight(tr, c2, lat2));
        const LinkagePose p = t.at(tr);
        worst = std::max({worst, std::abs(p.camber - c2), std::abs(p.lat - lat2)});
        CHECK_NEAR(p.camber, c2, 2e-5);
        CHECK_NEAR(p.lat, lat2, 2e-5);
        CHECK(std::abs(p.toe) < 1e-6);  // the shifted-arm tie rod gives no bump steer
    }
    std::cout << "    wishbone 3-D vs 2-D worst difference " << worst << "\n";
    // Short upper arm: negative camber in bump, positive in droop.
    CHECK(t.at(0.06).camber < -0.01);
    CHECK(t.at(-0.06).camber > 0.005);
}

TEST_CASE("kinematics: the MacPherson solve matches an independent 2-D construction") {
    LinkageDef l = planarWishbone(-0.40, -0.13, 0, 0, -0.14, 0, 0);
    l.type = LinkageType::MacPherson;
    l.strutTop = {0, -0.09, 0.46};
    l.springArm = l.lowerBallJoint + (l.strutTop - l.lowerBallJoint) * 0.3;
    l.springBody = l.strutTop;
    LinkageTable t;
    std::string err;
    CHECK(t.build(l, -0.09, 0.09, &err));
    const Planar2D ref(l);
    for (Real tr = -0.08; tr <= 0.0801; tr += 0.02) {
        Real c2 = 0, lat2 = 0;
        CHECK(ref.atHeight(tr, c2, lat2));
        const LinkagePose p = t.at(tr);
        CHECK_NEAR(p.camber, c2, 2e-5);
        CHECK_NEAR(p.lat, lat2, 2e-5);
    }
    // The strut telescopes: the spring shortens in bump, nearly 1:1.
    CHECK_RANGE(-t.designMotionRatio(), 0.9, 1.0);
}

TEST_CASE("kinematics: parallel equal arms keep camber; a raised rack gives bump steer") {
    LinkageDef eq = planarWishbone(-0.42, -0.14, -0.42, 0.16, -0.14, 0.16, -0.02);  // same length, same angle
    LinkageTable t;
    CHECK(t.build(eq, -0.08, 0.08));
    for (Real tr = -0.08; tr <= 0.0801; tr += 0.02) CHECK(std::abs(t.at(tr).camber) < 1e-7);
    LinkageDef bs = eq;
    bs.tieRodInner.z += 0.03;
    LinkageTable tb;
    CHECK(tb.build(bs, -0.08, 0.08));
    CHECK(std::abs(tb.at(0.05).toe) > 0.005);                       // a few tenths of a degree
    CHECK(tb.at(0.05).toe * tb.at(-0.05).toe < 0);                   // opposite sense in bump and droop
    CHECK(std::abs(tb.at(0.0).toe) < 1e-6);                          // none at the design position
}

TEST_CASE("kinematics: the roll centre from the path matches the graphical construction") {
    for (const LinkageDef& l : {planarWishbone(-0.42, -0.12, -0.32, 0.13, -0.14, 0.16, -0.07),
                                planarWishbone(-0.42, -0.16, -0.30, 0.10, -0.14, 0.16, -0.06),
                                planarWishbone(-0.40, -0.10, -0.40, 0.19, -0.14, 0.16, -0.02)}) {
        LinkageTable t;
        CHECK(t.build(l, -0.08, 0.08));
        const Planar2D ref(l);
        const Real h = rollCentreHeight(t.at(0.0), 0.78, 0.33), g = ref.rollCentre(0.78, 0.33);
        std::cout << "    roll centre: path " << h * 1000 << " mm, construction " << g * 1000 << " mm\n";
        CHECK_NEAR(h, g, 0.004);
    }
    // A slider (no scrub, no camber change) has its roll centre at the ground.
    CHECK_NEAR(rollCentreHeight(LinkagePose{}, 0.78, 0.33), 0.0, 1e-12);
}

TEST_CASE("kinematics: the helper linkages are sane and the motion ratio normalises to the design position") {
    for (const LinkageDef& l : {makeDoubleWishbone(0.40, 0.28, 0.30, 0.06, 0.0, 0.55), makeDoubleWishbone(0.45, 0.33, 0.30, 0.05, -0.08, 0.6),
                                makeMacPherson(0.38, 0.58, 0.07, 0.10), makeDoubleWishbone(0.55, 0.50, 0.26, 0.03, 0.05, 0.9)}) {
        LinkageTable t;
        std::string err;
        CHECK(t.build(l, -0.10, 0.10, &err));
        if (!t.valid()) std::cout << "    " << err << "\n";
        CHECK(t.at(0.06).camber < 0);                               // negative camber in bump
        CHECK(std::abs(t.at(0.06).toe) < 0.012);                    // under 0.7 deg of bump steer at 60 mm
        CHECK_NEAR(t.at(0.0).motionRatio / t.designMotionRatio(), 1.0, 1e-9);
        CHECK_NEAR(t.springDeflection(0.05), 0.05, 0.002);          // ~1:1 in wheel metres near design
        CHECK_RANGE(t.at(0.08).motionRatio / t.designMotionRatio(), 0.85, 1.2);
    }
    // Unreachable travel is refused, not folded over.
    LinkageDef tiny = makeDoubleWishbone(0.12, 0.10, 0.30, 0.06);
    LinkageTable t;
    std::string err;
    CHECK(!t.build(tiny, -0.25, 0.25, &err));
    CHECK(!err.empty());
}

TEST_CASE("kinematics: a high roll centre jacks the car and reduces roll at the same spring and bar rates") {
    const VehicleDefinition slider = vehicles::referenceSportsRwd();
    // Arms angled so the roll centre sits ~95 mm above the ground (front and rear).
    const LinkageDef wb = makeDoubleWishbone(0.40, 0.28, 0.30, 0.06, 0.0, 0.55);
    const VehicleDefinition wishbone = withLinkage(slider, wb, wb);
    const Steady a = steadyTurn(slider, 0.09, 20.0), b = steadyTurn(wishbone, 0.09, 20.0);
    const Real ga = a.roll / a.latG, gb = b.roll / b.latG;
    std::cout << "    roll gradient: slider " << ga * 57.3 << " deg/g, wishbone " << gb * 57.3 << " deg/g at "
              << a.latG << " / " << b.latG << " g\n";
    CHECK_RANGE(a.latG, 0.4, 1.0);
    CHECK_RANGE(b.latG, 0.4, 1.0);
    CHECK(gb < 0.9 * ga);  // the lateral force at the patch holds the body up through the links
    // Lateral load transfer is set by the CoM height and the track, not by the roll centre.
    const Real ta = (a.transferFront + a.transferRear) / a.latG, tb = (b.transferFront + b.transferRear) / b.latG;
    CHECK_NEAR(tb / ta, 1.0, 0.08);
}

TEST_CASE("kinematics: inclined pivot axes give anti-dive under braking") {
    const VehicleDefinition base = vehicles::referenceSportsRwd();
    const LinkageDef level = makeDoubleWishbone(0.40, 0.28, 0.30, 0.06, 0.0), inclined = makeDoubleWishbone(0.40, 0.28, 0.30, 0.06, 0.30);
    const VehicleDefinition noAnti = withLinkage(base, level, level), anti = withLinkage(base, inclined, level);
    const Dive a = brakeDive(noAnti, 0.45), b = brakeDive(anti, 0.45);
    std::cout << "    dive: level " << a.maxPitch * 57.3 << " deg, anti-dive " << b.maxPitch * 57.3 << " deg; distance "
              << a.distance << " / " << b.distance << " m\n";
    CHECK(a.maxPitch > 0.01);
    CHECK(b.maxPitch < 0.85 * a.maxPitch);
    CHECK_NEAR(b.distance / a.distance, 1.0, 0.03);
}

TEST_CASE("kinematics: lowering the car in the garage moves the operating point along the camber curve") {
    const LinkageDef wb = makeDoubleWishbone(0.40, 0.28, 0.30, 0.06);
    const VehicleDefinition stock = withLinkage(vehicles::referenceSportsRwd(), wb, wb);
    VehicleSetup s = stockSetup(stock);
    s.front.rideHeightMm = s.rear.rideHeightMm = -20;
    const VehicleDefinition low = applySetup(stock, s);
    auto restCamber = [](const VehicleDefinition& d) {
        Vehicle v(d, makePreset(DrivingPreset::Simulation));
        FlatGround g;
        v.reset({0, 0, 0.6}, 0, g);
        DriverInput in;
        for (int i = 0; i < 360; ++i) v.step(1.0 / 360, in, g);
        return v.telemetry().wheels[0].camber;
    };
    const Real c0 = restCamber(stock), c1 = restCamber(low);
    std::cout << "    camber at rest: stock " << c0 * 57.3 << " deg, lowered 20 mm " << c1 * 57.3 << " deg\n";
    LinkageTable t;
    CHECK(t.build(wb, -0.1, 0.1));
    CHECK_NEAR(c1 - c0, t.at(0.02).camber, 0.0025);  // 20 mm further into bump
}

TEST_CASE("kinematics: a linkage car stays rate-independent on a kerb strike and never allocates") {
    const LinkageDef wb = makeDoubleWishbone(0.40, 0.28, 0.30, 0.06, 0.1);
    const VehicleDefinition d = withLinkage(vehicles::referenceSportsRwd(), wb, wb);
    RunConfig ref;
    ref.vehicle = d;
    ref.assists = makePreset(DrivingPreset::Simulation);
    ref.hz = 1000;
    RunConfig h = ref;
    h.hz = 240;
    const RoughRoadResult a = curbStrike(ref, 60), b = curbStrike(h, 60);
    std::cout << "    kerb heave 1000 Hz " << a.maxHeave * 1000 << " mm, 240 Hz " << b.maxHeave * 1000 << " mm\n";
    CHECK_NEAR(b.maxHeave / a.maxHeave, 1.0, 0.10);
    CHECK_NEAR(b.rmsFrameAccelG / a.rmsFrameAccelG, 1.0, 0.12);
}

TEST_CASE("kinematics: perturbed linkages are either rejected or run without NaN") {
    std::mt19937 rng(7);
    std::uniform_real_distribution<Real> jitter(-0.04, 0.04);
    int valid = 0, rejected = 0;
    for (int i = 0; i < 40; ++i) {
        LinkageDef l = i % 2 ? makeDoubleWishbone(0.40, 0.28, 0.30, 0.06) : makeMacPherson(0.38, 0.58, 0.07);
        for (Vec3* p : {&l.lowerPivotFront, &l.lowerPivotRear, &l.lowerBallJoint, &l.upperPivotFront, &l.upperPivotRear,
                        &l.upperBallJoint, &l.strutTop, &l.tieRodInner, &l.tieRodOuter, &l.springArm, &l.springBody})
            *p += Vec3{jitter(rng), jitter(rng), jitter(rng)};
        const VehicleDefinition d = withLinkage(vehicles::referenceSportsRwd(), l, l);
        Vehicle v(d, makePreset(DrivingPreset::Fun));
        if (!v.isValid()) { ++rejected; continue; }
        ++valid;
        FlatGround g;
        v.reset({0, 0, 0.6}, 0, g);
        DriverInput in;
        for (int k = 0; k < 360 * 3; ++k) {
            in.throttle = 0.8;
            in.steer = std::sin(k * 0.01);
            v.step(1.0 / 360, in, g);
        }
        CHECK(!v.faulted());
        CHECK(std::isfinite(v.telemetry().speed));
    }
    std::cout << "    " << valid << " valid, " << rejected << " rejected\n";
    CHECK(valid >= 20);
}

TEST_CASE("kinematics: the Stella and the real cars build with their linkages and report roll centres") {
    for (const VehicleDefinition& d : {vehicles::rosserStellaV8(), vehicles::porsche911CarreraS(), vehicles::mercedesAmgS63(),
                                       vehicles::mercedesS600Guard(), vehicles::formula1Car2026()}) {
        Vehicle v(d, makePreset(DrivingPreset::Sport));
        CHECK(v.isValid());
        if (!v.isValid()) { std::cout << "    " << d.name << ": " << v.error() << "\n"; continue; }
        FlatGround g;
        v.reset({0, 0, 0.6}, 0, g);
        DriverInput in;
        for (int k = 0; k < 360; ++k) v.step(1.0 / 360, in, g);
        const TelemetryFrame& t = v.telemetry();
        std::cout << "    " << d.name << ": roll centres " << t.rollCentreHeight[0] * 1000 << " / " << t.rollCentreHeight[1] * 1000
                  << " mm, rest camber " << t.wheels[0].camber * 57.3 << " deg\n";
        CHECK_RANGE(t.rollCentreHeight[0], -0.05, 0.25);
        CHECK_RANGE(t.rollCentreHeight[1], -0.05, 0.30);
        CHECK(d.wheels[0].suspension.linkage.type != LinkageType::Slider);
    }
}
