// OpenRoad Dynamics — WebAssembly host ABI for many vehicles in a host rigid-body world.
// SPDX-License-Identifier: MIT
//
// Built for Georgia State Pursuit (zrossermcintosh.com/gsp), a police-pursuit physics
// demonstrator in which the page's rigid-body engine (Rapier) is the host: it owns every car's
// chassis, its collisions with other cars, barriers and props, and integrates it at the page's
// fixed step. The core runs each car in ChassisMode::External: every host step the page writes
// the chassis pose and velocity it integrated (gsp_set_body), calls gsp_step, and applies the
// average force and torque the tyres, suspension, drivetrain, aero and body hull produced over
// that step (gsp_forces) to its rigid body. Tyres and suspension run on vehicle-local substeps
// (Vehicle::stepSubstepped). Gravity is the host's.
//
// Every car is an ordinary ord::Vehicle built from a flat parameter block (gsp_params, the
// GSP_* indices below), so the page can expose all of it to sliders and JSON. Player, suspect,
// police and traffic all use this same path; only the fidelity tier may differ.
//
// The ground is a heightfield with a surface class per cell, uploaded by the page (it builds
// the same field into its own collider and render mesh), plus flat elevated decks (an overpass)
// and the weather: rain makes paved surfaces wet, lays a water film and fills puddles.
//
// Frames: ISO 8855 world (x east, y north, z up, metres). The body origin is the centre of mass.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

#include "ord/ground.hpp"
#include "ord/presets.hpp"
#include "ord/vehicle.hpp"

#ifdef __EMSCRIPTEN__
#include <emscripten/emscripten.h>
#define GSP_EXPORT extern "C" EMSCRIPTEN_KEEPALIVE
#else
#define GSP_EXPORT extern "C"
#endif

using namespace ord;

namespace {

// ---- Vehicle parameter block (mirrored in the page: src/sim/vehicle/params.ts) ----
enum Param : int {
    GSP_MASS = 0,          // kg, total
    GSP_IXX = 1,           // kg m^2 roll inertia about the CoM
    GSP_IYY = 2,           // pitch
    GSP_IZZ = 3,           // yaw
    GSP_WHEELBASE = 4,     // m
    GSP_FRONT_WEIGHT = 5,  // static fraction of the weight on the front axle (0.3..0.7)
    GSP_TRACK_F = 6,       // m
    GSP_TRACK_R = 7,
    GSP_CG_HEIGHT = 8,     // m above the ground at rest
    GSP_RADIUS_F = 9,      // m, tyre radius
    GSP_RADIUS_R = 10,
    GSP_MUX_F = 11,        // peak longitudinal friction, front
    GSP_MUY_F = 12,        // peak lateral friction, front
    GSP_MUX_R = 13,
    GSP_MUY_R = 14,
    GSP_CORNER_STIFF = 15, // cornering stiffness, Fz0 per rad
    GSP_SPRING_F = 16,     // N/m at the wheel
    GSP_SPRING_R = 17,
    GSP_TRAVEL = 18,       // m, full droop to full bump
    GSP_BUMP_F = 19,       // N s/m
    GSP_REBOUND_F = 20,
    GSP_BUMP_R = 21,
    GSP_REBOUND_R = 22,
    GSP_ARB_F = 23,        // N/m at the wheel
    GSP_ARB_R = 24,
    GSP_STEER_LOCK = 25,   // rad, road wheel at full lock
    GSP_BRAKE_F = 26,      // N m per wheel at full pedal
    GSP_BRAKE_R = 27,
    GSP_HANDBRAKE = 28,    // N m per rear wheel
    GSP_DRIVE = 29,        // 0 RWD, 1 FWD, 2 AWD
    GSP_AWD_FRONT = 30,    // AWD front torque share
    GSP_DIFF = 31,         // 0 open, 1 locked, 2 clutch LSD (driven axles)
    GSP_FINAL_DRIVE = 32,
    GSP_TRANS = 33,        // 0 automatic, 1 dual clutch, 2 manual H-pattern
    GSP_SHIFT_TIME = 34,   // s
    GSP_GEAR_COUNT = 35,   // 1..10
    GSP_GEAR0 = 36,        // 10 forward ratios
    GSP_REVERSE = 46,
    GSP_IDLE = 47,         // rpm
    GSP_LIMITER = 48,      // rpm
    GSP_ENGINE_INERTIA = 49,
    GSP_CURVE_COUNT = 50,  // 2..8 torque curve points
    GSP_CURVE0 = 51,       // 8 x (rpm, N m)
    GSP_DRAG = 67,         // Cd A, m^2
    GSP_TOP_SPEED = 68,    // m/s electronic limiter (0 = none)
    GSP_OVERHANG_F = 69,   // m ahead of the front axle (body hull)
    GSP_OVERHANG_R = 70,
    GSP_HALF_WIDTH = 71,
    GSP_BODY_HEIGHT = 72,  // floor to roof
    GSP_BODY_FRICTION = 73,
    GSP_UNSPRUNG = 74,     // kg per corner
    GSP_ACKERMANN = 75,
    GSP_CLUTCH_TORQUE = 76,
    GSP_WET_GRIP = 77,     // tyre mu multiplier on a fully wet surface
    GSP_LOOSE_GRIP = 78,   // ... on a fully loose surface
    GSP_LSD_PRELOAD = 79,
    GSP_LSD_POWER = 80,
    GSP_LSD_COAST = 81,
    GSP_PARAM_COUNT = 96,
};

constexpr int kMaxCars = 48;
constexpr int kHeader = 50, kPerWheel = 18;
constexpr int kStateSize = kHeader + kPerWheel * 4;

double clampd(double v, double lo, double hi) { return std::isfinite(v) ? std::min(hi, std::max(lo, v)) : lo; }

VehicleDefinition buildDefinition(const double* p, std::string* why) {
    VehicleDefinition d;
    d.name = "GSP vehicle";
    const double mass = clampd(p[GSP_MASS], 300, 12000);
    d.mass = mass;
    d.inertia = {clampd(p[GSP_IXX], 50, 50000), clampd(p[GSP_IYY], 100, 200000), clampd(p[GSP_IZZ], 100, 200000)};
    d.centerOfMass = {0, 0, 0};
    const double L = clampd(p[GSP_WHEELBASE], 1.5, 6.0);
    const double fw = clampd(p[GSP_FRONT_WEIGHT], 0.25, 0.75);
    const double xf = L * (1 - fw), xr = -L * fw;
    const double cg = clampd(p[GSP_CG_HEIGHT], 0.3, 1.6);
    const double unsprung = clampd(p[GSP_UNSPRUNG], 10, 150);
    const double travel = clampd(p[GSP_TRAVEL], 0.06, 0.45);
    const double loadF = mass * kGravity * fw * 0.5, loadR = mass * kGravity * (1 - fw) * 0.5;

    auto wheel = [&](bool front) {
        WheelDef w;
        w.unsprungMass = unsprung;
        w.tire.radius = clampd(front ? p[GSP_RADIUS_F] : p[GSP_RADIUS_R], 0.22, 0.6);
        w.tire.nominalLoad = std::max(1500.0, front ? loadF : loadR);
        w.tire.muX = clampd(front ? p[GSP_MUX_F] : p[GSP_MUX_R], 0.2, 2.5);
        w.tire.muY = clampd(front ? p[GSP_MUY_F] : p[GSP_MUY_R], 0.2, 2.5);
        w.tire.cornerStiffness = clampd(p[GSP_CORNER_STIFF], 4, 40);
        w.tire.wetGrip = clampd(p[GSP_WET_GRIP], 0.2, 1.5);
        w.tire.looseGrip = clampd(p[GSP_LOOSE_GRIP], 0.2, 1.5);
        w.tire.wheelInertia = 1.2 * std::pow(w.tire.radius / 0.33, 4) * (mass > 2300 ? 1.6 : 1.0);
        w.suspension.travel = travel;
        w.suspension.springRate = clampd(front ? p[GSP_SPRING_F] : p[GSP_SPRING_R], 8000, 400000);
        w.suspension.bumpDamping = clampd(front ? p[GSP_BUMP_F] : p[GSP_BUMP_R], 300, 40000);
        w.suspension.reboundDamping = clampd(front ? p[GSP_REBOUND_F] : p[GSP_REBOUND_R], 300, 60000);
        w.maxSteerAngle = front ? clampd(p[GSP_STEER_LOCK], 0.15, 1.1) : 0.0;
        w.maxBrakeTorque = clampd(front ? p[GSP_BRAKE_F] : p[GSP_BRAKE_R], 100, 20000);
        w.handbrakeTorque = front ? 0.0 : clampd(p[GSP_HANDBRAKE], 0, 20000);
        // Hardpoint height so that the car rests with its CoM at cg above flat ground.
        const double load = front ? loadF : loadR;
        const double xs = clampd((load - unsprung * kGravity) / w.suspension.springRate, 0.0, travel);
        const double pen = load / w.tire.verticalStiffness;
        const double hz = -cg + (w.tire.radius - pen) + (travel - xs);
        return std::make_pair(w, hz);
    };
    auto [wf, hzf] = wheel(true);
    auto [wr, hzr] = wheel(false);
    const double tf = 0.5 * clampd(p[GSP_TRACK_F], 1.0, 2.6), tr = 0.5 * clampd(p[GSP_TRACK_R], 1.0, 2.6);
    auto place = [&](WheelDef w, double x, double y, double z, bool left) {
        w.suspension.hardpoint = {x, y, z};
        w.isLeft = left;
        d.wheels.push_back(w);
    };
    place(wf, xf, tf, hzf, true);    // 0 FL
    place(wf, xf, -tf, hzf, false);  // 1 FR
    place(wr, xr, tr, hzr, true);    // 2 RL
    place(wr, xr, -tr, hzr, false);  // 3 RR
    d.axles = {{0, 1, clampd(p[GSP_ARB_F], 0, 200000)}, {2, 3, clampd(p[GSP_ARB_R], 0, 200000)}};
    d.steeringAckermann = clampd(p[GSP_ACKERMANN], 0, 1);
    d.topSpeedLimit = clampd(p[GSP_TOP_SPEED], 0, 150);

    auto& e = d.drivetrain.engine;
    const int nc = static_cast<int>(clampd(p[GSP_CURVE_COUNT], 2, 8));
    e.torqueCurve.clear();
    double lastRpm = 0;
    for (int k = 0; k < nc; ++k) {
        double rpm = clampd(p[GSP_CURVE0 + 2 * k], 300, 20000);
        if (rpm <= lastRpm) rpm = lastRpm + 50;
        e.torqueCurve.push_back({rpm, clampd(p[GSP_CURVE0 + 2 * k + 1], 0, 4000)});
        lastRpm = rpm;
    }
    e.idleRpm = clampd(p[GSP_IDLE], 400, 3000);
    e.limiterRpm = clampd(p[GSP_LIMITER], e.idleRpm + 1000, 20000);
    e.inertia = clampd(p[GSP_ENGINE_INERTIA], 0.03, 2.0);
    d.drivetrain.clutch.maxTorque = clampd(p[GSP_CLUTCH_TORQUE], 100, 5000);
    auto& gb = d.drivetrain.gearbox;
    const int ng = static_cast<int>(clampd(p[GSP_GEAR_COUNT], 1, 10));
    gb.forwardRatios.clear();
    for (int k = 0; k < ng; ++k) gb.forwardRatios.push_back(clampd(p[GSP_GEAR0 + k], 0.3, 8));
    gb.reverseRatio = clampd(p[GSP_REVERSE], 1, 8);
    gb.shiftTime = clampd(p[GSP_SHIFT_TIME], 0.03, 1.5);
    const int trans = static_cast<int>(p[GSP_TRANS]);
    gb.kind = trans == 1 ? TransmissionKind::DualClutch : trans == 2 ? TransmissionKind::ManualHPattern : TransmissionKind::Automatic;
    gb.upshift = trans == 2 ? UpshiftControl::LiftAndRamp : UpshiftControl::PowerShift;
    DifferentialDef diff;
    const int dt = static_cast<int>(p[GSP_DIFF]);
    diff.type = dt == 1 ? DifferentialType::Locked : dt == 2 ? DifferentialType::ClutchLsd : DifferentialType::Open;
    diff.finalDrive = clampd(p[GSP_FINAL_DRIVE], 1.5, 8);
    diff.preload = clampd(p[GSP_LSD_PRELOAD], 0, 1000);
    diff.powerRamp = clampd(p[GSP_LSD_POWER], 0, 1);
    diff.coastRamp = clampd(p[GSP_LSD_COAST], 0, 1);
    const int drive = static_cast<int>(p[GSP_DRIVE]);
    d.drivetrain.drivenAxles.clear();
    if (drive == 2) {
        const double share = clampd(p[GSP_AWD_FRONT], 0.05, 0.95);
        DrivenAxle a, b;
        a.axleIndex = 0; a.torqueShare = share; a.differential = diff; a.differential.type = DifferentialType::Open;
        b.axleIndex = 1; b.torqueShare = 1 - share; b.differential = diff;
        d.drivetrain.drivenAxles = {a, b};
        d.drivetrain.center = CenterCoupling::ClutchLsd;
    } else {
        DrivenAxle a;
        a.axleIndex = drive == 1 ? 0 : 1;
        a.torqueShare = 1;
        a.differential = diff;
        d.drivetrain.drivenAxles = {a};
    }
    d.aero.dragArea = clampd(p[GSP_DRAG], 0.2, 3.0);
    d.aero.sideForceArea = 1.2 * d.aero.dragArea;
    d.body.overhangFront = clampd(p[GSP_OVERHANG_F], 0.3, 2.5);
    d.body.overhangRear = clampd(p[GSP_OVERHANG_R], 0.3, 2.5);
    d.body.halfWidth = clampd(p[GSP_HALF_WIDTH], 0.6, 1.4);
    d.body.height = clampd(p[GSP_BODY_HEIGHT], 0.8, 2.6);
    d.body.friction = clampd(p[GSP_BODY_FRICTION], 0.1, 1.2);
    const std::string err = d.validate();
    if (why) *why = err;
    return d;
}

// ---- Ground ----
struct Deck { double cx, cy, z, hl, hw, c, s, grade; int cls; };

std::uint64_t mixHash(std::uint64_t h) {
    h ^= h >> 33; h *= 0xff51afd7ed558ccdULL; h ^= h >> 33; h *= 0xc4ceb9fe1a85ec53ULL; h ^= h >> 33;
    return h;
}
constexpr double kPuddleCell = 18.0;
constexpr int kPuddleClass = 15;

struct GspGround final : GroundQuery {
    int nx = 0, ny = 0;
    double x0 = 0, y0 = 0, cell = 1;
    std::vector<float> h;
    std::vector<uint8_t> cls;
    std::array<SurfaceProperties, 16> surf{};
    std::array<double, 16> wetLoss{};   // fraction of grip a fully wet surface loses
    std::array<uint8_t, 16> paved{};    // rain lays a film and puddles on these
    std::vector<Deck> decks;
    double rain = 0;
    double puddleAmount = 1;  // 0 = no puddles

    double cellH(int i, int j) const {
        i = std::clamp(i, 0, nx - 1); j = std::clamp(j, 0, ny - 1);
        return h[static_cast<std::size_t>(j) * nx + i];
    }
    double fieldHeight(double x, double y) const {
        if (nx < 2) return 0;
        const double fx = (x - x0) / cell, fy = (y - y0) / cell;
        const int i = static_cast<int>(std::floor(fx)), j = static_cast<int>(std::floor(fy));
        const double tx = fx - i, ty = fy - j;
        const double a = cellH(i, j), b = cellH(i + 1, j), c = cellH(i, j + 1), d = cellH(i + 1, j + 1);
        return (a * (1 - tx) + b * tx) * (1 - ty) + (c * (1 - tx) + d * tx) * ty;
    }
    int classAt(double x, double y) const {
        if (nx < 2) return 0;
        const int i = std::clamp(static_cast<int>(std::lround((x - x0) / cell)), 0, nx - 1);
        const int j = std::clamp(static_cast<int>(std::lround((y - y0) / cell)), 0, ny - 1);
        return cls[static_cast<std::size_t>(j) * nx + i] & 15;
    }
    // Top drivable height at (x, y) at or below zMax (decks count only below zMax).
    double heightAt(double x, double y, double zMax = 1e9, int* clsOut = nullptr) const {
        double best = fieldHeight(x, y);
        if (clsOut) *clsOut = classAt(x, y);
        for (const Deck& d : decks) {
            const double lx = (x - d.cx) * d.c + (y - d.cy) * d.s, ly = -(x - d.cx) * d.s + (y - d.cy) * d.c;
            if (std::abs(lx) > d.hl || std::abs(ly) > d.hw) continue;
            const double z = d.z + d.grade * lx;
            if (z > best && z <= zMax) { best = z; if (clsOut) *clsOut = d.cls; }
        }
        return best;
    }
    double puddleDepth(double x, double y) const {
        if (rain <= 0 || puddleAmount <= 0) return 0;
        const long i0 = static_cast<long>(std::floor(x / kPuddleCell)), j0 = static_cast<long>(std::floor(y / kPuddleCell));
        const std::uint64_t hh = mixHash((static_cast<std::uint64_t>(i0) * 0x9E3779B97F4A7C15ULL) ^ static_cast<std::uint64_t>(j0) ^ 0x5bd1e995ULL);
        if (static_cast<double>(hh & 1023) > 260 * puddleAmount) return 0;
        const double u1 = static_cast<double>((hh >> 10) & 1023) / 1023, u2 = static_cast<double>((hh >> 20) & 1023) / 1023;
        const double u3 = static_cast<double>((hh >> 30) & 1023) / 1023, u4 = static_cast<double>((hh >> 40) & 1023) / 1023;
        const double px = (i0 + 0.15 + 0.7 * u1) * kPuddleCell, py = (j0 + 0.15 + 0.7 * u2) * kPuddleCell;
        const double r = 0.8 + 1.6 * u3, depth = 0.005 + 0.013 * u4;
        const double dx = (x - px) / r, dy = (y - py) / (0.6 * r), q = dx * dx + dy * dy;
        if (q >= 1) return 0;
        const int c = classAt(px, py);
        if (!paved[static_cast<std::size_t>(c)]) return 0;
        return depth * std::min(1.0, 0.25 + rain) * (1 - q * q);
    }
    GroundHit raycast(const Vec3& o, const Vec3& d, Real maxDistance) const override {
        GroundHit r;
        if (d.z >= -1e-6) return r;
        // heightfield: fixed point (gentle terrain), then a march + bisection if it did not settle
        Real t = (fieldHeight(o.x, o.y) - o.z) / d.z;
        bool ok = false;
        for (int k = 0; k < 8; ++k) {
            const Vec3 p = o + d * t;
            const Real tn = (fieldHeight(p.x, p.y) - o.z) / d.z;
            if (std::abs(tn - t) < 1e-5) { t = tn; ok = true; break; }
            t = tn;
        }
        if (!ok) {
            auto f = [&](double tt) { const Vec3 p = o + d * tt; return p.z - fieldHeight(p.x, p.y); };
            double t0 = 0, f0 = f(0);
            if (f0 < 0) t = 0;
            else {
                double t1 = 0, f1 = f0;
                bool cross = false;
                while (true) {
                    t1 = t0 + std::max(0.02, 0.25 * f0);
                    if (t1 > maxDistance) { t1 = maxDistance; f1 = f(t1); cross = f1 <= 0; break; }
                    f1 = f(t1);
                    if (f1 <= 0) { cross = true; break; }
                    t0 = t1; f0 = f1;
                }
                if (!cross) t = maxDistance + 1;
                else {
                    for (int k = 0; k < 14; ++k) {
                        const double tm = 0.5 * (t0 + t1), fm = f(tm);
                        if (fm > 0) t0 = tm; else t1 = tm;
                    }
                    t = t1;
                }
            }
        }
        bool hit = t >= 0 && t <= maxDistance;
        Vec3 n{0, 0, 1};
        int c = 0;
        if (hit) {
            const Vec3 p = o + d * t;
            const double e = 0.5 * cell;
            const double gx = (fieldHeight(p.x + e, p.y) - fieldHeight(p.x - e, p.y)) / (2 * e);
            const double gy = (fieldHeight(p.x, p.y + e) - fieldHeight(p.x, p.y - e)) / (2 * e);
            n = normalized(Vec3{-gx, -gy, 1});
            c = classAt(p.x, p.y);
        }
        // decks: planes over rectangles; only from above
        for (const Deck& dk : decks) {
            // plane z = dk.z + grade * lx, lx along the deck
            const double gxw = dk.grade * dk.c, gyw = dk.grade * dk.s;   // dz/dx, dz/dy in world
            const double zc = dk.z - gxw * dk.cx - gyw * dk.cy;
            const double den = d.z - gxw * d.x - gyw * d.y;
            if (std::abs(den) < 1e-9) continue;
            const double tt = (gxw * o.x + gyw * o.y + zc - o.z) / den;
            if (tt < -0.05 || tt > maxDistance || (hit && tt >= t)) continue;
            const Vec3 p = o + d * tt;
            const double lx = (p.x - dk.cx) * dk.c + (p.y - dk.cy) * dk.s, ly = -(p.x - dk.cx) * dk.s + (p.y - dk.cy) * dk.c;
            if (std::abs(lx) > dk.hl || std::abs(ly) > dk.hw) continue;
            hit = true; t = std::max(0.0, tt); n = normalized(Vec3{-gxw, -gyw, 1}); c = dk.cls;
        }
        if (!hit) return r;
        r.hit = true;
        r.distance = t;
        r.point = o + d * t;
        r.normal = n;
        r.surfaceId = c;
        r.surface = surf[static_cast<std::size_t>(c)];
        if (rain > 0) {
            SurfaceProperties& s = r.surface;
            s.frictionScale *= 1 - wetLoss[static_cast<std::size_t>(c)] * rain;
            s.wetness = std::max(s.wetness, rain);
            if (paved[static_cast<std::size_t>(c)]) {
                s.waterDepth = std::max(s.waterDepth, 0.0012 * rain);
                const double pd = puddleDepth(r.point.x, r.point.y);
                if (pd > 0) { s.waterDepth = std::max(s.waterDepth, pd); r.surfaceId = kPuddleClass; }
            } else {
                s.looseness = std::max(s.looseness, 0.5 * rain);
            }
        }
        return r;
    }
};

struct Car {
    std::unique_ptr<Vehicle> v;
    AssistConfig assists;
    DriverInput input;
    int fidelity = 0;
    std::array<double, kStateSize> out{};
    std::array<double, 6> forces{};
};

GspGround g_ground;
std::array<Car, kMaxCars> g_cars;
std::array<double, GSP_PARAM_COUNT> g_params{};
std::vector<double> g_puddles;

Car* car(int h) {
    if (h < 0 || h >= kMaxCars || !g_cars[static_cast<std::size_t>(h)].v) return nullptr;
    return &g_cars[static_cast<std::size_t>(h)];
}

VehicleFidelity fidelityFor(int f) {
    VehicleFidelity v = f <= 0 ? VehicleFidelity::lod0() : VehicleFidelity::lod1();
    if (f == 0) v.envelopeProbes = 0;  // smooth roads: the patch is enough, probes cost rays
    v.telemetry = true;                // the page draws every car's wheels from it
    return v;
}

void publish(Car& c) {
    const Vehicle& v = *c.v;
    double* o = c.out.data();
    const RigidBodyState& b = v.state();
    const TelemetryFrame& t = v.telemetry();
    const VehicleDefinition& d = v.definition();
    o[0] = b.position.x; o[1] = b.position.y; o[2] = b.position.z;
    o[3] = b.orientation.w; o[4] = b.orientation.x; o[5] = b.orientation.y; o[6] = b.orientation.z;
    // speed, lateral velocity and yaw rate from the chassis state itself (also right after a placement)
    const Vec3 vb = v.bodyVelocity();
    o[7] = vb.x; o[8] = vb.y; o[9] = v.yawRate(); o[10] = std::abs(vb.x) > 0.5 ? std::atan2(vb.y, std::abs(vb.x)) : 0.0;
    o[11] = t.accelBody.x; o[12] = t.accelBody.y; o[13] = t.accelBody.z;
    o[14] = t.drivetrain.engineRpm; o[15] = t.gear; o[16] = t.steerAngle; o[17] = t.throttleOut;
    o[18] = t.assists.tcsThrottleCut; o[19] = t.assists.countersteerAngle; o[20] = t.assists.escYawError;
    o[21] = t.time; o[22] = static_cast<double>(t.wheelCount); o[23] = v.faulted() ? 1 : 0;
    o[24] = t.assists.shifting ? 1 : 0; o[25] = d.wheels.empty() ? 0 : d.wheels[0].tire.radius;
    o[26] = t.clutchEngagement; o[27] = t.steerInput; o[28] = t.throttleInput; o[29] = t.brakeInput;
    o[30] = static_cast<double>(v.groundQueries()); o[31] = t.drivetrain.engineTorque;
    o[32] = t.assists.driftActive ? 1 : 0; o[33] = t.assists.driftTargetSideslip;
    o[34] = t.assists.driftThrottleCut; o[35] = t.assists.driftSteerAngle;
    o[36] = t.upright; o[37] = static_cast<double>(t.bodyContacts); o[38] = t.bodyContactForce;
    o[39] = v.hostSync().positionError; o[40] = v.hostSync().angleError;
    o[41] = static_cast<double>(d.drivetrain.gearbox.forwardRatios.size());
    o[42] = d.drivetrain.engine.limiterRpm; o[43] = d.drivetrain.engine.idleRpm;
    o[44] = b.velocity.x; o[45] = b.velocity.y; o[46] = b.velocity.z;
    o[47] = b.angularVelocity.x; o[48] = b.angularVelocity.y; o[49] = b.angularVelocity.z;
    for (std::size_t w = 0; w < 4; ++w) {
        double* p = o + kHeader + kPerWheel * static_cast<int>(w);
        if (w >= t.wheelCount) { std::fill(p, p + kPerWheel, 0.0); continue; }
        const WheelTelemetry& wt = t.wheels[w];
        const WheelDef& wd = d.wheels[w];
        p[0] = wd.suspension.hardpoint.x; p[1] = wd.suspension.hardpoint.y;
        p[2] = wd.suspension.hardpoint.z - (wd.suspension.travel - wt.compression);  // wheel centre z, body frame
        p[3] = wd.tire.radius; p[4] = wt.steerAngle; p[5] = wt.omega;
        p[6] = wt.load; p[7] = wt.slipRatio; p[8] = wt.slipAngle; p[9] = wt.fx; p[10] = wt.fy;
        p[11] = wt.contact ? 1 : 0; p[12] = wt.brakeLocked ? 1 : 0;
        p[13] = t.assists.absActive[w] ? 1 : 0; p[14] = wt.compression / std::max(1e-6, wd.suspension.travel);
        p[15] = wt.surfaceId; p[16] = wt.gripUsed; p[17] = wt.brakeTemperature;
    }
}

}  // namespace

// ---------------- Ground ----------------

// Allocates the heightfield (nx * ny float heights, row-major, row j at y0 + j * cell) and its
// surface classes (uint8). Returns the heights; gsp_ground_classes() returns the classes.
GSP_EXPORT float* gsp_ground_alloc(int nx, int ny, double x0, double y0, double cell) {
    g_ground.nx = std::max(2, nx);
    g_ground.ny = std::max(2, ny);
    g_ground.x0 = x0; g_ground.y0 = y0; g_ground.cell = cell > 0.05 ? cell : 1.0;
    g_ground.h.assign(static_cast<std::size_t>(g_ground.nx) * g_ground.ny, 0.0f);
    g_ground.cls.assign(static_cast<std::size_t>(g_ground.nx) * g_ground.ny, 0);
    g_ground.decks.clear();
    return g_ground.h.data();
}
GSP_EXPORT uint8_t* gsp_ground_classes() { return g_ground.cls.data(); }

// Surface class `c` (0..15): grip relative to dry asphalt, rolling resistance, roughness (ground
// cache hint), looseness (gravel / soil), micro-texture amplitude and wavelength (rumble strips),
// the fraction of grip it loses when fully wet, and whether rain pools on it.
GSP_EXPORT void gsp_set_surface(int c, double friction, double rolling, double roughness, double looseness,
                                double texAmp, double texWave, double wetLoss, int paved) {
    if (c < 0 || c > 15) return;
    SurfaceProperties s;
    s.frictionScale = clampd(friction, 0.02, 2.0);
    s.rollingResistanceScale = clampd(rolling, 0.1, 30);
    s.roughness = clampd(roughness, 0, 1);
    s.looseness = clampd(looseness, 0, 1);
    s.textureAmplitude = clampd(texAmp, 0, 0.05);
    s.textureWavelength = clampd(texWave, 0.02, 2);
    g_ground.surf[static_cast<std::size_t>(c)] = s;
    g_ground.wetLoss[static_cast<std::size_t>(c)] = clampd(wetLoss, 0, 0.95);
    g_ground.paved[static_cast<std::size_t>(c)] = paved ? 1 : 0;
}

// A flat (or graded) drivable deck: centre, top height at the centre, half length along `yaw`,
// half width, grade (dz per m along the deck) and surface class.
GSP_EXPORT void gsp_add_deck(double cx, double cy, double z, double halfLength, double halfWidth, double yaw, double grade, int c) {
    g_ground.decks.push_back({cx, cy, z, halfLength, halfWidth, std::cos(yaw), std::sin(yaw), grade, std::clamp(c, 0, 15)});
}

GSP_EXPORT double gsp_ground_height(double x, double y, double zMax) { return g_ground.heightAt(x, y, zMax); }
GSP_EXPORT int gsp_ground_class(double x, double y, double zMax) {
    int c = 0;
    g_ground.heightAt(x, y, zMax, &c);
    return c;
}

// rain 0 (dry) .. 1 (downpour); puddles 0 (none) .. 1 (the usual amount) .. 2 (a lot).
GSP_EXPORT void gsp_set_weather(double rain, double puddles) {
    g_ground.rain = clampd(rain, 0, 1);
    g_ground.puddleAmount = clampd(puddles, 0, 3);
}

// Puddles whose centre is within `radius` of (x, y): fills gsp_puddle_buffer() with
// (x, y, radius, depth) per puddle and returns the count (max 256).
GSP_EXPORT int gsp_puddles(double x, double y, double radius) {
    g_puddles.clear();
    if (g_ground.rain <= 0 || g_ground.puddleAmount <= 0) return 0;
    const long i0 = static_cast<long>(std::floor((x - radius) / kPuddleCell)), i1 = static_cast<long>(std::floor((x + radius) / kPuddleCell));
    const long j0 = static_cast<long>(std::floor((y - radius) / kPuddleCell)), j1 = static_cast<long>(std::floor((y + radius) / kPuddleCell));
    for (long j = j0; j <= j1; ++j)
        for (long i = i0; i <= i1; ++i) {
            const std::uint64_t hh = mixHash((static_cast<std::uint64_t>(i) * 0x9E3779B97F4A7C15ULL) ^ static_cast<std::uint64_t>(j) ^ 0x5bd1e995ULL);
            if (static_cast<double>(hh & 1023) > 260 * g_ground.puddleAmount) continue;
            const double u1 = static_cast<double>((hh >> 10) & 1023) / 1023, u2 = static_cast<double>((hh >> 20) & 1023) / 1023;
            const double u3 = static_cast<double>((hh >> 30) & 1023) / 1023, u4 = static_cast<double>((hh >> 40) & 1023) / 1023;
            const double px = (i + 0.15 + 0.7 * u1) * kPuddleCell, py = (j + 0.15 + 0.7 * u2) * kPuddleCell;
            if (!g_ground.paved[static_cast<std::size_t>(g_ground.classAt(px, py))]) continue;
            g_puddles.insert(g_puddles.end(), {px, py, 0.8 + 1.6 * u3, (0.005 + 0.013 * u4) * std::min(1.0, 0.25 + g_ground.rain)});
            if (g_puddles.size() >= 4 * 256) return 256;
        }
    return static_cast<int>(g_puddles.size() / 4);
}
GSP_EXPORT double* gsp_puddle_buffer() { return g_puddles.data(); }

// ---------------- Vehicles ----------------

// The parameter block gsp_create / gsp_rebuild read (GSP_PARAM_COUNT doubles).
GSP_EXPORT double* gsp_params() { return g_params.data(); }
GSP_EXPORT int gsp_param_count() { return GSP_PARAM_COUNT; }
GSP_EXPORT int gsp_state_size() { return kStateSize; }
GSP_EXPORT int gsp_state_header() { return kHeader; }
GSP_EXPORT int gsp_state_per_wheel() { return kPerWheel; }
GSP_EXPORT int gsp_max_cars() { return kMaxCars; }

// Checks the parameter block: 1 if it makes a valid car.
GSP_EXPORT int gsp_validate() {
    std::string why;
    buildDefinition(g_params.data(), &why);
    return why.empty() ? 1 : 0;
}

// Creates a car from gsp_params() and places it at rest (or rolling at `speed`, m/s forward) on
// the top surface below zHint + 5 at (x, y), heading `yaw` (rad, about +z from +x).
// fidelity: 0 player / pursuit cars (5-point patch, unsprung wheels), 1 traffic (1 point).
// Returns a handle >= 0, or -1.
GSP_EXPORT int gsp_create(double x, double y, double zHint, double yaw, double speed, int fidelity) {
    int slot = -1;
    for (int k = 0; k < kMaxCars; ++k)
        if (!g_cars[static_cast<std::size_t>(k)].v) { slot = k; break; }
    if (slot < 0) return -1;
    std::string why;
    const VehicleDefinition def = buildDefinition(g_params.data(), &why);
    if (!why.empty()) return -1;
    Car& c = g_cars[static_cast<std::size_t>(slot)];
    c.assists = makePreset(DrivingPreset::Sport);
    c.fidelity = fidelity;
    c.v = std::make_unique<Vehicle>(def, c.assists, fidelityFor(fidelity));
    if (!c.v->isValid()) { c.v.reset(); return -1; }
    c.v->mutableState().position = {x, y, zHint};
    c.v->assists().setGear(1);
    if (std::abs(speed) > 0.5) c.v->selectGearForSpeed(speed);
    c.v->placeOnGround(x, y, yaw, g_ground, speed);
    c.v->setChassisMode(ChassisMode::External, true);
    c.input = {};
    publish(c);
    return slot;
}

GSP_EXPORT void gsp_remove(int h) {
    if (Car* c = car(h)) c->v.reset();
}
GSP_EXPORT void gsp_remove_all() {
    for (auto& c : g_cars) c.v.reset();
}

// Places the car again (respawn): same semantics as gsp_create's placement.
GSP_EXPORT void gsp_place(int h, double x, double y, double zHint, double yaw, double speed) {
    Car* c = car(h);
    if (!c) return;
    c->v->setChassisMode(ChassisMode::Internal, false);
    c->v->mutableState().position = {x, y, zHint};
    c->v->assists().setGear(1);
    if (std::abs(speed) > 0.5) c->v->selectGearForSpeed(speed);
    c->v->placeOnGround(x, y, yaw, g_ground, speed);
    c->v->setChassisMode(ChassisMode::External, true);
    publish(*c);
}

// Rebuilds car `h` from gsp_params() where it is, at the speed it has (live tuning). 1 = done.
GSP_EXPORT int gsp_rebuild(int h) {
    Car* c = car(h);
    if (!c) return 0;
    std::string why;
    const VehicleDefinition def = buildDefinition(g_params.data(), &why);
    if (!why.empty()) return 0;
    const RigidBodyState b = c->v->state();
    const Real vx = c->v->forwardSpeed();
    const int gear = static_cast<int>(c->v->telemetry().gear);
    auto nv = std::make_unique<Vehicle>(def, c->assists, fidelityFor(c->fidelity));
    if (!nv->isValid()) return 0;
    nv->setChassisMode(ChassisMode::External, true);
    nv->mutableState() = b;
    nv->setRollingSpeed(vx);
    nv->mutableState() = b;
    if (gear != 0) nv->assists().setGear(gear);
    c->v = std::move(nv);
    publish(*c);
    return 1;
}

// preset: 0 FUN, 1 SPORT, 2 SIMULATION, 3 DRIFT (same physics, different assists).
GSP_EXPORT void gsp_set_preset(int h, int preset) {
    Car* c = car(h);
    if (!c) return;
    const double drift = c->assists.driftAssist;
    c->assists = makePreset(preset == 0 ? DrivingPreset::Fun : preset == 1 ? DrivingPreset::Sport
                            : preset == 3 ? DrivingPreset::Drift : DrivingPreset::Simulation);
    if (preset != 3) c->assists.driftAssist = drift;
    c->v->assists().setConfig(c->assists);
}
// level: 0 off, 1 sport, 2 full.
GSP_EXPORT void gsp_set_aids(int h, int abs, int tcs, int esc, int automatic) {
    Car* c = car(h);
    if (!c) return;
    auto lvl = [](int l) { return l <= 0 ? InterventionLevel::Off : l == 1 ? InterventionLevel::Sport : InterventionLevel::Full; };
    c->assists.abs = abs != 0;
    c->assists.tractionControl = lvl(tcs);
    c->assists.stabilityControl = lvl(esc);
    c->assists.shiftMode = automatic ? ShiftMode::Automatic : ShiftMode::Manual;
    c->v->assists().setConfig(c->assists);
}
// Steering interpretation: mode 0 direct (AI, wheel), 1 speed sensitive (keyboard / pad);
// exponent (input shaping), rate and return rate (1/s, 0 = instant), the front slip angle allowed
// at full lock in speed-sensitive mode (rad), countersteer aid 0..1, drift assist 0..1.
GSP_EXPORT void gsp_set_steering(int h, int mode, double exponent, double rate, double returnRate, double targetSlip,
                                 double countersteer, double drift) {
    Car* c = car(h);
    if (!c) return;
    c->assists.steeringMode = mode == 0 ? SteeringMode::Direct : SteeringMode::SpeedSensitive;
    c->assists.steeringExponent = clampd(exponent, 0.5, 3);
    c->assists.steerRate = clampd(rate, 0, 50);
    c->assists.steerReturnRate = clampd(returnRate, 0, 50);
    c->assists.targetFrontSlipAngle = clampd(targetSlip, 0.03, 0.4);
    c->assists.countersteerAssist = clampd(countersteer, 0, 1);
    c->assists.driftAssist = clampd(drift, 0, 1);
    c->v->assists().setConfig(c->assists);
}

GSP_EXPORT void gsp_set_input(int h, double steer, double throttle, double brake, double handbrake, int shiftUp, int shiftDown) {
    Car* c = car(h);
    if (!c) return;
    c->input.steer = clampd(steer, -1, 1);
    c->input.throttle = clampd(throttle, 0, 1);
    c->input.brake = clampd(brake, 0, 1);
    c->input.handbrake = clampd(handbrake, 0, 1);
    c->input.shiftUp = c->input.shiftUp || shiftUp != 0;
    c->input.shiftDown = c->input.shiftDown || shiftDown != 0;
}

// The chassis state the host integrated: body origin (= CoM), orientation (w, x, y, z), CoM
// velocity and angular velocity, world frame.
GSP_EXPORT void gsp_set_body(int h, double px, double py, double pz, double qw, double qx, double qy, double qz,
                             double vx, double vy, double vz, double wx, double wy, double wz) {
    Car* c = car(h);
    if (!c) return;
    RigidBodyState s;
    s.position = {px, py, pz};
    s.orientation = Quat{qw, qx, qy, qz};
    s.orientation.normalize();
    s.velocity = {vx, vy, vz};
    s.angularVelocity = {wx, wy, wz};
    c->v->setChassisState(s);
}

// Advances car `h` by the host step `dt` in `substeps` vehicle-local steps. The average force
// and torque (about the CoM, world) to apply to the host body over `dt` are then in
// gsp_forces(h); the host applies gravity itself. Returns 0 if the car is faulted.
GSP_EXPORT int gsp_step(int h, double dt, int substeps) {
    Car* c = car(h);
    if (!c) return 0;
    c->v->stepSubstepped(dt, std::clamp(substeps, 1, 16), c->input, g_ground);
    c->input.shiftUp = c->input.shiftDown = false;
    const ChassisForces& f = c->v->chassisForces();
    c->forces = {f.force.x, f.force.y, f.force.z, f.torque.x, f.torque.y, f.torque.z};
    publish(*c);
    return c->v->faulted() ? 0 : 1;
}
// Steps every car with the same dt and substeps (one call per host step).
GSP_EXPORT void gsp_step_all(double dt, int substeps) {
    for (int k = 0; k < kMaxCars; ++k)
        if (g_cars[static_cast<std::size_t>(k)].v) gsp_step(k, dt, substeps);
}
GSP_EXPORT double* gsp_forces(int h) {
    Car* c = car(h);
    return c ? c->forces.data() : nullptr;
}
GSP_EXPORT double* gsp_state(int h) {
    Car* c = car(h);
    return c ? c->out.data() : nullptr;
}
// Clears a numerical fault (after the host has repositioned the car).
GSP_EXPORT void gsp_clear_fault(int h) {
    Car* c = car(h);
    if (!c || !c->v->faulted()) return;
    const RigidBodyState b = c->v->state();
    c->v->setChassisMode(ChassisMode::Internal, false);
    c->v->placeOnGround(b.position.x, b.position.y, 0, g_ground, 0);
    c->v->setChassisMode(ChassisMode::External, true);
}

// Mass properties and body hull for the host's collider: mass, CoM (body), principal inertia
// (roll, pitch, yaw), then the body box (front, rear, half width, floor, roof, belt, roof rear,
// roof front) in the body frame, then the wheels' hardpoints (x, y, z) x 4.
GSP_EXPORT double* gsp_body_info(int h) {
    static std::array<double, 32> o{};
    o.fill(0);
    Car* c = car(h);
    if (!c) return o.data();
    const VehicleDefinition& d = c->v->definition();
    o[0] = d.mass;
    o[1] = d.centerOfMass.x; o[2] = d.centerOfMass.y; o[3] = d.centerOfMass.z;
    o[4] = d.inertia.x; o[5] = d.inertia.y; o[6] = d.inertia.z;
    const CageBox& b = c->v->bodyBox();
    o[7] = b.front; o[8] = b.rear; o[9] = b.halfWidth; o[10] = b.floor; o[11] = b.roof; o[12] = b.belt;
    o[13] = b.roofRear; o[14] = b.roofFront;
    for (std::size_t w = 0; w < 4 && w < d.wheels.size(); ++w) {
        o[15 + 3 * w] = d.wheels[w].suspension.hardpoint.x;
        o[16 + 3 * w] = d.wheels[w].suspension.hardpoint.y;
        o[17 + 3 * w] = d.wheels[w].suspension.hardpoint.z;
    }
    return o.data();
}

// Tyre temperatures (deg C) and wind (m/s, the direction the air moves).
GSP_EXPORT void gsp_set_tyre_temperature(int h, double celsius) {
    if (Car* c = car(h)) c->v->setTyreTemperatures(celsius);
}
GSP_EXPORT void gsp_set_wind(double wx, double wy) {
    for (auto& c : g_cars)
        if (c.v) c.v->setWind({wx, wy, 0});
}
