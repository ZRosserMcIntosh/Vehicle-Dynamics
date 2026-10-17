// OpenRoad Dynamics — WebAssembly bindings for the browser proving ground.
// SPDX-License-Identifier: MIT
//
// A thin C ABI over ord::Vehicle so a web page can drive the real core. The
// proving-ground surface (heights and patches) is defined here once and also
// exported, so the page renders exactly the ground the physics sees.
#include <algorithm>
#include <array>
#include <cstdint>
#include <vector>
#include <cmath>
#include <memory>

#include "ord/ground.hpp"
#include "ord/presets.hpp"
#include "ord/vehicle.hpp"
#include "reference_vehicles.hpp"

#ifdef __EMSCRIPTEN__
#include <emscripten/emscripten.h>
#define ORD_EXPORT extern "C" EMSCRIPTEN_KEEPALIVE
#else
#define ORD_EXPORT extern "C"
#endif

using namespace ord;

namespace {

// ---- Proving ground (ISO 8855: x forward from the start line, y left, z up) ----
// Bump lane runs along y = kBumpLaneY: speed bumps, a rumble strip, a pothole,
// and an 8 cm kerb on its left edge. Everything else is flat asphalt.
constexpr Real kBumpLaneY = -110.0;

Real bumpLaneHeight(Real x, Real y) {
    Real h = 0;
    // Three 8 cm cosine speed bumps, 3.6 m long.
    for (Real c : {60.0, 95.0, 130.0}) {
        const Real d = std::abs(x - c);
        if (d < 1.8) h += 0.04 * (1 + std::cos(kPi * d / 1.8));
    }
    // Rumble strip: 2 cm sawtooth ridges every 0.5 m.
    if (x > 160 && x < 200) {
        const Real p = std::fmod(x - 160, 0.5) / 0.5;
        h += 0.02 * (p < 0.5 ? 2 * p : 2 * (1 - p));
    }
    // 6 cm deep pothole, 1.2 m across, under the left wheel track.
    {
        const Real dx = x - 235, dy = y - (kBumpLaneY + 0.8);
        const Real r = std::sqrt(dx * dx + dy * dy);
        if (r < 0.6) h -= 0.06 * 0.5 * (1 + std::cos(kPi * r / 0.6));
    }
    // 8 cm kerb along the left edge of the lane (rounded 10 cm shoulder).
    {
        const Real e = y - (kBumpLaneY + 3.0);
        if (e > 0.1 && e < 0.6 && x > 0 && x < 300) h += 0.08;
        else if (e > 0 && e <= 0.1 && x > 0 && x < 300) h += 0.08 * 0.5 * (1 - std::cos(kPi * e / 0.1));
    }
    return h;
}

// ---- Proving ground: the default start, west of the drag strip (x -480 .. -70) ----
// A central lane (y = 0) runs east into the drag strip. Beside it:
//  bump course (y = kPgBumpY): graded bumps 2-12 cm, washboard, alternating one-side bumps,
//    a crest and a dip, a pothole;
//  split-mu lane (y = kPgSplitY): left wheel track wet, right dry;
//  dry skidpad R30 at (-300, 55); wet skidpad R25 inside the wet square x -200..-140, y 25..85.
constexpr Real kPgBumpY = -30.0;
constexpr Real kPgSplitY = -55.0;
constexpr Real kPgX0 = -440.0, kPgX1 = -110.0;

Real cosBump(Real x, Real c, Real len, Real h) {
    const Real d = std::abs(x - c);
    return d < 0.5 * len ? h * 0.5 * (1 + std::cos(2 * kPi * d / len)) : 0.0;
}

Real provingBumpHeight(Real x, Real y) {
    const Real dy = y - kPgBumpY;
    Real h = 0;
    // graded cosine bumps across the lane: 2, 4, 6, 8 cm, a 10 cm speed hump, a sharp 12 cm ridge
    h += cosBump(x, -420, 1.0, 0.02) + cosBump(x, -395, 1.2, 0.04) + cosBump(x, -370, 1.4, 0.06);
    h += cosBump(x, -345, 1.6, 0.08) + cosBump(x, -320, 3.6, 0.10) + cosBump(x, -290, 0.6, 0.12);
    // washboard: 1.5 cm ridges every 0.6 m for 30 m
    if (x > -265 && x < -235) h += 0.015 * 0.5 * (1 - std::cos(2 * kPi * (x + 265) / 0.6));
    // alternating one-side 8 cm bumps (left, right, left, right): roll and warp
    for (int k = 0; k < 4; ++k) {
        const bool left = (k % 2) == 0;
        if ((left ? dy > 0 : dy < 0) && std::abs(dy) < 3.5) h += cosBump(x, -212 + 9.0 * k, 2.0, 0.08);
    }
    // crest (30 cm over 12 m) then a dip (20 cm over 10 m)
    if (x > -175 && x < -163) h += 0.30 * 0.5 * (1 - std::cos(2 * kPi * (x + 175) / 12));
    if (x > -155 && x < -145) h -= 0.20 * 0.5 * (1 - std::cos(2 * kPi * (x + 155) / 10));
    // 7 cm pothole under the left wheel track
    {
        const Real px = x + 128, py = y - (kPgBumpY + 0.8), r = std::sqrt(px * px + py * py);
        if (r < 0.7) h -= 0.07 * 0.5 * (1 + std::cos(kPi * r / 0.7));
    }
    return h;
}

Real provingGroundHeight(Real x, Real y) {
    if (y > kBumpLaneY - 4 && y < kBumpLaneY + 4 && x > 40 && x < 300) return bumpLaneHeight(x, y);
    if (y > kPgBumpY - 4 && y < kPgBumpY + 4 && x > kPgX0 && x < kPgX1) return provingBumpHeight(x, y);
    return 0.0;
}

SurfaceProperties wetSurface() {
    SurfaceProperties s;
    s.frictionScale = 0.6;
    s.wetSpeedFalloff = 0.008;
    return s;
}

SurfaceProperties bumpSurface() {
    SurfaceProperties s;
    s.roughness = 1.0;
    return s;
}

// ---- City world: a measured heightfield (top drivable surface) plus static obstacles ----
// The page uploads both (it gets them from the tile export): heights in cm on a regular grid,
// a surface class per cell, wall segments (building outlines, tile edge) and circles (tree
// trunks, poles, pillars). Collisions are resolved here, after every physics step, as a
// 2D oriented-box-vs-obstacle push-out plus a restitution/friction impulse on the chassis.
struct HeightfieldGround final : GroundQuery {
    int nx = 0, ny = 0;
    double x0 = 0, y0 = 0, cell = 1;
    std::vector<int16_t> h;
    std::vector<uint8_t> cls;
    SurfaceProperties surf[4];
    HeightfieldGround() {
        surf[2].roughness = 0.3;                                  // paving, curbs, plazas
        surf[3].frictionScale = 0.65; surf[3].rollingResistanceScale = 4.0; surf[3].roughness = 0.6;  // soil, lawn, park
    }
    double cellH(int i, int j) const {
        i = std::clamp(i, 0, nx - 1); j = std::clamp(j, 0, ny - 1);
        return h[static_cast<std::size_t>(j) * nx + i] * 0.01;
    }
    double heightAt(double x, double y) const {
        if (nx < 2) return 0;
        const double fx = (x - x0) / cell, fy = (y - y0) / cell;
        const int i = static_cast<int>(std::floor(fx)), j = static_cast<int>(std::floor(fy));
        const double tx = fx - i, ty = fy - j;
        const double a = cellH(i, j), b = cellH(i + 1, j), c = cellH(i, j + 1), d = cellH(i + 1, j + 1);
        return (a * (1 - tx) + b * tx) * (1 - ty) + (c * (1 - tx) + d * tx) * ty;
    }
    int classAt(double x, double y) const {
        if (nx < 2) return 1;
        const int i = std::clamp(static_cast<int>(std::lround((x - x0) / cell)), 0, nx - 1);
        const int j = std::clamp(static_cast<int>(std::lround((y - y0) / cell)), 0, ny - 1);
        return cls[static_cast<std::size_t>(j) * nx + i] & 3;
    }
    GroundHit raycast(const Vec3& origin, const Vec3& dir, Real maxDistance) const override {
        GroundHit r;
        if (dir.z >= -1e-6) return r;
        Real t = (heightAt(origin.x, origin.y) - origin.z) / dir.z;
        for (int k = 0; k < 6; ++k) { const Vec3 p = origin + dir * t; t = (heightAt(p.x, p.y) - origin.z) / dir.z; }
        if (t < 0 || t > maxDistance) return r;
        r.hit = true; r.distance = t; r.point = origin + dir * t;
        const double e = 0.05;
        const double dx = (heightAt(r.point.x + e, r.point.y) - heightAt(r.point.x - e, r.point.y)) / (2 * e);
        const double dy = (heightAt(r.point.x, r.point.y + e) - heightAt(r.point.x, r.point.y - e)) / (2 * e);
        r.normal = normalized(Vec3{-dx, -dy, 1});
        const int c = classAt(r.point.x, r.point.y);
        r.surface = surf[c]; r.surfaceId = 10 + c;
        return r;
    }
};

struct Colliders {
    struct Seg { double ax, ay, bx, by, z0, z1; };
    struct Circ { double x, y, r, z0, z1; };
    std::vector<Seg> segs;
    std::vector<Circ> circs;
    std::vector<double> upload;
    double gx0 = -300, gy0 = -300, gcell = 8; int gn = 75;
    std::vector<std::vector<int>> grid;   // >= 0 segment, < 0 circle (~index)
    void build() {
        grid.assign(static_cast<std::size_t>(gn) * gn, {});
        auto addBox = [&](double ax, double ay, double bx, double by, int id) {
            const int i0 = std::clamp(static_cast<int>((std::min(ax, bx) - gx0) / gcell), 0, gn - 1), i1 = std::clamp(static_cast<int>((std::max(ax, bx) - gx0) / gcell), 0, gn - 1);
            const int j0 = std::clamp(static_cast<int>((std::min(ay, by) - gy0) / gcell), 0, gn - 1), j1 = std::clamp(static_cast<int>((std::max(ay, by) - gy0) / gcell), 0, gn - 1);
            for (int j = j0; j <= j1; ++j) for (int i = i0; i <= i1; ++i) grid[static_cast<std::size_t>(j) * gn + i].push_back(id);
        };
        for (std::size_t k = 0; k < segs.size(); ++k) addBox(segs[k].ax, segs[k].ay, segs[k].bx, segs[k].by, static_cast<int>(k));
        for (std::size_t k = 0; k < circs.size(); ++k) addBox(circs[k].x - circs[k].r, circs[k].y - circs[k].r, circs[k].x + circs[k].r, circs[k].y + circs[k].r, ~static_cast<int>(k));
    }
};

struct Sim {
    std::unique_ptr<GroundQuery> ground;
    TestTrackGround* track = nullptr;
    HeightfieldGround* city = nullptr;
    int world = 0;
    Colliders col;
    double impact = 0;  // largest collision speed change since the last publish (m/s)
    std::unique_ptr<Vehicle> vehicle;
    DriverInput input;
    AssistConfig assists;
    Real accumulator = 0;
    int carIndex = 0;
};

Sim g;
constexpr Real kStep = 1.0 / 360.0;
// Frame state exported to JavaScript (see web/README.md for the layout).
constexpr int kHeader = 36, kPerWheel = 16;
std::array<double, kHeader + kPerWheel * kMaxWheels> g_out{};

VehicleDefinition carByIndex(int i) {
    switch (i) {
        case 1: return vehicles::referenceHatchFwd();
        case 2: return vehicles::referenceSaloonAwd();
        case 3: return vehicles::referenceHypercarAwd();
        case 4: return vehicles::referenceRaceDownforce();
        case 5: return vehicles::rosserStellaV8();
        default: return vehicles::referenceSportsRwd();
    }
}

void ensureGround() {
    if (g.ground) return;
    if (g.world == 1) {
        auto c = std::make_unique<HeightfieldGround>();
        g.city = c.get(); g.track = nullptr;
        g.ground = std::move(c);
        return;
    }
    auto t = std::make_unique<TestTrackGround>(provingGroundHeight);
    t->addPatch({250, 450, 25, 75, wetSurface(), 2});
    t->addPatch({40, 300, kBumpLaneY - 4, kBumpLaneY + 4, bumpSurface(), 3});
    // proving ground
    t->addPatch({kPgX0, kPgX1, kPgBumpY - 4, kPgBumpY + 4, bumpSurface(), 3});
    t->addPatch({-200, -140, 25, 85, wetSurface(), 2});               // wet skidpad square
    t->addPatch({kPgX0, kPgX1, kPgSplitY, kPgSplitY + 5, wetSurface(), 2});  // split-mu: left half wet
    g.track = t.get(); g.city = nullptr;
    g.ground = std::move(t);
}

double groundHeight(double x, double y) { return g.city ? g.city->heightAt(x, y) : provingGroundHeight(x, y); }

// Push the chassis out of walls / trunks and apply a contact impulse (restitution + friction).
void resolveCollisions() {
    if (!g.city || (g.col.segs.empty() && g.col.circs.empty())) return;
    Vehicle& v = *g.vehicle;
    const VehicleDefinition& d = v.definition();
    RigidBodyState b = v.state();
    // body footprint from the wheel layout (+ overhangs)
    double xf = -1e9, xr = 1e9, hw = 0;
    for (const auto& w : d.wheels) { xf = std::max(xf, w.suspension.hardpoint.x); xr = std::min(xr, w.suspension.hardpoint.x); hw = std::max(hw, std::abs(w.suspension.hardpoint.y)); }
    xf += 1.0; xr -= 1.15; hw += 0.16;
    const double ex = 0.5 * (xf - xr), ey = hw, cxl = 0.5 * (xf + xr);
    const double m = d.mass, izz = std::max(d.inertia.z, 1.0);
    bool changed = false;
    for (int iter = 0; iter < 4; ++iter) {
        Vec3 u = b.orientation.rotate({1, 0, 0}); u.z = 0;
        const double ul = std::sqrt(u.x * u.x + u.y * u.y); if (ul < 1e-6) return;
        u.x /= ul; u.y /= ul;
        const double vx = -u.y, vy = u.x;
        const double cx = b.position.x + u.x * cxl, cy = b.position.y + u.y * cxl;
        const double zc = b.position.z;
        // deepest contact this iteration
        double best = 0, nx = 0, ny = 0, px = 0, py = 0;
        const int gi = std::clamp(static_cast<int>((cx - g.col.gx0) / g.col.gcell), 0, g.col.gn - 1);
        const int gj = std::clamp(static_cast<int>((cy - g.col.gy0) / g.col.gcell), 0, g.col.gn - 1);
        auto obbProj = [&](double ax, double ay) { return ex * std::abs(u.x * ax + u.y * ay) + ey * std::abs(vx * ax + vy * ay); };
        for (int dj = -1; dj <= 1; ++dj) for (int di = -1; di <= 1; ++di) {
            const int i = gi + di, j = gj + dj;
            if (i < 0 || j < 0 || i >= g.col.gn || j >= g.col.gn) continue;
            for (int id : g.col.grid[static_cast<std::size_t>(j) * g.col.gn + i]) {
                if (id >= 0) {
                    const auto& s = g.col.segs[static_cast<std::size_t>(id)];
                    if (zc + 1.0 < s.z0 || zc - 0.5 > s.z1) continue;
                    // SAT: axes u, v, segment normal
                    double sx = s.bx - s.ax, sy = s.by - s.ay; const double sl = std::sqrt(sx * sx + sy * sy); if (sl < 1e-4) continue;
                    sx /= sl; sy /= sl;
                    const double axes[3][2] = {{u.x, u.y}, {vx, vy}, {-sy, sx}};
                    double minO = 1e9, mx = 0, my = 0;
                    bool sep = false;
                    for (auto& A : axes) {
                        const double c0 = A[0] * cx + A[1] * cy, r0 = obbProj(A[0], A[1]);
                        const double p1 = A[0] * s.ax + A[1] * s.ay, p2 = A[0] * s.bx + A[1] * s.by;
                        const double lo = std::min(p1, p2), hi = std::max(p1, p2);
                        const double o1 = (c0 + r0) - lo, o2 = hi - (c0 - r0);
                        if (o1 <= 0 || o2 <= 0) { sep = true; break; }
                        const double o = std::min(o1, o2);
                        if (o < minO) { minO = o; const double sg = (o1 < o2) ? -1 : 1; mx = A[0] * sg; my = A[1] * sg; }
                    }
                    if (sep || minO <= best) continue;
                    best = minO; nx = mx; ny = my;
                    // contact point: segment point closest to the box centre
                    const double t = std::clamp((cx - s.ax) * sx + (cy - s.ay) * sy, 0.0, sl);
                    px = s.ax + sx * t; py = s.ay + sy * t;
                } else {
                    const auto& c = g.col.circs[static_cast<std::size_t>(~id)];
                    if (zc + 1.0 < c.z0 || zc - 0.5 > c.z1) continue;
                    const double lx = (c.x - cx) * u.x + (c.y - cy) * u.y, ly = (c.x - cx) * vx + (c.y - cy) * vy;
                    const double qx = std::clamp(lx, -ex, ex), qy = std::clamp(ly, -ey, ey);
                    double dx = lx - qx, dy = ly - qy, dist = std::sqrt(dx * dx + dy * dy), pen, lnx, lny;
                    if (dist > 1e-6) { if (dist >= c.r) continue; pen = c.r - dist; lnx = -dx / dist; lny = -dy / dist; }
                    else {  // centre inside the box: push out along the shallow axis
                        const double ox = ex - std::abs(lx), oy = ey - std::abs(ly);
                        if (ox < oy) { pen = ox + c.r; lnx = lx > 0 ? -1 : 1; lny = 0; } else { pen = oy + c.r; lnx = 0; lny = ly > 0 ? -1 : 1; }
                    }
                    if (pen <= best) continue;
                    best = pen; nx = lnx * u.x + lny * vx; ny = lnx * u.y + lny * vy;
                    px = cx + qx * u.x + qy * vx; py = cy + qx * u.y + qy * vy;
                }
            }
        }
        if (best <= 0) break;
        // positional correction (slightly more than the overlap)
        b.position.x += nx * (best + 0.002); b.position.y += ny * (best + 0.002);
        // impulse at the contact point
        const Vec3 com = b.position + b.orientation.rotate(d.centerOfMass);
        const double rx = px - com.x, ry = py - com.y;
        const double vcx = b.velocity.x - b.angularVelocity.z * ry, vcy = b.velocity.y + b.angularVelocity.z * rx;
        const double vn = vcx * nx + vcy * ny;
        if (vn < 0) {
            const double rn = rx * ny - ry * nx;
            const double e = vn < -6 ? 0.15 : 0.05;
            const double jn = -(1 + e) * vn / (1 / m + rn * rn / izz);
            const double tx = -ny, ty = nx, vt = vcx * tx + vcy * ty, rt = rx * ty - ry * tx;
            double jt = -vt / (1 / m + rt * rt / izz);
            const double mu = 0.35; jt = std::clamp(jt, -mu * jn, mu * jn);
            b.velocity.x += (jn * nx + jt * tx) / m; b.velocity.y += (jn * ny + jt * ty) / m;
            b.angularVelocity.z += (rn * jn + rt * jt) / izz;
            g.impact = std::max(g.impact, -vn);
        }
        changed = true;
    }
    if (changed) v.setChassisState(b);
}

void publish() {
    const Vehicle& v = *g.vehicle;
    const RigidBodyState& b = v.state();
    const TelemetryFrame& t = v.telemetry();
    const VehicleDefinition& d = v.definition();
    double* o = g_out.data();
    o[0] = b.position.x; o[1] = b.position.y; o[2] = b.position.z;
    o[3] = b.orientation.w; o[4] = b.orientation.x; o[5] = b.orientation.y; o[6] = b.orientation.z;
    o[7] = t.speed; o[8] = t.lateralVelocity; o[9] = t.yawRate; o[10] = t.sideslip;
    o[11] = t.accelBody.x; o[12] = t.accelBody.y; o[13] = t.accelBody.z;
    o[14] = t.drivetrain.engineRpm; o[15] = t.gear; o[16] = t.steerAngle; o[17] = t.throttleOut;
    o[18] = t.assists.tcsThrottleCut; o[19] = t.assists.countersteerAngle; o[20] = t.assists.escYawError;
    o[21] = t.time; o[22] = static_cast<double>(t.wheelCount); o[23] = v.faulted() ? 1 : 0;
    o[24] = t.assists.shifting ? 1 : 0; o[25] = d.wheels.empty() ? 0 : d.wheels[0].tire.radius;
    o[26] = t.clutchEngagement; o[27] = t.steerInput; o[28] = t.throttleInput; o[29] = t.brakeInput;
    o[30] = static_cast<double>(t.groundQueries); o[31] = g.impact; g.impact = 0;
    o[32] = t.assists.driftActive ? 1 : 0; o[33] = t.assists.driftTargetSideslip;
    o[34] = t.assists.driftThrottleCut; o[35] = t.assists.driftSteerAngle;
    for (std::size_t w = 0; w < t.wheelCount; ++w) {
        const WheelTelemetry& wt = t.wheels[w];
        const WheelDef& wd = d.wheels[w];
        double* p = o + kHeader + kPerWheel * static_cast<int>(w);
        p[0] = wd.suspension.hardpoint.x; p[1] = wd.suspension.hardpoint.y;
        p[2] = wd.suspension.hardpoint.z - (wd.suspension.travel - wt.compression);  // wheel centre z, body frame
        p[3] = wd.tire.radius; p[4] = wt.steerAngle; p[5] = wt.omega;
        p[6] = wt.load; p[7] = wt.slipRatio; p[8] = wt.slipAngle; p[9] = wt.fx; p[10] = wt.fy;
        p[11] = wt.contact ? 1 : 0; p[12] = wt.brakeLocked ? 1 : 0;
        p[13] = t.assists.absActive[w] ? 1 : 0; p[14] = t.assists.escBrakeTorque[w]; p[15] = wt.surfaceId;
    }
}

}  // namespace

// Creates (or replaces) the car and places it at (x, y) heading `yaw`. Returns 1 on success.
ORD_EXPORT int ord_create(int carIndex, double x, double y, double yaw) {
    ensureGround();
    g.carIndex = carIndex;
    g.vehicle = std::make_unique<Vehicle>(carByIndex(carIndex), g.assists);
    if (!g.vehicle->isValid()) return 0;
    g.vehicle->reset({x, y, groundHeight(x, y) + 1.0}, yaw, *g.ground);
    g.accumulator = 0;
    g.input = {};
    publish();
    return 1;
}

ORD_EXPORT void ord_reset(double x, double y, double yaw) {
    if (!g.vehicle) return;
    g.vehicle->reset({x, y, groundHeight(x, y) + 1.0}, yaw, *g.ground);
    g.accumulator = 0;
    publish();
}

// preset: 0 FUN, 1 SPORT, 2 SIMULATION, 3 DRIFT. Same physics; only the assists change.
ORD_EXPORT void ord_set_preset(int preset) {
    g.assists = makePreset(preset == 0 ? DrivingPreset::Fun
                           : preset == 1 ? DrivingPreset::Sport
                           : preset == 3 ? DrivingPreset::Drift
                                         : DrivingPreset::Simulation);
    if (g.vehicle) g.vehicle->assists().setConfig(g.assists);
}

// Drift assist override, 0..1 (an aid with no real-car equivalent; see assists.hpp).
ORD_EXPORT void ord_set_drift_assist(double amount) {
    g.assists.driftAssist = amount < 0 ? 0 : amount > 1 ? 1 : amount;
    if (g.vehicle) g.vehicle->assists().setConfig(g.assists);
}

// Individual overrides on top of the preset. level: 0 off, 1 sport, 2 full.
ORD_EXPORT void ord_set_aids(int abs, int tcsLevel, int escLevel, int automatic) {
    auto lvl = [](int l) { return l <= 0 ? InterventionLevel::Off : l == 1 ? InterventionLevel::Sport : InterventionLevel::Full; };
    g.assists.abs = abs != 0;
    g.assists.tractionControl = lvl(tcsLevel);
    g.assists.stabilityControl = lvl(escLevel);
    g.assists.shiftMode = automatic ? ShiftMode::Automatic : ShiftMode::Manual;
    if (g.vehicle) g.vehicle->assists().setConfig(g.assists);
}

ORD_EXPORT void ord_set_input(double steer, double throttle, double brake, double handbrake, int shiftUp, int shiftDown) {
    g.input.steer = steer;
    g.input.throttle = throttle;
    g.input.brake = brake;
    g.input.handbrake = handbrake;
    // Edge-triggered: latched until the next fixed step consumes them.
    g.input.shiftUp = g.input.shiftUp || shiftUp != 0;
    g.input.shiftDown = g.input.shiftDown || shiftDown != 0;
}

// Advances real time `dt` (s) in fixed 1/360 s steps. Returns the number of steps taken.
ORD_EXPORT int ord_advance(double dt) {
    if (!g.vehicle || g.vehicle->faulted()) return 0;
    g.accumulator += std::min(dt, 0.1);  // never try to catch up more than 100 ms
    int n = 0;
    while (g.accumulator >= kStep) {
        g.vehicle->step(kStep, g.input, *g.ground);
        resolveCollisions();
        g.input.shiftUp = g.input.shiftDown = false;
        g.accumulator -= kStep;
        ++n;
    }
    publish();
    return n;
}

ORD_EXPORT double* ord_state() { return g_out.data(); }
ORD_EXPORT int ord_state_header() { return kHeader; }
ORD_EXPORT int ord_state_per_wheel() { return kPerWheel; }
ORD_EXPORT double ord_ground_height(double x, double y) { return groundHeight(x, y); }

// ---- world selection and city data upload ----
// 0 = test track (default), 1 = city heightfield. Call before ord_create; recreates the ground.
ORD_EXPORT void ord_set_world(int w) {
    g.world = w == 1 ? 1 : 0;
    g.vehicle.reset(); g.ground.reset(); g.city = nullptr; g.track = nullptr;
    ensureGround();
}
// Allocates the city heightfield; returns the int16 buffer (cm, rows = y) to fill from JS.
ORD_EXPORT int16_t* ord_city_heights(int nx, int ny, double x0, double y0, double cell) {
    if (!g.city) return nullptr;
    g.city->nx = nx; g.city->ny = ny; g.city->x0 = x0; g.city->y0 = y0; g.city->cell = cell;
    g.city->h.assign(static_cast<std::size_t>(nx) * ny, 0);
    g.city->cls.assign(static_cast<std::size_t>(nx) * ny, 1);
    return g.city->h.data();
}
ORD_EXPORT uint8_t* ord_city_classes() { return g.city ? g.city->cls.data() : nullptr; }
// Collider upload: a double buffer of nseg * 6 (ax, ay, bx, by, z0, z1) + ncirc * 5 (x, y, r, z0, z1).
ORD_EXPORT double* ord_city_colliders(int nseg, int ncirc) {
    g.col.segs.assign(static_cast<std::size_t>(nseg), {});
    g.col.circs.assign(static_cast<std::size_t>(ncirc), {});
    g.col.upload.assign(static_cast<std::size_t>(nseg) * 6 + static_cast<std::size_t>(ncirc) * 5, 0);
    return g.col.upload.data();
}
ORD_EXPORT void ord_city_commit(double gx0, double gy0, double gcell, int gn) {
    const double* u = g.col.upload.data();
    for (auto& s : g.col.segs) { s = {u[0], u[1], u[2], u[3], u[4], u[5]}; u += 6; }
    for (auto& c : g.col.circs) { c = {u[0], u[1], u[2], u[3], u[4]}; u += 5; }
    g.col.gx0 = gx0; g.col.gy0 = gy0; g.col.gcell = gcell; g.col.gn = gn;
    g.col.build();
    g.col.upload.clear(); g.col.upload.shrink_to_fit();
}
ORD_EXPORT double ord_bump_lane_y() { return kBumpLaneY; }
ORD_EXPORT double ord_proving_bump_y() { return kPgBumpY; }
ORD_EXPORT double ord_proving_split_y() { return kPgSplitY; }
