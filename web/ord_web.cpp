// OpenRoad Dynamics — WebAssembly bindings for the browser proving ground.
// SPDX-License-Identifier: MIT
//
// A thin C ABI over ord::Vehicle so a web page can drive the real core. The
// proving-ground surface (heights and patches) is defined here once and also
// exported, so the page renders exactly the ground the physics sees.
#include <algorithm>
#include <array>
#include <string_view>
#include <cstdint>
#include <vector>
#include <cmath>
#include <memory>
#include <optional>
#include <string>

#include "ord/crash.hpp"
#include "ord/ground.hpp"
#include "ord/presets.hpp"
#include "ord/setup.hpp"
#include "ord/two_wheeler.hpp"
#include "ord/vehicle.hpp"
#include "production_vehicles.hpp"
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
// Rollover area south of the split-mu lane: a 25 cm trip kerb along y = kRollKerbY (slide sideways
// into it), and a one-sided ramp under the left wheels of the lane y = kRollRampLaneY that lifts
// them 0.9 m over 10 m and ends in a drop (a corkscrew).
constexpr Real kRollKerbY = -120.0, kRollKerbX0 = -420.0, kRollKerbX1 = -300.0;
constexpr Real kRollRampLaneY = -87.0, kRollRampX0 = -265.0, kRollRampX1 = -255.0, kRollRampH = 0.9;
// Crash-test lane north of the skidpads: run-up from x = -440, barrier at x = -150.
constexpr Real kCrashLaneY = 125.0, kCrashBarrierX = -150.0, kCrashPoleX = -230.0;
// Smash lane north of it: three run-up lanes, each ending in a breakable wall at the barrier's x,
// built from three 2.5 m sections so a car punches a hole and leaves the rest standing.
// Fracture energies per section are representative estimates, not test data: a 10 mm storefront
// pane (~1.5 kJ, gives at a walking pace), an unreinforced hollow-block wall 2 m high (~25 kJ,
// a car breaches it from ~20 km/h), a double-wythe brick wall 2 m high (~120 kJ, ~45 km/h).
constexpr Real kSmashLaneY0 = kCrashLaneY + 28.0, kSmashLaneSpacing = 10.0;
constexpr double kBreakEnergy[3] = {1500.0, 25000.0, 120000.0};
Real smashLaneY(int k) { return kSmashLaneY0 + kSmashLaneSpacing * k; }

// ---- Test areas south of the rollover area (flat run-ups from x = -470) ----
// Moose test (ISO 3888-2 double lane change): the cones are the page's; the lane is flat asphalt.
constexpr Real kMooseY = -150.0;
// Aquaplaning lane: an 8 mm water pool across the lane, then a 12 mm puddle under the left
// wheels only (the pull toward the water). Surfaces lane: Sao Paulo cobblestones (paralelepipedo)
// and painted stripes, both much slicker than asphalt when wet.
constexpr Real kAquaY = -180.0, kAquaPoolX0 = -330.0, kAquaPoolX1 = -280.0, kAquaPuddleX0 = -220.0, kAquaPuddleX1 = -200.0;
constexpr Real kSurfY = -210.0, kCobbleX0 = -430.0, kCobbleX1 = -330.0, kPaintX0 = -300.0, kPaintX1 = -260.0;

// ---- Stunt park, north of the smash lanes ----
// Two kicker ramps and their landing ramps (a 1.5 m kicker with an 18 m gap, a 3 m kicker with
// a 28 m gap), a half-pipe along x, and a loop. The loop is not a height field: it is a ribbon
// on the inside of a general cylinder (axis along y) whose profile in the x-z plane widens at
// the bottom and tightens at the top, like real stunt loops: the radius of curvature runs from
// kLoopRb at the bottom to kLoopRt at the top, rho(theta) = Rb + (Rt - Rb)(1 - cos theta)/2
// with theta the slope angle. A circular loop needs at least 6 g at its bottom to make it over
// the top (v^2 = 5 g R there); this one needs ~4 g. The ribbon shifts kLoopShift sideways over
// its length so the way in and the way out do not meet: a car that drives in straight at
// kLoopHeading follows it (on a cylinder, unrolled flat, that path is a straight line).
constexpr Real kJumpY[2] = {212.0, 226.0};
constexpr Real kKickX0[2] = {-340.0, -314.0}, kKickLen[2] = {10.0, 14.0}, kKickH[2] = {1.5, 3.0};
constexpr Real kLandX0[2] = {-312.0, -272.0}, kLandLen[2] = {40.0, 60.0};
constexpr Real kPipeY = 262.0, kPipeX0 = -440.0, kPipeX1 = -240.0, kPipeFloor = 4.0, kPipeR = 9.0, kPipeDeck = 3.0;
constexpr Real kPipeMaxAngle = 50.0 * kPi / 180.0;  // walls steepen to 50 degrees (a stalled car tips past ~55), then a deck
constexpr Real kLoopX = -250.0, kLoopY = 300.0, kLoopRb = 22.0, kLoopRt = 9.0, kLoopShift = 9.0, kLoopHalfWidth = 2.0;
constexpr Real kLoopClothoid = 30.0;  // m of entry (and exit) over which the curvature ramps up from 0
constexpr Real kLoopR = kLoopRb;      // reported to the page (bottom radius)

// The loop's profile, tabulated once by integrating its curvature along the arc: a clothoid
// lead-in (curvature rising linearly to 1/Rb, so the load comes on over ~1 s instead of as a
// step that would bounce the car off its bump stops), the loop proper with
// rho(theta) = Rb + (Rt - Rb)(1 - cos theta)/2, and a mirror-image lead-out.
// Samples: x (relative to kLoopX), z, slope angle, arc length.
struct LoopProfile {
    std::vector<double> p;
    int n = 0;          // samples
    double length = 0;  // total arc length
    double xMin = 0, xMax = 0, zMax = 0;
    static constexpr double kCell = 1.0;
    int nx = 0, nz = 0;
    std::vector<std::vector<int>> grid;  // segments per 1 m cell
    LoopProfile() {
        const double kb = 1 / kLoopRb, thc = 0.5 * kb * kLoopClothoid;  // angle turned by a clothoid
        auto rho = [](double th) { return kLoopRb + (kLoopRt - kLoopRb) * 0.5 * (1 - std::cos(th)); };
        double x = 0, z = 0, th = 0, sArc = 0;
        constexpr double ds = 0.05;
        auto push = [&] { p.insert(p.end(), {x, z, th, sArc}); xMin = std::min(xMin, x); xMax = std::max(xMax, x); zMax = std::max(zMax, z); };
        auto advance = [&](double k) {
            const double tm = th + 0.5 * k * ds;
            x += ds * std::cos(tm); z += ds * std::sin(tm); th += k * ds; sArc += ds;
            push();
        };
        push();
        while (sArc < kLoopClothoid) advance(kb * sArc / kLoopClothoid);
        // loop proper: rho runs over the angle mapped onto 0..2 pi, so it is Rb where the clothoids
        // hand over and Rt at the top
        const double span = 2 * kPi - 2 * thc;
        while (th < 2 * kPi - thc) advance(1 / rho(2 * kPi * (th - thc) / span));
        const double kEnd = 1 / kLoopRb, s0 = sArc;
        while (sArc - s0 < kLoopClothoid) advance(kEnd * (1 - (sArc - s0) / kLoopClothoid));
        n = static_cast<int>(p.size() / 4);
        length = sArc;
        nx = static_cast<int>((xMax - xMin) / kCell) + 3; nz = static_cast<int>(zMax / kCell) + 3;
        grid.assign(static_cast<std::size_t>(nx) * nz, {});
        for (int i = 0; i + 1 < n; ++i) {
            const double x0 = std::min(p[4 * i], p[4 * i + 4]), x1 = std::max(p[4 * i], p[4 * i + 4]);
            const double z0 = std::min(p[4 * i + 1], p[4 * i + 5]), z1 = std::max(p[4 * i + 1], p[4 * i + 5]);
            for (int j = cz(z0); j <= cz(z1); ++j)
                for (int k = cx(x0); k <= cx(x1); ++k) grid[static_cast<std::size_t>(j) * nx + k].push_back(i);
        }
    }
    int cx(double x) const { return std::clamp(static_cast<int>((x - xMin) / kCell) + 1, 0, nx - 1); }
    int cz(double z) const { return std::clamp(static_cast<int>(z / kCell) + 1, 0, nz - 1); }
};
const LoopProfile& loopProfile() { static const LoopProfile lp; return lp; }
Real loopHeading() { return std::atan(kLoopShift / loopProfile().length); }
constexpr Real kStuntX0 = -470.0, kStuntX1 = -150.0, kStuntY0 = 202.0, kStuntY1 = 340.0;

Real stuntHeight(Real x, Real y) {
    for (int k = 0; k < 2; ++k) {
        if (std::abs(y - kJumpY[k]) > 2.5) continue;
        // kicker: a curved run-in over its first half, then straight to the lip, so the car leaves
        // without the pitch rate a curve at the lip would give it (that flips cars in the air)
        const Real u = x - kKickX0[k], a = 0.5 * kKickLen[k], slope = kKickH[k] / (kKickLen[k] - 0.5 * a);
        if (u >= 0 && u < a) return slope * u * u / (2 * a);
        if (u >= a && u < kKickLen[k]) return slope * (0.5 * a + (u - a));
        const Real s = (x - kLandX0[k]) / kLandLen[k];
        if (s >= 0 && s < 1) return kKickH[k] * 0.5 * (1 + std::cos(kPi * s));  // landing: eases out
    }
    if (x > kPipeX0 && x < kPipeX1) {
        const Real u = std::abs(y - kPipeY) - kPipeFloor;
        const Real uMax = kPipeR * std::sin(kPipeMaxAngle);
        if (u > 0 && u <= uMax) return kPipeR - std::sqrt(kPipeR * kPipeR - u * u);
        if (u > uMax && u < uMax + kPipeDeck) return kPipeR - std::sqrt(kPipeR * kPipeR - uMax * uMax);
    }
    return 0.0;
}

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

Real rolloverHeight(Real x, Real y) {
    // trip kerb: 60 cm wide, 25 cm high, a 5 cm rounded nose on the approach side
    if (x > kRollKerbX0 && x < kRollKerbX1 && y < kRollKerbY && y > kRollKerbY - 0.6) {
        const Real e = kRollKerbY - y;
        return e < 0.05 ? 0.25 * std::sin(0.5 * kPi * e / 0.05) : 0.25;
    }
    // corkscrew ramp: the strip from 0.4 m left of the lane centre to 4 m
    if (x > kRollRampX0 && x < kRollRampX1 && y > kRollRampLaneY + 0.4 && y < kRollRampLaneY + 4.0)
        return kRollRampH * (x - kRollRampX0) / (kRollRampX1 - kRollRampX0);
    return 0.0;
}

Real provingGroundHeight(Real x, Real y) {
    if (y < kRollKerbY + 0.1 && y > kRollKerbY - 1 && x > kRollKerbX0 && x < kRollKerbX1) return rolloverHeight(x, y);
    if (y > kRollRampLaneY && y < kRollRampLaneY + 5 && x > kRollRampX0 && x < kRollRampX1) return rolloverHeight(x, y);
    if (y > kBumpLaneY - 4 && y < kBumpLaneY + 4 && x > 40 && x < 300) return bumpLaneHeight(x, y);
    if (y > kPgBumpY - 4 && y < kPgBumpY + 4 && x > kPgX0 && x < kPgX1) return provingBumpHeight(x, y);
    if (y > kStuntY0 && y < kStuntY1 && x > kStuntX0 && x < kStuntX1) return stuntHeight(x, y);
    return 0.0;
}

SurfaceProperties cobbleSurface() {
    SurfaceProperties s;
    s.frictionScale = 0.85;  // polished granite setts
    s.roughness = 1.0;
    s.textureAmplitude = 0.012;
    s.textureWavelength = 0.14;
    s.textureVariation = 0.6;
    return s;
}
SurfaceProperties paintSurface() {
    SurfaceProperties s;
    s.frictionScale = 0.9;
    return s;
}
SurfaceProperties waterSurface(Real depth) {
    SurfaceProperties s;
    s.frictionScale = 0.75;
    s.wetness = 1.0;
    s.waterDepth = depth;
    return s;
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

// ---- The ground the demo hands the core: the world's ground, plus the stunt park's loop and a
// robust march for its steep faces, plus the weather. ----
std::uint64_t mixHash(std::uint64_t h) {
    h ^= h >> 33; h *= 0xff51afd7ed558ccdULL; h ^= h >> 33; h *= 0xc4ceb9fe1a85ec53ULL; h ^= h >> 33;
    return h;
}
// Rain puddles: one candidate per 22 m cell, about 30 % of cells, 1.2-3.5 m across, 6-20 mm deep.
struct Puddle { double x, y, r, depth; };
constexpr double kPuddleCell = 22.0;
bool puddleInCell(long i, long j, Puddle& p) {
    const std::uint64_t h = mixHash((static_cast<std::uint64_t>(i) * 0x9E3779B97F4A7C15ULL) ^ static_cast<std::uint64_t>(j) ^ 0x5bd1e995ULL);
    if ((h & 1023) > 300) return false;
    const double u1 = static_cast<double>((h >> 10) & 1023) / 1023, u2 = static_cast<double>((h >> 20) & 1023) / 1023;
    const double u3 = static_cast<double>((h >> 30) & 1023) / 1023, u4 = static_cast<double>((h >> 40) & 1023) / 1023;
    p = {(i + 0.15 + 0.7 * u1) * kPuddleCell, (j + 0.15 + 0.7 * u2) * kPuddleCell, 0.6 + 1.15 * u3, 0.006 + 0.014 * u4};
    return true;
}

class DemoGround final : public GroundQuery {
public:
    explicit DemoGround(std::unique_ptr<GroundQuery> inner, bool track) : inner_(std::move(inner)), track_(track) {}
    double rain = 0;  // 0 dry .. 1 downpour
    // Where puddles may form (the city: road classes only).
    std::function<bool(double, double)> puddleAllowed;

    GroundHit raycast(const Vec3& o, const Vec3& d, Real maxDistance) const override {
        GroundHit h;
        const bool stunt = track_ && o.x > kStuntX0 - 5 && o.x < kStuntX1 + 5 && o.y > kStuntY0 - 5 && o.y < kStuntY1 + 5;
        if (stunt) {
            h = march(o, d, maxDistance);
            const GroundHit l = loop(o, d, maxDistance);
            if (l.hit && (!h.hit || l.distance < h.distance)) h = l;
        } else {
            h = inner_->raycast(o, d, maxDistance);
        }
        if (h.hit) weather(h);
        return h;
    }
    // Puddle depth at a point (0 = none), rain included.
    double puddleDepth(double x, double y) const {
        if (rain <= 0) return 0;
        const long i0 = static_cast<long>(std::floor(x / kPuddleCell)), j0 = static_cast<long>(std::floor(y / kPuddleCell));
        Puddle p;
        if (!puddleInCell(i0, j0, p)) return 0;
        const double dx = (x - p.x) / p.r, dy = (y - p.y) / (0.6 * p.r), q = dx * dx + dy * dy;
        if (q >= 1) return 0;
        if (puddleAllowed && !puddleAllowed(p.x, p.y)) return 0;
        return p.depth * std::min(1.0, 0.25 + rain) * (1 - q * q);
    }
    TestTrackGround* track() const { return track_ ? static_cast<TestTrackGround*>(inner_.get()) : nullptr; }

private:
    std::unique_ptr<GroundQuery> inner_;
    bool track_;

    void weather(GroundHit& h) const {
        SurfaceProperties& s = h.surface;
        const int id = h.surfaceId;
        if (rain > 0) {
            if (id == 4) s.frictionScale *= 1 - 0.45 * rain;        // wet setts: ~0.45 of dry asphalt
            else if (id == 5) s.frictionScale *= 1 - 0.55 * rain;   // wet paint: ~0.4
            else if (id == 13) { s.frictionScale *= 1 - 0.3 * rain; s.looseness = std::max(s.looseness, 0.6 * rain); }  // mud
            else if (id != 2 && id != 6 && id != 7) s.frictionScale *= 1 - 0.25 * rain;  // wet asphalt / paving
            s.wetness = std::max(s.wetness, rain);
            s.waterDepth = std::max(s.waterDepth, 0.0012 * rain);  // the film in a downpour
            if (id != 2 && id != 6 && id != 7) {
                const double pd = puddleDepth(h.point.x, h.point.y);
                if (pd > 0) { s.waterDepth = std::max(s.waterDepth, pd); h.surfaceId = 8; }
            }
        }
    }

    // March a height-field ray in steps no longer than a quarter of the height gap (safe on
    // slopes up to ~75 deg, where a fixed-point solve diverges), then bisect the crossing.
    GroundHit march(const Vec3& o, const Vec3& d, Real maxDistance) const {
        GroundHit h;
        auto f = [&](double t) { const Vec3 p = o + d * t; return p.z - provingGroundHeight(p.x, p.y); };
        double t0 = 0, f0 = f(0);
        if (f0 < 0) return h;
        double t1 = 0, f1 = f0;
        while (true) {
            t1 = t0 + std::max(0.02, 0.25 * f0);
            if (t1 > maxDistance) { t1 = maxDistance; f1 = f(t1); if (f1 > 0) return h; break; }
            f1 = f(t1);
            if (f1 <= 0) break;
            t0 = t1; f0 = f1;
        }
        for (int k = 0; k < 14; ++k) {
            const double tm = 0.5 * (t0 + t1), fm = f(tm);
            if (fm > 0) { t0 = tm; f0 = fm; } else { t1 = tm; f1 = fm; }
        }
        h.hit = true;
        h.distance = t1;
        h.point = o + d * t1;
        constexpr double e = 0.01;
        const double dx = (provingGroundHeight(h.point.x + e, h.point.y) - provingGroundHeight(h.point.x - e, h.point.y)) / (2 * e);
        const double dy = (provingGroundHeight(h.point.x, h.point.y + e) - provingGroundHeight(h.point.x, h.point.y - e)) / (2 * e);
        h.normal = normalized(Vec3{-dx, -dy, 1});
        h.surfaceId = 0;
        return h;
    }

    // The loop ribbon (LoopProfile): 2-D ray against the profile's segments in the x-z plane near
    // the ray, then the lane test in y at that point's arc length. Only hits from the inside count.
    GroundHit loop(const Vec3& o, const Vec3& d, Real maxDistance) const {
        GroundHit h;
        const LoopProfile& L = loopProfile();
        const double ox = o.x - kLoopX, oz = o.z;
        if (ox < L.xMin - 4 || ox > L.xMax + 4 || oz > L.zMax + 4 || o.y < kLoopY - 4 || o.y > kLoopY + kLoopShift + 4) return h;
        const double ex = ox + d.x * maxDistance, ez = oz + d.z * maxDistance;
        double best = maxDistance + 1;
        int bestSeg = -1; double bestU = 0;
        const int i0 = L.cx(std::min(ox, ex)), i1 = L.cx(std::max(ox, ex)), j0 = L.cz(std::min(oz, ez)), j1 = L.cz(std::max(oz, ez));
        for (int j = j0; j <= j1; ++j)
            for (int i = i0; i <= i1; ++i)
                for (int sg : L.grid[static_cast<std::size_t>(j) * L.nx + i]) {
                    const double ax = L.p[4 * sg], az = L.p[4 * sg + 1], bx = L.p[4 * sg + 4], bz = L.p[4 * sg + 5];
                    const double sx = bx - ax, sz = bz - az;
                    const double den = d.x * sz - d.z * sx;
                    if (std::abs(den) < 1e-12) continue;
                    // o + d t = a + s u
                    const double t = ((ax - ox) * sz - (az - oz) * sx) / den;
                    const double u = ((ax - ox) * d.z - (az - oz) * d.x) / den;
                    if (t < 0 || t > maxDistance || u < 0 || u > 1 || t >= best) continue;
                    // inward normal (-sin th, cos th): the ray must arrive from the inside
                    const double th = L.p[4 * sg + 2] + u * (L.p[4 * sg + 6] - L.p[4 * sg + 2]);
                    if (-std::sin(th) * d.x + std::cos(th) * d.z >= 0) continue;
                    const double sArc = L.p[4 * sg + 3] + u * (L.p[4 * sg + 7] - L.p[4 * sg + 3]);
                    const double py = o.y + d.y * t;
                    if (std::abs(py - (kLoopY + kLoopShift * sArc / L.length)) > kLoopHalfWidth) continue;
                    if (az + u * sz < 0.02) continue;  // where the profile meets the road, the road wins
                    best = t; bestSeg = sg; bestU = u;
                }
        if (bestSeg < 0) return h;
        const double th = L.p[4 * bestSeg + 2] + bestU * (L.p[4 * bestSeg + 6] - L.p[4 * bestSeg + 2]);
        h.hit = true;
        h.distance = best;
        h.point = o + d * best;
        h.normal = Vec3{-std::sin(th), 0, std::cos(th)};
        h.surfaceId = 9;
        return h;
    }
};

struct Colliders {
    // A wall segment. material 0 is rigid; 1 glass, 2 hollow block, 3 brick break once a hit
    // carries more energy than what is left of breakEnergy (hits that fall short crack it:
    // half their energy stays as damage). A broken segment no longer collides.
    struct Seg { double ax, ay, bx, by, z0, z1; int material = 0; double breakEnergy = 0, damage = 0; bool broken = false; };
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
    DemoGround* demo = nullptr;
    TestTrackGround* track = nullptr;
    HeightfieldGround* city = nullptr;
    int world = 0;
    Colliders col;
    double impact = 0;  // largest collision speed change since the last publish (m/s)
    struct BreakEvent { int seg, material; double px, py, nx, ny, speed, kept; };
    std::vector<BreakEvent> breaks;  // walls broken since the page last asked
    // Every hit harder than 1 m/s since the page last asked (for its crumple): contact point, wall
    // normal (pointing at the car), closing speed, kind (0 wall, 1 post or tree, 2 breakable wall
    // that broke).
    struct HitEvent { double px, py, pz, nx, ny, speed; int kind; };
    std::vector<HitEvent> hits;
    // The "shadow" car: a second car driven by the same inputs, with another setup or another
    // model, that collides with nothing (see ord_shadow_start).
    std::unique_ptr<Vehicle> shadow;
    double rain = 0;
    std::unique_ptr<Vehicle> vehicle;
    DriverInput input;
    AssistConfig assists;
    Real accumulator = 0;
    int carIndex = 0;
    // Garage tuning, re-applied whenever the car is (re)created. lock <= 0 = stock.
    struct Tuning {
        double steerLock = 0;
        int diffType = -1;  // -1 stock, 0 open, 1 locked, 2 clutch LSD
        double preload = 80, powerRamp = 0.45, coastRamp = 0.25;
    } tuning;
    // Full garage setup (alignment, springs, dampers, tyres, brakes, gears,
    // aero, environment) for car `setupCar`; applied before `tuning`.
    bool hasSetup = false;
    int setupCar = -1;
    VehicleSetup setup;
    std::string text;      // JSON in/out buffer shared with JS
    Vec3 wind{};
    double tyreWarmers = -1;  // deg C the tyres start at on (re)creation; < 0 = ambient
    // Two-wheeler (bicycle / e-scooter / motorcycle), driven instead of the car when present.
    std::unique_ptr<TwoWheeler> bike;
    TwoWheelerInput bikeInput;
    TwoWheelerAssists bikeAssists;
    Real bikeAccumulator = 0;
    int bikeKind = 0;
    double bikeImpact = 0;
    // Crash dummies: driver (left seat) and passenger, and the bike rider.
    OccupantSim occupant[2];
    OccupantDef occupantDef[2];
    Vec3 pendingDeltaV{};  // collision speed change this step, body frame
    // Damage mode (ord_set_damage): 0 = the crash model's impulses (the page draws its own
    // dents), 1 = the node/beam crash structure and mechanical damage (damage.hpp).
    int damageMode = 0;
    InjuryReport riderReport;
    std::uint32_t riderReports = 0;
};

Sim g;
constexpr Real kStep = 1.0 / 360.0;

// The demo's walls and posts for the crash structure (damage.hpp). Segments are thin walls with
// a height range; a node is in one if it has crossed it from the side the car's centre is on.
// Circles are posts and trees. Segment ids are their index; circles are 100000 + index.
struct WebObstacles final : ObstacleQuery {
    static constexpr int kCircleId = 100000;
    template <class F>
    void forCells(double x0, double y0, double x1, double y1, F&& f) const {
        const Colliders& c = g.col;
        if (c.grid.empty()) return;
        const int i0 = std::clamp(static_cast<int>((std::min(x0, x1) - c.gx0) / c.gcell), 0, c.gn - 1);
        const int i1 = std::clamp(static_cast<int>((std::max(x0, x1) - c.gx0) / c.gcell), 0, c.gn - 1);
        const int j0 = std::clamp(static_cast<int>((std::min(y0, y1) - c.gy0) / c.gcell), 0, c.gn - 1);
        const int j1 = std::clamp(static_cast<int>((std::max(y0, y1) - c.gy0) / c.gcell), 0, c.gn - 1);
        for (int j = j0; j <= j1; ++j)
            for (int i = i0; i <= i1; ++i)
                for (int id : c.grid[static_cast<std::size_t>(j) * c.gn + i]) f(id);
    }
    bool anyNear(const Vec3& p, Real r) const override {
        bool near = false;
        forCells(p.x - r, p.y - r, p.x + r, p.y + r, [&](int id) {
            if (near) return;
            if (id >= 0) {
                const auto& s = g.col.segs[static_cast<std::size_t>(id)];
                if (s.broken || p.z + r < s.z0 || p.z - r > s.z1) return;
                const double sx = s.bx - s.ax, sy = s.by - s.ay, l2 = sx * sx + sy * sy;
                const double t = l2 > 1e-12 ? std::clamp(((p.x - s.ax) * sx + (p.y - s.ay) * sy) / l2, 0.0, 1.0) : 0.0;
                near = std::hypot(p.x - s.ax - sx * t, p.y - s.ay - sy * t) < r;
            } else {
                const auto& c = g.col.circs[static_cast<std::size_t>(~id)];
                near = p.z + r >= c.z0 && p.z - r <= c.z1 && std::hypot(p.x - c.x, p.y - c.y) < r + c.r;
            }
        });
        return near;
    }
    ObstacleContact point(const Vec3& p, const Vec3& ref, Real margin) const override {
        ObstacleContact best;
        best.depth = -1e9;
        forCells(p.x - 0.5, p.y - 0.5, p.x + 0.5, p.y + 0.5, [&](int id) {
            if (id >= 0) {
                const auto& s = g.col.segs[static_cast<std::size_t>(id)];
                if (s.broken || p.z < s.z0 - 0.05 || p.z > s.z1) return;
                const double sx = s.bx - s.ax, sy = s.by - s.ay, l = std::hypot(sx, sy);
                if (l < 1e-6) return;
                const double ux = sx / l, uy = sy / l;
                const double t = (p.x - s.ax) * ux + (p.y - s.ay) * uy;
                if (t < 0 || t > l) return;
                const double side = ux * (p.y - s.ay) - uy * (p.x - s.ax);        // + left of a->b
                const double sideRef = ux * (ref.y - s.ay) - uy * (ref.x - s.ax);
                const double sg = sideRef >= 0 ? 1.0 : -1.0;
                const double depth = -side * sg;  // > 0 once the point has crossed to the far side
                if (depth < -margin || depth > 1.0 || depth <= best.depth) return;
                best.hit = true; best.depth = depth; best.normal = {-uy * sg, ux * sg, 0}; best.friction = 0.35; best.id = id;
            } else {
                const auto& c = g.col.circs[static_cast<std::size_t>(~id)];
                if (p.z < c.z0 || p.z > c.z1) return;
                const double dx = p.x - c.x, dy = p.y - c.y, d = std::hypot(dx, dy);
                const double depth = c.r - d;
                if (d < 1e-9 || depth < -margin || depth <= best.depth) return;
                best.hit = true; best.depth = depth; best.normal = {dx / d, dy / d, 0}; best.friction = 0.35; best.id = kCircleId + ~id;
            }
        });
        return best;
    }
    // Posts, trees and wall ends poking between the structure's nodes.
    ObstacleContact edge(const Vec3& a, const Vec3& b, Real margin, Real& t) const override {
        ObstacleContact best;
        best.depth = -1e9;
        const double ex = b.x - a.x, ey = b.y - a.y, l2 = ex * ex + ey * ey;
        if (l2 < 1e-12) return best;
        const double z = 0.5 * (a.z + b.z);
        auto test = [&](double cx, double cy, double r, double z0, double z1, int id) {
            if (z < z0 || z > z1) return;
            const double tt = std::clamp(((cx - a.x) * ex + (cy - a.y) * ey) / l2, 0.0, 1.0);
            const double qx = a.x + ex * tt, qy = a.y + ey * tt, dx = qx - cx, dy = qy - cy, d = std::hypot(dx, dy);
            const double depth = r - d;
            if (d < 1e-9 || depth < -margin || depth <= best.depth) return;
            best.hit = true; best.depth = depth; best.normal = {dx / d, dy / d, 0}; best.friction = 0.35; best.id = id; t = tt;
        };
        forCells(std::min(a.x, b.x) - 0.5, std::min(a.y, b.y) - 0.5, std::max(a.x, b.x) + 0.5, std::max(a.y, b.y) + 0.5, [&](int id) {
            if (id >= 0) {
                const auto& s = g.col.segs[static_cast<std::size_t>(id)];
                if (s.broken) return;
                test(s.ax, s.ay, 0.04, s.z0 - 0.05, s.z1, id);
                test(s.bx, s.by, 0.04, s.z0 - 0.05, s.z1, id);
            } else {
                const auto& c = g.col.circs[static_cast<std::size_t>(~id)];
                test(c.x, c.y, c.r, c.z0, c.z1, kCircleId + ~id);
            }
        });
        return best;
    }
};
WebObstacles g_obstacles;

// Crash structure on the player's car when the damage mode asks for it (after every rebuild).
void applyDamage() {
    if (!g.vehicle) return;
    if (g.damageMode != 1) {
        if (g.vehicle->damageEnabled()) g.vehicle->disableDamage();
        g.vehicle->setObstacles(nullptr);
        return;
    }
    DamageConfig cfg;
    if (g.carIndex == 9) cfg.strength = 1.8;  // the S 600 Guard: an armoured (VR10) cell and doors
    g.vehicle->enableDamage(cfg);
    g.vehicle->setObstacles(&g_obstacles);
}
// Rewind: snapshots of everything a step changes, newest last (see ord_snap_push).
struct Snapshot {
    std::optional<Vehicle> vehicle, shadow;
    OccupantSim occupant[2];
    Real accumulator = 0;
    DriverInput input;
    std::vector<std::pair<double, bool>> walls;  // damage, broken per segment
};
constexpr int kSnapCapacity = 240;
std::vector<std::unique_ptr<Snapshot>> g_snaps;  // ring
int g_snapHead = 0, g_snapCount = 0;
// Frame state exported to JavaScript (see web/README.md for the layout).
constexpr int kHeader = 39, kPerWheel = 17;
std::array<double, kHeader + kPerWheel * kMaxWheels> g_out{};

VehicleDefinition carByIndex(int i) {
    switch (i) {
        case 1: return vehicles::referenceHatchFwd();
        case 2: return vehicles::referenceSaloonAwd();
        case 3: return vehicles::referenceHypercarAwd();
        case 4: return vehicles::referenceRaceDownforce();
        case 5: return vehicles::rosserStellaV8();
        case 6: return vehicles::porsche911CarreraS();
        case 7: return vehicles::porsche911Carrera4S();
        case 8: return vehicles::mercedesAmgS63();
        case 9: return vehicles::mercedesS600Guard();
        case 10: return vehicles::formula1Car2026();
        default: return vehicles::referenceSportsRwd();
    }
}

void ensureGround() {
    if (g.ground) return;
    if (g.world == 1) {
        auto c = std::make_unique<HeightfieldGround>();
        g.city = c.get(); g.track = nullptr;
        auto d = std::make_unique<DemoGround>(std::move(c), false);
        HeightfieldGround* city = g.city;
        d->puddleAllowed = [city](double x, double y) { return city->classAt(x, y) <= 1; };
        d->rain = g.rain;
        g.demo = d.get();
        g.ground = std::move(d);
        return;
    }
    auto t = std::make_unique<TestTrackGround>(provingGroundHeight);
    t->addPatch({250, 450, 25, 75, wetSurface(), 2});
    t->addPatch({40, 300, kBumpLaneY - 4, kBumpLaneY + 4, bumpSurface(), 3});
    // proving ground
    t->addPatch({kPgX0, kPgX1, kPgBumpY - 4, kPgBumpY + 4, bumpSurface(), 3});
    t->addPatch({-200, -140, 25, 85, wetSurface(), 2});               // wet skidpad square
    t->addPatch({kPgX0, kPgX1, kPgSplitY, kPgSplitY + 5, wetSurface(), 2});  // split-mu: left half wet
    // aquaplaning lane: a pool across the lane, then a puddle under the left wheels
    t->addPatch({kAquaPoolX0, kAquaPoolX1, kAquaY - 4, kAquaY + 4, waterSurface(0.008), 6});
    t->addPatch({kAquaPuddleX0, kAquaPuddleX1, kAquaY, kAquaY + 3, waterSurface(0.012), 7});
    // surfaces lane: cobblestones, then painted stripes 0.5 m wide every metre (a zebra crossing)
    t->addPatch({kCobbleX0, kCobbleX1, kSurfY - 4, kSurfY + 4, cobbleSurface(), 4});
    for (Real x = kPaintX0; x < kPaintX1; x += 1.0) t->addPatch({x, x + 0.5, kSurfY - 4, kSurfY + 4, paintSurface(), 5});
    g.track = t.get(); g.city = nullptr;
    // Crash-test lane: a rigid concrete barrier across the lane at its east
    // end (frontal tests) and a 30 cm pole beside it (side-impact pole test).
    g.col.segs = {{kCrashBarrierX, kCrashLaneY - 8, kCrashBarrierX, kCrashLaneY + 8, -1, 1.5}};
    g.col.circs = {{kCrashPoleX, kCrashLaneY + 16, 0.15, -1, 6}};
    for (int k = 0; k < 3; ++k) {
        const double y0 = smashLaneY(k) - 3.75;
        for (int i = 0; i < 3; ++i) {
            Colliders::Seg s{kCrashBarrierX, y0 + 2.5 * i, kCrashBarrierX, y0 + 2.5 * (i + 1), k == 0 ? 0.1 : -0.5, k == 0 ? 2.8 : 2.0};
            s.material = k + 1;
            s.breakEnergy = kBreakEnergy[k];
            g.col.segs.push_back(s);
        }
    }
    // the half-pipe's parapets along the back of each deck: climb out and you hit concrete
    {
        const double uMax = kPipeR * std::sin(kPipeMaxAngle), top = kPipeR * (1 - std::cos(kPipeMaxAngle));
        for (double side : {1.0, -1.0}) {
            const double y = kPipeY + side * (kPipeFloor + uMax + kPipeDeck);
            g.col.segs.push_back({kPipeX0, y, kPipeX1, y, top - 0.5, top + 1.1});
        }
    }
    // the landing ramps' front faces: undershoot a jump and you hit a wall
    for (int k = 0; k < 2; ++k)
        g.col.segs.push_back({kLandX0[k], kJumpY[k] - 2.5, kLandX0[k], kJumpY[k] + 2.5, -1.0, kKickH[k] - 0.05});
    g.breaks.reserve(16);
    g.hits.reserve(16);
    g.col.gx0 = -500; g.col.gy0 = -300; g.col.gcell = 8; g.col.gn = 100;
    g.col.build();
    auto d = std::make_unique<DemoGround>(std::move(t), true);
    d->rain = g.rain;
    g.demo = d.get();
    g.ground = std::move(d);
}

double groundHeight(double x, double y) { return g.city ? g.city->heightAt(x, y) : provingGroundHeight(x, y); }

// Push the chassis out of walls / trunks and apply a contact impulse (restitution + friction).
void resolveCollisions() {
    if (g.col.segs.empty() && g.col.circs.empty()) return;
    Vehicle& v = *g.vehicle;
    const VehicleDefinition& d = v.definition();
    RigidBodyState b = v.state();
    const Vec3 vBefore = b.velocity;
    // body footprint from the wheel layout (+ overhangs)
    double xf = -1e9, xr = 1e9, hw = 0;
    for (const auto& w : d.wheels) { xf = std::max(xf, w.suspension.hardpoint.x); xr = std::min(xr, w.suspension.hardpoint.x); hw = std::max(hw, std::abs(w.suspension.hardpoint.y)); }
    xf += 1.0; xr -= 1.15; hw += 0.16;
    const double ex = 0.5 * (xf - xr), ey = hw, cxl = 0.5 * (xf + xr);
    const double m = d.mass;
    bool changed = false;
    for (int iter = 0; iter < 4; ++iter) {
        Vec3 u = b.orientation.rotate({1, 0, 0}); u.z = 0;
        const double ul = std::sqrt(u.x * u.x + u.y * u.y); if (ul < 1e-6) return;
        u.x /= ul; u.y /= ul;
        const double vx = -u.y, vy = u.x;
        const double cx = b.position.x + u.x * cxl, cy = b.position.y + u.y * cxl;
        const double zc = b.position.z;
        // deepest contact this iteration
        double best = 0, nx = 0, ny = 0, px = 0, py = 0, bestZ0 = 0, bestZ1 = 0;
        int bestSeg = -1;
        const int gi = std::clamp(static_cast<int>((cx - g.col.gx0) / g.col.gcell), 0, g.col.gn - 1);
        const int gj = std::clamp(static_cast<int>((cy - g.col.gy0) / g.col.gcell), 0, g.col.gn - 1);
        auto obbProj = [&](double ax, double ay) { return ex * std::abs(u.x * ax + u.y * ay) + ey * std::abs(vx * ax + vy * ay); };
        for (int dj = -1; dj <= 1; ++dj) for (int di = -1; di <= 1; ++di) {
            const int i = gi + di, j = gj + dj;
            if (i < 0 || j < 0 || i >= g.col.gn || j >= g.col.gn) continue;
            for (int id : g.col.grid[static_cast<std::size_t>(j) * g.col.gn + i]) {
                if (id >= 0) {
                    const auto& s = g.col.segs[static_cast<std::size_t>(id)];
                    if (s.broken) continue;
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
                    best = minO; nx = mx; ny = my; bestZ0 = s.z0; bestZ1 = s.z1; bestSeg = id;
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
                    best = pen; nx = lnx * u.x + lny * vx; ny = lnx * u.y + lny * vy; bestZ0 = c.z0; bestZ1 = c.z1; bestSeg = -1;
                    px = cx + qx * u.x + qy * vx; py = cy + qx * u.y + qy * vy;
                }
            }
        }
        if (best <= 0) break;
        // Impulse at the contact point, in 3D: the hit can pitch and roll the car as well as
        // yaw it. The contact height is the CoM's, kept within the obstacle's vertical extent,
        // so a low wall or post hits below the CoM and can trip a car sliding into it.
        const Vec3 com = b.position + b.orientation.rotate(d.centerOfMass);
        const Vec3 n{nx, ny, 0};
        const Vec3 cp{px, py, std::clamp(com.z, bestZ0 + 0.05, std::max(bestZ0 + 0.05, bestZ1 - 0.05))};
        const Vec3 r = cp - com;
        const Mat3 R = b.orientation.toMatrix();
        const Mat3 invI = R * Mat3::diagonal({1 / std::max(d.inertia.x, 1.0), 1 / std::max(d.inertia.y, 1.0), 1 / std::max(d.inertia.z, 1.0)}) * R.transposed();
        const Vec3 vc = b.velocity + cross(b.angularVelocity, r);
        const double vn = dot(vc, n);
        auto invMass = [&](const Vec3& dir) { const Vec3 rd = cross(r, dir); return 1 / m + dot(rd, invI * rd); };
        if (bestSeg >= 0 && vn < 0) {
            // A breakable wall: the hit's energy along the normal (with the effective mass at
            // the contact, so a glancing corner hit carries less) against what is left of the
            // wall's fracture energy. Through: the wall takes that much, the car keeps the rest
            // and is not pushed out. Short: the wall holds like a rigid one and is weakened.
            auto& s = g.col.segs[static_cast<std::size_t>(bestSeg)];
            if (s.material != 0 && s.breakEnergy > 0) {
                const double kin = 0.5 * vn * vn / invMass(n);
                const double left = s.breakEnergy - s.damage;
                if (kin >= left) {
                    const double rest = kin - left;
                    const double vnAfter = -std::sqrt(2 * rest * invMass(n));
                    const Vec3 jv = n * ((vnAfter - vn) / invMass(n));
                    b.velocity = b.velocity + jv * (1 / m);
                    b.angularVelocity = b.angularVelocity + invI * cross(r, jv);
                    s.broken = true;
                    if (g.breaks.size() < 16) g.breaks.push_back({bestSeg, s.material, cp.x, cp.y, nx, ny, -vn, rest / std::max(kin, 1e-9)});
                    if (g.hits.size() < 16) g.hits.push_back({cp.x, cp.y, cp.z, nx, ny, -vn, 2});
                    g.impact = std::max(g.impact, vnAfter - vn);
                    changed = true;
                    continue;  // no push-out; the broken section no longer collides
                }
                s.damage += 0.5 * kin;
            }
        }
        // positional correction (slightly more than the overlap)
        b.position.x += nx * (best + 0.002); b.position.y += ny * (best + 0.002);
        if (vn < 0) {
            const double e = vn < -6 ? 0.15 : 0.05;
            const double jn = -(1 + e) * vn / invMass(n);
            Vec3 jv = n * jn;
            const Vec3 vt = vc - n * vn;
            const double vtl = length(vt);
            if (vtl > 1e-6) {
                const Vec3 t = vt * (1 / vtl);
                const double mu = 0.35;
                jv = jv - t * std::min(vtl / invMass(t), mu * jn);
            }
            b.velocity = b.velocity + jv * (1 / m);
            b.angularVelocity = b.angularVelocity + invI * cross(r, jv);
            g.impact = std::max(g.impact, -vn);
            if (vn < -1 && g.hits.size() < 16) g.hits.push_back({cp.x, cp.y, cp.z, nx, ny, -vn, bestSeg >= 0 ? 0 : 1});
        }
        changed = true;
    }
    if (changed) {
        v.setChassisState(b);
        // The car's speed change from the hit, body frame, for the occupants'
        // crash pulse (the impulse itself happens in one step).
        g.pendingDeltaV = g.pendingDeltaV + b.orientation.inverseRotate(b.velocity - vBefore);
    }
}

void publishVehicle(const Vehicle& v, double* o) {
    const RigidBodyState& b = v.state();
    const TelemetryFrame& t = v.telemetry();
    const VehicleDefinition& d = v.definition();
    o[0] = b.position.x; o[1] = b.position.y; o[2] = b.position.z;
    o[3] = b.orientation.w; o[4] = b.orientation.x; o[5] = b.orientation.y; o[6] = b.orientation.z;
    o[7] = t.speed; o[8] = t.lateralVelocity; o[9] = t.yawRate; o[10] = t.sideslip;
    o[11] = t.accelBody.x; o[12] = t.accelBody.y; o[13] = t.accelBody.z;
    o[14] = t.drivetrain.engineRpm; o[15] = t.gear; o[16] = t.steerAngle; o[17] = t.throttleOut;
    o[18] = t.assists.tcsThrottleCut; o[19] = t.assists.countersteerAngle; o[20] = t.assists.escYawError;
    o[21] = t.time; o[22] = static_cast<double>(t.wheelCount); o[23] = v.faulted() ? 1 : 0;
    o[24] = t.assists.shifting ? 1 : 0; o[25] = d.wheels.empty() ? 0 : d.wheels[0].tire.radius;
    o[26] = t.clutchEngagement; o[27] = t.steerInput; o[28] = t.throttleInput; o[29] = t.brakeInput;
    o[30] = static_cast<double>(t.groundQueries); o[31] = 0;
    o[32] = t.assists.driftActive ? 1 : 0; o[33] = t.assists.driftTargetSideslip;
    o[34] = t.assists.driftThrottleCut; o[35] = t.assists.driftSteerAngle;
    o[36] = t.upright; o[37] = static_cast<double>(t.bodyContacts); o[38] = t.bodyContactForce;
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
        p[16] = wt.gripUsed;
    }
}

void publish() {
    publishVehicle(*g.vehicle, g_out.data());
    g_out[31] = g.impact; g.impact = 0;
}

std::array<double, kHeader + kPerWheel * kMaxWheels> g_shadowOut{};
void publishShadow() {
    if (g.shadow) publishVehicle(*g.shadow, g_shadowOut.data());
    else g_shadowOut.fill(0);
}

}  // namespace

// The car as the garage set it up (stock when no setup was applied to it).
VehicleDefinition currentDef() {
    const VehicleDefinition stock = carByIndex(g.carIndex);
    return g.hasSetup && g.setupCar == g.carIndex ? applySetup(stock, g.setup) : stock;
}

void applyTuning() {
    if (!g.vehicle) return;
    Vehicle& v = *g.vehicle;
    const VehicleDefinition stock = currentDef();
    Real stockLock = 0;
    for (const auto& w : stock.wheels) stockLock = std::max(stockLock, w.maxSteerAngle);
    v.setSteeringLock(g.tuning.steerLock > 0 ? g.tuning.steerLock : stockLock);
    for (std::size_t a = 0; a < stock.drivetrain.drivenAxles.size(); ++a) {
        DifferentialDef d = stock.drivetrain.drivenAxles[a].differential;
        // Only the rear (or only) driven axle is retuned: that is the drift diff.
        const bool rear = a + 1 == stock.drivetrain.drivenAxles.size();
        if (g.tuning.diffType >= 0 && rear) {
            d.type = g.tuning.diffType == 0 ? DifferentialType::Open
                     : g.tuning.diffType == 1 ? DifferentialType::Locked
                                              : DifferentialType::ClutchLsd;
            d.preload = g.tuning.preload;
            d.powerRamp = g.tuning.powerRamp;
            d.coastRamp = g.tuning.coastRamp;
        }
        v.setDifferential(a, d);
    }
}

void setupOccupants() {
    if (!g.vehicle) return;
    const VehicleDefinition& d = g.vehicle->definition();
    double half = 0.7;
    for (const auto& w : d.wheels) half = std::max(half, std::abs(w.suspension.hardpoint.y));
    CrashStructure s;
    s.halfWidth = half + 0.05;  // doors just outside the track
    for (int i = 0; i < 2; ++i) {
        OccupantDef& o = g.occupantDef[i];
        o.seat = {0, i == 0 ? 0.37 : -0.37, 0};  // left-hand drive: driver on the left
        g.occupant[i] = OccupantSim(o, s);
    }
    g.pendingDeltaV = {};
}

constexpr int kOccupantState = 24;
std::array<double, kOccupantState> g_occ{};
int verdictCode(const char* v) {
    static const char* names[] = {"walks away", "bruised", "injured", "seriously injured", "critical"};
    for (int i = 0; i < 5; ++i)
        if (v && std::string_view(v) == names[i]) return i;
    return -1;
}
void fillReport(double* o, const InjuryReport& r) {
    o[0] = r.valid ? 1 : 0; o[1] = static_cast<double>(r.zone); o[2] = r.deltaV; o[3] = r.vehiclePeakG;
    o[4] = r.peakG; o[5] = r.headPeakG; o[6] = r.chest3ms; o[7] = r.hic15; o[8] = r.excursion; o[9] = r.headExcursion;
    o[10] = r.intrusion; o[11] = r.hardContact ? 1 : 0; o[12] = r.pHeadAis3; o[13] = r.pLifeThreatening;
    o[14] = r.pFatalDeltaV; o[15] = verdictCode(r.verdict);
}

// ---- two-wheeler state (see web/README.md) ----
constexpr Real kBikeStep = 1.0 / 500.0;
constexpr int kBikeState = 66;
std::array<double, kBikeState> g_bike{};

TwoWheelerDef bikeByIndex(int i) {
    switch (i) {
        case 1: return electricScooter();
        case 2: return sportMotorcycle();
        default: return benchmarkBicycle();
    }
}

// A wall or post closer than ~0.3 m to the bike's centre line ends the ride.
bool bikeHitsObstacle() {
    if (!g.bike || (g.col.segs.empty() && g.col.circs.empty())) return false;
    const SingleTrack& m = g.bike->model();
    const Vec3 rear = m.position();
    const double yaw = m.yawAngle(), wb = g.bike->def().frame.wheelbase;
    const double ux = std::cos(yaw), uy = std::sin(yaw), r = 0.3;
    for (double s : {-0.1, 0.5 * wb, wb + 0.15}) {
        const double px = rear.x + ux * s, py = rear.y + uy * s, pz = rear.z;
        const int gi = std::clamp(static_cast<int>((px - g.col.gx0) / g.col.gcell), 0, g.col.gn - 1);
        const int gj = std::clamp(static_cast<int>((py - g.col.gy0) / g.col.gcell), 0, g.col.gn - 1);
        for (int id : g.col.grid[static_cast<std::size_t>(gj) * g.col.gn + gi]) {
            if (id >= 0) {
                const auto& sg = g.col.segs[static_cast<std::size_t>(id)];
                if (pz + 1.2 < sg.z0 || pz > sg.z1) continue;
                const double sx = sg.bx - sg.ax, sy = sg.by - sg.ay, l2 = sx * sx + sy * sy;
                if (l2 < 1e-8) continue;
                const double t = std::clamp(((px - sg.ax) * sx + (py - sg.ay) * sy) / l2, 0.0, 1.0);
                const double dx = px - (sg.ax + sx * t), dy = py - (sg.ay + sy * t);
                if (dx * dx + dy * dy < r * r) return true;
            } else {
                const auto& c = g.col.circs[static_cast<std::size_t>(~id)];
                if (pz + 1.2 < c.z0 || pz > c.z1) continue;
                const double dx = px - c.x, dy = py - c.y, rr = c.r + r;
                if (dx * dx + dy * dy < rr * rr) return true;
            }
        }
    }
    return false;
}

void publishBike() {
    double* o = g_bike.data();
    if (!g.bike) { g_bike.fill(0); return; }
    const TwoWheeler& b = *g.bike;
    const SingleTrack& m = b.model();
    const TwoWheelerTelemetry& t = b.telemetry();
    const TwoWheelerDef& d = b.def();
    o[0] = g.bikeKind; o[1] = t.fallen ? 1 : 0; o[2] = m.time(); o[3] = t.speed;
    o[4] = t.lean; o[5] = t.steer; o[6] = t.pitch; o[7] = t.leanTarget; o[8] = t.steerTorque;
    o[9] = t.driveTorque; o[10] = t.frontBrake; o[11] = t.rearBrake; o[12] = t.balanceTorque;
    o[13] = t.absActive ? 1 : 0; o[14] = t.tcsActive ? 1 : 0; o[15] = t.wheelieActive ? 1 : 0;
    o[16] = m.tyreOutput(0).gripUsed; o[17] = m.tyreOutput(1).gripUsed;
    o[18] = m.normalLoad(0); o[19] = m.normalLoad(1);
    o[20] = m.tyreOutput(0).camber; o[21] = m.tyreOutput(1).camber;
    o[22] = m.yawAngle();
    const Vec3 p = m.position();
    o[23] = p.x; o[24] = p.y; o[25] = p.z;
    o[26] = d.frame.rearWheel.radius; o[27] = d.frame.frontWheel.radius; o[28] = d.frame.wheelbase;
    o[29] = g.bikeImpact; g.bikeImpact = 0;
    o[30] = g.bikeInput.steer; o[31] = g.bikeInput.throttle;
    for (int i = 0; i < SingleTrack::kBodies; ++i) {
        const SingleTrackBody& bd = m.body(i);
        double* q = o + 32 + 7 * i;
        q[0] = bd.position.x; q[1] = bd.position.y; q[2] = bd.position.z;
        q[3] = bd.orientation.w; q[4] = bd.orientation.x; q[5] = bd.orientation.y; q[6] = bd.orientation.z;
    }
    o[60] = m.wheelSpin(0); o[61] = m.wheelSpin(1);
    o[62] = d.frame.rearTyre.crownRadius; o[63] = d.frame.frontTyre.crownRadius;
    double mass = 0;
    for (int i = 0; i < SingleTrack::kBodies; ++i) mass += m.body(i).mass;
    o[64] = mass; o[65] = d.frame.steerAxisTilt;
}

// Creates (or replaces) the car and places it at (x, y) heading `yaw`. Returns 1 on success.
ORD_EXPORT int ord_create(int carIndex, double x, double y, double yaw) {
    ensureGround();
    g.carIndex = carIndex;
    g.shadow.reset();
    g_snapCount = 0;
    g.vehicle = std::make_unique<Vehicle>(currentDef(), g.assists);
    if (!g.vehicle->isValid()) return 0;
    applyTuning();
    setupOccupants();
    g.vehicle->reset({x, y, groundHeight(x, y) + 1.0}, yaw, *g.ground);
    g.vehicle->setWind(g.wind);
    if (g.tyreWarmers >= 0) g.vehicle->setTyreTemperatures(g.tyreWarmers);
    applyDamage();
    g.accumulator = 0;
    g.input = {};
    publish();
    return 1;
}

// Rebuilds every broken wall (R does this too).
ORD_EXPORT void ord_reset_obstacles() {
    for (auto& s : g.col.segs) { s.broken = false; s.damage = 0; }
    g.breaks.clear();
    g.hits.clear();
}

ORD_EXPORT void ord_reset(double x, double y, double yaw) {
    if (!g.vehicle) return;
    ord_reset_obstacles();
    g.shadow.reset();
    g.vehicle->reset({x, y, groundHeight(x, y) + 1.0}, yaw, *g.ground);
    if (g.tyreWarmers >= 0) g.vehicle->setTyreTemperatures(g.tyreWarmers);
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
namespace {
void stepDamaged() {
    // The crash structure handles every contact; walls break from the work its contacts do on
    // them, and the occupants ride the structure's own pulse (the step's real acceleration).
    Vehicle& v = *g.vehicle;
    const Vec3 vBefore = v.state().velocity;
    v.step(kStep, g.input, *g.ground);
    if (g.shadow && !g.shadow->faulted()) g.shadow->step(kStep, g.input, *g.ground);
    const DamageModel& dm = v.damage();
    bool obstacleHit = false;
    for (int i = 0; i < dm.impactCount(); ++i) {
        const ObstacleImpact& im = dm.impact(i);
        obstacleHit = true;
        g.impact = std::max(g.impact, im.closingSpeed);
        int kind = im.id >= WebObstacles::kCircleId ? 1 : 0;
        if (im.id >= 0 && im.id < static_cast<int>(g.col.segs.size())) {
            auto& sg = g.col.segs[static_cast<std::size_t>(im.id)];
            if (sg.material != 0 && sg.breakEnergy > 0 && !sg.broken) {
                sg.damage += im.work;
                if (sg.damage >= sg.breakEnergy) {
                    sg.broken = true;
                    kind = 2;
                    if (g.breaks.size() < 16)
                        g.breaks.push_back({im.id, sg.material, im.point.x, im.point.y, im.normal.x, im.normal.y, im.closingSpeed, 0.5});
                }
            }
        }
        if (im.closingSpeed > 1 && g.hits.size() < 16)
            g.hits.push_back({im.point.x, im.point.y, im.point.z, im.normal.x, im.normal.y, im.closingSpeed, kind});
    }
    Vec3 acc = v.telemetry().accelBody;
    const double am = length(acc), cap = 150 * 9.81;
    if (am > cap) acc = acc * (cap / am);
    if (obstacleHit) {
        const Vec3 dv = v.state().orientation.inverseRotate(v.state().velocity - vBefore);
        for (auto& o : g.occupant) o.recordImpact({dv.x, dv.y, 0}, v.damageState().intrusion);
    }
    for (auto& o : g.occupant) o.step(kStep, acc);
    g.input.shiftUp = g.input.shiftDown = false;
}

void stepOnce() {
    if (g.damageMode == 1 && g.vehicle->damageEnabled()) { stepDamaged(); return; }
    g.vehicle->step(kStep, g.input, *g.ground);
    if (g.shadow && !g.shadow->faulted()) g.shadow->step(kStep, g.input, *g.ground);
    // driving acceleration seen by the occupants (before any collision
    // impulse; a hit reaches them as a crush pulse instead)
    Vec3 acc = g.vehicle->telemetry().accelBody;
    const double am = length(acc), cap = 40 * 9.81;
    if (am > cap) acc = acc * (cap / am);
    resolveCollisions();
    if (length(g.pendingDeltaV) > 0.5)
        for (auto& o : g.occupant) o.impact(g.pendingDeltaV);
    g.pendingDeltaV = {};
    for (auto& o : g.occupant) o.step(kStep, acc);
    g.input.shiftUp = g.input.shiftDown = false;
}
}  // namespace

ORD_EXPORT int ord_advance(double dt) {
    if (!g.vehicle || g.vehicle->faulted()) return 0;
    g.accumulator += std::min(dt, 0.1);  // never try to catch up more than 100 ms
    int n = 0;
    while (g.accumulator >= kStep) {
        stepOnce();
        g.accumulator -= kStep;
        ++n;
        if (g.vehicle->faulted()) break;
    }
    publish();
    return n;
}

// Exactly n fixed steps, whatever the frame time (the crash lab samples every step).
ORD_EXPORT int ord_step(int n) {
    if (!g.vehicle) return 0;
    int k = 0;
    for (; k < n && !g.vehicle->faulted(); ++k) stepOnce();
    publish();
    return k;
}

ORD_EXPORT double* ord_state() { return g_out.data(); }
ORD_EXPORT int ord_state_header() { return kHeader; }
ORD_EXPORT int ord_state_per_wheel() { return kPerWheel; }
ORD_EXPORT double ord_ground_height(double x, double y) { return groundHeight(x, y); }

// ---- world selection and city data upload ----
// 0 = test track (default), 1 = city heightfield. Call before ord_create; recreates the ground.
ORD_EXPORT void ord_set_world(int w) {
    g.world = w == 1 ? 1 : 0;
    g.vehicle.reset(); g.shadow.reset(); g.ground.reset(); g.demo = nullptr; g.city = nullptr; g.track = nullptr;
    g.col.segs.clear(); g.col.circs.clear(); g.col.build();
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

// ---- car tuning ----
// Steering lock (rad, 0 = stock) and the rear differential (type -1 stock, 0 open,
// 1 locked, 2 clutch LSD with preload N*m and power/coast ramps). Applies now and on re-creation.
ORD_EXPORT void ord_set_tuning(double steerLock, int diffType, double preload, double powerRamp, double coastRamp) {
    g.tuning = {steerLock, diffType, preload, powerRamp, coastRamp};
    applyTuning();
}

// ---- garage setup (JSON; see ord/setup.hpp for the fields and units) ----
// JS writes UTF-8 JSON into ord_text_buffer(n) (n bytes, NUL added here), then
// calls ord_apply_setup(). Returns 1, or 0 with the reason in ord_text().
namespace {
std::string g_textOut;
void rebuildKeepingMotion() {
    // A new set of parts: rebuild the car where it is, at the speed it had.
    if (!g.vehicle) return;
    const RigidBodyState b = g.vehicle->state();
    const Real vx = g.vehicle->forwardSpeed();
    g.vehicle = std::make_unique<Vehicle>(currentDef(), g.assists);
    applyTuning();
    setupOccupants();
    g.vehicle->setChassisState(b);
    g.vehicle->setRollingSpeed(vx);
    applyDamage();
    g.vehicle->setWind(g.wind);
    if (g.tyreWarmers >= 0) g.vehicle->setTyreTemperatures(g.tyreWarmers);
    publish();
}
}  // namespace

ORD_EXPORT char* ord_text_buffer(int n) {
    g.text.assign(static_cast<std::size_t>(std::max(n, 0)) + 1, '\0');
    return g.text.data();
}
ORD_EXPORT const char* ord_text() { return g_textOut.c_str(); }
ORD_EXPORT int ord_text_length() { return static_cast<int>(g_textOut.size()); }

ORD_EXPORT int ord_apply_setup() {
    VehicleSetup s = g.hasSetup && g.setupCar == g.carIndex ? g.setup : stockSetup(carByIndex(g.carIndex));
    const std::string err = setupFromJson(std::string(g.text.c_str()), s);
    if (!err.empty()) { g_textOut = err; return 0; }
    const VehicleDefinition d = applySetup(carByIndex(g.carIndex), s);
    const auto problems = d.validate();
    if (!problems.empty()) { g_textOut = problems.front(); return 0; }
    g.setup = s;
    g.hasSetup = true;
    g.setupCar = g.carIndex;
    rebuildKeepingMotion();
    g_textOut.clear();
    return 1;
}
// Back to the car as it comes (also clears it for the next ord_create).
ORD_EXPORT void ord_clear_setup() {
    g.hasSetup = false;
    rebuildKeepingMotion();
}
// The current setup of the current car (stock if none) as JSON in ord_text().
ORD_EXPORT int ord_setup_json(int stock) {
    const VehicleSetup s = !stock && g.hasSetup && g.setupCar == g.carIndex ? g.setup : stockSetup(carByIndex(g.carIndex));
    g_textOut = setupToJson(s);
    return static_cast<int>(g_textOut.size());
}
// Tyre warmers: start (and restart) the tyres at this tread temperature, deg C; < 0 = ambient.
ORD_EXPORT void ord_set_tyre_warmers(double celsius) {
    g.tyreWarmers = celsius;
    if (g.vehicle) g.vehicle->setTyreTemperatures(celsius >= 0 ? celsius : g.vehicle->definition().aero.ambientTemperature);
}
// Steady wind, world frame (m/s, the direction the air moves toward).
ORD_EXPORT void ord_set_wind(double wx, double wy) {
    g.wind = {wx, wy, 0};
    if (g.vehicle) g.vehicle->setWind(g.wind);
}
namespace {
constexpr int kGarageState = 14 + 4 * kMaxWheels;
std::array<double, kGarageState> g_garage{};
}  // namespace
// Garage readouts, refreshed on call: aero (airspeed, airflow yaw, front/rear
// downforce, drag, wing downforce, floor clearance front/rear, wing stalled,
// floor stalled, ambient C, air density, front and rear roll-centre height
// above the ground in m), then per wheel: camber (rad, negative = top in),
// tread temperature (C), the tyre's optimal temperature, and its grip factor
// from temperature.
ORD_EXPORT double* ord_garage_state() {
    g_garage.fill(0);
    if (!g.vehicle) return g_garage.data();
    const Vehicle& v = *g.vehicle;
    const TelemetryFrame& t = v.telemetry();
    const AeroTelemetry& a = t.aero;
    double* o = g_garage.data();
    o[0] = a.airspeed; o[1] = a.yaw; o[2] = a.downforceFront; o[3] = a.downforceRear; o[4] = a.drag;
    o[5] = a.wingDownforce; o[6] = a.clearanceFront; o[7] = a.clearanceRear;
    o[8] = a.wingStalled ? 1 : 0; o[9] = a.floorStalled ? 1 : 0;
    o[10] = v.definition().aero.ambientTemperature; o[11] = v.definition().aero.airDensity;
    o[12] = t.rollCentreHeight[0]; o[13] = v.definition().axles.size() > 1 ? t.rollCentreHeight[1] : 0.0;
    for (std::size_t w = 0; w < t.wheelCount; ++w) {
        const TireParams& tp = v.definition().wheels[w].tire;
        double* p = o + 14 + 4 * static_cast<int>(w);
        p[0] = t.wheels[w].camber; p[1] = t.wheels[w].temperature; p[2] = tp.optimalTemperature;
        p[3] = tp.thermalCapacity > 0 ? MagicFormulaTire(tp).temperatureGrip(t.wheels[w].temperature) : 1.0;
    }
    return g_garage.data();
}
ORD_EXPORT int ord_garage_state_size() { return kGarageState; }

// ---- two-wheelers ----
// kind: 0 bicycle, 1 e-scooter, 2 sport motorcycle. Rear contact at (x, y), heading yaw, rolling at speed.
ORD_EXPORT int ord_bike_create(int kind, double x, double y, double yaw, double speed) {
    ensureGround();
    g.bikeKind = kind;
    g.bike = std::make_unique<TwoWheeler>(bikeByIndex(kind), g.bikeAssists);
    g.bikeInput = {};
    g.bikeAccumulator = 0;
    const bool ok = g.bike->reset(x, y, yaw, speed, g.ground.get());
    publishBike();
    return ok ? 1 : 0;
}
ORD_EXPORT void ord_bike_remove() { g.bike.reset(); publishBike(); }
// mode 0 = Lean (input asks for lean), 1 = Direct (input is handlebar torque).
ORD_EXPORT void ord_bike_set_assists(int mode, double balance, int abs, int tcs, int wheelie, double wheeliePitch) {
    g.bikeAssists.mode = mode == 1 ? RiderMode::Direct : RiderMode::Lean;
    g.bikeAssists.balanceAssist = balance < 0 ? 0 : balance > 1 ? 1 : balance;
    g.bikeAssists.abs = abs != 0;
    g.bikeAssists.tractionControl = tcs != 0;
    g.bikeAssists.wheelieControl = wheelie != 0;
    g.bikeAssists.wheeliePitch = wheeliePitch;
    if (g.bike) g.bike->setAssists(g.bikeAssists);
}
ORD_EXPORT void ord_bike_set_input(double steer, double throttle, double frontBrake, double rearBrake) {
    g.bikeInput = {steer, throttle, frontBrake, rearBrake};
}
// Advances real time dt in fixed 1/500 s steps. Returns the steps taken.
ORD_EXPORT int ord_bike_advance(double dt) {
    if (!g.bike || g.bike->fallen()) { publishBike(); return 0; }
    g.bikeAccumulator += std::min(dt, 0.1);
    int n = 0;
    while (g.bikeAccumulator >= kBikeStep) {
        g.bike->step(kBikeStep, g.bikeInput, g.ground.get());
        g.bikeAccumulator -= kBikeStep;
        ++n;
        const double vBike = g.bike->telemetry().speed;
        if (bikeHitsObstacle()) {
            g.bikeImpact = std::abs(vBike);
            g.bike->crash();
        }
        if (g.bike->fallen()) {
            // The rider: into the wall at the bike's speed; over the bars or
            // looped: head and shoulders first; a low-side: dropped from the
            // seat onto the road (the slide that follows decelerates at
            // ~0.5 g, which the occupant model does not count as injury).
            const double pitch = g.bike->telemetry().pitch;
            Vec3 dv{0, -std::sqrt(2 * 9.81 * 0.8), 0};
            if (g.bikeImpact > 0) dv = {-g.bikeImpact, 0, 0};
            else if (std::abs(pitch) > 1.0) dv = {-(4.0 + 0.3 * std::abs(vBike)), 0, 0};
            OccupantDef rider = motorcycleRider();
            rider.helmet = g.bikeKind != 0;  // the benchmark cyclist rides bare-headed in the demo
            g.riderReport = analyseImpact(dv, rider, riderBody());
            ++g.riderReports;
            break;
        }
    }
    publishBike();
    return n;
}
ORD_EXPORT double* ord_bike_state() { return g_bike.data(); }
ORD_EXPORT int ord_bike_state_size() { return kBikeState; }

// ---- crash dummies ----
// belted, frontAirbag, sideAirbag (applies to both front seats, now and on re-creation).
ORD_EXPORT void ord_set_occupants(int belted, int frontAirbag, int sideAirbag) {
    for (auto& o : g.occupantDef) {
        o.belted = belted != 0;
        o.frontAirbag = frontAirbag != 0;
        o.sideAirbag = sideAirbag != 0;
    }
    setupOccupants();
}
// Seat 0 driver, 1 passenger, 2 bike rider (last crash). 24 doubles: torso dx, dy, head dx, dy (m,
// body frame), current g, event active, events so far, then the last finished report (16 values,
// see web/README.md), seat y.
ORD_EXPORT double* ord_occupant_state(int seat) {
    g_occ.fill(0);
    double* o = g_occ.data();
    if (seat == 2) {
        o[6] = g.riderReports;
        fillReport(o + 7, g.riderReport);
        return o;
    }
    if (seat < 0 || seat > 1) return o;
    const OccupantSim& s = g.occupant[seat];
    const Vec3 t = s.displacement(), h = s.headDisplacement();
    o[0] = t.x; o[1] = t.y; o[2] = h.x; o[3] = h.y; o[4] = s.currentG();
    o[5] = s.eventActive() ? 1 : 0; o[6] = s.eventCount();
    fillReport(o + 7, s.report());
    o[23] = g.occupantDef[seat].seat.y;
    return o;
}
// Crash-test lane: y, barrier x, pole x (pole centre is 16 m north of the lane centre, 30 cm across).
ORD_EXPORT double ord_crash_lane_y() { return kCrashLaneY; }
// Smash lane k (0 glass storefront, 1 hollow-block wall, 2 brick wall): its centre y.
ORD_EXPORT double ord_smash_lane_y(int k) { return smashLaneY(k < 0 ? 0 : k > 2 ? 2 : k); }
// The world's wall segments for drawing: 8 doubles each (ax, ay, bx, by, z0, z1, material, broken).
namespace { std::vector<double> g_obst; std::array<double, 16 * 8> g_breaks{}; }
ORD_EXPORT int ord_obstacle_count() { return static_cast<int>(g.col.segs.size()); }
ORD_EXPORT double* ord_obstacles() {
    g_obst.resize(g.col.segs.size() * 8);
    double* o = g_obst.data();
    for (const auto& s : g.col.segs) { o[0] = s.ax; o[1] = s.ay; o[2] = s.bx; o[3] = s.by; o[4] = s.z0; o[5] = s.z1; o[6] = s.material; o[7] = s.broken ? 1 : 0; o += 8; }
    return g_obst.data();
}
// Walls broken since the last call: returns the count; ord_break_buffer() holds 8 doubles each
// (segment, material, hit x, y, wall normal x, y, closing speed m/s, fraction of the hit's energy
// the car kept).
ORD_EXPORT int ord_break_events() {
    const int n = static_cast<int>(g.breaks.size());
    for (int i = 0; i < n; ++i) {
        const auto& e = g.breaks[static_cast<std::size_t>(i)];
        double* o = g_breaks.data() + 8 * i;
        o[0] = e.seg; o[1] = e.material; o[2] = e.px; o[3] = e.py; o[4] = e.nx; o[5] = e.ny; o[6] = e.speed; o[7] = e.kept;
    }
    g.breaks.clear();
    return n;
}
ORD_EXPORT double* ord_break_buffer() { return g_breaks.data(); }
ORD_EXPORT double ord_roll_kerb_y() { return kRollKerbY; }
ORD_EXPORT double ord_roll_ramp_lane_y() { return kRollRampLaneY; }
ORD_EXPORT double ord_crash_barrier_x() { return kCrashBarrierX; }
ORD_EXPORT double ord_crash_pole_x() { return kCrashPoleX; }

// ---- weather ----
// Rain 0 (dry) .. 1 (downpour): wet asphalt (~0.75 of dry), a 1.2 mm water film, much slicker
// cobblestones and paint, muddy soil, and puddles 6-20 mm deep that make a tyre hydroplane
// (see hydroplaneLift in tire.hpp).
ORD_EXPORT void ord_set_weather(double rain) {
    g.rain = rain < 0 ? 0 : rain > 1 ? 1 : rain;
    if (g.demo) g.demo->rain = g.rain;
}
// Puddles within `radius` of (x, y) while it rains: returns the count; ord_puddle_buffer() holds
// 4 doubles each (x, y, half length along x, depth m); the half width along y is 0.6 of the length.
namespace { std::array<double, 4 * 256> g_puddles{}; }
ORD_EXPORT int ord_puddles(double x, double y, double radius) {
    if (!g.demo || g.rain <= 0) return 0;
    const long i0 = static_cast<long>(std::floor((x - radius) / kPuddleCell)), i1 = static_cast<long>(std::floor((x + radius) / kPuddleCell));
    const long j0 = static_cast<long>(std::floor((y - radius) / kPuddleCell)), j1 = static_cast<long>(std::floor((y + radius) / kPuddleCell));
    int n = 0;
    for (long j = j0; j <= j1; ++j)
        for (long i = i0; i <= i1 && n < 256; ++i) {
            Puddle p;
            if (!puddleInCell(i, j, p)) continue;
            if (g.demo->puddleDepth(p.x, p.y) <= 0) continue;  // not on a road, or a test patch
            const GroundHit h = g.ground->raycast({p.x, p.y, groundHeight(p.x, p.y) + 2}, {0, 0, -1}, 5);
            if (!h.hit || h.surfaceId != 8) continue;
            double* o = g_puddles.data() + 4 * n++;
            o[0] = p.x; o[1] = p.y; o[2] = p.r; o[3] = h.surface.waterDepth;
        }
    return n;
}
ORD_EXPORT double* ord_puddle_buffer() { return g_puddles.data(); }

// ---- test areas: centre-line y (and x ranges) for the page ----
// 0 moose lane y, 1 aquaplaning lane y, 2 pool x0, 3 pool x1, 4 puddle x0, 5 puddle x1,
// 6 surfaces lane y, 7 cobbles x0, 8 cobbles x1, 9 paint x0, 10 paint x1,
// 11/12 jump lanes y, 13/14 kicker x0, 15/16 kicker length, 17/18 kicker height,
// 19/20 landing x0, 21/22 landing length, 23 half-pipe y, 24 pipe x0, 25 pipe x1, 26 pipe floor
// half width, 27 pipe radius, 28 pipe deck, 29 loop x, 30 loop y, 31 loop radius, 32 loop shift
// per turn, 33 loop half width, 34 loop heading (rad).
namespace { std::array<double, 35> g_areas{}; }
ORD_EXPORT double* ord_test_areas() {
    const double a[35] = {kMooseY, kAquaY, kAquaPoolX0, kAquaPoolX1, kAquaPuddleX0, kAquaPuddleX1, kSurfY, kCobbleX0, kCobbleX1,
                          kPaintX0, kPaintX1, kJumpY[0], kJumpY[1], kKickX0[0], kKickX0[1], kKickLen[0], kKickLen[1], kKickH[0],
                          kKickH[1], kLandX0[0], kLandX0[1], kLandLen[0], kLandLen[1], kPipeY, kPipeX0, kPipeX1, kPipeFloor, kPipeR,
                          kPipeDeck, kLoopX, kLoopY, kLoopR, kLoopShift, kLoopHalfWidth, loopHeading()};
    std::copy(a, a + 35, g_areas.begin());
    return g_areas.data();
}

// ---- rewind ----
// Saves the whole simulation (car, shadow car, dummies, walls, input, step remainder). The page
// takes one every ~0.1 s and keeps the frame times and inputs in between, so restoring the
// snapshot and replaying those frames lands exactly on any earlier frame (the step is
// deterministic). Holds the last kSnapCapacity.
ORD_EXPORT int ord_snap_push() {
    if (!g.vehicle) return 0;
    if (g_snaps.empty()) g_snaps.resize(kSnapCapacity);
    auto& slot = g_snaps[static_cast<std::size_t>(g_snapHead)];
    if (!slot) slot = std::make_unique<Snapshot>();
    Snapshot& s = *slot;
    s.vehicle = *g.vehicle;
    if (g.shadow) s.shadow = *g.shadow; else s.shadow.reset();
    s.occupant[0] = g.occupant[0]; s.occupant[1] = g.occupant[1];
    s.accumulator = g.accumulator;
    s.input = g.input;
    s.walls.resize(g.col.segs.size());
    for (std::size_t i = 0; i < g.col.segs.size(); ++i) s.walls[i] = {g.col.segs[i].damage, g.col.segs[i].broken};
    g_snapHead = (g_snapHead + 1) % kSnapCapacity;
    g_snapCount = std::min(g_snapCount + 1, kSnapCapacity);
    return g_snapCount;
}
ORD_EXPORT int ord_snap_count() { return g_snapCount; }
// Restores the k-th newest snapshot (0 = newest) and forgets the newer ones. Returns 1 on success.
ORD_EXPORT int ord_snap_restore(int k) {
    if (k < 0 || k >= g_snapCount || !g.vehicle) return 0;
    const int idx = ((g_snapHead - 1 - k) % kSnapCapacity + kSnapCapacity) % kSnapCapacity;
    const Snapshot& s = *g_snaps[static_cast<std::size_t>(idx)];
    if (!s.vehicle) return 0;
    *g.vehicle = *s.vehicle;
    if (s.shadow) { if (g.shadow) *g.shadow = *s.shadow; else g.shadow = std::make_unique<Vehicle>(*s.shadow); }
    else g.shadow.reset();
    g.occupant[0] = s.occupant[0]; g.occupant[1] = s.occupant[1];
    g.accumulator = s.accumulator;
    g.input = s.input;
    for (std::size_t i = 0; i < g.col.segs.size() && i < s.walls.size(); ++i) {
        g.col.segs[i].damage = s.walls[i].first;
        g.col.segs[i].broken = s.walls[i].second;
    }
    g.pendingDeltaV = {};
    g.breaks.clear();
    g.hits.clear();
    g_snapHead = (idx + 1) % kSnapCapacity;
    g_snapCount -= k;
    publish();
    return 1;
}
ORD_EXPORT void ord_snap_clear() { g_snapCount = 0; }

// ---- shadow car ----
// Starts a second car where the car is now, moving as it moves: carIndex < 0 = the same model;
// stock = 1 ignores the garage setup (so you drive your setup against the stock car, or one model
// against another, with the same hands). It takes the same inputs, collides with nothing and
// does not touch the dummies. Returns 1 on success.
ORD_EXPORT int ord_shadow_start(int carIndex, int stock) {
    if (!g.vehicle) return 0;
    const int idx = carIndex < 0 ? g.carIndex : carIndex;
    VehicleDefinition d = carByIndex(idx);
    if (!stock && idx == g.carIndex && g.hasSetup && g.setupCar == g.carIndex) d = applySetup(d, g.setup);
    auto v = std::make_unique<Vehicle>(d, g.assists);
    if (!v->isValid()) return 0;
    const RigidBodyState& b = g.vehicle->state();
    const Vec3 fwd = b.orientation.rotate({1, 0, 0}), left = b.orientation.rotate({0, 1, 0});
    const double yaw = std::atan2(fwd.y, fwd.x);
    v->placeOnGround(b.position.x, b.position.y, yaw, *g.ground, dot(b.velocity, fwd), dot(b.velocity, left), b.angularVelocity.z);
    v->setWind(g.wind);
    g.shadow = std::move(v);
    publishShadow();
    return 1;
}
ORD_EXPORT void ord_shadow_stop() { g.shadow.reset(); }
ORD_EXPORT double* ord_shadow_state() { publishShadow(); return g_shadowOut.data(); }

// ---- crash lab ----
// Puts the car at (x, y) heading yaw, already rolling at `speed` (m/s, forward) and sliding at
// `lateral` (m/s, positive = toward its left), with fresh dummies: a test sled.
ORD_EXPORT void ord_launch(double x, double y, double yaw, double speed, double lateral) {
    if (!g.vehicle) return;
    g.vehicle->placeOnGround(x, y, yaw, *g.ground, speed, lateral, 0);
    if (g.tyreWarmers >= 0) g.vehicle->setTyreTemperatures(g.tyreWarmers);
    setupOccupants();
    g.accumulator = 0;
    g.input = {};
    g.hits.clear();
    publish();
}
// Hits since the last call (see Sim::HitEvent): count; ord_hit_buffer() holds 7 doubles each.
namespace { std::array<double, 16 * 7> g_hitOut{}; }
ORD_EXPORT int ord_hit_events() {
    const int n = static_cast<int>(g.hits.size());
    for (int i = 0; i < n; ++i) {
        const auto& e = g.hits[static_cast<std::size_t>(i)];
        double* o = g_hitOut.data() + 7 * i;
        o[0] = e.px; o[1] = e.py; o[2] = e.pz; o[3] = e.nx; o[4] = e.ny; o[5] = e.speed; o[6] = e.kind;
    }
    g.hits.clear();
    return n;
}
ORD_EXPORT double* ord_hit_buffer() { return g_hitOut.data(); }
// The crush structure the dummies' pulse comes from: front, rear, side natural frequency (rad/s),
// front, rear, side crush limit (m), cabin half width (m). The page's crumple uses the same.
namespace { std::array<double, 7> g_struct{}; }
ORD_EXPORT double* ord_crash_structure() {
    const CrashStructure s;
    double half = 0.7;
    if (g.vehicle) for (const auto& w : g.vehicle->definition().wheels) half = std::max(half, std::abs(w.suspension.hardpoint.y));
    g_struct = {s.frontOmega, s.rearOmega, s.sideOmega, s.frontCrush, s.rearCrush, s.sideCrush, half + 0.05};
    return g_struct.data();
}
// Mass properties for the overlay: CoM (body frame, m), mass (kg), inertia (kg m^2).
namespace { std::array<double, 7> g_info{}; }
// The loop's profile for drawing: samples of (x relative to the loop's x, z, slope angle,
// arc length); returns the pointer, ord_loop_profile_count() the sample count.
ORD_EXPORT double* ord_loop_profile() { return const_cast<double*>(loopProfile().p.data()); }
ORD_EXPORT int ord_loop_profile_count() { return loopProfile().n; }
ORD_EXPORT double* ord_vehicle_info() {
    g_info.fill(0);
    if (!g.vehicle) return g_info.data();
    const VehicleDefinition& d = g.vehicle->definition();
    g_info = {d.centerOfMass.x, d.centerOfMass.y, d.centerOfMass.z, d.mass, d.inertia.x, d.inertia.y, d.inertia.z};
    return g_info.data();
}

// ---- damage (damage.hpp) ----
// 0: the crash model's impulses (the page draws its own dents); 1: the node/beam crash structure
// with mechanical damage on the player's car (rebuilt with the car; repaired on change).
ORD_EXPORT void ord_set_damage(int mode) {
    g.damageMode = mode == 1 ? 1 : 0;
    applyDamage();
    publish();
}
ORD_EXPORT int ord_damage_mode() { return g.damageMode; }
ORD_EXPORT void ord_repair() {
    if (g.vehicle) g.vehicle->repairDamage();
}
// Parts and structure, 22 + 6 per wheel doubles (see web/README.md, Damage).
constexpr int kDamageHeader = 22, kDamagePerWheel = 6;
std::array<double, kDamageHeader + kDamagePerWheel * kMaxWheels> g_damage{};
ORD_EXPORT double* ord_damage_state() {
    g_damage.fill(0);
    if (!g.vehicle || !g.vehicle->damageEnabled()) return g_damage.data();
    const DamageState& d = g.vehicle->damageState();
    double* o = g_damage.data();
    o[0] = 1; o[1] = d.awake ? 1 : 0; o[2] = d.version;
    o[3] = d.crushFront; o[4] = d.crushRear; o[5] = d.crushLeft; o[6] = d.crushRight; o[7] = d.crushRoof;
    o[8] = d.intrusion; o[9] = d.plasticWork; o[10] = d.brokenBeams;
    o[11] = d.radiator; o[12] = d.coolant; o[13] = d.engineTemperature; o[14] = d.engineDamage; o[15] = d.engineSeized ? 1 : 0;
    o[16] = d.torqueScale; o[17] = d.steeringOffset; o[18] = d.steeringLock; o[19] = d.dragScale; o[20] = d.wingScale;
    o[21] = d.nodeContacts;
    const std::size_t nw = g.vehicle->definition().wheels.size();
    for (std::size_t w = 0; w < nw && w < kMaxWheels; ++w) {
        const WheelDamage& wd = d.wheels[w];
        double* q = o + kDamageHeader + kDamagePerWheel * w;
        q[0] = wd.toe; q[1] = wd.camber; q[2] = wd.bend; q[3] = wd.detached ? 1 : 0; q[4] = wd.punctured ? 1 : 0; q[5] = wd.deflation;
    }
    return o;
}
// The structure's nodes, 8 doubles each: rest x, y, z, deformed x, y, z (body frame, ISO 8855),
// zone (0 cell, 1 front, 2 rear, 3 side, 4 roof), free (0 anchored to the cell, 1 free).
std::vector<double> g_cage;
ORD_EXPORT int ord_cage_count() { return g.vehicle && g.vehicle->damageEnabled() ? g.vehicle->damage().nodeCount() : 0; }
ORD_EXPORT double* ord_cage_nodes() {
    const int n = ord_cage_count();
    g_cage.resize(static_cast<std::size_t>(std::max(n, 1)) * 8);
    for (int i = 0; i < n; ++i) {
        const CageNode& c = g.vehicle->damage().node(i);
        double* o = g_cage.data() + 8 * i;
        o[0] = c.rest.x; o[1] = c.rest.y; o[2] = c.rest.z; o[3] = c.local.x; o[4] = c.local.y; o[5] = c.local.z;
        o[6] = static_cast<double>(c.zone); o[7] = c.invMass > 0 ? 1 : 0;
    }
    return g_cage.data();
}
// Beams, 4 doubles each: node a, node b, broken, plastic deformation / length.
std::vector<double> g_beams;
ORD_EXPORT int ord_cage_beam_count() { return g.vehicle && g.vehicle->damageEnabled() ? g.vehicle->damage().beamCount() : 0; }
ORD_EXPORT double* ord_cage_beams() {
    const int n = ord_cage_beam_count();
    g_beams.resize(static_cast<std::size_t>(std::max(n, 1)) * 4);
    for (int i = 0; i < n; ++i) {
        const CageBeam& b = g.vehicle->damage().beam(i);
        double* o = g_beams.data() + 4 * i;
        o[0] = b.a; o[1] = b.b; o[2] = b.broken ? 1 : 0; o[3] = b.plastic / std::max(b.length0, 1e-6);
    }
    return g_beams.data();
}
