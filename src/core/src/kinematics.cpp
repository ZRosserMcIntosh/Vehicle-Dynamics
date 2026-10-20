// OpenRoad Dynamics — suspension linkage kinematics.
// SPDX-License-Identifier: MIT
#include "ord/kinematics.hpp"

#include <cmath>

namespace ord {

namespace {

// Rodrigues rotation of v about the unit axis k by angle a.
Vec3 rotateAbout(const Vec3& v, const Vec3& k, Real a) {
    const Real c = std::cos(a), s = std::sin(a);
    return v * c + cross(k, v) * s + k * (dot(k, v) * (1 - c));
}

// A point on an arm that pivots about the axis through a and b.
Vec3 swing(const Vec3& p, const Vec3& a, const Vec3& b, Real angle) {
    const Vec3 axis = normalized(b - a);
    return a + rotateAbout(p - a, axis, angle);
}

// Minimal rotation taking unit vector f to unit vector t, applied to v.
Vec3 alignRotate(const Vec3& v, const Vec3& f, const Vec3& t) {
    const Vec3 axis = cross(f, t);
    const Real s = length(axis), c = clamp(dot(f, t), -1.0, 1.0);
    if (s < 1e-12) return c > 0 ? v : v * -1.0;  // parallel (anti-parallel never happens for a sane linkage)
    return rotateAbout(v, axis / s, std::atan2(s, c));
}

// Newton's method on a scalar function with a numeric derivative.
template <class F>
bool newton(F f, Real& x, Real tol, int iters = 40) {
    for (int i = 0; i < iters; ++i) {
        const Real v = f(x);
        if (std::abs(v) < tol) return true;
        const Real h = 1e-6;
        const Real d = (f(x + h) - f(x - h)) / (2 * h);
        if (!(std::abs(d) > 1e-14) || !std::isfinite(d)) return false;
        Real step = v / d;
        step = clamp(step, -0.5, 0.5);  // radians; keeps the first iterations on the right branch
        x -= step;
        if (!std::isfinite(x)) return false;
    }
    return std::abs(f(x)) < tol * 100;
}

struct Upright {
    // Pose of the upright: a point fixed to it maps as P = lbj + R (P0 - lbj0).
    Vec3 lbj0, lbj, axis0, axis;
    Real phi = 0;
    Vec3 map(const Vec3& p0) const {
        const Vec3 r = alignRotate(p0 - lbj0, axis0, axis);
        return lbj + rotateAbout(r, axis, phi);
    }
    Vec3 mapDir(const Vec3& d0) const { return rotateAbout(alignRotate(d0, axis0, axis), axis, phi); }
};

}  // namespace

bool LinkageTable::solveAt(const LinkageDef& def, Real t, Real& thetaLower, Real& thetaUpper, Real& phi, Row& row,
                           std::string* error) const {
    const bool strut = def.type == LinkageType::MacPherson;
    const Vec3 lbj0 = def.lowerBallJoint;
    const Vec3 top0 = strut ? def.strutTop : def.upperBallJoint;
    const Real bjDist = length(top0 - lbj0);
    const Real tie0 = length(def.tieRodOuter - def.tieRodInner);
    const Vec3 axis0 = normalized(top0 - lbj0);

    // The pose for a given lower-arm angle: nested solves for the upper arm and the steer.
    auto pose = [&](Real thL, Real& thU, Real& ph, Upright& u) -> bool {
        u.lbj0 = lbj0;
        u.axis0 = axis0;
        u.lbj = swing(lbj0, def.lowerPivotFront, def.lowerPivotRear, thL);
        Vec3 top = top0;
        if (!strut) {
            // Upper ball joint: on its arm's circle, at the upright's fixed ball-joint distance.
            auto gap = [&](Real a) {
                const Vec3 p = swing(def.upperBallJoint, def.upperPivotFront, def.upperPivotRear, a);
                return length(p - u.lbj) - bjDist;
            };
            if (!newton(gap, thU, 1e-10)) return false;
            top = swing(def.upperBallJoint, def.upperPivotFront, def.upperPivotRear, thU);
        }
        u.axis = normalized(top - u.lbj);
        // Steer about the kingpin axis until the tie rod has its length again.
        auto tie = [&](Real a) {
            u.phi = a;
            return length(u.map(def.tieRodOuter) - def.tieRodInner) - tie0;
        };
        if (!newton(tie, ph, 1e-10)) return false;
        u.phi = ph;
        return true;
    };

    // Lower-arm angle that puts the wheel centre at height t.
    Real thU = thetaUpper, ph = phi;
    Upright u;
    auto height = [&](Real thL) {
        Real a = thU, b = ph;
        Upright uu;
        if (!pose(thL, a, b, uu)) return 1e9;
        return uu.map(Vec3{}).z - t;
    };
    Real thL = thetaLower;
    if (!newton(height, thL, 1e-9)) {
        if (error) *error = "linkage cannot reach wheel travel " + std::to_string(t) + " m";
        return false;
    }
    if (!pose(thL, thU, ph, u)) {
        if (error) *error = "linkage solve failed at wheel travel " + std::to_string(t) + " m";
        return false;
    }
    thetaLower = thL;
    thetaUpper = thU;
    phi = ph;

    const Vec3 wc = u.map(Vec3{});
    const Vec3 axle = u.mapDir({0, 1, 0});
    const Vec3 heading = u.mapDir({1, 0, 0});
    row.t = t;
    row.lon = wc.x;
    row.lat = wc.y;
    // Left wheel at +y: top leaning in (toward -y) raises the +y end of the axle.
    row.camber = -std::asin(clamp(axle.z, -1.0, 1.0));
    // Heading turned toward -y (toe-in on the left) is a negative yaw.
    row.toe = -std::atan2(heading.y, heading.x);
    const Vec3 springLow = strut ? u.map(def.springArm) : swing(def.springArm, def.lowerPivotFront, def.lowerPivotRear, thL);
    row.springLength = length(def.springBody - springLow);
    if (!std::isfinite(row.camber) || !std::isfinite(row.toe) || !std::isfinite(row.springLength)) {
        if (error) *error = "linkage pose is not finite";
        return false;
    }
    return true;
}

bool LinkageTable::build(const LinkageDef& def, Real tMin, Real tMax, std::string* error) {
    valid_ = false;
    if (def.type == LinkageType::Slider) {
        if (error) *error = "slider has no linkage";
        return false;
    }
    if (!(tMax > tMin) || !std::isfinite(tMin) || !std::isfinite(tMax)) {
        if (error) *error = "bad travel range";
        return false;
    }
    const bool strut = def.type == LinkageType::MacPherson;
    auto shortArm = [](const Vec3& a, const Vec3& b) { return length(b - a) < 0.02; };
    if (shortArm(def.lowerPivotFront, def.lowerPivotRear) || (!strut && shortArm(def.upperPivotFront, def.upperPivotRear))) {
        if (error) *error = "pivot axis points are too close together";
        return false;
    }
    if (length(def.lowerBallJoint - def.lowerPivotFront) < 0.05 || length(def.tieRodOuter - def.tieRodInner) < 0.05 ||
        length((strut ? def.strutTop : def.upperBallJoint) - def.lowerBallJoint) < 0.05) {
        if (error) *error = "an arm, the tie rod or the upright is shorter than 5 cm";
        return false;
    }
    tMin_ = tMin;
    tMax_ = tMax;
    const Real dt = (tMax - tMin) / (kSamples - 1);
    // March outward from the design pose (t = 0, all angles 0) so each solve
    // starts from its neighbour and stays on the right branch.
    int i0 = static_cast<int>(std::round((0.0 - tMin) / dt));
    i0 = i0 < 0 ? 0 : i0 >= kSamples ? kSamples - 1 : i0;
    Real thL = 0, thU = 0, ph = 0;
    for (int i = i0; i < kSamples; ++i) {
        if (!solveAt(def, tMin + i * dt, thL, thU, ph, rows_[static_cast<std::size_t>(i)], error)) return false;
    }
    thL = thU = ph = 0;
    for (int i = i0 - 1; i >= 0; --i) {
        if (!solveAt(def, tMin + i * dt, thL, thU, ph, rows_[static_cast<std::size_t>(i)], error)) return false;
    }
    // The path must be single-valued: lateral scrub and camber bounded, no fold-over.
    for (int i = 0; i < kSamples; ++i) {
        const Row& r = rows_[static_cast<std::size_t>(i)];
        if (std::abs(r.camber) > 0.6 || std::abs(r.toe) > 0.6 || std::abs(r.lat) > 0.3 || std::abs(r.lon) > 0.3) {
            if (error) *error = "linkage folds over within the travel (camber, toe or scrub out of range)";
            return false;
        }
    }
    // Derivatives along the travel (central differences, one-sided at the ends),
    // interpolated like the values so nothing steps between samples.
    for (int i = 0; i < kSamples; ++i) {
        const int a = i == 0 ? 0 : i - 1, b = i == kSamples - 1 ? kSamples - 1 : i + 1;
        const Row& ra = rows_[static_cast<std::size_t>(a)];
        const Row& rb = rows_[static_cast<std::size_t>(b)];
        const Real span = (b - a) * dt;
        Row& r = rows_[static_cast<std::size_t>(i)];
        r.dLon = (rb.lon - ra.lon) / span;
        r.dLat = (rb.lat - ra.lat) / span;
        r.dCamber = (rb.camber - ra.camber) / span;
        r.mr = (rb.springLength - ra.springLength) / span;
    }
    valid_ = true;  // at() may be used from here on
    mrDesign_ = at(0.0).motionRatio;
    if (std::abs(mrDesign_) < 0.05) {
        valid_ = false;
        if (error) *error = "spring hardly moves with the wheel (motion ratio below 0.05)";
        return false;
    }
    // Spring deflection in wheel metres: the integral of the normalised motion ratio.
    rows_[0].deflection = 0;
    for (int i = 1; i < kSamples; ++i) {
        const Real rho = 0.5 * (rows_[static_cast<std::size_t>(i - 1)].mr + rows_[static_cast<std::size_t>(i)].mr) / mrDesign_;
        if (rho < 0.1) {
            valid_ = false;
            if (error) *error = "spring motion ratio reverses within the travel";
            return false;
        }
        rows_[static_cast<std::size_t>(i)].deflection = rows_[static_cast<std::size_t>(i - 1)].deflection + rho * dt;
    }
    const Real s0 = springDeflection(0.0);
    for (Row& r : rows_) r.deflection -= s0;
    return true;
}

LinkagePose LinkageTable::at(Real t) const {
    LinkagePose p;
    if (!valid_) return p;
    const Real dt = (tMax_ - tMin_) / (kSamples - 1);
    Real f = (t - tMin_) / dt;
    int i = static_cast<int>(std::floor(f));
    if (i < 0) i = 0;
    if (i > kSamples - 2) i = kSamples - 2;
    const Real u = clamp(f - i, 0.0, 1.0);
    const Row& a = rows_[static_cast<std::size_t>(i)];
    const Row& b = rows_[static_cast<std::size_t>(i + 1)];
    const Real ext = f - i - u;  // beyond the table: continue linearly along the end segment
    auto lerpx = [&](Real va, Real vb) { return va + (vb - va) * (u + ext); };
    p.lon = lerpx(a.lon, b.lon);
    p.lat = lerpx(a.lat, b.lat);
    p.camber = lerpx(a.camber, b.camber);
    p.toe = lerpx(a.toe, b.toe);
    p.springLength = lerpx(a.springLength, b.springLength);
    p.dLon = lerpx(a.dLon, b.dLon);
    p.dLat = lerpx(a.dLat, b.dLat);
    p.dCamber = lerpx(a.dCamber, b.dCamber);
    p.motionRatio = lerpx(a.mr, b.mr);
    return p;
}

Real LinkageTable::springDeflection(Real t) const {
    if (!valid_) return t;
    const Real dt = (tMax_ - tMin_) / (kSamples - 1);
    const Real f = (t - tMin_) / dt;
    int i = static_cast<int>(std::floor(f));
    if (i < 0) i = 0;
    if (i > kSamples - 2) i = kSamples - 2;
    const Row& a = rows_[static_cast<std::size_t>(i)];
    const Row& b = rows_[static_cast<std::size_t>(i + 1)];
    return a.deflection + (b.deflection - a.deflection) * (f - i);
}

LinkageDef makeDoubleWishbone(Real armLength, Real upperArmLength, Real uprightHeight, Real kingpinOffset, Real antiDive,
                              Real springFraction) {
    LinkageDef d;
    d.type = LinkageType::DoubleWishbone;
    const Real lowerZ = -0.45 * uprightHeight, upperZ = 0.55 * uprightHeight;
    d.lowerBallJoint = {0.0, -0.02, lowerZ};
    d.upperBallJoint = {0.0, -0.02 - kingpinOffset, upperZ};
    const Real tilt = std::tan(antiDive);
    // The classic short-long-arm layout: the lower arm about level (inboard pivot
    // a touch higher), the shorter upper arm angled down toward its inboard
    // pivot. In bump the upper ball joint swings inboard faster than the lower
    // one, so the wheel gains negative camber.
    // Anti-dive: the pivot axes rise toward the rear, so the wheel's path leans
    // forward as it compresses and a braking force pushes it toward droop.
    d.lowerPivotFront = {0.18, d.lowerBallJoint.y - armLength, lowerZ + 0.01 - 0.18 * tilt};
    d.lowerPivotRear = {-0.18, d.lowerBallJoint.y - armLength, lowerZ + 0.01 + 0.18 * tilt};
    d.upperPivotFront = {0.12, d.upperBallJoint.y - upperArmLength, upperZ - 0.035 - 0.12 * tilt};
    d.upperPivotRear = {-0.12, d.upperBallJoint.y - upperArmLength, upperZ - 0.035 + 0.12 * tilt};
    // Tie rod behind the axle, parallel to the lower arm and nearly as long:
    // the textbook layout for little bump steer.
    {
        const Vec3 pivotMid = (d.lowerPivotFront + d.lowerPivotRear) * 0.5;
        const Vec3 arm = pivotMid - d.lowerBallJoint;  // outboard joint to inboard pivot
        d.tieRodOuter = d.lowerBallJoint + Vec3{-0.13, -0.03, 0.045};
        d.tieRodInner = d.tieRodOuter + arm * ((armLength - 0.06) / armLength);
    }
    // Coil-over on the lower arm, part way out, leaning in toward the body.
    const Vec3 pivotMid = (d.lowerPivotFront + d.lowerPivotRear) * 0.5;
    d.springArm = pivotMid + (d.lowerBallJoint - pivotMid) * springFraction + Vec3{0, 0, 0.01};
    d.springBody = {0.0, d.springArm.y - 0.04, d.springArm.z + 0.42};
    return d;
}

LinkageDef makeMacPherson(Real armLength, Real strutHeight, Real strutTopInboard, Real antiDive) {
    LinkageDef d;
    d.type = LinkageType::MacPherson;
    const Real lowerZ = -0.14;
    d.lowerBallJoint = {0.0, -0.03, lowerZ};
    const Real tilt = std::tan(antiDive);
    d.lowerPivotFront = {0.19, d.lowerBallJoint.y - armLength, lowerZ + 0.01 - 0.19 * tilt};
    d.lowerPivotRear = {-0.19, d.lowerBallJoint.y - armLength, lowerZ + 0.01 + 0.19 * tilt};
    d.strutTop = {0.0, d.lowerBallJoint.y - strutTopInboard, lowerZ + strutHeight};
    {
        const Vec3 pivotMid = (d.lowerPivotFront + d.lowerPivotRear) * 0.5;
        const Vec3 arm = pivotMid - d.lowerBallJoint;
        d.tieRodOuter = d.lowerBallJoint + Vec3{-0.14, -0.03, 0.05};
        d.tieRodInner = d.tieRodOuter + arm * ((armLength - 0.06) / armLength);
    }
    // The coil sits on the strut: its lower seat moves with the upright.
    d.springArm = d.lowerBallJoint + (d.strutTop - d.lowerBallJoint) * 0.35;
    d.springBody = d.strutTop;
    return d;
}

Real rollCentreHeight(const LinkagePose& pose, Real halfTrack, Real contactBelow) {
    // Front view, left wheel at +y. Per metre of wheel travel the upright moves
    // (dLat, 1) and rotates about body x by -dCamber (camber negative = top in
    // = rotation toward -y = positive about +x). Its instant centre is where
    // that velocity vanishes; the line from the contact patch through the
    // instant centre crosses the centreline at the roll centre:
    //   h = halfTrack * (dLat - contactBelow * dCamber)
    // which has no singularity when the camber does not change (instant centre
    // at infinity: the line is perpendicular to the path).
    return halfTrack * (pose.dLat - contactBelow * pose.dCamber);
}

}  // namespace ord
