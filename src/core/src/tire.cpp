// OpenRoad Dynamics — engine-independent vehicle physics core.
// SPDX-License-Identifier: MIT
#include "ord/tire.hpp"

#include <cstdint>

namespace ord {

namespace {

// Magic Formula core: y = D sin(C atan(B x - E (B x - atan(B x))))
Real magicFormula(Real x, Real B, Real C, Real D, Real E) {
    const Real bx = B * x;
    return D * std::sin(C * std::atan(bx - E * (bx - std::atan(bx))));
}

Real loadDelta(const TireParams& p, Real fz) { return (fz - p.nominalLoad) / p.nominalLoad; }

}  // namespace

Real MagicFormulaTire::pureLongitudinal(Real fz, Real kappa, Real muScale) const {
    if (fz <= 0) return 0;
    const Real dfz = loadDelta(p_, fz);
    const Real mu = std::max(0.0, p_.muX * (1 + p_.muXLoadSensitivity * dfz)) * muScale;
    const Real D = mu * fz;
    if (D <= 1e-9) return 0;
    const Real K = p_.longStiffnessPerLoad * fz;
    const Real C = p_.shapeX;
    const Real B = K / (C * D);
    return magicFormula(kappa, B, C, D, std::min(p_.curvatureX, 1.0));
}

Real MagicFormulaTire::pureLateral(Real fz, Real alpha, Real muScale, Real camber) const {
    if (fz <= 0) return 0;
    const Real dfz = loadDelta(p_, fz);
    const Real mu = std::max(0.0, p_.muY * (1 + p_.muYLoadSensitivity * dfz)) * muScale;
    const Real D = mu * fz;
    if (D <= 1e-9) return 0;
    const Real Fz0 = p_.nominalLoad;
    const Real K = p_.cornerStiffness * Fz0 * std::sin(2.0 * std::atan(fz / (p_.cornerStiffnessLoad * Fz0)));
    const Real C = p_.shapeY;
    const Real B = K / (C * D);
    if (p_.camberStiffness <= 0 || camber == 0) return magicFormula(alpha, B, C, D, std::min(p_.curvatureY, 1.0));
    // Motorcycle MF: camber adds to the argument of the sine, so camber thrust
    // and slip force share one friction limit D.
    const Real E = std::min(p_.curvatureY, 1.0);
    const Real bx = B * alpha;
    const Real Cg = std::max(p_.camberShape, 1e-3);
    const Real Bg = p_.camberStiffness * fz / (Cg * D);
    const Real arg = C * std::atan(bx - E * (bx - std::atan(bx))) + Cg * std::atan(Bg * camber);
    // Past pi/2 the force falls off as in pure slip; never past pi (sign flip).
    return D * std::sin(clamp(arg, -0.95 * kPi, 0.95 * kPi));
}

TireForces MagicFormulaTire::evaluate(Real fz, Real kappa, Real tanAlpha, const SurfaceProperties& surf,
                                      Real speed, Real camber, Real gripScale) const {
    TireForces out;
    if (fz <= 0) return out;

    const Real wet = 1.0 / (1.0 + std::max(0.0, surf.wetSpeedFalloff * p_.aquaplaneSensitivity) * std::abs(speed));
    const Real compound = (1 + clamp(surf.wetness, 0.0, 1.0) * (p_.wetGrip - 1)) *
                          (1 + clamp(surf.looseness, 0.0, 1.0) * (p_.looseGrip - 1));
    // A lifted share of the patch rides on water (mu ~0.05 of the road's).
    const Real lifted = surf.waterDepth > 0 ? hydroplaneLift(p_, surf.waterDepth, speed) : 0.0;
    const Real base = std::max(0.0, surf.frictionScale) * wet * std::max(0.0, compound) * std::max(0.0, gripScale) *
                      (1.0 - 0.95 * lifted);
    const Real alpha = std::atan(tanAlpha);
    // Camber and peak grip: lateral relative to leaning into the force
    // direction (smooth sign of the slip angle), longitudinal from |gamma|.
    Real muScaleY = base, muScaleX = base;
    if (p_.camberGripLoss > 0 || p_.camberLongGripLoss > 0) {
        const Real into = camber * std::tanh(alpha / 0.03);
        muScaleY *= std::max(0.5, 1 - p_.camberGripLoss * (into - p_.camberOptimum) * (into - p_.camberOptimum));
        muScaleX *= std::max(0.5, 1 - p_.camberLongGripLoss * camber * camber);
    }
    const Real muScale = muScaleX;

    auto combinedFx = [&](Real k) {
        const Real Bxa = p_.rBx1 * std::cos(std::atan(p_.rBx2 * k));
        const Real Gxa = std::cos(p_.rCx1 * std::atan(Bxa * alpha));
        return Gxa * pureLongitudinal(fz, k, muScale);
    };

    const Real Byk = p_.rBy1 * std::cos(std::atan(p_.rBy2 * alpha));
    const Real Gyk = std::cos(p_.rCy1 * std::atan(Byk * kappa));

    out.fx = combinedFx(kappa);
    out.fy = Gyk * pureLateral(fz, alpha, muScaleY, camber);

    // Central-difference slope for the linearly implicit wheel-spin solve.
    constexpr Real h = 1e-4;
    out.dFxdKappa = (combinedFx(kappa + h) - combinedFx(kappa - h)) / (2 * h);

    // Self-aligning torque from a collapsing pneumatic trail (simplified; the
    // MF Mz formulation with residual torque is roadmap work for force feedback).
    const Real trail = p_.pneumaticTrail * std::max(0.0, 1.0 - std::abs(alpha) / p_.trailCollapseAngle);
    out.mz = -trail * out.fy + p_.camberTwist * fz * camber;

    const Real dfz = loadDelta(p_, fz);
    out.muX = std::max(0.0, p_.muX * (1 + p_.muXLoadSensitivity * dfz)) * muScaleX;
    out.muY = std::max(0.0, p_.muY * (1 + p_.muYLoadSensitivity * dfz)) * muScaleY;
    return out;
}

Real MagicFormulaTire::temperatureGrip(Real t) const {
    // Cold rubber is hard and grips like a harder compound (floor 75%);
    // overheated rubber blisters and goes greasy (floor 60%).
    const Real x = (t - p_.optimalTemperature) / std::max(p_.temperatureWindow, 1.0);
    return std::max(x < 0 ? 0.75 : 0.6, 1 - p_.temperatureGripLoss * x * x);
}

Real hydroplaningSpeed(const TireParams& t) {
    constexpr Real rhoW = 1000.0, cl = 0.643;  // reproduces Horne's 6.35 sqrt(p[kPa]) km/h
    return std::sqrt(2.0 * std::max(t.inflationPressure, 10.0) * 1000.0 / (rhoW * cl));
}

Real hydroplaneLift(const TireParams& t, Real waterDepth, Real speed) {
    if (waterDepth <= 0) return 0;
    const Real d0 = 0.0005 + 0.1 * t.treadDepth, d1 = 0.001 + 0.4 * t.treadDepth;
    const Real depth = clamp((waterDepth - d0) / (d1 - d0), 0.0, 1.0);
    const Real r = clamp((std::abs(speed) / hydroplaningSpeed(t) - 0.5) / 0.5, 0.0, 1.0);
    return depth * r * r * (3 - 2 * r);
}

Real waterDrag(const TireParams& t, Real waterDepth, Real speed) {
    if (waterDepth <= 0) return 0;
    constexpr Real cd = 0.75;  // displacement drag coefficient (EASA CS-25 AMC 25.1591)
    return 0.5 * 1000.0 * cd * (2 * t.patchHalfWidth) * waterDepth * speed * speed;
}

Real surfaceTextureHeight(const SurfaceProperties& s, Real x, Real y) {
    if (s.textureAmplitude <= 0 || s.textureWavelength <= 1e-3) return 0;
    const Real l = s.textureWavelength;
    const Real ci = std::floor(x / l), cj = std::floor(y / l);
    // Integer hash of the stone cell -> [0, 1).
    auto mix = [](std::uint64_t h) {
        h ^= h >> 33; h *= 0xff51afd7ed558ccdULL; h ^= h >> 33; h *= 0xc4ceb9fe1a85ec53ULL; h ^= h >> 33;
        return h;
    };
    const std::uint64_t key = (static_cast<std::uint64_t>(static_cast<std::int64_t>(ci)) * 0x9E3779B97F4A7C15ULL) ^
                              static_cast<std::uint64_t>(static_cast<std::int64_t>(cj));
    const Real rnd = static_cast<Real>(mix(key) >> 11) * (1.0 / 9007199254740992.0);
    const Real scale = 1.0 - clamp(s.textureVariation, 0.0, 1.0) * rnd;
    const Real sx = std::sin(kPi * x / l), sy = std::sin(kPi * y / l);
    return s.textureAmplitude * scale * sx * sx * sy * sy;
}

void MagicFormulaTire::integrateLateralSlip(TireSlipState& s, Real vx, Real vy, Real dt) const {
    const Real sigma = std::max(p_.relaxationLengthY, 1e-3);
    s.tanAlpha = (s.tanAlpha - dt * vy / sigma) / (1.0 + dt * std::abs(vx) / sigma);
    s.tanAlpha = clamp(s.tanAlpha, -kSlipStateLimit, kSlipStateLimit);
}

}  // namespace ord
