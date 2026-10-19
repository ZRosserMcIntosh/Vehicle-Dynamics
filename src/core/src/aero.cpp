// OpenRoad Dynamics — engine-independent vehicle physics core.
// SPDX-License-Identifier: MIT
//
// Ported from Aviation Dynamics (src/core/src/aero.cpp, atmosphere.cpp; MIT,
// same author), reduced to what a car needs. See ord/aero.hpp.
#include "ord/aero.hpp"

#include <algorithm>

namespace ord {

namespace {

constexpr Real kHalfPi = kPi / 2;
constexpr Real kR = 287.05287;     // J/(kg K), USSA-1976
constexpr Real kG0 = 9.80665;
constexpr Real kT0 = 288.15, kLapse = -0.0065;

Real smoothstep01(Real e0, Real e1, Real x) {
    const Real t = clamp((x - e0) / (e1 - e0), 0.0, 1.0);
    return t * t * (3 - 2 * t);
}

struct Viterna {
    Real a1, a2, b1, b2;
    Viterna(Real alphaS, Real clS, Real cdS, Real cdMax) {
        const Real s = std::sin(alphaS), c = std::cos(alphaS);
        b1 = cdMax;
        a1 = b1 / 2;
        b2 = (cdS - cdMax * s * s) / c;
        a2 = (clS - cdMax * s * c) * s / (c * c);
    }
    Real cl(Real a) const {
        const Real s = std::max(std::sin(a), 1e-6), c = std::cos(a);
        return a1 * std::sin(2 * a) + a2 * c * c / s;
    }
    Real cd(Real a) const { return b1 * std::sin(a) * std::sin(a) + b2 * std::cos(a); }
};

}  // namespace

Real airPressure(Real h, Real p0) {
    const Real tIsa = kT0 + kLapse * std::min(h, 11000.0);
    return p0 * std::pow(tIsa / kT0, -kG0 / (kR * kLapse));
}

Real airDensity(Real h, Real temperatureC, Real p0) {
    // ISA pressure at h (troposphere), with the actual temperature for density.
    return airPressure(h, p0) / (kR * (temperatureC + 273.15));
}

Real enginePowerFactor(Real h, Real temperatureC) {
    return (airPressure(h) / 99000.0) * std::sqrt(298.15 / (temperatureC + 273.15));
}

WingSectionResult evaluateWingSection(const WingSectionPolar& p, Real alpha, Real a) {
    WingSectionResult r;
    alpha = clamp(alpha, -kHalfPi, kHalfPi);
    const Real clLin = a * (alpha - p.zeroLiftAlpha);
    const Real alphaSp = p.zeroLiftAlpha + p.clMax / a;
    const Real alphaSm = p.zeroLiftAlpha + p.clMin / a;
    auto attachedCd = [&](Real cl) { return p.cdMin + p.cdQuadratic * (cl - p.clAtCdMin) * (cl - p.clAtCdMin); };
    const Real w = p.stallWidth;
    if (alpha >= alphaSp) {
        const Real clAtt = std::min(clLin, p.clMax);
        const Real as = std::min(alphaSp, kHalfPi - 0.05);
        const Viterna v(as, p.clMax * (1 - p.postStallClDrop), attachedCd(p.clMax), p.flatPlateCd);
        const Real av = std::max(alpha, as);
        const Real blend = 1 - smoothstep01(alphaSp, alphaSp + w, alpha);
        r.cl = blend * clAtt + (1 - blend) * v.cl(av);
        r.cd = blend * attachedCd(clAtt) + (1 - blend) * v.cd(av);
        r.stalled = alpha > alphaSp + 0.5 * w;
        return r;
    }
    if (alpha <= alphaSm) {
        const Real clAtt = std::max(clLin, p.clMin);
        const Real as = std::min(-alphaSm, kHalfPi - 0.05);
        const Viterna v(as, -p.clMin * (1 - p.postStallClDrop), attachedCd(p.clMin), p.flatPlateCd);
        const Real av = std::max(-alpha, as);
        const Real blend = 1 - smoothstep01(-alphaSm, -alphaSm + w, -alpha);
        r.cl = blend * clAtt - (1 - blend) * v.cl(av);
        r.cd = blend * attachedCd(clAtt) + (1 - blend) * v.cd(av);
        r.stalled = alpha < alphaSm - 0.5 * w;
        return r;
    }
    r.cl = clLin;
    r.cd = attachedCd(clLin);
    return r;
}

Real liftCurveSlope3D(Real ar, Real a2d) {
    const Real kappa = a2d / (2 * kPi);
    return 2 * kPi * ar / (2 + std::sqrt((ar / kappa) * (ar / kappa) + 4));
}

Real groundEffectFactor(Real h, Real b) {
    if (b <= 0 || h <= 0) return h <= 0 ? 0.05 : 1.0;
    const Real x = (16 * h / b) * (16 * h / b);
    return std::max(x / (1 + x), 0.05);
}

}  // namespace ord
