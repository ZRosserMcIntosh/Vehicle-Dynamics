// OpenRoad Dynamics — engine-independent vehicle physics core.
// SPDX-License-Identifier: MIT
#include "ord/tire_generator.hpp"

#include <cctype>
#include <cmath>
#include <string>

namespace ord {

std::optional<TireSpec> parseTireSize(std::string_view s) {
    TireSpec t;
    std::string digits;
    Real vals[3] = {0, 0, 0};
    int n = 0;
    for (std::size_t i = 0; i <= s.size() && n < 3; ++i) {
        const char c = i < s.size() ? s[i] : '\0';
        if (std::isdigit(static_cast<unsigned char>(c)) || c == '.') {
            digits.push_back(c);
        } else if (!digits.empty()) {
            vals[n++] = std::stod(digits);
            digits.clear();
        }
    }
    if (n != 3 || vals[0] < 100 || vals[0] > 400 || vals[1] < 20 || vals[1] > 90 || vals[2] < 10 || vals[2] > 24)
        return std::nullopt;
    t.sectionWidthMm = vals[0];
    t.aspectRatio = vals[1];
    t.rimDiameterIn = vals[2];
    return t;
}

TireParams generateTireParams(const TireSpec& spec) {
    struct Cat { Real mu, loadSens, stiff, shapeY, curvY, rr; };
    // Peak dry-asphalt friction, load sensitivity, cornering-stiffness factor,
    // MF shape/curvature and rolling resistance per category (typical ranges).
    static const Cat cats[] = {
        {0.95, -0.08, 0.90, 1.30, -0.40, 0.008},  // Eco
        {1.00, -0.08, 1.00, 1.30, -0.40, 0.010},  // Touring
        {1.10, -0.09, 1.10, 1.35, -0.35, 0.012},  // Performance
        {1.18, -0.10, 1.15, 1.38, -0.30, 0.013},  // Ultra high performance
        {1.30, -0.12, 1.25, 1.42, -0.15, 0.014},  // Semi-slick (track day)
        {1.55, -0.15, 1.40, 1.45, 0.00, 0.015},   // Slick (racing; dry, warm)
        {0.90, -0.07, 0.80, 1.25, -0.50, 0.015},  // All-terrain
    };
    const Cat& c = cats[static_cast<int>(spec.category)];
    const Real w = spec.sectionWidthMm / 1000.0;
    const Real ar = spec.aspectRatio;
    const Real widthF = spec.sectionWidthMm / 225.0;
    const Real profileF = 55.0 / ar;  // lower profile -> stiffer sidewall

    TireParams p;
    const Real unloaded = spec.rimDiameterIn * 0.0254 / 2 + w * ar / 100.0;
    p.verticalStiffness = 250000.0 * std::pow(widthF, 0.7) * std::pow(profileF, 0.5);
    p.nominalLoad = spec.nominalLoad;
    const Real deflection = spec.nominalLoad / p.verticalStiffness;
    p.radius = unloaded - deflection;  // loaded rolling radius (approximation)
    p.muX = c.mu * 1.03;
    p.muY = c.mu;
    p.muXLoadSensitivity = c.loadSens;
    p.muYLoadSensitivity = c.loadSens - 0.01;
    p.shapeY = c.shapeY;
    p.curvatureY = c.curvY;
    p.cornerStiffness = 16.0 * std::pow(widthF, 0.4) * std::pow(profileF, 0.35) * c.stiff;
    p.longStiffnessPerLoad = 18.0 * std::pow(profileF, 0.2) * c.stiff;
    p.relaxationLengthY = 1.8 * w * std::sqrt(ar / 55.0);
    p.relaxationLengthX = 0.4 * p.relaxationLengthY;
    p.rollingResistance = c.rr;
    p.pneumaticTrail = 0.15 * std::sqrt(2 * unloaded * deflection);
    p.patchHalfWidth = 0.4 * w;
    p.treadDepth = spec.category == TireCategory::Slick ? 0.0 : 0.008;
    p.patchHalfLength = std::min(0.5 * p.radius, std::sqrt(2 * unloaded * deflection));
    const Real mass = 18.0 + 0.9 * (spec.rimDiameterIn - 15.0) + 0.03 * (spec.sectionWidthMm - 205.0);
    p.wheelInertia = 0.6 * mass * unloaded * unloaded;
    return p;
}

}  // namespace ord
