// OpenRoad Dynamics — tire model tests.
// SPDX-License-Identifier: MIT
#include "ord/tire.hpp"
#include "ord_test.hpp"

using namespace ord;

TEST_CASE("tire: zero slip gives zero force, forces oppose sliding") {
    MagicFormulaTire t{TireParams{}};
    const SurfaceProperties dry;
    const TireForces f0 = t.evaluate(4000, 0, 0, dry, 20);
    CHECK_NEAR(f0.fx, 0, 1e-9);
    CHECK_NEAR(f0.fy, 0, 1e-9);
    // Wheel spinning faster than the ground -> forward force; positive alpha -> +Y force.
    CHECK(t.evaluate(4000, 0.05, 0, dry, 20).fx > 0);
    CHECK(t.evaluate(4000, -0.05, 0, dry, 20).fx < 0);
    CHECK(t.evaluate(4000, 0, 0.05, dry, 20).fy > 0);
    CHECK(t.evaluate(4000, 0, -0.05, dry, 20).fy < 0);
}

TEST_CASE("tire: small-slip stiffness matches the parameterisation") {
    TireParams p;
    MagicFormulaTire t{p};
    const Real fz = p.nominalLoad;
    const Real kx = t.pureLongitudinal(fz, 1e-4, 1.0) / 1e-4;
    CHECK_NEAR(kx, p.longStiffnessPerLoad * fz, 0.01 * p.longStiffnessPerLoad * fz);
    const Real ky = t.pureLateral(fz, 1e-4, 1.0) / 1e-4;
    const Real expected = p.cornerStiffness * fz * std::sin(2 * std::atan(1.0 / p.cornerStiffnessLoad));
    CHECK_NEAR(ky, expected, 0.01 * expected);
}

TEST_CASE("tire: peak force equals mu * Fz and load sensitivity reduces mu") {
    TireParams p;
    MagicFormulaTire t{p};
    auto peak = [&](Real fz, bool lateral) {
        Real best = 0;
        for (int i = 1; i < 4000; ++i) {
            const Real s = i * 1e-4 * (lateral ? 4.0 : 1.0);
            best = std::max(best, lateral ? t.pureLateral(fz, s, 1.0) : t.pureLongitudinal(fz, s, 1.0));
        }
        return best;
    };
    CHECK_NEAR(peak(p.nominalLoad, false), p.muX * p.nominalLoad, 0.005 * p.muX * p.nominalLoad);
    CHECK_NEAR(peak(p.nominalLoad, true), p.muY * p.nominalLoad, 0.005 * p.muY * p.nominalLoad);
    const Real muLow = peak(0.5 * p.nominalLoad, true) / (0.5 * p.nominalLoad);
    const Real muHigh = peak(2.0 * p.nominalLoad, true) / (2.0 * p.nominalLoad);
    CHECK(muLow > p.muY);   // lightly loaded tires have a higher friction coefficient
    CHECK(muHigh < p.muY);
}

TEST_CASE("tire: combined slip stays inside the friction ellipse and reduces each component") {
    TireParams p;
    MagicFormulaTire t{p};
    const SurfaceProperties dry;
    const Real fz = p.nominalLoad;
    const Real fyPure = t.evaluate(fz, 0, std::tan(0.08), dry, 20).fy;
    const Real fxPure = t.evaluate(fz, 0.08, 0, dry, 20).fx;
    for (Real k : {0.02, 0.05, 0.1, 0.2, 0.5}) {
        for (Real a : {0.02, 0.05, 0.1, 0.2, 0.5}) {
            const TireForces f = t.evaluate(fz, k, std::tan(a), dry, 20);
            const Real e = (f.fx / (f.muX * fz)) * (f.fx / (f.muX * fz)) + (f.fy / (f.muY * fz)) * (f.fy / (f.muY * fz));
            CHECK(e <= 1.02);
        }
    }
    CHECK(std::abs(t.evaluate(fz, 0.1, std::tan(0.08), dry, 20).fy) < std::abs(fyPure));
    CHECK(std::abs(t.evaluate(fz, 0.08, std::tan(0.1), dry, 20).fx) < std::abs(fxPure));
}

TEST_CASE("tire: progressive breakaway - force past the peak falls gradually, never to zero") {
    TireParams p;
    MagicFormulaTire t{p};
    const Real fz = p.nominalLoad;
    const Real at90 = t.pureLateral(fz, 1.5, 1.0);
    CHECK(at90 > 0.6 * p.muY * fz);
    CHECK(at90 < p.muY * fz);
    const Real locked = std::abs(t.pureLongitudinal(fz, -1.0, 1.0));
    CHECK(locked > 0.6 * p.muX * fz);
    CHECK(locked < p.muX * fz);
}

TEST_CASE("tire: surface friction scale and wet falloff reduce grip") {
    MagicFormulaTire t{TireParams{}};
    SurfaceProperties wet;
    wet.frictionScale = 0.7;
    wet.wetSpeedFalloff = 0.005;
    const SurfaceProperties dry;
    CHECK(t.evaluate(4000, 0, 0.2, wet, 10).fy < t.evaluate(4000, 0, 0.2, dry, 10).fy);
    CHECK(t.evaluate(4000, 0, 0.2, wet, 40).fy < t.evaluate(4000, 0, 0.2, wet, 10).fy);
}

TEST_CASE("tire: lateral relaxation converges to steady slip and is stable for large dt") {
    MagicFormulaTire t{TireParams{}};
    TireSlipState s;
    // vx = 20 m/s, vy = -1 m/s -> steady tan(alpha) = 0.05
    for (int i = 0; i < 2000; ++i) t.integrateLateralSlip(s, 20, -1, 1.0 / 360);
    CHECK_NEAR(s.tanAlpha, 0.05, 1e-6);
    TireSlipState s2;
    for (int i = 0; i < 50; ++i) t.integrateLateralSlip(s2, 60, -3, 0.1);  // dt far larger than sigma/v
    CHECK_NEAR(s2.tanAlpha, 0.05, 1e-6);
}

#include "ord/tire_generator.hpp"

TEST_CASE("tire generator: parses sizes and produces plausible, monotonic parameters") {
    const auto s = parseTireSize("245/40R18");
    CHECK(s.has_value());
    CHECK_NEAR(s->sectionWidthMm, 245, 1e-9);
    CHECK_NEAR(s->aspectRatio, 40, 1e-9);
    CHECK_NEAR(s->rimDiameterIn, 18, 1e-9);
    CHECK(!parseTireSize("hello").has_value());
    TireSpec spec = *s;
    const TireParams p = generateTireParams(spec);
    CHECK_RANGE(p.radius, 0.31, 0.34);              // 245/40R18 is ~0.327 m unloaded
    CHECK_RANGE(p.relaxationLengthY, 0.25, 0.6);
    CHECK_RANGE(p.verticalStiffness, 2.0e5, 4.0e5);
    CHECK(p.patchHalfLength < p.radius);
    spec.category = TireCategory::Slick;
    CHECK(generateTireParams(spec).muY > p.muY);
    TireSpec wide = *s;
    wide.sectionWidthMm = 305;
    CHECK(generateTireParams(wide).cornerStiffness > p.cornerStiffness);
    TireSpec eco = *s;
    eco.category = TireCategory::Eco;
    CHECK(generateTireParams(eco).rollingResistance < p.rollingResistance);
    // Generated parameters must be accepted by the vehicle validator and the MF.
    MagicFormulaTire t{p};
    CHECK(t.pureLateral(4000, 0.1, 1.0) > 0);
}

TEST_CASE("tire: camber thrust pushes toward the lean, with the parameterised slope and shared friction limit") {
    TireParams p;
    const SurfaceProperties dry;
    // Default (cars): no camber response at all.
    CHECK_NEAR(MagicFormulaTire{p}.evaluate(4000, 0, 0, dry, 20, 0.2).fy, 0, 1e-12);

    p.camberStiffness = 0.9;
    p.camberTwist = 0.01;
    p.pneumaticTrail = 0;  // isolate the camber twisting torque
    MagicFormulaTire t{p};
    const Real fz = p.nominalLoad;
    // Top of the wheel toward +y -> force toward +y at zero slip angle.
    CHECK(t.evaluate(fz, 0, 0, dry, 20, 0.1).fy > 0);
    CHECK(t.evaluate(fz, 0, 0, dry, 20, -0.1).fy < 0);
    const Real slope = t.evaluate(fz, 0, 0, dry, 20, 1e-5).fy / 1e-5;
    CHECK_NEAR(slope, p.camberStiffness * fz, 0.01 * p.camberStiffness * fz);
    // Twisting torque steers toward the lean.
    CHECK(t.evaluate(fz, 0, 0, dry, 20, 0.2).mz > 0);
    // Camber and slip share one friction limit: |Fy| never exceeds mu Fz.
    const Real D = p.muY * fz;
    Real worst = 0;
    for (int i = -20; i <= 20; ++i)
        for (int j = -16; j <= 16; ++j)
            worst = std::max(worst, std::abs(t.evaluate(fz, 0, 0.02 * i, dry, 20, 0.1 * j).fy) / D);
    CHECK(worst <= 1.0 + 1e-12);
    // Slip opposing the lean cancels camber thrust (no sign flip at zero net).
    CHECK(t.evaluate(fz, 0, -0.05, dry, 20, 0.0).fy < t.evaluate(fz, 0, -0.05, dry, 20, 0.3).fy);
}
