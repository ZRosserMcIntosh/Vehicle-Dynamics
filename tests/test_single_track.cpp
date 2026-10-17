// OpenRoad Dynamics — single-track (two-wheeler) multibody model tests.
// SPDX-License-Identifier: MIT
//
// Validation against the benchmark bicycle of
//   J.P. Meijaard, J.M. Papadopoulos, A. Ruina, A.L. Schwab, "Linearized
//   dynamics equations for the balance and steer of a bicycle: a benchmark
//   and review", Proc. R. Soc. A 463:1955-1982, 2007.
//
// Two kinds of checks, kept apart on purpose:
//   [self-consistency]  the published reference numbers agree with each other
//                       (published matrices -> published critical speeds and
//                       eigenvalues). Says nothing about our code.
//   [independent]       our nonlinear multibody model (rigid bodies, joints,
//                       rolling contact; it never sees the published matrices)
//                       reproduces the published linear dynamics, and behaves
//                       physically in the time domain.
#include <algorithm>
#include <complex>
#include <cstdio>

#include "ord/single_track.hpp"
#include "ord_test.hpp"

using namespace ord;

namespace {

using Cx = std::complex<double>;

// ---------------------------------------------------------------- reference
// Meijaard et al. 2007, eq. (A 1)-(A 3) / section 4 (benchmark values).
struct LinearModel {
    double M[2][2], C1[2][2], K0[2][2], K2[2][2], g;
};
LinearModel published() {
    return {{{80.81722, 2.31941332208709}, {2.31941332208709, 0.29784188199686}},
            {{0, 33.86641391492494}, {-0.85035641456978, 1.68540397397560}},
            {{-80.95, -2.59951685249872}, {-2.59951685249872, -0.80329488458618}},
            {{0, 76.59734589573222}, {0, 2.65431523794604}},
            9.81};
}
constexpr double kPublishedWeaveSpeed = 4.292382536341111;
constexpr double kPublishedCapsizeSpeed = 6.024262015388372;
// v = 0 eigenvalues (+-). The published matrices give 3.13164324790656 and
// 5.53094371765389 (independently re-checked with numpy). A value of
// "3.13164324079320" has been quoted for the first one; it disagrees with the
// matrices from the 9th digit and looks like a transcription slip, so the
// matrix-consistent value is used.
constexpr double kPublishedStaticEig[2] = {3.13164324790656, 5.53094371765393};

// First-order form qdd = Aq q + Aqd qd.
struct StateMatrices {
    double Aq[2][2], Aqd[2][2];
};

void inverse2(const double a[2][2], double r[2][2]) {
    const double d = a[0][0] * a[1][1] - a[0][1] * a[1][0];
    r[0][0] = a[1][1] / d;
    r[1][1] = a[0][0] / d;
    r[0][1] = -a[0][1] / d;
    r[1][0] = -a[1][0] / d;
}
void mul2(const double a[2][2], const double b[2][2], double r[2][2]) {
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j) r[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j];
}

StateMatrices fromLinearModel(const LinearModel& m, double v) {
    double Mi[2][2], K[2][2], C[2][2];
    inverse2(m.M, Mi);
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j) {
            K[i][j] = m.g * m.K0[i][j] + v * v * m.K2[i][j];
            C[i][j] = v * m.C1[i][j];
        }
    StateMatrices s;
    mul2(Mi, K, s.Aq);
    mul2(Mi, C, s.Aqd);
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j) {
            s.Aq[i][j] = -s.Aq[i][j];
            s.Aqd[i][j] = -s.Aqd[i][j];
        }
    return s;
}

StateMatrices fromNonlinear(const SingleTrackDef& def, double v) {
    const SingleTrackLinearization L = linearizeSingleTrack(def, v);
    CHECK(L.ok);
    StateMatrices s;
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j) {
            s.Aq[i][j] = L.Aq[i][j];
            s.Aqd[i][j] = L.Aqd[i][j];
        }
    return s;
}

// det(lambda^2 I - lambda Aqd - Aq) = lambda^4 + c[3] lambda^3 + ... + c[0]
struct Quartic {
    double c[4];
};
Quartic characteristic(const StateMatrices& s) {
    const double a11 = s.Aq[0][0], a12 = s.Aq[0][1], a21 = s.Aq[1][0], a22 = s.Aq[1][1];
    const double b11 = s.Aqd[0][0], b12 = s.Aqd[0][1], b21 = s.Aqd[1][0], b22 = s.Aqd[1][1];
    Quartic q;
    q.c[3] = -(b11 + b22);
    q.c[2] = b11 * b22 - a11 - a22 - b12 * b21;
    q.c[1] = a11 * b22 + a22 * b11 - b12 * a21 - a12 * b21;
    q.c[0] = a11 * a22 - a12 * a21;
    return q;
}

// Durand-Kerner iteration with Newton polishing (4 roots, monic quartic).
void roots(const Quartic& q, Cx out[4]) {
    auto p = [&](Cx x) { return (((x + q.c[3]) * x + q.c[2]) * x + q.c[1]) * x + q.c[0]; };
    auto dp = [&](Cx x) { return ((4.0 * x + 3.0 * q.c[3]) * x + 2.0 * q.c[2]) * x + q.c[1]; };
    const Cx seed(0.4, 0.9);
    Cx z[4] = {Cx(1, 0), seed, seed * seed, seed * seed * seed};
    for (int i = 0; i < 4; ++i) z[i] *= 6.0;
    for (int it = 0; it < 500; ++it) {
        for (int i = 0; i < 4; ++i) {
            Cx den(1, 0);
            for (int j = 0; j < 4; ++j)
                if (j != i) den *= (z[i] - z[j]);
            z[i] -= p(z[i]) / den;
        }
    }
    for (int i = 0; i < 4; ++i) {
        for (int it = 0; it < 5; ++it) {
            const Cx d = dp(z[i]);
            if (std::abs(d) > 1e-12) z[i] -= p(z[i]) / d;
        }
        out[i] = z[i];
    }
}

// Routh-Hurwitz: a monic quartic is stable iff c_i > 0, c3 c2 > c1 and
// H3 = c3 c2 c1 - c1^2 - c3^2 c0 > 0. Weave (Hopf) speed: H3 = 0.
// Capsize speed: c0 = det(-Aq) = 0 (a real eigenvalue through zero).
double hurwitz3(const Quartic& q) {
    return q.c[3] * q.c[2] * q.c[1] - q.c[1] * q.c[1] - q.c[3] * q.c[3] * q.c[0];
}

template <class F>
double bisect(F f, double lo, double hi) {
    double flo = f(lo);
    CHECK(flo * f(hi) < 0);
    for (int i = 0; i < 80 && hi - lo > 1e-14; ++i) {
        const double mid = 0.5 * (lo + hi), fm = f(mid);
        if ((fm < 0) == (flo < 0)) {
            lo = mid;
            flo = fm;
        } else {
            hi = mid;
        }
    }
    return 0.5 * (lo + hi);
}

// Largest |a_i - nearest b| / max(|b|, 1) over the four eigenvalues.
double eigenvalueMismatch(const Cx a[4], const Cx b[4]) {
    double worst = 0;
    for (int i = 0; i < 4; ++i) {
        double best = 1e300;
        for (int j = 0; j < 4; ++j) best = std::min(best, std::abs(a[i] - b[j]) / std::max(std::abs(b[j]), 1.0));
        worst = std::max(worst, best);
    }
    return worst;
}

double maxRealPart(const Cx e[4]) {
    double m = -1e300;
    for (int i = 0; i < 4; ++i) m = std::max(m, e[i].real());
    return m;
}

void sortByReal(Cx e[4]) {
    std::sort(e, e + 4, [](Cx a, Cx b) { return a.real() != b.real() ? a.real() < b.real() : a.imag() < b.imag(); });
}

}  // namespace

// ------------------------------------------------------------------ tests

TEST_CASE("single_track: [self-consistency] published linear model reproduces its own critical speeds") {
    // Only checks that the reference numbers we compare against are mutually
    // consistent (and that our quartic/Hurwitz tooling is right).
    const LinearModel m = published();
    Cx e[4];
    roots(characteristic(fromLinearModel(m, 0.0)), e);
    sortByReal(e);
    CHECK_NEAR(e[0].real(), -kPublishedStaticEig[1], 1e-11);
    CHECK_NEAR(e[1].real(), -kPublishedStaticEig[0], 1e-11);
    CHECK_NEAR(e[2].real(), kPublishedStaticEig[0], 1e-11);
    CHECK_NEAR(e[3].real(), kPublishedStaticEig[1], 1e-11);
    for (const Cx& z : e) CHECK_NEAR(z.imag(), 0.0, 1e-11);

    const double vw = bisect([&](double v) { return hurwitz3(characteristic(fromLinearModel(m, v))); }, 3.5, 5.0);
    const double vc = bisect([&](double v) { return characteristic(fromLinearModel(m, v)).c[0]; }, 5.5, 7.0);
    CHECK_NEAR(vw, kPublishedWeaveSpeed, 1e-10);
    CHECK_NEAR(vc, kPublishedCapsizeSpeed, 1e-10);
    // Stable (all real parts < 0) strictly between them, unstable just outside.
    for (double v : {vw + 0.01, 5.0, vc - 0.01}) {
        roots(characteristic(fromLinearModel(m, v)), e);
        CHECK(maxRealPart(e) < 0);
    }
    for (double v : {vw - 0.01, vc + 0.01}) {
        roots(characteristic(fromLinearModel(m, v)), e);
        CHECK(maxRealPart(e) > 0);
    }
}

TEST_CASE("single_track: [independent] nonlinear model linearises to the published M, C1, K0, K2") {
    // Extract the coefficient matrices from the nonlinear multibody model:
    //   unit lean/steer torque responses -> M^-1,  Aq(0) -> K0,
    //   Aq(v) - Aq(0) -> K2,  Aqd(v) -> C1.
    const SingleTrackDef def = meijaardBenchmark();
    const LinearModel ref = published();
    const double v = 5.0;
    const SingleTrackLinearization L0 = linearizeSingleTrack(def, 0.0), Lv = linearizeSingleTrack(def, v);
    CHECK(L0.ok && Lv.ok);
    double M[2][2], K0[2][2], K2[2][2], C1[2][2], dA[2][2];
    inverse2(L0.Minv, M);
    mul2(M, L0.Aq, K0);
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j) dA[i][j] = Lv.Aq[i][j] - L0.Aq[i][j];
    mul2(M, dA, K2);
    mul2(M, Lv.Aqd, C1);
    double worst = 0;
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j) {
            const double k0 = -K0[i][j] / def.gravity, k2 = -K2[i][j] / (v * v), c1 = -C1[i][j] / v;
            // Absolute error scaled by each matrix's largest entry.
            worst = std::max(worst, std::abs(M[i][j] - ref.M[i][j]) / 80.81722);
            worst = std::max(worst, std::abs(k0 - ref.K0[i][j]) / 80.95);
            worst = std::max(worst, std::abs(k2 - ref.K2[i][j]) / 76.59734589573222);
            worst = std::max(worst, std::abs(c1 - ref.C1[i][j]) / 33.86641391492494);
            CHECK_NEAR(M[i][j], ref.M[i][j], 1e-9 * 80.81722);
            CHECK_NEAR(k0, ref.K0[i][j], 1e-9 * 80.95);
            CHECK_NEAR(k2, ref.K2[i][j], 1e-9 * 76.59734589573222);
            CHECK_NEAR(c1, ref.C1[i][j], 1e-9 * 33.86641391492494);
        }
    std::printf("    matrices: worst entry error %.2e (relative to each matrix's largest entry)\n", worst);
    // No velocity dependence at standstill.
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j) CHECK_NEAR(L0.Aqd[i][j], 0.0, 1e-9);
}

TEST_CASE("single_track: [independent] eigenvalues of the nonlinear model match the benchmark at 0-8 m/s") {
    const SingleTrackDef def = meijaardBenchmark();
    const LinearModel ref = published();
    for (double v : {0.0, 2.0, 4.0, 5.0, 6.0, 8.0}) {
        Cx mine[4], theirs[4];
        roots(characteristic(fromNonlinear(def, v)), mine);
        roots(characteristic(fromLinearModel(ref, v)), theirs);
        sortByReal(mine);
        sortByReal(theirs);
        const double err = eigenvalueMismatch(mine, theirs);
        std::printf("    v=%.0f  ord:", v);
        for (const Cx& z : mine) std::printf(" %+.9f%+.9fi", z.real(), z.imag());
        std::printf("  (rel err %.1e)\n", err);
        CHECK(err < 1e-8);
    }
    // At v = 0 the eigenvalues are the published static ones.
    Cx e[4];
    roots(characteristic(fromNonlinear(def, 0.0)), e);
    sortByReal(e);
    CHECK_NEAR(e[2].real(), kPublishedStaticEig[0], 1e-8);
    CHECK_NEAR(e[3].real(), kPublishedStaticEig[1], 1e-8);
}

TEST_CASE("single_track: [independent] weave and capsize speeds of the nonlinear model match the benchmark") {
    const SingleTrackDef def = meijaardBenchmark();
    const double vw = bisect([&](double v) { return hurwitz3(characteristic(fromNonlinear(def, v))); }, 3.5, 5.0);
    const double vc = bisect([&](double v) { return characteristic(fromNonlinear(def, v)).c[0]; }, 5.5, 7.0);
    std::printf("    weave   %.12f m/s (published %.12f, error %.1e)\n", vw, kPublishedWeaveSpeed,
                vw - kPublishedWeaveSpeed);
    std::printf("    capsize %.12f m/s (published %.12f, error %.1e)\n", vc, kPublishedCapsizeSpeed,
                vc - kPublishedCapsizeSpeed);
    // Requirement was 1e-3 m/s; the model reaches finite-difference precision.
    CHECK_NEAR(vw, kPublishedWeaveSpeed, 1e-8);
    CHECK_NEAR(vc, kPublishedCapsizeSpeed, 1e-8);
}

TEST_CASE("single_track: [independent] nonlinear free response is self-stable at 5 m/s and falls at 3 m/s") {
    const SingleTrackDef def = meijaardBenchmark();
    // RK4: 1e-3 s gives the same lean history to ~1e-6 rad; used in Debug for speed.
    const double dt = ordtest::kOptimisedBuild ? 5e-4 : 1e-3;
    const double T = ordtest::kOptimisedBuild ? 10.0 : 4.0;
    {
        // Inside the self-stable band (4.29..6.02 m/s): a 0.05 rad lean decays.
        SingleTrack bike(def);
        CHECK(bike.reset(5.0, 0.05));
        double peak = 0;
        for (int i = 0; i < int(T / dt + 0.5); ++i) {
            bike.step(dt);
            peak = std::max(peak, std::abs(bike.leanAngle()));
        }
        CHECK(!bike.faulted());
        CHECK(peak < 0.1);  // initial transient overshoots to ~0.075 rad, then decays
        if (ordtest::kOptimisedBuild)
            CHECK(std::abs(bike.leanAngle()) < 0.1 * 0.05);
        else
            CHECK(std::abs(bike.leanAngle()) < 0.5 * 0.05);
        CHECK(std::abs(bike.steerAngle()) < 0.01);
        CHECK_NEAR(bike.forwardSpeed(), 5.0, 0.01);
        CHECK(bike.constraintError() < 1e-9);
    }
    {
        // Below the weave speed the oscillating weave grows until the bicycle
        // falls (the run stops at 1 rad: there is no frame-ground contact).
        // Debug builds only check that the lean has grown 5x by 2.5 s.
        SingleTrack bike(def);
        CHECK(bike.reset(3.0, 0.02));
        double fellAt = -1, peak = 0;
        const double limit = ordtest::kOptimisedBuild ? 10.0 : 2.5;
        for (int i = 0; i < int(limit / dt + 0.5); ++i) {
            bike.step(dt);
            peak = std::max(peak, std::abs(bike.leanAngle()));
            if (peak > 1.0) {
                fellAt = bike.time();
                break;
            }
        }
        CHECK(!bike.faulted());
        CHECK(peak > 5 * 0.02);
        if (ordtest::kOptimisedBuild) CHECK_RANGE(fellAt, 1.0, 10.0);
    }
}

TEST_CASE("single_track: [independent] NoSlip energy is conserved without drive or damping") {
    // The rolling constraints do no work, so kinetic + potential energy is a
    // first integral. RK4 + projection keeps it to ~1e-11 J over 10 s
    // (E ~ 2000 J); the bound leaves margin for other compilers/platforms.
    const SingleTrackDef def = meijaardBenchmark();
    const double dt = ordtest::kOptimisedBuild ? 5e-4 : 1e-3, T = ordtest::kOptimisedBuild ? 10.0 : 2.0;
    for (double v : {5.0, 4.0}) {
        SingleTrack bike(def);
        CHECK(bike.reset(v, 0.05, 0.0, 0.2, 0.0));
        const double e0 = bike.totalEnergy();
        double worst = 0;
        for (int i = 0; i < int(T / dt + 0.5); ++i) {
            bike.step(dt);
            worst = std::max(worst, std::abs(bike.totalEnergy() - e0));
        }
        std::printf("    v=%.0f m/s: max |E - E0| = %.1e J over %.0f s (E0 = %.1f J)\n", v, worst, T, e0);
        CHECK(!bike.faulted());
        CHECK(worst < 1e-7);  // J
        CHECK(worst / e0 < 1e-10);
    }
}

TEST_CASE("single_track: [independent] steer torque to the right makes the bicycle lean left first (counter-steer)") {
    const SingleTrackDef def = meijaardBenchmark();
    SingleTrack bike(def);
    CHECK(bike.reset(5.0));
    SingleTrackInput in;
    in.steerTorque = 0.5;
    const double dt = 5e-4;
    for (int i = 0; i < int(0.3 / dt); ++i) bike.step(dt, in);
    CHECK(bike.steerAngle() > 0.0);
    CHECK(bike.leanAngle() < -0.002);
    // Rear drive torque accelerates the bicycle.
    SingleTrack driven(def);
    CHECK(driven.reset(5.0));
    SingleTrackInput drive;
    drive.driveTorque = 10.0;
    for (int i = 0; i < int(1.0 / dt); ++i) driven.step(dt, drive);
    CHECK(driven.forwardSpeed() > 5.1);
    CHECK(std::abs(driven.leanAngle()) < 1e-6);
}

TEST_CASE("single_track: [independent] Tyre mode converges toward NoSlip as the tyres stiffen") {
    // With tyre slip as the only lateral-force mechanism (no camber thrust
    // yet), stiffer tyres make the slip velocity -> 0, i.e. the Whipple
    // constraint. Compare lean histories from the same initial condition.
    const double dt = 2e-4, T = ordtest::kOptimisedBuild ? 4.0 : 0.5;
    const SingleTrackDef base = meijaardBenchmark();
    const int n = int(T / dt + 0.5);
    double ref[20001];
    {
        SingleTrack ns(base);
        CHECK(ns.reset(5.0, 0.05));
        for (int i = 0; i < n; ++i) {
            ns.step(dt);
            ref[i] = ns.leanAngle();
        }
    }
    auto deviation = [&](double stiffness, bool& stable) {
        SingleTrackDef d = base;
        for (TireParams* t : {&d.rearTyre, &d.frontTyre}) {
            t->cornerStiffness = stiffness;
            t->longStiffnessPerLoad = stiffness;
            t->relaxationLengthX = t->relaxationLengthY = 0.02;
            t->verticalStiffness = 1e6;
            t->verticalDamping = 1500;
            t->rollingResistance = 0;
        }
        SingleTrack ty(d, SingleTrackContact::Tyre);
        CHECK(ty.reset(5.0, 0.05));
        double dev = 0, peak = 0;
        for (int i = 0; i < n; ++i) {
            ty.step(dt);
            dev = std::max(dev, std::abs(ty.leanAngle() - ref[i]));
            peak = std::max(peak, std::abs(ty.leanAngle()));
        }
        stable = !ty.faulted() && peak < 0.1;
        return dev;
    };
    bool stableSoft = false, stableStiff = false;
    const double soft = deviation(15.0, stableSoft), stiff = deviation(80.0, stableStiff);
    std::printf("    max lean deviation from NoSlip: Ky=15 Fz/rad %.2e rad, Ky=80 Fz/rad %.2e rad\n", soft, stiff);
    CHECK(stableSoft && stableStiff);
    CHECK(stiff < soft);
    CHECK(stiff < 0.03 * 0.05);  // within 3% of the initial lean
}

TEST_CASE("single_track: Tyre mode with default bicycle tyres is stable, carries the weight, rolls down") {
    // Default (realistic, slip-only) bicycle tyres at 5 m/s: the lean
    // perturbation decays, the loads carry the weight, and the bicycle slows
    // at the rate set by rolling resistance: a = Crr * W / m_eff, where
    // m_eff = m + I_R/r_R^2 + I_F/r_F^2 includes the wheels' spin inertia.
    const SingleTrackDef def = meijaardBenchmark();
    SingleTrack bike(def, SingleTrackContact::Tyre);
    CHECK(bike.reset(5.0, 0.05));
    const double dt = 5e-4, T = ordtest::kOptimisedBuild ? 6.0 : 2.0;
    double peak = 0;
    for (int i = 0; i < int(T / dt + 0.5); ++i) {
        bike.step(dt);
        peak = std::max(peak, std::abs(bike.leanAngle()));
    }
    CHECK(!bike.faulted());
    double mass = 0;
    for (int i = 0; i < SingleTrack::kBodies; ++i) mass += bike.body(i).mass;
    const double weight = mass * def.gravity;
    CHECK_NEAR(bike.normalLoad(0) + bike.normalLoad(1), weight, 0.01 * weight);
    CHECK(peak < 0.1);
    if (ordtest::kOptimisedBuild) CHECK(std::abs(bike.leanAngle()) < 0.3 * 0.05);
    const double mEff = mass + def.rearWheel.inertiaAxial / (def.rearWheel.radius * def.rearWheel.radius) +
                        def.frontWheel.inertiaAxial / (def.frontWheel.radius * def.frontWheel.radius);
    const double expectedDecel = def.rearTyre.rollingResistance * weight / mEff;  // same Crr front and rear
    const double decel = (5.0 - bike.forwardSpeed()) / T;
    CHECK_NEAR(decel, expectedDecel, 0.1 * expectedDecel);
}
