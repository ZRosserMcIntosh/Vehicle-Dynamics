// OpenRoad Dynamics — crash survivability tests.
// SPDX-License-Identifier: MIT
//
// The occupant model is a game estimate, so these tests pin behaviour that
// must hold for any sensible model (orderings, published formulas, limits),
// plus the reference case it is calibrated to: a belted, airbagged occupant
// in a 56 km/h rigid frontal barrier crash lands in the range NCAP reports
// for modern cars (chest 3 ms ~30-50 g, HIC15 ~100-500).
#include <cstdio>
#include <string>
#include <vector>

#include "ord/crash.hpp"
#include "ord_test.hpp"

using namespace ord;

namespace {
constexpr double kRebound = 1.1;  // rigid barrier: delta-V includes ~10% rebound
Vec3 frontal(double kmh) { return {-kmh / 3.6 * kRebound, 0, 0}; }
void show(const char* name, const InjuryReport& r) {
    std::printf("    %-30s dV %5.1f km/h | chest %5.1f g (3 ms) | head %5.1f g, HIC15 %6.0f | AIS4+ %5.1f%% | %s\n", name,
                r.deltaV * 3.6, r.chest3ms, r.headPeakG, r.hic15, 100 * r.pLifeThreatening, r.verdict);
}
}  // namespace

TEST_CASE("crash: published formulas: HIC15 of a square pulse, 3 ms clip, risk curves, Joksch") {
    // Constant 60 g for 10 ms: HIC = 0.010 * 60^2.5 (any window <= 15 ms
    // covering the pulse gives the same value; longer windows dilute it).
    std::vector<double> a(200, 0.0);
    for (int i = 40; i < 60; ++i) a[static_cast<std::size_t>(i)] = 60.0;  // 0.5 ms samples
    CHECK_NEAR(hic15(a.data(), 200, 5e-4), 0.010 * std::pow(60.0, 2.5), 1.0);
    CHECK_NEAR(clip3ms(a.data(), 200, 5e-4), 60.0, 1e-9);
    a[45] = a[50] = a[55] = 20.0;  // brief dips: no 3 ms stretch stays at 60 g any more
    CHECK(clip3ms(a.data(), 200, 5e-4) < 60.0);
    // NHTSA head AIS3+ (lognormal): 50% at HIC = e^7.45231 ~ 1722; the FMVSS limit 700 is ~11%.
    CHECK_NEAR(headAis3Risk(std::exp(7.45231)), 0.5, 1e-9);
    CHECK_RANGE(headAis3Risk(700), 0.10, 0.13);
    CHECK_NEAR(chestAis4Risk(5.55 / 0.0693), 0.5, 1e-9);
    CHECK_NEAR(headAis4Risk(5.02 / 0.00351), 0.5, 1e-9);
    CHECK_NEAR(jokschFatalityRisk(71 / 2.23694), 1.0, 1e-6);
    CHECK_NEAR(jokschFatalityRisk(35.5 / 2.23694), 1.0 / 16, 1e-6);
}

TEST_CASE("crash: 56 km/h belted frontal lands in the NCAP range; risk rises steeply with speed") {
    const InjuryReport ncap = analyseImpact(frontal(56), beltedDriver());
    show("56 km/h frontal, belted", ncap);
    CHECK(ncap.valid && ncap.zone == ImpactZone::Front);
    CHECK_RANGE(ncap.chest3ms, 30.0, 50.0);
    CHECK_RANGE(ncap.hic15, 100.0, 500.0);
    CHECK(!ncap.hardContact);
    CHECK(ncap.pLifeThreatening < 0.15);
    double prev = -1;
    for (double kmh : {10.0, 30.0, 56.0, 80.0, 110.0}) {
        const InjuryReport r = analyseImpact(frontal(kmh), beltedDriver());
        char name[64];
        std::snprintf(name, sizeof name, "%3.0f km/h frontal, belted", kmh);
        show(name, r);
        CHECK(r.pLifeThreatening > prev);
        prev = r.pLifeThreatening;
        if (kmh <= 10) CHECK(std::string(r.verdict) == "walks away");
        if (kmh >= 110) CHECK(std::string(r.verdict) == "critical");
    }
}

TEST_CASE("crash: it's the sudden stop: same delta-V, harsher structure, worse injuries") {
    double prevChest = 0, prevHic = 0;
    for (double omega : {20.0, 32.0, 60.0}) {
        CrashStructure s;
        s.frontOmega = omega;
        s.frontCrush = 5.0;  // no bottoming out: only the pulse shape changes
        const InjuryReport r = analyseImpact(frontal(40), beltedDriver(), s);
        std::printf("    40 km/h, crush pulse %3.0f ms: chest %5.1f g, HIC15 %5.0f\n", 1000 * kPi / omega, r.chest3ms, r.hic15);
        CHECK(r.chest3ms > prevChest && r.hic15 > prevHic);
        prevChest = r.chest3ms;
        prevHic = r.hic15;
    }
}

TEST_CASE("crash: belts, airbags and the side of the car matter") {
    const InjuryReport belted = analyseImpact(frontal(56), beltedDriver());
    OccupantDef bare = unbeltedOccupant();
    bare.frontAirbag = false;
    const InjuryReport unrestrained = analyseImpact(frontal(56), bare);
    show("56 km/h, unbelted, no airbag", unrestrained);
    CHECK(unrestrained.hardContact);
    CHECK(unrestrained.hic15 > 4 * belted.hic15);
    CHECK(unrestrained.pLifeThreatening > 0.5);

    // Side impact at 30 km/h on the driver's (left) side vs. the other side.
    const OccupantDef driver = beltedDriver(0.37);
    OccupantDef noCurtain = driver;
    noCurtain.sideAirbag = false;
    const InjuryReport nearSide = analyseImpact({0, -30 / 3.6, 0}, driver);
    const InjuryReport nearBare = analyseImpact({0, -30 / 3.6, 0}, noCurtain);
    const InjuryReport farSide = analyseImpact({0, 30 / 3.6, 0}, driver);
    show("30 km/h side, near side", nearSide);
    show("30 km/h side, near, no airbags", nearBare);
    show("30 km/h side, far side", farSide);
    CHECK(nearSide.zone == ImpactZone::Left && farSide.zone == ImpactZone::Right);
    CHECK(nearBare.hic15 > 3 * nearSide.hic15);
    CHECK(farSide.pLifeThreatening < nearSide.pLifeThreatening);
    // A side has a third of the crush space: same delta-V, harsher than a frontal.
    const InjuryReport front30 = analyseImpact(frontal(30 / kRebound), driver);
    CHECK(nearSide.vehiclePeakG > front30.vehiclePeakG);
    // Past the side's crush depth the door comes in.
    CHECK(analyseImpact({0, -60 / 3.6, 0}, driver).intrusion > 0.05);
}

TEST_CASE("crash: riders: a helmeted rider into a wall at 30 vs 60 km/h") {
    const InjuryReport r30 = analyseImpact({-30 / 3.6, 0, 0}, motorcycleRider(), riderBody());
    const InjuryReport r60 = analyseImpact({-60 / 3.6, 0, 0}, motorcycleRider(), riderBody());
    show("rider into a wall, 30 km/h", r30);
    show("rider into a wall, 60 km/h", r60);
    CHECK(r60.pLifeThreatening > 0.9);
    CHECK(r30.pLifeThreatening < r60.pLifeThreatening);
    // The same 60 km/h in a car's crumple zone, belted, is far more survivable.
    CHECK(analyseImpact({-60 / 3.6, 0, 0}, beltedDriver()).pLifeThreatening < 0.5 * r60.pLifeThreatening);
}

TEST_CASE("crash: occupants sway in normal driving without injury events; events report once") {
    OccupantSim occ(beltedDriver(), CrashStructure{});
    // 1 g steady cornering to the left for 2 s: the occupant leans right a few cm (belt + muscles).
    for (int i = 0; i < 720; ++i) occ.step(1.0 / 360, {0, 9.81, 0});
    std::printf("    1 g cornering: occupant lateral travel %.3f m, %.2f g\n", occ.displacement().y, occ.currentG());
    CHECK_RANGE(occ.displacement().y, -0.15, -0.02);
    CHECK_NEAR(occ.currentG(), 1.0, 0.05);
    CHECK(!occ.eventActive() && occ.eventCount() == 0);
    // An impact while cornering: one event, reported after it ends.
    occ.impact({-8, 0, 0});
    CHECK(occ.eventActive());
    for (int i = 0; i < 360; ++i) occ.step(1.0 / 360, {0, 9.81, 0});
    CHECK(!occ.eventActive() && occ.eventCount() == 1);
    CHECK(occ.report().valid && occ.report().zone == ImpactZone::Front);
    CHECK_NEAR(occ.report().deltaV, 8.0, 1e-9);
    // Deterministic.
    const InjuryReport a = analyseImpact({-12, 3, 0}, beltedDriver()), b = analyseImpact({-12, 3, 0}, beltedDriver());
    CHECK(a.hic15 == b.hic15 && a.chest3ms == b.chest3ms);
}
