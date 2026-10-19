// OpenRoad Dynamics — engine-independent vehicle physics core.
// SPDX-License-Identifier: MIT
#include "ord/crash.hpp"

#include <algorithm>

namespace ord {

namespace {

constexpr Real kG = 9.81;

// Restraint natural frequencies (rad/s) and damping ratios, per unit mass.
// Belt and airbag values are chosen so a belted, airbagged occupant in a
// 56 km/h rigid frontal barrier crash sees ~35-45 g chest and a HIC of a few
// hundred, the range NCAP reports for modern cars.
constexpr Real kMuscleW = 8.0, kMuscleZ = 0.8;      // holding on in normal driving
constexpr Real kSeatBackW = 45.0, kSeatBackZ = 0.5;
constexpr Real kBeltW = 30.0, kBeltZ = 0.3, kBeltSlack = 0.03;
constexpr Real kBeltLatW = 12.0;                    // a belt barely holds you sideways
constexpr Real kAirbagW = 26.0, kAirbagZ = 0.5, kAirbagStart = 0.15;
constexpr Real kHardW = 180.0, kHardZ = 0.2;        // wheel, dash, pillar, unpadded door
constexpr Real kHelmetHardW = 110.0;
constexpr Real kSideAirbagW = 70.0;
constexpr Real kConsoleW = 60.0, kConsoleGap = 0.45;
// Head on the neck: ~2.5 Hz, lightly damped; it swings ~0.1-0.2 m past the
// torso in a frontal crash. Head contacts (relative to the seat).
constexpr Real kNeckW = 16.0, kNeckZ = 0.35;
constexpr Real kHeadAirbagW = 34.0, kHeadAirbagZ = 0.5, kHeadAirbagStart = 0.22;
constexpr Real kHeadReach = 0.12;   // the head reaches this much further forward than the torso's free space
constexpr Real kCurtainW = 55.0;    // curtain / side airbag for the head
constexpr Real kKneeW = 30.0, kKneeGap = 0.2;   // knee bolster, unbelted only (the knees carry part of the body)

Real spring(Real w, Real z, Real x, Real v) { return -w * w * x - 2 * z * w * v; }

}  // namespace

// ------------------------------------------------------------- risk curves

Real hic15(const Real* a, int n, Real dt) {
    const int wmax = std::max(1, static_cast<int>(0.015 / dt + 1e-9));
    Real best = 0;
    for (int i = 0; i < n; ++i) {
        Real sum = 0;
        for (int k = 1; k <= wmax && i + k <= n; ++k) {
            sum += std::max(0.0, a[i + k - 1]);
            const Real mean = sum / k;
            best = std::max(best, k * dt * std::pow(mean, 2.5));
        }
    }
    return best;
}

Real clip3ms(const Real* a, int n, Real dt) {
    const int w = std::max(1, static_cast<int>(0.003 / dt + 0.5));
    Real best = 0;
    for (int i = 0; i + w <= n; ++i) {
        Real lo = a[i];
        for (int k = 1; k < w; ++k) lo = std::min(lo, a[i + k]);
        best = std::max(best, lo);
    }
    return best;
}

Real headAis3Risk(Real hic) {
    if (hic <= 1) return 0;
    const Real z = (std::log(hic) - 7.45231) / 0.73998;
    return 0.5 * std::erfc(-z / std::sqrt(2.0));
}
Real headAis4Risk(Real hic) { return hic <= 0 ? 0 : 1.0 / (1.0 + std::exp(5.02 - 0.00351 * hic)); }
Real chestAis4Risk(Real a3ms) { return a3ms <= 0 ? 0 : 1.0 / (1.0 + std::exp(5.55 - 0.0693 * a3ms)); }
Real jokschFatalityRisk(Real deltaV) {
    const Real mph = std::abs(deltaV) * 2.23694;
    return std::min(1.0, std::pow(mph / 71.0, 4));
}
const char* injuryVerdict(Real pLife, Real pHead3) {
    // The logistic AIS4+ curves never reach 0 (a ~1% floor at zero load), so
    // the mildest bands are read from the head AIS3+ risk as well.
    if (pLife < 0.03 && pHead3 < 0.02) return "walks away";
    if (pLife < 0.08) return "bruised";
    if (pLife < 0.25) return "injured";
    if (pLife < 0.6) return "seriously injured";
    return "critical";
}

// -------------------------------------------------------------- occupants

OccupantSim::OccupantSim(const OccupantDef& def, const CrashStructure& s) : def_(def), s_(s) {}

void OccupantSim::reset() {
    const OccupantDef d = def_;
    const CrashStructure s = s_;
    *this = OccupantSim(d, s);
}

ImpactZone OccupantSim::impact(const Vec3& dv) {
    const Real dvx = dv.x, dvy = dv.y;
    if (std::abs(dvx) < 1e-3 && std::abs(dvy) < 1e-3) return ImpactZone::None;
    const ImpactZone zone = std::abs(dvx) >= std::abs(dvy) ? (dvx < 0 ? ImpactZone::Front : ImpactZone::Rear)
                                                           : (dvy > 0 ? ImpactZone::Right : ImpactZone::Left);
    for (int axis = 0; axis < 2; ++axis) {
        const Real d = axis == 0 ? dvx : dvy;
        if (std::abs(d) < 1e-3) continue;
        Real omega = axis == 0 ? (d < 0 ? s_.frontOmega : s_.rearOmega) : s_.sideOmega;
        const Real crushMax = axis == 0 ? (d < 0 ? s_.frontCrush : s_.rearCrush) : s_.sideCrush;
        // Crush the half sine needs; past the crush space the structure goes
        // solid: a harsher pulse, and the excess pushes into the cabin.
        const Real need = std::abs(d) * kPi / (2 * omega);
        Real intrusion = 0;
        if (need > crushMax) {
            omega = std::min(4 * omega, kPi * std::abs(d) / (2 * crushMax));
            intrusion = std::min(0.5, 0.5 * (need - crushMax));
        }
        // free a slot (oldest finished first)
        Pulse* slot = nullptr;
        for (Pulse& p : pulses_)
            if (!p.active) { slot = &p; break; }
        if (!slot) slot = &pulses_[0];
        *slot = {time_, omega, d * omega / 2, axis, true};
        if (intrusion > 0) {
            // Front: the dash and footwell come back; side: the door comes in.
            const int sign = axis == 0 ? +1 : (d > 0 ? -1 : +1);
            if (axis == 0 && d > 0) intrusion = 0;  // rear: the seat back takes it
            if (intrusion > intrusionNow_) {
                intrusionNow_ = intrusion;
                intrusionAxisSign_ = sign * (axis + 1);
            }
            evIntrusion_ = std::max(evIntrusion_, intrusion);
        }
        evDeltaV_[axis] += d;
    }
    if (!eventActive_) {
        eventActive_ = true;
        evZone_ = zone;
    } else if (std::hypot(dvx, dvy) > std::hypot(evDeltaV_[0] - dvx, evDeltaV_[1] - dvy)) {
        evZone_ = zone;
    }
    quiet_ = 0;
    return zone;
}

Real OccupantSim::headForce(int axis, Real h, Real hv, Real u, Real v) const {
    Real a = spring(kNeckW, kNeckZ, h - u, hv - v);
    if (axis == 0) {
        if (h < 0) a += spring(kSeatBackW, kSeatBackZ, h, hv);  // head restraint
        if (def_.frontAirbag && h > kHeadAirbagStart) a += spring(kHeadAirbagW, kHeadAirbagZ, h - kHeadAirbagStart, hv);
        const Real gap = def_.forwardSpace + kHeadReach - (intrusionAxisSign_ == 1 ? 0.5 * intrusionNow_ : 0.0);
        if (h > gap) a += spring(def_.helmet ? kHelmetHardW : kHardW, kHardZ, h - gap, hv);
    } else {
        const Real toLeftDoor = s_.halfWidth - def_.seat.y - 0.1;   // head is narrower than the shoulders
        const Real toRightDoor = s_.halfWidth + def_.seat.y - 0.1;
        for (int side : {+1, -1}) {
            const Real door = side > 0 ? toLeftDoor : toRightDoor;
            if (door > kConsoleGap + 0.2) continue;  // far side: the head swings free
            Real gap = door;
            if (intrusionAxisSign_ == side * 2) gap -= intrusionNow_;
            const Real x = side * h - std::max(gap, -0.2);
            if (x <= 0) continue;
            const Real w = def_.sideAirbag ? kCurtainW : (def_.helmet ? kHelmetHardW : kHardW);
            a += side * spring(w, def_.sideAirbag ? 0.4 : kHardZ, x, side * hv);
        }
    }
    return a;
}

Real OccupantSim::restraintForce(int axis, Real u, Real v) const {
    Real a = spring(kMuscleW, kMuscleZ, u, v);
    if (axis == 0) {
        if (u < 0) a += spring(kSeatBackW, kSeatBackZ, u, v);
        if (def_.belted && u > kBeltSlack) a += spring(kBeltW, kBeltZ, u - kBeltSlack, v);
        if (def_.frontAirbag && u > kAirbagStart) a += spring(kAirbagW, kAirbagZ, u - kAirbagStart, v);
        // Unbelted: nothing slows the body until the knees hit the dash.
        if (!def_.belted && def_.forwardSpace > kKneeGap && u > kKneeGap) a += spring(kKneeW, kHardZ, u - kKneeGap, v);
        const Real gap = def_.forwardSpace - (intrusionAxisSign_ == 1 ? 0.5 * intrusionNow_ : 0.0);
        if (u > gap) a += spring(def_.helmet ? kHelmetHardW : kHardW, kHardZ, u - gap, v);
    } else {
        if (def_.belted) a += -kBeltLatW * kBeltLatW * u;
        // toward +y (left) and -y (right): door on the near side, console or
        // the other occupant on the far side
        const Real toLeftDoor = s_.halfWidth - def_.seat.y - def_.halfWidth;
        const Real toRightDoor = s_.halfWidth + def_.seat.y - def_.halfWidth;
        for (int side : {+1, -1}) {
            const Real door = side > 0 ? toLeftDoor : toRightDoor;
            const bool nearDoor = door < kConsoleGap;
            Real gap = nearDoor ? door : kConsoleGap;
            if (intrusionAxisSign_ == side * 2) gap -= intrusionNow_;
            const Real x = side * u - std::max(gap, -0.2);
            if (x <= 0) continue;
            const Real w = nearDoor ? (def_.sideAirbag ? kSideAirbagW : kHardW) : kConsoleW;
            a += side * spring(w, nearDoor && !def_.sideAirbag ? kHardZ : 0.4, x, side * v);
        }
    }
    return a;
}

void OccupantSim::step(Real dt, const Vec3& vehicleAccel) {
    const int n = std::max(1, static_cast<int>(std::ceil(dt / kSubstep - 1e-9)));
    const Real h = dt / n;
    for (int i = 0; i < n; ++i) {
        // vehicle acceleration at the seat: driving + active crash pulses
        Real av[2] = {vehicleAccel.x, vehicleAccel.y};
        bool anyPulse = false;
        for (Pulse& p : pulses_) {
            if (!p.active) continue;
            const Real ph = p.omega * (time_ - p.t0);
            if (ph >= kPi) {
                p.active = false;
                continue;
            }
            av[p.axis] += p.amp * std::sin(ph);
            anyPulse = true;
        }
        // occupant relative motion (semi-implicit Euler at 0.1 ms; stiffest
        // contact is ~180 rad/s, h * w = 0.018)
        Real aOcc[2], aHead[2];
        for (int ax = 0; ax < 2; ++ax) {
            const Real r = restraintForce(ax, u_[ax], v_[ax]);  // torso absolute accel = restraint
            const Real rh = headForce(ax, hd_[ax], hv_[ax], u_[ax], v_[ax]);
            aOcc[ax] = r;
            aHead[ax] = rh;
            v_[ax] += (r - av[ax]) * h;
            u_[ax] += v_[ax] * h;
            hv_[ax] += (rh - av[ax]) * h;
            hd_[ax] += hv_[ax] * h;
        }
        time_ += h;
        lastG_ = std::hypot(aOcc[0], aOcc[1]) / kG;
        lastHeadG_ = std::hypot(aHead[0], aHead[1]) / kG;
        if (eventActive_) {
            evPeakVehicle_ = std::max(evPeakVehicle_, std::hypot(av[0], av[1]) / kG);
            evPeak_ = std::max(evPeak_, lastG_);
            evHeadPeak_ = std::max(evHeadPeak_, lastHeadG_);
            evExcursion_ = std::max(evExcursion_, std::hypot(u_[0], u_[1]));
            evHeadExc_ = std::max(evHeadExc_, std::hypot(hd_[0], hd_[1]));
            // hard contact?
            const Real gapF = def_.forwardSpace - (intrusionAxisSign_ == 1 ? 0.5 * intrusionNow_ : 0.0);
            if (u_[0] > gapF || hd_[0] > gapF + kHeadReach) evHard_ = true;
            const Real toLeft = s_.halfWidth - def_.seat.y - def_.halfWidth - (intrusionAxisSign_ == 2 ? intrusionNow_ : 0.0);
            const Real toRight = s_.halfWidth + def_.seat.y - def_.halfWidth - (intrusionAxisSign_ == -2 ? intrusionNow_ : 0.0);
            if ((toLeft < kConsoleGap && u_[1] > toLeft && !def_.sideAirbag) ||
                (toRight < kConsoleGap && -u_[1] > toRight && !def_.sideAirbag))
                evHard_ = true;
        } else if (lastG_ > 12.0 || lastHeadG_ > 20.0) {
            eventActive_ = true;  // a hard landing or a bump with no impact
            evZone_ = ImpactZone::None;
            quiet_ = 0;
        }
        // metric history, every 0.5 ms
        sampleAcc_ += h;
        if (sampleAcc_ >= kSampleDt - 1e-12) {
            sampleAcc_ -= kSampleDt;
            hist_[static_cast<std::size_t>(histHead_)] = lastHeadG_;
            histC_[static_cast<std::size_t>(histHead_)] = lastG_;
            histHead_ = (histHead_ + 1) % kHistory;
            histN_ = std::min(histN_ + 1, kHistory);
            if (eventActive_) {
                // HIC over windows ending now, and the 3 ms clip ending now
                const int wmax = static_cast<int>(0.015 / kSampleDt + 1e-9);
                Real sum = 0, lo = 1e30;
                for (int k = 1; k <= wmax && k <= histN_; ++k) {
                    const std::size_t idx = static_cast<std::size_t>((histHead_ - k + kHistory) % kHistory);
                    sum += hist_[idx];
                    evHic_ = std::max(evHic_, k * kSampleDt * std::pow(sum / k, 2.5));
                    if (k <= 6) lo = std::min(lo, histC_[idx]);
                    if (k == 6) evClip_ = std::max(evClip_, lo);
                }
                quiet_ = (anyPulse || lastG_ > 3.0 || lastHeadG_ > 3.0) ? 0 : quiet_ + kSampleDt;
                if (quiet_ > 0.3) finishEvent();
            }
        }
    }
}

InjuryReport OccupantSim::evaluate() const {
    InjuryReport r;
    r.valid = eventActive_ || eventCount_ > 0;
    r.zone = evZone_;
    r.deltaV = std::hypot(evDeltaV_[0], evDeltaV_[1]);
    r.vehiclePeakG = evPeakVehicle_;
    r.peakG = evPeak_;
    r.headPeakG = evHeadPeak_;
    r.chest3ms = evClip_;
    r.hic15 = evHic_;
    r.excursion = evExcursion_;
    r.headExcursion = evHeadExc_;
    r.intrusion = evIntrusion_;
    r.hardContact = evHard_;
    r.pHeadAis3 = headAis3Risk(r.hic15);
    r.pHeadAis4 = headAis4Risk(r.hic15);
    r.pChestAis4 = chestAis4Risk(r.chest3ms);
    r.pLifeThreatening = 1 - (1 - r.pHeadAis4) * (1 - r.pChestAis4);
    r.pFatalDeltaV = jokschFatalityRisk(r.deltaV);
    r.verdict = injuryVerdict(r.pLifeThreatening, r.pHeadAis3);
    return r;
}

void OccupantSim::finishEvent() {
    report_ = evaluate();
    report_.valid = true;
    ++eventCount_;
    eventActive_ = false;
    evDeltaV_[0] = evDeltaV_[1] = 0;
    evPeakVehicle_ = evPeak_ = evHeadPeak_ = evExcursion_ = evHeadExc_ = evIntrusion_ = evHic_ = evClip_ = 0;
    evHard_ = false;
    evZone_ = ImpactZone::None;
    intrusionNow_ = 0;
    intrusionAxisSign_ = 0;
    quiet_ = 0;
}

InjuryReport analyseImpact(const Vec3& deltaV, const OccupantDef& occ, const CrashStructure& s) {
    OccupantSim sim(occ, s);
    sim.step(0.05, {});  // settle
    sim.impact(deltaV);
    for (int i = 0; i < 2000 && (sim.eventActive() || sim.eventCount() == 0); ++i) sim.step(1.0 / 360, {});
    return sim.report();
}

OccupantDef beltedDriver(Real seatY) {
    OccupantDef d;
    d.seat = {0, seatY, 0};
    return d;
}
OccupantDef unbeltedOccupant(Real seatY) {
    OccupantDef d = beltedDriver(seatY);
    d.belted = false;
    return d;
}
OccupantDef motorcycleRider() {
    OccupantDef d;
    d.belted = d.frontAirbag = d.sideAirbag = false;
    d.helmet = true;
    d.forwardSpace = 0.0;  // nothing between the rider and what they hit
    return d;
}
CrashStructure riderBody() {
    CrashStructure s;
    // A body (and helmet) stopping against a wall or the road: a few cm of
    // give, no crumple zone.
    s.frontOmega = s.rearOmega = s.sideOmega = 90.0;
    s.frontCrush = s.rearCrush = s.sideCrush = 0.12;
    s.halfWidth = 0.25;
    return s;
}

}  // namespace ord
