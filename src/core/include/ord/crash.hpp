// OpenRoad Dynamics — engine-independent vehicle physics core.
// SPDX-License-Identifier: MIT
//
// Crash survivability: what the occupants feel, and how likely that is to
// hurt them.
//
// "Speed has never killed anyone. Suddenly becoming stationary, that's what
// gets you." The speed change of an impact (delta-V) sets how much energy
// the occupant must lose; how *suddenly* it happens (the crash pulse) and
// what the occupant hits on the way decide the injuries. So:
//
// 1. Crash pulse. A game's collision solver changes the car's velocity in one
//    step (an impulse), which says nothing about how fast it happened. Each
//    impact's delta-V is turned into the pulse a crumple zone of given
//    stiffness would produce: a half sine of duration T = pi / omega and
//    area delta-V (peak delta-V * omega / 2), the response of a linear crush
//    spring. The crush depth it needs is delta-V * pi / (2 omega); past the
//    zone's crush limit the structure stiffens (shorter, harsher pulse) and
//    the excess becomes intrusion into the occupant space. Separate stiffness
//    for front, rear and sides: a side has ~0.3 m of crush space instead of
//    ~0.9 m, which is why side impacts hurt at lower delta-V.
//
// 2. Occupants. Each seat holds a torso and a head (x and y in the body
//    frame) moving relative to the seat, stepped at 0.1 ms. The torso meets
//    a seat-belt spring after slack, the airbag, the seat back, the door or
//    the other occupant, and (unbelted) the knee bolster; the head swings on
//    a neck spring and meets the airbag or curtain airbag, or the steering
//    wheel, dash, pillar or window ("hard contact") past the free space.
//    Normal driving accelerations come in too (the occupants sway).
//
// 3. Injury metrics and risk (published curves, used by NHTSA/NCAP):
//    - HIC15: Head Injury Criterion, max over windows <= 15 ms of
//      (t2 - t1) * (mean acceleration in g)^2.5. Limit 700 (FMVSS 208).
//    - Chest 3 ms clip: highest acceleration sustained for 3 ms. Limit 60 g.
//      (Measured over contiguous 3 ms windows; FMVSS 208 counts cumulative
//      time above a level, which reads higher on a spiky trace.)
//    - Head AIS3+ risk: Phi((ln HIC15 - 7.45231) / 0.73998)  (Eppinger et al.,
//      NHTSA, 1999).
//    - Head AIS4+ risk: 1 / (1 + exp(5.02 - 0.00351 HIC)) and chest AIS4+
//      risk: 1 / (1 + exp(5.55 - 0.0693 A3ms)) (NCAP before 2011), joined as
//      1 - (1 - Ph)(1 - Pc): the life-threatening injury risk.
//    - Delta-V fatality cross-check: P = (delta-V / 71 mph)^4 (Joksch 1993),
//      a population statistic, independent of the occupant model.
//
// What this is: a game estimate built from published relations, that ranks
// crashes sensibly (belted vs. unbelted, frontal vs. side, soft vs. rigid,
// 30 vs. 60 km/h). What it is not: a validated crash simulation. The occupant
// is two point masses (torso, head), there is no rollover or vertical
// injury, no neck or leg metrics, the head's mass does not load the torso,
// and structure stiffness is per zone, not
// per vehicle design. Fixed-size buffers only: nothing allocates.
#pragma once

#include <array>
#include <cstdint>

#include "ord/math.hpp"

namespace ord {

// Crush stiffness per zone, as the natural frequency of the crush spring
// (omega = sqrt(k / m), so it does not depend on the mass), and crush depth.
struct CrashStructure {
    Real frontOmega = 32.0, rearOmega = 28.0, sideOmega = 45.0;  // rad/s
    Real frontCrush = 0.9, rearCrush = 0.7, sideCrush = 0.3;     // m
    Real halfWidth = 0.85;                                       // m, cabin half width at the doors
};

struct OccupantDef {
    Vec3 seat{};               // body frame (hip point), m
    Real halfWidth = 0.22;     // m, shoulder half width
    bool belted = true;
    bool frontAirbag = true;
    bool sideAirbag = true;
    bool helmet = false;       // riders: softer head contact
    Real forwardSpace = 0.45;  // m, free travel of the upper body to the wheel / dash
};

enum class ImpactZone { None, Front, Rear, Left, Right };

struct InjuryReport {
    bool valid = false;
    ImpactZone zone = ImpactZone::None;
    Real deltaV = 0;         // m/s, total of the event
    Real vehiclePeakG = 0;   // g, the synthesised pulse at the seat
    Real peakG = 0;          // g, torso resultant
    Real headPeakG = 0;      // g, head resultant
    Real chest3ms = 0;       // g
    Real hic15 = 0;
    Real excursion = 0;      // m, largest torso travel from the seat
    Real headExcursion = 0;  // m, largest head travel
    Real intrusion = 0;      // m, structure pushed into the occupant space
    bool hardContact = false;  // hit the wheel, dash, door or pillar
    Real pHeadAis3 = 0, pHeadAis4 = 0, pChestAis4 = 0;
    Real pLifeThreatening = 0;   // AIS4+, head or chest
    Real pFatalDeltaV = 0;       // Joksch (delta-V only)
    const char* verdict = "";    // "walks away", "bruised", "injured", "seriously injured", "critical"
};

// Risk curves, exposed for tests and tools.
Real hic15(const Real* accelG, int n, Real sampleDt);
Real clip3ms(const Real* accelG, int n, Real sampleDt);
Real headAis3Risk(Real hic);
Real headAis4Risk(Real hic);
Real chestAis4Risk(Real a3ms);
Real jokschFatalityRisk(Real deltaV);  // delta-V in m/s
const char* injuryVerdict(Real pLifeThreatening, Real pHeadAis3);

// One occupant, stepped with the vehicle. Vehicle acceleration in, body
// frame (x forward, y left); impacts add synthesised pulses on top.
class OccupantSim {
public:
    static constexpr Real kSubstep = 1e-4;         // s
    static constexpr Real kSampleDt = 5e-4;        // s, metric history
    static constexpr int kHistory = 600;           // 300 ms of history
    static constexpr int kMaxPulses = 6;

    OccupantSim() = default;
    OccupantSim(const OccupantDef& def, const CrashStructure& s);

    // Adds an impact: `deltaV` (body frame, the car's velocity change) as a
    // crush pulse per axis. Returns the zone it was assigned to.
    ImpactZone impact(const Vec3& deltaV);
    // Advances `dt` with the vehicle's body-frame acceleration (m/s^2).
    void step(Real dt, const Vec3& vehicleAccel);

    // Torso and head travel from the seat (body frame, m), current torso g.
    Vec3 displacement() const { return {u_[0], u_[1], 0}; }
    Vec3 headDisplacement() const { return {hd_[0], hd_[1], 0}; }
    Real currentG() const { return lastG_; }
    // The report of the last finished crash event (valid once it has ended,
    // ~0.3 s after the last pulse); `live()` while it is still running.
    const InjuryReport& report() const { return report_; }
    InjuryReport live() const { return evaluate(); }
    bool eventActive() const { return eventActive_; }
    std::uint32_t eventCount() const { return eventCount_; }
    void reset();

private:
    struct Pulse {
        Real t0 = 0, omega = 0, amp = 0;  // a(t) = amp * sin(omega (t - t0)) for omega (t - t0) in [0, pi]
        int axis = 0;                     // 0 x, 1 y
        bool active = false;
    };
    Real restraintForce(int axis, Real u, Real v) const;  // per unit mass
    Real headForce(int axis, Real h, Real hv, Real u, Real v) const;
    InjuryReport evaluate() const;
    void finishEvent();

    OccupantDef def_;
    CrashStructure s_;
    Real u_[2] = {0, 0}, v_[2] = {0, 0};
    Real hd_[2] = {0, 0}, hv_[2] = {0, 0};  // head
    Real time_ = 0, sampleAcc_ = 0, lastG_ = 0, lastHeadG_ = 0;
    std::array<Pulse, kMaxPulses> pulses_{};
    // event state
    bool eventActive_ = false;
    std::uint32_t eventCount_ = 0;
    Real quiet_ = 0;
    std::array<Real, kHistory> hist_{};   // head g, for HIC
    std::array<Real, kHistory> histC_{};  // torso g, for the 3 ms clip
    int histN_ = 0, histHead_ = 0;
    Real evDeltaV_[2] = {0, 0}, evPeakVehicle_ = 0, evPeak_ = 0, evHeadPeak_ = 0, evExcursion_ = 0, evHeadExc_ = 0;
    Real evIntrusion_ = 0, evHic_ = 0, evClip_ = 0;
    Real intrusionNow_ = 0;  // current reduction of the free space toward the impact
    int intrusionAxisSign_ = 0;
    bool evHard_ = false;
    ImpactZone evZone_ = ImpactZone::None;
    InjuryReport report_;
};

// Standalone: one impact of `deltaV` (body frame) on an occupant, run to the
// end (tests, tools, a rider hitting a wall: pass a structure whose omega is
// the body's own, no crumple zone).
InjuryReport analyseImpact(const Vec3& deltaV, const OccupantDef& occ, const CrashStructure& s = {});

// Presets.
OccupantDef beltedDriver(Real seatY = 0.37);
OccupantDef unbeltedOccupant(Real seatY = 0.37);
OccupantDef motorcycleRider();      // no belt or airbag, helmet
CrashStructure riderBody();         // a person hitting a wall or the ground: no crumple zone

}  // namespace ord
