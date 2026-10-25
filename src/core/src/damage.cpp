// OpenRoad Dynamics — engine-independent vehicle physics core.
// SPDX-License-Identifier: MIT
#include "ord/damage.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace ord {

namespace {

constexpr Real kEps = 1e-6;
constexpr Real kMinRestRatio = 0.15;   // a beam crushed to 15% of its length is solid (bottomed out)
constexpr Real kTearRatio = 1.45;      // stretched to 145% it tears
constexpr Real kElasticStrain = 0.01;  // elastic strain at a beam's full (hardened) strength

// The rigid body inside the solver: centre of mass, orientation and inverse inertia (world).
struct SolverBody {
    Vec3 com;
    Quat q;
    Mat3 invI;
    Real invMass = 0;
    Vec3 dP, dTheta;   // accumulated corrections this step (world)
    Vec3 comBody;      // body frame

    Vec3 point(const Vec3& local) const { return com + q.rotate(local - comBody); }
    Real invMassAt(const Vec3& p, const Vec3& n) const {
        const Vec3 rn = cross(p - com, n);
        return invMass + dot(rn, invI * rn);
    }
    // Applies a positional impulse `j` (kg*m) at world point p.
    void apply(const Vec3& p, const Vec3& j) {
        const Vec3 dp = j * invMass;
        const Vec3 dth = invI * cross(p - com, j);
        com += dp;
        dP += dp;
        dTheta += dth;
        const Quat w{0, dth.x, dth.y, dth.z};
        const Quat wq = w * q;
        q = {q.w + 0.5 * wq.w, q.x + 0.5 * wq.x, q.y + 0.5 * wq.y, q.z + 0.5 * wq.z};
        q.normalize();
    }
};


Real unitSmoothstep(Real t) {
    t = clamp(t, 0.0, 1.0);
    return t * t * (3 - 2 * t);
}

}  // namespace

int DamageModel::addNode(const Vec3& p, Real invMass, CageZone z, bool skin) {
    if (nodeCount_ >= kMaxCageNodes) return -1;
    CageNode& n = nodes_[static_cast<std::size_t>(nodeCount_)];
    n = CageNode{};
    n.rest = n.local = p;
    n.invMass = invMass;
    n.zone = z;
    n.skin = skin;
    return nodeCount_++;
}

void DamageModel::addBeam(int a, int b, CageZone z, Real yield, Real hardening) {
    if (a < 0 || b < 0 || a == b || beamCount_ >= kMaxCageBeams) return;
    const CageNode& na = nodes_[static_cast<std::size_t>(a)];
    const CageNode& nb = nodes_[static_cast<std::size_t>(b)];
    if (na.invMass <= 0 && nb.invMass <= 0) return;  // both in the rigid cell: nothing to solve
    CageBeam& bm = beams_[static_cast<std::size_t>(beamCount_++)];
    bm = CageBeam{};
    bm.a = static_cast<std::uint16_t>(a);
    bm.b = static_cast<std::uint16_t>(b);
    bm.length0 = bm.rest = length(na.rest - nb.rest);
    bm.zone = z;
    bm.yield = yield * cfg_.strength;
    bm.hardening = hardening * cfg_.strength;
    // elastic up to ~1% strain at the strength the beam reaches when crushed to bottoming
    const Real fullStrength = bm.yield + bm.hardening * (1 - kMinRestRatio) * bm.length0;
    bm.compliance = kElasticStrain * bm.length0 / std::max(fullStrength, 1.0);
    // damping time beta = 2 zeta sqrt(m alpha): ~30% of critical for an eighth of the car's
    // mass on this beam (the body resting or rocking on the structure)
    bm.damping = 2 * 0.3 * std::sqrt(mass_ / 8.0 * bm.compliance);
    // skin edges on the sides and the ends collide with posts (horizontal, both nodes outside)
    bm.edge = na.skin && nb.skin && std::abs(na.rest.z - nb.rest.z) < 1e-6;
}

void DamageModel::build(const DamageConfig& cfg, const CageBox& box, Real mass, std::size_t wheelCount,
                        const std::array<Vec3, kMaxWheels>& wheelPos, const std::array<Vec3, kMaxWheels>& shoulder,
                        const std::array<Real, kMaxWheels>& cornerLoad, Real ambient) {
    cfg_ = cfg;
    box_ = box;
    mass_ = mass;
    wheelCount_ = std::min(wheelCount, kMaxWheels);
    wheelPos_ = wheelPos;
    cornerLoad_ = cornerLoad;
    nodeCount_ = beamCount_ = 0;
    enabled_ = true;

    const Real L = box.front - box.rear;
    const Real hw = box.halfWidth;
    // The crush zones reach past the axles (the firewall sits ~0.3 m behind the front axle, the
    // rear seat bulkhead ~0.25 m ahead of the rear one), so a hard hit moves the wheels' mounts.
    Real xfAxle = -1e9, xrAxle = 1e9;
    for (std::size_t w = 0; w < wheelCount_; ++w) {
        xfAxle = std::max(xfAxle, wheelPos[w].x);
        xrAxle = std::min(xrAxle, wheelPos[w].x);
    }
    if (wheelCount_ == 0) xfAxle = xrAxle = 0.5 * (box.front + box.rear);
    const Real fc = clamp(std::max(cfg.crash.frontCrush, box.front - xfAxle + 0.3), 0.2, 0.45 * L);
    const Real rc = clamp(std::max(cfg.crash.rearCrush, xrAxle - box.rear + 0.25), 0.2, 0.35 * L);
    const Real sc = clamp(cfg.crash.sideCrush, 0.1, 0.45 * hw);
    xFire_ = box.front - fc;
    xBulk_ = box.rear + rc;
    yInner_ = hw - sc;

    // Stations along x, front to rear: three cells in each crush zone, ~0.6 m cells in the cell.
    std::array<Real, 32> xs{};
    int ns = 0;
    for (int k = 0; k <= 3; ++k) xs[static_cast<std::size_t>(ns++)] = box.front - fc * k / 3.0;
    const int nCell = std::max(2, static_cast<int>(std::lround((xFire_ - xBulk_) / 0.6)));
    for (int k = 1; k <= nCell; ++k) xs[static_cast<std::size_t>(ns++)] = xFire_ - (xFire_ - xBulk_) * k / nCell;
    for (int k = 1; k <= 3; ++k) xs[static_cast<std::size_t>(ns++)] = xBulk_ - rc * k / 3.0;
    const std::array<Real, 4> ys{hw, yInner_, -yInner_, -hw};
    const std::array<Real, 2> zs{box.floor, box.belt};

    // Count the free nodes first (their mass share depends on it).
    auto inCell = [&](Real x) { return x <= xFire_ + 1e-6 && x >= xBulk_ - 1e-6; };
    auto isFree = [&](int s, int c) { return !(inCell(xs[static_cast<std::size_t>(s)]) && (c == 1 || c == 2)); };
    // roof stations: cell stations over the greenhouse
    std::array<int, 32> roofSt{};
    int nRoof = 0;
    for (int s = 0; s < ns; ++s) {
        const Real x = xs[static_cast<std::size_t>(s)];
        if (inCell(x) && x >= box.roofRear - 0.15 && x <= box.roofFront + 0.15) roofSt[static_cast<std::size_t>(nRoof++)] = s;
    }
    if (nRoof < 2) {  // a short greenhouse: the two cell stations nearest its middle
        nRoof = 0;
        const Real mid = 0.5 * (box.roofRear + box.roofFront);
        int best = -1;
        for (int s = 0; s < ns; ++s)
            if (inCell(xs[static_cast<std::size_t>(s)]) &&
                (best < 0 || std::abs(xs[static_cast<std::size_t>(s)] - mid) < std::abs(xs[static_cast<std::size_t>(best)] - mid)))
                best = s;
        const int other = (best + 1 < ns && inCell(xs[static_cast<std::size_t>(best + 1)])) ? best + 1 : best - 1;
        roofSt[0] = std::min(best, other);
        roofSt[1] = std::max(best, other);
        nRoof = 2;
    }
    int nFree = 2 * nRoof;
    for (int s = 0; s < ns; ++s)
        for (int c = 0; c < 4; ++c)
            if (isFree(s, c)) nFree += 2;
    const Real nodeMass = std::max(cfg.nodeMassFraction, 0.005) * mass / std::max(nFree, 1);
    const Real invM = 1.0 / nodeMass;

    std::array<std::array<std::array<int, 2>, 4>, 32> id{};
    for (int s = 0; s < ns; ++s)
        for (int c = 0; c < 4; ++c)
            for (int l = 0; l < 2; ++l) {
                const Real x = xs[static_cast<std::size_t>(s)];
                const bool free = isFree(s, c);
                CageZone z = CageZone::Cell;
                if (x > xFire_ + 1e-6) z = CageZone::Front;
                else if (x < xBulk_ - 1e-6) z = CageZone::Rear;
                else if (free) z = CageZone::Side;
                const bool skin = c == 0 || c == 3 || s == 0 || s == ns - 1;
                id[static_cast<std::size_t>(s)][static_cast<std::size_t>(c)][static_cast<std::size_t>(l)] =
                    addNode({x, ys[static_cast<std::size_t>(c)], zs[static_cast<std::size_t>(l)]}, free ? invM : 0.0, z, skin);
            }
    const Real yRoof = std::min(0.8 * hw, std::max(yInner_, 0.3));
    std::array<std::array<int, 2>, 32> rid{};
    for (int r = 0; r < nRoof; ++r)
        for (int side = 0; side < 2; ++side)
            rid[static_cast<std::size_t>(r)][static_cast<std::size_t>(side)] =
                addNode({xs[static_cast<std::size_t>(roofSt[static_cast<std::size_t>(r)])], side == 0 ? yRoof : -yRoof, box.roof}, invM,
                        CageZone::Roof, true);
    // Tyre shoulders: contact points only (anchored, no beams), gone with their wheel.
    for (std::size_t w = 0; w < wheelCount_; ++w) {
        const int n = addNode(shoulder[w], 0.0, CageZone::Cell, false);
        if (n >= 0) nodes_[static_cast<std::size_t>(n)].wheel = static_cast<std::int8_t>(w);
    }

    // ---- Beam strengths, from the CrashStructure each zone must match ----
    // A zone of crush depth d and natural frequency omega behaves like a spring k = m omega^2: its
    // force reaches k d at full crush. The axial members share that; each yields a little early
    // (20% of its share) and hardens linearly to its share at full crush. Three segments in
    // series per rail, so each segment hardens three times as steeply. Cross members, verticals
    // and diagonals keep the shape: weaker, so the zone folds instead of shearing off.
    const CrashStructure& cs = cfg.crash;
    // kZoneShare: the axial members' part of the zone's force; the diagonals, cross members and
    // verticals carry the rest as the zone folds (measured: test_damage, calibration sweep).
    constexpr Real kZoneShare = 0.5;
    auto zoneBeams = [&](Real omega, Real depth, int axialMembers, int segments, Real& y0, Real& h) {
        const Real share = kZoneShare * mass * omega * omega * depth / std::max(axialMembers, 1);
        y0 = 0.15 * share;
        h = (share - y0) / ((1 - kMinRestRatio) * depth / segments);
    };
    Real fY = 0, fH = 0, rY = 0, rH = 0, sY = 0, sH = 0;
    zoneBeams(cs.frontOmega, fc, 8, 3, fY, fH);
    zoneBeams(cs.rearOmega, rc, 8, 3, rY, rH);
    zoneBeams(cs.sideOmega, sc, 2 * (nCell + 1), 1, sY, sH);
    const Real weak = 0.35;
    auto N = [&](int s, int c, int l) {
        return id[static_cast<std::size_t>(s)][static_cast<std::size_t>(c)][static_cast<std::size_t>(l)];
    };
    auto zoneOf = [&](int s0, int s1) {
        const Real x = 0.5 * (xs[static_cast<std::size_t>(s0)] + xs[static_cast<std::size_t>(s1)]);
        return x > xFire_ + 1e-6 ? CageZone::Front : x < xBulk_ - 1e-6 ? CageZone::Rear : CageZone::Side;
    };
    auto strength = [&](CageZone z, Real& y, Real& h) {
        if (z == CageZone::Front) { y = fY; h = fH; }
        else if (z == CageZone::Rear) { y = rY; h = rH; }
        else { y = sY; h = sH; }
    };
    for (int s = 0; s < ns; ++s)
        for (int c = 0; c < 4; ++c)
            for (int l = 0; l < 2; ++l) {
                Real y = 0, h = 0;
                // rails along x (axial in a frontal or rear crash)
                if (s + 1 < ns) {
                    const CageZone z = zoneOf(s, s + 1);
                    strength(z, y, h);
                    const Real k = z == CageZone::Side ? 0.6 : 1.0;  // door skins / sills along the side
                    addBeam(N(s, c, l), N(s + 1, c, l), z, k * y, k * h);
                    // diagonals in the floor/belt planes (shear across the car) and in the side planes
                    if (c + 1 < 4) {
                        addBeam(N(s, c, l), N(s + 1, c + 1, l), z, weak * y, weak * h);
                        addBeam(N(s, c + 1, l), N(s + 1, c, l), z, weak * y, weak * h);
                    }
                    if (l == 0) addBeam(N(s, c, 0), N(s + 1, c, 1), z, weak * y, weak * h);
                    else addBeam(N(s, c, 1), N(s + 1, c, 0), z, weak * y, weak * h);
                }
                // across the car (axial in a side impact between the skin and the cell wall)
                if (c + 1 < 4) {
                    const Real x = xs[static_cast<std::size_t>(s)];
                    const bool door = inCell(x) && (c == 0 || c == 2);
                    const CageZone z = door ? CageZone::Side : x > xFire_ + 1e-6 ? CageZone::Front : x < xBulk_ - 1e-6 ? CageZone::Rear : CageZone::Side;
                    strength(door ? CageZone::Side : z, y, h);
                    const Real k = door ? (l == 0 ? 1.5 : 1.0) : 0.5;  // sills are the strongest
                    addBeam(N(s, c, l), N(s, c + 1, l), z, k * y, k * h);
                }
            }
    // verticals and the cross-car diagonals in each station's plane
    for (int s = 0; s < ns; ++s) {
        const Real x = xs[static_cast<std::size_t>(s)];
        const CageZone z = x > xFire_ + 1e-6 ? CageZone::Front : x < xBulk_ - 1e-6 ? CageZone::Rear : CageZone::Side;
        Real y = 0, h = 0;
        strength(z, y, h);
        for (int c = 0; c < 4; ++c) addBeam(N(s, c, 0), N(s, c, 1), z, 0.5 * y, 0.5 * h);
        for (int c = 0; c + 1 < 4; ++c) {  // both diagonals, or the lattice leans to one side
            addBeam(N(s, c, 0), N(s, c + 1, 1), z, weak * y, weak * h);
            addBeam(N(s, c, 1), N(s, c + 1, 0), z, weak * y, weak * h);
        }
    }
    // Roof: pillars down to the belt (the roof holds roofStrength x the weight before it
    // yields, FMVSS 216), roof rails and bows across, and diagonals in the side planes.
    {
        const int pillars = 2 * nRoof * 2;
        const Real py = cfg.roofStrength * mass * kGravity / std::max(pillars, 1);
        const Real ph = py / 0.25;  // doubles over 25 cm of crush
        for (int r = 0; r < nRoof; ++r) {
            const int s = roofSt[static_cast<std::size_t>(r)];
            for (int side = 0; side < 2; ++side) {
                const int rn = rid[static_cast<std::size_t>(r)][static_cast<std::size_t>(side)];
                const int cOut = side == 0 ? 0 : 3, cIn = side == 0 ? 1 : 2;
                addBeam(rn, N(s, cOut, 1), CageZone::Roof, py, ph);
                addBeam(rn, N(s, cIn, 1), CageZone::Roof, py, ph);
                if (s > 0) addBeam(rn, N(s - 1, cOut, 1), CageZone::Roof, 0.5 * py, 0.5 * ph);
                if (s + 1 < ns) addBeam(rn, N(s + 1, cOut, 1), CageZone::Roof, 0.5 * py, 0.5 * ph);
                if (r + 1 < nRoof) addBeam(rn, rid[static_cast<std::size_t>(r + 1)][static_cast<std::size_t>(side)], CageZone::Roof, py, ph);
            }
            addBeam(rid[static_cast<std::size_t>(r)][0], rid[static_cast<std::size_t>(r)][1], CageZone::Roof, py, ph);
            if (r + 1 < nRoof) {
                addBeam(rid[static_cast<std::size_t>(r)][0], rid[static_cast<std::size_t>(r + 1)][1], CageZone::Roof, 0.5 * py, 0.5 * ph);
                addBeam(rid[static_cast<std::size_t>(r)][1], rid[static_cast<std::size_t>(r + 1)][0], CageZone::Roof, 0.5 * py, 0.5 * ph);
            }
        }
    }
    // Mirror pairs side by side: Gauss-Seidel sweeps alternate direction, so each left/right
    // pair is solved left-first, then right-first, and a symmetric crash stays symmetric.
    for (int i = 0; i < nodeCount_; ++i) {
        mirror_[static_cast<std::size_t>(i)] = -1;
        const Vec3 r = nodes_[static_cast<std::size_t>(i)].rest;
        for (int j = 0; j < nodeCount_; ++j) {
            const Vec3 q = nodes_[static_cast<std::size_t>(j)].rest;
            if (std::abs(q.x - r.x) < 1e-6 && std::abs(q.z - r.z) < 1e-6 && std::abs(q.y + r.y) < 1e-6) {
                mirror_[static_cast<std::size_t>(i)] = static_cast<std::int16_t>(j);
                break;
            }
        }
    }
    {
        std::vector<CageBeam> sorted(static_cast<std::size_t>(beamCount_));  // build time: the heap is fine
        std::vector<char> used(static_cast<std::size_t>(beamCount_), 0);
        int no = 0;
        for (int i = 0; i < beamCount_; ++i) {
            if (used[static_cast<std::size_t>(i)]) continue;
            const CageBeam& b = beams_[static_cast<std::size_t>(i)];
            used[static_cast<std::size_t>(i)] = 1;
            sorted[static_cast<std::size_t>(no++)] = b;
            const int ma = mirror_[b.a], mb = mirror_[b.b];
            for (int j = i + 1; j < beamCount_; ++j) {
                const CageBeam& c = beams_[static_cast<std::size_t>(j)];
                if (!used[static_cast<std::size_t>(j)] && ((c.a == ma && c.b == mb) || (c.a == mb && c.b == ma))) {
                    used[static_cast<std::size_t>(j)] = 1;
                    sorted[static_cast<std::size_t>(no++)] = c;
                    break;
                }
            }
        }
        for (int i = 0; i < no; ++i) beams_[static_cast<std::size_t>(i)] = sorted[static_cast<std::size_t>(i)];
    }
    {
        int cellFront = 0;
        for (int i = 0; i < nodeCount_; ++i) {
            const CageNode& n = nodes_[static_cast<std::size_t>(i)];
            if (n.invMass <= 0 && n.wheel < 0 && std::abs(n.rest.x - xFire_) < 1e-6) ++cellFront;
        }
        const Real w = 2.5 * cs.frontOmega;
        cellCompliance_ = std::max(cellFront, 1) / (mass * w * w * cfg.strength);
    }
    repair(ambient);
}

void DamageModel::repair(Real ambient) {
    for (int i = 0; i < nodeCount_; ++i) {
        CageNode& n = nodes_[static_cast<std::size_t>(i)];
        n.local = n.rest;
    }
    for (int i = 0; i < beamCount_; ++i) {
        CageBeam& b = beams_[static_cast<std::size_t>(i)];
        b.rest = b.length0;
        b.plastic = 0;
        b.broken = false;
        b.lambda = 0;
    }
    const bool wasAwake = state_.awake;
    const std::uint32_t version = state_.version;
    state_ = DamageState{};
    state_.engineTemperature = std::max(ambient, 85.0);
    state_.awake = wasAwake;
    state_.version = version + 1;
    impactCount_ = 0;
    quietSteps_ = 0;
    cellIntrusion_ = 0;
    shapeChanged_ = true;
}

void DamageModel::snapToBody(const RigidBodyState& body) {
    for (int i = 0; i < nodeCount_; ++i) {
        CageNode& n = nodes_[static_cast<std::size_t>(i)];
        n.x = n.xPrev = body.position + body.orientation.rotate(n.local);
        n.v = body.velocity;  // the next wake sets the proper point velocity
    }
    state_.awake = false;
    quietSteps_ = 0;
}

Vec3 DamageModel::displacementAt(const Vec3& p) const {
    // Gaussian-weighted average of the nodes' displacements near p (anchored nodes count as zero).
    constexpr Real kSigma2 = 0.35 * 0.35;
    Vec3 sum{};
    Real wsum = 0;
    for (int i = 0; i < nodeCount_; ++i) {
        const CageNode& n = nodes_[static_cast<std::size_t>(i)];
        if (n.wheel >= 0) continue;
        const Vec3 d = n.rest - p;
        const Real w = std::exp(-dot(d, d) / kSigma2);
        if (w < 1e-6) continue;
        sum += (n.local - n.rest) * w;
        wsum += w;
    }
    return wsum > 1e-9 ? sum / wsum : Vec3{};
}

bool DamageModel::stepStructure(Real dt, RigidBodyState& body, const RigidBodyState& start, const BodyParams& bp,
                                const GroundQuery& ground, const ObstacleQuery* obstacles, bool groundGate,
                                std::uint64_t& groundQueries) {
    impactCount_ = 0;
    state_.nodeContacts = 0;
    if (!enabled_ || !cfg_.structure || !(dt > 0)) return false;

    bool wheelGone = false;
    for (std::size_t w = 0; w < wheelCount_; ++w) wheelGone = wheelGone || state_.wheels[w].detached;
    const Vec3 comWorld = body.position + body.orientation.rotate(bp.com);
    Real bound = 0;
    for (int i = 0; i < nodeCount_; ++i) bound = std::max(bound, length(nodes_[static_cast<std::size_t>(i)].local - bp.com));
    const Real speed = length(body.velocity) + length(body.angularVelocity) * bound;
    const bool nearObstacle = obstacles && obstacles->anyNear(comWorld, bound + 2.0 * speed * dt + 0.4);
    const bool groundOn = groundGate || wheelGone;
    const bool want = nearObstacle || groundOn;

    auto pointVelocity = [&](const RigidBodyState& b, const Vec3& p) {
        const Vec3 c = b.position + b.orientation.rotate(bp.com);
        return b.velocity + cross(b.angularVelocity, p - c);
    };
    if (!state_.awake) {
        if (!want) return false;
        // wake: the free nodes start where the body carries them, at the body's point velocity
        for (int i = 0; i < nodeCount_; ++i) {
            CageNode& n = nodes_[static_cast<std::size_t>(i)];
            const Vec3 p0 = start.position + start.orientation.rotate(n.local);
            n.x = p0;
            n.v = pointVelocity(start, p0);
        }
        state_.awake = true;
        quietSteps_ = 0;
    }

    // The body enters the solve at the start-of-step pose with the velocity the step's forces
    // gave it (tyres, aero, gravity); the substeps below advance its pose themselves. XPBD
    // converges on a stiff structure through many short substeps rather than many iterations
    // (Macklin et al., "Small steps in physics simulation", 2019).
    SolverBody sb;
    sb.comBody = bp.com;
    sb.com = start.position + start.orientation.rotate(bp.com);
    sb.q = start.orientation;
    sb.invMass = 1.0 / std::max(bp.mass, 1e-6);
    // the step's forces (tyres, aero, gravity) act a share per substep
    Vec3 vBody = start.velocity, wBody = start.angularVelocity;
    Vec3 comPrev = sb.com;
    Quat qPrev = sb.q;

    auto pos = [&](int i) -> Vec3 {
        const CageNode& n = nodes_[static_cast<std::size_t>(i)];
        return n.invMass > 0 ? n.x : sb.point(n.rest);
    };
    auto prevPos = [&](int i) -> Vec3 {  // at the start of the substep
        const CageNode& n = nodes_[static_cast<std::size_t>(i)];
        return n.invMass > 0 ? n.xPrev : comPrev + qPrev.rotate(n.rest - bp.com);
    };
    auto startVel = [&](int i) -> Vec3 {
        const CageNode& n = nodes_[static_cast<std::size_t>(i)];
        const Vec3 p = start.position + start.orientation.rotate(n.local);
        return n.invMass > 0 ? n.v : vBody + cross(wBody, p - sb.com);
    };
    auto wAt = [&](int i, const Vec3& p, const Vec3& dir) {
        const CageNode& n = nodes_[static_cast<std::size_t>(i)];
        return n.invMass > 0 ? n.invMass : sb.invMassAt(p, dir);
    };
    auto move = [&](int i, const Vec3& p, const Vec3& j) {  // positional impulse j at node i
        CageNode& n = nodes_[static_cast<std::size_t>(i)];
        if (n.invMass > 0) n.x += j * n.invMass;
        else sb.apply(p, j);
    };

    // ---- Contacts: detected once per step where the nodes will be, solved as planes ----
    // scratch in the model, not on the stack (WebAssembly and game threads have small stacks)
    auto& contacts = contacts_;
    int nc = 0;
    const Vec3 up = body.orientation.rotate({0, 0, 1});
    const Real margin = 0.02 + speed * dt;
    auto endPos = [&](int i) -> Vec3 {  // where the step's motion takes the node (no corrections)
        const CageNode& n = nodes_[static_cast<std::size_t>(i)];
        if (n.invMass > 0) return n.x + n.v * dt;
        return body.position + body.orientation.rotate(n.rest);
    };
    for (int i = 0; i < nodeCount_ && nc < kMaxCageContacts; ++i) {
        const CageNode& n = nodes_[static_cast<std::size_t>(i)];
        if (n.wheel >= 0 && state_.wheels[static_cast<std::size_t>(n.wheel)].detached) continue;
        const Vec3 p = endPos(i);
        if (groundOn) {
            GroundHit gh = ground.raycast({p.x, p.y, p.z + 1.5}, {0, 0, -1}, 3.0);
            ++groundQueries;
            Real pen = gh.hit ? dot(gh.point - p, gh.normal) : -1.0;
            if (up.z < 0.5) {  // steeply tilted: also along the body's own down axis (see the hull)
                const GroundHit g2 = ground.raycast(p + up * 1.5, -up, 3.0);
                ++groundQueries;
                const Real pen2 = g2.hit ? dot(g2.point - p, g2.normal) : -1.0;
                if (pen2 > pen) { gh = g2; pen = pen2; }
            }
            if (gh.hit && pen > -margin) {
                CageContact& c = contacts[static_cast<std::size_t>(nc++)];
                c = CageContact{};
                c.a = i;
                c.surface = gh.point;
                c.normal = gh.normal;
                c.mu = bp.bodyFriction * gh.surface.frictionScale;
                c.closing = -dot(startVel(i), gh.normal);
            }
        }
        if (nearObstacle && nc < kMaxCageContacts && n.wheel < 0) {
            const ObstacleContact oc = obstacles->point(p, comWorld, margin);
            if (oc.hit) {
                CageContact& c = contacts[static_cast<std::size_t>(nc++)];
                c = CageContact{};
                c.a = i;
                c.surface = p + oc.normal * oc.depth;
                c.normal = oc.normal;
                c.mu = oc.friction;
                c.obstacle = oc.id;
                c.closing = -dot(startVel(i), oc.normal);
                // A cell node on an obstacle: the crush zone in front of it is used up. Past its
                // crush limit the structure stiffens (a zone of 2.5x the frequency, shared by the
                // cell's front or side nodes) and what it gives is intrusion into the cell.
                if (n.invMass <= 0) c.compliance = cellCompliance_;
            }
        }
    }
    if (nearObstacle) {
        for (int k = 0; k < beamCount_ && nc < kMaxCageContacts; ++k) {
            const CageBeam& bm = beams_[static_cast<std::size_t>(k)];
            if (!bm.edge || bm.broken) continue;
            Real t = 0;
            const Vec3 pa = endPos(bm.a), pb = endPos(bm.b);
            const ObstacleContact oc = obstacles->edge(pa, pb, margin, t);
            if (!oc.hit || t <= 0.02 || t >= 0.98) continue;  // at the ends the node contacts have it
            CageContact& c = contacts[static_cast<std::size_t>(nc++)];
            c = CageContact{};
            c.a = bm.a;
            c.b = bm.b;
            c.t = t;
            c.surface = pa * (1 - t) + pb * t + oc.normal * oc.depth;
            c.normal = oc.normal;
            c.mu = oc.friction;
            c.obstacle = oc.id;
            c.closing = -dot(startVel(bm.a) * (1 - t) + startVel(bm.b) * t, oc.normal);
        }
    }

    // contacts in mirror pairs too (see the beams): an order, not a copy
    std::array<std::uint16_t, kMaxCageContacts> order{};
    {
        std::array<bool, kMaxCageContacts> used{};
        int no = 0;
        for (int i = 0; i < nc; ++i) {
            if (used[static_cast<std::size_t>(i)]) continue;
            const CageContact& c = contacts[static_cast<std::size_t>(i)];
            used[static_cast<std::size_t>(i)] = true;
            order[static_cast<std::size_t>(no++)] = static_cast<std::uint16_t>(i);
            const int ma = mirror_[static_cast<std::size_t>(c.a)];
            const int mb = c.b >= 0 ? mirror_[static_cast<std::size_t>(c.b)] : -1;
            for (int j = i + 1; j < nc; ++j) {
                const CageContact& d = contacts[static_cast<std::size_t>(j)];
                if (!used[static_cast<std::size_t>(j)] && d.a == ma && d.b == mb && d.obstacle == c.obstacle) {
                    used[static_cast<std::size_t>(j)] = true;
                    order[static_cast<std::size_t>(no++)] = static_cast<std::uint16_t>(j);
                    break;
                }
            }
        }
    }

    const int S = std::clamp(cfg_.substeps, 1, 64);
    const Real h = dt / S;
    const Real h2 = h * h;
    const Vec3 dvExt = (body.velocity - start.velocity) / S, dwExt = (body.angularVelocity - start.angularVelocity) / S;
    const int iters = std::clamp(cfg_.iterations, 1, 64);
    const Real relDamp = std::exp(-h * 150.0);
    Real maxRel = 0;
    for (int sub = 0; sub < S; ++sub) {
        // ---- predict: body and free nodes move with their velocities ----
        comPrev = sb.com;
        qPrev = sb.q;
        vBody += dvExt;
        wBody += dwExt;
        sb.com += vBody * h;
        {
            const Quat wq = Quat{0, wBody.x, wBody.y, wBody.z} * sb.q;
            sb.q = {sb.q.w + 0.5 * h * wq.w, sb.q.x + 0.5 * h * wq.x, sb.q.y + 0.5 * h * wq.y, sb.q.z + 0.5 * h * wq.z};
            sb.q.normalize();
            const Mat3 R = sb.q.toMatrix();
            sb.invI = R * Mat3::diagonal(bp.invInertia) * R.transposed();
        }
        sb.dP = sb.dTheta = {};
        for (int i = 0; i < nodeCount_; ++i) {
            CageNode& n = nodes_[static_cast<std::size_t>(i)];
            if (n.invMass <= 0) continue;
            n.xPrev = n.x;
            n.x = n.x + n.v * h;
        }
        for (int k = 0; k < beamCount_; ++k) beams_[static_cast<std::size_t>(k)].lambda = 0;
        for (int k = 0; k < nc; ++k) contacts[static_cast<std::size_t>(k)].lambdaN = contacts[static_cast<std::size_t>(k)].lambdaT = 0;

        // ---- solve ----
        for (int it = 0; it < iters; ++it) {
            // alternate the sweep direction: Gauss-Seidel in one fixed order leans a symmetric
            // crash to one side
            const bool back = ((sub + it) & 1) != 0;
            for (int kk = 0; kk < beamCount_; ++kk) {
                const int k = back ? beamCount_ - 1 - kk : kk;
                CageBeam& bm = beams_[static_cast<std::size_t>(k)];
                if (bm.broken) continue;
                const Vec3 pa = pos(bm.a), pb = pos(bm.b);
                const Vec3 d = pa - pb;
                const Real l = length(d);
                if (l < kEps) continue;
                const Vec3 n = d / l;
                const Real C = l - bm.rest;
                const Real wa = wAt(bm.a, pa, n), wb = wAt(bm.b, pb, n);
                const Real at = bm.compliance / h2;
                // XPBD constraint damping (Macklin et al. 2016, eq. 26): gamma = alpha beta / h
                const Real gamma = at * bm.damping / h;
                const Real rate = dot(n, (pa - prevPos(bm.a)) - (pb - prevPos(bm.b)));
                const Real dl = (-C - at * bm.lambda - gamma * rate) / ((1 + gamma) * (wa + wb) + at);
                bm.lambda += dl;
                move(bm.a, pa, n * dl);
                move(bm.b, pb, n * -dl);
            }
            for (int kk = 0; kk < nc; ++kk) {
                CageContact& c = contacts[order[static_cast<std::size_t>(back ? nc - 1 - kk : kk)]];
                if (c.b < 0) {
                    const Vec3 p = pos(c.a);
                    const Real depth = dot(c.surface - p, c.normal);
                    if (depth > 0) {
                        // the cell crushing (compliant contact): loads at its stiffness, unloads
                        // along a line 20x stiffer from the deepest point (crushed metal does not
                        // push back: ~5% of the energy returns)
                        Real at = c.compliance / h2, target = depth;
                        if (c.compliance > 0 && depth < c.maxDepth) {
                            at /= 20.0;
                            target = depth - c.maxDepth * (1.0 - 1.0 / 20.0);
                        }
                        const Real dl = std::max(-c.lambdaN, (target - at * c.lambdaN) / (wAt(c.a, p, c.normal) + at));
                        c.lambdaN += dl;
                        move(c.a, p, c.normal * dl);
                    }
                    if (c.lambdaN > 0 && c.mu > 0) {
                        // static friction against the point's slip this substep, capped by
                        // Coulomb (mu x the normal multiplier so far)
                        const Vec3 p1 = pos(c.a);
                        Vec3 slip = p1 - prevPos(c.a);
                        slip -= c.normal * dot(slip, c.normal);
                        const Real sl = length(slip);
                        if (sl > kEps) {
                            const Vec3 t = slip / sl;
                            const Real dl = std::min(sl / wAt(c.a, p1, t), c.mu * c.lambdaN - c.lambdaT);
                            if (dl > 0) {
                                c.lambdaT += dl;
                                move(c.a, p1, t * -dl);
                            }
                        }
                    }
                } else {
                    const Vec3 pa = pos(c.a), pb = pos(c.b);
                    const Vec3 q = pa * (1 - c.t) + pb * c.t;
                    const Real depth = dot(c.surface - q, c.normal);
                    if (depth <= 0) continue;
                    const Real wa = wAt(c.a, pa, c.normal), wb = wAt(c.b, pb, c.normal);
                    const Real w = (1 - c.t) * (1 - c.t) * wa + c.t * c.t * wb;
                    const Real dl = depth / std::max(w, kEps);
                    c.lambdaN += dl;
                    move(c.a, pa, c.normal * (dl * (1 - c.t)));
                    move(c.b, pb, c.normal * (dl * c.t));
                }
            }
        }

        // ---- velocities ----
        vBody += sb.dP / h;
        wBody += sb.dTheta / h;
        for (int i = 0; i < nodeCount_; ++i) {
            CageNode& n = nodes_[static_cast<std::size_t>(i)];
            if (n.invMass <= 0) continue;
            n.v = (n.x - n.xPrev) / h;
            // the lattice's own vibration (small node masses on stiff beams) is not part of
            // the car's motion: damp the nodes' velocity relative to the body
            const Vec3 vb = vBody + cross(wBody, n.x - sb.com);
            const Vec3 rel = n.v - vb;
            maxRel = std::max(maxRel, length(rel));
            n.v = vb + rel * relDamp;
        }
        for (int k = 0; k < nc; ++k) {
            CageContact& c = contacts[static_cast<std::size_t>(k)];
            c.impulse += c.lambdaN / h;
            if (c.compliance > 0 && c.b < 0) {  // how far the cell was pushed in
                const Real depth = dot(c.surface - pos(c.a), c.normal);
                c.maxDepth = std::max(c.maxDepth, depth);
                if (depth > cellIntrusion_) { cellIntrusion_ = depth; shapeChanged_ = true; }
            }
        }

        // ---- plasticity and tearing (the beam force is its multiplier: -lambda / h^2) ----
        for (int k = 0; k < beamCount_; ++k) {
            CageBeam& bm = beams_[static_cast<std::size_t>(k)];
            if (bm.broken) continue;
            const Real f = -bm.lambda / h2;  // + tension
            const Real fy = bm.yield + bm.hardening * bm.plastic;
            if (std::abs(f) <= fy) continue;
            // the rest length moves so the beam is left carrying its yield force: of the elastic
            // stretch it holds now (f x compliance), all but the yield's share becomes permanent
            const Real l = length(pos(bm.a) - pos(bm.b));
            const Real elastic = l - bm.rest;
            Real dr = elastic - fy * bm.compliance * (f > 0 ? 1.0 : -1.0);
            if (dr * elastic < 0) continue;  // the multiplier and the geometry disagree (unconverged): leave it
            const Real minRest = kMinRestRatio * bm.length0;
            if (bm.rest + dr < minRest) dr = std::min(0.0, minRest - bm.rest);  // crushed solid
            if (dr == 0) continue;
            bm.rest += dr;
            bm.plastic += std::abs(dr);
            state_.plasticWork += fy * std::abs(dr);
            shapeChanged_ = true;
            if (bm.rest > kTearRatio * bm.length0) {
                bm.broken = true;
                ++state_.brokenBeams;
            }
        }
    }
    body.velocity = vBody;
    body.angularVelocity = wBody;
    body.orientation = sb.q;
    body.position = sb.com - sb.q.rotate(bp.com);

    // ---- Contacts out: counts, forces, obstacle impacts, cuts at the wheels ----
    Real contactForce = 0;
    for (int k = 0; k < nc; ++k) {
        const CageContact& c = contacts[static_cast<std::size_t>(k)];
        if (c.impulse <= 0) continue;
        ++state_.nodeContacts;
        const Real J = c.impulse;  // N*s over the step
        contactForce += J / dt;
        if (c.obstacle < 0) continue;
        const Vec3 p = c.b < 0 ? pos(c.a) : pos(c.a) * (1 - c.t) + pos(c.b) * c.t;
        int slot = -1;
        for (int s = 0; s < impactCount_; ++s)
            if (impacts_[static_cast<std::size_t>(s)].id == c.obstacle) slot = s;
        if (slot < 0 && impactCount_ < kMaxObstacleImpacts) {
            slot = impactCount_++;
            impacts_[static_cast<std::size_t>(slot)] = ObstacleImpact{};
            impacts_[static_cast<std::size_t>(slot)].id = c.obstacle;
        }
        if (slot >= 0) {
            ObstacleImpact& im = impacts_[static_cast<std::size_t>(slot)];
            im.point = (im.point * im.impulse + p * J) / (im.impulse + J);
            im.normal = normalized(im.normal * im.impulse + c.normal * J);
            im.impulse += J;
            im.closingSpeed = std::max(im.closingSpeed, c.closing);
            im.work += J * std::max(0.0, c.closing);
        }
        // a hard hit at a wheel cuts its tyre
        if (cfg_.mechanical && J / dt > 60000.0) {
            const Vec3 pl = body.orientation.inverseRotate(p - body.position);
            for (std::size_t w = 0; w < wheelCount_; ++w) {
                const Vec3 d = pl - wheelPos_[w];
                if (d.x * d.x + d.y * d.y < 0.45 * 0.45 && !state_.wheels[w].punctured) {
                    state_.wheels[w].punctured = true;
                    ++state_.version;
                }
            }
        }
    }
    lastContactForce_ = contactForce;

    // body-frame shape
    for (int i = 0; i < nodeCount_; ++i) {
        CageNode& n = nodes_[static_cast<std::size_t>(i)];
        if (n.invMass > 0) n.local = body.orientation.inverseRotate(n.x - body.position);
    }
    if (shapeChanged_) {
        measure();
        ++state_.version;
    }

    // sleep once nothing touches and the lattice has settled
    if (state_.nodeContacts == 0 && maxRel < 0.05 && !groundOn) {
        if (++quietSteps_ > 30 && !nearObstacle) state_.awake = false;
    } else {
        quietSteps_ = 0;
    }
    return nc > 0 || sb.dP.x != 0 || sb.dP.y != 0 || sb.dP.z != 0;
}

void DamageModel::measure() {
    DamageState& s = state_;
    s.crushFront = s.crushRear = s.crushLeft = s.crushRight = s.crushRoof = s.intrusion = 0;
    for (int i = 0; i < nodeCount_; ++i) {
        const CageNode& n = nodes_[static_cast<std::size_t>(i)];
        if (n.invMass <= 0) continue;
        const Vec3 d = n.local - n.rest;
        switch (n.zone) {
            case CageZone::Front: s.crushFront = std::max(s.crushFront, -d.x); break;
            case CageZone::Rear: s.crushRear = std::max(s.crushRear, d.x); break;
            case CageZone::Side:
                if (n.rest.y > 0) s.crushLeft = std::max(s.crushLeft, -d.y);
                else s.crushRight = std::max(s.crushRight, d.y);
                break;
            case CageZone::Roof: s.crushRoof = std::max(s.crushRoof, -d.z); break;
            default: break;
        }
        // intrusion: a free node inside the cell's box
        const Vec3 p = n.local;
        if (p.x < xFire_ && p.x > xBulk_ && std::abs(p.y) < yInner_ && p.z > box_.floor && p.z < box_.roof) {
            const Real depth = std::min({xFire_ - p.x, p.x - xBulk_, yInner_ - std::abs(p.y), box_.roof - p.z});
            // nodes that sit on the cell's boundary (the roof) only count once they are pushed in
            const Real start = std::min({xFire_ - n.rest.x, n.rest.x - xBulk_, yInner_ - std::abs(n.rest.y), box_.roof - n.rest.z});
            s.intrusion = std::max(s.intrusion, depth - std::max(start, 0.0));
        }
    }
    s.intrusion = std::max({s.intrusion, cellIntrusion_, 0.0});
}

void DamageModel::stepMechanical(Real dt, const std::array<Real, kMaxWheels>& wheelLoad,
                                 const std::array<Real, kMaxWheels>& wheelDeflection, const std::array<Real, kMaxWheels>& wheelSpeed,
                                 Real throttle, bool engineRunning, Real ambient) {
    if (!enabled_ || !cfg_.mechanical) return;
    DamageState& s = state_;
    const DamageConfig& c = cfg_;
    bool changed = false;
    if (shapeChanged_) {
        shapeChanged_ = false;
        // suspension mounts: where the crush moved them, and the alignment that follows
        Real frontX = -1e9;
        for (std::size_t w = 0; w < wheelCount_; ++w) frontX = std::max(frontX, wheelPos_[w].x);
        for (std::size_t w = 0; w < wheelCount_; ++w) {
            WheelDamage& wd = s.wheels[w];
            if (wd.detached) continue;
            const Vec3 p = wheelPos_[w];
            const Vec3 d = displacementAt(p);
            const Vec3 dF = displacementAt(p + Vec3{0.3, 0, -0.15});
            const Vec3 dR = displacementAt(p + Vec3{-0.3, 0, -0.15});
            const Vec3 dB = displacementAt(p + Vec3{0, 0, -0.3});
            wd.mountOffset = {clamp(d.x, -0.25, 0.25), clamp(d.y, -0.25, 0.25), clamp(d.z, -0.25, 0.25)};
            wd.toe = clamp(std::atan2(dF.y - dR.y, 0.6), -0.3, 0.3);
            wd.camber = clamp(std::atan2(d.y - dB.y, 0.3), -0.3, 0.3);
            if (length(d) > c.wheelDetachDisplacement) {
                wd.detached = true;
                wd.mountOffset = {};
            }
        }
        // steering rack at the front axle, just above the floor
        const Vec3 rack = displacementAt({frontX, 0, box_.floor + 0.1});
        s.steeringOffset = clamp(rack.y * 1.2, -0.2, 0.2);
        s.steeringLock = clamp(1 - std::max(0.0, -rack.x) / 0.35, 0.25, 1.0);
        // radiator behind the front bumper; engine in the middle of the front zone
        const Vec3 rad = displacementAt({box_.front - 0.12, 0, box_.floor + 0.3 * (box_.belt - box_.floor)});
        s.radiator = std::max(s.radiator, clamp((std::max(0.0, -rad.x) + 0.5 * std::abs(rad.y)) / std::max(c.radiatorCrush, 1e-3), 0.0, 1.0));
        const Vec3 eng = displacementAt({0.5 * (box_.front + xFire_), 0, 0.5 * (box_.floor + box_.belt)});
        s.engineDamage = std::max(s.engineDamage, clamp((std::max(0.0, -eng.x) - 0.15) / 0.35, 0.0, 1.0));
        if (s.engineDamage >= 1) s.engineSeized = true;
        // aero: crumpled panels, a crushed tail takes the wing
        s.dragScale = 1 + 0.5 * unitSmoothstep(s.crushFront / 0.5) + 0.15 * unitSmoothstep((s.crushLeft + s.crushRight) / 0.4) +
                      0.1 * unitSmoothstep(s.crushRoof / 0.3);
        s.wingScale = s.crushRear > 0.25 ? 0.0 : 1.0;
        changed = true;
    }
    for (std::size_t w = 0; w < wheelCount_; ++w) {
        WheelDamage& wd = s.wheels[w];
        if (wd.detached) continue;
        const Real ratio = wheelLoad[w] / std::max(cornerLoad_[w], 1.0);
        if (ratio > c.wheelOverload) {
            wd.detached = true;
            changed = true;
            continue;
        }
        if (ratio > 0.5 * c.wheelOverload) {  // a kerb strike bends the arm: toe-out
            const Real add = 0.02 * (ratio - 0.5 * c.wheelOverload) / c.wheelOverload;
            wd.bend = std::min(0.15, wd.bend + add * dt * 240.0);
            changed = true;
        }
        if (!wd.punctured && wheelDeflection[w] > c.pinchFlatDepth && std::abs(wheelSpeed[w]) > 3.0) {
            wd.punctured = true;
            changed = true;
        }
        if (wd.punctured && wd.deflation < 1) {
            wd.deflation = std::min(1.0, wd.deflation + dt / std::max(c.deflateTime, 0.1));
            changed = true;
        }
    }
    // Coolant and engine temperature: a thermostat holds 90 C with coolant; without it the
    // engine heats with load. Past 115 C it loses power, past 135 C it seizes.
    s.coolant = std::max(0.0, s.coolant - dt * s.radiator / std::max(c.coolantLossTime, 1.0));
    if (engineRunning) {
        const Real target = 90.0 + (1 - s.coolant) * 90.0 * (0.3 + 0.7 * clamp(throttle, 0.0, 1.0));
        const Real tau = 12.0 + 18.0 * s.coolant;
        s.engineTemperature += (target - s.engineTemperature) * std::min(1.0, dt / tau);
    } else {
        s.engineTemperature += (ambient - s.engineTemperature) * std::min(1.0, dt / 300.0);
    }
    if (s.engineTemperature > 135.0 && !s.engineSeized) {
        s.engineSeized = true;
        changed = true;
    }
    const Real derate = 1.0 - 0.5 * clamp((s.engineTemperature - 115.0) / 15.0, 0.0, 1.0);
    const Real ts = s.engineSeized ? 0.0 : (1.0 - 0.7 * s.engineDamage) * derate;
    if (std::abs(ts - s.torqueScale) > 0.01) changed = true;
    s.torqueScale = ts;
    if (changed) ++s.version;
}

}  // namespace ord
