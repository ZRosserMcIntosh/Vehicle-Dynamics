// OpenRoad Dynamics — engine-independent vehicle physics core.
// SPDX-License-Identifier: MIT
//
// Damage: a node/beam crash structure, and the mechanical damage it causes.
//
// 1. Crash structure ("cage"). The body is a lattice of nodes joined by beams, built from the
//    car's own box (the same overhangs, track, floor and roof as the body hull) and its crush
//    depths (CrashStructure: front, rear and side crush space). The passenger cell is the rigid
//    body itself: its nodes are anchored to it. The crush zones ahead of and behind the cell,
//    the doors and sills, and the roof are free nodes with a small share of the car's mass.
//    Beams are elastic-plastic: past their yield force they shorten or stretch permanently
//    (with linear hardening), bottom out when crushed flat, and tear when stretched too far.
//
//    The lattice is solved with XPBD (extended position-based dynamics: Macklin, Müller and
//    Chentanez, 2016), with the rigid body taking part in the solve through its generalised
//    inverse mass at each anchored point (Müller et al., "Detailed rigid body simulation with
//    extended position based dynamics", 2020). So when a free node meets a wall, the beams
//    behind it carry the contact force back into the cell, yield, and decelerate the car over
//    the time it takes the structure to crush: the crash pulse, the crush depth and the
//    intrusion come out of the structure instead of being synthesised from the speed change.
//    The same nodes are the car's collision hull against the ground (a roof crushes when the
//    car lands on it) and against the host's obstacles (ObstacleQuery).
//
//    Cost: the cage sleeps (its nodes ride with the body, nothing is queried or solved) unless
//    an obstacle is near, the car is tilted or airborne, or a wheel is gone. Normal driving costs
//    nothing. Fixed-size arrays: nothing allocates after construction.
//
// 2. Mechanical damage, read from the deformed lattice and from the loads the parts see:
//    - suspension mounts pushed back or sideways: toe, camber and wheel position change; pushed
//      far enough, or loaded far past their strength, the wheel comes off;
//    - tyres: a pinch flat when the tyre bottoms out on the rim, or a cut from a hit at the
//      wheel; the tyre then deflates over a few seconds (radius, stiffness and grip drop);
//    - steering: a rack knocked sideways steers the car off-centre; pushed back, it loses lock;
//    - cooling and engine: a crushed radiator leaks coolant, the engine runs hot, loses power
//      past ~115 C and seizes past ~135 C; a crushed engine bay costs power directly;
//    - aero: crumpled panels add drag; a rear wing on a crushed tail stops working.
//
// What this is: a game damage model built on a physical method (an elastic-plastic beam
// structure in a constraint solver) and calibrated so a car's crush depth and pulse agree with
// its CrashStructure (a 56 km/h full-width frontal crushes about delta-V / omega). What it is
// not: a finite-element crash simulation. The lattice is coarse (~100 nodes), beams are
// one-dimensional, there are no panels, welds or material models, and the cell is rigid.
#pragma once

#include <array>
#include <cstdint>

#include "ord/crash.hpp"
#include "ord/drivetrain.hpp"
#include "ord/ground.hpp"
#include "ord/math.hpp"
#include "ord/rigid_body.hpp"

namespace ord {

// One contact between a point (or an edge) and a static obstacle.
struct ObstacleContact {
    bool hit = false;
    Real depth = 0;      // m, penetration (> 0 inside); a small negative value is a speculative contact
    Vec3 normal;         // unit, pointing out of the obstacle (toward the car)
    Real friction = 0.4;
    int id = -1;         // the host's obstacle id (reported back in ObstacleImpact)
};

// Static obstacles the host knows about: walls, barriers, posts, trees. Implemented by the
// host (like GroundQuery); the core only asks. Must be thread-safe for concurrent const calls.
class ObstacleQuery {
public:
    virtual ~ObstacleQuery() = default;
    // Broadphase: could anything be within `radius` of `centre`? (cheap, conservative)
    virtual bool anyNear(const Vec3& centre, Real radius) const = 0;
    // The deepest obstacle the point `p` is in or within `margin` of. `ref` is where the point
    // came from (its position at the start of the step, a little in toward the car's centre):
    // thin walls push toward the side `ref` is on.
    virtual ObstacleContact point(const Vec3& p, const Vec3& ref, Real margin) const = 0;
    // Round or thin obstacles (posts, poles, trunks, wall ends) poking between nodes: the
    // deepest one the edge a-b is in, with the contact's parameter `t` along the edge (0..1).
    virtual ObstacleContact edge(const Vec3& a, const Vec3& b, Real margin, Real& t) const {
        (void)a; (void)b; (void)margin; t = 0;
        return {};
    }
};

// What hit what, for the host (breakable walls, sounds, particles): one entry per obstacle
// touched in the step, summed over the nodes touching it.
struct ObstacleImpact {
    int id = -1;
    Vec3 point;          // world, impulse-weighted
    Vec3 normal;         // out of the obstacle
    Real impulse = 0;    // N*s along the normal this step
    Real closingSpeed = 0;  // m/s, fastest node approach speed
    Real work = 0;       // J, impulse x closing speed (the energy the contact took this step)
};

struct DamageConfig {
    bool structure = true;    // the node/beam cage (contacts, crush, pulse)
    bool mechanical = true;   // parts damaged by it (alignment, wheels, tyres, steering, engine, aero)
    CrashStructure crash;     // crush depths and stiffness the cage is built and calibrated from
    Real nodeMassFraction = 0.05;  // of the vehicle mass, shared by the free nodes (inertia in the solve)
    int substeps = 16;        // XPBD substeps per vehicle step while awake
    int iterations = 2;       // XPBD iterations per substep
    Real strength = 1.0;      // scales every beam's yield force (armoured cars > 1, a kit car < 1)
    Real roofStrength = 3.0;  // the roof holds this many times the car's weight before it yields (FMVSS 216: 3)
    // Part strengths
    Real wheelDetachDisplacement = 0.35;  // m of mount displacement that tears a wheel off
    Real wheelOverload = 30.0;            // x static corner load: a suspension load past this breaks it
    Real pinchFlatDepth = 0.085;          // m of tyre deflection that pinches it on the rim
    Real deflateTime = 4.0;               // s for a punctured tyre to go flat
    Real maxDeflation = 1.0;              // how flat it goes (run-flats: 0.25)
    Real radiatorCrush = 0.10;            // m of crush at the radiator that destroys it
    Real coolantLossTime = 45.0;          // s to lose all coolant from a destroyed radiator
};

inline constexpr int kMaxCageNodes = 128;
inline constexpr int kMaxCageBeams = 640;
inline constexpr int kMaxObstacleImpacts = 8;
inline constexpr int kMaxCageContacts = 192;

enum class CageZone : std::uint8_t { Cell, Front, Rear, Side, Roof };

struct CageNode {
    Vec3 rest;           // body frame, as built
    Vec3 local;          // body frame, as deformed (plastic + whatever elastic shape it rests in)
    Vec3 x, xPrev, v;    // world (free nodes, while awake)
    Real invMass = 0;    // 0 = anchored to the rigid body (the passenger cell)
    CageZone zone = CageZone::Cell;
    std::int8_t wheel = -1;  // a tyre-shoulder contact node of this wheel (gone with the wheel)
    bool skin = false;       // on the outside of the lattice (edges between skin nodes collide)
};

struct CageBeam {
    std::uint16_t a = 0, b = 0;
    Real length0 = 0;    // m, as built
    Real rest = 0;       // m, current plastic rest length
    Real compliance = 0; // 1 / elastic stiffness (m/N)
    Real damping = 0;    // s, XPBD damping time (beta)
    Real yield = 0;      // N, initial yield force
    Real hardening = 0;  // N per metre of plastic deformation
    Real plastic = 0;    // m, accumulated plastic deformation (shortening + stretching)
    Real lambda = 0;     // XPBD multiplier (this step)
    CageZone zone = CageZone::Cell;
    bool broken = false;
    bool edge = false;   // skin edge: collides with posts and wall ends
};

// A contact of the structure in the step being solved (internal; kept in the model so the
// solver needs little stack).
struct CageContact {
    int a = -1, b = -1;   // node (and second node for an edge)
    Real t = 0;           // edge parameter
    Vec3 surface;         // a point on the contact plane
    Vec3 normal;
    Real mu = 0.5;
    int obstacle = -1;    // -1 ground
    Real lambdaN = 0, lambdaT = 0;
    Real closing = 0;     // m/s, approach speed at detection
    Real impulse = 0;     // N*s, normal, summed over the substeps
    Real compliance = 0;  // m/N: the cell's own crush past the zones (anchored nodes on obstacles)
    Real maxDepth = 0;    // deepest the cell went in: it unloads plastically from there
    Real allowance = 0;   // m of overlap left this substep (an existing overlap is pushed out gradually)
};

struct WheelDamage {
    Real toe = 0;            // rad, added to the wheel's steer angle (toward +yaw)
    Real camber = 0;         // rad, added to the wheel's camber (top toward +y)
    Vec3 mountOffset;        // m, body frame, the suspension hardpoint moved by the crush
    Real bend = 0;           // rad, toe added by overloads (kerb strikes)
    bool detached = false;
    bool punctured = false;
    Real deflation = 0;      // 0 inflated .. 1 flat
};

struct DamageState {
    std::array<WheelDamage, kMaxWheels> wheels{};
    Real steeringOffset = 0;   // rad at the road wheels (the car pulls)
    Real steeringLock = 1;     // fraction of the lock left
    Real radiator = 0;         // 0 intact .. 1 destroyed
    Real coolant = 1;          // fraction left
    Real engineTemperature = 90;  // deg C
    Real engineDamage = 0;     // 0 .. 1, crushed engine bay
    bool engineSeized = false;
    Real torqueScale = 1;      // what the engine can still give
    Real dragScale = 1;
    Real wingScale = 1;
    // Structure
    Real crushFront = 0, crushRear = 0, crushLeft = 0, crushRight = 0, crushRoof = 0;  // m, deepest
    Real intrusion = 0;        // m, deepest free node pushed past the cell's boundary
    Real plasticWork = 0;      // J absorbed by the structure since built or repaired
    int brokenBeams = 0;
    int nodeContacts = 0;      // nodes touching the ground or an obstacle in the last step
    std::uint32_t solves = 0;  // steps the structure was solved (diagnostics, cost)
    int droppedContacts = 0;   // contacts past kMaxCageContacts (not solved) since built or repaired
    bool awake = false;
    std::uint32_t version = 0;   // bumped whenever the shape or a part changes (redraw)
};

// Geometry the cage is built from (body frame).
struct CageBox {
    Real front = 0, rear = 0, halfWidth = 0, floor = 0, roof = 0, belt = 0;
    Real roofRear = 0, roofFront = 0;  // greenhouse extent in x
};

class DamageModel {
public:
    DamageModel() = default;

    // Builds the lattice. `wheelPos` are the wheels' hardpoints and `shoulder` the tyre
    // shoulder points (body frame), `cornerLoad` their static loads (N).
    void build(const DamageConfig& cfg, const CageBox& box, Real mass, std::size_t wheelCount,
               const std::array<Vec3, kMaxWheels>& wheelPos, const std::array<Vec3, kMaxWheels>& shoulder,
               const std::array<Real, kMaxWheels>& cornerLoad, Real ambient);
    bool enabled() const { return enabled_; }
    void disable() { enabled_ = false; }
    const DamageConfig& config() const { return cfg_; }
    // Everything as built: shape, parts, tyres, coolant.
    void repair(Real ambient);

    // Solves the structure for one step after the chassis has been integrated. `body` is the
    // integrated chassis (modified in place: contact and crush corrections), `start` the chassis
    // at the start of the step. Returns true if it touched the body.
    struct BodyParams {
        Real mass = 1;
        Vec3 invInertia;      // body frame, principal
        Vec3 com;             // body frame
        Real bodyFriction = 0.5;  // body sliding on the road (scaled by the surface)
    };
    bool stepStructure(Real dt, RigidBodyState& body, const RigidBodyState& start, const BodyParams& bp,
                       const GroundQuery& ground, const ObstacleQuery* obstacles, bool groundGate,
                       std::uint64_t& groundQueries);

    // Mechanical parts for one step: reads the lattice, the wheel loads and deflections, and
    // runs the engine temperature. Per wheel: load (N), tyre deflection (m), rolling speed (m/s).
    void stepMechanical(Real dt, const std::array<Real, kMaxWheels>& wheelLoad,
                        const std::array<Real, kMaxWheels>& wheelDeflection, const std::array<Real, kMaxWheels>& wheelSpeed,
                        Real throttle, bool engineRunning, Real ambient);

    const DamageState& state() const { return state_; }
    int nodeCount() const { return nodeCount_; }
    int beamCount() const { return beamCount_; }
    const CageNode& node(int i) const { return nodes_[static_cast<std::size_t>(i)]; }
    const CageBeam& beam(int i) const { return beams_[static_cast<std::size_t>(i)]; }
    // Total normal force the nodes carried in the last step (N): ground and obstacles.
    Real contactForce() const { return lastContactForce_; }
    int impactCount() const { return impactCount_; }
    const ObstacleImpact& impact(int i) const { return impacts_[static_cast<std::size_t>(i)]; }
    // Displacement of a body-frame point by the deformation (inverse-distance weighted).
    Vec3 displacementAt(const Vec3& p) const;
    // Puts the free nodes back on the body (teleports, resets): keeps the deformed shape.
    void snapToBody(const RigidBodyState& body);

private:
    int addNode(const Vec3& p, Real invMass, CageZone z, bool skin);
    void addBeam(int a, int b, CageZone z, Real yield, Real hardening);
    void measure();

    DamageConfig cfg_;
    bool enabled_ = false;
    CageBox box_;
    Real mass_ = 0;
    std::size_t wheelCount_ = 0;
    std::array<Vec3, kMaxWheels> wheelPos_{};
    std::array<Real, kMaxWheels> cornerLoad_{};
    Real xFire_ = 0, xBulk_ = 0, yInner_ = 0;  // the cell's boundary
    std::array<CageNode, kMaxCageNodes> nodes_{};
    std::array<CageBeam, kMaxCageBeams> beams_{};
    std::array<std::int16_t, kMaxCageNodes> mirror_{};  // the node's left/right twin (-1: none)
    int nodeCount_ = 0, beamCount_ = 0;
    std::array<ObstacleImpact, kMaxObstacleImpacts> impacts_{};
    std::array<CageContact, kMaxCageContacts> contacts_{};
    int impactCount_ = 0;
    DamageState state_;
    int quietSteps_ = 0;
    bool shapeChanged_ = false;     // the lattice yielded this step (measure, bump the version)
    bool mechanicalDirty_ = false;  // the parts have not been re-read since the shape changed
    Real lastContactForce_ = 0;
    Real cellCompliance_ = 0;
    Real cellIntrusion_ = 0;
};

}  // namespace ord
