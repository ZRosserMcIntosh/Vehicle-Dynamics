// OpenRoad Dynamics — reference vehicle definitions.
// SPDX-License-Identifier: MIT
#include "reference_vehicles.hpp"

#include "ord/setup.hpp"
#include "ord/tire_generator.hpp"

namespace ord::vehicles {

namespace {

struct Layout {
    Real frontAxleX, rearAxleX, halfTrackFront, halfTrackRear;
    Real hardpointZ;  // wheel-centre height at full bump relative to the CoM
};

void addFourWheels(VehicleDefinition& d, const Layout& l, const WheelDef& frontTemplate, const WheelDef& rearTemplate) {
    auto place = [&](WheelDef w, Real x, Real y, bool left) {
        w.suspension.hardpoint = {x, y, l.hardpointZ};
        w.isLeft = left;
        d.wheels.push_back(w);
    };
    place(frontTemplate, l.frontAxleX, l.halfTrackFront, true);    // 0 FL
    place(frontTemplate, l.frontAxleX, -l.halfTrackFront, false);  // 1 FR
    place(rearTemplate, l.rearAxleX, l.halfTrackRear, true);       // 2 RL
    place(rearTemplate, l.rearAxleX, -l.halfTrackRear, false);     // 3 RR
}

}  // namespace

VehicleDefinition referenceSportsRwd() {
    VehicleDefinition d;
    d.name = "ORD Reference Sports RWD";
    d.mass = 1450;
    d.inertia = {520, 2050, 2250};

    WheelDef f;
    f.tire.radius = 0.33;
    f.tire.nominalLoad = 3600;
    f.tire.muX = 1.15;
    f.tire.muY = 1.10;
    f.suspension.travel = 0.18;
    f.suspension.springRate = 45000;
    f.suspension.bumpDamping = 3200;
    f.suspension.reboundDamping = 4800;
    f.maxSteerAngle = 0.55;
    f.maxBrakeTorque = 2800;

    WheelDef r = f;
    r.tire.radius = 0.335;
    r.tire.muX = 1.18;
    r.tire.muY = 1.12;
    r.suspension.springRate = 50000;
    r.maxSteerAngle = 0;
    r.maxBrakeTorque = 1600;
    r.handbrakeTorque = 2500;

    addFourWheels(d, {1.25, -1.30, 0.80, 0.80, -0.05}, f, r);
    d.axles = {{0, 1, 22000}, {2, 3, 12000}};
    d.steeringAckermann = 0.5;

    auto& e = d.drivetrain.engine;
    e.torqueCurve = {{800, 260}, {1000, 300}, {2000, 480}, {3000, 520}, {5000, 520}, {6500, 440}, {7200, 380}};
    e.idleRpm = 850;
    e.limiterRpm = 7200;
    e.inertia = 0.18;
    d.drivetrain.clutch.maxTorque = 800;
    d.drivetrain.gearbox.forwardRatios = {3.30, 2.20, 1.60, 1.25, 1.00, 0.82};
    d.drivetrain.gearbox.reverseRatio = 3.2;
    d.drivetrain.gearbox.shiftTime = 0.15;
    d.drivetrain.gearbox.kind = TransmissionKind::DualClutch;
    DrivenAxle rear;
    rear.axleIndex = 1;
    rear.torqueShare = 1.0;
    rear.differential.type = DifferentialType::ClutchLsd;
    rear.differential.finalDrive = 3.5;
    d.drivetrain.drivenAxles = {rear};

    d.aero.dragArea = 0.68;
    d.aero.frontDownforceArea = 0.10;
    d.aero.rearDownforceArea = 0.15;
    return d;
}

VehicleDefinition referenceHatchFwd() {
    VehicleDefinition d;
    d.name = "ORD Reference Hatch FWD";
    d.mass = 1250;
    d.inertia = {420, 1600, 1750};
    d.centerOfMass = {0.0, 0, 0};

    WheelDef f;
    f.tire.radius = 0.31;
    f.tire.nominalLoad = 3500;
    f.tire.muX = 1.05;
    f.tire.muY = 1.00;
    f.tire.cornerStiffness = 14.0;
    f.suspension.travel = 0.20;
    f.suspension.springRate = 30000;
    f.suspension.bumpDamping = 2400;
    f.suspension.reboundDamping = 3400;
    f.maxSteerAngle = 0.60;
    f.maxBrakeTorque = 2000;

    WheelDef r = f;
    r.suspension.springRate = 26000;
    r.maxSteerAngle = 0;
    r.maxBrakeTorque = 900;
    r.handbrakeTorque = 1500;

    // 60/40 static weight distribution: CoM 1.02 m behind the front axle (L = 2.55 m).
    addFourWheels(d, {1.02, -1.53, 0.76, 0.75, -0.07}, f, r);
    d.axles = {{0, 1, 15000}, {2, 3, 8000}};

    auto& e = d.drivetrain.engine;
    e.torqueCurve = {{800, 100}, {1000, 120}, {2000, 170}, {4000, 180}, {5500, 175}, {6300, 160}, {6600, 140}};
    e.idleRpm = 800;
    e.limiterRpm = 6500;
    e.inertia = 0.14;
    d.drivetrain.clutch.maxTorque = 320;
    d.drivetrain.gearbox.forwardRatios = {3.50, 2.00, 1.35, 1.00, 0.80};
    d.drivetrain.gearbox.reverseRatio = 3.3;
    d.drivetrain.gearbox.shiftTime = 0.35;
    d.drivetrain.gearbox.kind = TransmissionKind::ManualHPattern;
    DrivenAxle front;
    front.axleIndex = 0;
    front.torqueShare = 1.0;
    front.differential.type = DifferentialType::Open;
    front.differential.finalDrive = 4.1;
    d.drivetrain.drivenAxles = {front};

    d.aero.dragArea = 0.65;
    return d;
}

VehicleDefinition referenceSaloonAwd() {
    VehicleDefinition d = referenceSportsRwd();
    d.name = "ORD Reference Saloon AWD";
    d.mass = 1600;
    d.inertia = {600, 2400, 2600};
    for (auto& w : d.wheels) {
        w.tire.nominalLoad = 4000;
        w.suspension.springRate *= 1.1;
    }
    auto& e = d.drivetrain.engine;
    e.torqueCurve = {{800, 280}, {1000, 320}, {2000, 500}, {4500, 500}, {6000, 440}, {6800, 380}};
    e.limiterRpm = 6800;
    DrivenAxle front;
    front.axleIndex = 0;
    front.torqueShare = 0.4;
    front.differential.type = DifferentialType::Open;
    front.differential.finalDrive = 3.6;
    DrivenAxle rear;
    rear.axleIndex = 1;
    rear.torqueShare = 0.6;
    rear.differential.type = DifferentialType::ClutchLsd;
    rear.differential.finalDrive = 3.6;
    d.drivetrain.drivenAxles = {front, rear};
    d.drivetrain.center = CenterCoupling::Open;
    d.aero.frontDownforceArea = 0.0;
    d.aero.rearDownforceArea = 0.0;
    d.aero.dragArea = 0.72;
    return d;
}

VehicleDefinition referenceRaceDownforce() {
    VehicleDefinition d;
    d.name = "ORD Reference Race Downforce";
    d.mass = 750;
    d.inertia = {120, 900, 950};
    d.centerOfMass = {0, 0, 0};
    TireSpec fs{305, 40, 13, TireCategory::Slick, 2400};  // generic 13-inch slick sizes
    TireSpec rs{405, 40, 13, TireCategory::Slick, 3000};
    WheelDef f;
    f.tire = generateTireParams(fs);
    f.unsprungMass = 14;
    f.suspension.travel = 0.06;
    f.suspension.springRate = 160000;
    f.suspension.bumpDamping = 5500;
    f.suspension.reboundDamping = 8000;
    f.suspension.bumpStopRate = 900000;
    f.maxSteerAngle = 0.35;
    f.maxBrakeTorque = 3200;
    WheelDef r = f;
    r.tire = generateTireParams(rs);
    r.unsprungMass = 16;
    r.suspension.springRate = 180000;
    r.maxSteerAngle = 0;
    r.maxBrakeTorque = 2400;
    // Wheelbase 3.0 m, 45/55 static split; hardpoints low (ride height ~3 cm at the stops).
    addFourWheels(d, {1.65, -1.35, 0.80, 0.78, 0.02}, f, r);
    d.axles = {{0, 1, 90000}, {2, 3, 60000}};
    auto& e = d.drivetrain.engine;
    e.torqueCurve = {{3000, 250}, {6000, 330}, {9000, 380}, {11500, 375}, {12500, 340}};
    e.idleRpm = 3000;
    e.limiterRpm = 12500;
    e.inertia = 0.06;
    d.drivetrain.clutch.maxTorque = 900;
    d.drivetrain.gearbox.forwardRatios = {2.9, 2.2, 1.8, 1.5, 1.28, 1.12, 1.0, 0.9};
    d.drivetrain.gearbox.shiftTime = 0.05;
    d.drivetrain.gearbox.kind = TransmissionKind::Sequential;
    DrivenAxle rear;
    rear.axleIndex = 1;
    rear.differential.type = DifferentialType::ClutchLsd;
    rear.differential.finalDrive = 3.4;
    d.drivetrain.drivenAxles = {rear};
    d.aero.dragArea = 1.10;
    d.aero.frontDownforceArea = 1.55;
    d.aero.rearDownforceArea = 2.05;
    return d;
}

VehicleDefinition referenceHypercarAwd() {
    VehicleDefinition d = referenceSportsRwd();
    d.name = "ORD Reference Hypercar AWD";
    d.mass = 1500;
    d.inertia = {550, 2100, 2300};
    TireSpec fs{285, 30, 20, TireCategory::UltraHighPerformance, 3500};
    TireSpec rs{355, 25, 21, TireCategory::UltraHighPerformance, 3900};
    for (std::size_t w = 0; w < 4; ++w) {
        d.wheels[w].tire = generateTireParams(w < 2 ? fs : rs);
        d.wheels[w].suspension.springRate = w < 2 ? 70000 : 80000;
        d.wheels[w].maxBrakeTorque = w < 2 ? 4500 : 3000;
    }
    auto& e = d.drivetrain.engine;
    e.torqueCurve = {{1000, 700}, {2500, 1300}, {4000, 1500}, {6500, 1500}, {7800, 1250}, {8500, 1100}};
    e.idleRpm = 1000;
    e.limiterRpm = 8500;
    e.inertia = 0.25;
    d.drivetrain.clutch.maxTorque = 2600;
    d.drivetrain.gearbox.forwardRatios = {3.0, 2.15, 1.65, 1.32, 1.08, 0.9, 0.76};
    d.drivetrain.gearbox.shiftTime = 0.08;
    DrivenAxle front;
    front.axleIndex = 0;
    front.torqueShare = 0.35;
    front.differential.finalDrive = 2.9;
    DrivenAxle rear;
    rear.axleIndex = 1;
    rear.torqueShare = 0.65;
    rear.differential.type = DifferentialType::ClutchLsd;
    rear.differential.finalDrive = 2.9;
    d.drivetrain.drivenAxles = {front, rear};
    d.aero.dragArea = 0.62;
    d.aero.frontDownforceArea = 0.25;
    d.aero.rearDownforceArea = 0.35;
    return d;
}

VehicleDefinition rosserStellaV8() {
    // The Rosser Stella V8: Rosser McIntosh's own mid-engine design for the São Paulo World game.
    // Fictional; layout and wheel positions match the Blender model (wheelbase 2.649 m).
    VehicleDefinition d = referenceSportsRwd();
    d.name = "Rosser Stella V8";
    // Target weight ~1,180 kg (Lotus Exige S V6 territory: 1,176 kg DIN): an aluminium tub,
    // carbon panels and very little electronics. It was 1,480 kg; the springs, dampers, bars and
    // inertias below are scaled with it so the car keeps its frequencies and balance.
    d.mass = 1180;
    d.inertia = {383, 1555, 1714}; // mass concentrated mid-car (was 480/1950/2150 at 1,480 kg)
    // 275/40 R21 front, 335/40 R21 rear (radii ~0.377 / 0.401 m, as modelled)
    TireSpec fs{275, 40, 21, TireCategory::UltraHighPerformance, 3050};
    TireSpec rs{335, 40, 21, TireCategory::UltraHighPerformance, 4200};
    d.wheels.clear();
    WheelDef f;
    f.tire = generateTireParams(fs);
    f.suspension.travel = 0.16;
    f.suspension.springRate = 41500;
    f.suspension.bumpDamping = 3040;
    f.suspension.reboundDamping = 4650;
    f.maxSteerAngle = 0.55;
    f.maxBrakeTorque = 3800;       // carbon-ceramic
    // Double wishbones at both ends (mid-engine sports car): the front with
    // mild anti-dive, the rear with a touch of anti-squat; representative
    // geometry, not from any drawing.
    f.suspension.linkage = makeDoubleWishbone(0.40, 0.29, 0.30, 0.06, 0.10, 0.55);
    f.camberGain = 0;              // the linkage provides the camber curve
    // Static camber and the tyres' camber response: without them the outside tyres ran +1.5 deg
    // to the road at the limit at no cost in grip (the camber model was only switched on by a
    // garage setup).
    f.staticCamber = -2.0 * kPi / 180;
    enableCarCamberModel(f.tire);
    // Cup-type road tyres: in a straight line they grip ~8 % more than the generic UHP tyre
    // (whose braking peak barely exceeded its cornering peak). With it the car stops from
    // 100 km/h in ~34 m and from 70 mph in ~140 ft (458 Italia: 32.5-33.3 m, 143 ft).
    f.tire.muX *= 1.08;
    WheelDef r = f;
    r.tire = generateTireParams(rs);
    enableCarCamberModel(r.tire);
    r.tire.muX *= 1.08;
    r.staticCamber = -1.5 * kPi / 180;
    r.suspension.springRate = 51000;
    r.suspension.linkage = makeDoubleWishbone(0.45, 0.33, 0.30, 0.05, -0.06, 0.60);
    r.maxSteerAngle = 0;
    r.maxBrakeTorque = 2300;
    r.handbrakeTorque = 2600;
    // 42/58 static split: CoM 1.536 m behind the front axle, 1.113 m ahead of the rear.
    addFourWheels(d, {1.536, -1.113, 0.836, 0.804, -0.05}, f, r);
    // Anti-roll bars: the front bar was 60 % stiffer than the rear on a car with 42 % of its
    // weight on the front, so the front took most of the lateral load transfer (inside front
    // down to ~600 N at the limit) and washed out first. Now about even front to rear; the
    // extra rear downforce below keeps it planted at 200 km/h.
    d.axles = {{0, 1, 12500}, {2, 3, 14600}};
    d.steeringAckermann = 0.5;

    // Game tune: naturally aspirated flat-plane V8, 9,000 rpm limiter, ~430 kW at 9,000 rpm.
    // (The real-world concept - 3.4 L, 11,000 rpm from two scaled F20C-style banks - is noted
    // in the project docs; the game uses this 9,000 rpm tune.)
    auto& e = d.drivetrain.engine;
    e.torqueCurve = {{900, 270}, {1500, 330}, {2500, 400}, {4000, 480}, {6000, 540},
                     {7500, 520}, {8500, 480}, {9000, 455}, {9300, 380}};
    e.idleRpm = 1000;
    e.limiterRpm = 9000;
    // A light flywheel (it was 0.14) and a little more friction and pumping (~64 N*m at
    // 8,000 rpm, was ~47): off the throttle the revs now fall ~1.7x as fast. More friction would
    // be closer to a race engine, but its engine braking made a lift in a fast corner spin the car.
    e.inertia = 0.11;
    e.frictionTorque = 22;
    e.frictionPerRadS = 0.05;
    d.drivetrain.clutch.maxTorque = 900;
    d.drivetrain.gearbox.forwardRatios = {3.08, 2.19, 1.63, 1.29, 1.03, 0.84, 0.69};
    d.drivetrain.gearbox.reverseRatio = 2.9;
    d.drivetrain.gearbox.shiftTime = 0.08;
    d.drivetrain.gearbox.kind = TransmissionKind::DualClutch;
    d.drivetrain.gearbox.upshift = UpshiftControl::PowerShift;
    DrivenAxle rear;
    rear.axleIndex = 1;
    rear.torqueShare = 1.0;
    rear.differential.type = DifferentialType::ClutchLsd;
    rear.differential.finalDrive = 4.9;
    d.drivetrain.drivenAxles = {rear};

    d.aero.dragArea = 0.76;
    d.aero.frontDownforceArea = 0.15;   // more at the rear: planted at high speed (the same
    d.aero.rearDownforceArea = 0.31;    // downforce on a lighter car matters more)
    return d;
}

VehicleDefinition rosserStellaV8Manuale() {
    // The same car with a six-speed gated manual (open gate, single dry clutch) instead of the
    // seven-speed dual clutch: longer ratios, a heavier flywheel, and shifts that open the clutch
    // (a ~0.3 s torque gap) instead of handing over clutch to clutch.
    VehicleDefinition d = rosserStellaV8();
    d.name = "Rosser Stella V8 Manuale";
    d.drivetrain.engine.inertia = 0.13;   // a heavier flywheel and single dry clutch
    d.drivetrain.clutch.maxTorque = 800;
    d.drivetrain.gearbox.forwardRatios = {3.08, 2.06, 1.52, 1.19, 0.97, 0.80};
    d.drivetrain.gearbox.reverseRatio = 3.1;
    d.drivetrain.gearbox.shiftTime = 0.30;
    d.drivetrain.gearbox.kind = TransmissionKind::ManualHPattern;
    d.drivetrain.gearbox.upshift = UpshiftControl::LiftAndRamp;
    return d;
}

}  // namespace ord::vehicles
