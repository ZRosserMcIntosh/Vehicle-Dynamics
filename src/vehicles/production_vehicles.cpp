// OpenRoad Dynamics — vehicle definitions approximating real production and racing cars.
// SPDX-License-Identifier: MIT
//
// "Published" values come from the manufacturers' technical data and press material (Porsche
// newsroom / US specification sheets for the 992.2 Carrera S and 4S; Mercedes-Benz USA and
// Mercedes media for the W222 S 63 4MATIC+ and S 600 Guard; FIA 2026 Formula 1 technical
// regulations, Sections C and the power-unit rules). Everything marked "est." is an
// engineering estimate in a plausible range, chosen so the car reproduces its published
// acceleration and top speed. See docs/architecture/production-cars.md.
#include "production_vehicles.hpp"

#include "ord/tire_generator.hpp"

namespace ord::vehicles {

namespace {

constexpr Real kmh(Real v) { return v / 3.6; }

// Four wheels from the wheelbase, the static front weight share and the tracks. Hardpoint z is
// the wheel-centre height at full bump relative to the centre of mass (sets the CoM height).
void addCorners(VehicleDefinition& d, Real wheelbase, Real frontShare, Real trackF, Real trackR, Real hardpointZ,
                const WheelDef& f, const WheelDef& r) {
    const Real a = wheelbase * (1 - frontShare);  // CoM to front axle
    const Real b = wheelbase * frontShare;        // CoM to rear axle
    auto place = [&](WheelDef w, Real x, Real y, bool left) {
        w.suspension.hardpoint = {x, y, hardpointZ};
        w.isLeft = left;
        d.wheels.push_back(w);
    };
    place(f, a, trackF / 2, true);
    place(f, a, -trackF / 2, false);
    place(r, -b, trackR / 2, true);
    place(r, -b, -trackR / 2, false);
}

Real cornerLoad(Real mass, Real share) { return mass * kGravity * share / 2; }

// ---- Porsche 911 (992.2) ----
VehicleDefinition porsche911Base(Real mass, Real frontShare) {
    VehicleDefinition d;
    d.mass = mass;
    d.inertia = {470, 1850, 2000};  // est.: low, short, engine behind the rear axle
    WheelDef f;
    f.tire = generateTireParams({245, 35, 20, TireCategory::UltraHighPerformance, cornerLoad(mass, frontShare)});  // published size
    f.unsprungMass = 38;
    f.suspension.travel = 0.15;
    f.suspension.springRate = 36000;  // est.
    f.suspension.bumpDamping = 3000;
    f.suspension.reboundDamping = 4500;
    f.maxSteerAngle = 0.50;           // est. from the 11.2 m turning circle (published)
    f.maxBrakeTorque = 3400;          // 408 mm front discs (published), torque est.
    // MacPherson strut front (published layout); representative geometry with a little anti-dive.
    f.suspension.linkage = makeMacPherson(0.38, 0.58, 0.07, 0.10);
    WheelDef r = f;
    r.tire = generateTireParams({305, 30, 21, TireCategory::UltraHighPerformance, cornerLoad(mass, 1 - frontShare)});
    r.unsprungMass = 42;
    r.suspension.springRate = 52000;  // est.: carries ~62% of the car
    // Multilink rear (published layout) approximated as a double wishbone with anti-squat.
    r.suspension.linkage = makeDoubleWishbone(0.44, 0.33, 0.30, 0.05, -0.08, 0.60);
    r.suspension.bumpDamping = 3800;
    r.suspension.reboundDamping = 5600;
    r.maxSteerAngle = 0;              // optional rear-axle steering not modelled
    r.maxBrakeTorque = 2000;          // 380 mm rear discs (published), torque est.
    r.handbrakeTorque = 1600;
    // Wheelbase 2.450 m, tracks 1.601 / 1.555 m (published); weight split est. (~38/62).
    addCorners(d, 2.450, frontShare, 1.601, 1.555, -0.02, f, r);
    d.axles = {{0, 1, 12000}, {2, 3, 20000}};  // est.: tuned toward the tested ~1 g skidpad
    d.steeringAckermann = 0.5;

    // 3.0 L twin-turbo flat-six: 353 kW (480 PS) at 6,500 rpm, 530 Nm from 2,300 to 5,000 rpm,
    // 7,500 rpm limit (published). Curve shape between those points est.
    auto& e = d.drivetrain.engine;
    e.torqueCurve = {{800, 240}, {1500, 380}, {2300, 530}, {5000, 530}, {6000, 525}, {6500, 518}, {7000, 470}, {7500, 410}, {7700, 330}};
    e.idleRpm = 800;
    e.limiterRpm = 7500;
    e.inertia = 0.15;
    d.drivetrain.clutch.maxTorque = 900;
    // 8-speed PDK ratios, reverse 3.99, final drive 3.12 (published: 3.39 axle x 0.92).
    d.drivetrain.gearbox.forwardRatios = {4.89, 3.17, 2.15, 1.56, 1.18, 0.94, 0.76, 0.61};
    d.drivetrain.gearbox.reverseRatio = 3.99;
    d.drivetrain.gearbox.shiftTime = 0.08;
    d.drivetrain.gearbox.kind = TransmissionKind::DualClutch;
    // Aero: Cd 0.30 x 2.07 m^2 frontal area (published 992.1), plus cooling and active-spoiler
    // drag est.; near-zero lift with the spoiler up.
    d.aero.dragArea = 0.70;
    d.aero.frontDownforceArea = 0.02;
    d.aero.rearDownforceArea = 0.06;
    return d;
}

}  // namespace

VehicleDefinition porsche911CarreraS() {
    // 1,553 kg US curb weight (published); ~38/62 split est.
    VehicleDefinition d = porsche911Base(1553, 0.38);
    d.name = "Porsche 911 Carrera S (992.2)";
    DrivenAxle rear;
    rear.axleIndex = 1;
    rear.torqueShare = 1.0;
    // electronically controlled rear differential lock (PTV Plus, published) ~ clutch LSD
    rear.differential.type = DifferentialType::ClutchLsd;
    rear.differential.finalDrive = 3.12;
    rear.differential.preload = 100;
    rear.differential.powerRamp = 0.5;
    rear.differential.coastRamp = 0.3;
    d.drivetrain.drivenAxles = {rear};
    return d;
}

VehicleDefinition porsche911Carrera4S() {
    // 1,585 kg US curb weight (published); front axle drive moves a little weight forward (est.).
    VehicleDefinition d = porsche911Base(1585, 0.39);
    d.name = "Porsche 911 Carrera 4S (992.2)";
    // PTM: rear permanently driven, a multi-plate clutch sends torque to the front axle.
    // Modelled as a rear-biased split with a slipping centre clutch (est.).
    DrivenAxle front;
    front.axleIndex = 0;
    front.torqueShare = 0.25;
    front.differential.type = DifferentialType::Open;
    front.differential.finalDrive = 3.12;
    DrivenAxle rear;
    rear.axleIndex = 1;
    rear.torqueShare = 0.75;
    rear.differential.type = DifferentialType::ClutchLsd;
    rear.differential.finalDrive = 3.12;
    rear.differential.preload = 100;
    rear.differential.powerRamp = 0.5;
    rear.differential.coastRamp = 0.3;
    d.drivetrain.drivenAxles = {front, rear};
    d.drivetrain.center = CenterCoupling::ClutchLsd;
    d.drivetrain.centerPreload = 60;
    d.drivetrain.centerRamp = 0.25;
    return d;
}

VehicleDefinition mercedesAmgS63() {
    VehicleDefinition d;
    d.name = "Mercedes-AMG S 63 4MATIC+ (W222)";
    d.mass = 2180;                   // US curb weight (published, Mercedes-Benz USA)
    d.inertia = {780, 3900, 4150};   // est.: long-wheelbase saloon
    const Real frontShare = 0.53;    // est.
    WheelDef f;
    f.tire = generateTireParams({255, 40, 20, TireCategory::UltraHighPerformance, cornerLoad(d.mass, frontShare)});
    f.unsprungMass = 50;
    f.suspension.travel = 0.20;
    f.suspension.springRate = 42000;  // air springs, est.
    f.suspension.bumpDamping = 3800;
    f.suspension.reboundDamping = 5600;
    f.maxSteerAngle = 0.54;           // est. from a ~12.3 m turning circle
    f.maxBrakeTorque = 3800;          // 390 mm front discs (published), torque est.
    // Four-link front and multilink rear (published layouts), both approximated as double
    // wishbones with representative geometry: long arms, moderate camber gain.
    f.suspension.linkage = makeDoubleWishbone(0.42, 0.32, 0.32, 0.06, 0.12, 0.50);
    WheelDef r = f;
    r.tire = generateTireParams({285, 35, 20, TireCategory::UltraHighPerformance, cornerLoad(d.mass, 1 - frontShare)});
    r.suspension.springRate = 40000;
    r.suspension.linkage = makeDoubleWishbone(0.45, 0.34, 0.30, 0.04, -0.08, 0.60);
    r.maxSteerAngle = 0;
    r.maxBrakeTorque = 2400;          // 360 mm rear discs (published), torque est.
    r.handbrakeTorque = 1500;
    // Wheelbase 3.165 m, tracks 1.643 m (published).
    addCorners(d, 3.165, frontShare, 1.643, 1.643, -0.11, f, r);
    d.axles = {{0, 1, 26000}, {2, 3, 16000}};
    d.steeringAckermann = 0.6;

    // M177 4.0 L twin-turbo V8: 450 kW (612 PS) at 5,500-6,000 rpm, 900 Nm at 2,750-4,500 rpm
    // (published). Limiter ~6,500 rpm est.
    auto& e = d.drivetrain.engine;
    e.torqueCurve = {{650, 400}, {1500, 620}, {2000, 780}, {2750, 900}, {4500, 900}, {5000, 850}, {5500, 781}, {6000, 716}, {6500, 600}, {6700, 450}};
    e.idleRpm = 650;
    e.limiterRpm = 6500;
    e.inertia = 0.22;
    d.drivetrain.clutch.maxTorque = 1300;  // MCT wet start-off clutch
    // AMG SPEEDSHIFT MCT 9G: ratios, reverse 4.80, final drive 2.82 (published).
    d.drivetrain.gearbox.forwardRatios = {5.35, 3.24, 2.25, 1.64, 1.21, 1.00, 0.86, 0.72, 0.60};
    d.drivetrain.gearbox.reverseRatio = 4.80;
    d.drivetrain.gearbox.shiftTime = 0.12;
    d.drivetrain.gearbox.kind = TransmissionKind::Automatic;
    // AMG Performance 4MATIC+: fully variable; modelled rear-biased with a slipping centre clutch (est.).
    DrivenAxle front;
    front.axleIndex = 0;
    front.torqueShare = 0.3;
    front.differential.type = DifferentialType::Open;
    front.differential.finalDrive = 2.82;
    DrivenAxle rear;
    rear.axleIndex = 1;
    rear.torqueShare = 0.7;
    rear.differential.type = DifferentialType::ClutchLsd;
    rear.differential.finalDrive = 2.82;
    rear.differential.preload = 60;
    rear.differential.powerRamp = 0.3;
    rear.differential.coastRamp = 0.2;
    d.drivetrain.drivenAxles = {front, rear};
    d.drivetrain.center = CenterCoupling::ClutchLsd;
    d.drivetrain.centerPreload = 80;
    d.drivetrain.centerRamp = 0.25;
    d.aero.dragArea = 0.66;          // Cd ~0.26-0.27 x ~2.45 m^2 (est.)
    d.topSpeedLimit = kmh(250);      // published (300 km/h with the AMG Driver's Package)
    return d;
}

VehicleDefinition mercedesS600Guard() {
    VehicleDefinition d;
    d.name = "Mercedes-Benz S 600 Guard (W222)";
    d.mass = 4160;                   // published kerb weight ~4.16 t (VR9 armour, ~2 t over an S 600)
    d.inertia = {2100, 8600, 9000};  // est.: armour in the doors, roof and floor
    const Real frontShare = 0.52;    // est.
    WheelDef f;
    // Michelin PAX run-flat tyres (published); size not published, a 19-inch touring tyre est.
    f.tire = generateTireParams({255, 45, 19, TireCategory::Touring, cornerLoad(d.mass, frontShare)});
    f.unsprungMass = 75;             // run-flat inserts, big brakes (est.)
    f.suspension.travel = 0.18;
    f.suspension.springRate = 105000;  // reinforced air + steel springs, est.
    f.suspension.bumpDamping = 9000;
    f.suspension.reboundDamping = 13000;
    f.maxSteerAngle = 0.52;
    f.maxBrakeTorque = 6200;         // six-piston front calipers, larger discs (published), torque est.
    f.suspension.linkage = makeDoubleWishbone(0.42, 0.32, 0.32, 0.06, 0.12, 0.50);  // W222 platform, as the S 63
    WheelDef r = f;
    r.tire = generateTireParams({255, 45, 19, TireCategory::Touring, cornerLoad(d.mass, 1 - frontShare)});
    r.suspension.springRate = 110000;
    r.suspension.linkage = makeDoubleWishbone(0.45, 0.34, 0.30, 0.04, -0.08, 0.60);
    r.maxSteerAngle = 0;
    r.maxBrakeTorque = 3800;
    r.handbrakeTorque = 2500;
    // long wheelbase 3.165 m (published); tracks as the W222 (est.)
    addCorners(d, 3.165, frontShare, 1.625, 1.635, -0.18, f, r);
    d.axles = {{0, 1, 60000}, {2, 3, 35000}};
    d.steeringAckermann = 0.6;

    // M277 6.0 L twin-turbo V12: 390 kW (530 PS), 830 Nm from 1,900 rpm (published); power peak
    // 4,900-5,300 rpm as the S 600 (published), limiter est.
    auto& e = d.drivetrain.engine;
    e.torqueCurve = {{600, 450}, {1200, 650}, {1900, 830}, {4000, 830}, {4900, 760}, {5300, 703}, {5800, 610}, {6000, 520}};
    e.idleRpm = 600;
    e.limiterRpm = 5800;
    e.inertia = 0.30;
    d.drivetrain.clutch.maxTorque = 1400;  // torque converter lock-up, est.
    // 7G-TRONIC (published as the gearbox); ratios are the standard 7G-TRONIC set (est. for
    // this car), final drive est.
    d.drivetrain.gearbox.forwardRatios = {4.38, 2.86, 1.92, 1.37, 1.00, 0.82, 0.73};
    d.drivetrain.gearbox.reverseRatio = 3.42;
    d.drivetrain.gearbox.shiftTime = 0.25;
    d.drivetrain.gearbox.kind = TransmissionKind::Automatic;
    DrivenAxle rear;
    rear.axleIndex = 1;
    rear.torqueShare = 1.0;
    rear.differential.type = DifferentialType::Open;
    rear.differential.finalDrive = 2.65;
    d.drivetrain.drivenAxles = {rear};
    d.aero.dragArea = 0.72;          // est.: S-Class shape, slightly taller on armour
    d.topSpeedLimit = kmh(210);      // published, electronically limited
    return d;
}

VehicleDefinition formula1Car2026() {
    VehicleDefinition d;
    d.name = "Formula 1 car (2026 regulations)";
    d.mass = 800;                    // 768 kg minimum incl. driver (published) + ~32 kg fuel mid-race (est.)
    d.inertia = {110, 850, 900};     // est.
    const Real frontShare = 0.45;    // est. (regulated weight-distribution window ~45%)
    // 18-inch slicks; 2026 sizes ~280 mm / 705 mm front, ~375 mm / 710 mm rear (derived from the
    // published width and diameter reductions).
    WheelDef f;
    f.tire = generateTireParams({280, 44, 18, TireCategory::Slick, cornerLoad(d.mass, frontShare) * 2.0});
    f.unsprungMass = 12;
    f.suspension.travel = 0.045;
    f.suspension.springRate = 200000;  // est.
    f.suspension.bumpDamping = 6000;
    f.suspension.reboundDamping = 9000;
    f.suspension.bumpStopRate = 1500000;
    f.maxSteerAngle = 0.30;
    f.maxBrakeTorque = 4500;         // carbon brakes (est.)
    // Double wishbones with long, nearly parallel arms (little camber change over the short
    // travel) and the spring reached through a pushrod from the upright (regulated layout).
    f.suspension.linkage = makeDoubleWishbone(0.55, 0.50, 0.26, 0.03, 0.05, 0.90);
    WheelDef r = f;
    r.tire = generateTireParams({375, 34, 18, TireCategory::Slick, cornerLoad(d.mass, 1 - frontShare) * 2.0});
    r.unsprungMass = 14;
    r.suspension.springRate = 220000;
    r.suspension.linkage = makeDoubleWishbone(0.55, 0.50, 0.26, 0.03, -0.03, 0.90);
    r.maxSteerAngle = 0;
    r.maxBrakeTorque = 3000;
    // Wheelbase 3.4 m max, 1.9 m overall width (published); tracks est.
    addCorners(d, 3.40, frontShare, 1.60, 1.55, 0.08, f, r);
    d.axles = {{0, 1, 120000}, {2, 3, 80000}};
    d.steeringAckermann = 0.2;

    // Power unit: 1.6 L V6 turbo (~400 kW) + MGU-K (350 kW), combined as one crank torque curve
    // (~750 kW at 10,500-12,000 rpm). Not modelled: battery energy limits and the MGU-K
    // deployment taper above 290 km/h (published), so this is a qualifying-lap car.
    auto& e = d.drivetrain.engine;
    e.torqueCurve = {{4000, 330}, {6000, 470}, {8000, 600}, {10000, 690}, {11000, 650}, {12000, 590}, {13000, 490}, {14000, 400}, {15000, 330}};
    e.idleRpm = 4000;
    e.limiterRpm = 15000;
    e.inertia = 0.04;
    e.frictionTorque = 30;
    d.drivetrain.clutch.maxTorque = 1400;
    // 8 forward gears (published); ratios est. (~100 km/h in 1st, ~345 km/h in 8th at 12,500 rpm).
    d.drivetrain.gearbox.forwardRatios = {4.00, 3.38, 2.86, 2.41, 2.04, 1.72, 1.46, 1.23};
    d.drivetrain.gearbox.reverseRatio = 4.0;
    d.drivetrain.gearbox.shiftTime = 0.03;
    d.drivetrain.gearbox.inputInertia = 0.01;
    d.drivetrain.gearbox.kind = TransmissionKind::Sequential;
    DrivenAxle rear;
    rear.axleIndex = 1;
    rear.torqueShare = 1.0;
    rear.differential.type = DifferentialType::ClutchLsd;
    rear.differential.finalDrive = 4.0;
    rear.differential.preload = 150;
    rear.differential.powerRamp = 0.5;
    rear.differential.coastRamp = 0.35;
    d.drivetrain.drivenAxles = {rear};
    // Corner mode of the active aero (est.): Cl*A ~4.2 m^2, Cd*A ~1.25 m^2.
    d.aero.dragArea = 1.25;
    d.aero.frontDownforceArea = 1.85;
    d.aero.rearDownforceArea = 2.35;
    return d;
}

}  // namespace ord::vehicles
