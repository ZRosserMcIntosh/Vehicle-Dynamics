// OpenRoad Dynamics — engine-independent vehicle physics core.
// SPDX-License-Identifier: MIT
//
// Tire force model: a reduced Magic Formula (Pacejka) with load sensitivity,
// combined-slip weighting functions and first-order transient slip
// (relaxation length).
//
// References (published mathematics; no third-party code):
//  - H.B. Pacejka, "Tire and Vehicle Dynamics", 3rd ed., Butterworth-Heinemann,
//    2012. Ch. 4 (Magic Formula, MF 5.2/6.x structure: B,C,D,E, load
//    sensitivity dfz, combined-slip weighting functions G_xa / G_yk) and
//    Ch. 7 (transient slip, relaxation length sigma).
//  - E. Bakker, L. Nyborg, H.B. Pacejka, "Tyre modelling for use in vehicle
//    dynamics studies", SAE 870421, 1987.
//
// This is deliberately a *reduced* parameter set (see TireParams). It keeps the
// MF structure so a full MF 6.x coefficient set can replace it later behind the
// same interface (docs/decisions/physics-foundation.md).
#pragma once

#include "ord/math.hpp"

namespace ord {

struct TireParams {
    Real radius = 0.33;           // m, loaded rolling radius (rigid-ring approximation)
    Real nominalLoad = 4000.0;    // N, Fz0

    // Peak friction coefficients at nominal load and their load sensitivity:
    //   mu(Fz) = mu0 * (1 + loadSensitivity * dfz),  dfz = (Fz - Fz0) / Fz0
    Real muX = 1.10, muXLoadSensitivity = -0.08;
    Real muY = 1.05, muYLoadSensitivity = -0.10;

    // Shape factors (C) and curvature factors (E, must be <= 1).
    Real shapeX = 1.65, curvatureX = 0.10;
    Real shapeY = 1.35, curvatureY = -0.40;

    // Longitudinal slip stiffness: Kx = longStiffnessPerLoad * Fz   [N per unit slip]
    Real longStiffnessPerLoad = 18.0;
    // Cornering stiffness (MF 6.x form): Ky = cornerStiffness * Fz0 * sin(2 atan(Fz / (cornerStiffnessLoad * Fz0)))
    Real cornerStiffness = 16.0;      // peak Ky in units of Fz0 per rad
    Real cornerStiffnessLoad = 1.9;   // load (in Fz0) at which Ky peaks

    // Combined-slip weighting functions (MF 5.2 / 6.x, without shifts):
    //   G_xa = cos(C_xa * atan(B_xa * alpha)),  B_xa = rBx1 * cos(atan(rBx2 * kappa))
    //   G_yk = cos(C_yk * atan(B_yk * kappa)),  B_yk = rBy1 * cos(atan(rBy2 * alpha))
    Real rBx1 = 13.0, rBx2 = 9.7, rCx1 = 1.0;
    Real rBy1 = 10.6, rBy2 = 7.8, rCy1 = 1.0;

    // Transient slip: relaxation lengths [m].
    Real relaxationLengthX = 0.15;
    Real relaxationLengthY = 0.45;
    // Tread/carcass hysteresis damping used at very low speed, where the
    // relaxation ODE alone is undamped (standstill holding). [N*s/m per N of load]
    Real lowSpeedDampingPerLoad = 3.0;
    Real lowSpeedDampingFadeSpeed = 3.0;  // m/s, damping fades out above this speed

    // Camber (inclination) response, used by two-wheelers. A wheel inclined by
    // gamma (top toward +y) pushes toward +y even at zero slip angle: camber
    // thrust, which carries most of a leaning bike's cornering force. Shape
    // after the motorcycle Magic Formula (Pacejka 2012 sec. 11.5; Sharp,
    // Evangelou, Limebeer 2004):
    //   Fy = D sin(C atan(B alpha - E(...)) + C_g atan(B_g gamma)),
    //   B_g = camberStiffness * Fz / (C_g * D)  (so dFy/dgamma = camberStiffness * Fz at 0).
    // 0 = no camber response; the four-wheel model passes gamma = 0 today.
    Real camberStiffness = 0.0;   // per rad, in units of Fz (bicycle ~0.9, motorcycle ~1.0)
    Real camberShape = 0.6;       // C_g
    // Camber twisting torque: an inclined, rolling contact patch twists the
    // wheel toward the inside of the turn. Mz += camberTwist * Fz * gamma.
    Real camberTwist = 0.0;       // m per rad
    // Toroidal (rounded) cross-section: the contact point rolls around the
    // crown as the wheel leans, moving sideways by crownRadius * gamma.
    // 0 = thin disc. Used by the single-track model; the four-wheel model
    // keeps its flat footprint.
    Real crownRadius = 0.0;       // m

    Real rollingResistance = 0.012;   // dimensionless coefficient
    Real pneumaticTrail = 0.035;      // m, at zero slip angle
    Real trailCollapseAngle = 0.20;   // rad, trail reaches zero at this slip angle

    Real wheelInertia = 1.2;          // kg*m^2, wheel + tire + hub + brake rotor

    // Vertical (radial) tire properties, used when the wheel has its own
    // unsprung-mass degree of freedom (VehicleFidelity::unsprungMass).
    Real verticalStiffness = 250000.0;  // N/m for the whole contact patch
    Real verticalDamping = 400.0;       // N*s/m
    // Contact-patch footprint half extents used for multi-point contact sampling.
    Real patchHalfLength = 0.09;        // m, along the wheel heading
    Real patchHalfWidth = 0.10;         // m, across
};

// Per-contact surface multipliers; supplied by the host's surface query.
struct SurfaceProperties {
    Real frictionScale = 1.0;           // multiplies muX / muY (asphalt dry = 1)
    Real rollingResistanceScale = 1.0;
    // Crude wet-road speed falloff: mu *= 1 / (1 + wetSpeedFalloff * v). 0 = dry.
    // Placeholder for a proper water-film / hydroplaning model (roadmap M3).
    Real wetSpeedFalloff = 0.0;
    // Geometric roughness hint for the ground cache: 0 = smooth road, where a
    // coarse plane is exact enough; > 0 = cobblestones, curbs, potholes,
    // gravel, where ground must be sampled densely. Does not change grip.
    Real roughness = 0.0;
    // Procedural micro-profile for surfaces whose collision mesh is flat but
    // whose real surface is not (paving stones, worn concrete, rumble strips
    // built as flat geometry): stones of `textureWavelength` pitch and up to
    // `textureAmplitude` height, added to the sampled ground height under each
    // contact sample. Deterministic in world position (multiplayer-safe), costs
    // no engine queries. Real geometry (curbs, potholes, bumps) should stay geometry.
    Real textureAmplitude = 0.0;     // m (0 = off)
    Real textureWavelength = 0.15;   // m
    Real textureVariation = 0.4;     // 0 = identical stones, 1 = heights vary 0..A per stone
};

// Height of the procedural micro-profile at a world point (0 when disabled).
Real surfaceTextureHeight(const SurfaceProperties& s, Real x, Real y);

struct TireSlipState {
    // Transient ("deflection-based") slip quantities, integrated with the
    // relaxation-length ODEs. In steady state they equal the practical slip
    // ratio kappa = (omega*r - vx)/|vx| and tan(alpha) = -vy/|vx|.
    Real kappa = 0.0;
    Real tanAlpha = 0.0;
};

struct TireForces {
    Real fx = 0.0;          // N, longitudinal force in the wheel frame (+ forward)
    Real fy = 0.0;          // N, lateral force in the wheel frame (+ left)
    Real mz = 0.0;          // N*m, self-aligning torque (about wheel Z)
    Real dFxdKappa = 0.0;   // N per unit slip, local slope, used by the implicit wheel solve
    Real muX = 0.0;         // effective peak coefficients after load/surface scaling
    Real muY = 0.0;
};

class MagicFormulaTire {
public:
    explicit MagicFormulaTire(const TireParams& p) : p_(p) {}

    const TireParams& params() const { return p_; }

    // Steady-state combined-slip force for the given (transient) slip.
    // `speed` is the contact-point ground speed magnitude, used only for the wet falloff.
    // `camber` is the inclination angle to the ground normal, rad, top of the
    // wheel toward +y positive (only used when camberStiffness > 0).
    TireForces evaluate(Real fz, Real kappa, Real tanAlpha, const SurfaceProperties& surf, Real speed,
                        Real camber = 0.0) const;

    // Pure-slip helpers (exposed for tests and tooling).
    Real pureLongitudinal(Real fz, Real kappa, Real muScale) const;
    Real pureLateral(Real fz, Real alpha, Real muScale, Real camber = 0.0) const;

    // Advance the lateral transient slip state implicitly:
    //   sigma_y * d(tanAlpha)/dt + |vx| * tanAlpha = -vy
    // Stable for any dt; at vx = 0 the state integrates lateral displacement,
    // which gives the tire its static (standstill) stiffness.
    void integrateLateralSlip(TireSlipState& s, Real vx, Real vy, Real dt) const;

    static constexpr Real kSlipStateLimit = 10.0;  // keeps transient slip finite

private:
    TireParams p_;
};

}  // namespace ord
