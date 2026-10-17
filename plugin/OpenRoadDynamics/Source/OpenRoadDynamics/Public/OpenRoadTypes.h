// OpenRoad Dynamics — Unreal Engine adapter.
// SPDX-License-Identifier: MIT
//
// Blueprint-facing enums and the per-aid assist override struct. These mirror
// ord::DrivingPreset / ord::InputDevice / ord::AssistConfig (presets.hpp,
// assists.hpp); conversion lives in Private/OrdConversions.cpp.
#pragma once

#include "CoreMinimal.h"

#include "OpenRoadTypes.generated.h"

/** Named assist bundles. Presets never change the physical car (VehicleDefinition). */
UENUM(BlueprintType)
enum class EOpenRoadPreset : uint8
{
	Fun,
	Sport,
	Simulation,
	/** SPORT steering, no TCS/ESC, drift assist on (an aid with no real-car equivalent). */
	Drift,
};

/** Input device; presets choose different steering interpretation per device. */
UENUM(BlueprintType)
enum class EOpenRoadInputDevice : uint8
{
	Gamepad,
	SteeringWheel,
	Keyboard,
};

UENUM(BlueprintType)
enum class EOpenRoadSteeringMode : uint8
{
	/** Input maps linearly to road-wheel angle (wheels, "simulation steering"). */
	Direct,
	/** Gamepad-friendly: range limited by speed and peak slip angle, rate limited. */
	SpeedSensitive,
};

UENUM(BlueprintType)
enum class EOpenRoadIntervention : uint8
{
	Off,
	Sport,
	Full,
};

UENUM(BlueprintType)
enum class EOpenRoadShiftMode : uint8
{
	Automatic,
	Manual,
};

/**
 * Per-aid overrides applied on top of the selected preset. Every aid acts only
 * through real actuators (steering, throttle, brakes, clutch, gear).
 */
USTRUCT(BlueprintType)
struct OPENROADDYNAMICS_API FOpenRoadAssistOverrides
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Assists", meta = (InlineEditConditionToggle))
	bool bOverrideSteeringMode = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Assists", meta = (EditCondition = "bOverrideSteeringMode", ToolTip = "How steering input becomes road-wheel angle."))
	EOpenRoadSteeringMode SteeringMode = EOpenRoadSteeringMode::Direct;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Assists", meta = (InlineEditConditionToggle))
	bool bOverrideSteeringExponent = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Assists", meta = (EditCondition = "bOverrideSteeringExponent", ClampMin = "0.5", ClampMax = "3.0", ToolTip = "Input shaping |x|^e * sign(x). 1 = linear."))
	float SteeringExponent = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Assists", meta = (InlineEditConditionToggle))
	bool bOverrideSteerRate = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Assists", meta = (EditCondition = "bOverrideSteerRate", ClampMin = "0.0", ToolTip = "Max change of steering input per second (1/s). 0 = unlimited."))
	float SteerRate = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Assists", meta = (InlineEditConditionToggle))
	bool bOverrideThrottleRate = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Assists", meta = (EditCondition = "bOverrideThrottleRate", ClampMin = "0.0", ToolTip = "Max change of throttle per second (1/s). 0 = unlimited."))
	float ThrottleRate = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Assists", meta = (InlineEditConditionToggle))
	bool bOverrideAbs = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Assists", meta = (EditCondition = "bOverrideAbs", ToolTip = "Anti-lock brakes."))
	bool bAbs = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Assists", meta = (InlineEditConditionToggle))
	bool bOverrideTractionControl = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Assists", meta = (EditCondition = "bOverrideTractionControl", ToolTip = "Traction control intervention level."))
	EOpenRoadIntervention TractionControl = EOpenRoadIntervention::Off;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Assists", meta = (InlineEditConditionToggle))
	bool bOverrideStabilityControl = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Assists", meta = (EditCondition = "bOverrideStabilityControl", ToolTip = "Stability control (ESC) intervention level."))
	EOpenRoadIntervention StabilityControl = EOpenRoadIntervention::Off;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Assists", meta = (InlineEditConditionToggle))
	bool bOverrideCountersteerAssist = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Assists", meta = (EditCondition = "bOverrideCountersteerAssist", ClampMin = "0.0", ClampMax = "1.0", ToolTip = "0..1. Driving aid with no real-car equivalent: blends the front wheels toward the direction of travel in slides."))
	float CountersteerAssist = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Assists", meta = (InlineEditConditionToggle))
	bool bOverrideDriftAssist = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Assists", meta = (EditCondition = "bOverrideDriftAssist", ClampMin = "0.0", ClampMax = "1.0", ToolTip = "0..1. Driving aid with no real-car equivalent: in a slide, steering sets the drift angle; the aid countersteers and trims throttle to hold it."))
	float DriftAssist = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Assists", meta = (InlineEditConditionToggle))
	bool bOverrideShiftMode = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Assists", meta = (EditCondition = "bOverrideShiftMode", ToolTip = "Automatic or manual gear selection."))
	EOpenRoadShiftMode ShiftMode = EOpenRoadShiftMode::Automatic;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Assists", meta = (InlineEditConditionToggle))
	bool bOverrideAutoClutch = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Assists", meta = (EditCondition = "bOverrideAutoClutch", ToolTip = "When off, the driver must operate the clutch pedal (SetClutch)."))
	bool bAutoClutch = true;
};

/** Per-wheel readout of the latest physics step (UE conventions where they differ from the core). */
USTRUCT(BlueprintType)
struct OPENROADDYNAMICS_API FOpenRoadWheelState
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadOnly, Category = "Wheel")
	bool bContact = false;

	/** Tire normal load Fz, N. */
	UPROPERTY(BlueprintReadOnly, Category = "Wheel")
	float LoadN = 0.0f;

	/** Transient longitudinal slip ratio (dimensionless). */
	UPROPERTY(BlueprintReadOnly, Category = "Wheel")
	float SlipRatio = 0.0f;

	/** Transient slip angle, degrees. Sign mirrored from the core's ISO value (Y axis flipped into UE's frame). */
	UPROPERTY(BlueprintReadOnly, Category = "Wheel")
	float SlipAngleDeg = 0.0f;

	/** Friction utilisation sqrt(Fx^2 + Fy^2) / Fz (dimensionless; compare with the tire's mu). */
	UPROPERTY(BlueprintReadOnly, Category = "Wheel")
	float ForceOverLoad = 0.0f;

	/** Longitudinal tire force, N (+ = forward). */
	UPROPERTY(BlueprintReadOnly, Category = "Wheel")
	float LongitudinalForceN = 0.0f;

	/** Lateral tire force, N, POSITIVE = to the RIGHT (UE convention). */
	UPROPERTY(BlueprintReadOnly, Category = "Wheel")
	float LateralForceN = 0.0f;

	/** Suspension compression from full droop, cm. */
	UPROPERTY(BlueprintReadOnly, Category = "Wheel")
	float CompressionCm = 0.0f;

	/** Road-wheel steer angle, degrees, POSITIVE = RIGHT (UE convention). */
	UPROPERTY(BlueprintReadOnly, Category = "Wheel")
	float SteerAngleDeg = 0.0f;

	/** Wheel spin, rad/s (+ = rolling forward). */
	UPROPERTY(BlueprintReadOnly, Category = "Wheel")
	float AngularVelocity = 0.0f;

	UPROPERTY(BlueprintReadOnly, Category = "Wheel")
	bool bAbsActive = false;

	UPROPERTY(BlueprintReadOnly, Category = "Wheel")
	int32 SurfaceId = 0;
};

/** Effective assist settings (preset + overrides) and what the aids did in the latest step. */
USTRUCT(BlueprintType)
struct OPENROADDYNAMICS_API FOpenRoadAssistState
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadOnly, Category = "Assists")
	bool bAbs = false;

	UPROPERTY(BlueprintReadOnly, Category = "Assists")
	EOpenRoadIntervention TractionControl = EOpenRoadIntervention::Off;

	UPROPERTY(BlueprintReadOnly, Category = "Assists")
	EOpenRoadIntervention StabilityControl = EOpenRoadIntervention::Off;

	UPROPERTY(BlueprintReadOnly, Category = "Assists")
	EOpenRoadShiftMode ShiftMode = EOpenRoadShiftMode::Automatic;

	/** Any wheel under ABS modulation in the latest step. */
	UPROPERTY(BlueprintReadOnly, Category = "Assists")
	bool bAbsActive = false;

	/** Fraction of driver throttle removed by TCS, 0..1. */
	UPROPERTY(BlueprintReadOnly, Category = "Assists")
	float TcsThrottleCut = 0.0f;

	/** Any ESC brake torque in the latest step. */
	UPROPERTY(BlueprintReadOnly, Category = "Assists")
	bool bEscActive = false;
};

/** Coupling and ground-cache health. See docs/AUDIT.md for what each value verifies in a real build. */
USTRUCT(BlueprintType)
struct OPENROADDYNAMICS_API FOpenRoadDiagnostics
{
	GENERATED_BODY()

	/** Cumulative ground-cache misses of this vehicle (contact queries outside every strip; extrapolated). */
	UPROPERTY(BlueprintReadOnly, Category = "Ground")
	int64 GroundCacheMisses = 0;

	/** Misses during the physics steps consumed in the last game frame. */
	UPROPERTY(BlueprintReadOnly, Category = "Ground")
	int32 GroundCacheMissesLastFrame = 0;

	/** Strip horizon currently requested, s (adaptive). */
	UPROPERTY(BlueprintReadOnly, Category = "Ground")
	float GroundHorizonSeconds = 0.0f;

	/** Largest cache age seen by the physics thread in the last game frame, s. Must stay below the horizon. */
	UPROPERTY(BlueprintReadOnly, Category = "Ground")
	float GroundCacheAgeSeconds = 0.0f;

	/** Caches allocated outside the pool because every pooled cache was still referenced. */
	UPROPERTY(BlueprintReadOnly, Category = "Ground")
	int32 TransientCacheAllocations = 0;

	/** Physics steps (snapshots) consumed in the last game frame. */
	UPROPERTY(BlueprintReadOnly, Category = "Coupling")
	int32 PhysicsStepsLastFrame = 0;

	/** Host-integrated vs core-predicted chassis position at the latest step, cm (expect < 0.1 on smooth driving). */
	UPROPERTY(BlueprintReadOnly, Category = "Coupling")
	float HostSyncPositionErrorCm = 0.0f;

	UPROPERTY(BlueprintReadOnly, Category = "Coupling")
	float HostSyncAngleErrorDeg = 0.0f;

	UPROPERTY(BlueprintReadOnly, Category = "Coupling")
	float HostSyncMaxPositionErrorCm = 0.0f;

	/** |finite-difference velocity of the Chaos CoM - reported V|, cm/s. ~0 when V() is the CoM velocity. -1 = n/a. */
	UPROPERTY(BlueprintReadOnly, Category = "Coupling")
	float VelocityCheckComCmS = -1.0f;

	/** Same for the body origin. ~0 (and the CoM value not) would mean V() is the ORIGIN velocity. */
	UPROPERTY(BlueprintReadOnly, Category = "Coupling")
	float VelocityCheckOriginCmS = -1.0f;

	/** Distance between Chaos' CoM and the definition's CoM, cm (expect < 1). */
	UPROPERTY(BlueprintReadOnly, Category = "Mass")
	float ComMismatchCm = 0.0f;

	UPROPERTY(BlueprintReadOnly, Category = "Mass")
	float HostMassKg = 0.0f;

	/** Chaos principal inertia (mass-space diagonal), kg*m^2. */
	UPROPERTY(BlueprintReadOnly, Category = "Mass")
	FVector HostInertiaKgM2 = FVector::ZeroVector;

	/** Angle of Chaos' principal inertia frame vs the body axes, degrees (expect 0). */
	UPROPERTY(BlueprintReadOnly, Category = "Mass")
	float RotationOfMassDeg = 0.0f;

	/** A step produced non-finite forces (not applied). */
	UPROPERTY(BlueprintReadOnly, Category = "Coupling")
	bool bFaulted = false;
};
