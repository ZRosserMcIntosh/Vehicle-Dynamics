// OpenRoad Dynamics — Unreal Engine adapter.
// SPDX-License-Identifier: MIT

#include "OrdConversions.h"

namespace OrdConvert
{
ord::DrivingPreset ToCore(EOpenRoadPreset Preset)
{
	switch (Preset)
	{
	case EOpenRoadPreset::Fun: return ord::DrivingPreset::Fun;
	case EOpenRoadPreset::Sport: return ord::DrivingPreset::Sport;
	case EOpenRoadPreset::Simulation: return ord::DrivingPreset::Simulation;
	case EOpenRoadPreset::Drift: return ord::DrivingPreset::Drift;
	}
	return ord::DrivingPreset::Sport;
}

ord::InputDevice ToCore(EOpenRoadInputDevice Device)
{
	switch (Device)
	{
	case EOpenRoadInputDevice::Gamepad: return ord::InputDevice::Gamepad;
	case EOpenRoadInputDevice::SteeringWheel: return ord::InputDevice::SteeringWheel;
	case EOpenRoadInputDevice::Keyboard: return ord::InputDevice::Keyboard;
	}
	return ord::InputDevice::Gamepad;
}

ord::SteeringMode ToCore(EOpenRoadSteeringMode Mode)
{
	return Mode == EOpenRoadSteeringMode::SpeedSensitive ? ord::SteeringMode::SpeedSensitive : ord::SteeringMode::Direct;
}

ord::InterventionLevel ToCore(EOpenRoadIntervention Level)
{
	switch (Level)
	{
	case EOpenRoadIntervention::Off: return ord::InterventionLevel::Off;
	case EOpenRoadIntervention::Sport: return ord::InterventionLevel::Sport;
	case EOpenRoadIntervention::Full: return ord::InterventionLevel::Full;
	}
	return ord::InterventionLevel::Off;
}

ord::ShiftMode ToCore(EOpenRoadShiftMode Mode)
{
	return Mode == EOpenRoadShiftMode::Manual ? ord::ShiftMode::Manual : ord::ShiftMode::Automatic;
}

ord::AssistConfig MakeAssistConfig(EOpenRoadPreset Preset, EOpenRoadInputDevice Device, const FOpenRoadAssistOverrides& O)
{
	ord::AssistConfig C = ord::makePreset(ToCore(Preset), ToCore(Device));
	if (O.bOverrideSteeringMode) C.steeringMode = ToCore(O.SteeringMode);
	if (O.bOverrideSteeringExponent) C.steeringExponent = O.SteeringExponent;
	if (O.bOverrideSteerRate) C.steerRate = O.SteerRate;
	if (O.bOverrideThrottleRate) C.throttleRate = O.ThrottleRate;
	if (O.bOverrideAbs) C.abs = O.bAbs;
	if (O.bOverrideTractionControl) C.tractionControl = ToCore(O.TractionControl);
	if (O.bOverrideStabilityControl) C.stabilityControl = ToCore(O.StabilityControl);
	if (O.bOverrideCountersteerAssist) C.countersteerAssist = O.CountersteerAssist;
	if (O.bOverrideDriftAssist) C.driftAssist = O.DriftAssist;
	if (O.bOverrideShiftMode) C.shiftMode = ToCore(O.ShiftMode);
	if (O.bOverrideAutoClutch) C.autoClutch = O.bAutoClutch;
	return C;
}

ord::SurfaceProperties ToCore(const FOpenRoadSurface& S)
{
	ord::SurfaceProperties P;
	P.frictionScale = S.FrictionScale;
	P.rollingResistanceScale = S.RollingResistanceScale;
	P.wetSpeedFalloff = S.WetSpeedFalloff;
	P.roughness = S.Roughness;
	P.textureAmplitude = S.TextureAmplitude;
	P.textureWavelength = S.TextureWavelength;
	P.textureVariation = S.TextureVariation;
	return P;
}

ord::TireParams ToCore(const FOpenRoadTireParams& T)
{
	ord::TireParams P;
	P.radius = T.Radius;
	P.nominalLoad = T.NominalLoad;
	P.muX = T.MuX;
	P.muXLoadSensitivity = T.MuXLoadSensitivity;
	P.muY = T.MuY;
	P.muYLoadSensitivity = T.MuYLoadSensitivity;
	P.shapeX = T.ShapeX;
	P.curvatureX = T.CurvatureX;
	P.shapeY = T.ShapeY;
	P.curvatureY = T.CurvatureY;
	P.longStiffnessPerLoad = T.LongStiffnessPerLoad;
	P.cornerStiffness = T.CornerStiffness;
	P.cornerStiffnessLoad = T.CornerStiffnessLoad;
	P.relaxationLengthX = T.RelaxationLengthX;
	P.relaxationLengthY = T.RelaxationLengthY;
	P.rollingResistance = T.RollingResistance;
	P.pneumaticTrail = T.PneumaticTrail;
	P.wheelInertia = T.WheelInertia;
	P.verticalStiffness = T.VerticalStiffness;
	P.verticalDamping = T.VerticalDamping;
	P.patchHalfLength = T.PatchHalfLength;
	P.patchHalfWidth = T.PatchHalfWidth;
	return P;
}
}  // namespace OrdConvert
