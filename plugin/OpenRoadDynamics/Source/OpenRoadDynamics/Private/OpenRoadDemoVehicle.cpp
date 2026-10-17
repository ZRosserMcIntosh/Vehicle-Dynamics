// OpenRoad Dynamics — Unreal Engine adapter.
// SPDX-License-Identifier: MIT
//
// Proving-ground demo pawn. NOT COMPILED here (no Unreal Engine in CI). Every
// engine call is standard UE 5.4 gameplay API (components, Enhanced Input,
// GEngine on-screen messages); see docs/PROVING_GROUND.md "Troubleshooting".

#include "OpenRoadDemoVehicle.h"

#include "Camera/CameraComponent.h"
#include "Components/BoxComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/CollisionProfile.h"
#include "Engine/Engine.h"
#include "Engine/LocalPlayer.h"
#include "Engine/StaticMesh.h"
#include "EnhancedInputComponent.h"
#include "EnhancedInputSubsystems.h"
#include "GameFramework/PlayerController.h"
#include "GameFramework/SpringArmComponent.h"
#include "InputAction.h"
#include "InputCoreTypes.h"
#include "InputMappingContext.h"
#include "InputModifiers.h"
#include "OpenRoadLog.h"
#include "OpenRoadVehicleComponent.h"
#include "OpenRoadVehicleDefinition.h"
#include "UObject/ConstructorHelpers.h"

namespace
{
const FName NThrottle(TEXT("Throttle"));
const FName NBrake(TEXT("Brake"));
const FName NSteerKeyboard(TEXT("SteerKeyboard"));
const FName NSteerGamepad(TEXT("SteerGamepad"));
const FName NHandbrake(TEXT("Handbrake"));
const FName NPresetFun(TEXT("PresetFun"));
const FName NPresetSport(TEXT("PresetSport"));
const FName NPresetSimulation(TEXT("PresetSimulation"));
const FName NPresetDrift(TEXT("PresetDrift"));
const FName NAbs(TEXT("ToggleAbs"));
const FName NTcs(TEXT("CycleTcs"));
const FName NEsc(TEXT("CycleEsc"));
const FName NShiftUp(TEXT("ShiftUp"));
const FName NShiftDown(TEXT("ShiftDown"));
const FName NShiftMode(TEXT("ToggleShiftMode"));
const FName NDebug(TEXT("ToggleDebug"));
const FName NTelemetry(TEXT("ToggleTelemetry"));
const FName NManeuverAccel(TEXT("ManeuverAccelBrake"));
const FName NManeuverStep(TEXT("ManeuverStepSteer"));

const TCHAR* WheelNames[4] = {TEXT("FL"), TEXT("FR"), TEXT("RL"), TEXT("RR")};

const TCHAR* PresetName(EOpenRoadPreset P)
{
	switch (P)
	{
	case EOpenRoadPreset::Fun: return TEXT("FUN");
	case EOpenRoadPreset::Sport: return TEXT("SPORT");
	case EOpenRoadPreset::Simulation: return TEXT("SIMULATION");
	case EOpenRoadPreset::Drift: return TEXT("DRIFT");
	}
	return TEXT("?");
}

const TCHAR* LevelName(EOpenRoadIntervention L)
{
	switch (L)
	{
	case EOpenRoadIntervention::Off: return TEXT("OFF");
	case EOpenRoadIntervention::Sport: return TEXT("SPORT");
	case EOpenRoadIntervention::Full: return TEXT("FULL");
	}
	return TEXT("?");
}

EOpenRoadIntervention NextLevel(EOpenRoadIntervention L)
{
	switch (L)
	{
	case EOpenRoadIntervention::Off: return EOpenRoadIntervention::Sport;
	case EOpenRoadIntervention::Sport: return EOpenRoadIntervention::Full;
	case EOpenRoadIntervention::Full: return EOpenRoadIntervention::Off;
	}
	return EOpenRoadIntervention::Off;
}

constexpr uint64 ReadoutKey = 0x0D0A0D11ull;  // one keyed, multi-line on-screen message
}  // namespace

// ============================================================================
// Construction
// ============================================================================

AOpenRoadDemoVehicle::AOpenRoadDemoVehicle()
{
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.TickGroup = TG_PostPhysics;  // visuals/readout after this frame's physics results

	static ConstructorHelpers::FObjectFinder<UStaticMesh> CubeMesh(TEXT("/Engine/BasicShapes/Cube.Cube"));
	static ConstructorHelpers::FObjectFinder<UStaticMesh> CylinderMesh(TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));

	// Chassis: the Chaos body. Scale 1 (the vehicle component requires it); size via the box extent.
	Chassis = CreateDefaultSubobject<UBoxComponent>(TEXT("Chassis"));
	Chassis->SetBoxExtent(FVector(220.0, 90.0, 30.0));
	Chassis->SetCollisionProfileName(UCollisionProfile::PhysicsActor_ProfileName);
	Chassis->SetSimulatePhysics(true);
	Chassis->SetHiddenInGame(true);
	RootComponent = Chassis;

	auto MakeVisual = [this](const TCHAR* Name, UStaticMesh* Mesh) {
		UStaticMeshComponent* C = CreateDefaultSubobject<UStaticMeshComponent>(Name);
		C->SetupAttachment(Chassis);
		C->SetStaticMesh(Mesh);
		C->SetCollisionEnabled(ECollisionEnabled::NoCollision);  // visual only: Chaos sees the box, the core casts rays
		C->SetGenerateOverlapEvents(false);
		C->SetCanEverAffectNavigation(false);
		return C;
	};

	BodyMesh = MakeVisual(TEXT("BodyMesh"), CubeMesh.Object);
	BodyMesh->SetRelativeScale3D(FVector(4.4, 1.8, 0.6));  // basic-shape cube is 100 cm
	CabinMesh = MakeVisual(TEXT("CabinMesh"), CubeMesh.Object);
	CabinMesh->SetRelativeLocation(FVector(-25.0, 0.0, 52.0));
	CabinMesh->SetRelativeScale3D(FVector(2.0, 1.5, 0.45));

	for (int32 I = 0; I < 4; ++I)
	{
		UStaticMeshComponent* Wheel = MakeVisual(*FString::Printf(TEXT("Wheel%s"), WheelNames[I]), CylinderMesh.Object);
		Wheel->SetRelativeScale3D(FVector(0.66, 0.66, 0.24));  // basic-shape cylinder: 100 cm tall, 50 cm radius
		WheelMeshes.Add(Wheel);
	}

	SpringArm = CreateDefaultSubobject<USpringArmComponent>(TEXT("SpringArm"));
	SpringArm->SetupAttachment(Chassis);
	SpringArm->TargetArmLength = 700.0f;
	SpringArm->SocketOffset = FVector(0.0, 0.0, 180.0);
	SpringArm->SetRelativeRotation(FRotator(-10.0, 0.0, 0.0));
	SpringArm->bInheritPitch = false;
	SpringArm->bInheritRoll = false;
	SpringArm->bDoCollisionTest = false;
	SpringArm->bEnableCameraLag = true;
	SpringArm->CameraLagSpeed = 8.0f;
	SpringArm->bEnableCameraRotationLag = true;
	SpringArm->CameraRotationLagSpeed = 6.0f;

	Camera = CreateDefaultSubobject<UCameraComponent>(TEXT("Camera"));
	Camera->SetupAttachment(SpringArm, USpringArmComponent::SocketName);

	Vehicle = CreateDefaultSubobject<UOpenRoadVehicleComponent>(TEXT("OpenRoadVehicle"));
	Vehicle->InputDevice = EOpenRoadInputDevice::Keyboard;
	Vehicle->Preset = EOpenRoadPreset::Sport;
}

void AOpenRoadDemoVehicle::BeginPlay()
{
	// Before Super::BeginPlay(): the vehicle component reads its definition in its own BeginPlay.
	if (!Vehicle->Definition)
	{
		// Asset defaults (generic 1450 kg RWD sports car, origin on the ground)
		// shifted so that positions are relative to the chassis-box centre.
		UOpenRoadVehicleDefinition* Def = NewObject<UOpenRoadVehicleDefinition>(this, TEXT("DemoDefinition"), RF_Transient);
		Def->VehicleName = TEXT("OpenRoad demo RWD");
		Def->CenterOfMass.Z -= ChassisCenterHeightCm;
		for (FOpenRoadAxle& Axle : Def->Axles)
		{
			Axle.Hardpoint.Z -= ChassisCenterHeightCm;
		}
		Vehicle->Definition = Def;
	}
	if (bAutomaticTransmission)
	{
		Vehicle->SetShiftMode(EOpenRoadShiftMode::Automatic);
	}
	Super::BeginPlay();
}

// ============================================================================
// Enhanced Input, created at runtime (no assets)
// ============================================================================

void AOpenRoadDemoVehicle::EnsureInputAssets()
{
	if (MappingContext)
	{
		return;
	}
	MappingContext = NewObject<UInputMappingContext>(this, TEXT("IMC_OpenRoadDemo"), RF_Transient);

	auto MakeAction = [this](FName Name, EInputActionValueType Type) {
		UInputAction* Action = NewObject<UInputAction>(this, *(TEXT("IA_") + Name.ToString()), RF_Transient);
		Action->ValueType = Type;
		Actions.Add(Name, Action);
		return Action;
	};
	auto Map = [this](UInputAction* Action, const FKey& Key, bool bNegate = false, float DeadZone = 0.0f) {
		FEnhancedActionKeyMapping& Mapping = MappingContext->MapKey(Action, Key);
		if (DeadZone > 0.0f)
		{
			UInputModifierDeadZone* Dz = NewObject<UInputModifierDeadZone>(MappingContext);
			Dz->LowerThreshold = DeadZone;
			Mapping.Modifiers.Add(Dz);
		}
		if (bNegate)
		{
			Mapping.Modifiers.Add(NewObject<UInputModifierNegate>(MappingContext));
		}
	};

	UInputAction* Throttle = MakeAction(NThrottle, EInputActionValueType::Axis1D);
	Map(Throttle, EKeys::W);
	Map(Throttle, EKeys::Up);
	Map(Throttle, EKeys::Gamepad_RightTriggerAxis, false, 0.05f);

	UInputAction* Brake = MakeAction(NBrake, EInputActionValueType::Axis1D);
	Map(Brake, EKeys::S);
	Map(Brake, EKeys::Down);
	Map(Brake, EKeys::Gamepad_LeftTriggerAxis, false, 0.05f);

	// Separate keyboard/gamepad steering actions so the assists get the right device.
	UInputAction* SteerKeys = MakeAction(NSteerKeyboard, EInputActionValueType::Axis1D);
	Map(SteerKeys, EKeys::D);
	Map(SteerKeys, EKeys::Right);
	Map(SteerKeys, EKeys::A, /*bNegate*/ true);
	Map(SteerKeys, EKeys::Left, /*bNegate*/ true);
	UInputAction* SteerPad = MakeAction(NSteerGamepad, EInputActionValueType::Axis1D);
	Map(SteerPad, EKeys::Gamepad_LeftX, false, 0.08f);

	UInputAction* Handbrake = MakeAction(NHandbrake, EInputActionValueType::Boolean);
	Map(Handbrake, EKeys::SpaceBar);
	Map(Handbrake, EKeys::Gamepad_FaceButton_Right);

	Map(MakeAction(NPresetFun, EInputActionValueType::Boolean), EKeys::One);
	Map(Actions[NPresetFun], EKeys::Gamepad_DPad_Left);
	Map(MakeAction(NPresetSport, EInputActionValueType::Boolean), EKeys::Two);
	Map(Actions[NPresetSport], EKeys::Gamepad_DPad_Up);
	Map(MakeAction(NPresetSimulation, EInputActionValueType::Boolean), EKeys::Three);
	Map(Actions[NPresetSimulation], EKeys::Gamepad_DPad_Right);
	Map(MakeAction(NPresetDrift, EInputActionValueType::Boolean), EKeys::Four);

	Map(MakeAction(NAbs, EInputActionValueType::Boolean), EKeys::B);
	Map(Actions[NAbs], EKeys::Gamepad_FaceButton_Left);
	Map(MakeAction(NTcs, EInputActionValueType::Boolean), EKeys::T);
	Map(Actions[NTcs], EKeys::Gamepad_FaceButton_Top);
	Map(MakeAction(NEsc, EInputActionValueType::Boolean), EKeys::E);
	Map(Actions[NEsc], EKeys::Gamepad_DPad_Down);

	Map(MakeAction(NShiftUp, EInputActionValueType::Boolean), EKeys::R);
	Map(Actions[NShiftUp], EKeys::Gamepad_RightShoulder);
	Map(MakeAction(NShiftDown, EInputActionValueType::Boolean), EKeys::F);
	Map(Actions[NShiftDown], EKeys::Gamepad_LeftShoulder);
	Map(MakeAction(NShiftMode, EInputActionValueType::Boolean), EKeys::M);

	// F1/F2 can be taken by editor viewport hotkeys in PIE; G and C are aliases.
	Map(MakeAction(NDebug, EInputActionValueType::Boolean), EKeys::F1);
	Map(Actions[NDebug], EKeys::G);
	Map(Actions[NDebug], EKeys::Gamepad_Special_Left);
	Map(MakeAction(NTelemetry, EInputActionValueType::Boolean), EKeys::F2);
	Map(Actions[NTelemetry], EKeys::C);
	Map(Actions[NTelemetry], EKeys::Gamepad_Special_Right);

	Map(MakeAction(NManeuverAccel, EInputActionValueType::Boolean), EKeys::Z);
	Map(MakeAction(NManeuverStep, EInputActionValueType::Boolean), EKeys::X);
}

void AOpenRoadDemoVehicle::PawnClientRestart()
{
	Super::PawnClientRestart();
	EnsureInputAssets();
	if (const APlayerController* PC = Cast<APlayerController>(GetController()))
	{
		if (const ULocalPlayer* LocalPlayer = PC->GetLocalPlayer())
		{
			if (UEnhancedInputLocalPlayerSubsystem* Subsystem = LocalPlayer->GetSubsystem<UEnhancedInputLocalPlayerSubsystem>())
			{
				Subsystem->AddMappingContext(MappingContext, /*Priority*/ 0);
			}
		}
	}
}

void AOpenRoadDemoVehicle::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);
	EnsureInputAssets();
	UEnhancedInputComponent* Input = Cast<UEnhancedInputComponent>(PlayerInputComponent);
	if (!Input)
	{
		UE_LOG(LogOpenRoad, Error,
		       TEXT("OpenRoad demo: the input component is not a UEnhancedInputComponent. Set Project Settings > Input > "
		            "Default Input Component Class = EnhancedInputComponent (and Default Player Input Class = EnhancedPlayerInput)."));
		return;
	}
	Input->BindAction(Actions[NThrottle], ETriggerEvent::Triggered, this, &ThisClass::OnThrottle);
	Input->BindAction(Actions[NThrottle], ETriggerEvent::Completed, this, &ThisClass::OnThrottleEnd);
	Input->BindAction(Actions[NBrake], ETriggerEvent::Triggered, this, &ThisClass::OnBrake);
	Input->BindAction(Actions[NBrake], ETriggerEvent::Completed, this, &ThisClass::OnBrakeEnd);
	Input->BindAction(Actions[NSteerKeyboard], ETriggerEvent::Triggered, this, &ThisClass::OnSteerKeyboard);
	Input->BindAction(Actions[NSteerKeyboard], ETriggerEvent::Completed, this, &ThisClass::OnSteerEnd);
	Input->BindAction(Actions[NSteerGamepad], ETriggerEvent::Triggered, this, &ThisClass::OnSteerGamepad);
	Input->BindAction(Actions[NSteerGamepad], ETriggerEvent::Completed, this, &ThisClass::OnSteerEnd);
	Input->BindAction(Actions[NHandbrake], ETriggerEvent::Started, this, &ThisClass::OnHandbrake);
	Input->BindAction(Actions[NHandbrake], ETriggerEvent::Completed, this, &ThisClass::OnHandbrakeEnd);
	Input->BindAction(Actions[NPresetFun], ETriggerEvent::Started, this, &ThisClass::OnPresetFun);
	Input->BindAction(Actions[NPresetSport], ETriggerEvent::Started, this, &ThisClass::OnPresetSport);
	Input->BindAction(Actions[NPresetSimulation], ETriggerEvent::Started, this, &ThisClass::OnPresetSimulation);
	Input->BindAction(Actions[NPresetDrift], ETriggerEvent::Started, this, &ThisClass::OnPresetDrift);
	Input->BindAction(Actions[NAbs], ETriggerEvent::Started, this, &ThisClass::OnToggleAbs);
	Input->BindAction(Actions[NTcs], ETriggerEvent::Started, this, &ThisClass::OnCycleTcs);
	Input->BindAction(Actions[NEsc], ETriggerEvent::Started, this, &ThisClass::OnCycleEsc);
	Input->BindAction(Actions[NShiftUp], ETriggerEvent::Started, this, &ThisClass::OnShiftUp);
	Input->BindAction(Actions[NShiftDown], ETriggerEvent::Started, this, &ThisClass::OnShiftDown);
	Input->BindAction(Actions[NShiftMode], ETriggerEvent::Started, this, &ThisClass::OnToggleShiftMode);
	Input->BindAction(Actions[NDebug], ETriggerEvent::Started, this, &ThisClass::OnToggleDebug);
	Input->BindAction(Actions[NTelemetry], ETriggerEvent::Started, this, &ThisClass::OnToggleTelemetry);
	Input->BindAction(Actions[NManeuverAccel], ETriggerEvent::Started, this, &ThisClass::OnManeuverAccelBrake);
	Input->BindAction(Actions[NManeuverStep], ETriggerEvent::Started, this, &ThisClass::OnManeuverStepSteer);
}

// ============================================================================
// Input handlers
// ============================================================================

void AOpenRoadDemoVehicle::UseDevice(bool bGamepad)
{
	if (bGamepad != bGamepadActive)
	{
		bGamepadActive = bGamepad;
		Vehicle->SetInputDevice(bGamepad ? EOpenRoadInputDevice::Gamepad : EOpenRoadInputDevice::Keyboard);
	}
}

void AOpenRoadDemoVehicle::OnThrottle(const FInputActionValue& Value)
{
	if (!IsManual())
	{
		return;  // a scripted manoeuvre owns the controls
	}
	ThrottleValue = FMath::Clamp(Value.Get<float>(), 0.0f, 1.0f);
	Vehicle->SetThrottle(ThrottleValue);
}
void AOpenRoadDemoVehicle::OnThrottleEnd(const FInputActionValue&)
{
	if (!IsManual())
	{
		return;  // a scripted manoeuvre owns the controls
	}
	ThrottleValue = 0.0f;
	Vehicle->SetThrottle(0.0f);
}
void AOpenRoadDemoVehicle::OnBrake(const FInputActionValue& Value)
{
	if (!IsManual())
	{
		return;  // a scripted manoeuvre owns the controls
	}
	BrakeValue = FMath::Clamp(Value.Get<float>(), 0.0f, 1.0f);
	Vehicle->SetBrake(BrakeValue);
}
void AOpenRoadDemoVehicle::OnBrakeEnd(const FInputActionValue&)
{
	if (!IsManual())
	{
		return;  // a scripted manoeuvre owns the controls
	}
	BrakeValue = 0.0f;
	Vehicle->SetBrake(0.0f);
}
void AOpenRoadDemoVehicle::OnSteerKeyboard(const FInputActionValue& Value)
{
	if (!IsManual())
	{
		return;  // a scripted manoeuvre owns the controls
	}
	UseDevice(false);
	SteerValue = FMath::Clamp(Value.Get<float>(), -1.0f, 1.0f);
	Vehicle->SetSteering(SteerValue);  // + = right
}
void AOpenRoadDemoVehicle::OnSteerGamepad(const FInputActionValue& Value)
{
	if (!IsManual())
	{
		return;  // a scripted manoeuvre owns the controls
	}
	UseDevice(true);
	SteerValue = FMath::Clamp(Value.Get<float>(), -1.0f, 1.0f);
	Vehicle->SetSteering(SteerValue);
}
void AOpenRoadDemoVehicle::OnSteerEnd(const FInputActionValue&)
{
	if (!IsManual())
	{
		return;  // a scripted manoeuvre owns the controls
	}
	SteerValue = 0.0f;
	Vehicle->SetSteering(0.0f);
}
void AOpenRoadDemoVehicle::OnHandbrake(const FInputActionValue&)
{
	if (!IsManual())
	{
		return;  // a scripted manoeuvre owns the controls
	}
	bHandbrake = true;
	Vehicle->SetHandbrake(1.0f);
}
void AOpenRoadDemoVehicle::OnHandbrakeEnd(const FInputActionValue&)
{
	if (!IsManual())
	{
		return;  // a scripted manoeuvre owns the controls
	}
	bHandbrake = false;
	Vehicle->SetHandbrake(0.0f);
}

void AOpenRoadDemoVehicle::OnPresetFun() { Vehicle->SetPreset(EOpenRoadPreset::Fun); }
void AOpenRoadDemoVehicle::OnPresetSport() { Vehicle->SetPreset(EOpenRoadPreset::Sport); }
void AOpenRoadDemoVehicle::OnPresetSimulation() { Vehicle->SetPreset(EOpenRoadPreset::Simulation); }
void AOpenRoadDemoVehicle::OnPresetDrift() { Vehicle->SetPreset(EOpenRoadPreset::Drift); }

void AOpenRoadDemoVehicle::OnToggleAbs() { Vehicle->SetAbsEnabled(!Vehicle->GetAssistState().bAbs); }
void AOpenRoadDemoVehicle::OnCycleTcs() { Vehicle->SetTractionControl(NextLevel(Vehicle->GetAssistState().TractionControl)); }
void AOpenRoadDemoVehicle::OnCycleEsc() { Vehicle->SetStabilityControl(NextLevel(Vehicle->GetAssistState().StabilityControl)); }
void AOpenRoadDemoVehicle::OnShiftUp() { Vehicle->ShiftUp(); }
void AOpenRoadDemoVehicle::OnShiftDown() { Vehicle->ShiftDown(); }

void AOpenRoadDemoVehicle::OnToggleShiftMode()
{
	const bool bManual = Vehicle->GetAssistState().ShiftMode == EOpenRoadShiftMode::Manual;
	Vehicle->SetShiftMode(bManual ? EOpenRoadShiftMode::Automatic : EOpenRoadShiftMode::Manual);
}

void AOpenRoadDemoVehicle::OnToggleDebug() { Vehicle->SetDebugDrawEnabled(!Vehicle->IsDebugDrawEnabled()); }

void AOpenRoadDemoVehicle::OnToggleTelemetry()
{
	if (Vehicle->IsCapturingTelemetry())
	{
		LastTelemetryFile = Vehicle->StopTelemetryCapture();
		TelemetryStartTime = -1.0;
	}
	else
	{
		Vehicle->StartTelemetryCapture();
		TelemetryStartTime = GetWorld() ? GetWorld()->GetTimeSeconds() : 0.0;
	}
}

// ============================================================================
// Scripted parity manoeuvres (inputs as in src/scenarios/scenarios.cpp)
// ============================================================================

void AOpenRoadDemoVehicle::OnManeuverAccelBrake()
{
	IsManual() ? StartManeuver(EOpenRoadDemoManeuver::AccelBrake) : StopManeuver(false);
}

void AOpenRoadDemoVehicle::OnManeuverStepSteer()
{
	IsManual() ? StartManeuver(EOpenRoadDemoManeuver::StepSteer) : StopManeuver(false);
}

void AOpenRoadDemoVehicle::StartManeuver(EOpenRoadDemoManeuver NewManeuver)
{
	if (Vehicle->IsCapturingTelemetry())
	{
		LastTelemetryFile = Vehicle->StopTelemetryCapture();
	}
	Maneuver = NewManeuver;
	ManeuverPhase = 0;
	ManeuverTimer = 0.0f;
	ManeuverStatus = TEXT("preparing");
	bHandbrake = false;
	Vehicle->SetHandbrake(0.0f);
}

void AOpenRoadDemoVehicle::StopManeuver(bool bWriteTelemetry)
{
	if (Vehicle->IsCapturingTelemetry())
	{
		const FString File = Vehicle->StopTelemetryCapture();
		if (bWriteTelemetry)
		{
			LastTelemetryFile = File;
		}
	}
	TelemetryStartTime = -1.0;
	Maneuver = EOpenRoadDemoManeuver::None;
	ThrottleValue = BrakeValue = SteerValue = 0.0f;
	Vehicle->SetThrottle(0.0f);
	Vehicle->SetBrake(0.0f);
	Vehicle->SetSteering(0.0f);
	ManeuverStatus = bWriteTelemetry ? TEXT("done") : TEXT("cancelled");
}

void AOpenRoadDemoVehicle::UpdateManeuver(float DeltaSeconds)
{
	if (IsManual())
	{
		return;
	}
	const float SpeedKmh = Vehicle->GetSpeedKmh();
	const float SpeedMs = SpeedKmh / 3.6f;
	ManeuverTimer += DeltaSeconds;
	auto Begin = [this]() {
		Vehicle->StartTelemetryCapture();
		TelemetryStartTime = GetWorld() ? GetWorld()->GetTimeSeconds() : 0.0;
	};
	auto Hold = [this, SpeedMs](float TargetMs) {
		// ord::scenarios::holdSpeed()
		const float Err = TargetMs - SpeedMs;
		ThrottleValue = FMath::Clamp(0.25f + 0.4f * Err, 0.0f, 1.0f);
		BrakeValue = Err < -1.5f ? FMath::Clamp(-0.1f * Err, 0.0f, 1.0f) : 0.0f;
	};

	if (Maneuver == EOpenRoadDemoManeuver::AccelBrake)
	{
		switch (ManeuverPhase)
		{
		case 0:  // come to rest
			ThrottleValue = 0.0f;
			BrakeValue = 1.0f;
			SteerValue = 0.0f;
			ManeuverStatus = TEXT("stopping");
			if (FMath::Abs(SpeedKmh) < 0.2f && ManeuverTimer > 1.0f)
			{
				Begin();
				ManeuverPhase = 1;
			}
			break;
		case 1:  // accelerate0to100: full throttle, automatic shifting
			ThrottleValue = 1.0f;
			BrakeValue = 0.0f;
			ManeuverStatus = TEXT("0-100 km/h");
			if (SpeedKmh >= 100.0f)
			{
				ManeuverPhase = 2;
			}
			break;
		default:  // brake100to0: full brake
			ThrottleValue = 0.0f;
			BrakeValue = 1.0f;
			ManeuverStatus = TEXT("100-0 km/h");
			if (SpeedKmh < 0.5f)
			{
				StopManeuver(true);
				return;
			}
			break;
		}
	}
	else  // StepSteer
	{
		const float Target = 80.0f / 3.6f;
		switch (ManeuverPhase)
		{
		case 0:  // reach 80 km/h straight
			Hold(Target);
			SteerValue = 0.0f;
			ManeuverStatus = TEXT("reaching 80 km/h");
			if (FMath::Abs(SpeedMs - Target) < 0.3f)
			{
				ManeuverPhase = 1;
				ManeuverTimer = 0.0f;
			}
			break;
		case 1:  // hold speed 2 s (as the scenario does), recording
			if (ManeuverTimer < DeltaSeconds * 1.5f)
			{
				Begin();
			}
			Hold(Target);
			ManeuverStatus = TEXT("holding 80 km/h");
			if (ManeuverTimer >= 2.0f)
			{
				ManeuverThrottle = ThrottleValue;  // frozen, brake released, like stepSteer()
				ManeuverPhase = 2;
				ManeuverTimer = 0.0f;
			}
			break;
		default:  // step: core steer +0.08 (left) == UE SetSteering(-0.08), 4 s
			ThrottleValue = ManeuverThrottle;
			BrakeValue = 0.0f;
			SteerValue = -0.08f;
			ManeuverStatus = TEXT("step steer");
			if (ManeuverTimer >= 4.0f)
			{
				StopManeuver(true);
				return;
			}
			break;
		}
	}
	Vehicle->SetThrottle(ThrottleValue);
	Vehicle->SetBrake(BrakeValue);
	Vehicle->SetSteering(SteerValue);
}

// ============================================================================
// Per frame: wheel visuals and readout
// ============================================================================

void AOpenRoadDemoVehicle::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	UpdateManeuver(DeltaSeconds);
	UpdateWheelVisuals(DeltaSeconds);
	if (bShowReadout)
	{
		ShowReadout();
	}
}

void AOpenRoadDemoVehicle::UpdateWheelVisuals(float DeltaSeconds)
{
	const UOpenRoadVehicleDefinition* Def = Vehicle->Definition;
	if (!Def)
	{
		return;
	}
	for (int32 I = 0; I < WheelMeshes.Num() && I / 2 < Def->Axles.Num(); ++I)
	{
		const FOpenRoadAxle& Axle = Def->Axles[I / 2];
		const bool bLeft = (I % 2) == 0;
		FOpenRoadWheelState W;
		const bool bHave = Vehicle->GetWheelState(I, W);
		// Wheel centre: hardpoint (full bump) moved down by the remaining travel.
		const float Droop = Axle.Travel * 100.0f - (bHave ? W.CompressionCm : 0.0f);
		const FVector Local(Axle.Hardpoint.X, bLeft ? -FMath::Abs(Axle.Hardpoint.Y) : FMath::Abs(Axle.Hardpoint.Y),
		                    Axle.Hardpoint.Z - Droop);
		if (bHave)
		{
			WheelSpinDeg[I] = FMath::Fmod(WheelSpinDeg[I] + FMath::RadiansToDegrees(W.AngularVelocity) * DeltaSeconds, 360.0f);
		}
		// Cylinder axis (local Z) -> lateral axis, then spin about it, then steer about the vertical.
		const FQuat Steer(FRotator(0.0, bHave ? W.SteerAngleDeg : 0.0f, 0.0));
		const FQuat Spin(FRotator(-WheelSpinDeg[I], 0.0, 0.0));
		const FQuat AxleToY(FRotator(0.0, 0.0, 90.0));
		WheelMeshes[I]->SetRelativeLocationAndRotation(Local, Steer * Spin * AxleToY);
	}
}

void AOpenRoadDemoVehicle::ShowReadout() const
{
	if (!GEngine)
	{
		return;
	}
	const FOpenRoadAssistState A = Vehicle->GetAssistState();
	const FOpenRoadDiagnostics D = Vehicle->GetDiagnostics();

	FString Text;
	Text += FString::Printf(TEXT("OpenRoad Dynamics demo   preset %s   device %s   gearbox %s%s\n"),
	                        PresetName(Vehicle->GetPreset()), bGamepadActive ? TEXT("gamepad") : TEXT("keyboard"),
	                        A.ShiftMode == EOpenRoadShiftMode::Automatic ? TEXT("AUTO") : TEXT("MANUAL"),
	                        Vehicle->IsVehicleActive() ? TEXT("") : TEXT("   [VEHICLE INACTIVE - see Output Log]"));
	Text += FString::Printf(TEXT("%6.1f km/h   gear %d   %5.0f rpm   thr %.2f  brk %.2f  steer %+.2f  hb %d\n"),
	                        Vehicle->GetSpeedKmh(), Vehicle->GetGear(), Vehicle->GetEngineRpm(), ThrottleValue, BrakeValue,
	                        SteerValue, bHandbrake ? 1 : 0);
	Text += FString::Printf(TEXT("ABS %s%s   TCS %s (cut %.2f)   ESC %s%s\n"), A.bAbs ? TEXT("ON") : TEXT("OFF"),
	                        A.bAbsActive ? TEXT(" [active]") : TEXT(""), LevelName(A.TractionControl), A.TcsThrottleCut,
	                        LevelName(A.StabilityControl), A.bEscActive ? TEXT(" [active]") : TEXT(""));
	Text += TEXT("wheel  load N   slip ratio  slip deg  F/Fz   comp cm  contact\n");
	for (int32 I = 0; I < FMath::Min(4, Vehicle->GetNumWheels()); ++I)
	{
		FOpenRoadWheelState W;
		if (Vehicle->GetWheelState(I, W))
		{
			Text += FString::Printf(TEXT("%s   %7.0f   %+8.3f   %+7.2f   %5.2f   %6.1f   %s%s\n"), WheelNames[I], W.LoadN, W.SlipRatio,
			                        W.SlipAngleDeg, W.ForceOverLoad, W.CompressionCm, W.bContact ? TEXT("yes") : TEXT("no"),
			                        W.bAbsActive ? TEXT(" ABS") : TEXT(""));
		}
	}
	Text += FString::Printf(TEXT("ground: horizon %.3f s  cache age %.3f s  misses %lld (+%d)  steps/frame %d  transient caches %d\n"),
	                        D.GroundHorizonSeconds, D.GroundCacheAgeSeconds, D.GroundCacheMisses, D.GroundCacheMissesLastFrame,
	                        D.PhysicsStepsLastFrame, D.TransientCacheAllocations);
	Text += FString::Printf(TEXT("coupling: sync %.3f cm / %.3f deg (max %.3f cm)  V-check CoM %.2f / origin %.2f cm/s  CoM off %.1f cm  mass %.0f kg  rotMass %.1f deg%s\n"),
	                        D.HostSyncPositionErrorCm, D.HostSyncAngleErrorDeg, D.HostSyncMaxPositionErrorCm, D.VelocityCheckComCmS,
	                        D.VelocityCheckOriginCmS, D.ComMismatchCm, D.HostMassKg, D.RotationOfMassDeg,
	                        D.bFaulted ? TEXT("  FAULTED") : TEXT(""));
	if (Vehicle->IsCapturingTelemetry())
	{
		const double Elapsed = GetWorld() ? GetWorld()->GetTimeSeconds() - TelemetryStartTime : 0.0;
		Text += FString::Printf(TEXT("telemetry: RECORDING %.1f s (F2/C to stop and write CSV)\n"), Elapsed);
	}
	else if (!LastTelemetryFile.IsEmpty())
	{
		Text += FString::Printf(TEXT("telemetry: wrote %s\n"), *LastTelemetryFile);
	}
	if (!IsManual() || !ManeuverStatus.IsEmpty())
	{
		Text += FString::Printf(TEXT("manoeuvre: %s %s%s\n"),
		                        Maneuver == EOpenRoadDemoManeuver::AccelBrake ? TEXT("0-100-0") :
		                        Maneuver == EOpenRoadDemoManeuver::StepSteer  ? TEXT("step steer 80 km/h") : TEXT(""),
		                        *ManeuverStatus,
		                        Vehicle->GetPreset() != EOpenRoadPreset::Simulation ? TEXT("  (parity runs use preset 3, SIMULATION)") : TEXT(""));
	}
	Text += TEXT("W/S A/D Space | 1 2 3 presets | B ABS  T TCS  E ESC | R/F shift  M auto/manual | F1/G debug  F2/C telemetry | Z 0-100-0  X step steer");
	GEngine->AddOnScreenDebugMessage(ReadoutKey, /*TimeToDisplay*/ 0.0f, FColor::White, Text, /*bNewerOnTop*/ false);
}

// ============================================================================

AOpenRoadDemoGameMode::AOpenRoadDemoGameMode()
{
	DefaultPawnClass = AOpenRoadDemoVehicle::StaticClass();
}
