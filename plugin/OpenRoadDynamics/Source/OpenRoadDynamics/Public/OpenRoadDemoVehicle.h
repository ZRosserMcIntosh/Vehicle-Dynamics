// OpenRoad Dynamics — Unreal Engine adapter.
// SPDX-License-Identifier: MIT
//
// Proving-ground demo pawn. NOT COMPILED in this repository (no Unreal Engine
// in CI); see docs/PROVING_GROUND.md.
//
// A primitive RWD sports car built only from engine basic shapes:
//   * root: UBoxComponent (4.4 x 1.8 x 0.6 m, Simulate Physics, scale 1) = the
//     Chaos chassis body; the definition's positions are relative to its centre;
//   * UOpenRoadVehicleComponent with a transient definition (asset defaults
//     shifted to the box centre) unless one is assigned;
//   * visual-only body/cabin cubes and four cylinders animated from telemetry
//     (suspension compression, steer angle, wheel spin);
//   * spring-arm chase camera;
//   * Enhanced Input actions and a mapping context created in C++ at runtime
//     (no input assets needed), keyboard + gamepad;
//   * an on-screen readout (GEngine->AddOnScreenDebugMessage; not in Shipping).
//
// Controls (keyboard / gamepad):
//   throttle W, Up / right trigger        brake S, Down / left trigger
//   steer A D, Left Right / left stick    handbrake Space / B (face right)
//   presets 1 FUN, 2 SPORT, 3 SIMULATION / D-pad left, up, right
//   ABS B / X (face left)    TCS T / Y (face top)    ESC E / D-pad down
//   shift up R / right shoulder   shift down F / left shoulder   auto/manual M
//   debug draw F1 or G / View     telemetry start/stop F2 or C / Menu
//   scripted parity runs (record telemetry automatically; use preset 3):
//   Z = 0-100 km/h then full brake to rest   X = step steer at 80 km/h (steer 0.08 left)
//   pressing Z or X again cancels.
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "GameFramework/Pawn.h"

#include "OpenRoadDemoVehicle.generated.h"

class UBoxComponent;
class UCameraComponent;
class UInputAction;
class UInputMappingContext;
class UOpenRoadVehicleComponent;
class USpringArmComponent;
class UStaticMeshComponent;
struct FInputActionValue;

/** Scripted manoeuvres mirroring ord::scenarios (accelerate0to100 + brake100to0, stepSteer). */
UENUM()
enum class EOpenRoadDemoManeuver : uint8
{
	None,
	AccelBrake,
	StepSteer,
};

UCLASS(Blueprintable)
class OPENROADDYNAMICS_API AOpenRoadDemoVehicle : public APawn
{
	GENERATED_BODY()

public:
	AOpenRoadDemoVehicle();

	/** Chaos chassis body (root). Its centre is the definition's origin. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "OpenRoad Demo")
	TObjectPtr<UBoxComponent> Chassis;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "OpenRoad Demo")
	TObjectPtr<UStaticMeshComponent> BodyMesh;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "OpenRoad Demo")
	TObjectPtr<UStaticMeshComponent> CabinMesh;

	/** FL, FR, RL, RR (core wheel order: 2i = left, 2i+1 = right of axle i). Visual only. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "OpenRoad Demo")
	TArray<TObjectPtr<UStaticMeshComponent>> WheelMeshes;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "OpenRoad Demo")
	TObjectPtr<USpringArmComponent> SpringArm;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "OpenRoad Demo")
	TObjectPtr<UCameraComponent> Camera;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "OpenRoad Demo")
	TObjectPtr<UOpenRoadVehicleComponent> Vehicle;

	/** Height of the chassis-box centre above the ground-level origin assumed by the definition defaults, cm. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "OpenRoad Demo")
	float ChassisCenterHeightCm = 55.0f;

	/** Keep automatic gear selection in every preset (SIMULATION is manual by default). Toggle in game with M. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "OpenRoad Demo")
	bool bAutomaticTransmission = true;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "OpenRoad Demo")
	bool bShowReadout = true;

protected:
	virtual void BeginPlay() override;
	virtual void Tick(float DeltaSeconds) override;
	virtual void SetupPlayerInputComponent(UInputComponent* PlayerInputComponent) override;
	virtual void PawnClientRestart() override;

private:
	void EnsureInputAssets();
	void UpdateWheelVisuals(float DeltaSeconds);
	void ShowReadout() const;
	void UseDevice(bool bGamepad);

	// Input handlers (game thread).
	void OnThrottle(const FInputActionValue& Value);
	void OnThrottleEnd(const FInputActionValue& Value);
	void OnBrake(const FInputActionValue& Value);
	void OnBrakeEnd(const FInputActionValue& Value);
	void OnSteerKeyboard(const FInputActionValue& Value);
	void OnSteerGamepad(const FInputActionValue& Value);
	void OnSteerEnd(const FInputActionValue& Value);
	void OnHandbrake(const FInputActionValue& Value);
	void OnHandbrakeEnd(const FInputActionValue& Value);
	void OnPresetFun();
	void OnPresetSport();
	void OnPresetSimulation();
	void OnPresetDrift();
	void OnToggleAbs();
	void OnCycleTcs();
	void OnCycleEsc();
	void OnShiftUp();
	void OnShiftDown();
	void OnToggleShiftMode();
	void OnToggleDebug();
	void OnToggleTelemetry();
	void OnManeuverAccelBrake();
	void OnManeuverStepSteer();
	void StartManeuver(EOpenRoadDemoManeuver NewManeuver);
	void StopManeuver(bool bWriteTelemetry);
	void UpdateManeuver(float DeltaSeconds);
	bool IsManual() const { return Maneuver == EOpenRoadDemoManeuver::None; }

	UPROPERTY(Transient)
	TObjectPtr<UInputMappingContext> MappingContext;

	UPROPERTY(Transient)
	TMap<FName, TObjectPtr<UInputAction>> Actions;

	float WheelSpinDeg[4] = {0.0f, 0.0f, 0.0f, 0.0f};
	float ThrottleValue = 0.0f;
	float BrakeValue = 0.0f;
	float SteerValue = 0.0f;
	bool bHandbrake = false;
	bool bGamepadActive = false;
	double TelemetryStartTime = -1.0;
	FString LastTelemetryFile;

	EOpenRoadDemoManeuver Maneuver = EOpenRoadDemoManeuver::None;
	int32 ManeuverPhase = 0;
	float ManeuverTimer = 0.0f;
	float ManeuverThrottle = 0.0f;
	FString ManeuverStatus;
};

/** Game mode for the proving ground: spawns AOpenRoadDemoVehicle at the PlayerStart if no pawn is auto-possessed. */
UCLASS()
class OPENROADDYNAMICS_API AOpenRoadDemoGameMode : public AGameModeBase
{
	GENERATED_BODY()

public:
	AOpenRoadDemoGameMode();
};
