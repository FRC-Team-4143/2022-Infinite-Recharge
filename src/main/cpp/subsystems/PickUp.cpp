#include "subsystems/PickUp.h"
#include "commands/PickUpIntake.h"
#include "controllers/VictorController.h"

// ==========================================================================

PickUp::PickUp(int forwardChannel, int reverseChannel, int intakeCANId)
:	IPickUp("PickUp") {
	_solenoid = std::make_unique<frc::DoubleSolenoid>(frc::PneumaticsModuleType::CTREPCM,forwardChannel, reverseChannel);
	_intakeMotor = std::make_unique<VictorController>(intakeCANId);
}

// ==========================================================================

void PickUp::InitDefaultCommand() {
	SetDefaultCommand(new PickUpIntake());
}

// ==========================================================================

void PickUp::Extend() {
	_solenoid->Set(frc::DoubleSolenoid::kForward);
}

// ==========================================================================

void PickUp::Retract() {
	_solenoid->Set(frc::DoubleSolenoid::kReverse);
}

// ==========================================================================

void PickUp::Intake(float intakeSpeed) {
	_intakeMotor->SetPercentPower(intakeSpeed);
}

// ==========================================================================

void PickUp::IntakeStop() {
	Intake(0);
}

// ==========================================================================
