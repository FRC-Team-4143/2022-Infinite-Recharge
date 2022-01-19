#include "subsystems/Climber.h"
#include "controllers/VictorController.h"

// ==========================================================================

Climber::Climber(int extenderForwardChannel, int extenderReverseChannel, int boatCanId, int rollerbrakeForwarcChannel, int rollerbrakeReverseChannel)
:	IClimber("Climber") {
	_extender = std::make_unique<frc::DoubleSolenoid>(extenderForwardChannel, extenderReverseChannel);
	_boatroller= std::make_unique<VictorController>(boatCanId);
	_rollerbrake= std::make_unique<frc::DoubleSolenoid>(rollerbrakeForwarcChannel, rollerbrakeReverseChannel);
}

// ==========================================================================

void Climber::InitDefaultCommand() {
	// No default command needed for this subsystem.
}

// ==========================================================================

void Climber::Extend() {
	_extender->Set(frc::DoubleSolenoid::kReverse);
}

// ==========================================================================

void Climber::Retract() {
	_extender->Set(frc::DoubleSolenoid::kForward);
}

// ==========================================================================

void Climber::BalanceLeft() {
	DisEngageRollerBrake();
	if(counter++>20) _boatroller->SetPercentPower(-0.50);
}

// ==========================================================================

void Climber::BalanceRight() {

	DisEngageRollerBrake();
	if(counter++>20) _boatroller->SetPercentPower(0.50);

}

// ==========================================================================

void Climber::BalanceStop() {
	_boatroller->SetPercentPower(0);
	EngageRollerBrake();
	counter=0;
}

// ==========================================================================

void Climber::EngageRollerBrake() {
	_rollerbrake->Set(frc::DoubleSolenoid::kReverse);
}

// ==========================================================================

void Climber::DisEngageRollerBrake() {
	_rollerbrake->Set(frc::DoubleSolenoid::kForward);
}

// ==========================================================================