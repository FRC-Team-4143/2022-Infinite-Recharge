#include "commands/RetractPickUp.h"
#include "Robot.h"

// ==========================================================================

RetractPickUp::RetractPickUp()
:	frc::Command("Retract PickUp") {
	Requires(Robot::pickUp.get());
}

// ==========================================================================

void RetractPickUp::Initialize() {
}

// ==========================================================================

void RetractPickUp::Execute() {
	Robot::pickUp->Retract();
}

// ==========================================================================

bool RetractPickUp::IsFinished() {
	return true;
}

// ==========================================================================

void RetractPickUp::End() {
}

// ==========================================================================

void RetractPickUp::Interrupted() {
	End();
}

// ==========================================================================
