#include "commands/ExtendPickUp.h"
#include "Robot.h"

// ==========================================================================

ExtendPickUp::ExtendPickUp()
:	frc::Command("Extend PickUp") {
	Requires(Robot::pickUp.get());
}

// ==========================================================================

void ExtendPickUp::Initialize() {
}

// ==========================================================================

void ExtendPickUp::Execute() {
	Robot::pickUp->Extend();
}

// ==========================================================================

bool ExtendPickUp::IsFinished() {
	return true;
}

// ==========================================================================

void ExtendPickUp::End() {
}

// ==========================================================================

void ExtendPickUp::Interrupted() {
	End();
}

// ==========================================================================
