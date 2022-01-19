#include "commands/RetractClimber.h"
#include "Robot.h"

// ==========================================================================

RetractClimber::RetractClimber()
:	frc::Command("Retract Climber") {
	Requires(Robot::climber.get());
}

// ==========================================================================

void RetractClimber::Initialize() {
}

// ==========================================================================

void RetractClimber::Execute() {
	Robot::climber->Retract();
}

// ==========================================================================

bool RetractClimber::IsFinished() {
	return true;
}

// ==========================================================================

void RetractClimber::End() {
}

// ==========================================================================

void RetractClimber::Interrupted() {
	End();
}

// ==========================================================================
