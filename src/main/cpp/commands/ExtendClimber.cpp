#include "commands/ExtendClimber.h"
#include "Robot.h"

// ==========================================================================

ExtendClimber::ExtendClimber()
:	frc::Command("Extend Climber") {
	Requires(Robot::climber.get());
}

// ==========================================================================

void ExtendClimber::Initialize() {
}

// ==========================================================================

void ExtendClimber::Execute() {
	// untested change
	if(Robot::shooter->GetDegrees() < 20) 
		Robot::climber->Extend();
}

// ==========================================================================

bool ExtendClimber::IsFinished() {
	return true;
}

// ==========================================================================

void ExtendClimber::End() {
}

// ==========================================================================

void ExtendClimber::Interrupted() {
	End();
}

// ==========================================================================
