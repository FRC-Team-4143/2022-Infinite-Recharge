#include "commands/ScriptPickUpIntake.h"
#include "Modules/Logger.h"
#include "Robot.h"

// ==========================================================================

ScriptPickUpIntake::ScriptPickUpIntake(float seconds)
:	frc::Command("ScriptPickUpIntake"), _seconds(seconds) {
	char szParams[64];
	sprintf(szParams, "(%f)", seconds);
	LOG(GetName() + "::ctor" + szParams);

	Requires(Robot::pickUp.get());
}

// ==========================================================================

void ScriptPickUpIntake::Initialize() {
	LOG(GetName() + "::Initialize");

	SetTimeout(units::second_t(_seconds));
}

// ==========================================================================

void ScriptPickUpIntake::Execute() {
	Robot::pickUp->Intake(-1.00);
}

// ==========================================================================

bool ScriptPickUpIntake::IsFinished() {
	return IsTimedOut();
}

// ==========================================================================

void ScriptPickUpIntake::End() {
	LOG(GetName() + "::End");
}

// ==========================================================================

void ScriptPickUpIntake::Interrupted() {
	LOG(GetName() + "::Interrupted");
}

// ==========================================================================
