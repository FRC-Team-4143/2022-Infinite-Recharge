#include "commands/ScriptStirReverse.h"
#include "Modules/Logger.h"
#include "Robot.h"

// ==========================================================================

ScriptStirReverse::ScriptStirReverse(float seconds)
:	frc::Command("ScriptStirReverse"), _seconds(seconds) {
	char szParams[64];
	sprintf(szParams, "(%f)", seconds);
	LOG(GetName() + "::ctor" + szParams);

	// ----------------------------------------------------
	// Do NOT require the Shooter subsystem. Otherwise, we
	// cannot do parallel Shooter-related script commands.
	// ----------------------------------------------------
	//Requires(Robot::shooter.get());
}

// ==========================================================================

void ScriptStirReverse::Initialize() {
	LOG(GetName() + "::Initialize");

	SetTimeout(units::second_t(_seconds));
}

// ==========================================================================

void ScriptStirReverse::Execute() {
	Robot::shooter->StirReverse();
}

// ==========================================================================

bool ScriptStirReverse::IsFinished() {
	return IsTimedOut();
}

// ==========================================================================

void ScriptStirReverse::End() {
	LOG(GetName() + "::End");
	Robot::shooter->StirStop();
}

// ==========================================================================

void ScriptStirReverse::Interrupted() {
	LOG(GetName() + "::Interrupted");
}

// ==========================================================================
