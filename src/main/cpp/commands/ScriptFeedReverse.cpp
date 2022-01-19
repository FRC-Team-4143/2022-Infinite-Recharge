#include "commands/ScriptFeedReverse.h"
#include "Modules/Logger.h"
#include "Robot.h"

// ==========================================================================

ScriptFeedReverse::ScriptFeedReverse(float seconds)
:	frc::Command("ScriptFeedReverse"), _seconds(seconds) {
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

void ScriptFeedReverse::Initialize() {
	LOG(GetName() + "::Initialize");

	SetTimeout(units::second_t(_seconds));
}

// ==========================================================================

void ScriptFeedReverse::Execute() {
	Robot::shooter->Feed(-0.5);
}

// ==========================================================================

bool ScriptFeedReverse::IsFinished() {
	return IsTimedOut();
}

// ==========================================================================

void ScriptFeedReverse::End() {
	LOG(GetName() + "::End");
	Robot::shooter->FeedStop();
}

// ==========================================================================

void ScriptFeedReverse::Interrupted() {
	LOG(GetName() + "::Interrupted");
}

// ==========================================================================
