#include "commands/ScriptFeedForward.h"
#include "Modules/Logger.h"
#include "Robot.h"

// ==========================================================================

ScriptFeedForward::ScriptFeedForward(float seconds)
:	frc::Command("ScriptFeedForward"), _seconds(seconds) {
	char szParams[64];
	sprintf(szParams, "(%f)", seconds);
	LOG(GetName() + "::ctor" + szParams);

	// ----------------------------------------------------
	// Do NOT require the Shooter subsystem. Otherwise, we
	// cannot do parallel Shooter-related script commands.
	// ----------------------------------------------------
	Requires(Robot::shooter.get());
}

// ==========================================================================

void ScriptFeedForward::Initialize() {
	LOG(GetName() + "::Initialize");

	SetTimeout(units::second_t(_seconds));

}

// ==========================================================================

void ScriptFeedForward::Execute() {
	Robot::shooter->Feed(1);
	//Robot::shooter->LimeLightControl(true);
}

// ==========================================================================

bool ScriptFeedForward::IsFinished() {
	return IsTimedOut();
}

// ==========================================================================

void ScriptFeedForward::End() {
	LOG(GetName() + "::End");
	Robot::shooter->FeedStop();
}

// ==========================================================================

void ScriptFeedForward::Interrupted() {
	LOG(GetName() + "::Interrupted");
}

// ==========================================================================
