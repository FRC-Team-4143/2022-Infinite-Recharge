#include "commands/ScriptControlPanel.h"
#include "Modules/Logger.h"
#include "Robot.h"

// ==========================================================================

ScriptControlPanel::ScriptControlPanel(float seconds)
:	frc::Command("ScriptControlPanel"), _seconds(seconds) {
	char szParams[64];
	sprintf(szParams, "(%f)", seconds);
	LOG(GetName() + "::ctor" + szParams);

	// ----------------------------------------------------
	// Do NOT require the Shooter subsystem. Otherwise, we
	// cannot do parallel Shooter-related script commands.
	// ----------------------------------------------------
	Requires(Robot::controlPanel.get());
    //Requires(Robot::climber.get()); needed to run the control panel motor
}

// ==========================================================================

void ScriptControlPanel::Initialize() {
	LOG(GetName() + "::Initialize");

	SetTimeout(units::second_t(_seconds));

    Robot::controlPanel->ColorUp();

}

// ==========================================================================

void ScriptControlPanel::Execute() {
	
}

// ==========================================================================

bool ScriptControlPanel::IsFinished() {
	return IsTimedOut();
}

// ==========================================================================

void ScriptControlPanel::End() {
	LOG(GetName() + "::End");
	Robot::controlPanel->ColorDown();
}

// ==========================================================================

void ScriptControlPanel::Interrupted() {
	LOG(GetName() + "::Interrupted");
}

// ==========================================================================
