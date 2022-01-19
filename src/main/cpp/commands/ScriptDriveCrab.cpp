#include "commands/ScriptDriveCrab.h"
#include "Modules/Logger.h"
#include "Robot.h"

// ==========================================================================

ScriptDriveCrab::ScriptDriveCrab(std::string name, float x, float y, float z, float seconds)
:	frc::Command(name), _x(x), _y(y), _z(z), _seconds(seconds) {
	char szParams[64];
	sprintf(szParams, "(%f, %f, %f, %f)", x, y, z, seconds);
	LOG(GetName() + "::ctor" + szParams);

	Requires(Robot::driveTrain);
}

// ==========================================================================

void ScriptDriveCrab::Initialize() {
	LOG(GetName() + "::Initialize");
	SetTimeout(units::second_t(_seconds));
}

// ==========================================================================

void ScriptDriveCrab::Execute() {
	Robot::driveTrain->Crab(_z, _y, _x, false);
}

// ==========================================================================

bool ScriptDriveCrab::IsFinished() {
	return IsTimedOut();
}

// ==========================================================================

void ScriptDriveCrab::End() {
	LOG(GetName() + "::End");
	_Cleanup();
}

// ==========================================================================

void ScriptDriveCrab::Interrupted() {
	LOG(GetName() + "::Interrupted");
	_Cleanup();
}

// ==========================================================================

void ScriptDriveCrab::_Cleanup() {
	Robot::driveTrain->Crab(0, 0, 0, false);
}

// ==========================================================================
