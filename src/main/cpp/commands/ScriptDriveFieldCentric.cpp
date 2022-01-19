#include "commands/ScriptDriveFieldCentric.h"
#include "Modules/Logger.h"
#include "Robot.h"

// ==========================================================================

ScriptDriveFieldCentric::ScriptDriveFieldCentric(std::string name, float x, float y, float z, float seconds)
:	frc::Command(name), _x(x), _y(y), _z(z), _seconds(seconds) {
	char szParams[64];
	sprintf(szParams, "(%f, %f, %f, %f)", x, y, z, seconds);
	LOG(GetName() + "::ctor" + szParams);

	Requires(Robot::driveTrain);
}

// ==========================================================================

void ScriptDriveFieldCentric::Initialize() {
	LOG(GetName() + "::Initialize");
	SetTimeout(units::second_t(_seconds));
}

// ==========================================================================

void ScriptDriveFieldCentric::Execute() {
	Robot::driveTrain->FieldCentricCrab(_z, _y, _x, false);
}

// ==========================================================================

bool ScriptDriveFieldCentric::IsFinished() {
	return IsTimedOut();
}

// ==========================================================================

void ScriptDriveFieldCentric::End() {
	LOG(GetName() + "::End");
	_Cleanup();
}

// ==========================================================================

void ScriptDriveFieldCentric::Interrupted() {
	LOG(GetName() + "::Interrupted");
	_Cleanup();
}

// ==========================================================================

void ScriptDriveFieldCentric::_Cleanup() {
	Robot::driveTrain->Crab(0, 0, 0, false);
}

// ==========================================================================
