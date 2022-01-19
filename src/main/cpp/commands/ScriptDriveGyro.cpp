#include "commands/ScriptDriveGyro.h"
#include "Modules/Logger.h"
#include "Robot.h"

// ==========================================================================

ScriptDriveGyro::ScriptDriveGyro(std::string name, float x, float y, float desiredAngle, float seconds)
:	frc::Command(name), _x(x), _y(y), _desiredAngle(desiredAngle), _seconds(seconds) {
	char szParams[64];
	sprintf(szParams, "(%f, %f, %f, %f)", x, y, desiredAngle, seconds);
	LOG(GetName() + "::ctor" + szParams);

	Requires(Robot::driveTrain);
}

// ==========================================================================

void ScriptDriveGyro::Initialize() {
	LOG(GetName() + "::Initialize");
	SetTimeout(units::second_t(_seconds));
}

// ==========================================================================

void ScriptDriveGyro::Execute() {
	Robot::driveTrain->GyroCrab(_x, _y, _desiredAngle, false);
}

// ==========================================================================

bool ScriptDriveGyro::IsFinished() {
	return IsTimedOut();
}

// ==========================================================================

void ScriptDriveGyro::End() {
	LOG(GetName() + "::End");
	_Cleanup();
}

// ==========================================================================

void ScriptDriveGyro::Interrupted() {
	LOG(GetName() + "::Interrupted");
	_Cleanup();
}

// ==========================================================================

void ScriptDriveGyro::_Cleanup() {
	Robot::driveTrain->Crab(0, 0, 0, false);
}

// ==========================================================================
