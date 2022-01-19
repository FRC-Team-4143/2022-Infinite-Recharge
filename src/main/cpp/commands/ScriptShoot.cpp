#include "commands/ScriptShoot.h"
#include "Modules/Logger.h"
#include "Robot.h"

// ==========================================================================

ScriptShoot::ScriptShoot(float angle, float speed, float seconds)
:	frc::Command("ScriptShoot"), _angle(angle), _speed(speed), _seconds(seconds) {
	char szParams[64];
	sprintf(szParams, "(%f)", seconds);
	LOG(GetName() + "::ctor" + szParams);

	Requires(Robot::shooter.get());
}

// ==========================================================================

void ScriptShoot::Initialize() {
	LOG(GetName() + "::Initialize");
	SetTimeout(units::second_t(_seconds));
	Robot::shooter->Feed(0);
	Robot::shooter->TurretMove(_angle);
	Robot::shooter->ShootStart(_speed);
	_counter = 0;
	
}

// ==========================================================================

void ScriptShoot::Execute() {
	_counter++;
	if(_counter > 1*50)
		Robot::shooter->LimeLightControl(true);
}

// ==========================================================================

bool ScriptShoot::IsFinished() {
	return IsTimedOut();
}

// ==========================================================================

void ScriptShoot::End() {
	LOG(GetName() + "::End");
	//Robot::shooter->ShootStop();
}

// ==========================================================================

void ScriptShoot::Interrupted() {
	LOG(GetName() + "::Interrupted");
}

// ==========================================================================
