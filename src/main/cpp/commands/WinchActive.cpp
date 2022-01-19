#include "commands/WinchActive.h"
#include "Robot.h"

// ==========================================================================

WinchActive::WinchActive()
:	frc::Command("WinchActive") {
	Requires(Robot::winch.get());
}

// ==========================================================================

void WinchActive::Initialize() {
	_counter = 0;
}

// ==========================================================================

void WinchActive::Execute() {

	// --------------
	// Winch control
	// --------------

	float winchSpeed = Robot::oi->GetJoystick2Y() * 0.5;

	//untested change
	//if(frc::Timer::GetMatchTime() < .5 /*&& frc::DriverStation::IsFMSAttached() && frc::DriverStation::IsOperatorControl()*/)   winchSpeed = 0;
	Robot::winch->WinchControl(winchSpeed);
/*
	if (winchSpeed != 0) {
		_counter++;
		Robot::winch->ReleaseBrake();
		if (_counter > 25) {
			Robot::winch->WinchDrive(winchSpeed);
		}
	}
	else {
		_counter = 0;
		Robot::winch->EngageBrake();
		Robot::winch->WinchDrive(0);
	}
*/

	// ---------------
	// Roller control
	// ---------------

	if (Robot::oi->GetButtonStart()) {
		Robot::climber->BalanceRight();
	}
	else if (Robot::oi->GetButtonBack()) {
		Robot::climber->BalanceLeft();
	}
	else {
		Robot::climber->BalanceStop();
	}
}

// ==========================================================================

bool WinchActive::IsFinished() {
	return false;
}

// ==========================================================================

void WinchActive::End() {
	Robot::winch->WinchControl(0);
	//Robot::winch->WinchStop();
}

// ==========================================================================

void WinchActive::Interrupted() {
	End();
}

// ==========================================================================
