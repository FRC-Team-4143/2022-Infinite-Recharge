#include "subsystems/Winch.h"
#include <ctre/Phoenix.h>
#include "commands/WinchActive.h"
#include "controllers/TalonFXController.h"
#include "frc/Solenoid.h"

// ==========================================================================

Winch::Winch(int canId, int brakeChannel)
:	IWinch("Winch") {
	_winch = std::make_unique<TalonFXController>(canId);
	_winchBrake = std::make_unique<frc::Solenoid>(brakeChannel);
	_brakeCounter = 0;
	_brakeEngaged = false;
}

// ==========================================================================

void Winch::InitDefaultCommand() {
	SetDefaultCommand(new WinchActive());
}

// ==========================================================================

void Winch::WinchControl(float speed) {
	if (_TryingToGoUp(speed)) {
		if (_AtUpperLimit()) {
			_WinchStop();
		}
		else {
			_WinchDrive(speed);
		}
	}
	else if (_TryingToGoDown(speed)) {
		_WinchDrive(speed);
	}
	else {
		_WinchStop();
	}
}

// ==========================================================================

bool Winch::_AtUpperLimit() const {
	return _winch->IsForwardLimitSwitchClosed();
}

// ==========================================================================

void Winch::_EngageBrake() {
	if (!_brakeEngaged) {
		_winchBrake->Set(false);
		_brakeCounter = 0;
		_brakeEngaged = true;
	}
}

// ==========================================================================

void Winch::_ReleaseBrake() {
	if (_brakeEngaged) {
		_winchBrake->Set(true);
		_brakeEngaged = false;
	}
}

// ==========================================================================

bool Winch::_TryingToGoUp(float speed) const {
	return speed > 0;
}

// ==========================================================================

bool Winch::_TryingToGoDown(float speed) const {
	return speed < 0;
}

// ==========================================================================

void Winch::_WinchDrive(float speed) {
	constexpr int BRAKE_TRANSITION_TICKS = 25;

	_ReleaseBrake();
	if (_brakeCounter++ > BRAKE_TRANSITION_TICKS) {
		_winch->SetPercentPower(speed);
	}
}

// ==========================================================================

void Winch::_WinchStop() {
	_EngageBrake();
	_winch->SetPercentPower(0);
}

// ==========================================================================
