#include "controllers/TalonSRXController.h"

// ==========================================================================

TalonSRXController::TalonSRXController(WPI_TalonSRX* motor) {
	_motor = motor;
}

// ==========================================================================

TalonSRXController::TalonSRXController(int canId) {
	_motor = new WPI_TalonSRX(canId);
}

// ==========================================================================

void TalonSRXController::SetPercentPower(double value) {
	_motor->Set(ControlMode::PercentOutput, value);
}

// ==========================================================================

double TalonSRXController::GetEncoderPosition() {
	return _motor->GetSelectedSensorPosition();
}

// ==========================================================================
