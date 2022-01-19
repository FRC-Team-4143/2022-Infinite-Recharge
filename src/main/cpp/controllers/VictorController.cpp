#include "controllers/VictorController.h"

VictorController::VictorController(WPI_VictorSPX* motor) {
	_motor = motor;
}

VictorController::VictorController(int canId) {
	_motor = new WPI_VictorSPX(canId);
}

void VictorController::SetPercentPower(double value) {
	_motor->Set(ControlMode::PercentOutput, value);
}

double VictorController::GetEncoderPosition() {
	return 0;
}
