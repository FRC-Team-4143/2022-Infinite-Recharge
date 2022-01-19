#include "controllers/SparkMaxController.h"

SparkMaxController::SparkMaxController(rev::CANSparkMax* motor) {
	_motor = motor;
}

SparkMaxController::SparkMaxController(int canId) {
	_motor = new rev::CANSparkMax(canId, rev::CANSparkMax::MotorType::kBrushless);
}

void SparkMaxController::SetPercentPower(double value) {
	_motor->Set(value);
}

double SparkMaxController::GetEncoderPosition() {
	return _motor->GetEncoder().GetPosition();
}
