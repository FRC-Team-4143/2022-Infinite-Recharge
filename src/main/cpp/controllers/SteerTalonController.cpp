#include "controllers/SteerTalonController.h"

#define P 1.0
#define I 0.005
#define D 0.1
#define F 0.0
#define IZone 100
#define STEERPOW 0.7

#define ENCODER_COUNTS_PER_TURN 4096

SteerTalonController::SteerTalonController(WPI_TalonSRX* motor) {
	_motor = motor;
	ConfigPID();
}

SteerTalonController::SteerTalonController(int canId) {
	_motor = new WPI_TalonSRX(canId);
	ConfigPID();
}

void SteerTalonController::SetPercentPower(double value) {
	_motor->Set(ControlMode::PercentOutput, value);
}

double SteerTalonController::GetEncoderPosition() {
	return _motor->GetSelectedSensorPosition();
}

void SteerTalonController::SetPosition(double value) {
	_motor->Set(ControlMode::Position, value * ENCODER_COUNTS_PER_TURN);
}

void SteerTalonController::ConfigPID() {
	_motor->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 10);
	_motor->SelectProfileSlot(0, 0);
	_motor->Config_kP(0, P, 10);
	_motor->Config_kI(0, I, 10);
	_motor->Config_kD(0, D, 10);
	_motor->Config_kF(0, F, 10);
	_motor->Config_IntegralZone(0, IZone, 10);
	_motor->SetSensorPhase(true);
	_motor->ConfigNominalOutputForward(0, 10);
	_motor->ConfigNominalOutputReverse(0, 10);
	_motor->ConfigPeakOutputForward(STEERPOW, 10);
	_motor->ConfigPeakOutputReverse(-STEERPOW, 10);
}
