#include "controllers/PositionSparkController.h"
//#include <iostream>

#define BOTTOMLIMIT 1

#define kP 0.00005
#define kI 0.000001
#define kD 0
#define kIZONE 0
#define kFF 0.000156
#define kMINOUTPUT -1
#define kMAXOUTPUT 1

#define ENCODER_COUNTS_PER_TURN 42

PositionSparkController::PositionSparkController(rev::CANSparkMax* motor) {
	_motor = motor;
	ConfigPID();
}

PositionSparkController::PositionSparkController(int canId) {
	_motor = new rev::CANSparkMax(canId, rev::CANSparkMax::MotorType::kBrushless);
	ConfigPID();
}

void PositionSparkController::SetPercentPower(double value) {
	_motor->Set(value);
}

double PositionSparkController::GetEncoderPosition() {
	return _motor->GetEncoder().GetPosition();
}

void PositionSparkController::SetPosition(double value) {
	//std::cout << "Set Position" << value << std::endl;
	//std::cout.flush();
	auto pidController = _motor->GetPIDController();
	if (value == 0 && fabs(GetEncoderPosition()) < BOTTOMLIMIT) {
		//pidController.SetReference(0, rev::ControlType::kVelocity);
		SetPercentPower(0);
	}
	else {
		pidController.SetReference(value, rev::ControlType::kSmartMotion);
	}
}

void PositionSparkController::ConfigPID() {
	kMaxVel = 5500;
	kMinVel = 0;
	kMaxAcc = 6000;
	kAllErr = 0;

	auto pidController = _motor->GetPIDController();
	_motor->RestoreFactoryDefaults();

	pidController.SetP(kP);
	pidController.SetI(kI);
	pidController.SetD(kD);
	pidController.SetIZone(kIZONE);
	pidController.SetFF(kFF);
	pidController.SetOutputRange(kMINOUTPUT, kMAXOUTPUT);	
	pidController.SetSmartMotionMaxVelocity(kMaxVel);
	pidController.SetSmartMotionMinOutputVelocity(kMinVel);
	pidController.SetSmartMotionMaxAccel(kMaxAcc);
	pidController.SetSmartMotionAllowedClosedLoopError(kAllErr);
}
