/*
#include "controllers/PositionTalonFXController.h"
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

PositionTalonFXController::PositionTalonFXController(WPI_TalonFX* motor) {
	_motor = motor;
	ConfigPID();
}

PositionTalonFXController::PositionTalonFXController(int canId) {
	_motor{std::make_unique<WPI_TalonFX>(canId)} {
	ConfigPID();
}

void PositionTalonFXController::SetPercentPower(double value) {
	_motor->Set(ControlMode::PercentOutput, value);
}

double PositionTalonFXController::GetEncoderPosition() {
	return _motor->GetEncoder().GetPosition();
}

void PositionTalonFXController::SetPosition(double value) {
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

void PositionTalonFXController::ConfigPID() {
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
*/