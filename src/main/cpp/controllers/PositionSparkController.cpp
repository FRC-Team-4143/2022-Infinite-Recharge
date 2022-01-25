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

PositionSparkController::PositionSparkController(rev::CANSparkMax* motor) 
: _motor{motor}, _pidController{_motor->GetPIDController()}, _encoder{_motor->GetEncoder()}
{

	ConfigPID();
}

PositionSparkController::PositionSparkController(int canId)
: _motor{new rev::CANSparkMax(canId, rev::CANSparkMax::MotorType::kBrushless)}, _pidController{_motor->GetPIDController()}, _encoder{_motor->GetEncoder()}
{
	ConfigPID();
}

void PositionSparkController::SetPercentPower(double value) {
	_motor->Set(value);
}

double PositionSparkController::GetEncoderPosition() {
	return _encoder.GetPosition();
}

void PositionSparkController::SetPosition(double value) {
	//std::cout << "Set Position" << value << std::endl;
	//std::cout.flush();
	if (value == 0 && fabs(GetEncoderPosition()) < BOTTOMLIMIT) {
		//_pidController.SetReference(0, rev::ControlType::kVelocity);
		SetPercentPower(0);
	}
	else {
		//_pidController.SetReference(value, rev::ControlType::kSmartMotion);
		_pidController.SetReference(value, rev::CANSparkMax::ControlType::kSmartMotion);

	}
}

void PositionSparkController::ConfigPID() {
	kMaxVel = 5500;
	kMinVel = 0;
	kMaxAcc = 6000;
	kAllErr = 0;

	_motor->RestoreFactoryDefaults();

	_pidController.SetP(kP);
	_pidController.SetI(kI);
	_pidController.SetD(kD);
	_pidController.SetIZone(kIZONE);
	_pidController.SetFF(kFF);
	_pidController.SetOutputRange(kMINOUTPUT, kMAXOUTPUT);	
	_pidController.SetSmartMotionMaxVelocity(kMaxVel);
	_pidController.SetSmartMotionMinOutputVelocity(kMinVel);
	_pidController.SetSmartMotionMaxAccel(kMaxAcc);
	_pidController.SetSmartMotionAllowedClosedLoopError(kAllErr);
}
