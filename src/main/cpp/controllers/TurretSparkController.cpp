#include "controllers/TurretSparkController.h"
//#include <iostream>

#define BOTTOMLIMIT 0
#define TOPLIMIT 10000000

#define kP 0.0001
#define kI 0.00000
#define kD 0
#define kIZONE 0
#define kFF 0.000
#define kMINOUTPUT -.3
#define kMAXOUTPUT .3


#define ENCODER_COUNTS_PER_TURN 42

TurretSparkController::TurretSparkController(rev::CANSparkMax* motor)
: _motor{motor}, _pidController{_motor->GetPIDController()}, _encoder{_motor->GetEncoder()}
{
	ConfigPID();
}

TurretSparkController::TurretSparkController(int canId)
:_motor{new rev::CANSparkMax(canId, rev::CANSparkMax::MotorType::kBrushless)}, _pidController{_motor->GetPIDController()}, _encoder{_motor->GetEncoder()}
{
	ConfigPID();
}

void TurretSparkController::SetPercentPower(double value) {
	_motor->Set(value);
}

double TurretSparkController::GetEncoderPosition() {
	
	return _encoder.GetPosition() * 360 / (204/22) / (9);
}

void TurretSparkController::SetPosition(double value) {
	value = value / 360 * (204/22) * (9/1);
	//std::cout << "Set Position" << value << std::endl;
	//std::cout.flush();
	if (value == 0 && fabs(GetEncoderPosition()) < BOTTOMLIMIT) {
		//pidController.SetReference(0, rev::ControlType::kVelocity);
		SetPercentPower(0);
	}
	else if(value >= TOPLIMIT && GetEncoderPosition() > TOPLIMIT) SetPercentPower(0);
	else {
		_pidController.SetReference(value, rev::ControlType::kSmartMotion);
	}
}

void TurretSparkController::ConfigPID() {
	kMaxVel = 10000;
	kMinVel = 0;
	kMaxAcc = 5000;
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
	_motor->SetSmartCurrentLimit(5);
}

void TurretSparkController::ZeroPosition() {
	_encoder.SetPosition(0);
}
