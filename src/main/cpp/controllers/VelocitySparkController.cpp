#include "controllers/VelocitySparkController.h"
#include "Modules/Logger.h"
//#include <iostream>

//#define BOTTOMLIMIT 1

#define kP 0.00005
#define kI 0.000001
#define kD 0
#define kIZONE 0
#define kFF 0
//kFF 0.000156
#define kMINOUTPUT -1
#define kMAXOUTPUT 1

#define ENCODER_COUNTS_PER_TURN 42

// ================================================================

//VelocitySparkController::VelocitySparkController(rev::CANSparkMax* motor)
//:	_pidController(motor->GetPIDController()),
//	_encoder(motor->GetEncoder()) {
//	_motor = motor;
//	ConfigPID();
//}

// ================================================================

VelocitySparkController::VelocitySparkController(int canId)
	: _motor{canId, rev::CANSparkMax::MotorType::kBrushless},
	_pidController(_motor.GetPIDController()),
	_encoder(_motor.GetEncoder()) {
	ConfigPID();
}

// ================================================================

void VelocitySparkController::SetPercentPower(double value) {
	LOG("The motor controller is not configured for PercentOutput.");
}

// ================================================================

double VelocitySparkController::GetEncoderPosition() {
	return _encoder.GetVelocity();
}

// ================================================================

void VelocitySparkController::SetVelocity(double value) {
	//if (fabs(value) > .001) {
	//	LOG("SetVelocity");
	//	std::cout << value << std::endl;
	//}
	_pidController.SetReference(value, rev::ControlType::kVelocity);
	//_motor->Set(value/5000.);
}

// ================================================================

void VelocitySparkController::ConfigPID() {
	kMaxVel = 5500;
	kMinVel = -kMaxVel;
	kMaxAcc = 1000;
	kAllErr = 0;

	_motor.RestoreFactoryDefaults();
	//_motor->SetSecondaryCurrentLimit(20);
	_motor.SetSmartCurrentLimit(20);

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

// ================================================================
