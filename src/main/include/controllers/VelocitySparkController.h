#pragma once
#include "controllers/IVelocityMultiController.h"
#include <rev/CANSparkMax.h>

// ================================================================

class VelocitySparkController : public IVelocityMultiController {
public:

	//VelocitySparkController(rev::CANSparkMax* motor);
	VelocitySparkController(int canId);

	// IVelocityMultiController methods
	virtual void SetPercentPower(double value) override;
	virtual double GetEncoderPosition() override;
	virtual void SetVelocity(double value) override;
	virtual void ConfigPID() override;

private:

	double kMaxVel;
	double kMinVel;
	double kMaxAcc;
	double kAllErr;
	rev::CANSparkMax _motor;
	rev::SparkMaxPIDController _pidController;
	//rev::CANEncoder _encoder;
	rev::SparkMaxRelativeEncoder _encoder;
};

// ================================================================
