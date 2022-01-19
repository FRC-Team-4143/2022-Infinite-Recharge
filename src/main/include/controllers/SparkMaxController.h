#pragma once
#include "controllers/IMultiController.h"
#include <rev/CANSparkMax.h>

class SparkMaxController : public IMultiController {
public:

	SparkMaxController(rev::CANSparkMax* motor);
	SparkMaxController(int canId);

	// IMultiController methods
	virtual void SetPercentPower(double value) override;
	virtual double GetEncoderPosition() override;

private:

	rev::CANSparkMax* _motor;
};
