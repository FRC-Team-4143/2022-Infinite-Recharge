#pragma once
#include "controllers/IPositionMultiController.h"
#include <ctre/Phoenix.h>

class SteerTalonController : public IPositionMultiController {
public:

	SteerTalonController(WPI_TalonSRX* motor);
	SteerTalonController(int canId);

	// IPositionMultiController methods
	virtual void SetPercentPower(double value) override;
	virtual double GetEncoderPosition() override;
	virtual void SetPosition(double value) override;
	virtual void ConfigPID() override;

private:

	WPI_TalonSRX* _motor;
};
