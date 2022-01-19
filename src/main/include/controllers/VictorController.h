#pragma once
#include "controllers/IMultiController.h"
#include <ctre/Phoenix.h>

class VictorController : public IMultiController {
public:

	VictorController(WPI_VictorSPX* motor);
	VictorController(int canId);

	// IMultiController methods
	virtual void SetPercentPower(double value) override;
	virtual double GetEncoderPosition() override;

private:

	WPI_VictorSPX* _motor;
};
