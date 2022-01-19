#pragma once
#include "controllers/IMultiController.h"
#include <ctre/Phoenix.h>

// ==========================================================================

class TalonSRXController : public IMultiController {
public:

	TalonSRXController(WPI_TalonSRX* motor);
	TalonSRXController(int canId);

	// IMultiController methods
	virtual void SetPercentPower(double value) override;
	virtual double GetEncoderPosition() override;

private:

	WPI_TalonSRX* _motor;
};

// ==========================================================================
