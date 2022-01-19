#pragma once
#include "controllers/IVelocityMultiController.h"
#include <ctre/Phoenix.h>
#include <memory>

// ================================================================

class ShooterTalonFXController : public IVelocityMultiController {
public:

	ShooterTalonFXController(int canId);

	// IVelocityMultiController methods
	virtual void SetPercentPower(double value) override;
	virtual double GetEncoderPosition() override;
	virtual void SetVelocity(double value) override;
	virtual void ConfigPID() override;

private:

	std::unique_ptr<WPI_TalonFX> _motor;
};

// ================================================================
