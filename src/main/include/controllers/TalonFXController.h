#pragma once
#include "controllers/IMultiController.h"
#include <ctre/Phoenix.h>
#include <functional>
#include <memory>

// ================================================================

class TalonFXController : public IMultiController {
public:

	TalonFXController(int canId, std::function<void(WPI_TalonFX*)> fnInitialize = nullptr);
	TalonFXController(WPI_TalonFX* motor, std::function<void(WPI_TalonFX*)> fnInitialize = nullptr);

	// IMultiController methods
	virtual void SetPercentPower(double value) override;
	virtual double GetEncoderPosition() override;

	virtual double GetPosition() const override;
	virtual double GetVelocity() const override;

	virtual bool IsForwardLimitSwitchClosed() const override;
	virtual bool IsReverseLimitSwitchClosed() const override;

private:

	std::unique_ptr<WPI_TalonFX> _motor;
};

// ================================================================
