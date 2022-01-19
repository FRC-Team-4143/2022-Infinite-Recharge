/*
#pragma once
#include "controllers/IPositionMultiController.h"
#include <ctre/Phoenix.h>
#include <memory>
//#include <rev/CANSparkMax.h>

// ================================================================

class PositionTalonFXController : public IPositionMultiController {
public:

	PositionTalonFXController(int canId);

	// IPositionMultiController methods
	virtual void SetPercentPower(double value) override;
	virtual double GetEncoderPosition() override;
	virtual void SetPosition(double value) override;
	virtual void ConfigPID() override;

private:

	double kMaxVel;
	double kMinVel;
	double kMaxAcc;
	double kAllErr;
	//WPI_TalonFX* _motor;
	std::unique_ptr<WPI_TalonFX> _motor;
	//rev::CANPIDController _pidController;
};
*/