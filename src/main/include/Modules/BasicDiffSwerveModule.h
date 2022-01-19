#pragma once
#include "Modules/ISwerveModule.h"
#include "controllers/IVelocityMultiController.h"
#include "encoders/IPositionEncoder.h"
#include <string>

// ==========================================================================

class BasicDiffSwerveModule : public ISwerveModule {
public:

	BasicDiffSwerveModule(IVelocityMultiController* master, IVelocityMultiController* slave, std::string configName, IPositionEncoder* headingSensor);

	// ISwerveModule methods
	virtual void SetGeometry(double x, double y, double maxRadius) override;
	virtual void SetWheelOffset() override;
	virtual void LoadWheelOffset() override;
	virtual void SetDriveSpeed(float speed) override;
	virtual double GetSteerPosition() override;
	virtual double SetSteerDrive(double x, double y, double twist, bool operatorControl) override;
	virtual double GetSetpoint() override;
	virtual double GetPower() override;
	virtual void Stop() override;

private:

	IVelocityMultiController* _master; // speed controller for the drive motor
	IVelocityMultiController* _slave; // speed controller for the steer motor
	std::string _configName;
	IPositionEncoder* _headingSensor;

	double _x;
	double _y;
	double _radius;
	double _setpoint;
	double _lastPower;
	int _inverse;

	static bool InDeadZone(double value);

	void SetSteerSetpoint(float setpoint);
};

// ==========================================================================
