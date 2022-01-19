#pragma once
#include "Modules/ISwerveModule.h"
#include "controllers/IMultiController.h"
#include "controllers/IPositionMultiController.h"
#include <string>

class SwerveModule : public ISwerveModule {
public:

	SwerveModule(IMultiController* drive, IPositionMultiController* steer, std::string configName);

	// ISwerveModule methods
	virtual void SetGeometry(double x, double y, double maxradius) override;
	virtual void SetWheelOffset() override;
	virtual void LoadWheelOffset() override;
	virtual void SetDriveSpeed(float speed) override;
	virtual double GetSteerPosition();
	virtual double SetSteerDrive(double x, double y, double twist, bool operatorControl) override;
	virtual double GetSetpoint() override;
	virtual double GetPower() override;

private:

	IMultiController* _drive; // speed controller for the drive motor
	IPositionMultiController* _steer; // speed controller for the steer motor
	std::string _configName;

	double _x;
	double _y;
	double _radius;
	double _offset;
	double _setpoint;
	double _lastPow;
	int _inverse;

	static bool InDeadZone(double value);

	void SetOffset(float offset);
	void SetSteerSetpoint(float setpoint);
};
