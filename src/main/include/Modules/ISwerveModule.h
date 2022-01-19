#pragma once

class ISwerveModule {
public:

	virtual void SetGeometry(double x, double y, double maxradius) = 0;
	virtual void SetWheelOffset() = 0;
	virtual void LoadWheelOffset() = 0;
	virtual void SetDriveSpeed(float speed) = 0;
	virtual double GetSteerPosition() = 0;
	virtual double SetSteerDrive(double x, double y, double twist, bool operatorControl) = 0;
	virtual double GetSetpoint() = 0;
	virtual double GetPower() = 0;
	virtual void Stop() = 0;
};