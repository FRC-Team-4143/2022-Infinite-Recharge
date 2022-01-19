#pragma once
#include "Modules/ISwerveModule.h"

class NullSwerveModule : public ISwerveModule {
public:
	NullSwerveModule();

	// ISwerveModule methods
	virtual void SetGeometry(double x, double y, double maxradius) override;
	virtual void SetWheelOffset() override;
	virtual void LoadWheelOffset() override;
	virtual void SetDriveSpeed(float speed) override;
	virtual double GetSteerPosition() override;
	virtual double SetSteerDrive(double x, double y, double twist, bool operatorControl) override;
	virtual double GetSetpoint() override;
	virtual double GetPower() override;
};
