#pragma once
#include "Modules/ISwerveModule.h"
#include <ctre/Phoenix.h>
#include <string>

class TalonFXDiffSwerveModule : public ISwerveModule {
public:

	TalonFXDiffSwerveModule(int masterId, int slaveId, std::string configName, CANCoder* headingSensor, bool invert);

	// ISwerveModule methods
	virtual void SetGeometry(double x, double y, double maxradius) override;
	virtual void SetWheelOffset() override;
	virtual void LoadWheelOffset() override;
	virtual void SetDriveSpeed(float speed) override;
	virtual double GetSteerPosition() override;
	virtual double SetSteerDrive(double x, double y, double twist, bool operatorControl) override;
	virtual double GetSetpoint() override;
	virtual double GetPower() override;
	virtual void Stop() override;

private:

	TalonFX _master;
	TalonFX _slave;
	std::string _configName;
	CANCoder* _headingSensor;
	double _x;
	double _y;
	double _radius;
	double _setpoint;
	double _offset;
	double _lastPow;
	int _inverse;

	static bool InDeadZone(double value);

	void ConfigMotors(bool invert);
	void ConfigureNeutralOnLOS(bool enable);
	void SetOffset(float offset);
	void SetSteerSetpoint(float setpoint);
	void ZeroSensors();
};
