#pragma once
#include <frc/commands/Subsystem.h>
#include "Modules/ISwerveModule.h"

// ================================================================

class DriveTrain : public frc::Subsystem {
public:

	DriveTrain();

	// Subsystem methods
	virtual void InitDefaultCommand() override;

	void SetWheelbase(double width, double length, double xOffset = 0, double yOffset = 0);
	void SetWheelOffsets();
	void LoadWheelOffsets();
	void Crab(float twist, float y, float x, bool operatorControl);
	void FieldCentricCrab(float twist, float y, float x, bool operatorControl);
	void GyroCrab(float x, float y, float desiredAngle, bool operatorControl);
	double GyroRotate();
	double GetNearestHeading();
	void RotateAboutPoint(double currentheading);

private:

	static constexpr double pi = 3.141592653589793238462643383; // many more digits than necessary

	double X;
	double Y;
	double yaw;
	double joystickAngle;
	bool fieldCentricMode;
	float lastx;
	float lasty;
	float lasttwist;

	ISwerveModule* frontLeftModule;
	ISwerveModule* frontRightModule;
	ISwerveModule* rearLeftModule;
	ISwerveModule* rearRightModule;
};

// ================================================================
