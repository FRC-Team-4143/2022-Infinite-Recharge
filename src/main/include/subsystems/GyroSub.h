// ==========================================================================
// GyroSub class
// The GyroSub subsystem represents the NavX accelerometer.
//
// FRC 4143: MARS/WARS
// ==========================================================================
#pragma once

#include <frc/commands/Subsystem.h>
#include <frc/PIDSource.h>
#include <AHRS.h>

// ==========================================================================

class GyroSub : public frc::Subsystem, public frc::PIDSource {
public:
	GyroSub();

	// Subsystem methods
	virtual void InitDefaultCommand();

	// PIDSource methods
	virtual double PIDGet();

	double GetHeading();
	bool IsCalibrating();
	void ResetGyro();

	float GetDisplacementX();
	float GetDisplacementY();

private:

	AHRS* theGyro();
};

// ==========================================================================
