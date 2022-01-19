#include "commands/CrabDrive.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "Robot.h"

// ==========================================================================

CrabDrive::CrabDrive() {
	// Use Requires() here to declare subsystem dependencies
	Requires(Robot::driveTrain);
}

// ==========================================================================

void CrabDrive::Initialize() {
	frc::SmartDashboard::PutString("Driving Mode", "Crab Drive");
}

// ==========================================================================

void CrabDrive::Execute() {
	auto x = Robot::oi->GetJoystickX();
	auto y = Robot::oi->GetJoystickY();
	auto z = Robot::oi->GetJoystickZ();

	x *= fabs(x);
	y *= fabs(y);
	z *= fabs(z);

	//x *= (x < 0 ? -x: x);
	//z *= (z < 0 ? -z: z);
	if (fabs(y) > 0.5) {
		z *= 0.75;
	}
	//y *= (y < 0 ? -y: y);

	Robot::driveTrain->Crab(-z, -y, x, true);
}

// ==========================================================================

bool CrabDrive::IsFinished() {
	return false;
}

// ==========================================================================

void CrabDrive::End() {
	frc::SmartDashboard::PutString("Driving Mode", "Unknown");
}

// ==========================================================================

void CrabDrive::Interrupted() {
	End();
}

// ==========================================================================
