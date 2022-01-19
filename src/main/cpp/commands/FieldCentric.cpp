#include "commands/FieldCentric.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "Robot.h"

// ==========================================================================

FieldCentric::FieldCentric() {
	// Use Requires() here to declare subsystem dependencies
	Requires(Robot::driveTrain);
}

// ==========================================================================

void FieldCentric::Initialize() {
	frc::SmartDashboard::PutString("Driving Mode", "Field Centric");
}

// ==========================================================================

void FieldCentric::Execute() {
	auto x = Robot::oi->GetJoystickX();
	auto y = Robot::oi->GetJoystickY();
	auto z = Robot::oi->GetJoystickZ();
	//if (x > 0) x = x* x + 0.1;
	//else if (x < 0) x = -(x* x + 0.1);

	//if (y > 0) y = y* y + 0.1;
	//else if (y < 0) y = -(y* y + 0.1);

	//if (z > 0) z = z* z + 0.1;
	//else if (z < 0) z = -(z* z + 0.1);

	//if (Robot::oi->GetButtonLeft() == false) {
		Robot::driveTrain->FieldCentricCrab(-z, -y, x, true);
		//frc::SmartDashboard::PutString("Driving Speed", "Full");	
	//}
	//else {
	//	frc::SmartDashboard::PutString("Driving Speed", "Half");
	//	Robot::driveTrain->FieldCentricCrab(-z/2, -y/2, x/2, true);
	//}
}

// ==========================================================================

bool FieldCentric::IsFinished() {
	return false;
}

// ==========================================================================

void FieldCentric::End() {
	frc::SmartDashboard::PutString("Driving Mode", "Unknown");
}

// ==========================================================================

void FieldCentric::Interrupted() {
	End();
}

// ==========================================================================
