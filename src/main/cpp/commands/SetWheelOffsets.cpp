#include "commands/SetWheelOffsets.h"
#include "Robot.h"
#include <iostream>

// ==========================================================================

SetWheelOffsets::SetWheelOffsets()
:	frc::Command("Set WheelOffsets") {
	SetRunWhenDisabled(true);
}

// ==========================================================================

void SetWheelOffsets::Initialize() {
	SetTimeout(units::second_t(1));
	Robot::driveTrain->SetWheelOffsets();
}

// ==========================================================================

void SetWheelOffsets::Execute() {
}

// ==========================================================================

bool SetWheelOffsets::IsFinished() {
	return IsTimedOut();
}

// ==========================================================================

void SetWheelOffsets::End() {
	std::cout << "SetWheelOffsets Complete" << std::endl;
	std::cout.flush();
}

// ==========================================================================

void SetWheelOffsets::Interrupted() {
	End();
}

// ==========================================================================
