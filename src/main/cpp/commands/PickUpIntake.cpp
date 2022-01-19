#include "commands/PickUpIntake.h"
#include "Robot.h"

// ==========================================================================

PickUpIntake::PickUpIntake()
:	frc::Command("PickUp Intake") {
	Requires(Robot::pickUp.get());
}

// ==========================================================================

void PickUpIntake::Initialize() {
}

// ==========================================================================

void PickUpIntake::Execute() {
	if (Robot::oi->GetLeftBumper() > 0.2) {
		Robot::pickUp->Intake(-1.00);
	}
	else if(Robot::oi->GetPOV1() == 180) {
		Robot::pickUp->Intake(1.00);
	}
	else {
		Robot::pickUp->Intake(0.0);
	}
}

// ==========================================================================

bool PickUpIntake::IsFinished() {
	return false;
}

// ==========================================================================

void PickUpIntake::End() {
	Robot::pickUp->IntakeStop();
}

// ==========================================================================

void PickUpIntake::Interrupted() {
	End();
}

// ==========================================================================
