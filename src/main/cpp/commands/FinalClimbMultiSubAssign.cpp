#include "commands/FinalClimbMultiSubAssign.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "Robot.h"
//#include "subsystems/Shooter.h"



// ==========================================================================

FinalClimbMultiSubAssign::FinalClimbMultiSubAssign() {
	Requires(Robot::climber.get());
    Requires(Robot::shooter.get());
    Requires(Robot::winch.get());
}

// ==========================================================================

void FinalClimbMultiSubAssign::Initialize() {
	frc::SmartDashboard::PutString("Climbing:", "Init");
}

// ==========================================================================

void FinalClimbMultiSubAssign::Execute() {
    frc::SmartDashboard::PutString("Climbing:", "Execute");
	
    float speed = 0.25; // To be tested

    //Robot::shooter->TurretZero();
    Robot::winch->WinchControl(speed);
    //if (Robot::shooter->_turret->GetEncoderPosition() < -15) {
     Robot::climber->Extend();
    //}

}

// ==========================================================================

bool FinalClimbMultiSubAssign::IsFinished() {
	frc::SmartDashboard::PutString("Climbing:", "Finnished");
    return true;
}

// ==========================================================================

void FinalClimbMultiSubAssign::End() {
	frc::SmartDashboard::PutString("Climbing:", "Unknown");
}

// ==========================================================================

void FinalClimbMultiSubAssign::Interrupted() {
	End();
}

// ==========================================================================