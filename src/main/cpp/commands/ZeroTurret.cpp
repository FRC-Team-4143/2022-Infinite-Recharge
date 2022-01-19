#include "commands/ZeroTurret.h"
#include "Robot.h"

// ==========================================================================

ZeroTurret::ZeroTurret()
:	frc::Command("Zero Turret") {
	SetRunWhenDisabled(true);
}

// ==========================================================================

void ZeroTurret::Initialize() {
}

// ==========================================================================

void ZeroTurret::Execute() {
	Robot::shooter->TurretZero();
}

// ==========================================================================

bool ZeroTurret::IsFinished() {
	return true;
}

// ==========================================================================

void ZeroTurret::End() {
}

// ==========================================================================

void ZeroTurret::Interrupted() {
	End();
}

// ==========================================================================
