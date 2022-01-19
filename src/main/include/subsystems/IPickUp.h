// ==========================================================================
// IPickUp interface
// The IPickUp interface represents the functionality of a pickup subsystem.
// ==========================================================================
#pragma once
#include <frc/commands/Subsystem.h>

class IPickUp : public frc::Subsystem {
public:

	IPickUp(std::string_view name);

	virtual void Extend() = 0;
	virtual void Retract() = 0;
	virtual void Intake(float intakeSpeed) = 0;
	virtual void IntakeStop() = 0;
};

// ==========================================================================
