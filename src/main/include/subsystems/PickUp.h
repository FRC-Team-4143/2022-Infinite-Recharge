// ==========================================================================
// PickUp class
// The PickUp class implements an IPickUp subsystem.
// ==========================================================================
#pragma once
#include "subsystems/IPickup.h"
#include <frc/DoubleSolenoid.h>
#include "controllers/IMultiController.h"
#include <memory>

class PickUp : public IPickUp {
public:

	PickUp(int forwardChannel, int reverseChannel, int intakeCANId);

	// Subsystem methods
	virtual void InitDefaultCommand() override;

	// IPickUp methods
	virtual void Extend() override;
	virtual void Retract() override;
	virtual void Intake(float intakeSpeed) override;
	virtual void IntakeStop() override;

private:

	std::unique_ptr<frc::DoubleSolenoid> _solenoid;
	std::unique_ptr<IMultiController> _intakeMotor; 
};

// ==========================================================================
