// ==========================================================================
// Climber class
// The Climber class implements an IClimber subsystem.
// ==========================================================================
#pragma once
#include "subsystems/IClimber.h"
#include <frc/DoubleSolenoid.h>
#include <memory>
#include "controllers/IMultiController.h"

class Climber : public IClimber {
public:

	Climber(int extenderForwardChannel, int extenderReverseChannel, int boatCanId, int rollerbrakeForwardChannel, int rollerbrakeReverseChannel);

	// Subsystem methods
	virtual void InitDefaultCommand() override;

	// IClimber methods
	virtual void Extend() override;
	virtual void Retract() override;
	virtual void BalanceRight() override; 
	virtual void BalanceLeft() override; 
	virtual void BalanceStop() override;
	virtual void EngageRollerBrake() override;
	virtual void DisEngageRollerBrake() override;

private:

	std::unique_ptr<frc::DoubleSolenoid> _extender;
	std::unique_ptr<frc::DoubleSolenoid> _rollerbrake;
	std::unique_ptr<IMultiController> _boatroller;
	int counter;
};

// ==========================================================================
