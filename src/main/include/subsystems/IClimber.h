// ==========================================================================
// IClimber interface
// The IClimber interface represents the functionality of a climber subsystem.
// ==========================================================================
#pragma once
#include <frc/commands/Subsystem.h>

class IClimber : public frc::Subsystem {
public:

	IClimber(std::string_view name);

	virtual void Extend() = 0;
	virtual void Retract() = 0;
	virtual void BalanceRight() = 0; 
	virtual void BalanceLeft() = 0; 
	virtual void BalanceStop() = 0;
	virtual void EngageRollerBrake() = 0;
	virtual void DisEngageRollerBrake() = 0;
};

// ==========================================================================
