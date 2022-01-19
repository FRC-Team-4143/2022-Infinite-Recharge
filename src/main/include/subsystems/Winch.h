// ==========================================================================
// Winch class
// The Winch class implements an IWinch subsystem.
// ==========================================================================
#pragma once
#include "subsystems/IWinch.h"
#include "controllers/IMultiController.h"
#include <memory>
#include "frc/Solenoid.h"

class Winch : public IWinch {
public:

	Winch(int canId, int brakeChannel);

	// Subsystem methods
	virtual void InitDefaultCommand() override;

	// IWinch methods
	virtual void WinchControl(float speed) override;

private:

	bool _AtUpperLimit() const;
	bool _TryingToGoUp(float speed) const;
	bool _TryingToGoDown(float speed) const;
	void _WinchDrive(float speed);
	void _WinchStop();
	void _EngageBrake();
	void _ReleaseBrake();

	std::unique_ptr<IMultiController> _winch;
	std::unique_ptr<frc::Solenoid> _winchBrake;
	int _brakeCounter;
	bool _brakeEngaged;
};

// ==========================================================================
