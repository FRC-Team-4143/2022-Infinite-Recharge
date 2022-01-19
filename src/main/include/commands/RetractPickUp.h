#pragma once
#include <frc/commands/Command.h>

// ==========================================================================

class RetractPickUp : public frc::Command {
public:

	RetractPickUp();

	// Command methods
	virtual void Initialize() override;
	virtual void Execute() override;
	virtual bool IsFinished() override;
	virtual void End() override;
	virtual void Interrupted() override;
};

// ==========================================================================
