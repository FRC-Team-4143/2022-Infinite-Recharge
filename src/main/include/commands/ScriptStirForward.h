#pragma once
#include <frc/commands/Command.h>

// ==========================================================================

class ScriptStirForward : public frc::Command {
public:

	ScriptStirForward(float seconds);

	// Command methods
	virtual void Initialize() override;
	virtual void Execute() override;
	virtual bool IsFinished() override;
	virtual void End() override;
	virtual void Interrupted() override;

private:

	float _seconds;
};

// ==========================================================================
