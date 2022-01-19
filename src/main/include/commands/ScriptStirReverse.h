#pragma once
#include <frc/commands/Command.h>

// ==========================================================================

class ScriptStirReverse : public frc::Command {
public:

	ScriptStirReverse(float seconds);

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
