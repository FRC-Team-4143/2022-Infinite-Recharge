#pragma once
#include <frc/commands/Command.h>
#include <string>

// ==========================================================================

class ScriptValidate : public frc::Command {
public:

	ScriptValidate();
	ScriptValidate(std::string dashboardInput, std::string dashboardOutput);

	// Command methods
	virtual void Initialize() override;
	virtual void Execute() override;
	virtual bool IsFinished() override;
	virtual void End() override;
	virtual void Interrupted() override;

protected:

	void ValidateCommands();

private:

	std::string _dashboardInput;
	std::string _dashboardOutput;
};

// ==========================================================================
