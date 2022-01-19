#pragma once
#include <frc/commands/Command.h>
#include <string>

// ==========================================================================

class ScriptDriveFieldCentric : public frc::Command {
public:

	ScriptDriveFieldCentric(std::string name, float x, float y, float z, float seconds);

	// Command methods
	virtual void Initialize() override;
	virtual void Execute() override;
	virtual bool IsFinished() override;
	virtual void End() override;
	virtual void Interrupted() override;

private:

	float _x;
	float _y;
	float _z;
	float _seconds;

	void _Cleanup();
};

// ==========================================================================
