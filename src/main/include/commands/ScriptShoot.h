#pragma once
#include <frc/commands/Command.h>

// ==========================================================================

class ScriptShoot : public frc::Command {
public:

	ScriptShoot(float angle, float speed, float seconds);

	// Command methods
	virtual void Initialize() override;
	virtual void Execute() override;
	virtual bool IsFinished() override;
	virtual void End() override;
	virtual void Interrupted() override;

private:

	float _seconds;
	float _angle;
	float _speed;
	float _counter;
};

// ==========================================================================
