#pragma once
#include <frc/commands/Command.h>

// ==========================================================================

class ScriptEngine {
public:

	static ScriptEngine& GetInstance();

	void Initialize();
	frc::Command* GetCommand();

private:

	bool _initialized;

	ScriptEngine();

	void _InitializeDashboard();
	void _InitializeParser();
};

// ==========================================================================
