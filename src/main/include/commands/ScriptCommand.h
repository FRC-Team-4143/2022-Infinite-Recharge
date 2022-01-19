#pragma once
#include <frc/commands/CommandGroup.h>
#include <string>

// ==========================================================================

class ScriptCommand : public frc::CommandGroup {
public:

	ScriptCommand(std::string name);
	ScriptCommand(std::string name, std::string script);

	static void InitParameters();

protected:

	void ParseCommands(std::string commands);

private:
};

// ==========================================================================
