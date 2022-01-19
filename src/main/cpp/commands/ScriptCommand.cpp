#include "commands/ScriptCommand.h"
#include "Modules/CommandListParser.h"
#include "Modules/Logger.h"
#include <frc/smartdashboard/SmartDashboard.h>

// ==========================================================================

ScriptCommand::ScriptCommand(std::string name)
: ScriptCommand(name, frc::SmartDashboard::GetString("ScriptCommand", "")) {
}

// ==========================================================================

ScriptCommand::ScriptCommand(std::string name, std::string script)
: frc::CommandGroup(name) {
	LOG(GetName() + "::ctor");
	ParseCommands(script);
}

// ==========================================================================

void ScriptCommand::InitParameters() {
	try {
		frc::SmartDashboard::GetString("ScriptCommand", "");
	} catch (...) {
		// SmartDashboard parameter does not exist; create it.
		frc::SmartDashboard::PutString("ScriptCommand", "S(1)");
	}
}

// ==========================================================================

void ScriptCommand::ParseCommands(std::string script) {
	CommandListParser &parser(CommandListParser::GetInstance());
	parser.Parse(script, [this](bool parallel, Command *cmd, float timeout) {
		if (parallel) {
			if (timeout) {
				AddParallel(cmd, units::second_t(timeout));
			} else {
				AddParallel(cmd);
			}
		} else {
			if (timeout) {
				AddSequential(cmd, units::second_t(timeout));
			} else {
				AddSequential(cmd);
			}
		}
	});
}

// ==========================================================================
