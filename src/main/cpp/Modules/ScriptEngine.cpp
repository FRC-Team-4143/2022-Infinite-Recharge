#include "Modules/ScriptEngine.h"
#include "Modules/ScriptCommandFactory.h"
#include "Modules/CommandListParser.h"
#include "Modules/Logger.h"

#include "commands/ExtendPickUp.h"
#include "commands/RetractPickUp.h"
#include "commands/ScriptDriveCrab.h"
#include "commands/ScriptDriveFieldCentric.h"
#include "commands/ScriptDriveGyro.h"
#include "commands/ScriptFeedForward.h"
#include "commands/ScriptFeedReverse.h"
#include "commands/ScriptPickUpIntake.h"
#include "commands/ScriptShoot.h"
#include "commands/ScriptStirForward.h"
#include "commands/ScriptStirReverse.h"
#include "commands/ScriptSleep.h"
#include "commands/ScriptControlPanel.h"

#include <frc/smartdashboard/SmartDashboard.h>

// ==========================================================================

ScriptEngine::ScriptEngine()
:	_initialized(false) {
}

// ==========================================================================

ScriptEngine& ScriptEngine::GetInstance() {
	static ScriptEngine instance;
	return instance;
}

// ==========================================================================

void ScriptEngine::Initialize() {
	if (!_initialized) {
		_InitializeDashboard();
		_InitializeParser();
		_initialized = true;
	}
}

// ==========================================================================

frc::Command* ScriptEngine::GetCommand() {
return ScriptCommandFactory::GetInstance().GetCommand().release();
}

// ==========================================================================

void ScriptEngine::_InitializeDashboard() {
	frc::SmartDashboard::PutString("ScriptCommand", "S(1)");
	frc::SmartDashboard::PutString("ScriptValid", "");
}

// ==========================================================================

void ScriptEngine::_InitializeParser() {
	LOG("ScriptEngine::_InitializeParser");

	if (_initialized) {
		return;
	}
	_initialized = true;

	CommandListParser& parser(CommandListParser::GetInstance());

	parser.AddCommand(
		CommandParseInfo(
			"DriveCrab", {"DC", "dc"},
			[](std::vector<float> parameters, std::function<void(frc::Command*, float)> fCreateCommand) {
				parameters.resize(4);
				auto x = parameters[0];
				auto y = parameters[1];
				auto z = parameters[2];
				auto seconds = parameters[3];
				frc::Command* command = new ScriptDriveCrab("DriveCrab", x, y, z, seconds);
				fCreateCommand(command, 0);
			}
		)
	);

	parser.AddCommand(
		CommandParseInfo(
			"DriveFieldCentric", {"DF", "df"},
			[](std::vector<float> parameters, std::function<void(frc::Command*, float)> fCreateCommand) {
				parameters.resize(4);
				auto x = parameters[0];
				auto y = parameters[1];
				auto z = parameters[2];
				auto seconds = parameters[3];
				frc::Command* command = new ScriptDriveFieldCentric("DriveFieldCentric", x, y, z, seconds);
				fCreateCommand(command, 0);
			}
		)
	);

	parser.AddCommand(
		CommandParseInfo(
			"DriveGyro", {"DG", "dg"},
			[](std::vector<float> parameters, std::function<void(frc::Command*, float)> fCreateCommand) {
				parameters.resize(4);
				auto x = parameters[0];
				auto y = parameters[1];
				auto desiredAngle = parameters[2];
				auto seconds = parameters[3];
				frc::Command* command = new ScriptDriveGyro("DriveGyro", x, y, desiredAngle, seconds);
				fCreateCommand(command, 0);
			}
		)
	);

	parser.AddCommand(
		CommandParseInfo(
			"FeedForward", {"FF", "ff"},
			[](std::vector<float> parameters, std::function<void(frc::Command*, float)> fCreateCommand) {
				parameters.resize(1);
				auto seconds = parameters[0];
				frc::Command* command = new ScriptFeedForward(seconds);
				fCreateCommand(command, 0);
			}
		)
	);

	parser.AddCommand(
		CommandParseInfo(
			"FeedReverse", {"FR", "fr"},
			[](std::vector<float> parameters, std::function<void(frc::Command*, float)> fCreateCommand) {
				parameters.resize(1);
				auto seconds = parameters[0];
				frc::Command* command = new ScriptFeedReverse(seconds);
				fCreateCommand(command, 0);
			}
		)
	);

	parser.AddCommand(
		CommandParseInfo(
			"PickupExtend", {"PE", "pe"},
			[](std::vector<float> parameters, std::function<void(frc::Command*, float)> fCreateCommand) {
				frc::Command* command = new ExtendPickUp();
				fCreateCommand(command, 0);
			}
		)
	);

	parser.AddCommand(
		CommandParseInfo(
			"PickupIntake", {"PI", "pi"},
			[](std::vector<float> parameters, std::function<void(frc::Command*, float)> fCreateCommand) {
				parameters.resize(1);
				auto seconds = parameters[0];
				frc::Command* command = new ScriptPickUpIntake(seconds);
				fCreateCommand(command, 0);
			}
		)
	);

	parser.AddCommand(
		CommandParseInfo(
			"PickupRetract", {"PR", "pr"},
			[](std::vector<float> parameters, std::function<void(frc::Command*, float)> fCreateCommand) {
				frc::Command* command = new RetractPickUp();
				fCreateCommand(command, 0);
			}
		)
	);

	parser.AddCommand(
		CommandParseInfo(
			"Shoot", {"SH", "sh"},
			[](std::vector<float> parameters, std::function<void(frc::Command*, float)> fCreateCommand) {
				parameters.resize(3);
				auto angle = parameters[0];
				auto speed = parameters[1];
				auto seconds = parameters[2];
				frc::Command* command = new ScriptShoot(angle, speed, seconds);
				fCreateCommand(command, 0);
			}
		)
	);

	parser.AddCommand(
		CommandParseInfo(
			"Sleep", {"S", "s"},
			[](std::vector<float> parameters, std::function<void(frc::Command*, float)> fCreateCommand) {
				parameters.resize(1);
				auto seconds = parameters[0];
				frc::Command* command = new ScriptSleep(seconds);
				fCreateCommand(command, 0);
			}
		)
	);

	parser.AddCommand(
		CommandParseInfo(
			"StirForward", {"SF", "sf"},
			[](std::vector<float> parameters, std::function<void(frc::Command*, float)> fCreateCommand) {
				parameters.resize(1);
				auto seconds = parameters[0];
				frc::Command* command = new ScriptStirForward(seconds);
				fCreateCommand(command, 0);
			}
		)
	);

	parser.AddCommand(
		CommandParseInfo(
			"StirReverse", {"SR", "sr"},
			[](std::vector<float> parameters, std::function<void(frc::Command*, float)> fCreateCommand) {
				parameters.resize(1);
				auto seconds = parameters[0];
				frc::Command* command = new ScriptStirReverse(seconds);
				fCreateCommand(command, 0);
			}
		)
	);

	parser.AddCommand(
		CommandParseInfo(
			"TogglePanel", {"TC", "tc"},
			[](std::vector<float> parameters, std::function<void(frc::Command*, float)> fCreateCommand) {
				parameters.resize(1);
				auto seconds = parameters[0];
				frc::Command* command = new ScriptControlPanel(seconds);
				fCreateCommand(command, 0);
			}
		)
	);

	// ------------------------------------------------
	// Call IsValid to ensure that regular expressions
	// get built before the start of autonomous.
	// ------------------------------------------------
	parser.IsValid("S(0)");
}

// ==========================================================================
