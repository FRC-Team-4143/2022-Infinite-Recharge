#include "MOdules/CommandParseInfo.h"

// ==========================================================================

CommandParseInfo::CommandParseInfo(
		std::string name, std::vector<std::string> aliases,
		std::function<void(std::vector<float>,
				std::function<void(frc::Command *, float)>)> createCommand)
: _name(name), _aliases(aliases), _createCommand(createCommand) {
}

// ==========================================================================

CommandParseInfo::~CommandParseInfo() {
}

// ==========================================================================

CommandParseInfo::CommandParseInfo(const CommandParseInfo &o)
: _name(o._name), _aliases(o._aliases), _createCommand(o._createCommand) {
}

// ==========================================================================

std::string CommandParseInfo::GetName() const {
	return _name;
}

// ==========================================================================

std::vector<std::string> CommandParseInfo::GetAliases() const {
	return _aliases;
}

// ==========================================================================

bool CommandParseInfo::IsMatch(std::string s) const {
	if (GetName() == s) {
		return true;
	}
	for (const auto &a : GetAliases()) {
		if (a == s) {
			return true;
		}
	}
	return false;
}

// ==========================================================================

void CommandParseInfo::CreateCommand(std::vector<float> parameters, std::function<void(frc::Command *, float)> addCommand) const {
	if (_createCommand != nullptr) {
		_createCommand(parameters, addCommand);
	}
}

// ==========================================================================
