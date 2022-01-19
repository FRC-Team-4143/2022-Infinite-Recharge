#pragma once
#include <frc/commands/Command.h>
#include <functional>
#include <string>
#include <vector>

// ==========================================================================

class CommandParseInfo {
public:
	CommandParseInfo(std::string name, std::vector<std::string> aliases =
			std::vector<std::string>(),
			std::function<void(std::vector<float>,
					std::function<void(frc::Command *, float)>)>
	createCommand = nullptr);
	~CommandParseInfo();

	CommandParseInfo(const CommandParseInfo &o);

	std::string GetName() const;
	std::vector<std::string> GetAliases() const;

	bool IsMatch(std::string) const;

	void CreateCommand(std::vector<float> parameters, std::function<void(frc::Command *, float)> addCommand) const;

private:
	std::string _name;
	std::vector<std::string> _aliases;
	std::function<void(std::vector<float>, std::function<void(frc::Command *, float)>)> _createCommand;

	CommandParseInfo &operator=(const CommandParseInfo &o); // Not implemented
};

// ==========================================================================
