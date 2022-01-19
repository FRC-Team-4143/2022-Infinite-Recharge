#pragma once
#include "Modules/CommandParseInfo.h"
#include <functional>
#include <regex>
#include <string>
#include <vector>

// ==========================================================================

class CommandListParser {
public:
	static CommandListParser &GetInstance() {
		static CommandListParser instance;
		return instance;
	}

	static void EnableDebug(bool debug);
	static bool IsDebug();

	bool AddCommand(const CommandParseInfo &cpi);

	bool IsValid(std::string s);
	void Parse(std::string commands, std::function<void(bool, frc::Command *, float)> fAddCommand);

protected:
	std::vector<float> _ParseParameterList(std::string s);
	std::regex &_GetCommandListRx();
	std::regex &_GetParameterListRx();
	std::string _GetCommandName(const std::string &alias) const;
	CommandParseInfo _GetCommandParseInfo(const std::string &alias) const;
	std::string _BuildModeRx(bool capturing) const;
	std::string _BuildNameRx(bool capturing) const;
	std::string _BuildParametersRx(bool capturing) const;
	bool _IsDuplicateName(const std::string &s);
	bool _IsValidName(const std::string &s);

private:
	CommandListParser();

	static bool _debug;

	std::vector<CommandParseInfo> _commands;
	std::regex _rxCommandList;
	std::regex _rxParameterList;
	std::regex _rxValidName;
	bool _dirty;
	bool _haveParameterListRx;
};

// ==========================================================================
