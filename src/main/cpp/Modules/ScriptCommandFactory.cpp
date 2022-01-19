// ==========================================================================
// ScriptCommandFactory class
//
// FRC 4143: MARS/WARS
// ==========================================================================
// 2017-01-28 JKSalmon - Initial development
// ==========================================================================
#include "Modules/ScriptCommandFactory.h"
#include "commands/ScriptCommand.h"

// ==========================================================================

ScriptCommandFactory::ScriptCommandFactory()
: m_script(), m_pCommand() {
}

// ==========================================================================

ScriptCommandFactory::~ScriptCommandFactory() {
}

// ==========================================================================

ScriptCommandFactory& ScriptCommandFactory::GetInstance() {
	static ScriptCommandFactory instance;
	return instance;
}

// ==========================================================================

void ScriptCommandFactory::SetBlueprint(std::string script, bool prepare) {
	if (m_script != script) {
		m_script = script;
		m_pCommand.reset();
	}

	if (prepare) {
		PrepareCommand();
	}
}

// ==========================================================================

std::unique_ptr<frc::Command> ScriptCommandFactory::GetCommand() {
	PrepareCommand();
	return std::move(m_pCommand);
}

// ==========================================================================

void ScriptCommandFactory::PrepareCommand() {
	if (!_HaveOne()) {
		_BuildOne();
	}
}

// ==========================================================================

void ScriptCommandFactory::_BuildOne() {
	m_pCommand.reset(new ScriptCommand("ScriptCommand", m_script));
}

// ==========================================================================

bool ScriptCommandFactory::_HaveOne() const {
	return !!m_pCommand;
}

// ==========================================================================
