#include "commands/TogglePanel.h"
#include "Robot.h"

// ==========================================================================

TogglePanel::TogglePanel()
:	frc::Command("TogglePanel") {
	Requires(Robot::controlPanel.get());
}

// ==========================================================================

void TogglePanel::Initialize() {
}

// ==========================================================================

void TogglePanel::Execute() {
	Robot::controlPanel->TogglePanel();
}

// ==========================================================================

bool TogglePanel::IsFinished() {
	return true;
}

// ==========================================================================

void TogglePanel::End() {
}

// ==========================================================================

void TogglePanel::Interrupted() {
	End();
}

// ==========================================================================
