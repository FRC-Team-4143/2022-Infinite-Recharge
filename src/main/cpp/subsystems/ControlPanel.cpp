#include "subsystems/ControlPanel.h"

// ==========================================================================

ControlPanel::ControlPanel(int channel)
:	IControlPanel("ControlPanel") {
	_solenoid1 = std::make_unique<frc::Solenoid>(frc::PneumaticsModuleType::CTREPCM, channel);
	_panelIsUp = false;
}

// ==========================================================================

void ControlPanel::InitDefaultCommand() {
	// No default command needed for this subsystem.
}

// ==========================================================================

void ControlPanel::ColorUp() {
	_solenoid1->Set(true);
}

// ==========================================================================

void ControlPanel::ColorDown() {
	_solenoid1->Set(false);
}

// ==========================================================================

void ControlPanel::TogglePanel() {
	if (_panelIsUp) {
		ColorDown();
		_panelIsUp = false;
	}
	else {
		ColorUp();
		_panelIsUp = true;
	}
}

// ==========================================================================
