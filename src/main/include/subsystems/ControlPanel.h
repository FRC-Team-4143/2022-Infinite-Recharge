// ==========================================================================
// ControlPanel class
// The ControlPanel class implements an IControlPanel subsystem.
// ==========================================================================
#pragma once
#include "subsystems/IControlPanel.h"
#include <frc/Solenoid.h>
#include <memory>

// ==========================================================================

class ControlPanel : public IControlPanel {
public:

	ControlPanel(int channel);

	// Subsystem methods
	virtual void InitDefaultCommand() override;

	// IControlPanel methods
	virtual void ColorUp() override;
	virtual void ColorDown() override; 
	virtual void TogglePanel() override;

private:

	std::unique_ptr<frc::Solenoid> _solenoid1;
	bool _panelIsUp;
};

// ==========================================================================
