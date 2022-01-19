// ==========================================================================
// IWinch interface
// ==========================================================================
#pragma once
#include <frc/commands/Subsystem.h>

class IWinch : public frc::Subsystem {
public:

	IWinch(std::string_view name);

	virtual void WinchControl(float speed) = 0;
};

// ==========================================================================
