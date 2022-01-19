#include "controllers/IMultiController.h"

// ================================================================

double IMultiController::GetPosition() const {
	return 0;
}

// ================================================================

double IMultiController::GetVelocity() const {
	return 0;
}

// ================================================================

bool IMultiController::IsForwardLimitSwitchClosed() const {
	return false;
}

// ================================================================

bool IMultiController::IsReverseLimitSwitchClosed() const {
	return false;
}

// ================================================================
