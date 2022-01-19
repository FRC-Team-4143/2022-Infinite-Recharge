#include "controllers/TalonFXController.h"

// ==========================================================================
// Number of sensor units per rotation for the motor sensor.
// @link https://github.com/CrossTheRoadElec/Phoenix-Documentation#what-are-the-units-of-my-sensor
const static int kSensorUnitsPerRotation = 2048;

// ==========================================================================

TalonFXController::TalonFXController(int canId, std::function<void(WPI_TalonFX*)> fnInitialize)
:	_motor{std::make_unique<WPI_TalonFX>(canId)} {
	if (fnInitialize != nullptr) {
		fnInitialize(_motor.get());
	}
}

// ==========================================================================

TalonFXController::TalonFXController(WPI_TalonFX* motor, std::function<void(WPI_TalonFX*)> fnInitialize)
:	_motor(motor) {
	if (fnInitialize != nullptr) {
		fnInitialize(_motor.get());
	}
}

// ==========================================================================

void TalonFXController::SetPercentPower(double value) {
	_motor->Set(ControlMode::PercentOutput, value);
}

// ==========================================================================

double TalonFXController::GetEncoderPosition() {
	return _motor->GetSelectedSensorPosition();
}

// ==========================================================================

double TalonFXController::GetPosition() const {
	auto position = _motor->GetSelectedSensorPosition(); // This returns sensor units.
	position /= kSensorUnitsPerRotation; // Convert to rotations.
	return position;
}

// ================================================================

double TalonFXController::GetVelocity() const {
	auto velocity = _motor->GetSelectedSensorVelocity(); // This returns sensor units per 100 ms.
	velocity /= kSensorUnitsPerRotation; // Convert to rotations per 100 ms.
	velocity *= 10; // Convert to rotations per second;
	velocity *= 60; // Convert to rotations per minute (RPM).
	return velocity;
}

// ================================================================

bool TalonFXController::IsForwardLimitSwitchClosed() const {
	return _motor->IsFwdLimitSwitchClosed() != 0; // Returns int; convert to bool.
}

// ================================================================

bool TalonFXController::IsReverseLimitSwitchClosed() const {
	return _motor->IsRevLimitSwitchClosed() != 0; // Returns int; convert to bool.
}

// ================================================================
