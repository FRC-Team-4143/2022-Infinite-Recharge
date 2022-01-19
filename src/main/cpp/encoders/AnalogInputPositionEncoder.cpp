#include "encoders/AnalogInputPositionEncoder.h"

// ==========================================================================

AnalogInputPositionEncoder::AnalogInputPositionEncoder(int channel)
:	_encoder(std::make_unique<frc::AnalogPotentiometer>(channel, 360, 0)),
	_offset(0) {
}

// ==========================================================================
// Return position in degrees.

double AnalogInputPositionEncoder::GetPosition() const {
	return _encoder->Get() + _offset;
}

// ==========================================================================
// Establish initial position and offset.

void AnalogInputPositionEncoder::InitializePosition(double degrees) {
	_offset = degrees - _encoder->Get();
}

// ==========================================================================
// Return offset so it can be saved to persistent storage.

double AnalogInputPositionEncoder::GetOffset() const {
	return _offset;
}

// ==========================================================================
// Restore offset from persistent storage and adjust position.

void AnalogInputPositionEncoder::SetOffset(double offset) {
	_offset = offset;
}

// ==========================================================================
