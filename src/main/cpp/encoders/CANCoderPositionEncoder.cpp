#include "encoders/CANCoderPositionEncoder.h"

// ==========================================================================

CANCoderPositionEncoder::CANCoderPositionEncoder(int canId)
:	_encoder(std::make_unique<CANCoder>(canId)),
	_offset(0) {
	_encoder->ConfigFactoryDefault();
	_encoder->SetStatusFramePeriod(CANCoderStatusFrame_SensorData, 5, 20);
}

// ==========================================================================
// Return position in degrees.

double CANCoderPositionEncoder::GetPosition() const {
	return _encoder->GetPosition();
}

// ==========================================================================
// Establish initial position and offset.

void CANCoderPositionEncoder::InitializePosition(double degrees) {
	_offset = degrees - _encoder->GetAbsolutePosition();
	_encoder->SetPosition(degrees);
}

// ==========================================================================
// Return offset so it can be saved to persistent storage.

double CANCoderPositionEncoder::GetOffset() const {
	return _offset;
}

// ==========================================================================
// Restore offset from persistent storage and adjust position.

void CANCoderPositionEncoder::SetOffset(double offset) {
	_offset = offset;
	_encoder->SetPosition(_encoder->GetAbsolutePosition() + _offset);
}

// ==========================================================================
