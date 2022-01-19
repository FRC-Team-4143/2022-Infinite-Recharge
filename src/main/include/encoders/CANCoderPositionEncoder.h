#pragma once
#include "encoders/IPositionEncoder.h"
#include <ctre/Phoenix.h>
#include <memory>

// ==========================================================================

class CANCoderPositionEncoder : public IPositionEncoder {
public:

	CANCoderPositionEncoder(int canId);

	// IPositionEncoder methods

	// Return position in degrees.
	virtual double GetPosition() const override;

	// Establish initial position and offset.
	virtual void InitializePosition(double degrees) override;

	// Return offset so it can be saved to persistent storage.
	virtual double GetOffset() const override;

	// Restore offset from persistent storage and adjust position.
	virtual void SetOffset(double offset) override;

private:

	std::unique_ptr<CANCoder> _encoder;
	double _offset;
};

// ==========================================================================
