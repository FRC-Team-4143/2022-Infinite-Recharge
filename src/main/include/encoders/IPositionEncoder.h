#pragma once

// ==========================================================================

class IPositionEncoder {
public:

	// Return position in degrees.
	virtual double GetPosition() const = 0;

	// Establish initial position and offset.
	virtual void InitializePosition(double degrees) = 0;

	// Return offset so it can be saved to persistent storage.
	virtual double GetOffset() const = 0;

	// Restore offset from persistent storage and adjust position.
	virtual void SetOffset(double offset) = 0;
};

// ==========================================================================
