#pragma once

// ================================================================

class IMultiController {
public:

	virtual void SetPercentPower(double value) = 0;
	virtual double GetEncoderPosition() = 0;

	virtual double GetPosition() const; // Return position in rotations
	virtual double GetVelocity() const; // Return velocity in rotations per minute (RPM)

	virtual bool IsForwardLimitSwitchClosed() const;
	virtual bool IsReverseLimitSwitchClosed() const;
};

// ================================================================
