#pragma once
#include <frc/Joystick.h>
#include "commands/CrabDrive.h"

// ==========================================================================

class OI {
public:

	OI();

	float GetJoystickX();
	float GetJoystickY();
	float GetJoystickZ();
	float GetJoystick2X();
	float GetJoystick2Y();
	float GetJoystick2Z();

	float GetRightJoystickY();

	float GetLeftTrigger();
	float GetRightTrigger();
	float GetLeftTrigger2();
	float GetRightTrigger2();

	bool GetButtonLeft();
	bool GetButtonRight();
	bool GetButtonA();
	bool GetButtonB();
	bool GetButtonA2();
	bool GetButtonB2();
	bool GetButtonX();
	bool GetButtonX2();
	bool GetButtonY();
	bool GetButtonY2();
	bool GetButtonBack();
	bool GetButtonStart();
	bool GetLeftBumper();
	bool GetRightBumper();
	bool GetLeftBumper2();
	bool GetRightBumper2();
	int GetPOV2();
	int GetPOV1();
	bool GetButtonLeft2();

private:

	frc::Joystick* _driverJoystick;
	frc::Joystick* _driverJoystick2;
	CrabDrive* _crabDrive;
};

// ==========================================================================
