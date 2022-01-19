#include "OI.h"
#include "commands/ExtendClimber.h"
#include "commands/ExtendPickUp.h"
#include "commands/RetractClimber.h"
#include "commands/RetractPickUp.h"
#include "commands/ScriptValidate.h"
#include "commands/Shoot.h"
#include "commands/TogglePanel.h"
#include "commands/FinalClimbMultiSubAssign.h"

//non-joystick commands
#include "commands/SetWheelOffsets.h"
#include "commands/ZeroYaw.h"
#include "commands/ZeroTurret.h"

#include "Modules/Constants.h"
#include <frc/buttons/JoystickButton.h>
#include <frc/smartdashboard/SmartDashboard.h>

const uint32_t JOYSTICK_LX_AXIS = 0;
const uint32_t JOYSTICK_LY_AXIS = 1;
const uint32_t JOYSTICK_LTRIG_AXIS = 2;
const uint32_t JOYSTICK_RTRIG_AXIS = 3;
const uint32_t JOYSTICK_RX_AXIS = 4;
const uint32_t JOYSTICK_RY_AXIS = 5;

const uint32_t JOYSTICK_BUTTON_A = 1;
const uint32_t JOYSTICK_BUTTON_B = 2;
const uint32_t JOYSTICK_BUTTON_X = 3;
const uint32_t JOYSTICK_BUTTON_Y = 4;
const uint32_t JOYSTICK_BUTTON_LB = 5;
const uint32_t JOYSTICK_BUTTON_RB = 6;
const uint32_t JOYSTICK_BUTTON_BACK = 7;
const uint32_t JOYSTICK_BUTTON_START = 8;
const uint32_t JOYSTICK_BUTTON_LEFT = 9;
const uint32_t JOYSTICK_BUTTON_RIGHT = 10;

// ==========================================================================

OI::OI() {
	_driverJoystick = new frc::Joystick(0);
	_driverJoystick2 = new frc::Joystick(1);
	_crabDrive = new CrabDrive();

	frc::SmartDashboard::PutData("Set WheelOffsets", new SetWheelOffsets());
	frc::SmartDashboard::PutData("Zero Yaw", new ZeroYaw());
	frc::SmartDashboard::PutData("Zero Turret", new ZeroTurret());
	(new frc::JoystickButton(_driverJoystick, JOYSTICK_BUTTON_LEFT))->ToggleWhenPressed(_crabDrive);
	//Right Trigger mapped to Turbo Mode
	//Left Trigger mapped to Pickup Intake Roller
	(new frc::JoystickButton(_driverJoystick, JOYSTICK_BUTTON_RIGHT))->WhenPressed( new TogglePanel());
	(new frc::JoystickButton(_driverJoystick, JOYSTICK_BUTTON_LB))->WhenPressed(new ExtendPickUp());
	(new frc::JoystickButton(_driverJoystick, JOYSTICK_BUTTON_RB))->WhenPressed(new RetractPickUp());
	//(new frc::JoystickButton(_driverJoystick, JOYSTICK_BUTTON_B))->WhenPressed(new Shoot());

	//Turret mapped to Right Joystick (X)
	//Shooter mapped to Right Trigger 
	//Winch mapped to Left Joystick
	//Stir mapped to A and B
	//(new frc::JoystickButton(_driverJoystick2, JOYSTICK_BUTTON_START))->WhenPressed(new ExtendClimber()); // old climber
	(new frc::JoystickButton(_driverJoystick2, JOYSTICK_BUTTON_START))->WhenPressed(new FinalClimbMultiSubAssign()); // new climber
	(new frc::JoystickButton(_driverJoystick2, JOYSTICK_BUTTON_BACK))->WhenPressed(new RetractClimber());
	//(new frc::JoystickButton(_driverJoystick2, ))->ToggleWhenPressed(new Shoot());

	frc::SmartDashboard::PutData("Validate Script", new ScriptValidate());
}

// ==========================================================================

float OI::GetJoystickX() {
	auto value = _driverJoystick->GetRawAxis(JOYSTICK_LX_AXIS);
	return (fabs(value) <= Constants::DEAD_ZONE) ? 0 : value;
}

// ==========================================================================

float OI::GetJoystick2X() {
	auto value = _driverJoystick2->GetRawAxis(JOYSTICK_LX_AXIS);
	return (fabs(value) <= Constants::DEAD_ZONE*3) ? 0 : value;
}

// ==========================================================================

float OI::GetJoystick2Y() {
	auto value = _driverJoystick2->GetRawAxis(JOYSTICK_LY_AXIS);
	if(value > Constants::DEAD_ZONE*3) return value - Constants::DEAD_ZONE*3;
	if(value < -Constants::DEAD_ZONE*3) return value + Constants::DEAD_ZONE*3;
	return 0;
	//return (fabs(value) <= Constants::DEAD_ZONE*2) ? 0 : value;
}

// ==========================================================================

float OI::GetJoystick2Z() {
	auto value = _driverJoystick2->GetRawAxis(JOYSTICK_RX_AXIS);
	if(value > Constants::DEAD_ZONE*3) return value - Constants::DEAD_ZONE*3;
	if(value < -Constants::DEAD_ZONE*3) return value + Constants::DEAD_ZONE*3;
	return 0;

	//return (fabs(value) <= Constants::DEAD_ZONE*2) ? 0 : value;
}

// ==========================================================================

float OI::GetJoystickY() {
	auto value = _driverJoystick->GetRawAxis(JOYSTICK_LY_AXIS);
	return (fabs(value) <= Constants::DEAD_ZONE) ? 0 : value;
}

// ==========================================================================

float OI::GetJoystickZ() {
	auto value = _driverJoystick->GetRawAxis(JOYSTICK_RX_AXIS);
	return (fabs(value) <= Constants::DEAD_ZONE) ? 0 : value;
}

// ==========================================================================

float OI::GetRightJoystickY() {
	auto value = _driverJoystick->GetRawAxis(JOYSTICK_RY_AXIS);
	return (fabs(value) <= Constants::DEAD_ZONE) ? 0 : value;
}

// ==========================================================================

float OI::GetLeftTrigger() {
	float value = fabs(_driverJoystick->GetRawAxis(JOYSTICK_LTRIG_AXIS));
	return (value <= Constants::DEAD_ZONE) ? 0 : value;
}

// ==========================================================================

float OI::GetRightTrigger() {
	float value = fabs(_driverJoystick->GetRawAxis(JOYSTICK_RTRIG_AXIS));
	return (value <= Constants::DEAD_ZONE) ? 0 : value;
}

// ==========================================================================

float OI::GetLeftTrigger2() {
	float value = fabs(_driverJoystick2->GetRawAxis(JOYSTICK_LTRIG_AXIS));
	return (value <= Constants::DEAD_ZONE) ? 0 : value;
}

// ==========================================================================

float OI::GetRightTrigger2() {
	float value = fabs(_driverJoystick2->GetRawAxis(JOYSTICK_RTRIG_AXIS));
	return (value <= Constants::DEAD_ZONE) ? 0 : value;
}

// ==========================================================================

bool OI::GetButtonLeft() {
	return _driverJoystick->GetRawButton(JOYSTICK_BUTTON_LEFT);
}

// ==========================================================================

bool OI::GetButtonRight() {
	return _driverJoystick->GetRawButton(JOYSTICK_BUTTON_RIGHT);
}

// ==========================================================================

bool OI::GetButtonA() {
	return _driverJoystick->GetRawButton(JOYSTICK_BUTTON_A);
}

// ==========================================================================

bool OI::GetButtonB() {
	return _driverJoystick->GetRawButton(JOYSTICK_BUTTON_B);
}

// ==========================================================================

bool OI::GetButtonA2() {
	return _driverJoystick2->GetRawButton(JOYSTICK_BUTTON_A);
}

// ==========================================================================

bool OI::GetButtonB2() {
	return _driverJoystick2->GetRawButton(JOYSTICK_BUTTON_B);
}

// ==========================================================================

bool OI::GetButtonX() {
	return _driverJoystick->GetRawButton(JOYSTICK_BUTTON_X);
}

// ==========================================================================

bool OI::GetButtonX2() {
	return _driverJoystick2->GetRawButton(JOYSTICK_BUTTON_X);
}

// ==========================================================================

bool OI::GetButtonY() {
	return _driverJoystick->GetRawButton(JOYSTICK_BUTTON_Y);
}

// ==========================================================================

bool OI::GetButtonY2() {
	return _driverJoystick2->GetRawButton(JOYSTICK_BUTTON_Y);
}

// ==========================================================================

bool OI::GetButtonBack() {
	return _driverJoystick->GetRawButton(JOYSTICK_BUTTON_BACK);
}

// ==========================================================================

bool OI::GetButtonStart() {
	return _driverJoystick->GetRawButton(JOYSTICK_BUTTON_START);
}

// ==========================================================================

bool OI::GetLeftBumper() {
	return _driverJoystick->GetRawButton(JOYSTICK_BUTTON_LB);
}

// ==========================================================================

bool OI::GetRightBumper() {
	return _driverJoystick->GetRawButton(JOYSTICK_BUTTON_RB);
}

// ==========================================================================

bool OI::GetLeftBumper2() {
	return _driverJoystick2->GetRawButton(JOYSTICK_BUTTON_LB);
}

// ==========================================================================

bool OI::GetRightBumper2() {
	return _driverJoystick2->GetRawButton(JOYSTICK_BUTTON_RB);
}

// ==========================================================================

int OI::GetPOV2() {
	return _driverJoystick2->GetPOV();
}

int OI::GetPOV1() {
	return _driverJoystick->GetPOV();
}
bool OI::GetButtonLeft2() {
	return _driverJoystick2->GetRawButton(JOYSTICK_BUTTON_LEFT);
}