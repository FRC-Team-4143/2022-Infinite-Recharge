#include "Modules/BasicDiffSwerveModule.h"
#include "Modules/Constants.h"
#include <frc/Preferences.h>
#include <cmath>
// ================================================================

BasicDiffSwerveModule::BasicDiffSwerveModule(IVelocityMultiController* master, IVelocityMultiController* slave, std::string configName, IPositionEncoder* headingSensor)
:	_master(master), _slave(slave), _configName(configName), _headingSensor(headingSensor) {
	_x = 3;
	_y = 4;
	_radius = 5;
	_setpoint = 0;
	_lastPower = 0;
	_inverse = 1;
}

// ================================================================

void BasicDiffSwerveModule::SetGeometry(double x, double y, double maxRadius) {
	_x = x;
	_y = y;
	_radius = maxRadius;
}

void BasicDiffSwerveModule::Stop() {
	_master->SetVelocity(0);
	_slave->SetVelocity(0);
}

// ================================================================

void BasicDiffSwerveModule::SetWheelOffset() {
	_headingSensor->InitializePosition(0); // Used to be 0
	auto offset = _headingSensor->GetOffset();
	frc::Preferences::SetDouble(_configName, offset);
}

// ================================================================

void BasicDiffSwerveModule::LoadWheelOffset() {
	auto offset = frc::Preferences::GetDouble(_configName);
	_headingSensor->SetOffset(offset);
}

// ================================================================

void BasicDiffSwerveModule::SetDriveSpeed(float speed) {
	_lastPower = speed;

	auto turn = GetSteerPosition() - _setpoint;

	if (std::fabs(turn) < 1) {
		turn = 0;
	}
	else {
	    //turn *= 2000.0 / 180; // simple gain on steering error
		turn *= 1. / 180.;
	}

	//_master->SetVelocity((speed * _inverse * 6000) - turn);
	//_slave->SetVelocity(-(speed * _inverse * 6000) - turn);
	_master->SetPercentPower((speed * _inverse) - turn);
	_slave->SetPercentPower(-(speed * _inverse) - turn);
}

// ================================================================

double BasicDiffSwerveModule::GetSteerPosition() {
	return _headingSensor->GetPosition();
}

// ================================================================

double BasicDiffSwerveModule::SetSteerDrive(double x, double y, double twist, bool operatorControl) {
	static constexpr double pi = 3.141592653589793238462643383;

	//auto signX = (_x >= 0) ? 1 : -1;
	//auto signY = (_y >= 0) ? 1 : -1;
	

	auto BP = x + twist * _x / _radius;
	auto CP = y - twist * _y / _radius;

	float setpoint = 180;

	if (BP != 0 || CP != 0) {
		setpoint = 180 + 180 / pi * atan2(BP, CP);
	}

	setpoint = -setpoint;
	SetSteerSetpoint(setpoint);

	auto power = sqrt(pow(BP, 2) + pow(CP, 2));

	if (operatorControl && InDeadZone(x) && InDeadZone(y) && InDeadZone(twist)) {
		power = 0;
	}
/*
	if (signX == signY) {
		power = -power;
	}
	if (signX == -1) {
		power = -power;
	}
*/
	return power;
}

// ================================================================

double BasicDiffSwerveModule::GetSetpoint() {
	return _setpoint;
}

// ================================================================

double BasicDiffSwerveModule::GetPower() {
	return _lastPower;
}

// ================================================================

bool BasicDiffSwerveModule::InDeadZone(double value) {
	return fabs(value) <= Constants::DEAD_ZONE;
}

// ================================================================

constexpr static int kCountsPerRotation = 360;

void BasicDiffSwerveModule::SetSteerSetpoint(float setpoint) {
	auto currentPosition = GetSteerPosition() / kCountsPerRotation;
	auto turns = trunc(currentPosition);

	currentPosition *= 360;
	turns *= 360;

	float angleOptions[6];
	angleOptions[0] = turns + setpoint - 360;
	angleOptions[1] = turns + setpoint - 180;
	angleOptions[2] = turns + setpoint;
	angleOptions[3] = turns + setpoint + 180;
	angleOptions[4] = turns + setpoint + 360;
	angleOptions[5] = turns + setpoint + 540;

	int firstoption = 0;
	int optionincr = 1;

	// this prevents motors from having to reverse
	// if they are already rotating
	// they may take a longer rotation but will keep spinning the same way
	if (_lastPower > .3) { // maybe should read speed instead of last power
		optionincr = 2;
		if (_inverse == -1) {
			firstoption = 1;
		}
	}

	float minMove = 360 * 5; // impossibly big angle
	int minI = 0;
	for (int i = firstoption; i < 6; i += optionincr) {
		if (fabs(currentPosition - angleOptions[i]) < minMove) {
			minMove = fabs(currentPosition - angleOptions[i]);
			minI = i;
		}
	}

	_setpoint = angleOptions[minI];

	_inverse = (minI % 2) ? -1 : 1;
}

// ================================================================
