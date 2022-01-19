#include "Modules/NullSwerveModule.h"

NullSwerveModule::NullSwerveModule() {
}

void NullSwerveModule::SetGeometry(double x, double y, double maxradius) {
}

void NullSwerveModule::SetWheelOffset() {
}

void NullSwerveModule::LoadWheelOffset() {
}

void NullSwerveModule::SetDriveSpeed(float speed) {
}

double NullSwerveModule::GetSteerPosition() {
	return 0;
}

double NullSwerveModule::SetSteerDrive(double x, double y, double twist, bool operatorControl) {
	return 0;
}

double NullSwerveModule::GetSetpoint() {
	return 0;
}

double NullSwerveModule::GetPower() {
	return 0;
}
