#include "Modules/TalonFXDiffSwerveModule.h"
#include "Modules/Constants.h"
#include <frc/Preferences.h>
#include <ctre/Phoenix.h>

// ================================================================

struct Gains {
	double kP;
	double kI;
	double kD;
	double kF;
	double kIzone;
	double kPeakOutput;
};

// ================================================================

const static int REMOTE_0 = 0;
const static int REMOTE_1 = 1;
const static int PID_PRIMARY = 0;
const static int PID_TURN = 1;
const static int SLOT_1 = 1;
const static int SLOT_2 = 2;

// Number of sensor units per rotation for the motor sensor.
// @link https://github.com/CrossTheRoadElec/Phoenix-Documentation#what-are-the-units-of-my-sensor
const static int kSensorUnitsPerRotation = 2048;

// Number of sensor units per rotation for the CANCoder.
constexpr static int kCANCoderUnitsPerRotation = 360;

// Using the config feature, scale units to 3600 per rotation.
// This is nice as it keeps 0.1 deg resolution, and is fairly intuitive.
constexpr static double kTurnTravelUnitsPerRotation = 3600;

// Set to zero to skip waiting for confirmation.
// Set to nonzero to wait and report to DS if action fails.
const static int kTimeoutMs = 30;

// Motor neutral dead-band, set to the minimum 0.1%.
constexpr static double kNeutralDeadband = 0.001;

// PID constants                           kP   kI   kD    kF   Iz  PeakOut
constexpr static Gains kGains_Velocit = { 0.2, 0.0, 0.0, 0.05, 300, 0.50 };
constexpr static Gains kGains_Turning = { 0.8, 0.0, 0.0, 0.00, 200, 0.25 };

const static int kSlot_Turning = SLOT_1;
const static int kSlot_Velocit = SLOT_2;

// ================================================================

TalonFXDiffSwerveModule::TalonFXDiffSwerveModule(int masterId, int slaveId, std::string configName, CANCoder* headingSensor, bool invert)
	: _master(masterId),
	_slave(slaveId),
	_configName(configName),
	_headingSensor(headingSensor) {
	_x = 3;
	_y = 4;
	_radius = 5;
	_setpoint = 0;
	_offset = 0;
	_lastPow = 0;
	_inverse = 1;

	ConfigMotors(invert);
}

// ================================================================

void TalonFXDiffSwerveModule::SetGeometry(double x, double y, double maxradius) {
	_x = x;
	_y = y;
	_radius = maxradius;
}
void TalonFXDiffSwerveModule::Stop() {
	_master.Set(ControlMode::Velocity,0);
	_slave.Set(ControlMode::Velocity,0);
}

// ================================================================

void TalonFXDiffSwerveModule::SetWheelOffset() {
	auto steerPosition = GetSteerPosition();
	auto prefs = frc::Preferences::GetInstance();
	prefs->PutDouble(_configName, steerPosition);
	SetOffset(steerPosition);

}

// ================================================================

void TalonFXDiffSwerveModule::LoadWheelOffset() {
	auto prefs = frc::Preferences::GetInstance();
	auto steerPosition = prefs->GetDouble(_configName);
	SetOffset(steerPosition);
}

// ================================================================

void TalonFXDiffSwerveModule::SetDriveSpeed(float speed) {
	_lastPow = speed;

	constexpr int MAX_RPM = 3000;
	double target_RPM = speed * MAX_RPM; // +- MAX_RPM
	auto target_unitsPer100ms = target_RPM * kSensorUnitsPerRotation / (60 * 10.0);

	auto heading_units = _setpoint * (kTurnTravelUnitsPerRotation / kCANCoderUnitsPerRotation);

	auto target0 = target_unitsPer100ms; // speed
	auto target1 = heading_units; // turning

	_master.Set(ControlMode::Velocity, target0 * _inverse, DemandType_AuxPID, target1);
	_slave.Follow(_master, FollowerType::FollowerType_AuxOutput1);
}

// ================================================================

double TalonFXDiffSwerveModule::GetSteerPosition() {
	return _headingSensor->GetPosition();
}

// ================================================================

double TalonFXDiffSwerveModule::SetSteerDrive(double x, double y, double twist, bool operatorControl) {
	static constexpr double pi = 3.141592653589793238462643383;

	//auto signX = (_x >= 0) ? 1 : -1;
	//auto signY = (_y >= 0) ? 1 : -1;

	auto BP = x + twist * (_x) / _radius;
	auto CP = y - twist * (_y) / _radius;

	const auto HALF_TURN = kCANCoderUnitsPerRotation / 2;
	float setpoint = HALF_TURN;

	if (BP != 0 || CP != 0) {
		setpoint = (HALF_TURN + HALF_TURN / pi * atan2(BP, CP));
	}

	setpoint = -setpoint;
		SetSteerSetpoint(setpoint + (_offset));

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

double TalonFXDiffSwerveModule::GetSetpoint() {
	return _setpoint;
}

// ================================================================

double TalonFXDiffSwerveModule::GetPower() {
	return _lastPow;
}

// ================================================================

bool TalonFXDiffSwerveModule::InDeadZone(double value) {
	return fabs(value) <= Constants::DEAD_ZONE;
}

// ================================================================

void TalonFXDiffSwerveModule::ConfigMotors(bool invert) {
	_master.ConfigFactoryDefault();
	_slave.ConfigFactoryDefault();

	_headingSensor->ConfigFactoryDefault();
	_headingSensor->SetStatusFramePeriod(CANCoderStatusFrame_SensorData, 5,20);

	_master.Set(ControlMode::PercentOutput, 0);
	_slave.Set(ControlMode::PercentOutput, 0);

	constexpr double MAX_CURRENT = 20.0;

	SupplyCurrentLimitConfiguration supply(true, MAX_CURRENT, MAX_CURRENT, 10);
	_slave.ConfigSupplyCurrentLimit(supply);
	_master.ConfigSupplyCurrentLimit(supply);

	StatorCurrentLimitConfiguration stator(true, MAX_CURRENT, MAX_CURRENT, 10);
	_slave.ConfigStatorCurrentLimit(stator);
	_master.ConfigStatorCurrentLimit(stator);

	_slave.SetInverted(true);
	_slave.SetSensorPhase(true);
	_master.SetInverted(invert);
	_master.SetSensorPhase(true);

	// other side is quad
	_slave.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, PID_PRIMARY, kTimeoutMs);

	// Remote 0 will be the other side's Talon
	_master.ConfigRemoteFeedbackFilter(_slave.GetDeviceID(), RemoteSensorSource::RemoteSensorSource_TalonFX_SelectedSensor, REMOTE_0, kTimeoutMs);

	// Remote 1 will be a CANCoder
	_master.ConfigRemoteFeedbackFilter(_headingSensor->GetDeviceNumber(), RemoteSensorSource::RemoteSensorSource_CANCoder, REMOTE_1, kTimeoutMs);

	// setup sum and difference signals
	_master.ConfigSensorTerm(SensorTerm::SensorTerm_Sum0, FeedbackDevice::RemoteSensor0, kTimeoutMs);
	_master.ConfigSensorTerm(SensorTerm::SensorTerm_Sum1, FeedbackDevice::QuadEncoder, kTimeoutMs);
	_master.ConfigSensorTerm(SensorTerm::SensorTerm_Diff1, FeedbackDevice::RemoteSensor0, kTimeoutMs);
	_master.ConfigSensorTerm(SensorTerm::SensorTerm_Diff0, FeedbackDevice::QuadEncoder, kTimeoutMs);

	// select sum for distance(0), different for turn(1)
	_master.ConfigSelectedFeedbackSensor(FeedbackDevice::SensorSum, PID_PRIMARY, kTimeoutMs);

	/* do not scale down the primary sensor (distance).  If selected sensor is going to be a sensorSum
	 * user can pass 0.5 to produce an average.
	 */
	_master.ConfigSelectedFeedbackCoefficient(1.0, PID_PRIMARY, kTimeoutMs);
	_master.ConfigSelectedFeedbackSensor(FeedbackDevice::RemoteSensor1, PID_TURN, kTimeoutMs);
	_master.ConfigSelectedFeedbackCoefficient(kTurnTravelUnitsPerRotation / kCANCoderUnitsPerRotation, PID_TURN, kTimeoutMs);

	_master.SetStatusFramePeriod(StatusFrame::Status_12_Feedback1_, 20, kTimeoutMs);
	_master.SetStatusFramePeriod(StatusFrame::Status_13_Base_PIDF0_, 20, kTimeoutMs);
	_master.SetStatusFramePeriod(StatusFrame::Status_14_Turn_PIDF1_, 20, kTimeoutMs);
	_master.SetStatusFramePeriod(StatusFrame::Status_10_Targets_, 20, kTimeoutMs);

	// Speed up the left since we are polling its sensor
	_slave.SetStatusFramePeriod(StatusFrame::Status_2_Feedback0_, 20, kTimeoutMs);

	_master.ConfigNeutralDeadband(kNeutralDeadband, kTimeoutMs);
	_slave.ConfigNeutralDeadband(kNeutralDeadband, kTimeoutMs);

	// max out the peak output (for all modes). However you can
	// limit the output of a given PID object with ConfigClosedLoopPeakOutput().
	_slave.ConfigPeakOutputForward(+1.0, kTimeoutMs);
	_slave.ConfigPeakOutputReverse(-1.0, kTimeoutMs);
	_master.ConfigPeakOutputForward(+1.0, kTimeoutMs);
	_master.ConfigPeakOutputReverse(-1.0, kTimeoutMs);

	// turn servo
	_master.Config_kP(kSlot_Turning, kGains_Turning.kP, kTimeoutMs);
	_master.Config_kI(kSlot_Turning, kGains_Turning.kI, kTimeoutMs);
	_master.Config_kD(kSlot_Turning, kGains_Turning.kD, kTimeoutMs);
	_master.Config_kF(kSlot_Turning, kGains_Turning.kF, kTimeoutMs);
	_master.Config_IntegralZone(kSlot_Turning, kGains_Turning.kIzone, kTimeoutMs);
	_master.ConfigClosedLoopPeakOutput(kSlot_Turning, kGains_Turning.kPeakOutput, kTimeoutMs);

	// velocity servo
	_master.Config_kP(kSlot_Velocit, kGains_Velocit.kP, kTimeoutMs);
	_master.Config_kI(kSlot_Velocit, kGains_Velocit.kI, kTimeoutMs);
	_master.Config_kD(kSlot_Velocit, kGains_Velocit.kD, kTimeoutMs);
	_master.Config_kF(kSlot_Velocit, kGains_Velocit.kF, kTimeoutMs);
	_master.Config_IntegralZone(kSlot_Velocit, kGains_Velocit.kIzone, kTimeoutMs);
	_master.ConfigClosedLoopPeakOutput(kSlot_Velocit, kGains_Velocit.kPeakOutput, kTimeoutMs);

	_slave.SetNeutralMode(NeutralMode::Brake);
	_master.SetNeutralMode(NeutralMode::Brake);

	/* 1ms per loop.  PID loop can be slowed down if need be.
	 * For example,
	 * - if sensor updates are too slow
	 * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
	 * - sensor movement is very slow causing the derivative error to be near zero.
	 */
	int closedLoopTimeMs = 1;
	_master.ConfigSetParameter(ParamEnum::ePIDLoopPeriod, closedLoopTimeMs, 0x00, PID_PRIMARY, kTimeoutMs);
	_master.ConfigSetParameter(ParamEnum::ePIDLoopPeriod, closedLoopTimeMs, 0x00, PID_TURN, kTimeoutMs);

	/**
	 * false means talon's local output is PID0 + PID1, and other side Talon is PID0 - PID1
	 * true means talon's local output is PID0 - PID1, and other side Talon is PID0 + PID1
	 */
	_master.ConfigAuxPIDPolarity(false, kTimeoutMs);

	ConfigureNeutralOnLOS(true);

	ZeroSensors();

	// Moved here from the "first call" section of the two-axis velocity method
	_master.SelectProfileSlot(kSlot_Velocit, PID_PRIMARY);
	_master.SelectProfileSlot(kSlot_Turning, PID_TURN);
}

// ================================================================
// Configure neutral on loss of signal

void TalonFXDiffSwerveModule::ConfigureNeutralOnLOS(bool enable) {
	double value = enable ? 0 : 1;
	_master.ConfigSetParameter(
		ParamEnum::eRemoteSensorClosedLoopDisableNeutralOnLOS,
		value,
		0x00,
		0x00,
		kTimeoutMs
	);
}

// ================================================================

void TalonFXDiffSwerveModule::SetOffset(float offset) {
	_offset = offset;
}

// ================================================================

void TalonFXDiffSwerveModule::SetSteerSetpoint(float setpoint) {
	float currentPosition = _headingSensor->GetPosition() / kCANCoderUnitsPerRotation;
	int turns = trunc(currentPosition);
	float currentAngle = currentPosition - turns;

	const auto FULL_TURN = kCANCoderUnitsPerRotation;
	const auto HALF_TURN = FULL_TURN / 2;

	currentPosition *= FULL_TURN;
	turns *= FULL_TURN;
	currentAngle *= FULL_TURN;

	float angleOptions[6];
	angleOptions[0] = turns - FULL_TURN + setpoint;
	angleOptions[1] = turns - FULL_TURN + setpoint + HALF_TURN;
	angleOptions[2] = turns + setpoint;
	angleOptions[3] = turns + setpoint + HALF_TURN;
	angleOptions[4] = turns + FULL_TURN + setpoint;
	angleOptions[5] = turns + FULL_TURN + setpoint + HALF_TURN;

	int firstoption = 0;
	int optionincr = 1;

	// This prevents motors from having to reverse if they are already
	// rotating. They may take a longer rotation but will keep spinning
	// the same way.
#if 0
	if (_lastPow > .3) { // maybe should read speed instead of last power
		optionincr = 2;
		if (_inverse == -1) {
			firstoption = 1;
		}
	}
#endif

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

void TalonFXDiffSwerveModule::ZeroSensors() {
	_slave.GetSensorCollection().SetIntegratedSensorPosition(0, kTimeoutMs);
	_master.GetSensorCollection().SetIntegratedSensorPosition(0, kTimeoutMs);
	_headingSensor->SetPosition(0, kTimeoutMs);
}

// ================================================================
