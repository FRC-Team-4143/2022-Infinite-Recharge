#include "Robot.h"
#include <frc/commands/Scheduler.h>
#include <frc/RobotController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/livewindow/LiveWindow.h>

#include <rev/ColorSensorV3.h>
#include <rev/ColorMatch.h>
#include <iostream>

#include "controllers/SparkMaxController.h"
#include "controllers/SteerTalonController.h"
#include "controllers/TalonSRXController.h"
#include "controllers/VelocitySparkController.h"
#include "controllers/VelocityTalonFXController.h"
#include "controllers/VictorController.h"

#include "encoders/CANCoderPositionEncoder.h"

#include "Modules/BasicDiffSwerveModule.h"
#include "Modules/CommandListParser.h"
#include "Modules/Constants.h"
#include "Modules/DiffSwerveModule.h"
#include "Modules/Logger.h"
#include "Modules/ScriptEngine.h"
#include "Modules/SwerveModule.h"
#include "Modules/TalonFXDiffSwerveModule.h"

#include "subsystems/Climber.h"
#include "subsystems/ControlPanel.h"
#include "subsystems/PickUp.h"
#include "subsystems/Shooter.h"
#include "subsystems/Winch.h"

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include "commands/Shoot.h"

// If not using DIFFSWERVE, must set ONE of the following to 1:
#define USING_SPARKMAX_DRIVE 1
#define USING_VICTOR_DRIVE 0
#define USING_TALON_DRIVE 0

#define NAVX_MXP 1

// Channels and CAN IDS for climber Subsystem
constexpr int CLIMBER_EXTENDER_FWD = 3;
constexpr int CLIMBER_EXTENDER_REV = 2;
constexpr int CLIMBER_BOATROLLER= 16;

//CAN IDs for PickUp Subsystem
constexpr int PICKUP_INTAKE_CAN = 10;

// PCM channels for pickup solenoids
constexpr int PICKUP_SOL_FWD = 0;
constexpr int PICKUP_SOL_REV = 4;

// PCM channel for brakes
constexpr int WINCH_BRAKE = 5;
constexpr int CONTROLPANEL_SOL= 1;
constexpr int ROLLERBRAKE_SOL_FWD = 6;
constexpr int ROLLERBRAKE_SOL_REV = 7;

// Shooter Ids
constexpr int FEED_MOTOR = 11;
constexpr int SHOOTER_MOTOR = 14;
constexpr int STIR_MOTOR = 12;
constexpr int TURRET_MOTOR = 13;
constexpr int WINCH_MOTOR = 15;

//#define CLAMP 12

//======= System Definition =======//

OI* Robot::oi = nullptr;
DriveTrain* Robot::driveTrain = nullptr;
GyroSub* Robot::gyroSub = nullptr;
VisionBridgeSub* Robot::visionBridge = nullptr;

static constexpr auto i2cPort = frc::I2C::Port::kOnboard;

//======= Motor Definition =======//

ISwerveModule* Robot::frontLeftModule = nullptr;
ISwerveModule* Robot::frontRightModule = nullptr;
ISwerveModule* Robot::rearLeftModule = nullptr;
ISwerveModule* Robot::rearRightModule  = nullptr;

std::unique_ptr<IClimber> Robot::climber;
std::unique_ptr<IPickUp> Robot::pickUp;
std::unique_ptr<IShooter> Robot::shooter;
std::unique_ptr<IWinch> Robot::winch;
std::unique_ptr<IControlPanel> Robot::controlPanel;

//IMultiController* Robot::clampMotor = nullptr;

AHRS* Robot::navx = nullptr;

/*
rev::ColorSensorV3 m_colorSensor{i2cPort};
rev::ColorMatch m_colorMatcher;

static constexpr frc::Color kBlueTarget = frc::Color(0.121, 0.406, 0.472);
static constexpr frc::Color kGreenTarget = frc::Color(0.163, 0.572, 0.264);
static constexpr frc::Color kRedTarget = frc::Color(0.524, 0.339, 0.139);
static constexpr frc::Color kYellowTarget = frc::Color(0.317, 0.558, 0.124);
*/

// ================================================================

void Robot::RobotInit() {
	DeviceInitialization();

	frc::SmartDashboard::PutNumber("Yaw Offset", 0);

	ScriptEngine::GetInstance().Initialize();
	_autonomousCommand = nullptr;

	driveTrain->LoadWheelOffsets();
	driveTrain->SetWheelbase(30.25 ,20.5);

	/*
	m_colorMatcher.AddColorMatch(kBlueTarget);
	m_colorMatcher.AddColorMatch(kGreenTarget);
	m_colorMatcher.AddColorMatch(kRedTarget);
	m_colorMatcher.AddColorMatch(kYellowTarget);
	*/

	LOG("RobotInit end");
}

// ================================================================

void Robot::RobotPeriodic() {

	//Subsystem Commands

	if (navx != nullptr) {
		auto yawOff = frc::SmartDashboard::GetNumber("Yaw Offset", 0);
		frc::SmartDashboard::PutNumber("Yaw", navx->GetYaw() + yawOff);
	}
	//color stuff
		/*
		frc::Color detectedColor = m_colorSensor.GetColor();

		std::string colorString;
		double confidence = 0.0;
		frc::Color matchedColor = m_colorMatcher.MatchClosestColor(detectedColor, confidence);

		if (matchedColor == kBlueTarget) {
			colorString = "Blue";
		}
		else if (matchedColor == kRedTarget) {
			colorString = "Red";
		}
		else if (matchedColor == kGreenTarget) {
			colorString = "Green";
		}
		else if (matchedColor == kYellowTarget) {
			colorString = "Yellow";
		}
		else {
			colorString = "Unknown";
		}

		frc::SmartDashboard::PutNumber("Red", detectedColor.red);
		frc::SmartDashboard::PutNumber("Green", detectedColor.green);
		frc::SmartDashboard::PutNumber("Blue", detectedColor.blue);
		frc::SmartDashboard::PutNumber("Confidence", confidence);
		frc::SmartDashboard::PutString("ColorDetected", colorString);
		*/

	//Wheel Offset Code;
		if (frc::RobotController::GetUserButton() == 1 && _counter == 0) {
			driveTrain->SetWheelOffsets();
			_counter = 100;
			std::cout << "SetWheelOffsets Complete" << std::endl;
			std::cout.flush();
		}

		if (_counter > 0) {
			_counter -= 1;
		}
		/*
		frc::SmartDashboard::PutNumber("fl position", frontLeftModule->GetSteerPosition());
		frc::SmartDashboard::PutNumber("fr position", frontRightModule->GetSteerPosition());
		frc::SmartDashboard::PutNumber("rl position", rearLeftModule->GetSteerPosition());
		frc::SmartDashboard::PutNumber("rr position", rearRightModule->GetSteerPosition());

		frc::SmartDashboard::PutNumber("fl setpoint", frontLeftModule->GetSetpoint());
		frc::SmartDashboard::PutNumber("fr setpoint", frontRightModule->GetSetpoint());
		frc::SmartDashboard::PutNumber("rl setpoint", rearLeftModule->GetSetpoint());
		frc::SmartDashboard::PutNumber("rr setpoint", rearRightModule->GetSetpoint());
		*/

		/*
		frc::SmartDashboard::PutNumber("fl power", frontLeftModule->GetPower());
		frc::SmartDashboard::PutNumber("fr power", frontRightModule->GetPower());
		frc::SmartDashboard::PutNumber("rl power", rearLeftModule->GetPower());
		frc::SmartDashboard::PutNumber("rr power", rearRightModule->GetPower());
		*/

	//RPM Code:
	
	//frc::SmartDashboard::PutNumber("Shooter Actual Rpm", shooter->_shooter->GetEncoderPosition()); //RPM TEST
		/*
		frc::SmartDashboard::PutNumber("FLD Rpm", _driveTrainFrontLeftDrive->GetEncoderPosition()); //RPM TEST
		frc::SmartDashboard::PutNumber("FLS Rpm", _driveTrainFrontLeftSteer->GetEncoderPosition()); //RPM TEST
		frc::SmartDashboard::PutNumber("FRD Rpm", _driveTrainFrontRightDrive->GetEncoderPosition()); //RPM TEST
		frc::SmartDashboard::PutNumber("FLS Rpm", _driveTrainFrontRightSteer->GetEncoderPosition()); //RPM TEST
		*/

		/*
		frc::SmartDashboard::PutNumber("Fuse 0 amps", _pdp->GetCurrent(0));
		frc::SmartDashboard::PutNumber("Fuse 1 amps", _pdp->GetCurrent(1));
		frc::SmartDashboard::PutNumber("Fuse 2 amps", _pdp->GetCurrent(2));
		frc::SmartDashboard::PutNumber("Fuse 3 amps", _pdp->GetCurrent(3));

		frc::SmartDashboard::PutNumber("Fuse 8 amps", _pdp->GetCurrent(8));
		frc::SmartDashboard::PutNumber("Fuse 9 amps", _pdp->GetCurrent(9));
		frc::SmartDashboard::PutNumber("Fuse 10 amps", _pdp->GetCurrent(10));
		frc::SmartDashboard::PutNumber("Fuse 11 amps", _pdp->GetCurrent(11));
		*/
}

// ================================================================

void Robot::DisabledInit() {
}

// ================================================================

void Robot::DisabledPeriodic() {
	frc::Scheduler::GetInstance()->Run();
}

// ================================================================

void Robot::AutonomousInit() {
	//_compressor->SetClosedLoopControl(true);
	_compressor->EnableDigital();
	climber->Retract();
	gyroSub->ResetGyro();
	shooter->TurretZero();
	_autonomousCommand = ScriptEngine::GetInstance().GetCommand();
	if (_autonomousCommand != nullptr) {
		_autonomousCommand->Start();
	}
}

// ================================================================

void Robot::AutonomousPeriodic() {
	frc::Scheduler::GetInstance()->Run();
}

// ================================================================

void Robot::TeleopInit() {
	//_compressor->SetClosedLoopControl(true);
	_compressor->EnableDigital();
	climber->Retract();
	_shootCommand->Start();
	// The Cancel call ensures that the autonomous command stops running when
	// teleop starts running. To let the autonomous command continue until
	// finished or interrupted by another command, remove the Cancel call.
	if (_autonomousCommand != nullptr) {
		_autonomousCommand->Cancel();
	}
}

// ================================================================

void Robot::TeleopPeriodic() {
	frc::Scheduler::GetInstance()->Run();
}

// ================================================================

void Robot::TestPeriodic() {
}

// ================================================================

void Robot::DeviceInitialization() {
	//CameraServer::GetInstance()->StartAutomaticCapture();

	_compressor = new frc::Compressor(0, frc::PneumaticsModuleType::CTREPCM);

	_pdp = new frc::PowerDistribution(0, frc::PowerDistribution::ModuleType::kCTRE);

#if DIFFSWERVE

#if USING_DIFFSWERVE_TALON_FX

#elif USING_DIFFSWERVE_BASIC

	_driveTrainFrontLeftSteer = new VelocityTalonFXController(21);
	_driveTrainFrontLeftDrive = new VelocityTalonFXController(22);
	_driveTrainFrontRightSteer = new VelocityTalonFXController(27);
	_driveTrainFrontRightDrive = new VelocityTalonFXController(28);
	_driveTrainRearLeftSteer = new VelocityTalonFXController(23);
	_driveTrainRearLeftDrive = new VelocityTalonFXController(24); 
	_driveTrainRearRightSteer = new VelocityTalonFXController(25);
	_driveTrainRearRightDrive = new VelocityTalonFXController(26);

#elif USING_DIFFSWERVE_SPARK

	_driveTrainFrontLeftSteer = new VelocitySparkController(5);
	_driveTrainFrontLeftDrive = new VelocitySparkController(6);
	_driveTrainFrontRightSteer = new VelocitySparkController(3);
	_driveTrainFrontRightDrive = new VelocitySparkController(4);
	_driveTrainRearLeftSteer = new VelocitySparkController(7);
	_driveTrainRearLeftDrive = new VelocitySparkController(8);
	_driveTrainRearRightSteer = new VelocitySparkController(1);
	_driveTrainRearRightDrive = new VelocitySparkController(2);

#else
#error Unsupported configuration. Check USING_DIFFSWERVE_* #defines.
#endif

#else

#define FLD 1
#define FLS 2

#define FRD 3
#define FRS 4

#define RLD 5
#define RLS 6

#define RRD 7
#define RRS 8

	_driveTrainFrontLeftSteer = new SteerTalonController(FLS);
	_driveTrainFrontRightSteer = new SteerTalonController(FRS);
	_driveTrainRearLeftSteer = new SteerTalonController(RLS);
	_driveTrainRearRightSteer = new SteerTalonController(RRS);

#if USING_SPARKMAX_DRIVE
	_driveTrainFrontLeftDrive = new SparkMaxController(FLD);
	_driveTrainFrontRightDrive = new SparkMaxController(FRD);
	_driveTrainRearLeftDrive = new SparkMaxController(RLD);
	_driveTrainRearRightDrive = new SparkMaxController(RRD);
#elif USING_VICTOR_DRIVE
	_driveTrainFrontLeftDrive = new VictorController(FLD);
	_driveTrainFrontRightDrive = new TalonSRXController(FRD);
	_driveTrainRearLeftDrive = new VictorController(RLD);
	_driveTrainRearRightDrive = new VictorController(RRD);
#elif USING_TALON_DRIVE
	_driveTrainFrontLeftDrive = new TalonSRXController(FLD);
	_driveTrainFrontRightDrive = new TalonSRXController(FRD);
	_driveTrainRearLeftDrive = new TalonSRXController(RLD);
	_driveTrainRearRightDrive = new TalonSRXController(RRD);
#else
#error Unsupported configuration. Check USING_*_DRIVE #defines.
#endif // USING_SPARKMAX_DRIVE
#endif // DIFFSWERVE

	//======= Subsystem Motor Initialization =======//

	climber = std::make_unique<Climber>(CLIMBER_EXTENDER_FWD, CLIMBER_EXTENDER_REV, CLIMBER_BOATROLLER, ROLLERBRAKE_SOL_FWD, ROLLERBRAKE_SOL_REV);
	pickUp = std::make_unique<PickUp>(PICKUP_SOL_FWD, PICKUP_SOL_REV, PICKUP_INTAKE_CAN);
	shooter = std::make_unique<Shooter>(SHOOTER_MOTOR, TURRET_MOTOR, FEED_MOTOR, STIR_MOTOR);
	winch = std::make_unique<Winch>(WINCH_MOTOR, WINCH_BRAKE);
	controlPanel= std::make_unique<ControlPanel>(CONTROLPANEL_SOL);

	//clampMotor = new TalonSRXController(CLAMP);

	//======= Sensor and Camera Initialization =======//

	LOG("DeviceInit NavX");

#if NAVX_MXP
	navx = new AHRS(frc::I2C::Port::kMXP);
#else
	navx = new AHRS(frc::I2C::Port::kOnboard);
#endif

	//======= System Initialization =======//

	gyroSub = new GyroSub();
	visionBridge = new VisionBridgeSub();


	//======== Swerve Module Initialization =========//

#if DIFFSWERVE

#if USING_DIFFSWERVE_TALON_FX

	LOG("DeviceInit CANCoder");
	_frontLeftPot = new CANCoder(1);
	_frontRightPot = new CANCoder(4);
	_rearLeftPot = new CANCoder(2);
	_rearRightPot = new CANCoder(3);

	frontLeftModule = new TalonFXDiffSwerveModule(21, 22, Constants::FL_POS_NAME, _frontLeftPot, false);
	frontRightModule = new TalonFXDiffSwerveModule(27, 28, Constants::FR_POS_NAME, _frontRightPot, false);
	rearLeftModule = new TalonFXDiffSwerveModule(23, 24, Constants::RL_POS_NAME, _rearLeftPot, false);
	rearRightModule = new TalonFXDiffSwerveModule(25, 26, Constants::RR_POS_NAME, _rearRightPot, false);

#elif USING_DIFFSWERVE_BASIC

	LOG("DeviceInit CANCoderPositionEncoder");
	_frontLeftPot = new CANCoderPositionEncoder(1);
	_frontRightPot = new CANCoderPositionEncoder(4);
	_rearLeftPot = new CANCoderPositionEncoder(2);
	_rearRightPot = new CANCoderPositionEncoder(3);

	frontLeftModule = new BasicDiffSwerveModule(_driveTrainFrontLeftDrive, _driveTrainFrontLeftSteer, Constants::FL_POS_NAME, _frontLeftPot);
	frontRightModule = new BasicDiffSwerveModule(_driveTrainFrontRightDrive, _driveTrainFrontRightSteer, Constants::FR_POS_NAME, _frontRightPot);
	rearLeftModule = new BasicDiffSwerveModule(_driveTrainRearLeftDrive, _driveTrainRearLeftSteer, Constants::RL_POS_NAME, _rearLeftPot);
	rearRightModule = new BasicDiffSwerveModule(_driveTrainRearRightDrive, _driveTrainRearRightSteer, Constants::RR_POS_NAME, _rearRightPot);

#elif USING_DIFFSWERVE_SPARK

	LOG("DeviceInit Analog");
	_frontLeftPot = new frc::AnalogInput(2);
	_frontRightPot = new frc::AnalogInput(1);
	_rearLeftPot = new frc::AnalogInput(3);
	_rearRightPot = new frc::AnalogInput(0);

	frontLeftModule = new DiffSwerveModule(_driveTrainFrontLeftDrive, _driveTrainFrontLeftSteer, Constants::FL_POS_NAME, _frontLeftPot);
	frontRightModule = new DiffSwerveModule(_driveTrainFrontRightDrive, _driveTrainFrontRightSteer, Constants::FR_POS_NAME, _frontRightPot);
	rearLeftModule = new DiffSwerveModule(_driveTrainRearLeftDrive, _driveTrainRearLeftSteer, Constants::RL_POS_NAME, _rearLeftPot);
	rearRightModule = new DiffSwerveModule(_driveTrainRearRightDrive, _driveTrainRearRightSteer, Constants::RR_POS_NAME, _rearRightPot);

#else
#error Unsupported configuration. Check USING_DIFFSWERVE_* #defines.
#endif

#else
	frontLeftModule = new SwerveModule(_driveTrainFrontLeftDrive, _driveTrainFrontLeftSteer, Constants::FL_POS_NAME);
	frontRightModule = new SwerveModule(_driveTrainFrontRightDrive, _driveTrainFrontRightSteer, Constants::FR_POS_NAME);
	rearLeftModule = new SwerveModule(_driveTrainRearLeftDrive, _driveTrainRearLeftSteer, Constants::RL_POS_NAME);
	rearRightModule = new SwerveModule(_driveTrainRearRightDrive, _driveTrainRearRightSteer, Constants::RR_POS_NAME);
#endif

	LOG("DeviceInit DriveTrain");
	driveTrain = new DriveTrain();

	oi = new OI();

	_shootCommand = new Shoot();

	//untested change
	frc::LiveWindow::DisableAllTelemetry();

	LOG("DeviceInit end");
}

// ================================================================

#ifndef RUNNING_FRC_TESTS
int main() {
	return frc::StartRobot<Robot>();
}
#endif

// ================================================================
