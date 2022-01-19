#include "subsystems/Shooter.h"
#include <ctre/Phoenix.h>
#include "controllers/VictorController.h"
#include "controllers/TurretSparkController.h"
#include "controllers/ShooterTalonFXController.h"
#include "commands/Shoot.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTableInstance.h>
#include "OI.h"
#include "Robot.h"

// ==========================================================================

Shooter::Shooter(int shooterCANId, int turretCANId, int feederCANId, int stirCANId)
:	IShooter("Shooter") {
	_shooter = std::make_unique<ShooterTalonFXController>(shooterCANId);
	_turret = std::make_unique<TurretSparkController>(turretCANId);
	_feeder = std::make_unique<VictorController>(feederCANId);
	_stir = std::make_unique<VictorController>(stirCANId);
	_limelightTable= nt::NetworkTableInstance::GetDefault().GetTable("limelight");
	_limelightTable->PutNumber("ledMode", 1);
}

// ==========================================================================

void Shooter::InitDefaultCommand() {
	// No default command needed for this subsystem.
	//SetDefaultCommand(new Shoot());
}

// ==========================================================================

void Shooter::TurretMove(float degrees){
	_targetDegrees = degrees;
	_turret->SetPosition(degrees); // big gear 220, driving gear 22, VP 1/10 
                                                              // need to verify
}

// ==========================================================================

void Shooter::TurretLeft() {
	_turret->SetPercentPower(0.10);
}

// ==========================================================================

void Shooter::TurretStop() {
	_turret->SetPercentPower(0);
}

// ==========================================================================

void Shooter::TurretRight() {
	_turret->SetPercentPower(-0.10);
}

// ==========================================================================

void Shooter::Feed( float feedspeed) {
	//frc::SmartDashboard::PutNumber("shooter speed", _shooter->GetEncoderPosition());
	//if(_shooter->GetEncoderPosition() > 3000) {
		_feeder->SetPercentPower(feedspeed);
	//}
}

// ==========================================================================

void Shooter::FeedStop() {
	_feeder->SetPercentPower(0.00);
}

// ==========================================================================

void Shooter::Stir() {
	_stir->SetPercentPower(1.00);
}

// ==========================================================================

void Shooter::StirReverse() {
	_stir->SetPercentPower(-1.00);
}

// ==========================================================================

void Shooter::StirStop() {
	_stir->SetPercentPower(0);
}

// ==========================================================================

void Shooter::ShootStart(float speedPercent) {
	//_shooter->SetPercentPower(.65);
	_limelightTable->PutNumber("ledMode", 0);
	_shooter->SetVelocity(speedPercent);
	}

// ==========================================================================

void Shooter::ShootStop() {
	_shooter->SetPercentPower(0);
	_limelightTable->PutNumber("ledMode", 1);
}

// ==========================================================================

void Shooter::TurretZero() {
	_turret->ZeroPosition();
}

// ==========================================================================

void Shooter::SetDegrees(float degrees) {
	_targetDegrees = degrees;
}

float Shooter::GetDegrees() {
	//untested change
	//return _targetDegrees;
	return _turret->GetPosition();
}

void Shooter::LimeLightControl(bool controlmode) {
	
	auto joyz = Robot::oi->GetJoystick2Z();
	std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
	float tx = table->GetNumber("tx", 0.0f);
	frc::SmartDashboard::PutNumber("TX",tx); //MAX: 15 MIN: -15
	_gyroCounter++;
	float newtarget = 0;
	
	//VISION AIM DISABLED

	// if ((Robot::oi->GetLeftTrigger2() < .5 && _gyroCounter >= GYRO_DELAY_TICKS) || controlmode) {
		
	// 		if (Robot::oi->GetRightTrigger2() > 0.75 ){
	// 		  if (_targetDegrees > 0.001 || _targetDegrees < -0.001)
	// 			_targetDegrees += tx/4.0;
	// 		} else {
	// 			_targetDegrees = -Robot::gyroSub->PIDGet() + 180;
	// 			// Gyro returns -180 to +180. Convert to 0 to 360.
	// 			if (_targetDegrees < 0) _targetDegrees += 360;
	// 			}
	// } 

	if(controlmode) {
		joyz=0;
		newtarget = _targetDegrees + tx*0.1;

		// untested changes
		if(_targetDegrees >= 10. && _targetDegrees <= 45. && newtarget < 10.) _targetDegrees = 10.;
		else if(_targetDegrees <= 45. && newtarget > 45.) _targetDegrees = 45.;
		else if(_targetDegrees >= 112. && newtarget < 112.) _targetDegrees = 112.;
		else _targetDegrees = newtarget;
	}

	if (joyz != 0) {
		_targetDegrees += joyz;
		_gyroCounter = 0;
	}

	if (_targetDegrees < MIN_ALLOWED_ANGLE) {
		_targetDegrees = MIN_ALLOWED_ANGLE;
	}
	else if (_targetDegrees > MAX_ALLOWED_ANGLE) {
		_targetDegrees = MAX_ALLOWED_ANGLE;
	}

	Robot::shooter->TurretMove(_targetDegrees);
	frc::SmartDashboard::PutNumber("Target Degrees", _targetDegrees);  
}

// ==========================================================================