// ==========================================================================
// IShooter interface
// ==========================================================================
#pragma once
#include <frc/commands/Subsystem.h>
#include "controllers/IPositionMultiController.h"
#include "controllers/IVelocityMultiController.h"

class IShooter : public frc::Subsystem {
public:

	IShooter(std::string_view name);
	virtual void TurretMove(float velocity) = 0; 
	virtual void TurretLeft() = 0;
	virtual void TurretRight() = 0;
	virtual void TurretStop() = 0;
	virtual void Feed(float feedspeed) = 0;
	virtual void FeedStop() = 0;
	virtual void Stir() = 0;
	virtual void StirReverse() = 0;
	virtual void StirStop() = 0;
	virtual void ShootStart(float speedPercent) = 0;
	virtual void ShootStop() = 0;
	virtual void TurretZero() = 0;
	virtual void LimeLightControl(bool controlmode) = 0;

	virtual void SetDegrees(float degrees) = 0;
	virtual float GetDegrees() = 0;

	std::unique_ptr<IPositionMultiController> _turret = 0;
	std::unique_ptr<IVelocityMultiController> _shooter=0;
};

// ==========================================================================
