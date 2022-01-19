#pragma once
#include "controllers/IMultiController.h"

class IVelocityMultiController : public IMultiController {
public:

	virtual void SetVelocity(double value) = 0;
	virtual void ConfigPID() = 0;
};
