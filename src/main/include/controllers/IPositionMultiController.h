#pragma once
#include "controllers/IMultiController.h"

class IPositionMultiController : public IMultiController {
public:

	virtual void SetPosition(double value) = 0;
	virtual void ZeroPosition() = 0;
	virtual void ConfigPID() = 0;
};
