// ==========================================================================
// Constants class
//
// FRC 4143 "MARS WARS"
// ==========================================================================
// 2015-02-01 JKSalmon - Initial development
// ==========================================================================
#pragma once

// ==========================================================================

class Constants {
public:
	Constants() {}

	static const char* FL_POS_NAME;
	static const char* FR_POS_NAME;
	static const char* RL_POS_NAME;
	static const char* RR_POS_NAME;
	static const char* ARM_POSITION_NAME;

	static const double FL_POS_DEFAULT;
	static const double FR_POS_DEFAULT;
	static const double RL_POS_DEFAULT;
	static const double RR_POS_DEFAULT;
	static const double ARM_POSITION_DEFAULT;

	static const float DEAD_ZONE;

	/**
	 * Which PID slot to pull gains from.  Starting 2018, you can choose
	 * from 0,1,2 or 3.  Only the first two (0,1) are visible in web-based configuration.
	 */
	static const int kSlotIdx = 0;

	/* Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops.
	 * For now we just want the primary one.
	 */
	static const int kPIDLoopIdx = 0;

	/*
	 * set to zero to skip waiting for confirmation, set to nonzero to wait
	 * and report to DS if action fails.
	 */};

// ============================================================================