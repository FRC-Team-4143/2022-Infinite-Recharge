#include "subsystems/EncoderConstants.h"
#include "Robot.h"

// ==========================================================================

#ifdef DIFFSWERVE
const double EncoderConstants::COUNTS_PER_TURN = 5.0;
#else
const double EncoderConstants::COUNTS_PER_TURN = 4096;
#endif

const double EncoderConstants::QUARTER_TURN = 1.25;
const double EncoderConstants::HALF_TURN = 2.5;
const double EncoderConstants::THREEQUARTER_TURN = 3.75;
const double EncoderConstants::FULL_TURN = 5.0;

// ==========================================================================
