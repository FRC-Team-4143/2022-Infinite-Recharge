// ==========================================================================
// Logger class
//
// FRC 4143: MARS/WARS
// ==========================================================================
// 2015-02-01 JKSalmon - Initial development
// 2017-01-28 JKSalmon - Minor code and performance improvements
// ==========================================================================
#include "Modules/Logger.h"
#include <time.h>
#include <iostream>

// ==========================================================================

std::mutex Logger::m_mutex;
int Logger::m_counter = 0;

// ==========================================================================

Logger::Logger() {
}

// ==========================================================================

void Logger::Log(std::string msg) {
	// DriverStation::ReportError(msg.append("\r\n"));
	std::lock_guard<std::mutex> lck(m_mutex);

	time_t rawtime = time(nullptr);
	tm localTime = *localtime(&rawtime);

	char szTimestamp[64];
	strftime(szTimestamp, 64, "%Y-%m-%d %H:%M:%S", &localTime);

	int counter = ++m_counter;
	char szPrefix[64];
	sprintf(szPrefix, "%s %i ", szTimestamp, counter);

	msg = szPrefix + msg;

	std::cout << msg << std::endl;
	std::cout.flush();
}

// ==========================================================================
