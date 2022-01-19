#pragma once
#include <frc/commands/Subsystem.h>
#include <mutex>
#include <thread>

// ==========================================================================

class VisionBridgeSub : public frc::Subsystem {
public:
	// Initialize the subsystem with the default UDP port.
	VisionBridgeSub();

	// Initialize the subsystem, specifying the UDP port.
	VisionBridgeSub(uint16_t listeningPort);

	void InitDefaultCommand();

	// Enable/disable debug mode. When debug is enabled,
	// received packets are written to standard output.
	void EnableDebug(bool debug);

	// Get the latest vision data.
	double GetPositionX();
	double GetPositionY();
	double GetYaw();

private:

	std::recursive_mutex _mutex;
	uint16_t _listeningPort;

	// Vision Values
	double _pitch;
	double _roll;
	double _yaw;
	double _x;
	double _y;
	double _z;

	bool _debug;
	std::thread _listeningThread;

	void DebugOutput(std::string packet);
	void Listen();
	void ParsePacket(char packet[]);
	void SetValues(double pitch, double roll, double yaw, double x, double y, double z);
};

// ==========================================================================
