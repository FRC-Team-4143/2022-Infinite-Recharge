#include "subsystems/VisionBridgeSub.h"
#include "Modules/CastUtil.h"
#include "Modules/Logger.h"
#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>

#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <condition_variable>
#include <iostream>

const uint16_t DEFAULT_LISTENING_PORT = 4143; // TODO - Change to allowed port range

#define MAX_PACKET_SIZE 2048

// ==========================================================================

VisionBridgeSub::VisionBridgeSub()
:	VisionBridgeSub(DEFAULT_LISTENING_PORT) {
}

// ==========================================================================

VisionBridgeSub::VisionBridgeSub(uint16_t listeningPort)
:	frc::Subsystem("VisionBridgeSub"),
	_mutex(),
	_listeningPort(listeningPort),
	_pitch(0),
	_roll(0),
	_yaw(0),
	_x(0),
	_y(0),
	_z(0),
	_debug(false),
	_listeningThread(&VisionBridgeSub::Listen, this) {
}

// ==========================================================================

void VisionBridgeSub::InitDefaultCommand() {
	//SetDefaultCommand(new MySpecialCommand());
}

// ==========================================================================

void VisionBridgeSub::EnableDebug(bool debug) {
	_debug = debug;
}

// ==========================================================================
// Side 0 is left
// Side 1 is right

double VisionBridgeSub::GetPositionX() {
	std::unique_lock<std::recursive_mutex> lock(_mutex);

	return _x;
}

// ==========================================================================

double VisionBridgeSub::GetPositionY() {
	std::unique_lock<std::recursive_mutex> lock(_mutex);

	return _y;
}

// ==========================================================================

double VisionBridgeSub::GetYaw() {
	std::unique_lock<std::recursive_mutex> lock(_mutex);

	return _yaw;
}


// ==========================================================================

void VisionBridgeSub::DebugOutput(std::string packet) {
	LOG("VisionPacket=" + packet);
}

// ==========================================================================

void VisionBridgeSub::Listen() {
	sockaddr_in serverAddr; // server's socket address
	sockaddr_in clientAddr; // client's socket address
	socklen_t sockAddrSize; // size of socket address structure
	int sockListener;       // socket file descriptor
	char buf[MAX_PACKET_SIZE + 1];

	// Set up the local address.
	sockAddrSize = sizeof(sockaddr_in);
	bzero(&serverAddr, sockAddrSize);
	serverAddr.sin_family = AF_INET;
	serverAddr.sin_port = htons(_listeningPort);
	serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);

	std::cout << GetName() << " Starting UDP listener on port " << _listeningPort << std::endl;

	// Create a UDP socket.
	sockListener = socket(AF_INET, SOCK_DGRAM, 0);
	//if ((sockListener = socket(AF_INET, SOCK_DGRAM, 0)) == ERROR) {
	//    perror("socket");
	//    return;
	//}
	std::cout << GetName() << " Created socket" << std::endl;

	// Bind socket to local address.
	bind(sockListener, S_CAST(sockaddr *, S_CAST(void *, &serverAddr)), sockAddrSize);
	//if (bind(sockListener, S_CAST(sockaddr*, S_CAST(void*, &serverAddr)), sockAddrSize) == ERROR) {
	//    perror("bind");
	//    close(sockListener);
	//    return;
	//}
	std::cout << GetName() << " Bound socket" << std::endl;

	for (;;) {
		//if (_debug) std::cout << "[DEBUG] " << GetName() << " Waiting on port" << std::endl;
		int recvlen = recvfrom(sockListener, buf, sizeof(buf) - 1, 0,
				S_CAST(sockaddr *, S_CAST(void *, &clientAddr)),
				&sockAddrSize);
		//if (_debug) std::cout << "[DEBUG] " << GetName() << " Received " << recvlen << " bytes" << std::endl;
		if (recvlen > 0) {
			buf[recvlen] = 0;
			ParsePacket(buf);
		}
	}

	close(sockListener);
}

// ==========================================================================

void VisionBridgeSub::ParsePacket(char packet[]) {
	if (_debug) {
		DebugOutput(packet);
	}
	try {
		char* pch = std::strtok(packet, " ");
		auto pitch = std::stod(pch);
		pch = std::strtok(nullptr, " ");
		auto roll = std::stod(pch);
		pch = std::strtok(nullptr, " ");
		auto yaw = std::stod(pch);
		pch = std::strtok(nullptr, " ");
		auto x = std::stod(pch);
		pch = std::strtok(nullptr, " ");
		auto y = std::stod(pch);
		pch = std::strtok(nullptr, " ");
		auto z = std::stod(pch);

		SetValues(pitch, roll, yaw, x, y, z);
	}
	catch (...) {
	}
}

// ==========================================================================

void VisionBridgeSub::SetValues(double pitch, double roll, double yaw, double x, double y, double z) {
	std::unique_lock<std::recursive_mutex> lock(_mutex);
	_pitch = pitch;
	_roll = roll;
	_yaw = yaw;
	_x = x;
	_y = y;
	_z = z;

	frc::SmartDashboard::PutNumber("realsense yaw", yaw);
}

// ==========================================================================
