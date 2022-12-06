#pragma once

#include <string>
#include <iostream>
#include <thread> 
#include <mutex>
#include <atomic>
#include <optional>

#include "iso22133.h"
#include "iso22133state.hpp"
#include "trajDecoder.hpp"
#include "server.hpp"
#include "socket.hpp"
#include "signal.hpp"
#include "tcpServer.hpp"
#include "udpServer.hpp"

namespace ISO22133 {
class State;
class Unknown;
class Off;
class Init;
class Armed;
class Disarmed;
class Running;
class PostRun;
class RemoteControlled;
class Aborting;
class PreArming;
class PreRunning;

/**
 * @brief The TestObject class is an abstract form of  a
 *          ISO22133 object. It needs to be ineherited and
 *          the inheriting class must implement the pure
 *          virtual functions.
 */
class TestObject {
	friend class State;
	friend class Unknown;
	friend class Off;
	friend class Init;
	friend class Armed;
	friend class Disarmed;
	friend class Running;
	friend class PostRun;
	friend class RemoteControlled;
	friend class Aborting;
	friend class PreArming;
	friend class PreRunning;

public:
	TestObject(const std::string& listenIP = "");

	virtual ~TestObject();

	void disconnect();

	std::string getCurrentStateName() const { return state->getName(); }
	std::string getName() const { return name; }
	CartesianPosition getPosition() const { return position; }
	SpeedType getSpeed() const { return speed; }
	AccelerationType getAcceleration() const { return acceleration; }
	DriveDirectionType getDriveDirection() const { return driveDirection; }
	TrajectoryHeaderType getTrajectoryHeader() const { return trajDecoder.getTrajHeader(); }
	std::vector<TrajectoryWaypointType> getTrajectory() const { return trajDecoder.getTraj(); }
	GeographicPositionType getOrigin() const { return origin; }
	std::string getLocalIP() const { return localIP; }
	uint32_t getTransmitterID() const { return transmitterID; }

protected:

	//! Pure virtual safety function that must be implemented by the user.
	virtual void handleAbort() { throw std::logic_error("Use of unimplemented abort handler"); }

	void setPosition(const CartesianPosition& pos) { position = pos; }
	void setSpeed(const SpeedType& spd) { speed = spd; }
	void setAcceleration(const AccelerationType& acc) { acceleration = acc; }
	void setDriveDirection(const DriveDirectionType& drd) { driveDirection = drd; }
	void setObjectState(const ObjectStateID& ost) { objectState = ost; }
	void setName(const std::string nm) { name = nm; }
	void setReadyToArm(const int& rdy) { readyToArm = rdy; }
	void setErrorState(const char& err) { errorState = err; }

	//! Virtual function for adding handling of vendor specific messages if needed.
	//! Expected to return the number of handled bytes or <= 0 if message was not recognized
	//! or decoding failed.
	virtual int handleVendorSpecificMessage(const int msgType, const std::vector<char>& data) { return -1; }

	// These should be overridden if extending one of the states
	// Example of override:
	// return dynamic_cast<ISO22133::PreArming*>(new myPreArming);
	//! Must be overridden if modifying the Unknown state
	virtual Unknown* createUnknown() const { return new Unknown; }
	//! Must be overridden if modifying the Off state
	virtual Off* createOff() const { return new Off; }
	//! Must be overridden if modifying the Init state
	virtual Init* createInit() const { return new Init; }
	//! Must be overridden if modifying the Armed state
	virtual Armed* createArmed() const { return new Armed; }
	//! Must be overridden if modifying the Disarmed state
	virtual Disarmed* createDisarmed() const { return new Disarmed; }
	//! Must be overridden if modifying the Running state
	virtual Running* createRunning() const { return new Running; }
	//! Must be overridden if modifying the Postrun state
	virtual PostRun* createPostRun() const { return new PostRun; }
	//! Must be overridden if modifying the Remote Controlled state
	virtual RemoteControlled* createRemoteControlled() const { return new RemoteControlled; }
	//! Must be overridden if modifying the Abort state
	virtual Aborting* createAborting() const { return new Aborting; }
	//! Must be overridden if modifying the Pre-Arm state
	virtual PreArming* createPreArming() const { return new PreArming; }
	//! Must be overridden if modifying the Pre-Running state
	virtual PreRunning* createPreRunning() const { return new PreRunning; }


	//! Signals for events
	sigslot::signal<> stateChangeSig;
	sigslot::signal<ObjectSettingsType&> osemSig;
	sigslot::signal<HeabMessageDataType&> heabSig;
	sigslot::signal<> trajSig;
	sigslot::signal<ObjectCommandType&> ostmSig;
	sigslot::signal<StartMessageType&> strtSig;

	//! These funcitons are called asynchronously as
	//! events occur. The user is responsible for
	//! overriding and implementing them if needed.
	//! Preferable using threads as to not slow down
	//! the main thread.
	virtual void onStateChange() {};
	virtual void onOSEM(ObjectSettingsType&) {};
	virtual void onHEAB(HeabMessageDataType&) {};
	virtual void onTRAJ() {};
	virtual void onOSTM(ObjectCommandType&) {};
	virtual void onSTRT(StartMessageType&) {};

	std::chrono::milliseconds expectedHeartbeatPeriod = std::chrono::milliseconds(1000 / HEAB_FREQUENCY_HZ);
	std::chrono::milliseconds monrPeriod = std::chrono::milliseconds(1000 / MONR_EXPECTED_FREQUENCY_HZ);
	std::chrono::milliseconds heartbeatTimeout = 10*expectedHeartbeatPeriod;
	std::chrono::milliseconds maxSafeNetworkDelay = std::chrono::milliseconds(200);

private:

	//! TCP receiver loop that should be run in its own thread.
	void receiveTCP();
	//! HEAB receiving / MONR sending loop that should be run in its own thread.
	void heabMonrLoop();
	//! HEAB timeout checking loop that should be run in its own thread.
	void checkHeabLoop();
	void startHandleTCP() { tcpReceiveThread = std::thread(&TestObject::receiveTCP, this); }
	void startHandleUDP() { heabMonrThread = std::thread(&TestObject::heabMonrLoop, this); }
	void startHEABCheck() { heabTimeoutThread = std::thread(&TestObject::checkHeabLoop, this); }
	//! Function for handling received ISO messages. Calls corresponding
	//! handler in the current state.
	int handleMessage(std::vector<char>&);
	void handleHEAB(HeabMessageDataType& heab);
	//! Sends MONR message on process channel
	void sendMONR(bool debug = false);
	//! Called if HEAB messages do not arrive on time
	void onHeabTimeout();
	//! Function that checks if HEABs arrive on time
	void checkHeabTimeout();

	sigslot::signal<>heabTimeout;
	std::mutex recvMutex;
	std::thread tcpReceiveThread;
	std::string localIP;
	std::thread heabMonrThread;
	std::thread heabTimeoutThread;
	ISO22133::State* state;
	std::string name = "unnamed";
	TcpServer ctrlChannel;
	UdpServer processChannel;
	TrajDecoder trajDecoder;
	std::mutex heabMutex;
	std::chrono::steady_clock::time_point lastHeabTime;
	std::atomic<GeographicPositionType> origin;
	std::atomic<ControlCenterStatusType> ccStatus;
	std::atomic<CartesianPosition> position;
	std::atomic<SpeedType> speed;
	std::atomic<AccelerationType> acceleration;
	std::atomic<DriveDirectionType> driveDirection { OBJECT_DRIVE_DIRECTION_UNAVAILABLE };
	std::atomic<ObjectStateID> objectState  { ISO_OBJECT_STATE_UNKNOWN };
	std::atomic<int> readyToArm { OBJECT_READY_TO_ARM_UNAVAILABLE };
	std::atomic<int> transmitterID;
	std::atomic<char> errorState { 0 };

	bool awaitingFirstHeab = true;
	bool on = true;
};
} // namespace ISO22133
