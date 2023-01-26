#pragma once

#include <string>
#include <iostream>
#include <thread> 
#include <mutex>
#include <atomic>
#include <optional>

#include <bitset>

#include "iso22133.h"
#include "iso22133state.hpp"
#include "trajDecoder.hpp"
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

	void setPosition(const CartesianPosition& pos) { position = pos; }
	void setSpeed(const SpeedType& spd) { speed = spd; }
	void setAcceleration(const AccelerationType& acc) { acceleration = acc; }
	void setDriveDirection(const DriveDirectionType& drd) { driveDirection = drd; }
	void setObjectState(const ObjectStateID& ost) { objectState = ost; }
	void setName(const std::string nm) { name = nm; }
	void setReadyToArm(const int& rdy) { readyToArm = rdy; }
	void setErrorState(const char err) { errorState = err; }

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

	//! Used to get estimated network delay 
	std::chrono::milliseconds getNetworkDelay();

private:

	//! TCP receiver loop that should be run in its own thread.
	void receiveTCP();
	//! UDP receiver loop that should be run in its own thread.
	void receiveUDP();
	//! HEAB timeout checking loop that should be run in its own thread.
	void checkHeabLoop();
	//! MONR sending loop that should be run in its own thread.
	void sendMonrLoop();
	
	//! Used to start the threads
	void startHandleTCP() { tcpReceiveThread = std::thread(&TestObject::receiveTCP, this); }
	void startHandleUDP() { udpReceiveThread = std::thread(&TestObject::receiveUDP, this); }
	void startHEABCheck() { heabTimeoutThread = std::thread(&TestObject::checkHeabLoop, this); }
	void startSendMonr()  { monrThread = std::thread(&TestObject::sendMonrLoop, this); }
	
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

	//! Set estimated network delay from HEAB times
	void setNetworkDelay(std::chrono::milliseconds);

	sigslot::signal<>heabTimeout;
	std::mutex recvMutex;
	std::mutex heabMutex;
	std::mutex netwrkDelayMutex;
	std::string localIP;
	std::thread tcpReceiveThread;
	std::thread udpReceiveThread;
	std::thread monrThread;
	std::thread heabTimeoutThread;
	std::thread delayedStrtThread;
	ISO22133::State* state;
	std::string name = "unnamed";
	TcpServer ctrlChannel;
	UdpServer processChannel;
	TrajDecoder trajDecoder;
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

	std::chrono::milliseconds estimatedNetworkDelay = std::chrono::milliseconds(0);
};
} // namespace ISO22133

// TODO: get this from maestroTime.h in the future
namespace std::chrono {
using quartermilliseconds = std::chrono::duration<int64_t, std::ratio<1, 4000>>;
using weeks = std::chrono::duration<uint16_t, std::ratio<7 * 24 * 60 * 60, 1>>;

template <typename Duration>
struct timeval to_timeval(Duration&& d) {
	std::chrono::seconds const sec = std::chrono::duration_cast<std::chrono::seconds>(d);
	struct timeval tv;
	tv.tv_sec = sec.count();
	tv.tv_usec = std::chrono::duration_cast<std::chrono::microseconds>(d - sec).count();
	return tv;
}

// template <typename Duration>
// void from_timeval(timeval&& tv, Duration& d) {
// 	const auto sec = std::chrono::seconds(tv.tv_sec);
// 	const auto usec = std::chrono::microseconds(tv.tv_usec);
// 	d = sec + usec;
// }
}  // namespace std::chrono

inline bool operator< (const timeval &lhs, const timeval &rhs) {
	return (lhs.tv_sec + lhs.tv_usec/1e6) < (rhs.tv_sec + rhs.tv_usec/1e6);
}
inline bool operator> (const timeval &lhs, const timeval &rhs) {
	return (lhs.tv_sec + lhs.tv_usec/1e6) > (rhs.tv_sec + rhs.tv_usec/1e6);
}

