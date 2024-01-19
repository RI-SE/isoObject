#pragma once

#include <string>
#include <iostream>
#include <thread> 
#include <mutex>
#include <atomic>

#include "header.h"
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


#define DEFAULT_CONTROL_CENTER_ID 0 // 0 is reserved for the control center
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
	TestObject(const std::string& listenIP = "0.0.0.0");
	TestObject(int tcpSocketNativeHandle);
	virtual ~TestObject();

	void disconnect();
	void shutdown();

	void setPosition(const CartesianPosition& pos) { position = pos; }
	void setSpeed(const SpeedType& spd) { speed = spd; }
	void setObjectSettings(const ObjectSettingsType& osem) { objectSettings = osem; }
	void setAcceleration(const AccelerationType& acc) { acceleration = acc; }
	void setDriveDirection(const DriveDirectionType& drd) { driveDirection = drd; }
	void setObjectState(const ObjectStateID& ost) { objectState = ost; }
	void setName(const std::string nm) { name = nm; }
	void setReadyToArm(const int& rdy) { readyToArm = rdy; }
	void setErrorState(const char err) { errorState = err; }

	std::string getCurrentStateName() const { return state->getName(); }
	int getCurrentStateID() const { return state->getStateID(); }
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
	uint32_t getReceiverID() const { return receiverID; }
	ObjectSettingsType getObjectSettings() const { return objectSettings; }


	/** SWIG Wrappers **/
	//! Wrapper for handling tcp messages when using an iso connector for simulation
	int handleTCPMessage(char *buffer, int bufferLen);
	//! Wrapper for handling udp message when using an iso connector for simulation
	int handleUDPMessage(char *buffer, int bufferLen, int udpSocket, char *addr, const uint32_t port);


	//! Used to start the threads
	void startHandleTCP() { tcpReceiveThread = std::thread(&TestObject::receiveTCP, this); }
	void startHandleUDP() { udpReceiveThread = std::thread(&TestObject::receiveUDP, this); }
	void startHEABCheck() { heabTimeoutThread = std::thread(&TestObject::checkHeabLoop, this); }
	void startSendMonr()  { monrThread = std::thread(&TestObject::sendMonrLoop, this); }

protected:
	//! Wrapper for handling function that converts to char vector
	int handleMessage(char *buffer, int bufferLen);

	//! Fill message header with receiver/transmitter id and messageCounter. Returns pointer to input header.
	MessageHeaderType *populateMessageHeader(MessageHeaderType *header);

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
	virtual void onOSEM(ObjectSettingsType& osem){};
	virtual void onHEAB(HeabMessageDataType& heab) {};
	virtual void onTRAJ() {};
	virtual void onOSTM(ObjectCommandType& ostm) {};
	virtual void onSTRT(StartMessageType& strt) {};

	std::chrono::milliseconds expectedHeartbeatPeriod = std::chrono::milliseconds(1000 / HEAB_FREQUENCY_HZ);
	std::chrono::milliseconds monrPeriod = std::chrono::milliseconds(1000 / MONR_EXPECTED_FREQUENCY_HZ);
	std::chrono::milliseconds heartbeatTimeout = 10*expectedHeartbeatPeriod;
	std::chrono::milliseconds maxSafeNetworkDelay = std::chrono::milliseconds(200);

	//! Used to get estimated network delay 
	std::chrono::milliseconds getNetworkDelay();

	ISO22133::State* state;
private:
	//! Initializer for commonalities of the constructs
	void initialize();

	//! TCP receiver loop that should be run in its own thread.
	void receiveTCP();
	//! UDP receiver loop that should be run in its own thread.
	void receiveUDP();
	//! HEAB timeout checking loop that should be run in its own thread.
	void checkHeabLoop();
	//! MONR sending loop that should be run in its own thread.
	void sendMonrLoop();
	
	//! Function for handling received ISO messages. Calls corresponding
	//! handler in the current state.
	int handleMessage(std::vector<char>&);
	void handleHEAB(HeabMessageDataType& heab);
	//! Sends MONR message on process channel
	void sendMONR(bool debug = false);
	//! Sends GREM message on control channel
	void sendGREM(HeaderType header, GeneralResponseStatus responseCode, bool debug = false);
	//! Called if HEAB messages do not arrive on time
	void onHeabTimeout();
	//! Function that checks if HEABs arrive on time
	void checkHeabTimeout();

	//! Set estimated network delay from HEAB times
	void setNetworkDelay(std::chrono::milliseconds);

	//! Get the Next message counter to send
	char getNextSentMessageCounter() { return sentMessageCounter = (sentMessageCounter + 1) % 256; }

	//! Check if the received message counter is correct and update regardless to the next expected
	void checkAndUpdateMessageCounter(const char receivedMessageCounter) {
		if (receivedMessageCounter != expectedMessageCounter) {
			std::cout << "Message counter mismatch. Expected: " << (int)expectedMessageCounter << " Received: " << (int)receivedMessageCounter << std::endl;
		}
		expectedMessageCounter = (expectedMessageCounter + 1) % 256;
	}

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
	std::string name = "unnamed";
	TcpServer ctrlChannel;
	UdpServer processChannel;
	TrajDecoder trajDecoder;
	std::chrono::steady_clock::time_point lastHeabTime;
	std::atomic<HeaderType> lastReceivedMsgHeader;
	std::atomic<GeographicPositionType> origin;
	std::atomic<ControlCenterStatusType> ccStatus;
	std::atomic<CartesianPosition> position;
	std::atomic<SpeedType> speed;
	std::atomic<ObjectSettingsType> objectSettings;
	std::atomic<AccelerationType> acceleration;
	std::atomic<DriveDirectionType> driveDirection { OBJECT_DRIVE_DIRECTION_UNAVAILABLE };
	std::atomic<ObjectStateID> objectState  { ISO_OBJECT_STATE_UNKNOWN };
	std::atomic<int> readyToArm { OBJECT_READY_TO_ARM_UNAVAILABLE };
	std::atomic<int> transmitterID;
	std::atomic<int> receiverID;
	std::atomic<char> expectedMessageCounter;
	std::atomic<char> sentMessageCounter;
	std::atomic<bool> socketsReceivedFromController { false };
	std::atomic<char> errorState { 0 };
	std::atomic<bool> awaitingFirstHeab { true };
	std::atomic<bool> osemReceived { false };
	std::atomic<bool> on { true };

	std::chrono::milliseconds estimatedNetworkDelay = std::chrono::milliseconds(0);
};
} // namespace ISO22133

namespace std::chrono {
template <typename Duration>
struct timeval to_timeval(Duration&& d) {
	std::chrono::seconds const sec = std::chrono::duration_cast<std::chrono::seconds>(d);
	struct timeval tv;
	tv.tv_sec = sec.count();
	tv.tv_usec = std::chrono::duration_cast<std::chrono::microseconds>(d - sec).count();
	return tv;
}
}  // namespace std::chrono

inline bool operator< (const timeval &lhs, const timeval &rhs) {
	return (lhs.tv_sec + lhs.tv_usec/1e6) < (rhs.tv_sec + rhs.tv_usec/1e6);
}
inline bool operator> (const timeval &lhs, const timeval &rhs) {
	return (lhs.tv_sec + lhs.tv_usec/1e6) > (rhs.tv_sec + rhs.tv_usec/1e6);
}

