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
#include "tcphandler.hpp"
#include "udphandler.hpp"
#include "signal.hpp"

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
    
    bool isServerConnected() const { return controlChannel.isConnected(); }
    bool isUdpOk() const { return udpOk; }
    std::string getCurrentStateName() const { return state->getName(); }
    std::string getName() const { return name; }
    CartesianPosition getPosition() const { return position; }
    SpeedType getSpeed() const { return speed; }
    AccelerationType getAcceleration() const { return acceleration; }
    DriveDirectionType getDriveDirection() const { return driveDirection; }
    TrajectoryHeaderType getTrajectoryHeader() { return trajDecoder.getTrajHeader(); }
    std::vector<TrajectoryWaypointType> getTrajectory() { return trajDecoder.getTraj(); }

protected:
    
    //! Pure virtual safety function that must be implemented by the user.
    virtual void handleAbort() = 0;

    void setPosition(CartesianPosition& pos) { position = pos; }
    void setSpeed(SpeedType& spd) { speed = spd; }
    void setAcceleration(AccelerationType& acc) { acceleration = acc; }
    void setDriveDirection(DriveDirectionType& drd) { driveDirection = drd; }
    void setObjectState(ObjectStateID& ost) { objectState = ost; }
    void setName(std::string name) { name = name; }
    void setReadyToArm(const int& rdy) { readyToArm = rdy; }
    void setErrorState(const char& err) { errorState = err; }      

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
	std::chrono::milliseconds heartbeatTimeout = 5*expectedHeartbeatPeriod;
private:
    
    //! UDP receiver loop that should be run in its own thread.
    void receiveUDP();
    //! TCP receiver loop that should be run in its own thread.
    void receiveTCP();
    //! MONR sending loop that should be run in its own thread.
    void monrLoop();
    void startHandleTCP() { tcpReceiveThread = std::thread(&TestObject::receiveTCP, this); }
    void startHandleUDP() { udpReceiveThread = std::thread(&TestObject::receiveUDP, this); }
    void startSendMONR() { monrThread = std::thread(&TestObject::monrLoop, this); }
    void startHEABCheck() { heabTimeoutThread = std::thread(&TestObject::checkHeabTimeout, this); }
    //! Function for handling received ISO messages. Calls corresponding 
    //! handler in the current state.
    int handleMessage(std::vector<char>*);
    //! Sends MONR message on process channel
    void sendMONR(bool debug = false);
    //! Called if HEAB messages do not arrive on time
    void onHeabTimeout();
    //! Loop function that checks if HEABs arrive on time
    void checkHeabTimeout();

    sigslot::signal<>heabTimeout;
    std::mutex recvMutex;
    std::thread tcpReceiveThread;
    std::thread udpReceiveThread;
	std::string localIP;
    std::thread monrThread;
    std::thread heabTimeoutThread;
    ISO22133::State* state;
    std::string name;        
    TCPHandler controlChannel;
    UDPHandler processChannel;   
    TrajDecoder trajDecoder;     
    std::atomic<bool> udpOk { false };
    std::atomic<bool> on { true };
    std::atomic<bool> firstHeab { true };
    std::atomic<struct timeval> lastHeabTime;
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



};
} // namespace ISO22133
