#pragma once

#include <string>
#include <iostream>
#include <thread> 
#include <mutex>

#include <optional>

#include "iso22133.h"
#include "iso22133state.hpp"
#include "tcphandler.hpp"
#include "udphandler.hpp"
#include "sigslot/signal.hpp"

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
    TestObject() : name("myTestObject"), controlChannel(), processChannel() {
        this->state = this->createInit();
        this->startHandleTCP();
        this->stateChangeSig.connect(&TestObject::onStateChange, this);
        this->osemSig.connect(&TestObject::onOSEM, this);
        this->heabSig.connect(&TestObject::onHEAB, this);
        this->ostmSig.connect(&TestObject::onOSTM, this);
        this->trajSig.connect(&TestObject::onTRAJ, this);
        this->strtSig.connect(&TestObject::onSTRT, this);
    }

    virtual ~TestObject() {
        on = false;
        tcpReceiveThread.join();
        udpReceiveThread.join();
    }; 
    
    /**
     * @brief The user is responsible for calling this function
     *          at 100Hz to send MONR messages.
     * 
     * @param debug true / false
     */
    void sendMONR(bool debug = false);
    
    bool isServerConnected() const { return controlChannel.isConnected(); }
    bool isUdpOk() const { return udpOk; }
    std::string getCurrentStateName() const { return state->getName(); }
    std::string getName() const { return name; }
    CartesianPosition getPosition() const { return position; }
    SpeedType getSpeed() const { return speed; }
    AccelerationType getAcceleration() const { return acceleration; }
    DriveDirectionType getDriveDirection() const { return driveDirection; }



protected:
    
    //! Pure virtual safety function that must be implemented by the user.
    virtual void handleAbort() = 0;

    void setPosition(CartesianPosition& pos) { position = pos; }
    void setSpeed(SpeedType& spd) { speed = spd; }
    void setAcceleration(AccelerationType& acc) { acceleration = acc; }
    void setDriveDirection(DriveDirectionType& drd) { driveDirection = drd; }
    void setObjectState(ObjectStateID& ost) { objectState = ost; }
    void setName(std::string name) { name = name; }
    void setReadyToArm(const int& rdy) {readyToArm = rdy;}
    void setErrorState(const char& err) {errorState = err;}      

    // These should be overridden if extending one of the states
    // Example of override:
    // return dynamic_cast<ISO22133::PreArming*>(new myPreArming);
    //! Must be overridden if modifying the Unknown state
    virtual Unknown* createUnknown() const { return new Unknown; }
    //! Must be overridden id modifying the Off state
    virtual Off* createOff() const { return new Off; }
    //! Must be overridden id modifying the Init state
    virtual Init* createInit() const { return new Init; }
    //! Must be overridden id modifying the Armed state
    virtual Armed* createArmed() const { return new Armed; }
    //! Must be overridden id modifying the Disarmed state
    virtual Disarmed* createDisarmed() const { return new Disarmed; }
    //! Must be overridden id modifying the Running state
    virtual Running* createRunning() const { return new Running; }
    //! Must be overridden id modifying the Postrun state
    virtual PostRun* createPostRun() const { return new PostRun; }
    //! Must be overridden id modifying the Remote Controlled state
    virtual RemoteControlled* createRemoteControlled() const { return new RemoteControlled; }
    //! Must be overridden id modifying the Abort state
    virtual Aborting* createAborting() const { return new Aborting; }
    //! Must be overridden id modifying the Pre-Arm state
    virtual PreArming* createPreArming() const { return new PreArming; }
    //! Must be overridden id modifying the Pre-Running state
    virtual PreRunning* createPreRunning() const { return new PreRunning; }


    //! Signals for events
    sigslot::signal<> stateChangeSig;
    sigslot::signal<ObjectSettingsType&> osemSig;
	sigslot::signal<HeabMessageDataType&> heabSig;
    sigslot::signal<> trajSig; // TODO type
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

    TrajectorHeaderType trajectoryHeader;
    std::vector<TrajectorWaypointType> trajectory;

    
private:
    
    //! UDP receiver loop that should be run in its own thread.
    void receiveUDP();
    //! TCP receiver loop that should be run in its own thread.
    void receiveTCP();
    void startHandleTCP() { tcpReceiveThread = std::thread(&TestObject::receiveTCP, this); }
    void startHandleUDP() { udpReceiveThread = std::thread(&TestObject::receiveUDP, this); }
    //! Function for handling received ISO messages. Calls corresponding 
    //! handler in the current state.
    int handleMessage(std::vector<char>*);

    std::mutex recvMutex;
    bool udpOk = false;
    bool on = true;
    bool firstHeab = true;
    std::thread tcpReceiveThread;
    std::thread udpReceiveThread;
    ISO22133::State* state;
    std::string name;        
    TCPHandler controlChannel;
    UDPHandler processChannel;        
    GeographicPositionType origin; 
    ControlCenterStatusType ccStatus;
    CartesianPosition position;
    SpeedType speed;
    AccelerationType acceleration;
    DriveDirectionType driveDirection = OBJECT_DRIVE_DIRECTION_UNAVAILABLE;
    ObjectStateID objectState = ISO_OBJECT_STATE_UNKNOWN;
    int readyToArm = OBJECT_READY_TO_ARM_UNAVAILABLE;
    int transmitterID;
    char errorState = 0;


};
} // namespace ISO22133
