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
    TestObject() : name_("myTestObject"), controlChannel_(), processChannel_() {
        this->state = this->createInit();
        this->startHandleTCP();
    }

    virtual ~TestObject(){
        on_ = false;
        tcpReceiveThread_.join();
        udpReceiveThread_.join();
        }; 
    
    /**
     * @brief The user is responsible for calling this function
     *          at 100Hz to send MONR messages.
     * 
     * @param debug 1 = debug, 0 = no debug
     */
    void sendMONR(char debug);
    
    bool isServerConnected(){return controlChannel_.isConnected();}
    bool isUdpOk(){ return udpOk_;};
    std::string getCurrentStateName() { return state->getName();}
    std::string getName() { return name_; }
    CartesianPosition getPosition() { return position_;}
    SpeedType getSpeed() { return speed_;}
    AccelerationType getAcceleration() { return acceleration_;}
    DriveDirectionType getDriveDirection() { return driveDirection_;}



protected:
    
    //! Pure virtual safety function that must be implemented by the user.
    virtual void handleAbort() = 0;
    
    void setPosition(CartesianPosition& pos) {position_ = pos;}
    void setSpeed(SpeedType& spd) {speed_ = spd;}
    void setAcceleration(AccelerationType& acc) {acceleration_ = acc;}
    void setDriveDirection(DriveDirectionType& drd) {driveDirection_ = drd;}
    void setObjectState(ObjectStateID& ost) {objectState_ = ost;}
    void setName(std::string name) {name_ = name;}
    void setReadyToArm(const int& rdy) {readyToArm_ = rdy;}
    void setErrorState(const char& err) {errorState_ = err;}      

    // These should be overridden if extending one of the states
    // Example of override:
    // return dynamic_cast<ISO22133::PreArming*>(new myPreArming);
    virtual Unknown* createUnknown() { return new Unknown; }
    virtual Off* createOff() { return new Off; }
    virtual Init* createInit() { return new Init; }
    virtual Armed* createArmed() { return new Armed; }
    virtual Disarmed* createDisarmed() { return new Disarmed; }
    virtual Running* createRunning() { return new Running; }
    virtual PostRun* createPostRun() { return new PostRun; }
    virtual RemoteControlled* createRemoteControlled() { return new RemoteControlled; }
    virtual Aborting* createAborting() { return new Aborting; }
    virtual PreArming* createPreArming() { return new PreArming; }
    virtual PreRunning* createPreRunning() { return new PreRunning; }
    
    
private:
    
    //! UDP receiver loop that should be run in its own thread.
    void receiveUDP();
    //! TCP receiver loop that should be run in its own thread.
    void receiveTCP();
    void startHandleTCP(){tcpReceiveThread_ = std::thread(&TestObject::receiveTCP, this);};
    void startHandleUDP(){udpReceiveThread_ = std::thread(&TestObject::receiveUDP, this);};
    //! Function for handling received ISO messages. Calls corresponding 
    //! handler in the current state.
    int handleMessage(std::vector<char>*);

    std::mutex recvMutex_;
    const static int controlChannelPort_ = 53241; // TCP
    const static int processChannelPort_ = 53240; // UDP
    bool udpOk_ = false;
    bool on_ = true;
    std::thread tcpReceiveThread_;
    std::thread udpReceiveThread_;
    ISO22133::State* state;
    std::string name_;        
    TCPHandler controlChannel_ ;
    UDPHandler processChannel_ ;        
    GeographicPositionType origin_; 
    ControlCenterStatusType ccStatus_;
    CartesianPosition position_;
    SpeedType speed_;
    AccelerationType acceleration_;
    DriveDirectionType driveDirection_ = OBJECT_DRIVE_DIRECTION_UNAVAILABLE;
    ObjectStateID objectState_ = ISO_OBJECT_STATE_UNKNOWN;
    int readyToArm_ = OBJECT_READY_TO_ARM_UNAVAILABLE;
    int transmitterID_;
    char errorState_ = 0;


};
} // namespace ISO22133

