#pragma once

#include <set>
#include <map>
#include <iostream>
#include <functional>

#include "iso22133.h"

namespace ISO22133 {
class Object;


// TODO this should be in iso22133.h
typedef ObjectStateType ObjectStateID;

namespace Events {
typedef enum {
	D, //!< system started
	L, //!< control center connection lost
	B, //!< control center connection established
	H, //!< remote control requested
	M, //!< arm requested
	N, //!< arm completed
	S, //!< start requested
	T, //!< start completed
	U, //!< soft stop requested
	Y, //!< scenario completed
	R, //!< object stopped
	W, //!< emergency stop requested
	E, //!< internal error detected
	F, //!< disarm requested
	X  //!< emergency stop reset
} EventType;

static const std::map<EventType, std::string> descriptions = {
	{D, "system started"},
	{L, "control center connection lost"},
	{B, "control center connection established"},
	{H, "remote control requested"},
	{M, "arm requested"},
	{N, "arm completed"},
	{S, "start requested"},
	{T, "start completed"},
	{U, "soft stop requested"},
	{Y, "scenario completed"},
	{R, "object stopped"},
	{W, "emergency stop requested"},
	{E, "internal error detected"},
	{F, "disarm requested"},
	{X, "emergency stop reset"}
};

}

// TODO move to iso22133.h
// and get rid of the std::map :(
static const std::map<ObjectStateID, std::string> stateNames = {
	{OBJECT_STATE_UNKNOWN, "Unknown"},
	{OBJECT_STATE_INIT, "Init"},
	{OBJECT_STATE_ARMED, "Armed"},
	{OBJECT_STATE_DISARMED, "Disarmed"},
	{OBJECT_STATE_RUNNING, "Running"},
	{OBJECT_STATE_POSTRUN, "NormalStop"},
	{OBJECT_STATE_REMOTE_CONTROL, "RemoteControlled"},
	{OBJECT_STATE_ABORTING, "EmergencyStop"},
	{OBJECT_STATE_PRE_ARMING, "PreArming"},
	{OBJECT_STATE_PRE_RUNNING, "PreRunning"}
};


/*!
 * \brief The State class is an abstract form of the states used in
 *			ISO 22133. It is not intended to be used outside of
 *			iso22133object.h
 */
class State {
	friend class Object;
public:
	State() {}
	virtual ~State() {}

	virtual ObjectStateID getStateID() const = 0;
	std::string getName() const { return stateNames.at(getStateID()); }

	//! Handle events according to ISO22133 state change description
	void handleEvent(Object&, const Events::EventType);
	//! Will be called on transition, indended to be
	//! overridden by the inheriting class if necessary
	virtual void onEnter(Object&) {}
	virtual void onExit(Object&) {}

	//! When a message arrives, these methods are
	//! the 'front line' handlers.
	//! Handles ISO22133 required actions from control center
	virtual void handleTRAJ(Object&) { unexpectedMessageWarning("TRAJ"); }
	virtual void handleOSEM(Object&, ObjectSettingsType&) { unexpectedMessageWarning("OSEM"); }
	virtual void handleOSTM(Object&, ObjectCommandType&);
	virtual void handleSTRT(Object&, StartMessageType&);
protected:
#ifndef SWIG
	[[noreturn]]
#endif
	void unexpectedMessageError(const std::string& msgName) {
		throw std::runtime_error("Unexpected " + msgName + " in state " + this->getName());
	}
	void unexpectedMessageWarning(const std::string& msgName) {
		std::cerr << "Unexpected " + msgName + " in state " + this->getName() << std::endl;
	}
};

class Unknown : public State {
public:
	virtual ObjectStateID getStateID() const final override { return OBJECT_STATE_UNKNOWN; }
};

class Init : public State {
public:
	virtual ObjectStateID getStateID() const final override { return OBJECT_STATE_INIT; }
	virtual void onExit(Object&) override;
};

class Armed : public State {
public:
	virtual ObjectStateID getStateID() const final override { return OBJECT_STATE_ARMED; }
};

class Disarmed : public State {
public:
	virtual ObjectStateID getStateID() const final override { return OBJECT_STATE_DISARMED; }
	virtual void onEnter(Object& obj);
	virtual void onExit(Object& obj);
	//virtual void handleOSEM(Object&, ObjectCommandType&) final override;
    virtual void handleTRAJ(Object&) final override;
};

class Running : public State {
public:
	virtual ObjectStateID getStateID() const final override { return OBJECT_STATE_RUNNING; }
};

class PostRun : public State {
public:
	virtual ObjectStateID getStateID() const final override { return OBJECT_STATE_POSTRUN; }
};

class RemoteControlled : public State {
public:
	virtual ObjectStateID getStateID() const final override { return OBJECT_STATE_REMOTE_CONTROL; }
};
class Aborting : public State {
public:
	virtual ObjectStateID getStateID() const final override { return OBJECT_STATE_ABORTING; }
};

class PreArming : public State {
public:
	virtual ObjectStateID getStateID() const final override { return OBJECT_STATE_PRE_ARMING; }
	virtual void onEnter(Object& obj) {this->handleEvent(obj, Events::N);}
};

class PreRunning : public State {
public:
	virtual ObjectStateID getStateID() const final override { return OBJECT_STATE_PRE_RUNNING; }
	virtual void onEnter(Object& obj) {this->handleEvent(obj, Events::T);}
};


typedef struct {
	ObjectStateID source;
	Events::EventType event;
	ObjectStateID target;
} Transition;

inline bool operator< (const Transition &lhs, const Transition &rhs){
	return std::tie(lhs.source, lhs.event, lhs.target) < std::tie(rhs.source, rhs.event, rhs.target);
}

static const std::set<Transition> language = {
	{OBJECT_STATE_UNKNOWN,				Events::D,		OBJECT_STATE_INIT},
	{OBJECT_STATE_INIT,					Events::L,		OBJECT_STATE_INIT},
	{OBJECT_STATE_INIT,					Events::B,		OBJECT_STATE_DISARMED},
	{OBJECT_STATE_ARMED,				Events::E,		OBJECT_STATE_DISARMED},
	{OBJECT_STATE_ARMED,				Events::F,		OBJECT_STATE_DISARMED},
	{OBJECT_STATE_ARMED,				Events::W,		OBJECT_STATE_ABORTING},
	{OBJECT_STATE_ARMED,				Events::S,		OBJECT_STATE_PRE_RUNNING},
	{OBJECT_STATE_ARMED,				Events::L,		OBJECT_STATE_INIT},
	{OBJECT_STATE_DISARMED,				Events::M,		OBJECT_STATE_PRE_ARMING},
	{OBJECT_STATE_DISARMED,				Events::H,		OBJECT_STATE_REMOTE_CONTROL},
	{OBJECT_STATE_DISARMED,				Events::L,		OBJECT_STATE_INIT},
	{OBJECT_STATE_DISARMED,				Events::W,		OBJECT_STATE_ABORTING},
	{OBJECT_STATE_RUNNING,				Events::U,		OBJECT_STATE_POSTRUN},
	{OBJECT_STATE_RUNNING,				Events::Y,		OBJECT_STATE_POSTRUN},
	{OBJECT_STATE_RUNNING,				Events::W,		OBJECT_STATE_ABORTING},
	{OBJECT_STATE_RUNNING,				Events::L,		OBJECT_STATE_INIT},
	{OBJECT_STATE_POSTRUN,				Events::R,		OBJECT_STATE_DISARMED},
	{OBJECT_STATE_POSTRUN,				Events::W,		OBJECT_STATE_ABORTING},
	{OBJECT_STATE_POSTRUN,				Events::L,		OBJECT_STATE_INIT},
	{OBJECT_STATE_REMOTE_CONTROL,   	Events::F,		OBJECT_STATE_DISARMED},
	{OBJECT_STATE_REMOTE_CONTROL,   	Events::W,		OBJECT_STATE_ABORTING},
	{OBJECT_STATE_REMOTE_CONTROL,   	Events::L,		OBJECT_STATE_INIT},
	{OBJECT_STATE_ABORTING,				Events::W,		OBJECT_STATE_ABORTING},
	{OBJECT_STATE_ABORTING,				Events::X,		OBJECT_STATE_DISARMED},
	{OBJECT_STATE_ABORTING,				Events::L,		OBJECT_STATE_INIT},
	{OBJECT_STATE_PRE_ARMING,			Events::N,		OBJECT_STATE_ARMED},
	{OBJECT_STATE_PRE_ARMING,			Events::L,		OBJECT_STATE_INIT},
	{OBJECT_STATE_PRE_ARMING,			Events::W,		OBJECT_STATE_ABORTING},
	{OBJECT_STATE_PRE_RUNNING,			Events::T,		OBJECT_STATE_RUNNING},
	{OBJECT_STATE_PRE_RUNNING,			Events::L,		OBJECT_STATE_INIT},
	{OBJECT_STATE_PRE_RUNNING,			Events::W,		OBJECT_STATE_ABORTING}
};

} // end namespace ISO22133
