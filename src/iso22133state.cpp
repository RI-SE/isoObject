#include "iso22133object.hpp"
#include "iso22133state.hpp"
#include <algorithm>

/**
 * @brief Handle events according to ISO22133 state change description
 * 
 * @param obj TestObject reference
 * @param event Event according to ISO22133::EventType
 */
void ISO22133::State::handleEvent(
		TestObject& obj,
		const ISO22133::Events::EventType event) {
	auto transition = std::find_if(language.begin(), language.end(),
								   [&event, this](const ISO22133::Transition& tr) {
		return event == tr.event && this->getStateID() == tr.source;
	});
	if (transition == language.end()) {
		throw std::runtime_error(std::string("Unexpected event '") 
								 + Events::descriptions.at(event)
								 + "' in state " + this->getName());
	}

	std::cout << "Leaving state: " << obj.state->getName() << std::endl;
	obj.state->onExit(obj);
	State* temp = obj.state;

	switch (transition->target) {
	case ISO_OBJECT_STATE_OFF:
		obj.state = obj.createOff();
		break;
	case ISO_OBJECT_STATE_INIT:
		obj.state = obj.createInit();
		break;
	case ISO_OBJECT_STATE_ARMED:
		obj.state = obj.createArmed();
		break;
	case ISO_OBJECT_STATE_DISARMED:
		obj.state = obj.createDisarmed();
		break;
	case ISO_OBJECT_STATE_RUNNING:
		obj.state = obj.createRunning();
		break;
	case ISO_OBJECT_STATE_POSTRUN:
		obj.state = obj.createPostRun();
		break;
	case ISO_OBJECT_STATE_REMOTE_CONTROLLED:
		obj.state = obj.createRemoteControlled();
		break;
	case ISO_OBJECT_STATE_ABORTING:
		obj.state = obj.createAborting();
		break;
	case ISO_OBJECT_STATE_PRE_ARMING:
		obj.state = obj.createPreArming();
		break;
	case ISO_OBJECT_STATE_PRE_RUNNING:
		obj.state = obj.createPreRunning();
		break;
	case ISO_OBJECT_STATE_UNKNOWN:
	default:
		obj.state = obj.createUnknown();
		break;
	}
	delete temp;
	std::cout << "Entering state: " << obj.state->getName() << std::endl;
	obj.stateChangeSig();
	obj.state->onEnter(obj);
}

/**
 * @brief Generates state changes based on control center status
 * 
 * @param obj TestObject reference
 * @param heab struct HeabMessageDataType
 */
void ISO22133::State::handleHEAB(TestObject& obj,HeabMessageDataType& heab) {
	// Order matters here, below may change state
	// causing the signal to not be triggered if placed
	// after the handleEvent() calls
	obj.heabSig(heab);
	switch (heab.controlCenterStatus) {
	case CONTROL_CENTER_STATUS_NORMAL_STOP:
		this->handleEvent(obj, ISO22133::Events::U);
		break;
	case CONTROL_CENTER_STATUS_ABORT:
		if(this->getStateID() != ISO_OBJECT_STATE_ABORTING) {
			this->handleEvent(obj, ISO22133::Events::W);
			obj.handleAbort();
		}
		break;
	case CONTROL_CENTER_STATUS_TEST_DONE:
		this->handleEvent(obj, ISO22133::Events::Y);
		break;
	default:
		break;
	}
	return;
}

/**
 * @brief Handles state change requests from Control center
 * 
 * @param obj TestObject reference
 * @param ostm struct ObjectCommandType
 */
void ISO22133::State::handleOSTM(TestObject& obj,ObjectCommandType& ostm) {
	// Order matters here, below may change state
	// causing the signal to not be triggered if placed
	// after the handleEvent() calls
	obj.ostmSig(ostm); 
	switch (ostm) {
	case OBJECT_COMMAND_ARM:
		this->handleEvent(obj, ISO22133::Events::M);
		break;
	case OBJECT_COMMAND_DISARM:
		this->handleEvent(obj, ISO22133::Events::F);
		break;
	case OBJECT_COMMAND_REMOTE_CONTROL:
		this->handleEvent(obj, ISO22133::Events::H);
		break;
	default:
		break;

	}
}

/**
 * @brief Changes object settings
 * 
 * @param obj TestObject reference
 * @param osem struct ObjectSettingsType
 */
void ISO22133::State::handleOSEM(TestObject& obj,ObjectSettingsType& osem) {
	obj.origin = osem.coordinateSystemOrigin;
	obj.transmitterID = osem.desiredTransmitterID;
	std::cout << "Got OSEM - set transmitter ID to " << osem.desiredTransmitterID << std::endl;
	setTransmitterID(osem.desiredTransmitterID);

	obj.osemSig(osem);
	return; 
}

/**
 * @brief Handles start request from Control center
 * 
 * @param obj TestObject reference
 * @param strt struct TODO
 */
void ISO22133::State::handleSTRT(TestObject& obj,StartMessageType& strt) {
	// Order matters here, below changes state
	// causing the signal to not be triggered if placed
	// after the handleEvent() calls
	obj.strtSig(strt);
	this->handleEvent(obj, ISO22133::Events::S);
	return; 
}

/**
 * @brief TODO
 * 
 * @param obj 
 * @param traj 
 */
void ISO22133::State::handleTRAJ(TestObject& obj) {
	// Signal TRAJ is now available
	obj.trajSig();
	return; 
}

/**
 * @brief Sets TestObject ready to arm flag
 * 
 * @param obj TestObject reference
 */
void ISO22133::Disarmed::onEnter(TestObject& obj) {
	obj.setReadyToArm(OBJECT_READY_TO_ARM);
}

/**
 * @brief Resets TestObject ready to arm flag
 * 
 * @param obj TestObject reference
 */
void ISO22133::Disarmed::onExit(TestObject& obj) {
	obj.setReadyToArm(OBJECT_NOT_READY_TO_ARM);
}
