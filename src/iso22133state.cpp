#include "iso22133object.hpp"
#include "iso22133state.hpp"
#include <algorithm>

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
	obj.state->onEnter(obj);
}

void ISO22133::State::_handleHEAB(TestObject& obj,HeabMessageDataType& heab) {
	switch (heab.controlCenterStatus) {
	case CONTROL_CENTER_STATUS_NORMAL_STOP:
		this->handleEvent(obj, ISO22133::Events::U);
		break;
	case CONTROL_CENTER_STATUS_ABORT:
		this->handleEvent(obj, ISO22133::Events::W);
		break;
	case CONTROL_CENTER_STATUS_TEST_DONE:
		this->handleEvent(obj, ISO22133::Events::Y);
		break;
	default:
		break;
	}
	return;
}


void ISO22133::Armed::_handleSTRT(TestObject&,strt&) {
	return; // TODO
}

void ISO22133::Armed::_handleOSTM(TestObject& obj,ObjectCommandType& ostm) {
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


void ISO22133::Disarmed::_handleHEAB(TestObject& obj,HeabMessageDataType& heab) {
	switch (heab.controlCenterStatus) {
	case CONTROL_CENTER_STATUS_NORMAL_STOP:
		this->handleEvent(obj, ISO22133::Events::U);
		break;
	case CONTROL_CENTER_STATUS_ABORT:
		this->handleEvent(obj, ISO22133::Events::W);
		break;
	case CONTROL_CENTER_STATUS_TEST_DONE:
		this->handleEvent(obj, ISO22133::Events::Y);
		break;
	default:
		break;
	}
	return; 
}

void ISO22133::Disarmed::_handleTRAJ(TestObject&,traj&) {
	return; // TODO
}

void ISO22133::Disarmed::_handleOSEM(TestObject& obj,ObjectSettingsType& osem) {
	obj.origin = osem.coordinateSystemOrigin;
	obj.transmitterID = osem.desiredTransmitterID;
	std::cout << "Got OSEM - set transmitter ID to " << osem.desiredTransmitterID << std::endl;
	setTransmitterID(osem.desiredTransmitterID);
	return; 
}

void ISO22133::Disarmed::_handleOSTM(TestObject& obj,ObjectCommandType& ostm) {
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
	return; 
}

void ISO22133::Disarmed::onEnter(TestObject& obj) {
	obj.setReadyToArm(OBJECT_READY_TO_ARM);
}

void ISO22133::Disarmed::onExit(TestObject& obj) {
	obj.setReadyToArm(OBJECT_NOT_READY_TO_ARM);
}


void ISO22133::Running::_handleHEAB(TestObject& obj,HeabMessageDataType& heab) {
	switch (heab.controlCenterStatus) {
	case CONTROL_CENTER_STATUS_NORMAL_STOP:
		this->handleEvent(obj, ISO22133::Events::U);
		break;
	case CONTROL_CENTER_STATUS_ABORT:
		this->handleEvent(obj, ISO22133::Events::W);
		break;
	case CONTROL_CENTER_STATUS_TEST_DONE:
		this->handleEvent(obj, ISO22133::Events::Y);
		break;
	default:
		break;
	}
	return; 
}


void ISO22133::RemoteControlled::_handleOSTM(TestObject& obj,ObjectCommandType& ostm) {
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
	return; 
}


void ISO22133::Aborting::_handleHEAB(TestObject& obj,HeabMessageDataType& heab) {
	// Do nothing here since we require a event X to leave state
	switch (heab.controlCenterStatus) {
	case CONTROL_CENTER_STATUS_NORMAL_STOP:
		break;
	case CONTROL_CENTER_STATUS_ABORT:
		break;
	case CONTROL_CENTER_STATUS_TEST_DONE:
		break;
	default:
		break;
	}
	return;
}
