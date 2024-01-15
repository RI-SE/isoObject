#include <algorithm>

#include "iso22133object.hpp"
#include "iso22133state.hpp"

/**
 * @brief Handle events according to ISO22133 state change description
 *
 * @param obj TestObject reference
 * @param event Event according to ISO22133::EventType
 */
void ISO22133::State::handleEvent(TestObject& obj, const ISO22133::Events::EventType event) {
	std::scoped_lock lock(eventMutex);
	auto transition
		= std::find_if(language.begin(), language.end(), [&event, this](const ISO22133::Transition& tr) {
			  return event == tr.event && this->getStateID() == tr.source;
		  });
	if (transition == language.end()) {
		throw std::runtime_error(std::string("Unexpected event '") + Events::descriptions.at(event)
								 + "' in state " + this->getName());
	}
	if (transition->source == transition->target) {
		return;
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
		obj.handleAbort();
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
 * @brief Handles state change requests from Control center
 *
 * @param obj TestObject reference
 * @param ostm struct ObjectCommandType
 */
void ISO22133::State::handleOSTM(TestObject& obj, ObjectCommandType& ostm) {
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
	case OBJECT_COMMAND_ALL_CLEAR:
		this->handleEvent(obj, ISO22133::Events::X);
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
void ISO22133::State::handleOSEM(TestObject& obj, ObjectSettingsType& osem) {
	obj.origin = osem.coordinateSystemOrigin;
	obj.transmitterID = osem.desiredID.transmitter;
	obj.receiverID = osem.desiredID.controlCentre;

	std::stringstream msg;	// Remove risk of not printing the whole message due to threading
	msg << "Got OSEM - set transmitter ID to " << obj.transmitterID << std::endl;
	std::cout << msg.str();

	obj.expectedHeartbeatPeriod = std::chrono::milliseconds(1000 / (uint)osem.rate.heab);
	msg.str(std::string());
	msg << "Setting HEAB period to " << obj.expectedHeartbeatPeriod.count() << " ms. ("
		<< 1000 / obj.expectedHeartbeatPeriod.count() << " Hz) " << std::endl;
	std::cout << msg.str();

	obj.heartbeatTimeout = 10*obj.expectedHeartbeatPeriod;
	msg.str(std::string());
	msg << "Set HEAB timeout to " << obj.heartbeatTimeout.count() << " ms. ("
		<< 1000 / obj.heartbeatTimeout.count() << " Hz) " << std::endl;
	std::cout << msg.str();

	obj.monrPeriod = std::chrono::milliseconds(1000 / (uint)osem.rate.monr);
	msg.str(std::string());
	msg << "Setting MONR period to " << obj.monrPeriod.count() << " ms. ("
		<< 1000 / obj.monrPeriod.count() << " Hz) " << std::endl;
	std::cout << msg.str();

	obj.osemSig(osem);
	return;
}

/**
 * @brief Handles start request from Control center
 *
 * @param obj TestObject reference
 * @param strt struct TODO
 */
void ISO22133::State::handleSTRT(TestObject& obj, StartMessageType& strt) {

	// No delayed start, start immediately
	if(!strt.isTimestampValid) {
		// Order matters here, below changes state
		// causing the signal to not be triggered if placed
		// after the handleEvent() calls
		obj.strtSig(strt);
		this->handleEvent(obj, ISO22133::Events::S);
		return;
	}
	
	// Current time with compensation for network delay
	auto currentTime =  std::chrono::to_timeval(
		std::chrono::system_clock::now().time_since_epoch() - 
		obj.getNetworkDelay()
	);

	struct timeval diff;
	timersub(&strt.startTime, &currentTime, &diff);
	uint32_t diffmySec = diff.tv_sec*1e6 + diff.tv_usec;
	int diffint = diff.tv_sec*1e6 + diff.tv_usec;
	
	// Start time already passed. Request abort from Control Center
	// resolution is 0,25ms (250 microseconds) in ISO spec.
	if(diffint > -250) {
		std::stringstream ss;
		ss << "Got STRT message with start time in " << diff.tv_sec << " seconds, " << diff.tv_usec << " mySecs. Waiting" << std::endl;
		std::cout << ss.str();

		obj.delayedStrtThread = std::thread([&, diffmySec]() {
			std::this_thread::sleep_for(std::chrono::microseconds(diffmySec));
			// Order matters here, below changes state
			// causing the signal to not be triggered if placed
			// after the handleEvent() calls
			obj.strtSig(strt);
			this->handleEvent(obj, ISO22133::Events::S);
		});		
	}
	else {
		std::stringstream ss;
		ss << "Got STRT message with start time in the past. Requesting abort." << std::endl;
		ss << "Requested time: " << strt.startTime.tv_sec << " seconds, " << strt.startTime.tv_usec << " mySecs." << std::endl;
		ss << "Current time: " << currentTime.tv_sec << " seconds, " << currentTime.tv_usec << " mySecs." << std::endl;
		ss << "Estimated network delay: " << obj.getNetworkDelay().count() << " mySecs." << std::endl;
		std::cout << ss.str();
		uint8_t error = 0;
		error |= 1 << 7; // Abort request is MSB of error mask 
		obj.setErrorState(error);
		return;
	}
	return;
}

/**
 * @brief Signals that a new TRAJ is available and sends GREM if in online planned mode
 *		  and object is receiving TRAJ chunks. 
 * @param obj
 * @param msgHeader
 */
void ISO22133::State::handleTRAJ(TestObject& obj, std::atomic<HeaderType>& msgHeader) {
	// Signal TRAJ is now available
	obj.trajSig();
	if (obj.getObjectSettings().testMode == TEST_MODE_ONLINE) {
		// Acknowledge TRAJ chunk with GREM when in online planned mode
		obj.sendGREM(msgHeader, GREM_CHUNK_RECEIVED, false);
	}
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
