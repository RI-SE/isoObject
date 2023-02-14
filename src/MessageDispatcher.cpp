#include "MessageDispatcher.hpp"
//#include "State.hpp"
#include "Object.hpp"
#include "TrajDecoder.hpp"
#include "utilities.hpp"
#include "iso22133.h"
#include <iostream>

MessageDispatcher::MessageDispatcher(
	ISO22133::State& state,
	ISO22133::Object& object)
	: state(state),
	object(object),
	trajDecoder(std::make_unique<ISO22133::TrajDecoder>())
{}

MessageDispatcher::~MessageDispatcher() = default;

size_t MessageDispatcher::dispatch(
    const std::vector<char>& message)
{
	int bytesHandled = 0;
    char debug = 0;
	auto currentTime = std::chrono::to_timeval(std::chrono::system_clock::now().time_since_epoch());

    ISOMessageID msgType = getISOMessageType(message.data(), message.size(), false);
	// Ugly check here since we don't know if it is UDP or the rest of TRAJ
	if (msgType == MESSAGE_ID_INVALID && trajDecoder->expectingTrajPoints()) {
		msgType = MESSAGE_ID_TRAJ;
	}

	switch (msgType) {
	case MESSAGE_ID_TRAJ:
		bytesHandled = trajDecoder->decode(message);
		if (bytesHandled < 0) {
			throw std::invalid_argument("Error decoding TRAJ");
		}
		if (!trajDecoder->expectingTrajPoints()) {
			std::cout << "Received TRAJ" << std::endl;
			//state.handleTRAJ(*this);
		}
		break;
	case MESSAGE_ID_OSEM:
		ObjectSettingsType OSEMstruct;
		bytesHandled = decodeOSEMMessage(&OSEMstruct, message.data(), message.size(), debug);
		if (bytesHandled < 0) {
			throw std::invalid_argument("Error decoding OSEM");
		}
		std::cout << "Received OSEM" << std::endl;
		//state.handleOSEM(*this, OSEMstruct);
		break;

	case MESSAGE_ID_OSTM:
		ObjectCommandType OSTMdata;
		bytesHandled = decodeOSTMMessage(message.data(), message.size(), &OSTMdata, debug);
		if (bytesHandled < 0) {
			throw std::invalid_argument("Error decoding OSTM");
		}
		std::cout << "Received OSTM" << std::endl;
		//state.handleOSTM(*this, OSTMdata);
		
		break;

	case MESSAGE_ID_STRT:
		StartMessageType STRTdata;
		bytesHandled
			= decodeSTRTMessage(message.data(), message.size(), &currentTime, &STRTdata, debug);
		if (bytesHandled < 0) {
			throw std::invalid_argument("Error decoding STRT");
		}
		std::cout << "Received STRT" << std::endl;
		//state.handleSTRT(*this, STRTdata);
		break;

	case MESSAGE_ID_HEAB:
		HeabMessageDataType HEABdata;

		bytesHandled = decodeHEABMessage(message.data(), message.size(), currentTime, &HEABdata, debug);
		if (bytesHandled < 0) {
			throw std::invalid_argument("Error decoding HEAB");
		}
		//object.handleHEAB(HEABdata);
		break;
	default:
		//bytesHandled = state.handleVendorSpecificMessage(msgType, message);
		if (bytesHandled < 0) {
			throw std::invalid_argument(std::string("Unable to decode ISO-22133 message with MsgID ")
										+ std::to_string(msgType));
		}
		bytesHandled = message.size();
		break;
	}
	return bytesHandled;
}