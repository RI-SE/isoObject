#include <chrono>
#include <thread>

#include "iso22133object.hpp"
#include "iso22133state.hpp"
#include "defines.h"  // From ISO22133 lib

#define TCP_BUFFER_SIZE 1024
#define UDP_BUFFER_SIZE 1024


namespace ISO22133 {
TestObject::TestObject(const std::string& listenIP)
	: name("myTestObject"),
	  trajDecoder(),
	  ctrlChannel(listenIP, ISO_22133_DEFAULT_OBJECT_TCP_PORT),
	  processChannel(listenIP, ISO_22133_OBJECT_UDP_PORT),
	  transmitterID(TRANSMITTER_ID_UNAVAILABLE_VALUE),
	  serverID(0),
	  expectedMessageCounter(0),
	  sentMessageCounter(0),
	  socketsReceivedFromController(false),
	  on(true) {
	this->initialize();
	this->startHandleTCP();
	this->startHEABCheck();
}

TestObject::TestObject(int tcpSocket, int id)
	: name("myTestObject"),
	  trajDecoder(),
	  ctrlChannel(tcpSocket),
	  processChannel(),
	  transmitterID(id),
	  serverID(0),
	  expectedMessageCounter(0),
	  sentMessageCounter(0),
	  socketsReceivedFromController(true),
	  on(true) {
	this->initialize();
	this->startHEABCheck();
}

void TestObject::initialize() {
	CartesianPosition initPos;
	SpeedType initSpd;
	AccelerationType initAcc;
	TestModeType initTm;
	localIP = "0.0.0.0";
	initPos.isHeadingValid = false;
	initPos.isPositionValid = false;
	initSpd.isLateralValid = false;
	initSpd.isLongitudinalValid = false;
	initAcc.isLateralValid = false;
	initAcc.isLongitudinalValid = false;
	initTm = TEST_MODE_UNAVAILABLE;
	this->setPosition(initPos);
	this->setSpeed(initSpd);
	this->setAcceleration(initAcc);
	this->state = this->createInit();
	this->stateChangeSig.connect(&TestObject::onStateChange, this);
	this->osemSig.connect(&TestObject::onOSEM, this);
	this->heabSig.connect(&TestObject::onHEAB, this);
	this->ostmSig.connect(&TestObject::onOSTM, this);
	this->trajSig.connect(&TestObject::onTRAJ, this);
	this->strtSig.connect(&TestObject::onSTRT, this);
	this->heabTimeout.connect(&TestObject::onHeabTimeout, this);
}

TestObject::~TestObject() {
	on = false;
	try {
		join(monrThread);
	} catch (std::system_error& e) {}
	try {
		join(heabTimeoutThread);
	} catch (std::system_error& e) {}
	try {
		join(udpReceiveThread);
	} catch (std::system_error& eaccess) {}
	try {
		join(tcpReceiveThread);
	} catch (std::system_error& e) {}
	try {
		join(delayedStrtThread);
	} catch (std::system_error& e) {}
};

void TestObject::disconnect() {
	std::scoped_lock lock(disconnectMutex);
	try {
		ctrlChannel.disconnect();  // Close TCP socket
	} catch (const std::exception& e) {
		std::cerr << "TCP socket close error: " << e.what() << '\n';
	}

	if (!socketsReceivedFromController) {
		processChannel.disconnect();  // Close UDP socket
	}
	awaitingFirstHeab = true;	  // Reset HEAB timeout check

	if (!socketsReceivedFromController) {
		try {
			join(udpReceiveThread);
		} catch (std::system_error& eaccess) {}
		try {
			join(monrThread);
		} catch (std::system_error& eaccess) {}
	}
}

MessageHeaderType *TestObject::populateMessageHeader(MessageHeaderType *header) {
	memset(header, 0, sizeof(MessageHeaderType));
	header->transmitterID = this->transmitterID;
	header->receiverID = this->serverID;
	header->messageCounter = this->sentMessageCounter++;
	return header;
}

void TestObject::receiveTCP() {
	std::stringstream ss;
	ss << "Started TCP thread." << std::endl;
	std::cout << ss.str();
	bool continueReceiving = true;

	while (this->on && continueReceiving) {
		if (!socketsReceivedFromController) {
			ss.str(std::string());
			ss << "Awaiting TCP connection from ATOS..." << std::endl;
			std::cout << ss.str();

			try {
				ctrlChannel.acceptConnection();
			} catch (boost::system::system_error& e) {
				ss.str(std::string());
				ss << "TCP accept failed: " << e.what() << std::endl;
				std::cout << ss.str();
				throw e;
			}
			ss.str(std::string());
			ss << "TCP connection to ATOS running at " << ctrlChannel.getEndPoint().address().to_string()
			<< " established." << std::endl;
			std::cout << ss.str();
		}

		if (!this->socketsReceivedFromController)
			state->handleEvent(*this, ISO22133::Events::B);
		try {
			while (true) {
				auto data = ctrlChannel.receive();

				int nBytesHandled = 0;
				do {
					try {
						nBytesHandled = handleMessage(data);
					} catch (const std::exception& e) {
						std::cerr << e.what() << std::endl;
						break;
					}
					data.erase(data.begin(), data.begin() + nBytesHandled);
				} while (data.size() > 0);
			}
		} catch (boost::system::system_error&) {
			std::cerr << transmitterID << " lost connection to ATOS" << std::endl;
			disconnect();
			state->handleEvent(*this, ISO22133::Events::L);
			if (socketsReceivedFromController) {
				continueReceiving = false;
			}
		}
	}
	ss.str(std::string());
	ss << transmitterID << " is exiting TCP thread." << std::endl;
	std::cout << ss.str();
}

void TestObject::sendMONR(bool debug) {
	std::vector<char> buffer(UDP_BUFFER_SIZE);
	struct timeval time;
	auto nanos = std::chrono::system_clock::now().time_since_epoch().count();
	time.tv_sec = nanos / 1e9;
	time.tv_usec = nanos / 1e3 - time.tv_sec * 1e6;

	MessageHeaderType header;
	auto nBytesWritten = encodeMONRMessage(this->populateMessageHeader(&header), &time, this->position, this->speed, this->acceleration,
									this->driveDirection, this->state->getStateID(), this->readyToArm,
									this->errorState, 0x0000, buffer.data(), buffer.size(), debug);

	if (nBytesWritten < 0) { throw(std::invalid_argument("Failed to encode MONR data"));}
	processChannel.send(buffer, static_cast<size_t>(nBytesWritten));
}

void TestObject::sendGREM(HeaderType msgHeader, GeneralResponseStatus responseCode, bool debug) {
	std::vector<char> buffer(TCP_BUFFER_SIZE);
	GeneralResponseMessageType grem;

	grem.receivedHeaderTransmitterID = msgHeader.transmitterID;
	grem.receivedHeaderMessageID = msgHeader.messageID;
	grem.receivedHeaderMessageCounter = msgHeader.messageCounter;
	grem.responseCode = responseCode;

	MessageHeaderType header;
	auto nBytesWritten = encodeGREMMessage(this->populateMessageHeader(&header), &grem, buffer.data(), buffer.size(), debug);

	if (nBytesWritten < 0) { throw(std::invalid_argument("Failed to encode GREM data"));}
	ctrlChannel.send(buffer, static_cast<size_t>(nBytesWritten));
}

void TestObject::sendMonrLoop() {
	std::stringstream ss;
	ss << transmitterID << " has started MONR thread." << std::endl;
	std::cout << ss.str();

	while (this->on && ctrlChannel.isOpen()) {
		// Only send monr if transmitterID has been set by an OSEM message
		if (osemReceived){
			sendMONR();
		}
		auto t = std::chrono::steady_clock::now();
		std::this_thread::sleep_until(t + monrPeriod);
	}
	ss.str(std::string());
	ss << transmitterID << " is exiting MONR thread." << std::endl;
	std::cout << ss.str();

}

void TestObject::receiveUDP() {
	if (socketsReceivedFromController) {
		return;
	}
	std::stringstream ss;
	ss << "Started UDP communication thread." << std::endl;
	ss << "Awaiting UDP data from ATOS..." << std::endl;
	std::cout << ss.str();

	awaitingFirstHeab = true;

	while (this->on && ctrlChannel.isOpen()) {
		auto data = processChannel.receive();

		// Connection lost
		if (data.size() <= 0) {
			continue;
		}

		if (awaitingFirstHeab) {
			ss.str(std::string());
			ss << "Received UDP data from ATOS" << std::endl;
			std::cout << ss.str();
			startSendMonr(); // UDP connection established, start sending MONR
		}

		int nBytesHandled = 0;
		do {
			try {
				nBytesHandled = handleMessage(data);
			} catch (const std::exception& e) {
				std::cerr << e.what() << std::endl;
				break;
			}
			data.erase(data.begin(), data.begin() + nBytesHandled);
		} while (data.size() > 0);
	}

	ss.str(std::string());
	ss << "Exiting UDP communication thread." << std::endl;
	std::cout << ss.str();

}

void TestObject::checkHeabTimeout() {
	using namespace std::chrono;
	std::scoped_lock lock(heabMutex);
	// Check time difference of received HEAB and last HEAB
	auto timeSinceHeab = steady_clock::now() - lastHeabTime;
	if (!awaitingFirstHeab && timeSinceHeab > heartbeatTimeout) {
		std::stringstream ss;
		ss << "Heartbeat timeout: " << duration_cast<milliseconds>(timeSinceHeab).count()
		   << " ms since last heartbeat exceeds limit of " << heartbeatTimeout.count() << " ms." << std::endl;
		std::cerr << ss.str();
		heabTimeout();
	}
}

void TestObject::checkHeabLoop() {
	std::stringstream ss;
	ss << "Started HEAB timeout thread." << std::endl;
	std::cout << ss.str();

	using namespace std::chrono;
	while (this->on) {
		auto t = std::chrono::steady_clock::now();
		checkHeabTimeout();
		// Don't lock the mutex all the time
		std::this_thread::sleep_until(t + expectedHeartbeatPeriod);
	}
	ss.str(std::string());
	ss << "Exiting HEAB timeout thread." << std::endl;
	std::cout << ss.str();
}

void TestObject::onHeabTimeout() {
	disconnect();
	this->state->handleEvent(*this, Events::L);
}

int TestObject::handleTCPMessage(char *buffer, int bufferLen) {
	state->handleEvent(*this, ISO22133::Events::B);

	std::vector<char> data;
	for (int i = 0; i < bufferLen; i++) {
		data.push_back(buffer[i]);
	}

	int nBytesHandled = handleMessage(data);

	this->startHandleTCP();
	return nBytesHandled;
}

int TestObject::handleUDPMessage(char *buffer, int bufferLen, int udpSocket, char *addr, const uint32_t port) {
	if (awaitingFirstHeab) {
		std::stringstream ss;
		ss.str(std::string());
		ss << "Received UDP data from ATOS" << std::endl;
		std::cout << ss.str();
		boost::asio::ip::udp::endpoint udpEp = boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(addr), port);
		processChannel.setEndpoint(udpSocket, udpEp);
		startSendMonr(); // UDP connection established, start sending MONR
	}
	std::vector<char> data;
	for (int i = 0; i < bufferLen; i++) {
		data.push_back(buffer[i]);
	}
	return handleMessage(data);
}

int TestObject::handleMessage(std::vector<char>& dataBuffer) {
	std::lock_guard<std::mutex> lock(this->recvMutex);	// Both TCP and UDP threads end up in here
	int bytesHandled = 0;
	int debug = 0;
	struct timeval currentTime;

	currentTime = std::chrono::to_timeval(std::chrono::system_clock::now().time_since_epoch());


	HeaderType msgHeader;
	enum ISOMessageReturnValue retval = decodeISOHeader(dataBuffer.data(), dataBuffer.size(), &msgHeader, debug);
	if (retval == MESSAGE_OK) {
		lastReceivedMsgHeader = msgHeader;
	}
	// Ugly check here since we don't know if it is UDP or the rest of TRAJ
	if (retval != MESSAGE_OK && this->trajDecoder.ExpectingTrajPoints()) {
		msgHeader.messageID = MESSAGE_ID_TRAJ;
	}

	this->checkAndUpdateMessageCounter(msgHeader.messageCounter);
	switch (msgHeader.messageID) {
	case MESSAGE_ID_TRAJ:
		bytesHandled = this->trajDecoder.DecodeTRAJ(dataBuffer);
		if (bytesHandled < 0) {
			throw std::invalid_argument("Error decoding TRAJ");
		}
		if (!this->trajDecoder.ExpectingTrajPoints()) {
			this->state->handleTRAJ(*this, lastReceivedMsgHeader);
		}
		break;
	case MESSAGE_ID_OSEM:
		ObjectSettingsType OSEMstruct;
		bytesHandled = decodeOSEMMessage(&OSEMstruct, dataBuffer.data(), dataBuffer.size(), debug);
		if (bytesHandled < 0) {
			throw std::invalid_argument("Error decoding OSEM");
		}
		std::cout << "Received OSEM \n";
		this->osemReceived = true;
		this->state->handleOSEM(*this, OSEMstruct);
		break;

	case MESSAGE_ID_OSTM:
		ObjectCommandType OSTMdata;
		bytesHandled = decodeOSTMMessage(dataBuffer.data(), dataBuffer.size(), &OSTMdata, debug);
		if (bytesHandled < 0) {
			throw std::invalid_argument("Error decoding OSTM");
		}
		this->state->handleOSTM(*this, OSTMdata);
		break;

	case MESSAGE_ID_STRT:
		StartMessageType STRTdata;
		bytesHandled
			= decodeSTRTMessage(dataBuffer.data(), dataBuffer.size(), &currentTime, &STRTdata, debug);
		if (bytesHandled < 0) {
			throw std::invalid_argument("Error decoding STRT");
		}
		this->state->handleSTRT(*this, STRTdata);
		break;

	case MESSAGE_ID_HEAB:
		HeabMessageDataType HEABdata;
		bytesHandled = decodeHEABMessage(dataBuffer.data(), dataBuffer.size(), currentTime, &HEABdata, debug);
		if (bytesHandled < 0) {
			throw std::invalid_argument("Error decoding HEAB");
		}
		this->handleHEAB(HEABdata);
		break;
	default:
		bytesHandled = handleVendorSpecificMessage(msgHeader.messageID, dataBuffer);
		if (bytesHandled < 0) {
			throw std::invalid_argument(std::string("Unable to decode ISO-22133 message with MsgID ")
										+ std::to_string(msgHeader.messageID));
		}
		bytesHandled = static_cast<int>(dataBuffer.size());
		break;
	}
	return bytesHandled;
}

/**
 * @brief Generates state changes based on control center status
 * @param heab struct HeabMessageDataType
 */
void TestObject::handleHEAB(HeabMessageDataType& heab) {
	using namespace std::chrono;
	// Order matters here, below may change state
	// causing the signal to not be triggered if placed
	// after the handleEvent() calls
	heabSig(heab);

	// Check network delay: difference between
	// timestamp in HEAB and local time
	// Requires the system clocks of ATOS 
	// and object to be synced!!
	auto heabTime = seconds(heab.dataTimestamp.tv_sec) + microseconds(heab.dataTimestamp.tv_usec);
	auto networkDelay = system_clock::now().time_since_epoch() - heabTime;
	setNetworkDelay(duration_cast<milliseconds>(networkDelay));

	if (networkDelay > maxSafeNetworkDelay) {
		std::stringstream ss;
		ss << "Network delay of " << duration_cast<milliseconds>(networkDelay).count()
		   << " ms exceeds safe limit of " << maxSafeNetworkDelay.count() << " ms." << std::endl;
		std::cerr << ss.str();
		// TODO: do something
	}
	std::scoped_lock lock(heabMutex);
	lastHeabTime = steady_clock::now();
	awaitingFirstHeab = false;

	switch (heab.controlCenterStatus) {
	case CONTROL_CENTER_STATUS_NORMAL_STOP:
		this->state->handleEvent(*this, ISO22133::Events::U);
		break;
	case CONTROL_CENTER_STATUS_ABORT:
		this->state->handleEvent(*this, ISO22133::Events::W);
		break;
	case CONTROL_CENTER_STATUS_TEST_DONE:
		this->state->handleEvent(*this, ISO22133::Events::Y);
		break;
	default:
		break;
	}
	ccStatus = heab.controlCenterStatus;
	return;
}

std::chrono::milliseconds TestObject::getNetworkDelay() {
	std::scoped_lock lock(netwrkDelayMutex);
	if (awaitingFirstHeab) {
		return std::chrono::milliseconds(0);
	}
	return estimatedNetworkDelay;
}

void TestObject::setNetworkDelay(std::chrono::milliseconds delay) {
	std::scoped_lock lock(netwrkDelayMutex);
	estimatedNetworkDelay = delay;
}

void TestObject::checkAndUpdateMessageCounter(const char receivedMessageCounter) {
	if (expectedMessageCounter != receivedMessageCounter) {
		std::stringstream ss;
		ss << "Received message counter " << receivedMessageCounter << " does not match expected "
		<< expectedMessageCounter << std::endl;
		std::cerr << ss.str();
	}
	expectedMessageCounter = receivedMessageCounter + 1;
}

}  // namespace ISO22133
