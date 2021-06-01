#include <chrono>
#include <thread>

#include "iso22133object.hpp"
#include "iso22133state.hpp"
#include "iso22133.h"
#include "maestroTime.h"

#define TCP_BUFFER_SIZE 1024
#define UDP_BUFFER_SIZE 1024

namespace ISO22133 {
void TestObject::receiveTCP() {
	while(this->on) {
		std::cout << "Awaiting connection to server..." << std::endl;
		if(this->controlChannel.CreateServer(ISO_22133_DEFAULT_OBJECT_TCP_PORT, "") < 0) {
			continue;
		}
		std::cout << "Connected to server..." << std::endl;

		try {
			this->state->handleEvent(*this, ISO22133::Events::B);
		}
		catch(const std::runtime_error& e) {
			std::cerr << e.what() << '\n';
		}
		this->startHandleUDP();

		std::vector<char> TCPReceiveBuffer(TCP_BUFFER_SIZE);
		int nBytesReceived, nBytesHandled;
		while(this->isServerConnected()) {
			std::fill(TCPReceiveBuffer.begin(), TCPReceiveBuffer.end(), 0);
			nBytesReceived = this->controlChannel.receiveTCP(TCPReceiveBuffer,0);
			
			if (nBytesReceived > 0) {
				TCPReceiveBuffer.resize(static_cast<size_t>(nBytesReceived));
				do {
					try {
						nBytesHandled = this->handleMessage(&TCPReceiveBuffer);
					}
					catch(const std::exception& e) {
						std::cerr << e.what() << '\n';
						nBytesHandled = nBytesReceived;
					}
					nBytesReceived -= nBytesHandled;
					TCPReceiveBuffer.erase(TCPReceiveBuffer.begin(), TCPReceiveBuffer.begin() + nBytesHandled);
				}
				while(nBytesReceived > 0);
			}
			else if (nBytesReceived < 0) {
				break;
			}
			TCPReceiveBuffer.resize(TCP_BUFFER_SIZE);
		}
		std::cout << "Connection to control center lost" << std::endl;
		this->udpOk = false;
		this->setFirstHeab(true);
		try {
			this->state->handleEvent(*this, ISO22133::Events::L);
		}
		catch(const std::runtime_error& e) {
			std::cerr << e.what() << '\n';
		}		
		this->controlChannel.TCPHandlerclose();
		try {
			this->udpReceiveThread.join();
		}
		catch (const std::system_error& e) {
			std::cerr << e.what() << '\n';
		}		
	}
}

void TestObject::receiveUDP(){
	this->processChannel.CreateServer(ISO_22133_OBJECT_UDP_PORT,"",0);
	std::vector<char> UDPReceiveBuffer(UDP_BUFFER_SIZE);
	
	this->startSendMONR();

	while(this->processChannel.receiveUDP(UDPReceiveBuffer) < 0){ // Would prefer blocking behavior on UDPhandler..
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
	}
	this->udpOk = true;
	int nBytesReceived, nBytesHandled;

	

	while(this->isServerConnected() && this->udpOk) {
		std::fill(UDPReceiveBuffer.begin(), UDPReceiveBuffer.end(), 0);
		nBytesReceived = this->processChannel.receiveUDP(UDPReceiveBuffer);
		if (nBytesReceived > 0) {
			UDPReceiveBuffer.resize(static_cast<size_t>(nBytesReceived));
			do {
				try {
					nBytesHandled = this->handleMessage(&UDPReceiveBuffer);
					nBytesReceived -= nBytesHandled;
					UDPReceiveBuffer.erase(UDPReceiveBuffer.begin(), UDPReceiveBuffer.begin() + nBytesHandled);
				}
				catch(const std::exception& e) {
					std::cerr << e.what() << '\n';
					nBytesHandled = nBytesReceived;
				}
			}
			while (nBytesReceived > 0);
		}
		else if (nBytesReceived < 0) {
			break;
		}
		UDPReceiveBuffer.resize(UDP_BUFFER_SIZE);
		std::this_thread::sleep_for(std::chrono::milliseconds(5)); // Would prefer blocking behavior on UDPhandler..
	}
	this->udpOk = false;
	this->processChannel.UDPHandlerclose();
	try {
		this->monrThread.join();
	}
	catch (const std::system_error& e) {
		std::cerr << e.what() << '\n';
	}
}

void TestObject::sendMONR(bool debug) {
	if(!this->udpOk) {
		std::cout << "UDP communication not set up. Can't send MONR" << std::endl;
		return;
	}
	std::vector<char> buffer(UDP_BUFFER_SIZE);
	struct timeval time;

	TimeSetToCurrentSystemTime(&time);
	encodeMONRMessage(&time, this->position, this->speed, this->acceleration,
					this->driveDirection, this->state->getStateID(), this->readyToArm,
					this->errorState, buffer.data(), buffer.size(),debug);
	this->processChannel.sendUDP(buffer);
}

void TestObject::monrLoop() {
	while(!this->udpOk) {
		// Wait for UDP connection
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
	}
	std::chrono::time_point<std::chrono::system_clock> monrTime;
	while(this->isServerConnected() && this->udpOk) {
		monrTime = std::chrono::system_clock::now();
		this->sendMONR();
		std::this_thread::sleep_for(
			std::chrono::milliseconds(10) - 
			monrTime.time_since_epoch() +
			std::chrono::system_clock::now().time_since_epoch()
			);
	}
}

int TestObject::handleMessage(std::vector<char>* dataBuffer) {
	std::lock_guard<std::mutex> lock(this->recvMutex); // Both TCP and UDP threads end up in here
	int bytesHandled = 0;
	int debug = 0;
	struct timeval currentTime;
	TimeSetToCurrentSystemTime(&currentTime);

	ISOMessageID msgType = getISOMessageType(dataBuffer->data(), dataBuffer->size(), 0);
	// Ugly check here since we don't know if it is UDP or the rest of TRAJ
	if(msgType == MESSAGE_ID_INVALID && this->trajDecoder.ExpectingTrajPoints()) {
		msgType = MESSAGE_ID_TRAJ;
	}

	switch(msgType) {
		case MESSAGE_ID_TRAJ:
			std::cout << "Receiving TRAJ" << std::endl;
			bytesHandled = this->trajDecoder.DecodeTRAJ(dataBuffer);
			if(bytesHandled < 0) {
				throw std::invalid_argument("Error decoding TRAJ");
			};
			this->state->handleTRAJ(*this);
			break;

		case MESSAGE_ID_OSEM:
			ObjectSettingsType OSEMstruct;
			bytesHandled = decodeOSEMMessage(
				&OSEMstruct,
				dataBuffer->data(),
				dataBuffer->size(),
				nullptr,
				debug);
			if(bytesHandled < 0) {
				throw std::invalid_argument("Error decoding OSEM");
			}		
			std::cout << "Received OSEM " << std::endl;
			this->state->handleOSEM(*this, OSEMstruct);
			break;

		case MESSAGE_ID_OSTM:
			ObjectCommandType OSTMdata;
			bytesHandled = decodeOSTMMessage(
				dataBuffer->data(),
				dataBuffer->size(),
				&OSTMdata,
				debug);
			if(bytesHandled < 0) {
				throw std::invalid_argument("Error decoding OSTM");
			}
			this->state->handleOSTM(*this, OSTMdata);
			break;

		case MESSAGE_ID_STRT:
			StartMessageType STRTdata;
			bytesHandled = decodeSTRTMessage(
				dataBuffer->data(),
				dataBuffer->size(),
				&currentTime,
				&STRTdata,
				debug);
			if(bytesHandled < 0) {
				throw std::invalid_argument("Error decoding STRT");
			}
			this->state->handleSTRT(*this, STRTdata);
			break;

		case MESSAGE_ID_HEAB:
			HeabMessageDataType HEABdata;
	
			bytesHandled = decodeHEABMessage(
				dataBuffer->data(),
				dataBuffer->size(),
				currentTime,
				&HEABdata,
				debug);
			if(bytesHandled < 0) {
				throw std::invalid_argument("Error decoding HEAB");
			}
			this->ccStatus = HEABdata.controlCenterStatus;			
			this->state->handleHEAB(*this, HEABdata);
			this->lastHeabTime = currentTime;
			this->setFirstHeab(false);
			break;

		default:
			bytesHandled = static_cast<int>(dataBuffer->size());
			break;
	}

	return bytesHandled;
}

void TestObject::initializeValues() {
	this->position.isHeadingValid = false;
	this->position.isPositionValid = false;
	this->speed.isLateralValid = false;
	this->speed.isLongitudinalValid = false;
	this->acceleration.isLateralValid = false;
	this->acceleration.isLongitudinalValid = false;
}

void TestObject::checkHeabTimeout() {
	while(this->on) {
		if(!this->checkFirstHeab()) {
			struct timeval currentTime;
			TimeSetToCurrentSystemTime(&currentTime);
			uint64_t timeDiff = TimeGetTimeDifferenceMS(&currentTime, &this->lastHeabTime);
			if(timeDiff >= maxAllowedHeabTimeout_ms) {
				std::cerr << "Did not recevie HEAB in time, differance is " << 
				timeDiff << " ms" << std::endl;
				this->setFirstHeab(true);
				this->heabTimeout();
			}
			else {
				uint32_t sleepPeriod = maxAllowedHeabTimeout_ms - timeDiff;
				std::this_thread::sleep_for(std::chrono::milliseconds(sleepPeriod));
			}
		}
	}
}

bool TestObject::checkFirstHeab() {
	std::lock_guard<std::mutex> lock(this->heabGuard); 
	return this->firstHeab;
}

bool TestObject::setFirstHeab(bool first) {
	std::lock_guard<std::mutex> lock(this->heabGuard); 
	this->firstHeab = first;
}

} //namespace ISO22133
