#include <chrono>
#include <thread>

#include "iso22133object.hpp"
#include "iso22133state.hpp"
#include "iso22133.h"
#include "maestroTime.h"

#define TCP_BUFFER_SIZE 1024
#define UDP_BUFFER_SIZE 1024

namespace ISO22133 {
TestObject::TestObject(const std::string& listenIP) : name("myTestObject"),
							controlChannel(), 
							processChannel(), 
							trajDecoder() {
	CartesianPosition initPos;
	SpeedType initSpd;
	AccelerationType initAcc;
	std::cout << "Listen IP: " << listenIP << std::endl;
	localIP = listenIP;
	initPos.isHeadingValid = false;
	initPos.isPositionValid = false;
	initSpd.isLateralValid = false;
	initSpd.isLongitudinalValid = false;
	initAcc.isLateralValid = false;
	initAcc.isLongitudinalValid = false;
	this->setPosition(initPos);
	this->setSpeed(initSpd);
	this->setAcceleration(initAcc);
	this->state = this->createInit();
	this->startHandleTCP();
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
		monrThread.join();
	} catch (std::system_error) {}
	try {
		tcpReceiveThread.join(); // This blocks forever when the receive thread has not finished accept()
	} catch (std::system_error) {}
	try {
		udpReceiveThread.join();
	} catch (std::system_error) {}
	try {
		heabTimeoutThread.join();
	} catch (std::system_error) {}
}; 

void TestObject::receiveTCP() {
	while(this->on) {
		std::cout << "Started TCP thread." << std::endl;
		std::cout << "Awaiting TCP connection to server..." << std::endl;
		if(this->controlChannel.CreateServer(ISO_22133_DEFAULT_OBJECT_TCP_PORT, localIP) < 0) {
			continue;
		}
		std::cout << "TCP Connected to server..." << std::endl;

		try {
			this->state->handleEvent(*this, ISO22133::Events::B);
		}
		catch(const std::runtime_error& e) {
			std::cerr << e.what() << '\n';
		}
		this->startHandleUDP();
		this->startHEABCheck();

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
		this->firstHeab = true;
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
	std::cout << "Exiting TCP thread." << std::endl;
}

void TestObject::receiveUDP(){
	std::cout << "Started UDP thread." << std::endl;
	this->processChannel.CreateServer(ISO_22133_OBJECT_UDP_PORT,localIP,0);
	std::vector<char> UDPReceiveBuffer(UDP_BUFFER_SIZE);
	
	this->startSendMONR();

	while(this->processChannel.receiveUDP(UDPReceiveBuffer) < 0){ // Would prefer blocking behavior on UDPhandler..
		std::this_thread::sleep_for(expectedHeartbeatPeriod / 2);
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
		std::this_thread::sleep_for(expectedHeartbeatPeriod / 2); // Would prefer blocking behavior on UDPhandler..
	}
	this->udpOk = false;
	this->processChannel.UDPHandlerclose();
	try {
		this->monrThread.join();
	}
	catch (const std::system_error& e) {
		std::cerr << e.what() << '\n';
	}
	std::cout << "Exiting UDP thread." << std::endl;
}

void TestObject::sendMONR(bool debug) {
	if(!this->udpOk) {
		std::cout << "UDP communication not set up. Can't send MONR" << std::endl;
		return;
	}
	std::vector<char> buffer(UDP_BUFFER_SIZE);
	struct timeval time;

	TimeSetToCurrentSystemTime(&time);
	auto result = encodeMONRMessage(&time, this->position, this->speed, this->acceleration,
					this->driveDirection, this->state->getStateID(), this->readyToArm,
					this->errorState, buffer.data(), buffer.size(),debug);
	if (result < 0) {
		std::cout << "Failed to encode MONR data" << std::endl;
	}
	else {
		buffer.resize(static_cast<size_t>(result));
		this->processChannel.sendUDP(buffer);
	}
}

void TestObject::monrLoop() {
	std::cout << "Started MONR thread." << std::endl;
	while(!this->udpOk) {
		// Wait for UDP connection
		std::this_thread::sleep_for(monrPeriod / 2);
	}
	std::chrono::time_point<std::chrono::system_clock> monrTime;
	while(this->isServerConnected() && this->udpOk) {
		monrTime = std::chrono::system_clock::now();
		this->sendMONR();
		std::this_thread::sleep_for(
			monrPeriod -
			monrTime.time_since_epoch() +
			std::chrono::system_clock::now().time_since_epoch()
			);
	}
	std::cout << "Exiting MONR thread." << std::endl;
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
			try {
				bytesHandled = this->trajDecoder.DecodeTRAJ(dataBuffer);
			}
			catch(const std::exception& e) {
				throw e;
			}
			if(bytesHandled < 0) {
				throw std::invalid_argument("Error decoding TRAJ");
			};
			if (!this->trajDecoder.ExpectingTrajPoints()) {
				this->state->handleTRAJ(*this);
			}
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
			this->firstHeab = false;
			break;

		default:
			bytesHandled = static_cast<int>(dataBuffer->size());
			break;
	}

	return bytesHandled;
}

void TestObject::checkHeabTimeout() {
	std::cout << "Started HEAB thread." << std::endl;
	while(this->on) {
		if(!this->firstHeab) {
			struct timeval currentTime, lastTime;
			TimeSetToCurrentSystemTime(&currentTime);
			lastTime = this->lastHeabTime;
			auto timeDiff = std::chrono::milliseconds(TimeGetTimeDifferenceMS(&currentTime, &lastTime));
			if(timeDiff >= heartbeatTimeout) {
				std::cerr << "Did not receive HEAB in time, difference is " << 
				timeDiff.count() << " ms" << std::endl;
				this->firstHeab = true;
				this->heabTimeout();
			}
			else {
				auto sleepPeriod = heartbeatTimeout - timeDiff;
				std::this_thread::sleep_for(sleepPeriod);
			}
		}
		// Don't lock the mutex all the time
		std::this_thread::sleep_for(expectedHeartbeatPeriod);
	}
	std::cout << "Exiting HEAB thread." << std::endl;
}

void TestObject::onHeabTimeout() { 
	// If we are alredy in abort, stay there
	if(this->state->getStateID() != ISO_OBJECT_STATE_ABORTING){
		this->state->handleEvent(*this, Events::W);
	}
}
} //namespace ISO22133
