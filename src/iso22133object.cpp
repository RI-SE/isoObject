#include <chrono>
#include <thread>


#include "iso22133object.hpp"
#include "iso22133state.hpp"
#include "iso22133.h"
#include "maestroTime.h"

#define ALLOWED_HEAB_DIFF_MS 50 // Time before aborting due to missing HEAB
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
		this->firstHeab = true;
		try {
			this->state->handleEvent(*this, ISO22133::Events::L);
		}
		catch(const std::runtime_error& e) {
			std::cerr << e.what() << '\n';
		}		
		this->controlChannel.TCPHandlerclose();
		this->udpReceiveThread.join();
	}
}

void TestObject::receiveUDP(){
	this->processChannel.CreateServer(ISO_22133_OBJECT_UDP_PORT,"",0);
	std::vector<char> UDPReceiveBuffer(UDP_BUFFER_SIZE);
	while(this->processChannel.receiveUDP(UDPReceiveBuffer) < 0){ // Would prefer blocking behavior on UDPhandler..
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
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
}

void TestObject::sendMONR(bool debug) {
	if(!this->udpOk) {
		std::cout << "UDP communication not set up yet. Can't send MONR" << std::endl;
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

int TestObject::handleMessage(std::vector<char>* dataBuffer) {
	std::lock_guard<std::mutex> lock(this->recvMutex); // Both TCP and UDP threads end up in here
	int bytesHandled = 0;
	int debug = 0;
	static bool expectingTRAJPoints = false;
	struct timeval currentTime;
	TimeSetToCurrentSystemTime(&currentTime);
	std::vector<char> copiedData; // Needed for TRAJ

	ISOMessageID msgType = getISOMessageType(dataBuffer->data(), dataBuffer->size(), 0);
	if(msgType == MESSAGE_ID_INVALID && expectingTRAJPoints) {
		msgType = MESSAGE_ID_TRAJ;
	}

	switch(msgType) {
		case MESSAGE_ID_TRAJ:
			std::cout << "Receiving TRAJ" << std::endl;
			static int nPointsHandled = 0;
			static std::vector<char> unhandledBytes; 
			copiedData = *dataBuffer;

			// Decode TRAJ Header
			if(!expectingTRAJPoints) {
				bytesHandled = decodeTRAJMessageHeader(&this->trajectoryHeader, copiedData.data(), copiedData.size(), 1);
				if(bytesHandled < 0) {
					throw std::invalid_argument("Error decoding TRAJ Header");	
				}
				copiedData.erase(copiedData.begin(), copiedData.begin()+bytesHandled);	
				expectingTRAJPoints = true;
				this->trajectory.resize(this->trajectoryHeader.wayPoints);
			}

			// Decode TRAJ waypoints
			int tmpByteCounter;
			TrajectorWaypointType waypoint;
			if(expectingTRAJPoints) {
				std::cout << "inserting " << unhandledBytes.size() << " unhandled bytes " << std::endl;
				copiedData.insert(copiedData.begin(), unhandledBytes.begin(), unhandledBytes.end());
			}

			for(int i = 0; i < this->trajectoryHeader.wayPoints - nPointsHandled; i++) {
				if(copiedData.size() < ISO_TRAJ_WAYPOINT_SIZE) { // Save the bytes remaining and return
					unhandledBytes.resize(copiedData.size());
					unhandledBytes = copiedData; 	
					std::cout << "saving remaing bytes: " << unhandledBytes.size() << std::endl;
					std::cout << "copied buffer size " << copiedData.size() << std::endl;
					break;
				}

				tmpByteCounter = decodeTRAJMessagePoint(&waypoint, copiedData.data(), 1);
				if(tmpByteCounter < 0) {
					std::cout << "\nbyteshandled " << bytesHandled << std::endl;
					std::cout << "buffer size " << dataBuffer->size() << std::endl;
					std::cout << "copied buffer size " << copiedData.size() << std::endl;
					throw std::invalid_argument("Error decoding TRAJ Waypoint");
				}
				copiedData.erase(copiedData.begin(), copiedData.begin()+tmpByteCounter);	
				std::cout << "buffer size after erase: " << copiedData.size() << std::endl;	
				this->trajectory[i+nPointsHandled] = waypoint;
				bytesHandled += tmpByteCounter;
				nPointsHandled += 1;
			}	

			if(nPointsHandled == this->trajectoryHeader.wayPoints) {
				expectingTRAJPoints = false; // Complete TRAJ received
			}

			std::cout << "nPointsHandled " << nPointsHandled << std::endl;
			std::cout << "nPointsExpected " << this->trajectoryHeader.wayPoints << std::endl;
			bytesHandled = static_cast<int>(dataBuffer->size());
			break;

		case MESSAGE_ID_OSEM:
			ObjectSettingsType OSEMstruct;
			bytesHandled = decodeOSEMMessage(&OSEMstruct,dataBuffer->data(),dataBuffer->size(),nullptr,debug);
			if(bytesHandled < 0) {
				throw std::invalid_argument("Error decoding OSEM");
			}		
			std::cout << "Received OSEM " << std::endl;
			this->state->handleOSEM(*this, OSEMstruct);
			break;

		case MESSAGE_ID_OSTM:
			ObjectCommandType OSTMdata;
			bytesHandled = decodeOSTMMessage(dataBuffer->data(),dataBuffer->size(),&OSTMdata,debug);
			if(bytesHandled < 0) {
				throw std::invalid_argument("Error decoding OSTM");
			}
			this->state->handleOSTM(*this, OSTMdata);
			break;

		case MESSAGE_ID_STRT:
			StartMessageType STRTdata;
			bytesHandled = decodeSTRTMessage(dataBuffer->data(),dataBuffer->size(),&currentTime,&STRTdata,debug);
			if(bytesHandled < 0) {
				throw std::invalid_argument("Error decoding STRT");
			}
			this->state->handleSTRT(*this, STRTdata);
			break;

		case MESSAGE_ID_HEAB:
			HeabMessageDataType HEABdata;
			static struct timeval lastHeabTime; 
	
			bytesHandled = decodeHEABMessage(dataBuffer->data(),dataBuffer->size(),currentTime,&HEABdata,debug);
			if(bytesHandled < 0) {
				throw std::invalid_argument("Error decoding HEAB");
			}
			this->ccStatus = HEABdata.controlCenterStatus;

			if(!this->firstHeab && TimeGetTimeDifferenceMS(&currentTime, &lastHeabTime) > ALLOWED_HEAB_DIFF_MS) {
				std::cerr << "Did not recevie HEAB in time, differance is " << TimeGetTimeDifferenceMS(&currentTime, &lastHeabTime) << " ms" << std::endl;
				this->handleAbort();
				HEABdata.controlCenterStatus = CONTROL_CENTER_STATUS_ABORT;
			}
			
			this->state->handleHEAB(*this, HEABdata);

			lastHeabTime = HEABdata.dataTimestamp;
			this->firstHeab = false;
			break;

		default:
			bytesHandled = static_cast<int>(dataBuffer->size());
			break;
	}

	return bytesHandled;
}
} //namespace ISO22133
