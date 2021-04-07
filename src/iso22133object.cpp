#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

#include "iso22133object.hpp"
#include "iso22133state.hpp"
#include "iso22133.h"
#include "maestroTime.h"


#define TCP_BUFFER_SIZE 1024
#define UDP_BUFFER_SIZE 1024
#define ERROR -1


namespace ISO22133 {
void TestObject::receiveTCP(){
	while(this->on_) {
		std::cout << "Awaiting connection to server..." << std::endl;
		if(this->controlChannel_.CreateServer(53241, "") < 0) {
			continue;
		}
		std::cout << "Connected to server..." << std::endl;

		this->state->handleEvent(*this, ISO22133::Events::B);
		this->startHandleUDP();

		std::vector<char> TCPReceiveBuffer(TCP_BUFFER_SIZE);
		int nBytesReceived, nBytesHandled;
		while(this->isServerConnected()){
			std::fill(TCPReceiveBuffer.begin(), TCPReceiveBuffer.end(), 0);
			nBytesReceived = this->controlChannel_.receiveTCP(TCPReceiveBuffer,0);
			
			if (nBytesReceived > 0) {
				TCPReceiveBuffer.resize(static_cast<size_t>(nBytesReceived));
				do{
					nBytesHandled = this->handleMessage(&TCPReceiveBuffer);
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
		this->udpOk_ = false;
		this->state->handleEvent(*this, ISO22133::Events::L);
		this->controlChannel_.TCPHandlerclose();
		this->udpReceiveThread_.join();
	}
}

void TestObject::receiveUDP(){
	this->processChannel_.CreateServer(processChannelPort_,"",0);
	std::vector<char> UDPReceiveBuffer(UDP_BUFFER_SIZE);
	while(this->processChannel_.receiveUDP(UDPReceiveBuffer) < 0){ // Would prefer blocking behavior on UDPhandler..
		usleep(10000);
	}
	this->udpOk_ = true;
	int nBytesReceived, nBytesHandled;

	while(this->isServerConnected() && this->udpOk_){
		std::fill(UDPReceiveBuffer.begin(), UDPReceiveBuffer.end(), 0);
		nBytesReceived = this->processChannel_.receiveUDP(UDPReceiveBuffer);
		if (nBytesReceived > 0) {
			UDPReceiveBuffer.resize(static_cast<size_t>(nBytesReceived));
			do{
				nBytesHandled = this->handleMessage(&UDPReceiveBuffer);
				nBytesReceived -= nBytesHandled;
				UDPReceiveBuffer.erase(UDPReceiveBuffer.begin(), UDPReceiveBuffer.begin() + nBytesHandled);
			}
			while (nBytesReceived > 0);
		}
		else if (nBytesReceived < 0) {
			break;
		}
		UDPReceiveBuffer.resize(UDP_BUFFER_SIZE);
		usleep(1000); // Would prefer blocking behavior on UDPhandler..
	}
	this->udpOk_ = false;
	this->processChannel_.UDPHandlerclose();
}

void TestObject::sendMONR(char debug = 0){
	if(!this->udpOk_) {
		std::cout << "UDP communication not set up yet. Can't send MONR" << std::endl;
		return;
	}
	std::vector<char> buffer(UDP_BUFFER_SIZE);
	struct timeval time;

	TimeSetToCurrentSystemTime(&time);
	encodeMONRMessage(&time, this->position_, this->speed_, this->acceleration_,
					this->driveDirection_, this->state->getStateID(), this->readyToArm_,
					this->errorState_, buffer.data(), buffer.size(),debug);
	this->processChannel_.sendUDP(buffer);
}

int TestObject::handleMessage(std::vector<char>* dataBuffer){
	std::lock_guard<std::mutex> lock(this->recvMutex_); // Both TCP and UDP threads end up in here
	int bytesHandled = 0;
	int debug = 0;

	switch(getISOMessageType(dataBuffer->data(), dataBuffer->size(), 0)){
		case MESSAGE_ID_TRAJ:
			//TODO. Needs TRAJ deoder
			std::cout << "Discarding TRAJ" << std::endl;
			bytesHandled = static_cast<int>(dataBuffer->size());
			break;

		case MESSAGE_ID_OSEM:
			ObjectSettingsType OSEMstruct;
			bytesHandled = decodeOSEMMessage(&OSEMstruct,dataBuffer->data(),dataBuffer->size(),nullptr,debug);
			if(bytesHandled < 0){
				std::cerr << "Error decoding OSEM" << std::endl;
				return ERROR;
			}		
			std::cout << "Received OSEM " << std::endl;
			this->state->_handleOSEM(*this, OSEMstruct);
			break;

		case MESSAGE_ID_OSTM:
			ObjectCommandType OSTMdata;
			bytesHandled = decodeOSTMMessage(dataBuffer->data(),dataBuffer->size(),&OSTMdata,debug);
			if(bytesHandled < 0){
				std::cerr << "Error decoding OSTM" << std::endl;
				return ERROR;
			}
			this->state->_handleOSTM(*this, OSTMdata);
			break;

		case MESSAGE_ID_STRT:
			//TODO. Needs STRT decoder
			bytesHandled = static_cast<int>(dataBuffer->size());
			break;

		case MESSAGE_ID_HEAB:
			HeabMessageDataType HEABdata;
			struct timeval currentTime;

			TimeSetToCurrentSystemTime(&currentTime);
			bytesHandled = decodeHEABMessage(dataBuffer->data(),dataBuffer->size(),currentTime,&HEABdata,debug);
			if(bytesHandled < 0){
				std::cerr << "Error decoding HEAB" << std::endl;
				return ERROR;
			}
			this->ccStatus_ = HEABdata.controlCenterStatus;

			if(HEABdata.controlCenterStatus == CONTROL_CENTER_STATUS_ABORT) {
				this->handleAbort();
			}
			this->state->_handleHEAB(*this, HEABdata);
			
			break;

		default:
			bytesHandled = static_cast<int>(dataBuffer->size());
			break;
	}

	return bytesHandled;
}
}
