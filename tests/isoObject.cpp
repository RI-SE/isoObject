#include <gtest/gtest.h>
#include <thread>
#include <boost/asio.hpp>
#include <boost/system/system_error.hpp>
#include "iso22133object.hpp"
extern "C" {
	#include "iso22133.h"
}


using namespace ISO22133;
using namespace boost::asio;
using namespace std::chrono_literals;

struct timeval * TimeSetToCurrentSystemTime(struct timeval *time)
{
	// Get system time
	struct timespec currentTime;
	clock_gettime(CLOCK_REALTIME, &currentTime);
	time->tv_sec = currentTime.tv_sec;
	time->tv_usec = currentTime.tv_nsec / 1000;
	return time;
}

class SimulatedTestObject : public TestObject {
	public:
	SimulatedTestObject(int tcpSocket) : TestObject(tcpSocket) {
		this->startHandleTCP();
		state->handleEvent(*this, ISO22133::Events::B);
	}
};

class ControlCenterEmulator
{
	public:
		ControlCenterEmulator(const std::string& ip = "0.0.0.0", int id = 1, int transmitterID = -1, char messCnt = 0) : 
		listenIP(ip),
		tcpSocket(context),
		udpSocket(context, ip::udp::v4()),
		receiverID(id),
		messageCounter(messCnt),
		transmitterID(transmitterID) {
		}
		~ControlCenterEmulator() {

		}
		void connect() {
			try {
				tcpSocket.connect(ip::tcp::endpoint(ip::address_v4::from_string(listenIP), ISO_22133_DEFAULT_OBJECT_TCP_PORT));
			} catch (boost::system::system_error err) {
				EXPECT_EQ(1, 2) << "TCP Connection failed";
			}
		}
		void sendTCP(const std::vector<char>& data) {
			tcpSocket.send(buffer(data));
		}
		void sendUDP(const std::vector<char>& data) {
			ip::udp::endpoint receiverEP(ip::address_v4::from_string(listenIP), ISO_22133_OBJECT_UDP_PORT);
			udpSocket.send_to(buffer(data), receiverEP);
		}
		void disconnect() {
			try {
				tcpSocket.close();
			} catch (boost::system::system_error err) {
				std::cout << "Error closing TCP socket" << std::endl;
			}
			try {
				udpSocket.close();
			} catch (boost::system::system_error err) {
				std::cout << "Error closing UDP socket" << std::endl;
			}
		}
		std::size_t receiveUDP(std::vector<char>& data, ip::udp::endpoint& ep) {
			return udpSocket.receive_from(buffer(data), ep);
		}

		void receiveTCP(std::vector<char>& data) {
			tcpSocket.receive(buffer(data));
		}

		void sendHeartbeat(const ControlCenterStatusType ccStatus) {
			HeabMessageDataType heartbeat;
			TimeSetToCurrentSystemTime(&heartbeat.dataTimestamp);
			heartbeat.controlCenterStatus = ccStatus;
			std::vector<char> transmitBuffer(1024);
			MessageHeaderType header;
			header.receiverID = this->receiverID;
			header.messageCounter = this->messageCounter;
			header.transmitterID = this->transmitterID;
			auto nBytesWritten = encodeHEABMessage(&header, &heartbeat.dataTimestamp, heartbeat.controlCenterStatus,
								transmitBuffer.data(), transmitBuffer.size(), false);
			transmitBuffer.resize(nBytesWritten);
			sendUDP(transmitBuffer);
		}

		void buildOSEM(ObjectSettingsType &objSettings) {
			
			objSettings.desiredID.transmitter = receiverID;
			objSettings.desiredID.controlCentre = transmitterID;
			objSettings.desiredID.subTransmitter = this->transmitterID;

			memset(&objSettings.coordinateSystemOrigin, 0, sizeof(objSettings.coordinateSystemOrigin));
			objSettings.coordinateSystemType = COORDINATE_SYSTEM_WGS84;
			objSettings.coordinateSystemRotation_rad = 0.0;

			TimeSetToCurrentSystemTime(&objSettings.currentTime);
			
			objSettings.heabTimeout.tv_usec = 20000;
			objSettings.heabTimeout.tv_sec = 0;

			objSettings.rate.heab = 10;
			objSettings.rate.monr = 100;
			objSettings.rate.monr2 = 1;

			objSettings.maxDeviation.lateral_m = 5.0;
			objSettings.maxDeviation.position_m = 5.0;
			objSettings.maxDeviation.yaw_rad = 15.0 * M_PI/180.0;

			objSettings.minRequiredPositioningAccuracy_m = 1.0;

			objSettings.timeServer.ip = 0;
			objSettings.timeServer.port = 0;
		}

		void sendOSEM() {
			ObjectSettingsType objSettings;
			buildOSEM(objSettings);
			std::vector<char> transmitBuffer(1024);
			MessageHeaderType header;
			header.receiverID = this->receiverID;
			header.messageCounter = this->messageCounter;
			header.transmitterID = this->transmitterID;
			auto nBytesWritten = encodeOSEMMessage(&header, &objSettings, transmitBuffer.data(), transmitBuffer.size(), false);
			transmitBuffer.resize(nBytesWritten);
			sendTCP(transmitBuffer);
		}

		ip::tcp::socket *getTCPSocket() {
			return &tcpSocket;
		}
		ip::udp::socket *getUDPSocket() {
			return &udpSocket;
		}

		void sendUDPNoop() {
			std::vector<char> noopSend(1);
			sendUDP(noopSend);
		}

	private:
	const std::string listenIP;
	io_context context;
	ip::tcp::socket tcpSocket;
	ip::udp::socket udpSocket;
	const int receiverID;
	char messageCounter;
	const int transmitterID;
};

class ControllerEmulator
{
	public:
		ControllerEmulator(const std::string& listenIP = "127.0.0.1") : 
		acceptor(context, ip::tcp::endpoint(ip::address_v4::from_string(listenIP), ISO_22133_DEFAULT_OBJECT_TCP_PORT)),
		udpSocket(context, ip::udp::endpoint(ip::address_v4::from_string(listenIP), ISO_22133_OBJECT_UDP_PORT)) {}
		virtual ~ControllerEmulator() = default;
		/*
		* Remember to delete the socket after!!
		* Not done here! 
		*/
		ip::tcp::socket *acceptConnection() {
			io_context cont;
			ip::tcp::socket *sock = new ip::tcp::socket(cont);
			acceptor.accept(*sock);
			return sock;
		}

		ip::udp::socket *getUDPSocket() {
			return &udpSocket;
		}

		void disconnect() {
			acceptor.close();
			try {
				udpSocket.shutdown(boost::asio::socket_base::shutdown_both);
			} catch (boost::system::system_error err) { }
		}

		std::size_t receiveUDP(std::vector<char>& data, ip::udp::endpoint& ep) {
			return udpSocket.receive_from(buffer(data), ep);
		}

		void sendUDPNoop(ip::udp::endpoint &ep) {
			udpSocket.send_to(buffer(std::vector<char>(1)), ep);
		}
	
	private:
		io_context context;
		ip::tcp::acceptor acceptor;
		ip::udp::socket udpSocket;
		int timeout;
};

class test_multipleSimulatedISOObjects : public ::testing::Test
{
protected:
	test_multipleSimulatedISOObjects():
	obj1Conn("127.0.0.1", 1, 0xF00F),
	obj2Conn("127.0.0.1", 2, 0xF00F)
	{
		threadListener = std::thread(&test_multipleSimulatedISOObjects::tcpListen, this);
		obj1Conn.connect();
		threadListener.join();
		threadListener = std::thread(&test_multipleSimulatedISOObjects::tcpListen, this);
		obj2Conn.connect();
		threadListener.join();
		obj1 = new SimulatedTestObject(sockets[0]->native_handle());
		obj2 = new SimulatedTestObject(sockets[1]->native_handle());
		CartesianPosition pos;
		pos.xCoord_m = 0.0;
		pos.yCoord_m = 0.0;
		pos.zCoord_m = 0.0;
		pos.isXcoordValid = true;
		pos.isYcoordValid = true;
		pos.isZcoordValid = true;
		pos.isPositionValid = true;
		pos.isHeadingValid = true;
		obj1->setPosition(pos);
		obj2->setPosition(pos);
		SpeedType spd;
		spd.lateral_m_s = 0.0;
		spd.longitudinal_m_s = 0.0;
		spd.isLateralValid = true;
		spd.isLongitudinalValid = true;
		obj1->setSpeed(spd);
		obj2->setSpeed(spd);
		threadListener = std::thread(&test_multipleSimulatedISOObjects::udpReceive, this);
	
	}
	void SetUp() override {}

	void TearDown() override {}
	virtual ~test_multipleSimulatedISOObjects() {
		listenToUDP = false;
		obj1Conn.sendUDPNoop();
		obj1Conn.disconnect();
		delete obj1;

		obj2Conn.sendUDPNoop();
		obj2Conn.disconnect();
		delete obj2;

		try {
			threadListener.join();
		} catch (const std::exception& e) {
			std::cout << "Exception joining thread" << std::endl;
		}

		try {
			listener.disconnect();
		} catch (const std::exception& e) {
			std::cout << "Exception disconnecting listener" << std::endl;
		}
	}


	void sendUDPNoopToClient(int id) {
		listener.sendUDPNoop(udpEndpoints[id]);
	}

	void tcpListen() {
		sockets.push_back(listener.acceptConnection());
	}

	void udpReceive() {
		try {
			while (listenToUDP) {
				int nBytesHandled = 0;
				std::vector<char> data(1024);
				ip::udp::endpoint ep;
				std::size_t num_received = listener.receiveUDP(data, ep);
				if (num_received > 1) {
					data.resize(num_received);
					while (data.size() > 0) {
						try {
							HeaderType HeaderData;
							decodeISOHeader(data.data(), num_received, &HeaderData, false);
							for (int i = 0; i < num_received; i++) {
								printf("0x%02X ", data.at(i));
							}
							SimulatedTestObject *obj = nullptr;
							udpEndpoints[HeaderData.receiverID] = ep;
							if (HeaderData.receiverID == 1) {
								obj = obj1;
							}
							else if (HeaderData.receiverID == 2) {
								obj = obj2;
							}
							if (obj != nullptr) {
								char address[ep.address().to_string().length() +1];
								memset(address, 0, sizeof(address));
								memcpy(address, ep.address().to_string().c_str(), ep.address().to_string().length());
								int handled = obj->handleUDPMessage(data.data(), data.size(), listener.getUDPSocket()->native_handle(), address, ep.port());
								nBytesHandled += handled;
							}
							else {
								EXPECT_NE(obj, nullptr) << "Unknown receiver ID";
								break;
							}
						} catch (const std::exception& e) {
							break;
						}
						data.erase(data.begin(), data.begin() + nBytesHandled);
					} 
				}
			}
		} catch(const std::exception& e) {}
	}

	std::vector<ip::tcp::socket*> sockets;
	std::map<int, ip::udp::endpoint> udpEndpoints;
	ControlCenterEmulator obj1Conn;
	ControlCenterEmulator obj2Conn;
	ControllerEmulator listener;
	SimulatedTestObject *obj1;
	SimulatedTestObject *obj2;
	std::thread threadListener;
	std::thread threadObj1;
	std::thread threadObj2;
	bool listenToUDP = true;
};

TEST_F(test_multipleSimulatedISOObjects, HEAB_Sent_And_MONR_Not_received_due_to_no_OSEM) {
	ip::udp::endpoint ep1, ep2;
	std::vector<char> receivedData1(4096);
	std::vector<char> receivedData2(4096);
	

	obj1Conn.sendHeartbeat(ControlCenterStatusType::CONTROL_CENTER_STATUS_INIT);
	bool sent1 = false;
	std::thread timeoutThread1([this, sent1](){
		std::this_thread::sleep_for(10ms);
		if (!sent1) {
			this->sendUDPNoopToClient(1);
		}
	});
	std::size_t received = obj1Conn.receiveUDP(receivedData1, ep1);
	sent1 = true;
	EXPECT_EQ(received, 1) << "Received data on test-object1";


	obj2Conn.sendHeartbeat(ControlCenterStatusType::CONTROL_CENTER_STATUS_INIT);
	bool sent2 = false;
	std::thread timeoutThread2([this, sent2](){
		std::this_thread::sleep_for(10ms);
		if (!sent2) {
			this->sendUDPNoopToClient(2);
		}
	});
	received = obj2Conn.receiveUDP(receivedData2, ep2);
	sent2 = true;
	EXPECT_EQ(received, 1) << "Received data on test-object1";
	timeoutThread1.join();
	timeoutThread2.join();
}

TEST_F(test_multipleSimulatedISOObjects, OSEM_Sent_MONR_Received_as_READY) {
	ip::udp::endpoint ep1, ep2;
	std::vector<char> receivedData1(4096);
	std::vector<char> receivedData2(4096);
	obj1Conn.sendOSEM();
	bool sent1 = false;
	std::thread timeoutThread1([&, sent1](){
		std::this_thread::sleep_for(100ms);
		if (!sent1) {
			this->sendUDPNoopToClient(1);
		}
	});	
	obj1Conn.sendHeartbeat(ControlCenterStatusType::CONTROL_CENTER_STATUS_INIT);
	std::size_t received = obj1Conn.receiveUDP(receivedData1, ep1);
	sent1 = true;
	EXPECT_EQ(received, 60);

	obj2Conn.sendOSEM();
	obj2Conn.sendHeartbeat(ControlCenterStatusType::CONTROL_CENTER_STATUS_INIT);
	bool sent2 = false;
	std::thread timeoutThread2([&, sent2](){
		std::this_thread::sleep_for(100ms);
		if (!sent2) {
			this->sendUDPNoopToClient(2);
		}
	});
	received = obj2Conn.receiveUDP(receivedData2, ep2);
	sent2 = true;
	EXPECT_EQ(received, 60);

	timeoutThread1.join();
	timeoutThread2.join();
}