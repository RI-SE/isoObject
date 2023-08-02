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

class TCPConnection
{
	public:
		TCPConnection(const std::string& ip = "0.0.0.0", int id = 1, char messCnt = 0) : 
		listenIP(ip),
		tcpSocket(context),
		udpSocket(context, ip::udp::v4()),
		ep(ip::address_v4::from_string(ip), ISO_22133_OBJECT_UDP_PORT),
		receiverID(id),
		messageCounter(messCnt) {
		}
		virtual ~TCPConnection() = default;
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
			tcpSocket.close();
			udpSocket.close();
		}
		std::future<long unsigned int> receiveUDP(std::vector<char>& data, ip::udp::endpoint& ep) {
			return udpSocket.async_receive_from(buffer(data), ep, use_future);
		}

		void receiveTCP(std::vector<char>& data) {
			tcpSocket.receive(buffer(data));
		}

		void sendHeartbeat(const ControlCenterStatusType ccStatus) {
			HeabMessageDataType heartbeat;
			TimeSetToCurrentSystemTime(&heartbeat.dataTimestamp);
			heartbeat.controlCenterStatus = ccStatus;
			std::vector<char> transmitBuffer(1024);
			auto nBytesWritten = encodeHEABMessage(this->receiverID, this->messageCounter, &heartbeat.dataTimestamp, heartbeat.controlCenterStatus,
								transmitBuffer.data(), transmitBuffer.size(), false);
			transmitBuffer.resize(nBytesWritten);
			sendUDP(transmitBuffer);
		}

		void sendOSEM(const int transmitterID) {
			ObjectSettingsType objSettings;
			objSettings.desiredID.transmitter = receiverID;
			objSettings.desiredID.controlCentre = transmitterID;
			objSettings.desiredID.subTransmitter = 0;

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


			std::vector<char> transmitBuffer(1024);
			auto nBytesWritten = encodeOSEMMessage(receiverID, 0, &objSettings, transmitBuffer.data(), transmitBuffer.size(), false);
			transmitBuffer.resize(nBytesWritten);
			std::cout << "Sending TCP " << nBytesWritten << " bytes" << std::endl;
			sendTCP(transmitBuffer);
		}

		ip::tcp::socket *getTCPSocket() {
			return &tcpSocket;
		}
		ip::udp::socket *getUDPSocket() {
			return &udpSocket;
		}
	private:
	const std::string listenIP;
	io_context context;
	ip::tcp::socket tcpSocket;
	ip::udp::socket udpSocket;
	ip::udp::endpoint ep;
	const int receiverID;
	char messageCounter;
};

class IPListener
{
	public:
		IPListener(const std::string& listenIP = "127.0.0.1") : 
		acceptor(context, ip::tcp::endpoint(ip::address_v4::from_string(listenIP), ISO_22133_DEFAULT_OBJECT_TCP_PORT)),
		udpSocket(context, ip::udp::endpoint(ip::address_v4::from_string(listenIP), ISO_22133_OBJECT_UDP_PORT)) {}
		virtual ~IPListener() = default;
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
			udpSocket.close();
		}

		std::size_t receiveUDP(std::vector<char>& data, ip::udp::endpoint& ep) {
			size_t received = udpSocket.receive_from(buffer(data), ep);
			return received;
		}
		
	private:
		io_context context;
		ip::tcp::acceptor acceptor;
		ip::udp::socket udpSocket;
		int timeout;
};

class IsoObjectCreateMultiple : public ::testing::Test
{
protected:
	IsoObjectCreateMultiple():
	obj1Conn("127.0.0.1", 1),
	obj2Conn("127.0.0.1", 2)
	{
		threadListener = std::thread(&IsoObjectCreateMultiple::tcpListen, this);
		obj1Conn.connect();
		threadListener.join();
		// threadListener = std::thread(&IsoObjectCreateMultiple::tcpListen, this);
		// obj2Conn.connect();
		// threadListener.join();
		obj1 = new TestObject(sockets[0]->native_handle());
		// obj2 = new TestObject(sockets[1]->native_handle());
		threadListener = std::thread(&IsoObjectCreateMultiple::udpReceive, this);
	}
	void SetUp() override {}

	void TearDown() override {}
	virtual ~IsoObjectCreateMultiple() {
		obj1Conn.disconnect();
		std::cerr << "Before disc5" << std::endl;
		obj1->shutdown_threads();
		std::cerr << "After Threads" << std::endl;
		delete obj1;

		// std::cerr << "Before disc6" << std::endl;
		// obj2Conn.disconnect();	
		// delete obj2;

		listenToUDP = false;
		std::vector<char> noopSend(1);
		// obj2Conn.sendUDP(noopSend);
		threadListener.join();

		listener.disconnect();
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
							TestObject *obj = nullptr;
							if (HeaderData.receiverID == 1) {
								obj = obj1;
							}
							else if (HeaderData.receiverID == 2) {
								obj = obj2;
							}
							if (obj != nullptr) {
								std::cerr << "Received UDP message for object " << HeaderData.receiverID << std::endl;
								int handled = obj->handleUDPMessage(data, listener.getUDPSocket()->native_handle(), ep);
								nBytesHandled += handled;
							}
							else {
								EXPECT_NE(obj, nullptr) << "Unknown receiver ID";
								break;
							}
						} catch (const std::exception& e) {
							std::cerr << e.what() << std::endl;
							break;
						}
						data.erase(data.begin(), data.begin() + nBytesHandled);
					} 
				}
			}
		} catch(const std::exception& e) {
			std::cerr << e.what() << std::endl;
		}
	}

	std::vector<ip::tcp::socket*> sockets;
	TCPConnection obj1Conn;
	TCPConnection obj2Conn;
	IPListener listener;
	TestObject *obj1;
	TestObject *obj2;
	std::thread threadListener;
	std::thread threadObj1;
	std::thread threadObj2;
	bool listenToUDP = true;
};

TEST_F(IsoObjectCreateMultiple, HEAB_Sent_And_MONR_Not_received_due_to_no_OSEM) {
	ip::udp::endpoint ep1, ep2;
	std::vector<char> receivedData1(4096);
	std::vector<char> receivedData2(4096);
	auto future_resp1 = obj1Conn.receiveUDP(receivedData1, ep1);
	// auto future_resp2 = obj2Conn.receiveUDP(receivedData2, ep2);
	obj1Conn.sendHeartbeat(ControlCenterStatusType::CONTROL_CENTER_STATUS_INIT);
	switch (future_resp1.wait_for(10ms)) {
		case std::future_status::ready: {
			int received = future_resp1.get();
			receivedData1.resize(received);
			EXPECT_EQ(received, 0) << "Unknown message received on test-object2";
			break;
		}
		case std::future_status::timeout:
        case std::future_status::deferred:{
			EXPECT_TRUE(true);
            break;
        }
	}


	// obj2Conn.sendHeartbeat(ControlCenterStatusType::CONTROL_CENTER_STATUS_INIT);
	// switch (future_resp2.wait_for(10ms)) {
	// 	case std::future_status::ready: {
	// 		int received = future_resp2.get();
	// 		receivedData2.resize(received);
	// 		EXPECT_EQ(received, 0) << "Unknown message received on test-object1";
	// 		break;
	// 	}
	// 	case std::future_status::timeout:
    //     case std::future_status::deferred:{
	// 		EXPECT_TRUE(true);
    //         break;
    //     }
	// }
	std::cout << "Finished HEAB_Sent_And_MONR_Not_received_due_to_no_OSEM" << std::endl;
}

TEST_F(IsoObjectCreateMultiple, OSEM_Sent_MONR_Received_as_READY) {
	ip::udp::endpoint ep1, ep2;
	std::vector<char> receivedData1(4096);
	std::vector<char> receivedData2(4096);
	auto future_resp1 = obj1Conn.receiveUDP(receivedData1, ep1);
	auto future_resp2 = obj2Conn.receiveUDP(receivedData2, ep2);
	obj1Conn.sendOSEM(0xF00F);
	obj2Conn.sendOSEM(0xF00F);
	obj1Conn.sendHeartbeat(ControlCenterStatusType::CONTROL_CENTER_STATUS_INIT);
	obj2Conn.sendHeartbeat(ControlCenterStatusType::CONTROL_CENTER_STATUS_INIT);
	switch (future_resp1.wait_for(100ms)) {
		case std::future_status::ready: {
			int received = future_resp1.get();
			receivedData1.resize(received);
			struct timeval currentTime;
			ObjectMonitorType monitorData;
			TimeSetToCurrentSystemTime(&currentTime);
			int handled = decodeMONRMessage(receivedData1.data(), receivedData1.size(), currentTime, &monitorData, false);
			EXPECT_EQ(monitorData.state, OBJECT_STATE_DISARMED) << "Unknown State in MONR message";
			break;
		}
		case std::future_status::timeout:
        case std::future_status::deferred:{
			EXPECT_TRUE(false) << "No message received on test-object1";
            break;
        }
	}
	switch (future_resp2.wait_for(100ms)) {
		case std::future_status::ready: {
			int received = future_resp2.get();
			receivedData2.resize(received);
			struct timeval currentTime;
			ObjectMonitorType monitorData;
			TimeSetToCurrentSystemTime(&currentTime);
			int handled = decodeMONRMessage(receivedData2.data(), receivedData2.size(), currentTime, &monitorData, false);
			EXPECT_EQ(monitorData.state, OBJECT_STATE_DISARMED) << "Unknown State in MONR message";
			break;
		}
		case std::future_status::timeout:
        case std::future_status::deferred:{
			EXPECT_TRUE(false) << "No message received on test-object2";
            break;
        }
	}
}