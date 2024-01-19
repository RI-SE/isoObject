#pragma once

#include <boost/asio.hpp>
#include <boost/system/system_error.hpp>
#include <iostream>
#include <vector>

// These namespace declarations does not work in SWIG :( 
// using namespace boost::asio;
// using ip::udp;

/**
 * @brief UDP server based on Boost::asio. Contains the basic functions needed for ISO22133 UDP communication
 *
 */
class UdpServer {
   public:
	UdpServer(const std::string &ip, uint32_t port) : 
	socket(context, boost::asio::ip::udp::endpoint(boost::asio::ip::address_v4::from_string(ip), port)) {
		setBufferSize(defaultBufferSize);
	};
	UdpServer() :
	socket(context) {
		setBufferSize(defaultBufferSize);
	};

	void setEndpoint(int native_socket, boost::asio::ip::udp::endpoint &ep) {
		socket.assign(boost::asio::ip::udp::v4(), native_socket);
		senderEndpoint = ep;
	};

	void setBufferSize(size_t size) { dataBuffer.resize(size); };

	void disconnect() {
		// This call may throw but shutdown is still successful.
		try {
			socket.shutdown(boost::asio::socket_base::shutdown_receive);
		} catch (const boost::system::system_error& e) {} 
	};

	std::vector<char> receive() {
		try {
			auto nBytes = socket.receive_from(boost::asio::buffer(dataBuffer), senderEndpoint);
			setBufferSize(nBytes);
			std::vector<char> result(dataBuffer);
			setBufferSize(defaultBufferSize);
			return result;
		} catch (const std::exception& e) {
			std::stringstream ss;
			ss << "Exception in receive UDP: " << e.what() << '\n';
			std::cerr << ss.str();
			throw e;
		}
	};

	size_t send(std::vector<char> data, size_t nbytes) {
		try {
			std::vector<char> sendBuffer(data);
			sendBuffer.resize(nbytes);
			auto bytesSent = socket.send_to(boost::asio::buffer(sendBuffer), senderEndpoint);
			return bytesSent;
		} catch (const std::exception& e) {
			std::stringstream ss;
			ss << "Exception in send UDP: " << e.what() << '\n';
			std::cerr << ss.str();
			throw e;
		}
	};

   private:
	std::vector<char> dataBuffer;
	size_t defaultBufferSize = 4096;

	boost::asio::io_context context;
	boost::asio::ip::udp::socket socket;
	boost::asio::ip::udp::endpoint senderEndpoint;
};
