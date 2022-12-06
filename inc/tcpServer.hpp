#pragma once

#include <boost/asio.hpp>
#include <boost/system/system_error.hpp>
#include <vector>

using namespace boost::asio;
using ip::tcp;

/**
 * @brief TCP server based on Boost::asio. Contains the basic functions needed for ISO22133 TCP communication
 *
 */
class TcpServer {
   public:
	TcpServer(uint32_t port) : acceptor(context, tcp::endpoint(tcp::v4(), port)), socket(context) {
		setBufferSize(defaultBufferSize);
	};
	virtual ~TcpServer() = default;
	void disconnect() {
		try {
			socket.shutdown(socket_base::shutdown_both);
			socket.close();
		} catch (boost::system::system_error& e) {}
	};

	void acceptConnection() {
		try {
			acceptor.accept(socket);
		} catch (boost::system::system_error& e) {
			std::cerr << "TCP socket accept failed: " << e.what() << std::endl;
		}
	}

	void setBufferSize(size_t size) { dataBuffer.resize(size); };
	size_t getBuffferSize() const { return dataBuffer.size(); };

	tcp::endpoint getEndPoint() const { return socket.remote_endpoint(); };

	bool isOpen() const { return socket.is_open(); };

	std::vector<char> receive() {
		try {
			auto nBytes = socket.receive(buffer(dataBuffer));

			setBufferSize(nBytes);
			std::vector<char> result(dataBuffer);
			setBufferSize(defaultBufferSize);
			return result;
		} catch (boost::system::system_error& e) {
			if (e.code() == error::eof) {
				std::cerr << "Peer closed connection" << std::endl;
				throw e;
			} else {
				std::cerr << "TCP socket receive failed: " << e.what() << std::endl;
				throw e;
			}
		}
	};

   private:
	std::vector<char> dataBuffer;
	size_t defaultBufferSize = 4096;

	io_context context;
	tcp::acceptor acceptor;
	tcp::socket socket;
};
