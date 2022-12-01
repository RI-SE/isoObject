#pragma once

#include <boost/asio.hpp>
#include <vector>

using namespace boost::asio;
using ip::tcp;

/**
 * @brief TCP server based on Boost::asio. Contains the basic functions needed for ISO22133 TCP communication
 *
 */
class TcpServer {
   public:
	TcpServer(uint32_t port)
		: acceptor(context, tcp::endpoint(tcp::v4(), port)), socket(context) {
            setBufferSize(defaultBufferSize);
        };
	virtual ~TcpServer() = default;
	void disconnect() { socket.close(); };
	void acceptConnection() { acceptor.accept(socket); };
	void setBufferSize(size_t size) { dataBuffer.resize(size); };
	size_t getBuffferSize() const { return dataBuffer.size(); };

	std::vector<char> receive() {
		auto nBytes = socket.receive(buffer(dataBuffer));
		setBufferSize(nBytes);
		std::vector<char> result(dataBuffer);
		setBufferSize(defaultBufferSize);
		return result;
	};

   private:
	std::vector<char> dataBuffer;
	size_t defaultBufferSize = 4096;

	io_context context;
	tcp::acceptor acceptor;
	tcp::socket socket;
};
