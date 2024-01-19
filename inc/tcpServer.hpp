#pragma once

#include <boost/asio.hpp>
#include <boost/system/system_error.hpp>
#include <vector>

// These namespace declarations does not work in SWIG :( 
// using namespace boost::asio;
// using boost::asio::ip::tcp;

/**
 * @brief TCP server based on Boost::asio. Contains the basic functions needed for ISO22133 TCP communication
 *
 */
class TcpServer {
   public:
	TcpServer(std::string ip, uint32_t port) :
	acceptor(context, boost::asio::ip::tcp::endpoint(boost::asio::ip::address_v4::from_string(ip), port)),
	socket(context),
	acceptIncoming(true) {
		setBufferSize(defaultBufferSize);
	};

	TcpServer(int sock) : 
	socket(context),
	acceptor(context),
	acceptIncoming(false) {
		setBufferSize(defaultBufferSize);
		socket.assign(boost::asio::ip::tcp::v4(), sock);
	};

	virtual ~TcpServer() = default;
	void disconnect() {
		try {
			if (this->acceptIncoming) {
				acceptor.cancel();
			}
			if (socket.is_open()){
				socket.shutdown(boost::asio::socket_base::shutdown_both);
				socket.close();
			}
			context.reset();
		} catch (boost::system::system_error& e) {
			std::cerr << "Error when closing socket: " << e.what() << std::endl;
		}
	};

	void acceptConnection() {
		if (!this->acceptIncoming) {
			throw std::runtime_error("Cannot accept connections on a socket that is not listening");
		}
		acceptor.async_accept(socket, [this](const boost::system::error_code& error) {
			if (error) {
				// Accepting failed, handle the error
				// Print the error message for example
				if (error == boost::asio::error::operation_aborted) {
					std::cerr << "TCP Accept aborted" << std::endl;
				} else {
					std::cerr << "TCP Accept error: " << error.message() << std::endl;
					throw boost::system::system_error(boost::asio::error::eof);
				}
			}
		});
		context.run_one();
		context.restart();
	}

	void setBufferSize(size_t size) { dataBuffer.resize(size); };
	size_t getBuffferSize() const { return dataBuffer.size(); };

	boost::asio::ip::tcp::endpoint getEndPoint() const { return socket.remote_endpoint(); };

	bool isOpen() const { return socket.is_open(); };

	std::vector<char> receive() {
		try {
			auto nBytes = socket.receive(boost::asio::buffer(dataBuffer));

			setBufferSize(nBytes);
			std::vector<char> result(dataBuffer);
			setBufferSize(defaultBufferSize);
			return result;
		} catch (boost::system::system_error& e) {
			if (e.code() == boost::asio::error::eof) {
				std::cerr << "Peer closed connection" << std::endl;
				throw e;
			} else {
				std::cerr << "TCP socket receive failed: " << e.what() << std::endl;
				throw e;
			}
		}
	};

	void send(std::vector<char> data, size_t nbytes) {
		std::vector<char> sendBuffer(data);
		sendBuffer.resize(nbytes);
		socket.async_send(boost::asio::buffer(sendBuffer, nbytes), 
			[](const boost::system::error_code& error, std::size_t bytes_transferred) {
				if (error) {
					// Sending failed, handle the error
					// Print the error message for example
					std::cerr << "Send error: " << error.message() << std::endl;
					throw boost::system::system_error(boost::asio::error::eof);
				}
			}
		);
	};

   private:
	std::vector<char> dataBuffer;
	size_t defaultBufferSize = 4096;
	bool acceptIncoming;

	boost::asio::io_context context;
	boost::asio::ip::tcp::acceptor acceptor;
	boost::asio::ip::tcp::socket socket;
};
