
#include <string>
#include <chrono>
#include <thread>
#include <future>
#include <gtest/gtest.h>
#include <boost/asio.hpp>
#include <boost/system/system_error.hpp>

#include "tcpServer.hpp"

extern "C" {
	#include "iso22133.h"
}

using namespace boost::asio;

class tcpServer_connect_new_socket : public ::testing::Test
{
protected:
	void SetUp() override 


    {
        // Start server
        const std::string ip = "127.0.0.1";
        const uint32_t port = 1234;
        server = new TcpServer(ip, port);
        acceptThread = new std::thread(&TcpServer::acceptConnection, server);
    }

	void TearDown() override
    {
        // Stop server
        server->disconnect();
        acceptThread->join();
        delete server;
    }
    std::thread *acceptThread;
    TcpServer *server;
};

TEST_F(tcpServer_connect_new_socket, connect)
{
    // Connect to server
    
    io_service io_service;
    ip::tcp::socket socket(io_service);
    ip::tcp::endpoint endpoint(ip::address::from_string("127.0.0.1"), 1234);
    try {
        socket.connect(endpoint);
    } catch (boost::system::system_error &e) {
        FAIL() << "Failed to connect to server: " << e.what();
    }
}


class tcpServer_on_existing_socket : public ::testing::Test
{
protected:
    tcpServer_on_existing_socket() : connecterSocket(io_service), listener(io_service) {}
	void SetUp() override 
    {
        server = NULL;
        std::shared_future<void> readyFuture(readyPromise.get_future());
        // Start server
        ip::tcp::endpoint ep(ip::address_v4::from_string("127.0.0.1"), 1234);
        listener.open(ep.protocol());
        listener.set_option(ip::tcp::acceptor::reuse_address(true));
        listener.bind(ep);
        listener.listen(1);
        listener.async_accept([this](const boost::system::error_code& error, ip::tcp::socket peer) {
            sockets.push_back(std::move(peer));
            server = new TcpServer(sockets.back().native_handle());
            readyPromise.set_value();
        });
        // Connect to server

        try {
            connecterSocket.connect(ep);
        } catch (boost::system::system_error &e) {
            FAIL() << "Failed to connect to server: " << e.what();
        }
        io_service.run_one();
        readyFuture.wait();
    }

	void TearDown() override
    {
        try {
            listener.cancel();
            listener.close();
        } catch (boost::system::system_error &e) {
            std::cerr << "Failed to close listener: " << e.what() << std::endl;
        }
        // Close connection
        for (auto &socket : sockets) {
            try {
                socket.shutdown(ip::tcp::socket::shutdown_both);
            } catch (boost::system::system_error &e) {
                std::cerr << "Failed to close listener: " << e.what() << std::endl;
            }
        }

        // Stop server
        if (server != NULL) {
            server->disconnect();
            delete server;
        }
    }

    io_context io_service;
    std::promise<void> readyPromise;
    ip::tcp::socket connecterSocket;
    ip::tcp::acceptor listener;
    std::thread *acceptThread;
    TcpServer *server;
    std::vector<ip::tcp::socket> sockets;
};

TEST_F(tcpServer_on_existing_socket, acceptConnection_should_throw_error)
{
    // Connect to server
    try {
        server->acceptConnection();
        FAIL() << "Should throw error";
    } catch (std::runtime_error &e) {
        SUCCEED() << "Failed to accept connection: " << e.what();
    }
}