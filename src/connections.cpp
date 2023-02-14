#include "connections.hpp"
#include <functional>
#include <iostream>

ControlChannel::ptr ControlChannel::create(
    boost::asio::io_context& ioContext,
    EventDispatcher& dispatcher)
{
    return ptr(new ControlChannel(ioContext, dispatcher));
}

MonitorChannel::ptr MonitorChannel::create(
    boost::asio::executor executor,
    EventDispatcher& dispatcher,
    const unsigned short port)
{
    return ptr(new MonitorChannel(executor, dispatcher, port));
}

void ControlChannel::start()
{
    std::cout << "Control channel from "
        << sock.remote_endpoint().address().to_string()
        << " established." << std::endl;
    //state->handleEvent(*this, ISO22133::Events::B);
    
    monitorChannel = MonitorChannel::create(sock.get_executor(), dispatcher);
    startReceive();
    monitorChannel->start();
}

void MonitorChannel::start()
{
    std::cout << "Monitor channel established." << std::endl;
    startReceive();
}

void ControlChannel::startReceive()
{
    std::cout << "Waiting for TCP data..." << std::endl;
    sock.async_receive(boost::asio::buffer(readData),
        std::bind(&Channel::handleRead,
            shared_from_this(),
            std::placeholders::_1,
            std::placeholders::_2));
}

void MonitorChannel::startReceive()
{
    // Use a weak pointer so that the socket is not kept alive
    // by the asynchronous operation. This is necessary because
    // the socket is owned by the TCP connection - when the TCP
    // connection is closed, the UDP socket will be closed as well.
    std::weak_ptr<Channel> wptr(shared_from_this());
    std::cout << "Waiting for UDP data..." << std::endl;
    sock.async_receive_from(
        boost::asio::buffer(readData, MAX_LENGTH),
        remoteEndpoint,
        [wptr](const boost::system::error_code& ec, std::size_t length) {
            auto self = wptr.lock();
            if (self) {
                self->handleRead(ec, length);
            }
        });
}

void Channel::handleRead(
    boost::system::error_code ec,
    std::size_t length)
{
    if (ec) {
        if (ec != boost::asio::error::eof) {
            std::cerr << "Error: " << ec.message() << std::endl;
        }
        // state->handleEvent(*this, ISO22133::Events::L);
        return;
    }
    data.insert(data.end(), readData.begin(), readData.begin() + length);
    int nBytesHandled = 0;
    do {
        try {
            nBytesHandled = dispatcher.dispatch(data);
        } catch (const std::exception& e) {
            std::cerr << e.what() << std::endl;
            break;
        }
        data.erase(data.begin(), data.begin() + nBytesHandled);
    } while (data.size() > 0);
    startReceive();
}

void TCPServer::startAccept()
{
    ControlChannel::ptr newConnection = ControlChannel::create(ioContext, dispatcher);
    acceptor.async_accept(
        newConnection->socket(),
        std::bind(&TCPServer::handleAccept,
            this,
            newConnection,
            std::placeholders::_1));
}

void TCPServer::handleAccept(
    ControlChannel::ptr newConnection,
    const boost::system::error_code& error)
{
    if (!error) {
        newConnection->start();
    }
    startAccept();
}
