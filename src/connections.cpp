#include "connections.hpp"
#include <functional>

MonitorChannel::ptr MonitorChannel::create(
    boost::asio::io_context& ioContext,
    MessageDispatcher& dispatcher,
    const unsigned short port)
{
    return ptr(new MonitorChannel(ioContext, dispatcher, port));
}

void ControlChannel::start()
{
    std::cout << "Control channel from "
        << sock.remote_endpoint().address().to_string()
        << " established." << std::endl;
    //state->handleEvent(*this, ISO22133::Events::B);
    boost::asio::ip::udp::socket sock2 = boost::asio::ip::udp::socket(sock.get_executor;
    monitorChannel = MonitorChannel::create(ioContext, dispatcher);
    
    startReceive();
}

void MonitorChannel::start()
{
    std::cout << "Monitor channel established." << std::endl;
    //state->handleEvent(*this, ISO22133::Events::B);
    
    startReceive();
}

void Channel::handleRead(
    boost::system::error_code ec,
    std::size_t length)
{
    std::cout << "Received " << length << " bytes." << std::endl;
    if (ec) {
        std::cerr << "Error: " << ec.message() << std::endl;
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

void ControlChannel::startReceive()
{
    std::cout << "Waiting for data..." << std::endl;
    sock.async_receive(boost::asio::buffer(readData),
        std::bind(&Channel::handleRead,
            shared_from_this(),
            std::placeholders::_1,
            std::placeholders::_2));
}

void MonitorChannel::startReceive()
{
    sock.async_receive_from(
        boost::asio::buffer(readData, MAX_LENGTH),
        remoteEndpoint,
        std::bind(&Channel::handleRead,
            shared_from_this(),
            std::placeholders::_1,
            std::placeholders::_2));
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