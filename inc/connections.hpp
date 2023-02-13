#pragma once
#include <memory>
#include <iostream>
#include <vector>
#include <boost/asio.hpp>

#include "MessageDispatcher.hpp"

class Channel : public std::enable_shared_from_this<Channel>
{
public:
protected:
    Channel(MessageDispatcher& dispatcher)
        : readData(MAX_LENGTH),
        dispatcher(dispatcher) {}
    virtual void handleRead(
        boost::system::error_code ec,
        std::size_t length);
    enum { MAX_LENGTH = 4096 };
    std::vector<char> data;
    std::vector<char> readData;
    MessageDispatcher& dispatcher;
    virtual void startReceive() = 0;
};

class MonitorChannel
    : public Channel
{
private:
    const static unsigned short ISO_22133_DEFAULT_UDP_PORT = 53240;
public:
    typedef std::shared_ptr<MonitorChannel> ptr;

    static ptr create(
        boost::asio::io_context& ioContext
        MessageDispatcher& dispatcher)
    {
        return ptr(new MonitorChannel(ioContext, dispatcher));
    }

    boost::asio::ip::udp::socket& socket()
    {
        return sock;
    }

    void start();
private:
    boost::asio::ip::udp::socket sock;
    boost::asio::ip::udp::endpoint remoteEndpoint;

    MonitorChannel(
        boost::asio::io_context& ioContext,
        MessageDispatcher& dispatcher,
        const unsigned short port = ISO_22133_DEFAULT_UDP_PORT)
    : sock(ioContext, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), port)),
    Channel(dispatcher) {}

    void startReceive() override;
};

class ControlChannel 
    : public Channel
{
public:
    typedef std::shared_ptr<ControlChannel> ptr;

    static ptr create(
        boost::asio::io_context& ioContext
        MessageDispatcher& dispatcher)
    {
        return ptr(new ControlChannel(ioContext, dispatcher));
    }

    boost::asio::ip::tcp::socket& socket()
    {
        return sock;
    }

    void start();

private:
    boost::asio::ip::tcp::socket sock;

    ControlChannel(
        boost::asio::io_context& ioContext,
        MessageDispatcher& dispatcher)
    : Channel(dispatcher),
    sock(ioContext) {}

    virtual void startReceive() override;
};




class TCPServer
{
private:
    const static unsigned short ISO_22133_DEFAULT_TCP_PORT = 53241;
public:
    TCPServer(boost::asio::io_context& ioContext, MessageDispatcher& dispatcher, const unsigned short port = ISO_22133_DEFAULT_TCP_PORT)
    : ioContext(ioContext), dispatcher(dispatcher), acceptor(ioContext, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), port)) {
        startAccept();
    }
    virtual ~TCPServer() = default;
private:
    boost::asio::ip::tcp::acceptor acceptor;
    boost::asio::io_context& ioContext;
    MessageDispatcher& dispatcher;
    void startAccept()
    {
        ControlChannel::ptr newConnection = ControlChannel::create(ioContext, dispatcher);
        acceptor.async_accept(
            newConnection->socket(),
            std::bind(&TCPServer::handleAccept,
            this,
            newConnection,
            boost::asio::placeholders::error));
    }

    void handleAccept(
        ControlChannel::ptr newConnection,
        const boost::system::error_code& error)
    {
        if (!error) {
            newConnection->start();
        }
        startAccept();
    }
};
