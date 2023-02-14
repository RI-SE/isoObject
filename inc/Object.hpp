#pragma once

#include <memory>
#include <boost/asio/io_context.hpp>

#include "connections.hpp"
#include "EventDispatcher.hpp"

//struct CartesianPosition;

namespace ISO22133
{

class State {

};

class Object
{
public:
    Object();
    virtual ~Object();

    void run() {
        ioContext.run();
    }

    void setPosition(const double x, const double y, const double z) {}

private:
    //void sendMONR();
    boost::asio::io_context ioContext;
    EventDispatcher dispatcher;
	ISO22133::State* state;
    TCPServer tcpServer;

    //std::unique_ptr<CartesianPosition> position;
};

}