#pragma once

#include <memory>
#include <boost/asio.hpp>

class CartesianPosition;

class Object
{
public:
    Object();
    virtual ~Object();

    void setPosition(const double x, const double y, const double z);

private:
    void sendMONR();
    
    boost::asio::io_context ioContext;
    boost::asio::ip::tcp::socket ctrlChannel;
    boost::asio::ip::udp::socket mntrChannel;

    std::unique_ptr<CartesianPosition> position;
};