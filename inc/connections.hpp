#pragma once
#include <memory>
#include <vector>
#include <boost/asio.hpp>

#include "EventDispatcher.hpp"

class Channel : public std::enable_shared_from_this<Channel>
{
public:
    /*!
    * \brief Handle received data from a client. The call repeats
    *   until the buffer is empty. After a successful read, the
    *   dispatcher is called to handle the message. The handler enqueues
    *   itself again to handle the next incoming data.
    * \param ec The error code.
    * \param length The number of bytes read.
    */
    virtual void handleRead(
        boost::system::error_code ec,
        std::size_t length);
protected:
    Channel(EventDispatcher& dispatcher)
        : readData(MAX_LENGTH),
        dispatcher(dispatcher) {}
    enum { MAX_LENGTH = 4096 };
    std::vector<char> data;
    std::vector<char> readData;
    EventDispatcher& dispatcher;
    
    /*!
    * \brief Receive data from the client. The call is nonblocking
    *   and expects the io_context to be run in order to function.
    */
    virtual void startReceive() = 0;
};

/*!
* \brief A connection channel for the ISO 22133 protocol, over UDP. The
*   channel is used to send and receive messages from the client. The
*   object lifespan is controlled by the ControlChannel.
*/
class MonitorChannel
    : public Channel
{
private:
    const static unsigned short ISO_22133_DEFAULT_UDP_PORT = 53240;
public:
    typedef std::shared_ptr<MonitorChannel> ptr;

    /*!
    * \brief Create a new MonitorChannel.
    * \param executor The executor to use for the channel.
    * \param dispatcher The message dispatcher to use for received messages.
    * \param port The port to use for the channel.
    * \return A shared pointer to the new server.
    */
    static ptr create(
        boost::asio::executor executor,
        EventDispatcher& dispatcher,
        const unsigned short port = ISO_22133_DEFAULT_UDP_PORT);

    /*!
    * \brief Get the socket used by the channel.
    * \return Reference to the socket.
    */
    boost::asio::ip::udp::socket& socket() { return sock; }

    /*!
    * \brief Start handling messages from the client. The call is
    *   nonblocking and expects the io_context to be run in order to
    *   function.
    */
    void start();
private:
    boost::asio::ip::udp::socket sock;
    boost::asio::ip::udp::endpoint remoteEndpoint;

    MonitorChannel(
        boost::asio::executor executor,
        EventDispatcher& dispatcher,
        const unsigned short port = ISO_22133_DEFAULT_UDP_PORT)
    : sock(executor, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), port)),
    Channel(dispatcher) {}

    /*!
    * \brief Receive data from the client. The call is nonblocking
    *   and expects the io_context to be run in order to function.
    */
    void startReceive() override;
};


/*!
* \brief A connection channel for the ISO 22133 protocol, over TCP. The
*   channel is used to send and receive messages from the client. The
*   object lives in the io_context and is destroyed when the io_context
*   is destroyed, or when the channel is closed.
*/
class ControlChannel 
    : public Channel
{
public:
    typedef std::shared_ptr<ControlChannel> ptr;

    /*!
    * \brief Create a new ControlChannel.
    * \param ioContext The io_context to use for the channel.
    * \param dispatcher The message dispatcher to use for received messages.
    * \return A shared pointer to the new server.
    */
    static ptr create(
        boost::asio::io_context& ioContext,
        EventDispatcher& dispatcher);

    /*!
    * \brief Get the socket used by the channel.
    * \return Reference to the socket.
    */
    boost::asio::ip::tcp::socket& socket() { return sock; }

   /*!
    * \brief Start handling messages from the client and set up the
    *   parallel monitor channel. The call is nonblocking and expects
    *   the io_context to be run in order to function.
    */
    void start();

private:
    boost::asio::ip::tcp::socket sock;
    MonitorChannel::ptr monitorChannel;

    ControlChannel(
        boost::asio::io_context& ioContext,
        EventDispatcher& dispatcher)
    : Channel(dispatcher),
    sock(ioContext) {}

    /*!
    * \brief Receive data from the client. The call is nonblocking
    *   and expects the io_context to be run in order to function.
    */
    virtual void startReceive() override;
};


/*!
* \brief The TCP server class. It is responsible for accepting
*   new TCP connections and creating the corresponding control
*   channels.
*/
class TCPServer
{
private:
    const static unsigned short ISO_22133_DEFAULT_TCP_PORT = 53241;
public:
    TCPServer(
        boost::asio::io_context& ioContext,
        EventDispatcher& dispatcher,
        const unsigned short port = ISO_22133_DEFAULT_TCP_PORT)
    : ioContext(ioContext),
    dispatcher(dispatcher),
    acceptor(ioContext, boost::asio::ip::tcp::endpoint(
        boost::asio::ip::tcp::v4(), port))
    {
        startAccept();
    }
    virtual ~TCPServer() = default;
private:
    boost::asio::ip::tcp::acceptor acceptor;
    boost::asio::io_context& ioContext;
    EventDispatcher& dispatcher;

    /*!
    * \brief Handle the next TCP connection. The call is nonblocking
    *   and expects the io_context to be run in order to function.
    */
    void startAccept();

    /*!
    * \brief Handle the next TCP connection. The handler enqueues
    *   itself after it is done. The call is nonblocking and expects
    *   the io_context to be run in order to function.
    */
    void handleAccept(
        ControlChannel::ptr newConnection,
        const boost::system::error_code& error);
};
