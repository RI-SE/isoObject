#include "Object.hpp"

using namespace ISO22133;

Object::Object() : state(new ISO22133::State()), dispatcher(*state, *this), tcpServer(ioContext, dispatcher)
{
    // TODO initialize state
}

Object::~Object()
{
}
