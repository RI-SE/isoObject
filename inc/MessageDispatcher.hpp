#pragma once
#include <vector>
#include <cstddef>
#include <memory>

namespace ISO22133
{
    class State;
    class Object;
    class TrajDecoder;
}

class MessageDispatcher
{
public:
    MessageDispatcher(ISO22133::State& state, ISO22133::Object& object);
    ~MessageDispatcher();

    virtual size_t dispatch(const std::vector<char>& message);
private:
    ISO22133::State& state;
    ISO22133::Object& object;
    std::unique_ptr<ISO22133::TrajDecoder> trajDecoder;
};