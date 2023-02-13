#pragma once
#include <vector>
#include <cstddef>

#include "State.hpp"

class MessageDispatcher
{
public:
    MessageDispatcher(ISO22133::State& state, ISO22133::Object& object);
    virtual ~MessageDispatcher() = default;

    virtual size_t dispatch(const std::vector<char>& message);
private:
    ISO22133::State& state;
    ISO22133::Object& object;
};