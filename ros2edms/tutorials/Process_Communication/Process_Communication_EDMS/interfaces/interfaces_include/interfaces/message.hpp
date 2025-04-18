#pragma once

#include <cstdint>

namespace processchain
{

struct Message
{
    int number{-1};
    int64_t time{-1};
};

} // namespace processchain