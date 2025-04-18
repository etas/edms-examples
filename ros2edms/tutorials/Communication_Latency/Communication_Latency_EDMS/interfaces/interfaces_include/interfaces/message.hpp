#pragma once

#include <cstdint>

namespace processchain
{
const int MY_ARRAY_SIZE=10*1024*1024;

struct Message
{
    int64_t time{-1};
    int array[MY_ARRAY_SIZE];
};

} // namespace processchain