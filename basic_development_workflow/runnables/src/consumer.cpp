#include "Consumer.hpp"

void Consumer_onInit(const Consumer_InStruct&, mwala::generic_io::State<Consumer_State>&, Consumer_RunContext&)
{
}

void Consumer_onUpdate(const Consumer_InStruct& in,
                       Consumer_OutStruct&,
                       mwala::generic_io::State<Consumer_State>&,
                       Consumer_RunContext&)
{
    if (in.m_input.empty())
    {
        LOG_INFO("m_input is empty");
    }
    else
    {
        for (auto& x : in.m_input)
        {
            LOG_INFO("received number: {}", x.get().number);
            if (x.get().number % 2 == 0)
            {
                LOG_INFO("number is even");
            }
            else
            {
                LOG_INFO("number is odd");
            }            
        }
    }
}
