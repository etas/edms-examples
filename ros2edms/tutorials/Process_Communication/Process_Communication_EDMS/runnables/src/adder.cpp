#include "Adder.hpp"

void Adder_onInit(const Adder_InStruct&, mwala::generic_io::State<Adder_State>&, Adder_RunContext&)
{
}

void Adder_onUpdate(const Adder_InStruct& in,
                       Adder_OutStruct& out,
                       mwala::generic_io::State<Adder_State>&,
                       Adder_RunContext&)
{
    for (auto& x : in.m_input)
    {
        out.m_output.get().number = x.get().number + 10;
        out.m_output.get().time = x.get().time;
    }
    out.m_output.setSendState(mwala::generic_io::SendState::SEND);
}