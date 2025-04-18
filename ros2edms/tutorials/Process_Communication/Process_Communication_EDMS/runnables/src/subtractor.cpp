#include "Subtractor.hpp"

void Subtractor_onInit(const Subtractor_InStruct&, mwala::generic_io::State<Subtractor_State>&, Subtractor_RunContext&)
{
}

void Subtractor_onUpdate(const Subtractor_InStruct& in,
                       Subtractor_OutStruct& out,
                       mwala::generic_io::State<Subtractor_State>&,
                       Subtractor_RunContext&)
{
    for (auto& x : in.m_input)
    {
        out.m_output.get().number = x.get().number - 5;
        out.m_output.get().time = x.get().time;
    }
    out.m_output.setSendState(mwala::generic_io::SendState::SEND);
}