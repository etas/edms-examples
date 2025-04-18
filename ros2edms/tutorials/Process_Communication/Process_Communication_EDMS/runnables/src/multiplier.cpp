#include "Multiplier.hpp"

void Multiplier_onInit(const Multiplier_InStruct&, mwala::generic_io::State<Multiplier_State>&, Multiplier_RunContext&)
{
}

void Multiplier_onUpdate(const Multiplier_InStruct& in,
                       Multiplier_OutStruct& out,
                       mwala::generic_io::State<Multiplier_State>&,
                       Multiplier_RunContext&)
{
    for (auto& x : in.m_input)
    {
        out.m_output.get().number = x.get().number * 2;
        out.m_output.get().time = x.get().time;
    }
    out.m_output.setSendState(mwala::generic_io::SendState::SEND);
}