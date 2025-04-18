#include "Producer.hpp"

void Producer_onInit(const Producer_InStruct&, mwala::generic_io::State<Producer_State>&, Producer_RunContext&)
{
}

void Producer_onUpdate(const Producer_InStruct&,
                       Producer_OutStruct& out,
                       mwala::generic_io::State<Producer_State>& state,
                       Producer_RunContext&)
{
    constexpr int MAX_LEN = 11; // Falls das Array nur 10 Zeichen + Terminator hat

    for (int i = 0; i < MAX_LEN; ++i)
    {
        out.m_output.get().information[i] = state.get().message[i];
    }

    // String mit Null-Terminierung abschließen (wichtig für LOG_INFO)
    out.m_output.get().information[MAX_LEN] = '\0';

    out.m_output.setSendState(mwala::generic_io::SendState::SEND);

    LOG_INFO("sending: {}", out.m_output.get().information);
}
