#include "Consumer.hpp"
#include <numeric>
#include <chrono>

constexpr int LATENCY_SIZE = 60;
double latencies[LATENCY_SIZE] = {0};
int latency_index = 0;

void Consumer_onInit(const Consumer_InStruct&, mwala::generic_io::State<Consumer_State>&, Consumer_RunContext&)
{
}

void Consumer_onUpdate(const Consumer_InStruct& in,
                       Consumer_OutStruct&,
                       mwala::generic_io::State<Consumer_State>&,
                       Consumer_RunContext&)
{
    auto now = std::chrono::system_clock::now();
    auto now_ms = std::chrono::duration<double, std::milli>(now.time_since_epoch()).count(); // Jetzt in ms

    if (!in.m_input.empty())
    {
        for (auto& x : in.m_input)
        {
            // Konvertiere x.get().time explizit in double, falls nötig
            double input_time = static_cast<double>(x.get().time);  

            // Falls x.get().time in Sekunden ist, muss es noch in Millisekunden umgerechnet werden:
            // input_time *= 1000.0;  // Falls es in Sekunden ist

            double new_time = now_ms - input_time; // Zeitdifferenz in ms

            latencies[latency_index] = new_time;
            latency_index++;

            if (latency_index == LATENCY_SIZE)
            {
                double avg_latency = std::accumulate(latencies, latencies + LATENCY_SIZE, 0.0) / LATENCY_SIZE;
                LOG_INFO("Average latency over last 60 samples: {:.2f} ms", avg_latency);

                latency_index = 0;
            }
        }
    }
}
