#include "Consumer.hpp"
#include <numeric>
#include <chrono>

constexpr int MAX_SAMPLES = 60;
double latencies[MAX_SAMPLES] = {0};
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
    auto now_ms = std::chrono::duration<double, std::milli>(now.time_since_epoch()).count();

    if (!in.m_input.empty())
    {
        for (auto& x : in.m_input)
        {
            double input_time = static_cast<double>(x.get().time);  
            double new_time = now_ms - input_time;

            latencies[latency_index] = new_time;
            latency_index++;

            if (latency_index == MAX_SAMPLES)
            {
                double avg_latency;
                avg_latency = std::accumulate(latencies, latencies + MAX_SAMPLES, 0.0) / MAX_SAMPLES;
                LOG_INFO("Average latency over last 60 samples: {:.2f} ms", avg_latency);
                std::cout << "latency avg " << avg_latency <<  " ms" << std::endl; 

                latency_index = 0;
            }
        }
    }
}
