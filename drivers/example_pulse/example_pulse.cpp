// An example driver (§4.7): a value every 500 ms until the box stops (graphs/example.graph.yaml)
#include <chrono>

#include "fw/fw.hpp"

namespace {
void example_pulse(fw::DriverContext& ctx) {
    for (int k = 1; ctx.sleep_for(std::chrono::milliseconds(500)); ++k)
        ctx.emit("n", k, fw::Time{static_cast<std::int64_t>(k) * 500000000});
}
}  // namespace

FW_REGISTER_DRIVER(example_pulse, OUTPUTS(("n", int)));
