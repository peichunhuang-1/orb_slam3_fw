// An example finite driver (§4.7): counts to 20, then sends EOS (graphs/example_job.graph.yaml)
#include <chrono>

#include "fw/fw.hpp"

namespace {
void example_count(fw::DriverContext& ctx) {
    for (int k = 1; k <= 20; ++k) {
        if (!ctx.sleep_period(std::chrono::milliseconds(100))) return;  // stopped: not the end of the data
        ctx.emit("n", k, fw::Time{static_cast<std::int64_t>(k) * 100000000});
    }
    ctx.eos();  // a finite driver ends with EOS
}
}  // namespace

FW_REGISTER_DRIVER(example_count, OUTPUTS(("n", int)));
