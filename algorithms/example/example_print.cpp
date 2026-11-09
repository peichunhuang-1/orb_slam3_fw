// An example algorithm (§4.1): doubles its input and prints it; ON_EOS prints how many it saw.
#include <cstdio>

#include "fw/fw.hpp"

namespace {
struct ExamplePrint {
    int count = 0;
    void on_n(const int& n) {
        ++count;
        std::printf("example_print: %d -> %d\n", n, 2 * n);
        std::fflush(stdout);
    }
    void on_eos() {
        std::printf("example_print: done after %d value(s)\n", count);
        std::fflush(stdout);
    }
};
}  // namespace

FW_REGISTER_ALGORITHM_CLASS(ExamplePrint, ON(("n", int, trigger, QUEUE(16, drop_oldest)), &ExamplePrint::on_n),
                            ON_EOS(&ExamplePrint::on_eos));
