#pragma once
// A problem solved one LM iteration at a time (nls::Problem::start / step), for the global BA: gba_solver runs
// one step per message so local mapping can stop it between iterations, as the original's mbStopGBA stops g2o
// (algorithms/orb_loop_closing/docs/DESIGN.md, "global BA：分段迭代"). Solving it in steps to the end gives what
// the kind's solve function gives.
#include "types/ba/problem.hpp"

namespace ba_solver {

class SteppedSolve {
public:
    virtual ~SteppedSolve() = default;
    virtual void step() = 0;              // one iteration (nothing once finished)
    virtual bool finished() const = 0;    // no iteration left: result() is final
    virtual int iterations() const = 0;   // run so far
    virtual ba::Result result() const = 0;
};

inline ba::Result solveAll(SteppedSolve& s) {
    while (!s.finished()) s.step();
    return s.result();
}

}  // namespace ba_solver
