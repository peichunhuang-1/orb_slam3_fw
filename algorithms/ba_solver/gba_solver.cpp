// gba_solver: loop closing's global BA (the original's RunGlobalBundleAdjustment thread), solved one LM iteration
// per message so local mapping can stop it between two (algorithms/orb_loop_closing/docs/DESIGN.md, "global BA：
// 分段迭代"). Its own block, beside ba_solver, so the seconds it takes do not hold up local BA.
//   problem   (global: Optimizer::BundleAdjustment; full_inertial: FullInertialBA) replaces the one in hand,
//             runs its first iteration
//   continue  with the problem's seq: the next iteration; another seq (a stopped or replaced BA): ignored
// After each iteration a Result: partial (seq, kind, iterations, the time so far) while iterations remain, then the
// final one, and the problem is dropped. Solved to the end in steps it is what ba_solver would give.
#include <chrono>
#include <cstdio>
#include <memory>

#include "algorithms/ba_solver/global_ba.hpp"
#include "algorithms/ba_solver/inertial.hpp"
#include "algorithms/ba_solver/stepped.hpp"
#include "fw/fw.hpp"
#include "types/ba/problem.hpp"

namespace {

class GbaSolver {
public:
    void on_problem(const ba::Problem& p, fw::Optional<ba::Result>& result) {
        const auto t0 = std::chrono::steady_clock::now();
        seq_ = p.seq;
        kind_ = p.kind;
        ms_ = 0;
        if (p.kind == ba::Kind::global)
            solve_ = ba_solver::startGlobal(p);
        else if (p.kind == ba::Kind::full_inertial)
            solve_ = ba_solver::startFullInertial(p);
        else {
            std::fprintf(stderr, "gba_solver: not a global BA kind (%d), ignored\n", int(p.kind));
            solve_.reset();
            return;
        }
        build_ms_ = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        ms_ = build_ms_;
        stepOnce(result);
    }

    void on_continue(const ba::Continue& c, fw::Optional<ba::Result>& result) {
        if (!solve_ || c.seq != seq_) return;
        stepOnce(result);
    }

private:
    void stepOnce(fw::Optional<ba::Result>& result) {
        const auto t0 = std::chrono::steady_clock::now();
        solve_->step();
        ms_ += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        ba::Result r;
        if (solve_->finished()) {
            r = solve_->result();
            solve_.reset();
        } else {
            r.seq = seq_;
            r.kind = kind_;
            r.iterations = solve_->iterations();
            r.partial = true;
        }
        r.solve_ms = ms_;
        result = std::move(r);
    }

    std::unique_ptr<ba_solver::SteppedSolve> solve_;
    std::uint64_t seq_ = 0;
    ba::Kind kind_ = ba::Kind::global;
    double ms_ = 0, build_ms_ = 0;
};

}  // namespace

FW_REGISTER_ALGORITHM_CLASS(GbaSolver,
    OUTPUTS(("result", ba::Result)),
    ON(("problem", ba::Problem, trigger, QUEUE(2, drop_oldest)), &GbaSolver::on_problem, EMITS("result")),
    ON(("continue", ba::Continue, trigger, QUEUE(2, drop_oldest)), &GbaSolver::on_continue, EMITS("result")));
