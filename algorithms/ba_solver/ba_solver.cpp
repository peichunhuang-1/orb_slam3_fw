// ba_solver: solves the bundle adjustment problems local mapping copies out of the atlas (ba/Problem -> ba/Result),
// without touching any resource (algorithms/orb_local_mapping/docs/DESIGN.md, "鎖外計算", "ba_solver").
//   local          Optimizer::LocalBundleAdjustment
//   local_inertial Optimizer::LocalInertialBA
//   inertial_init  Optimizer::InertialOptimization
//   full_inertial  Optimizer::FullInertialBA
// `local` (local_ba), `inertial_init` and `full_inertial` (inertial) are solved; `local_inertial` is not yet: its
// result echoes the problem's state.
#include <chrono>

#include "algorithms/ba_solver/inertial.hpp"
#include "algorithms/ba_solver/local_ba.hpp"
#include "fw/fw.hpp"
#include "types/ba/problem.hpp"

namespace {

class BaSolver {
public:
    void on_problem(const ba::Problem& p, ba::Result& result) {
        const auto t0 = std::chrono::steady_clock::now();
        ba::Result r;
        if (p.kind == ba::Kind::local) {
            r = ba_solver::solveLocal(p);
        } else if (p.kind == ba::Kind::inertial_init) {
            r = ba_solver::solveInertialInit(p);
        } else if (p.kind == ba::Kind::full_inertial) {
            r = ba_solver::solveFullInertial(p);
        } else {
            r.seq = p.seq;
            r.kind = p.kind;
            r.keyframes = p.keyframes;  // TODO local_inertial
            for (auto& k : r.keyframes) k.preintegrated.reset();
            r.points = p.points;
            r.Rwg = p.Rwg;
            r.scale = p.scale;
            r.bg = p.bg;
            r.ba = p.ba;
            r.ok = true;
        }
        r.solve_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        result = std::move(r);
        ++solved_;
    }

private:
    long solved_ = 0;
};

}  // namespace

FW_REGISTER_ALGORITHM_CLASS(BaSolver,
    OUTPUTS(("result", ba::Result)),
    ON(("problem", ba::Problem, trigger, QUEUE(2, drop_oldest)), &BaSolver::on_problem, EMITS("result")));
