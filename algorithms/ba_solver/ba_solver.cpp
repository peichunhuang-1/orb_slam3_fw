// ba_solver: solves the bundle adjustment problems local mapping copies out of the atlas (ba/Problem -> ba/Result),
// without touching any resource (algorithms/orb_local_mapping/docs/DESIGN.md, "鎖外計算", "ba_solver").
//   local          Optimizer::LocalBundleAdjustment
//   local_inertial Optimizer::LocalInertialBA
//   inertial_init  Optimizer::InertialOptimization
//   full_inertial  Optimizer::FullInertialBA
//   pose_graph, pose_graph_4dof  Optimizer::OptimizeEssentialGraph (6DoF), OptimizeEssentialGraph4DoF
//   global         Optimizer::BundleAdjustment
// local_ba.hpp, local_inertial.hpp, inertial.hpp, pose_graph.hpp, global_ba.hpp. Each problem is solved to the end;
// the global BA of loop closing goes to a second instance of the solvers, gba_solver (gba_solver.cpp), in steps.
#include <chrono>

#include "algorithms/ba_solver/global_ba.hpp"
#include "algorithms/ba_solver/inertial.hpp"
#include "algorithms/ba_solver/local_ba.hpp"
#include "algorithms/ba_solver/local_inertial.hpp"
#include "algorithms/ba_solver/pose_graph.hpp"
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
        } else if (p.kind == ba::Kind::local_inertial) {
            r = ba_solver::solveLocalInertial(p);
        } else if (p.kind == ba::Kind::inertial_init) {
            r = ba_solver::solveInertialInit(p);
        } else if (p.kind == ba::Kind::pose_graph) {
            r = ba_solver::solvePoseGraph(p);
        } else if (p.kind == ba::Kind::pose_graph_4dof) {
            r = ba_solver::solvePoseGraph4DoF(p);
        } else if (p.kind == ba::Kind::global) {
            r = ba_solver::solveGlobal(p);
        } else {
            r = ba_solver::solveFullInertial(p);
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
