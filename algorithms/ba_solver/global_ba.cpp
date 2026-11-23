#include "algorithms/ba_solver/global_ba.hpp"

#include <array>
#include <cmath>
#include <utility>
#include <vector>

#include "algorithms/ba_solver/reprojection.hpp"
#include "libs/nls/problem.hpp"

namespace ba_solver {

namespace {

class GlobalSolve final : public SteppedSolve {
public:
    explicit GlobalSolve(ba::Problem in) : p_(std::move(in)) {
        const ba::Problem& p = p_;  // the residuals keep references to its rigs' cameras
        const auto se3 = std::make_shared<nls::SE3Left>();
        poses_.resize(p.keyframes.size());
        std::vector<int> kf_block(p.keyframes.size());
        for (std::size_t i = 0; i < p.keyframes.size(); ++i) {
            const ba::KeyFrameState& k = p.keyframes[i];
            const Eigen::Quaterniond q = k.Tcw.unitQuaternion().cast<double>();
            const Eigen::Vector3d t = k.Tcw.translation().cast<double>();
            poses_[i] = {q.x(), q.y(), q.z(), q.w(), t.x(), t.y(), t.z()};
            kf_block[i] = prob_.addBlock(poses_[i].data(), se3);
            prob_.setFixed(kf_block[i], k.fixed);
        }
        std::vector<int> n_obs(p.points.size(), 0);
        for (const ba::Observation& o : p.observations) ++n_obs[std::size_t(o.point)];
        points_.resize(p.points.size());
        pt_block_.assign(p.points.size(), -1);
        for (std::size_t i = 0; i < p.points.size(); ++i) {
            points_[i] = p.points[i].pos.cast<double>();
            // nEdges counts the observing keyframes, not the edges: a point whose observations were all dropped (the
            // right index check) stays in, unchanged, and is written back as optimized (mnBAGlobalForKF)
            if (n_obs[i] == 0) continue;
            pt_block_[i] = prob_.addBlock(points_[i].data(), 3);
            prob_.setEliminated(pt_block_[i]);
        }

        // thHuber2D = sqrt(5.99) in a float; g2o keeps the square in a float too
        const float th_huber = float(std::sqrt(5.99));
        const double delta = th_huber;
        const auto huber = std::make_shared<nls::Huber>(delta, double(float(delta * delta)));
        auto residuals = std::make_shared<std::vector<Kb8Reprojection>>();
        residuals->reserve(p.observations.size());
        for (const ba::Observation& o : p.observations) {
            const ba::KeyFrameState& k = p.keyframes[std::size_t(o.keyframe)];
            const camera::StereoRig& rig = p.rigs[std::size_t(k.rig)];
            Eigen::Quaterniond q_rl = Eigen::Quaterniond::Identity();
            Eigen::Vector3d t_rl = Eigen::Vector3d::Zero();
            if (o.right) {
                q_rl = k.Trl.unitQuaternion().cast<double>();
                t_rl = k.Trl.translation().cast<double>();
            }
            residuals->emplace_back(rig.camera(o.right ? 1 : 0), q_rl, t_rl, o.uv.cast<double>());
            prob_.addResidual(std::shared_ptr<const nls::Residual>(residuals, &residuals->back()),
                              {kf_block[std::size_t(o.keyframe)], pt_block_[std::size_t(o.point)]},
                              Eigen::Matrix2d::Identity() * double(o.inv_sigma2), o.right ? huber : nullptr);
        }

        nls::Options opt;
        opt.max_iterations = p.iterations;
        opt.initial_lambda = p.initial_lambda;
        prob_.start(opt);
    }

    void step() override { prob_.step(); }
    bool finished() const override { return prob_.finished(); }
    int iterations() const override { return prob_.summary().iterations; }

    ba::Result result() const override {
        const ba::Problem& p = p_;
        ba::Result r;
        r.seq = p.seq;
        r.kind = p.kind;
        r.iterations = iterations();
        for (std::size_t i = 0; i < p.keyframes.size(); ++i) {
            ba::KeyFrameState k;
            k.id = p.keyframes[i].id;
            const std::array<double, 7>& x = poses_[i];
            k.Tcw = geometry::SE3f(Eigen::Quaterniond(x[3], x[0], x[1], x[2]).normalized().cast<float>(),
                                   Eigen::Vector3f(float(x[4]), float(x[5]), float(x[6])));
            r.keyframes.push_back(k);
        }
        for (std::size_t i = 0; i < p.points.size(); ++i)
            r.points.push_back({p.points[i].id, points_[i].cast<float>(), false});
        r.ok = true;
        return r;
    }

private:
    ba::Problem p_;
    nls::Problem prob_;
    std::vector<std::array<double, 7>> poses_;
    std::vector<Eigen::Vector3d> points_;
    std::vector<int> pt_block_;
};

}  // namespace

std::unique_ptr<SteppedSolve> startGlobal(ba::Problem p) { return std::make_unique<GlobalSolve>(std::move(p)); }

ba::Result solveGlobal(const ba::Problem& p) { return solveAll(*startGlobal(p)); }

}  // namespace ba_solver
