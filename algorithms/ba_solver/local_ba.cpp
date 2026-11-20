#include "algorithms/ba_solver/local_ba.hpp"

#include <array>
#include <cmath>
#include <memory>

#include "algorithms/ba_solver/reprojection.hpp"
#include "libs/nls/problem.hpp"

namespace ba_solver {

ba::Result solveLocal(const ba::Problem& p) {
    ba::Result r;
    r.seq = p.seq;
    r.kind = p.kind;

    nls::Problem prob;
    const auto se3 = std::make_shared<nls::SE3Left>();
    std::vector<std::array<double, 7>> poses(p.keyframes.size());
    std::vector<Eigen::Vector3d> points(p.points.size());
    std::vector<int> kf_block(p.keyframes.size()), pt_block(p.points.size());
    for (std::size_t i = 0; i < p.keyframes.size(); ++i) {
        const ba::KeyFrameState& k = p.keyframes[i];
        const Eigen::Quaterniond q = k.Tcw.unitQuaternion().cast<double>();
        const Eigen::Vector3d t = k.Tcw.translation().cast<double>();
        poses[i] = {q.x(), q.y(), q.z(), q.w(), t.x(), t.y(), t.z()};
        kf_block[i] = prob.addBlock(poses[i].data(), se3);
        prob.setFixed(kf_block[i], k.fixed);
    }
    for (std::size_t i = 0; i < p.points.size(); ++i) {
        points[i] = p.points[i].pos.cast<double>();
        pt_block[i] = prob.addBlock(points[i].data(), 3);
        prob.setEliminated(pt_block[i]);
        prob.setFixed(pt_block[i], p.points[i].fixed);
    }

    // g2o's RobustKernelHuber::setDelta(float(sqrt(5.991))): the delta in double, its square kept in a float
    const float th_huber = std::sqrt(5.991);
    const double delta = th_huber;
    const auto huber = std::make_shared<nls::Huber>(delta, double(float(delta * delta)));
    // The residuals in one array (aliasing shared_ptrs into it), for the solver's sequential passes over them
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
        prob.addResidual(std::shared_ptr<const nls::Residual>(residuals, &residuals->back()),
                         {kf_block[std::size_t(o.keyframe)], pt_block[std::size_t(o.point)]},
                         Eigen::Matrix2d::Identity() * double(o.inv_sigma2), huber);
    }

    nls::Options opt;
    opt.max_iterations = p.iterations;
    opt.initial_lambda = p.initial_lambda;
    const nls::Summary s = prob.solve(opt);
    r.iterations = s.iterations;

    for (std::size_t i = 0; i < p.observations.size(); ++i) {
        const ba::Observation& o = p.observations[i];
        const double chi2 = prob.chi2(int(i));
        const double depth =
            (*residuals)[i].depth(poses[std::size_t(o.keyframe)].data(), points[std::size_t(o.point)].data());
        if (chi2 > 5.991 || !(depth > 0.0))
            r.outliers.push_back({p.keyframes[std::size_t(o.keyframe)].id, p.points[std::size_t(o.point)].id});
    }
    for (std::size_t i = 0; i < p.keyframes.size(); ++i) {
        if (p.keyframes[i].fixed) continue;
        ba::KeyFrameState k;
        k.id = p.keyframes[i].id;
        const std::array<double, 7>& x = poses[i];
        k.Tcw = geometry::SE3f(Eigen::Quaterniond(x[3], x[0], x[1], x[2]).normalized().cast<float>(),
                               Eigen::Vector3f(float(x[4]), float(x[5]), float(x[6])));
        r.keyframes.push_back(k);
    }
    for (std::size_t i = 0; i < p.points.size(); ++i) {
        if (p.points[i].fixed) continue;
        r.points.push_back({p.points[i].id, points[i].cast<float>(), false});
    }
    r.ok = true;
    return r;
}

}  // namespace ba_solver
