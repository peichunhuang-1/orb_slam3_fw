#include "algorithms/ba_solver/merge_inertial.hpp"

#include <array>
#include <cmath>
#include <memory>

#include "algorithms/ba_solver/inertial_residuals.hpp"
#include "libs/nls/problem.hpp"

namespace ba_solver {

namespace {

// g2o's RobustKernelHuber: the delta in double, its square kept in a float
std::shared_ptr<const nls::Huber> huber(double delta) {
    return std::make_shared<nls::Huber>(delta, double(float(delta * delta)));
}

}  // namespace

ba::Result solveMergeInertial(const ba::Problem& p) {
    ba::Result r;
    r.seq = p.seq;
    r.kind = p.kind;

    const std::size_t n = p.keyframes.size();
    // Which blocks an edge uses: the others stay out of the optimization (held fixed)
    std::vector<bool> pose_used(n, false), imu_used(n, false);
    std::vector<bool> has_edge(n, false);
    for (std::size_t i = 0; i < n; ++i) {
        const ba::KeyFrameState& k = p.keyframes[i];
        if (k.prev < 0 || !k.preintegrated || !k.inertial || !p.keyframes[std::size_t(k.prev)].inertial) continue;
        has_edge[i] = true;
        pose_used[i] = pose_used[std::size_t(k.prev)] = true;
        imu_used[i] = imu_used[std::size_t(k.prev)] = true;
    }
    for (const ba::Observation& o : p.observations) pose_used[std::size_t(o.keyframe)] = true;

    nls::Problem prob;
    const auto pose_m = std::make_shared<ImuPoseManifold>();
    std::vector<std::array<double, 12>> poses(n);
    std::vector<Eigen::Vector3d> vel(n), bgs(n), bas(n);
    std::vector<int> pose_b(n), vel_b(n, -1), g_b(n, -1), a_b(n, -1);
    for (std::size_t i = 0; i < n; ++i) {
        const ba::KeyFrameState& k = p.keyframes[i];
        imuPoseFrom(k.Tcw, k.T_cb, poses[i].data());
        pose_b[i] = prob.addBlock(poses[i].data(), pose_m);
        prob.setFixed(pose_b[i], k.fixed || !pose_used[i]);
        if (!k.inertial) continue;
        vel[i] = k.velocity.cast<double>();
        bgs[i] = k.bias.g.cast<double>();
        bas[i] = k.bias.a.cast<double>();
        vel_b[i] = prob.addBlock(vel[i].data(), 3);
        g_b[i] = prob.addBlock(bgs[i].data(), 3);
        a_b[i] = prob.addBlock(bas[i].data(), 3);
        const bool fixed = k.fixed || !imu_used[i];
        prob.setFixed(vel_b[i], fixed);
        prob.setFixed(g_b[i], fixed);
        prob.setFixed(a_b[i], fixed);
    }

    const auto inertial_loss = huber(std::sqrt(16.92));
    for (std::size_t i = 0; i < n; ++i) {
        if (!has_edge[i]) continue;
        const ba::KeyFrameState& k = p.keyframes[i];
        const std::size_t j = std::size_t(k.prev);
        prob.addResidual(std::make_shared<InertialResidual>(k.preintegrated),
                         {pose_b[j], vel_b[j], g_b[j], a_b[j], pose_b[i], vel_b[i]}, inertialInformation(*k.preintegrated),
                         inertial_loss);
        const Eigen::Matrix3d info_g = k.preintegrated->C.block<3, 3>(9, 9).cast<double>().inverse();
        const Eigen::Matrix3d info_a = k.preintegrated->C.block<3, 3>(12, 12).cast<double>().inverse();
        prob.addResidual(std::make_shared<BiasWalkResidual>(), {g_b[j], g_b[i]}, info_g);
        prob.addResidual(std::make_shared<BiasWalkResidual>(), {a_b[j], a_b[i]}, info_a);
    }
    const int first_visual = prob.numResiduals();

    std::vector<Eigen::Vector3d> points(p.points.size());
    std::vector<int> pt_b(p.points.size());
    for (std::size_t i = 0; i < p.points.size(); ++i) {
        points[i] = p.points[i].pos.cast<double>();
        pt_b[i] = prob.addBlock(points[i].data(), 3);
        prob.setEliminated(pt_b[i]);
    }
    const float th_mono = std::sqrt(5.991);  // thHuberMono, a float
    const auto mono_loss = huber(th_mono);
    std::vector<Eigen::Matrix3d> Rcb(n);
    std::vector<Eigen::Vector3d> tcb(n);
    for (std::size_t i = 0; i < n; ++i) cameraInBody(p.keyframes[i].T_cb, nullptr, Rcb[i], tcb[i]);
    auto residuals = std::make_shared<std::vector<BodyReprojectionResidual>>();
    residuals->reserve(p.observations.size());
    for (const ba::Observation& o : p.observations) {
        const std::size_t k = std::size_t(o.keyframe);
        const camera::StereoRig& rig = p.rigs[std::size_t(p.keyframes[k].rig)];
        residuals->emplace_back(rig.camera(0), Rcb[k], tcb[k], o.uv.cast<double>());
        prob.addResidual(std::shared_ptr<const nls::Residual>(residuals, &residuals->back()),
                         {pose_b[k], pt_b[std::size_t(o.point)]}, Eigen::Matrix2d::Identity() * double(o.inv_sigma2),
                         mono_loss);
    }

    nls::Options opt;
    opt.max_iterations = p.iterations;
    opt.initial_lambda = p.initial_lambda;
    r.iterations = prob.solve(opt).iterations;

    for (std::size_t i = 0; i < p.observations.size(); ++i)
        if (prob.chi2(first_visual + int(i)) > 5.991) {
            const ba::Observation& o = p.observations[i];
            r.outliers.push_back({p.keyframes[std::size_t(o.keyframe)].id, p.points[std::size_t(o.point)].id});
        }

    bool finite = true;
    for (std::size_t i = 0; i < n; ++i) {
        if (p.keyframes[i].fixed) continue;
        for (double x : poses[i]) finite = finite && std::isfinite(x);
        if (p.keyframes[i].inertial) finite = finite && vel[i].allFinite() && bgs[i].allFinite() && bas[i].allFinite();
    }
    for (const Eigen::Vector3d& x : points) finite = finite && x.allFinite();
    if (!finite) {
        r.ok = false;
        return r;
    }

    for (std::size_t i = 0; i < n; ++i) {
        const ba::KeyFrameState& in = p.keyframes[i];
        if (in.fixed) continue;
        ba::KeyFrameState k;
        k.id = in.id;
        k.Tcw = cameraPoseOf(poses[i].data(), in.T_cb);
        k.inertial = in.inertial;
        if (in.inertial) {
            k.velocity = vel[i].cast<float>();
            k.bias.g = bgs[i].cast<float>();
            k.bias.a = bas[i].cast<float>();
        }
        r.keyframes.push_back(std::move(k));
    }
    for (std::size_t i = 0; i < p.points.size(); ++i) r.points.push_back({p.points[i].id, points[i].cast<float>(), false});
    r.ok = true;
    return r;
}

}  // namespace ba_solver
