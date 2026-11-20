#include "algorithms/ba_solver/inertial.hpp"

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

imu::Bias biasOf(const Eigen::Vector3d& g, const Eigen::Vector3d& a) {
    imu::Bias b;
    b.g = g.cast<float>();
    b.a = a.cast<float>();
    return b;
}

}  // namespace

ba::Result solveInertialInit(const ba::Problem& p) {
    ba::Result r;
    r.seq = p.seq;
    r.kind = p.kind;

    nls::Problem prob;
    const auto pose_m = std::make_shared<ImuPoseManifold>();
    const std::size_t n = p.keyframes.size();
    std::vector<std::array<double, 12>> poses(n);
    std::vector<Eigen::Vector3d> vel(n);
    std::vector<int> pose_b(n), vel_b(n);
    for (std::size_t i = 0; i < n; ++i) {
        const ba::KeyFrameState& k = p.keyframes[i];
        imuPoseFrom(k.Tcw, k.T_cb, poses[i].data());
        pose_b[i] = prob.addBlock(poses[i].data(), pose_m);
        prob.setFixed(pose_b[i]);
        vel[i] = k.velocity.cast<double>();
        vel_b[i] = prob.addBlock(vel[i].data(), 3);
    }
    Eigen::Vector3d bg = p.bg, ba = p.ba;
    const int g_b = prob.addBlock(bg.data(), 3), a_b = prob.addBlock(ba.data(), 3);
    prob.addResidual(std::make_shared<BiasPriorResidual>(Eigen::Vector3d::Zero()), {a_b},
                     Eigen::Matrix3d::Identity() * double(p.prior_a));
    prob.addResidual(std::make_shared<BiasPriorResidual>(Eigen::Vector3d::Zero()), {g_b},
                     Eigen::Matrix3d::Identity() * double(p.prior_g));
    Eigen::Matrix3d Rwg = p.Rwg;
    double scale = p.scale;
    const int r_b = prob.addBlock(Rwg.data(), std::make_shared<GravityDirManifold>());
    const int s_b = prob.addBlock(&scale, std::make_shared<ScaleManifold>());
    prob.setFixed(s_b);  // stereo
    for (std::size_t i = 0; i < n; ++i) {
        const ba::KeyFrameState& k = p.keyframes[i];
        if (k.prev < 0 || !k.preintegrated) continue;
        const std::size_t j = std::size_t(k.prev);
        prob.addResidual(std::make_shared<InertialGSResidual>(k.preintegrated),
                         {pose_b[j], vel_b[j], g_b, a_b, pose_b[i], vel_b[i], r_b, s_b},
                         inertialInformation(*k.preintegrated));
    }

    nls::Options opt;
    opt.max_iterations = p.iterations;
    opt.initial_lambda = p.initial_lambda;
    r.iterations = prob.solve(opt).iterations;

    r.Rwg = Rwg;
    r.scale = scale;
    r.bg = bg;
    r.ba = ba;
    const imu::Bias b = biasOf(bg, ba);
    for (std::size_t i = 0; i < n; ++i) {
        ba::KeyFrameState k;
        k.id = p.keyframes[i].id;
        k.inertial = true;
        k.velocity = vel[i].cast<float>();
        k.bias = b;
        r.keyframes.push_back(std::move(k));
    }
    r.ok = true;
    return r;
}

ba::Result solveFullInertial(const ba::Problem& p) {
    ba::Result r;
    r.seq = p.seq;
    r.kind = p.kind;

    nls::Problem prob;
    const auto pose_m = std::make_shared<ImuPoseManifold>();
    const std::size_t n = p.keyframes.size();
    std::vector<std::array<double, 12>> poses(n);
    std::vector<Eigen::Vector3d> vel(n), bgs(n), bas(n);
    std::vector<int> pose_b(n), vel_b(n, -1), g_b(n, -1), a_b(n, -1);
    for (std::size_t i = 0; i < n; ++i) {
        const ba::KeyFrameState& k = p.keyframes[i];
        imuPoseFrom(k.Tcw, k.T_cb, poses[i].data());
        pose_b[i] = prob.addBlock(poses[i].data(), pose_m);
        if (!k.inertial) continue;
        vel[i] = k.velocity.cast<double>();
        vel_b[i] = prob.addBlock(vel[i].data(), 3);
        if (p.shared_bias) continue;
        bgs[i] = k.bias.g.cast<double>();
        bas[i] = k.bias.a.cast<double>();
        g_b[i] = prob.addBlock(bgs[i].data(), 3);
        a_b[i] = prob.addBlock(bas[i].data(), 3);
    }
    // bInit: one bias, from the last keyframe (pIncKF)
    Eigen::Vector3d bg = Eigen::Vector3d::Zero(), ba = Eigen::Vector3d::Zero();
    int sg_b = -1, sa_b = -1;
    if (p.shared_bias && n > 0) {
        bg = p.keyframes.back().bias.g.cast<double>();
        ba = p.keyframes.back().bias.a.cast<double>();
        sg_b = prob.addBlock(bg.data(), 3);
        sa_b = prob.addBlock(ba.data(), 3);
    }

    const auto inertial_loss = huber(std::sqrt(16.92));
    for (std::size_t i = 0; i < n; ++i) {
        const ba::KeyFrameState& k = p.keyframes[i];
        if (k.prev < 0 || !k.preintegrated) continue;
        const std::size_t j = std::size_t(k.prev);
        if (!k.inertial || !p.keyframes[j].inertial) continue;
        const int g1 = p.shared_bias ? sg_b : g_b[j], a1 = p.shared_bias ? sa_b : a_b[j];
        prob.addResidual(std::make_shared<InertialResidual>(k.preintegrated),
                         {pose_b[j], vel_b[j], g1, a1, pose_b[i], vel_b[i]}, inertialInformation(*k.preintegrated),
                         inertial_loss);
        if (p.shared_bias) continue;
        const Eigen::Matrix3d info_g = k.preintegrated->C.block<3, 3>(9, 9).cast<double>().inverse();
        const Eigen::Matrix3d info_a = k.preintegrated->C.block<3, 3>(12, 12).cast<double>().inverse();
        prob.addResidual(std::make_shared<BiasWalkResidual>(), {g_b[j], g_b[i]}, info_g);
        prob.addResidual(std::make_shared<BiasWalkResidual>(), {a_b[j], a_b[i]}, info_a);
    }
    if (p.shared_bias && n > 0) {
        prob.addResidual(std::make_shared<BiasPriorResidual>(Eigen::Vector3d::Zero()), {sa_b},
                         Eigen::Matrix3d::Identity() * double(p.prior_a));
        prob.addResidual(std::make_shared<BiasPriorResidual>(Eigen::Vector3d::Zero()), {sg_b},
                         Eigen::Matrix3d::Identity() * double(p.prior_g));
    }

    // Points: those with an observation
    std::vector<int> n_obs(p.points.size(), 0);
    for (const ba::Observation& o : p.observations) ++n_obs[std::size_t(o.point)];
    std::vector<Eigen::Vector3d> points(p.points.size());
    std::vector<int> pt_b(p.points.size(), -1);
    for (std::size_t i = 0; i < p.points.size(); ++i) {
        if (n_obs[i] == 0) continue;
        points[i] = p.points[i].pos.cast<double>();
        pt_b[i] = prob.addBlock(points[i].data(), 3);
        prob.setEliminated(pt_b[i]);
    }
    const float th_mono = std::sqrt(5.991);
    const auto mono_loss = huber(th_mono);
    // The cameras in the body, per keyframe
    std::vector<std::array<Eigen::Matrix3d, 2>> Rcb(n);
    std::vector<std::array<Eigen::Vector3d, 2>> tcb(n);
    for (std::size_t i = 0; i < n; ++i) {
        cameraInBody(p.keyframes[i].T_cb, nullptr, Rcb[i][0], tcb[i][0]);
        cameraInBody(p.keyframes[i].T_cb, &p.keyframes[i].Trl, Rcb[i][1], tcb[i][1]);
    }
    auto residuals = std::make_shared<std::vector<BodyReprojectionResidual>>();
    residuals->reserve(p.observations.size());
    for (const ba::Observation& o : p.observations) {
        const std::size_t k = std::size_t(o.keyframe);
        const int c = o.right ? 1 : 0;
        const camera::StereoRig& rig = p.rigs[std::size_t(p.keyframes[k].rig)];
        residuals->emplace_back(rig.camera(c), Rcb[k][std::size_t(c)], tcb[k][std::size_t(c)], o.uv.cast<double>());
        prob.addResidual(std::shared_ptr<const nls::Residual>(residuals, &residuals->back()),
                         {pose_b[k], pt_b[std::size_t(o.point)]}, Eigen::Matrix2d::Identity() * double(o.inv_sigma2),
                         mono_loss);
    }

    nls::Options opt;
    opt.max_iterations = p.iterations;
    opt.initial_lambda = p.initial_lambda;
    r.iterations = prob.solve(opt).iterations;

    for (std::size_t i = 0; i < n; ++i) {
        const ba::KeyFrameState& in = p.keyframes[i];
        ba::KeyFrameState k;
        k.id = in.id;
        k.Tcw = cameraPoseOf(poses[i].data(), in.T_cb);
        k.inertial = in.inertial;
        if (in.inertial) {
            k.velocity = vel[i].cast<float>();
            k.bias = p.shared_bias ? biasOf(bg, ba) : biasOf(bgs[i], bas[i]);
        }
        r.keyframes.push_back(std::move(k));
    }
    for (std::size_t i = 0; i < p.points.size(); ++i)
        if (pt_b[i] >= 0) r.points.push_back({p.points[i].id, points[i].cast<float>(), false});
    r.ok = true;
    return r;
}

}  // namespace ba_solver
