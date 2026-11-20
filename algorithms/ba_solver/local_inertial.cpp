#include "algorithms/ba_solver/local_inertial.hpp"

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

ba::Result solveLocalInertial(const ba::Problem& p) {
    ba::Result r;
    r.seq = p.seq;
    r.kind = p.kind;

    nls::Problem prob;
    // Each residual's loss, for activeRobustChi2
    std::vector<std::shared_ptr<const nls::Loss>> losses;
    auto add = [&](std::shared_ptr<const nls::Residual> res, std::vector<int> blocks, Eigen::MatrixXd info,
                   std::shared_ptr<const nls::Loss> loss = nullptr) {
        prob.addResidual(std::move(res), std::move(blocks), std::move(info), loss);
        losses.push_back(std::move(loss));
    };

    const auto pose_m = std::make_shared<ImuPoseManifold>();
    const std::size_t n = p.keyframes.size();
    std::vector<std::array<double, 12>> poses(n);
    std::vector<Eigen::Vector3d> vel(n), bgs(n), bas(n);
    std::vector<int> pose_b(n), vel_b(n, -1), g_b(n, -1), a_b(n, -1);
    for (std::size_t i = 0; i < n; ++i) {
        const ba::KeyFrameState& k = p.keyframes[i];
        imuPoseFrom(k.Tcw, k.T_cb, poses[i].data());
        pose_b[i] = prob.addBlock(poses[i].data(), pose_m);
        prob.setFixed(pose_b[i], k.fixed);
        if (!k.inertial) continue;
        vel[i] = k.velocity.cast<double>();
        bgs[i] = k.bias.g.cast<double>();
        bas[i] = k.bias.a.cast<double>();
        vel_b[i] = prob.addBlock(vel[i].data(), 3);
        g_b[i] = prob.addBlock(bgs[i].data(), 3);
        a_b[i] = prob.addBlock(bas[i].data(), 3);
        prob.setFixed(vel_b[i], k.fixed);
        prob.setFixed(g_b[i], k.fixed);
        prob.setFixed(a_b[i], k.fixed);
    }

    const auto inertial_loss = huber(std::sqrt(16.92));
    for (std::size_t i = 0; i < n; ++i) {
        const ba::KeyFrameState& k = p.keyframes[i];
        if (k.fixed || k.prev < 0 || !k.preintegrated) continue;
        const std::size_t j = std::size_t(k.prev);
        if (!k.inertial || !p.keyframes[j].inertial) continue;
        // The edge into the window from the fixed keyframe before it: down-weighted and robust
        const bool last = p.keyframes[j].fixed;
        Eigen::MatrixXd info = inertialInformation(*k.preintegrated);
        if (last) info *= 1e-2;
        add(std::make_shared<InertialResidual>(k.preintegrated), {pose_b[j], vel_b[j], g_b[j], a_b[j], pose_b[i], vel_b[i]},
            info, last || p.rec_init ? inertial_loss : nullptr);
        const Eigen::Matrix3d info_g = k.preintegrated->C.block<3, 3>(9, 9).cast<double>().inverse();
        const Eigen::Matrix3d info_a = k.preintegrated->C.block<3, 3>(12, 12).cast<double>().inverse();
        add(std::make_shared<BiasWalkResidual>(), {g_b[j], g_b[i]}, info_g);
        add(std::make_shared<BiasWalkResidual>(), {a_b[j], a_b[i]}, info_a);
    }
    const int first_visual = prob.numResiduals();

    std::vector<Eigen::Vector3d> points(p.points.size());
    std::vector<int> pt_b(p.points.size());
    for (std::size_t i = 0; i < p.points.size(); ++i) {
        points[i] = p.points[i].pos.cast<double>();
        pt_b[i] = prob.addBlock(points[i].data(), 3);
        prob.setEliminated(pt_b[i]);
    }
    const float th_mono = std::sqrt(5.991);
    const auto mono_loss = huber(th_mono);
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
        add(std::shared_ptr<const nls::Residual>(residuals, &residuals->back()), {pose_b[k], pt_b[std::size_t(o.point)]},
            Eigen::Matrix2d::Identity() * double(o.inv_sigma2), mono_loss);
    }

    // activeRobustChi2: Σ ρ(chi2) over the errors last computed
    auto robustChi2 = [&] {
        double sum = 0;
        for (int i = 0; i < prob.numResiduals(); ++i) {
            const double chi2 = prob.chi2(i);
            if (losses[std::size_t(i)]) {
                double rho[2];
                losses[std::size_t(i)]->evaluate(chi2, rho);
                sum += rho[0];
            } else {
                sum += chi2;
            }
        }
        return float(sum);
    };
    for (int i = 0; i < prob.numResiduals(); ++i) prob.computeError(i);
    const float err = robustChi2();
    nls::Options opt;
    opt.max_iterations = p.iterations;
    opt.initial_lambda = p.initial_lambda;
    r.iterations = prob.solve(opt).iterations;
    const float err_end = robustChi2();
    if ((2 * err < err_end || std::isnan(err) || std::isnan(err_end)) && !p.large) {
        r.ok = false;
        return r;
    }

    const float chi2_mono = 5.991f, chi2_close = 1.5f * chi2_mono;
    for (std::size_t i = 0; i < p.observations.size(); ++i) {
        const ba::Observation& o = p.observations[i];
        const double chi2 = prob.chi2(first_visual + int(i));
        const bool close = p.points[std::size_t(o.point)].close;
        const double depth =
            (*residuals)[i].depth(poses[std::size_t(o.keyframe)].data(), points[std::size_t(o.point)].data());
        if ((chi2 > chi2_mono && !close) || (chi2 > chi2_close && close) || !(depth > 0.0))
            r.outliers.push_back({p.keyframes[std::size_t(o.keyframe)].id, p.points[std::size_t(o.point)].id});
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
