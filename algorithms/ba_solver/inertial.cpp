#include "algorithms/ba_solver/inertial.hpp"

#include <array>
#include <cmath>
#include <memory>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

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

// The gauge of an inertial problem with nothing fixed (FullInertialBA, bFixLocal = false): the world's position
// and its yaw about gravity (z) are unobservable, so LM's steps along them follow rounding, not the measurements
// (seen: a loop's global BA lifted a whole room4 map by 0.31 m and turned it 1.6°, jumping the online trajectory).
// The 4-dof motion (yaw about z, translation) that best takes the solution's keyframe centers back onto the
// input's, least squares: applied to the solution, it leaves every residual as it was.
struct Gauge {
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t = Eigen::Vector3d::Zero();
};

Gauge gaugeBack(const std::vector<Eigen::Vector3d>& before, const std::vector<Eigen::Vector3d>& after) {
    Gauge g;
    const std::size_t n = before.size();
    if (n == 0) return g;
    Eigen::Vector3d mb = Eigen::Vector3d::Zero(), ma = Eigen::Vector3d::Zero();
    for (std::size_t i = 0; i < n; ++i) {
        mb += before[i];
        ma += after[i];
    }
    mb /= double(n);
    ma /= double(n);
    double sdot = 0, scross = 0;  // the 2-d Umeyama rotation in the horizontal plane
    for (std::size_t i = 0; i < n; ++i) {
        const Eigen::Vector3d a = after[i] - ma, b = before[i] - mb;
        sdot += a.x() * b.x() + a.y() * b.y();
        scross += a.x() * b.y() - a.y() * b.x();
    }
    const double yaw = n > 1 ? std::atan2(scross, sdot) : 0.0;
    g.R = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    g.t = mb - g.R * ma;
    return g;
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
    prob.setFixed(r_b, p.fixed_gravity);
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

namespace {

// FullInertialBA's problem, kept for solving in steps; its blocks point into the members (so it does not move)
class FullInertialSolve final : public SteppedSolve {
public:
    explicit FullInertialSolve(ba::Problem in) : p_(std::move(in)) {
        const ba::Problem& p = p_;  // the residuals keep references to its rigs' cameras
        const std::size_t n = p.keyframes.size();
        poses_.resize(n);
        vel_.resize(n);
        bgs_.resize(n);
        bas_.resize(n);
        std::vector<int> pose_b(n), vel_b(n, -1), g_b(n, -1), a_b(n, -1);
        const auto pose_m = std::make_shared<ImuPoseManifold>();
        for (std::size_t i = 0; i < n; ++i) {
            const ba::KeyFrameState& k = p.keyframes[i];
            imuPoseFrom(k.Tcw, k.T_cb, poses_[i].data());
            pose_b[i] = prob_.addBlock(poses_[i].data(), pose_m);
            if (!k.inertial) continue;
            vel_[i] = k.velocity.cast<double>();
            vel_b[i] = prob_.addBlock(vel_[i].data(), 3);
            if (p.shared_bias) continue;
            bgs_[i] = k.bias.g.cast<double>();
            bas_[i] = k.bias.a.cast<double>();
            g_b[i] = prob_.addBlock(bgs_[i].data(), 3);
            a_b[i] = prob_.addBlock(bas_[i].data(), 3);
        }
        // bInit: one bias, from the last keyframe (pIncKF)
        int sg_b = -1, sa_b = -1;
        if (p.shared_bias && n > 0) {
            bg_ = p.keyframes.back().bias.g.cast<double>();
            ba_ = p.keyframes.back().bias.a.cast<double>();
            sg_b = prob_.addBlock(bg_.data(), 3);
            sa_b = prob_.addBlock(ba_.data(), 3);
        }

        const auto inertial_loss = huber(std::sqrt(16.92));
        for (std::size_t i = 0; i < n; ++i) {
            const ba::KeyFrameState& k = p.keyframes[i];
            if (k.prev < 0 || !k.preintegrated) continue;
            const std::size_t j = std::size_t(k.prev);
            if (!k.inertial || !p.keyframes[j].inertial) continue;
            const int g1 = p.shared_bias ? sg_b : g_b[j], a1 = p.shared_bias ? sa_b : a_b[j];
            prob_.addResidual(std::make_shared<InertialResidual>(k.preintegrated),
                              {pose_b[j], vel_b[j], g1, a1, pose_b[i], vel_b[i]}, inertialInformation(*k.preintegrated),
                              inertial_loss);
            if (p.shared_bias) continue;
            const Eigen::Matrix3d info_g = k.preintegrated->C.block<3, 3>(9, 9).cast<double>().inverse();
            const Eigen::Matrix3d info_a = k.preintegrated->C.block<3, 3>(12, 12).cast<double>().inverse();
            prob_.addResidual(std::make_shared<BiasWalkResidual>(), {g_b[j], g_b[i]}, info_g);
            prob_.addResidual(std::make_shared<BiasWalkResidual>(), {a_b[j], a_b[i]}, info_a);
        }
        if (p.shared_bias && n > 0) {
            prob_.addResidual(std::make_shared<BiasPriorResidual>(Eigen::Vector3d::Zero()), {sa_b},
                              Eigen::Matrix3d::Identity() * double(p.prior_a));
            prob_.addResidual(std::make_shared<BiasPriorResidual>(Eigen::Vector3d::Zero()), {sg_b},
                              Eigen::Matrix3d::Identity() * double(p.prior_g));
        }

        // Points: those with an observation
        std::vector<int> n_obs(p.points.size(), 0);
        for (const ba::Observation& o : p.observations) ++n_obs[std::size_t(o.point)];
        points_.resize(p.points.size());
        pt_b_.assign(p.points.size(), -1);
        for (std::size_t i = 0; i < p.points.size(); ++i) {
            if (n_obs[i] == 0) continue;
            points_[i] = p.points[i].pos.cast<double>();
            pt_b_[i] = prob_.addBlock(points_[i].data(), 3);
            prob_.setEliminated(pt_b_[i]);
        }
        const float th_mono = std::sqrt(5.991);
        const auto mono_loss = huber(th_mono);
        // The cameras in the body, per keyframe
        Rcb_.resize(n);
        tcb_.resize(n);
        for (std::size_t i = 0; i < n; ++i) {
            cameraInBody(p.keyframes[i].T_cb, nullptr, Rcb_[i][0], tcb_[i][0]);
            cameraInBody(p.keyframes[i].T_cb, &p.keyframes[i].Trl, Rcb_[i][1], tcb_[i][1]);
        }
        auto residuals = std::make_shared<std::vector<BodyReprojectionResidual>>();
        residuals->reserve(p.observations.size());
        for (const ba::Observation& o : p.observations) {
            const std::size_t k = std::size_t(o.keyframe);
            const int c = o.right ? 1 : 0;
            const camera::StereoRig& rig = p.rigs[std::size_t(p.keyframes[k].rig)];
            residuals->emplace_back(rig.camera(c), Rcb_[k][std::size_t(c)], tcb_[k][std::size_t(c)], o.uv.cast<double>());
            prob_.addResidual(std::shared_ptr<const nls::Residual>(residuals, &residuals->back()),
                              {pose_b[k], pt_b_[std::size_t(o.point)]},
                              Eigen::Matrix2d::Identity() * double(o.inv_sigma2), mono_loss);
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
        std::vector<geometry::SE3d> Tcw(p.keyframes.size());
        std::vector<Eigen::Vector3d> before(p.keyframes.size()), after(p.keyframes.size());
        for (std::size_t i = 0; i < p.keyframes.size(); ++i) {
            Tcw[i] = cameraPoseOf(poses_[i].data(), p.keyframes[i].T_cb).cast<double>();
            before[i] = p.keyframes[i].Tcw.inverse().translation().cast<double>();
            after[i] = Tcw[i].inverse().translation();
        }
        const Gauge g = gaugeBack(before, after);  // world_in <- world_solved
        const geometry::SE3d A_inv = geometry::SE3d(Eigen::Quaterniond(g.R), g.t).inverse();
        for (std::size_t i = 0; i < p.keyframes.size(); ++i) {
            const ba::KeyFrameState& in = p.keyframes[i];
            ba::KeyFrameState k;
            k.id = in.id;
            k.Tcw = (Tcw[i] * A_inv).cast<float>();
            k.inertial = in.inertial;
            if (in.inertial) {
                k.velocity = (g.R * vel_[i]).cast<float>();
                k.bias = p.shared_bias ? biasOf(bg_, ba_) : biasOf(bgs_[i], bas_[i]);
            }
            r.keyframes.push_back(std::move(k));
        }
        for (std::size_t i = 0; i < p.points.size(); ++i)
            if (pt_b_[i] >= 0) r.points.push_back({p.points[i].id, (g.R * points_[i] + g.t).cast<float>(), false});
        r.ok = true;
        return r;
    }

private:
    ba::Problem p_;  // the preintegrations and rigs the residuals refer to
    nls::Problem prob_;
    std::vector<std::array<double, 12>> poses_;
    std::vector<Eigen::Vector3d> vel_, bgs_, bas_;
    Eigen::Vector3d bg_ = Eigen::Vector3d::Zero(), ba_ = Eigen::Vector3d::Zero();
    std::vector<Eigen::Vector3d> points_;
    std::vector<int> pt_b_;
    std::vector<std::array<Eigen::Matrix3d, 2>> Rcb_;
    std::vector<std::array<Eigen::Vector3d, 2>> tcb_;
};

}  // namespace

std::unique_ptr<SteppedSolve> startFullInertial(ba::Problem p) {
    return std::make_unique<FullInertialSolve>(std::move(p));
}

ba::Result solveFullInertial(const ba::Problem& p) { return solveAll(*startFullInertial(p)); }

}  // namespace ba_solver
