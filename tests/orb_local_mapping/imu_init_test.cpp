// initialize_imu/imu_init + ba_solver's inertial_init and full_inertial vs tests/fixtures/orb_slam3/imu_init
// (README.md, LocalMapping::InitializeIMU): the map is rebuilt from the fixture and InitializeIMU's steps are run in
// order, each compared with the original's state after it: the gravity / velocity guess, InertialOptimization,
// its write-back and ApplyScaledRotation, FullInertialBA (with the extra keyframe queued meanwhile and its own new
// points), its processing and the correction down the spanning tree.
//
// Each step after an optimization starts from the original's result (read from the fixture), so the write-backs
// and the correction are compared tightly. The optimizations themselves can only agree loosely: InertialOptimization
// ends on a flat cost (both sides stop at costs within 1e-5 of each other, after a different number of iterations,
// with the accelerometer bias up to 5e-4 apart, well inside its prior's 3e-3); FullInertialBA has no fixed vertex,
// so its translation and yaw are unobservable, and once LM's lambda is down to ~1e-10 rounding moves the whole map
// along them (0.15 m in viba2), as do the depths of points with a single observation. So its result is compared
// after the best rigid alignment of the keyframe centres, on points with two or more observations, relative to
// their distance (a few points with outlier observations end kilometres away on both sides). The spanning
// tree (each keyframe's parent), which preintegrations were re-integrated and which points the full BA left out
// must match exactly.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

#include <Eigen/Geometry>

#include "algorithms/ba_solver/inertial.hpp"
#include "algorithms/orb_local_mapping/initialize_imu/imu_init.hpp"
#include "algorithms/orb_local_mapping/shared/process_keyframe.hpp"
#include "tests/orb_local_mapping/fixture_keyframes.hpp"

namespace {

int failures = 0;
double worst_pose = 0, worst_vel = 0, worst_bias = 0, worst_point = 0;  // the write-backs and the correction
double io_worst = 0, fiba_worst = 0;                                      // the optimizations

void check(bool ok, const std::string& what) {
    if (!ok) {
        ++failures;
        std::printf("FAIL %s\n", what.c_str());
    }
}

void near(double err, double tol, const std::string& what, double* worst = nullptr) {
    if (worst) *worst = std::max(*worst, err);
    if (!(err <= tol)) {
        ++failures;
        std::printf("FAIL %s: %.3g > %.3g\n", what.c_str(), err, tol);
    }
}

imu::Noise noise() {
    const fixture::File r("rig.txt");
    const Eigen::MatrixXd& d = r["imu_calib_discrete"];
    return imu::Noise::fromDiscrete(float(d(0)), float(d(1)), float(d(2)), float(d(3)));
}

imu::Bias biasOf(const Eigen::MatrixXd& v) {
    imu::Bias b;
    b.g = Eigen::Vector3f(float(v(0)), float(v(1)), float(v(2)));
    b.a = Eigen::Vector3f(float(v(3)), float(v(4)), float(v(5)));
    return b;
}

double biasErr(const imu::Bias& b, const Eigen::MatrixXd& v) {
    return std::max((b.g - biasOf(v).g).norm(), (b.a - biasOf(v).a).norm());
}

std::shared_ptr<const imu::Preintegrated> integrate(const Eigen::MatrixXd& m, const imu::Bias& b,
                                                    const imu::Noise& n) {
    auto p = std::make_shared<imu::Preintegrated>(b, n);
    for (int i = 0; i < m.rows(); ++i)
        p->integrate(Eigen::Vector3f(float(m(i, 0)), float(m(i, 1)), float(m(i, 2))),
                     Eigen::Vector3f(float(m(i, 3)), float(m(i, 4)), float(m(i, 5))), float(m(i, 6)));
    return p;
}

double poseErr(const geometry::SE3f& T, const Eigen::MatrixXd& want) {
    const geometry::SE3f W = fixture::se3f(want);
    const double dt = (T.inverse().translation() - W.inverse().translation()).norm();
    const double dr = T.unitQuaternion().angularDistance(W.unitQuaternion());
    return std::max(dt, dr);
}

// The extra keyframe's data, IMU state and features; queued (in the map's id range but not inserted)
map::KeyFrameId makeExtra(map::Atlas& a, const fixture::File& f, const fixture::Rig& rig, int X,
                          map::KeyFrameId prev, const imu::Noise& n) {
    const std::string p = "kf" + std::to_string(X) + "_";
    auto d = std::make_shared<map::KeyFrameData>();
    d->frame_id = X;
    d->timestamp = f.scalar(p + "timestamp");
    d->features = fixture::keyframeFeatures(f, p, rig);
    d->rig = 0;
    d->Tlr = rig.Tlr;
    d->Trl = rig.Tlr.inverse();
    d->baseline = rig.Tlr.translation().norm();
    d->th_depth = rig.th_depth;
    d->has_imu_calib = true;
    d->T_cb = rig.T_bc.inverse();
    d->bow[0] = 1;  // so ProcessNewKeyFrame's ComputeBoW has nothing to do
    d->feat[0].push_back(0);
    map::KeyFrame kf;
    kf.data = std::move(d);
    kf.setPose(fixture::se3f(f["x_Tcw"]));
    kf.setVelocity(f.f<3, 1>("x_velocity"));
    kf.bias = biasOf(f["x_bias"]);
    kf.preintegrated = integrate(f[p + "meas"], kf.bias, n);
    kf.imu = true;
    kf.prev = prev;
    const map::KeyFrameId id = map::createKeyFrame(a, std::move(kf));
    a.editKeyFrame(prev)->next = id;
    return id;
}

void runCase(const std::string& name, const fixture::Rig& rig) {
    const fixture::File f("imu_init/" + name + ".txt");
    const imu::Noise n = noise();
    const int K = int(f.scalar("n_keyframes")), cur = K - 1, X = K;
    const bool first = f.scalar("first") != 0;
    const float prior_g = float(f.scalar("prior_g")), prior_a = float(f.scalar("prior_a"));

    // ---- the map
    map::Atlas a;
    const std::vector<map::KeyFrameId> ids = fixture::addKeyFrames(a, f, rig, K);
    {
        map::Map& M = a.editCurrentMap();
        M.inertial = true;
        M.imu_initialized = !first;
        M.keyframe_origins = {ids[0]};
    }
    for (int k = 0; k < K; ++k) {
        const std::string p = "kf" + std::to_string(k) + "_";
        map::KeyFrame* kf = a.editKeyFrame(ids[std::size_t(k)]);
        auto d = std::make_shared<map::KeyFrameData>(*kf->data);
        d->has_imu_calib = true;
        d->T_cb = rig.T_bc.inverse();
        kf->data = std::move(d);
        kf->setPose(kf->pose());
        kf->bias = biasOf(f[p + "bias"]);
        kf->preintegrated = integrate(f[p + "meas"], kf->bias, n);  // KF0's: empty
        if (f.scalar(p + "has_velocity") != 0) kf->setVelocity(f.f<3, 1>(p + "velocity"));
        kf->imu = !first;
    }
    const Eigen::MatrixXd& pts = f["points"];
    const int P = int(pts.rows());
    std::vector<map::MapPointId> pid;
    for (int r = 0; r < P; ++r)
        pid.push_back(map::createMapPoint(a, pts.block<1, 3>(r, 1).transpose().cast<float>(), ids[std::size_t(pts(r, 0))]));
    for (int k = 0; k < K; ++k) {
        const Eigen::MatrixXd& kp = f["kf" + std::to_string(k) + "_points"];
        for (int i = 0; i < kp.rows(); ++i) {
            if (kp(i) < 0) continue;
            const map::MapPointId p = pid[std::size_t(kp(i))];
            map::addObservation(a, p, ids[std::size_t(k)], i);
            a.editKeyFrame(ids[std::size_t(k)])->points[std::size_t(i)] = p;
        }
        map::updateConnections(a, ids[std::size_t(k)]);
    }
    for (const map::MapPointId p : pid) {
        map::computeDistinctiveDescriptors(a, p);
        map::updateNormalAndDepth(a, p);
        map::insertMapPoint(a, p);
    }
    auto kfIndex = [&](map::KeyFrameId id) { return int(std::find(ids.begin(), ids.end(), id) - ids.begin()); };
    {
        const Eigen::MatrixXd& parent = f["parent"];
        bool same = true;
        for (int k = 0; k < K; ++k) {
            const map::KeyFrameId par = a.keyframe(ids[std::size_t(k)])->parent;
            same &= (par == map::kNone ? -1 : kfIndex(par)) == int(parent(k));
        }
        check(same, name + ": spanning tree");
    }
    const map::MapId M = a.current;

    local_mapping::LocalMapper m;
    m.inertial = true;
    m.current = ids[std::size_t(cur)];

    // ---- stage 1: due, guess, InertialOptimization
    std::vector<map::KeyFrameId> vpKF;
    double first_ts = 0;
    check(orb_local_mapping::imuInitDue(a, m, vpKF, first_ts), name + ": due");
    check(vpKF == ids, name + ": keyframes in temporal order");
    ba::Problem p1 = orb_local_mapping::inertialInitProblem(a, m, vpKF, first_ts, prior_g, prior_a);
    near((m.Rwg - f["guess_Rwg"]).norm(), 1e-6, name + ": guess Rwg");
    near(std::abs(m.t_init - f.scalar("guess_t_init")), 1e-6, name + ": mTinit");
    for (int k = 0; k < K; ++k)
        near((a.keyframe(ids[std::size_t(k)])->velocity() - f.f<3, 1>("guess_kf" + std::to_string(k) + "_velocity")).norm(),
             1e-5, name + ": guess velocity " + std::to_string(k));
    check(p1.keyframes.size() == std::size_t(K), name + ": inertial_init keyframes");
    const ba::Result r1 = ba_solver::solveInertialInit(p1);
    std::printf("%s: InertialOptimization %d iterations\n", name.c_str(), r1.iterations);
    near((r1.Rwg - f["io_Rwg"]).norm(), 2e-4, name + ": io Rwg");
    near(std::abs(r1.scale - f.scalar("io_scale")), 1e-9, name + ": io scale");
    near((r1.bg - f["io_bg"]).norm(), 2e-5, name + ": io bg", &io_worst);
    near((r1.ba - f["io_ba"]).norm(), 1e-3, name + ": io ba", &io_worst);
    for (const ba::KeyFrameState& k : r1.keyframes)
        near((k.velocity - f.f<3, 1>("io_kf" + std::to_string(kfIndex(k.id)) + "_velocity")).norm(), 1e-3,
             name + ": io velocity", &io_worst);

    // The original's result, for the next steps
    ba::Result r1_ref = r1;
    r1_ref.Rwg = f["io_Rwg"];
    r1_ref.scale = f.scalar("io_scale");
    r1_ref.bg = f["io_bg"];
    r1_ref.ba = f["io_ba"];
    for (ba::KeyFrameState& k : r1_ref.keyframes) {
        const std::string p = "io_kf" + std::to_string(kfIndex(k.id)) + "_";
        k.velocity = f.f<3, 1>(p + "velocity");
        k.bias = biasOf(f[p + "bias"]);
    }

    // ---- stage 2: write-back, ApplyScaledRotation, then FullInertialBA
    check(orb_local_mapping::applyInertialInit(a, m, M, vpKF, r1_ref), name + ": scale ok");
    check(m.imu_update_seq == 1 && m.imu_keyframe == ids[std::size_t(cur)], name + ": UpdateFrameIMU");
    check(a.findMap(M)->imu_initialized, name + ": IMU initialized");
    for (int k = 0; k < K; ++k) {
        const std::string p = "io_kf" + std::to_string(k) + "_";
        const map::KeyFrame& kf = *a.keyframe(ids[std::size_t(k)]);
        near(biasErr(kf.bias, f[p + "bias"]), 1e-7, name + ": io bias " + std::to_string(k));
        const bool reint = f.scalar(p + "reintegrated") != 0;
        const bool got = kf.preintegrated->bias().g != biasOf(f["kf" + std::to_string(k) + "_bias"]).g ||
                         kf.preintegrated->bias().a != biasOf(f["kf" + std::to_string(k) + "_bias"]).a;
        check(reint == got, name + ": re-integrated " + std::to_string(k));
        near(biasErr(kf.preintegrated->bias(), f[p + "lin_bias"]), 1e-7, name + ": linearization bias " + std::to_string(k));
        check(kf.imu, name + ": bImu " + std::to_string(k));
        near(poseErr(kf.pose(), f["asr_kf" + std::to_string(k) + "_Tcw"]), 1e-5, name + ": asr pose " + std::to_string(k),
             &worst_pose);
        near((kf.velocity() - f.f<3, 1>("asr_kf" + std::to_string(k) + "_velocity")).norm(), 1e-5,
             name + ": asr velocity " + std::to_string(k), &worst_vel);
    }
    {
        const Eigen::MatrixXd& asr = f["asr_points"];
        double e = 0, en = 0;
        for (int r = 0; r < P; ++r) {
            const map::MapPoint& mp = *a.point(pid[std::size_t(r)]);
            e = std::max(e, double((mp.pos - asr.block<1, 3>(r, 0).transpose().cast<float>()).norm()));
            en = std::max(en, double((mp.normal - asr.block<1, 3>(r, 3).transpose().cast<float>()).norm()));
        }
        near(e, 1e-5, name + ": asr points", &worst_point);
        near(en, 1e-5, name + ": asr normals");
    }

    // The extra keyframe, queued, and the new points tracking made with it
    const map::KeyFrameId xid = makeExtra(a, f, rig, X, ids[std::size_t(cur)], n);
    const Eigen::MatrixXd& xp = f["x_points"];
    std::vector<map::MapPointId> all = pid;
    for (int r = 0; r < xp.rows(); ++r)
        all.push_back(map::createMapPoint(a, xp.row(r).transpose().cast<float>(), xid));
    {
        const Eigen::MatrixXd& kp = f["x_kf_points"];
        for (int i = 0; i < kp.rows(); ++i) {
            if (kp(i) < 0) continue;
            const map::MapPointId p = all[std::size_t(kp(i))];
            a.editKeyFrame(xid)->points[std::size_t(i)] = p;
            if (kp(i) >= P) map::addObservation(a, p, xid, i);
        }
        for (std::size_t r = std::size_t(P); r < all.size(); ++r) {
            map::computeDistinctiveDescriptors(a, all[r]);
            map::updateNormalAndDepth(a, all[r]);
            map::insertMapPoint(a, all[r]);
        }
    }
    m.queue.push_back(xid);

    const ba::Problem p2 = orb_local_mapping::fullInertialProblem(a, M, prior_g, prior_a);
    check(p2.keyframes.size() == std::size_t(K), name + ": full_inertial keyframes");
    const ba::Result r2 = ba_solver::solveFullInertial(p2);
    std::printf("%s: FullInertialBA %d iterations, %zu points\n", name.c_str(), r2.iterations, r2.points.size());
    {
        // The rigid transform taking the port's keyframe centres onto the original's
        Eigen::Matrix3Xd src(3, r2.keyframes.size()), dst(3, r2.keyframes.size());
        for (std::size_t i = 0; i < r2.keyframes.size(); ++i) {
            const std::string p = "gba_kf" + std::to_string(kfIndex(r2.keyframes[i].id)) + "_";
            src.col(Eigen::Index(i)) = r2.keyframes[i].Tcw.inverse().translation().cast<double>();
            dst.col(Eigen::Index(i)) = fixture::se3f(f[p + "Tcw"]).inverse().translation().cast<double>();
        }
        const Eigen::Matrix4d A = Eigen::umeyama(src, dst, false);
        const geometry::SE3f Ainv = geometry::SE3f(Eigen::Matrix4f(A.cast<float>())).inverse();
        const Eigen::Matrix3f RA = A.block<3, 3>(0, 0).cast<float>();
        std::printf("%s: full BA gauge %.3g m\n", name.c_str(), A.block<3, 1>(0, 3).norm());
        for (const ba::KeyFrameState& k : r2.keyframes) {
            const std::string p = "gba_kf" + std::to_string(kfIndex(k.id)) + "_";
            near(poseErr(k.Tcw * Ainv, f[p + "Tcw"]), 1e-3, name + ": gba pose " + p, &fiba_worst);
            near((RA * k.velocity - f.f<3, 1>(p + "velocity")).norm(), 1e-2, name + ": gba velocity " + p, &fiba_worst);
            near(biasErr(k.bias, f[p + "bias"]), 1e-3, name + ": gba bias " + p, &fiba_worst);
        }
        const Eigen::MatrixXd& g = f["gba_points"];
        std::vector<int> got(all.size(), 0), n_obs(p2.points.size(), 0);
        for (const ba::Observation& o : p2.observations) ++n_obs[std::size_t(o.point)];
        double e = 0;
        for (const ba::PointState& s : r2.points) {
            const int r = int(std::find(all.begin(), all.end(), s.id) - all.begin());
            got[std::size_t(r)] = 1;
            const int pi = int(std::find_if(p2.points.begin(), p2.points.end(),
                                            [&](const ba::PointState& q) { return q.id == s.id; }) - p2.points.begin());
            if (n_obs[std::size_t(pi)] < 2) continue;
            const Eigen::Vector3f aligned = RA * s.pos + A.block<3, 1>(0, 3).cast<float>();
            const Eigen::Vector3f want = g.block<1, 3>(r, 1).transpose().cast<float>();
            e = std::max(e, double((aligned - want).norm() / std::max(1.f, want.norm())));
        }
        bool same = true;
        for (std::size_t r = 0; r < all.size(); ++r) same &= got[r] == int(g(Eigen::Index(r), 0));
        check(same, name + ": points in the full BA");
        near(e, 1e-2, name + ": gba points (relative)");
    }

    // The original's result, for the correction
    ba::Result r2_ref = r2;
    for (ba::KeyFrameState& k : r2_ref.keyframes) {
        const std::string p = "gba_kf" + std::to_string(kfIndex(k.id)) + "_";
        k.Tcw = fixture::se3f(f[p + "Tcw"]);
        k.velocity = f.f<3, 1>(p + "velocity");
        k.bias = biasOf(f[p + "bias"]);
    }
    for (ba::PointState& s : r2_ref.points) {
        const int r = int(std::find(all.begin(), all.end(), s.id) - all.begin());
        s.pos = f["gba_points"].block<1, 3>(r, 1).transpose().cast<float>();
    }

    // ---- stage 3
    const map::KeyFrameId gba_id = m.current;
    orb_local_mapping::storeFullInertial(m, gba_id, r2_ref);
    map::Vocabulary voc;
    check(orb_local_mapping::processNewKeyFrame(a, m, voc) && m.current == xid, name + ": extra processed");
    vpKF.push_back(m.current);
    {
        const map::KeyFrameId par = a.keyframe(xid)->parent;
        check((par == map::kNone ? -1 : kfIndex(par)) == int(f.scalar("x_parent")), name + ": extra's parent");
    }
    orb_local_mapping::correctFromOrigins(a, m, M, gba_id, vpKF.size());
    check(m.tracking_ok_seq == 1 && !m.initializing && m.n_kfs == K + 1, name + ": bookkeeping");
    for (int k = 0; k <= K; ++k) {
        const std::string p = "final_kf" + std::to_string(k) + "_";
        const map::KeyFrame& kf = *a.keyframe(k == X ? xid : ids[std::size_t(k)]);
        near(poseErr(kf.pose(), f[p + "Tcw"]), 1e-5, name + ": final pose " + std::to_string(k), &worst_pose);
        near((kf.velocity() - f.f<3, 1>(p + "velocity")).norm(), 1e-5, name + ": final velocity " + std::to_string(k),
             &worst_vel);
        near(biasErr(kf.bias, f[p + "bias"]), 1e-7, name + ": final bias " + std::to_string(k), &worst_bias);
    }
    {
        const Eigen::MatrixXd& fp = f["final_points"];
        double e = 0;
        for (std::size_t r = 0; r < all.size(); ++r)
            e = std::max(e, double((a.point(all[r])->pos - fp.row(Eigen::Index(r)).transpose().cast<float>()).norm()));
        near(e, 1e-5, name + ": final points", &worst_point);
    }
}

}  // namespace

int main() {
    const fixture::Rig rig;
    for (const char* c : {"init", "viba1", "viba2"}) runCase(c, rig);
    std::printf("write-backs, worst: pose %.3g, velocity %.3g, bias %.3g, point %.3g; optimizations, worst: "
                "inertial %.3g, full %.3g\n", worst_pose, worst_vel, worst_bias, worst_point, io_worst, fiba_worst);
    if (failures) {
        std::printf("%d failures\n", failures);
        return 1;
    }
    std::printf("imu_init: ok\n");
    return 0;
}
