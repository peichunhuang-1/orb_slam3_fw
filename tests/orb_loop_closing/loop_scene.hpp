#pragma once
// The scene of tests/fixtures/orb_slam3/loop (README.md) as an atlas: keyframes with the original's BoW, points
// with its descriptors, normals and scale ranges, the covisibility from the observations. The correct cases also
// have the original's spanning tree, velocities, T_cb and the map's inertial flags.
#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "tests/orb_local_mapping/fixture_keyframes.hpp"
#include "types/map/atlas_ops.hpp"

namespace fixture {

struct Scene {
    map::Atlas a;
    std::vector<map::KeyFrameId> kfs;
    std::vector<map::MapPointId> pts;

    int kfIndex(map::KeyFrameId id) const {
        if (id == map::kNone) return -1;
        return int(std::find(kfs.begin(), kfs.end(), id) - kfs.begin());
    }
    int ptIndex(map::MapPointId id) const {
        if (id == map::kNone) return -1;
        return int(std::find(pts.begin(), pts.end(), id) - pts.begin());
    }
    std::vector<map::MapPointId> points(const Eigen::MatrixXd& idx) const {
        std::vector<map::MapPointId> r;
        for (int i = 0; i < idx.rows(); ++i) r.push_back(idx(i) < 0 ? map::kNone : pts[std::size_t(idx(i))]);
        return r;
    }
};

inline void load(Scene& s, const fixture::File& f, const fixture::Rig& rig) {
    map::Atlas& a = s.a;
    const int K = int(f.scalar("n_keyframes")), KA = int(f.scalar("pass_a"));
    const bool merge = f.scalar("merge") != 0;
    a.currentMap();
    map::addRig(a, rig.rig);
    for (int k = 0; k < K; ++k) {
        if (merge && k == KA) map::createNewMap(a);
        const std::string p = "kf" + std::to_string(k) + "_";
        auto d = std::make_shared<map::KeyFrameData>();
        d->frame_id = k;
        d->timestamp = f.scalar(p + "timestamp");
        d->features = fixture::keyframeFeatures(f, p, rig);
        d->rig = 0;
        d->Tlr = rig.Tlr;
        d->Trl = rig.Tlr.inverse();
        d->baseline = rig.Tlr.translation().norm();
        d->th_depth = rig.th_depth;
        const Eigen::MatrixXd& bow = f[p + "bow"];
        for (int i = 0; i < bow.rows(); ++i) d->bow[map::WordId(bow(i, 0))] = bow(i, 1);
        const Eigen::MatrixXd& feat = f[p + "feat"];
        for (int i = 0; i < feat.rows(); ++i) d->feat[map::NodeId(feat(i, 0))].push_back(unsigned(feat(i, 1)));
        if (f.has("Tcb")) {
            d->has_imu_calib = true;
            d->T_cb = fixture::se3f(f["Tcb"]);
        }
        map::KeyFrame kf;
        kf.data = std::move(d);
        kf.setPose(fixture::se3f(f[p + "Tcw"]));
        if (k > 0 && k != KA) kf.prev = s.kfs.back();
        const map::KeyFrameId id = map::createKeyFrame(a, std::move(kf));
        if (k > 0 && k != KA) a.editKeyFrame(s.kfs.back())->next = id;
        map::insertKeyFrame(a, id);
        s.kfs.push_back(id);
    }

    const Eigen::MatrixXd& P = f["points"];
    const Eigen::MatrixXd& D = f["point_desc"];
    for (int r = 0; r < P.rows(); ++r)
        s.pts.push_back(map::createMapPoint(a, P.block<1, 3>(r, 1).transpose().cast<float>(), s.kfs[std::size_t(P(r, 0))]));
    for (int k = 0; k < K; ++k) {
        const Eigen::MatrixXd& kp = f["kf" + std::to_string(k) + "_points"];
        for (int i = 0; i < kp.rows(); ++i) {
            if (kp(i) < 0) continue;
            const map::MapPointId p = s.pts[std::size_t(kp(i))];
            map::addObservation(a, p, s.kfs[std::size_t(k)], i);
            a.editKeyFrame(s.kfs[std::size_t(k)])->points[std::size_t(i)] = p;
        }
    }
    for (int r = 0; r < P.rows(); ++r) {
        map::insertMapPoint(a, s.pts[std::size_t(r)]);
        map::MapPoint* p = a.editPoint(s.pts[std::size_t(r)]);
        p->normal = P.block<1, 3>(r, 4).transpose().cast<float>();
        p->min_distance = float(P(r, 7));
        p->max_distance = float(P(r, 8));
        for (int b = 0; b < 32; ++b) p->descriptor[std::size_t(b)] = std::uint8_t(D(r, b));
    }
    for (map::KeyFrameId id : s.kfs) map::updateConnections(a, id);
    for (int k = 0; k < K - 1; ++k) map::addToBowIndex(a, s.kfs[std::size_t(k)]);
    if (!f.has("parents")) return;

    // The correct cases: the original's spanning tree (its ties between equal weights go by pointer), the map
    // with the IMU initialized (inertial: every keyframe bImu), the velocities
    const bool inertial = f.scalar("inertial") != 0;
    map::Map* m = a.editMap(a.keyframe(s.kfs[0])->map);
    m->inertial = inertial;
    m->imu_initialized = true;
    const Eigen::MatrixXd& par = f["parents"];
    const Eigen::MatrixXd& vel = f["velocity"];
    for (int k = 0; k < K; ++k) a.editKeyFrame(s.kfs[std::size_t(k)])->children.clear();
    for (int k = 0; k < K; ++k) {
        map::KeyFrame* kf = a.editKeyFrame(s.kfs[std::size_t(k)]);
        kf->parent = par(k) < 0 ? map::kNone : s.kfs[std::size_t(par(k))];
        kf->imu = inertial;
        kf->setVelocity(vel.row(k).transpose().cast<float>());
        if (kf->parent != map::kNone) a.editKeyFrame(kf->parent)->children.insert(kf->id);
    }
}

}  // namespace fixture
