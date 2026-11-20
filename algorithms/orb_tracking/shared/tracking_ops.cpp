#include "algorithms/orb_tracking/shared/tracking_ops.hpp"

#include <stdexcept>
#include <utility>

#include "types/geometry/so3.hpp"
#include "types/map/atlas_ops.hpp"

namespace orb_tracking {

const tracking::MapperStatus& mapperOf(const fw::Optional<const tracking::MapperStatus&>& m) {
    static const tracking::MapperStatus idle;
    return m.has_value() ? *m : idle;
}

void addEdit(tracking::Tracker& t, tracking::AtlasEdit e) {
    e.seq = t.next_edit_seq++;
    auto p = std::make_shared<const tracking::AtlasEdit>(std::move(e));
    applyToView(t, *p);
    t.pending.push_back(std::move(p));
}

void applyToView(tracking::Tracker& t, const tracking::AtlasEdit& e) {
    if (e.kind != tracking::AtlasEdit::Kind::counts) tracking::applyEdit(t.view, e);
}

void flushCounts(tracking::Tracker& t) {
    if (t.counts.empty()) return;
    tracking::AtlasEdit e;
    e.kind = tracking::AtlasEdit::Kind::counts;
    e.counts.assign(t.counts.begin(), t.counts.end());
    t.counts.clear();
    addEdit(t, std::move(e));
}

void sendEdits(const tracking::Tracker& t, std::uint64_t seq_before, fw::Optional<tracking::AtlasEdits>& out) {
    if (t.next_edit_seq == seq_before) return;
    tracking::AtlasEdits m;
    m.edits = t.pending;
    out = std::move(m);
}

void requestResetActiveMap(tracking::Tracker& t) {
    if (t.pending_reset == tracking::PendingReset::none) t.pending_reset = tracking::PendingReset::active_map;
}

void createMapInAtlas(tracking::Tracker& t, tracking::State& state) {
    t.last_init_frame_id = t.current.id;
    tracking::AtlasEdit e;
    e.kind = tracking::AtlasEdit::Kind::new_map;
    e.map = t.view.next_map_id;
    e.inertial = t.inertial;  // SetInertialSensor
    addEdit(t, std::move(e));
    t.initial_frame_id = t.current.id + 1;
    state = tracking::State::NO_IMAGES_YET;
    t.has_velocity = false;
    t.vo = false;
    if (t.inertial && t.preintegrated_from_kf) t.preintegrated_from_kf = newPreintegration(t, imu::Bias());
    t.last_keyframe = map::kNone;
    t.reference_kf = map::kNone;
    t.last = tracking::Frame();
    t.current = tracking::Frame();
    t.created_map = true;
}

std::shared_ptr<imu::Preintegrated> newPreintegration(const tracking::Tracker& t, const imu::Bias& b) {
    auto p = std::make_shared<imu::Preintegrated>(imu::Bias(), t.imu_noise);
    p->setNewBias(b);
    return p;
}

namespace {

constexpr float kGravity = 9.81f;  // IMU::GRAVITY_VALUE

// Rwb2, twb2, vwb2 from (Rwb1, twb1, vwb1) and a preintegration's deltas at bias b
void predict(const Eigen::Matrix3f& Rwb1, const Eigen::Vector3f& twb1, const Eigen::Vector3f& Vwb1,
             const imu::Preintegrated& p, const imu::Bias& b, tracking::Frame& f) {
    const Eigen::Vector3f Gz(0, 0, -kGravity);
    const float t12 = p.dT;
    const Eigen::Matrix3f Rwb2 = geometry::normalizeRotation(Eigen::Matrix3f(Rwb1 * p.deltaRotation(b)));
    const Eigen::Vector3f twb2 = twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz + Rwb1 * p.deltaPosition(b);
    const Eigen::Vector3f Vwb2 = Vwb1 + t12 * Gz + Rwb1 * p.deltaVelocity(b);
    f.setImuPoseVelocity(Rwb2, twb2, Vwb2);
}

}  // namespace

bool predictStateImu(const map::Atlas& a, tracking::Tracker& t) {
    tracking::Frame& cur = t.current;
    if (!t.inertial) return false;  // mpPrevFrame: set for inertial frames only (the tracker's last frame)
    const map::KeyFrame* kf = a.keyframe(t.last_keyframe);
    if (t.map_updated && kf) {
        if (!t.preintegrated_from_kf) return false;
        predict(kf->imuRotation(), kf->imuPosition(), kf->velocity(), *t.preintegrated_from_kf, kf->bias, cur);
        cur.bias = kf->bias;
        return true;
    }
    if (!t.map_updated) {
        if (!cur.preintegrated_frame) return false;
        predict(t.last.imuRotation(), t.last.imuPosition(), t.last.velocity(), *cur.preintegrated_frame, t.last.bias, cur);
        cur.bias = t.last.bias;
        return true;
    }
    return false;
}

tracking::PointScratch& pointScratch(tracking::Tracker& t, map::MapPointId p) {
    if (p >= t.point_scratch.size()) t.point_scratch.resize(std::size_t(p) + 1);
    return t.point_scratch[std::size_t(p)];
}

tracking::RelocScratch& relocScratch(tracking::Tracker& t, map::KeyFrameId k) {
    if (k >= t.reloc.size()) t.reloc.resize(std::size_t(k) + 1);
    return t.reloc[std::size_t(k)];
}

int discardPoseOutliers(const map::Atlas& a, tracking::Tracker& t, int& nmatches, OutlierRule rule) {
    tracking::Frame& f = t.current;
    int nmatches_map = 0;
    for (std::size_t i = 0; i < f.points.size(); ++i) {
        const map::MapPointId id = f.points[i];
        if (id == map::kNone) continue;
        if (f.outlier[i]) {
            f.points[i] = map::kNone;
            f.outlier[i] = false;
            tracking::PointScratch& s = pointScratch(t, id);
            if (int(i) < f.nLeft()) s.in_view = false;
            else s.in_view_r = false;
            if (rule == OutlierRule::reference_keyframe) s.in_view = false;
            s.last_frame_seen = f.id;
            --nmatches;
        } else if (const map::MapPoint* p = a.point(id); p && p->n_obs > 0) {
            ++nmatches_map;
        }
    }
    return nmatches_map;
}

std::vector<const Eigen::Vector3f*> worldPositions(const map::Atlas& a, const std::vector<map::MapPointId>& points,
                                                   bool good_only) {
    std::vector<const Eigen::Vector3f*> w(points.size(), nullptr);
    for (std::size_t i = 0; i < points.size(); ++i) {
        const map::MapPoint* p = good_only ? a.goodPoint(points[i]) : a.point(points[i]);
        if (p) w[i] = &p->pos;
    }
    return w;
}

const camera::StereoRig& rigOf(const map::Atlas& a, const tracking::Frame& f) {
    if (f.rig < 0 || f.rig >= a.n_rigs) throw std::logic_error("orb_tracking: the frame has no rig");
    return a.rig(f.rig);
}

map::KeyFrame keyFrameOf(const tracking::Frame& f) {
    auto d = std::make_shared<map::KeyFrameData>();
    d->frame_id = f.id;
    d->timestamp = f.timestamp;
    d->features = f.features;
    d->rig = f.rig;
    d->Tlr = f.Tlr;
    d->Trl = f.Trl;
    d->baseline = f.baseline;
    d->th_depth = f.th_depth;
    d->bow = f.bow;
    d->feat = f.feat;
    d->has_imu_calib = f.has_imu_calib;
    d->T_cb = f.T_cb;
    map::KeyFrame kf;
    kf.data = std::move(d);
    kf.points = f.points;
    kf.preintegrated = f.preintegrated;
    if (f.hasVelocity()) kf.setVelocity(f.velocity());
    kf.bias = f.bias;
    kf.setPose(f.pose());
    return kf;
}

}  // namespace orb_tracking
