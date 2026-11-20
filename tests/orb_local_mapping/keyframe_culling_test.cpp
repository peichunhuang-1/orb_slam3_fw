// keyframe_culling/culling (LocalMapping::KeyFrameCulling) on synthetic keyframes: the redundancy count (close
// points only, other keyframes at the same or a finer level, more than 3 of them), the map's first keyframe, and
// the inertial bounds (keyframes in the map, the last two, the time between the neighbours, not_erase) with the
// prev/next chain and the preintegration merge.
#include <cmath>
#include <cstdio>
#include <string>

#include "algorithms/orb_local_mapping/keyframe_culling/culling.hpp"
#include "types/map/atlas_ops.hpp"

namespace {

int failures = 0;

void check(bool ok, const std::string& what) {
    if (!ok) {
        ++failures;
        std::printf("FAIL %s\n", what.c_str());
    }
}

constexpr int kSlots = 80;      // left features per keyframe, plus one right feature (index kSlots)
constexpr float kThDepth = 40;  // depth 50 is far

struct Spec {
    std::vector<int> fine_slots;  // these slots at level 2, the others at 0
    std::vector<int> far_slots;   // depth 50, the others 1
};

std::shared_ptr<const orb::FrameFeatures> features(const Spec& spec) {
    auto s = std::make_shared<orb::StereoFrame>();
    s->width = 752;
    s->height = 480;
    s->scale_factors = {1.f, 1.2f, 1.44f};
    s->descriptors[0] = cv::Mat(kSlots, 32, CV_8U, cv::Scalar(0));
    s->descriptors[1] = cv::Mat(1, 32, CV_8U, cv::Scalar(0));
    for (int i = 0; i < kSlots; ++i) s->keypoints[0].emplace_back(float(i), 10.f, 31.f, -1.f, 0.f, 0);
    for (int i : spec.fine_slots) s->keypoints[0][std::size_t(i)].octave = 2;
    s->keypoints[1].emplace_back(5.f, 10.f, 31.f, -1.f, 0.f, 0);
    s->left_to_right.assign(kSlots, -1);
    s->right_to_left.assign(1, -1);
    s->depth.assign(kSlots, 1.f);
    for (int i : spec.far_slots) s->depth[std::size_t(i)] = 50.f;
    s->points.assign(kSlots, Eigen::Vector3f::Zero());
    return std::make_shared<const orb::FrameFeatures>(s);
}

// n keyframes 0.1 s apart, linked prev / next, each with a 2-sample preintegration (0.05 s each)
std::vector<map::KeyFrameId> keyframes(map::Atlas& a, int n, const std::vector<Spec>& specs = {}) {
    map::createNewMap(a);
    std::vector<map::KeyFrameId> ids;
    for (int i = 0; i < n; ++i) {
        auto d = std::make_shared<map::KeyFrameData>();
        d->frame_id = i;
        d->timestamp = 0.1 * i;
        d->features = features(std::size_t(i) < specs.size() ? specs[std::size_t(i)] : Spec());
        d->th_depth = kThDepth;
        d->has_imu_calib = true;  // T_cb identity: the IMU at the camera
        map::KeyFrame kf;
        kf.data = std::move(d);
        kf.setPose(geometry::SE3f(Eigen::Matrix3f::Identity(), Eigen::Vector3f(-0.1f * float(i), 0, 0)));  // 10 cm apart
        auto pre = std::make_shared<imu::Preintegrated>();
        for (int k = 0; k < 2; ++k) pre->integrate(Eigen::Vector3f(0, 0, 9.81f), Eigen::Vector3f::Zero(), 0.05f);
        kf.preintegrated = std::move(pre);
        if (!ids.empty()) kf.prev = ids.back();
        const map::KeyFrameId id = map::createKeyFrame(a, std::move(kf));
        if (!ids.empty()) a.editKeyFrame(ids.back())->next = id;
        map::insertKeyFrame(a, id);
        ids.push_back(id);
    }
    return ids;
}

// A point at feature `slot` of each of `seen`
map::MapPointId point(map::Atlas& a, std::initializer_list<map::KeyFrameId> seen, int slot) {
    const map::MapPointId p = map::createMapPoint(a, Eigen::Vector3f(0, 0, 1), *seen.begin());
    for (map::KeyFrameId k : seen) {
        map::addObservation(a, p, k, slot);
        a.editKeyFrame(k)->points[std::size_t(slot)] = p;
    }
    map::insertMapPoint(a, p);
    return p;
}

// t's points at slots [first, first + n), each also seen by `others` at the same slot
void points(map::Atlas& a, map::KeyFrameId t, int first, int n, std::initializer_list<map::KeyFrameId> others) {
    for (int s = first; s < first + n; ++s) {
        const map::MapPointId p = point(a, {t}, s);
        for (map::KeyFrameId k : others) {
            map::addObservation(a, p, k, s);
            a.editKeyFrame(k)->points[std::size_t(s)] = p;
        }
    }
}

void connect(map::Atlas& a, map::KeyFrameId cur, map::KeyFrameId k, int w) {
    map::addConnection(a, cur, k, w);
    map::addConnection(a, k, cur, w);
}

bool bad(const map::Atlas& a, map::KeyFrameId k) { return a.keyframe(k)->bad; }

void setTime(map::Atlas& a, map::KeyFrameId k, double t) {
    auto d = std::make_shared<map::KeyFrameData>(*a.keyframe(k)->data);
    d->timestamp = t;
    a.editKeyFrame(k)->data = std::move(d);
}

void stereo() {
    // 0 the map's first; 1..5 covisibles of the current one (9); 6..8 other observers
    map::Atlas a;
    Spec observer;
    for (int s = 20; s < 30; ++s) observer.fine_slots.push_back(s);
    Spec t4;
    t4.far_slots = {40, 41};
    const std::vector<map::KeyFrameId> k =
        keyframes(a, 10, {Spec(), Spec(), Spec(), Spec(), t4, Spec(), observer, observer, observer, observer});
    const map::KeyFrameId cur = k[9];
    const auto o = {cur, k[6], k[7], k[8]};  // 4 others

    points(a, k[0], 50, 10, o);  // redundant, but the map's first
    points(a, k[1], 0, 10, o);   // 10 of 10 redundant
    points(a, k[2], 10, 9, o);   // 9 of 10: not more than 90%
    points(a, k[2], 19, 1, {cur, k[6], k[7]});  // 3 others
    points(a, k[3], 20, 10, o);  // the others see them at level 2 (> 0 + 1)
    points(a, k[4], 30, 10, o);  // 10 close, redundant
    points(a, k[4], 40, 2, {cur});  // far: not counted
    // a right feature: not counted (culling.hpp)
    point(a, {k[4], cur}, kSlots);
    points(a, k[5], 60, 10, o);  // redundant, but bad already
    map::setKeyFrameBad(a, k[5]);
    for (int i = 0; i <= 5; ++i) connect(a, cur, k[std::size_t(i)], 10 + i);

    local_mapping::LocalMapper m;
    m.current = cur;
    const int culled = orb_local_mapping::keyFrameCulling(a, m);
    check(culled == 2, "stereo: 2 culled, got " + std::to_string(culled));
    check(!bad(a, k[0]), "stereo: the map's first keyframe kept");
    check(bad(a, k[1]), "stereo: all points redundant: culled");
    check(!bad(a, k[2]), "stereo: 90% redundant: kept");
    check(!bad(a, k[3]), "stereo: others at a coarser level: kept");
    check(bad(a, k[4]), "stereo: far and right points not counted: culled");
    check(!bad(a, cur) && !bad(a, k[6]), "stereo: others kept");
    // Culled: out of the covisibility, the points no longer see it
    check(!a.keyframe(cur)->connected.count(k[1]), "stereo: culled keyframe left the covisibility");
    const map::MapPoint* p = a.point(a.keyframe(cur)->points[0]);
    check(p && !p->inKeyFrame(k[1]) && p->n_obs == 4, "stereo: culled keyframe's observation erased");
}

void inertial() {
    // 26 keyframes, the current one last; 20..23 observers
    map::Atlas a;
    std::vector<map::KeyFrameId> k = keyframes(a, 26);
    const map::KeyFrameId cur = k[25];
    const auto o = {cur, k[20], k[21], k[22]};
    points(a, k[10], 0, 10, o);  // t = 0.2: culled
    points(a, k[12], 10, 6, o);  // 6 of 10 > 50%: culled
    points(a, k[12], 16, 4, {cur});
    points(a, k[14], 20, 5, o);  // 5 of 10: kept
    points(a, k[14], 25, 5, {cur});
    points(a, k[5], 30, 10, o);  // not_erase: unlinked, only marked
    a.editKeyFrame(k[5])->not_erase = true;
    points(a, k[18], 40, 10, o);  // its neighbours 3.2 s apart: kept
    for (std::size_t i = 19; i < k.size(); ++i) setTime(a, k[i], a.keyframe(k[i])->data->timestamp + 3.8);
    a.editKeyFrame(k[18])->setPose(a.keyframe(k[17])->pose());  // only the time keeps it
    points(a, k[24], 50, 10, o);  // the last but one: kept
    points(a, k[16], 60, 10, o);  // neighbours 1 s apart, the IMU moved 10 cm: kept
    setTime(a, k[17], 2.5);
    points(a, k[8], 70, 5, o);  // neighbours 1 s apart, the IMU within 2 cm of the previous one: culled
    setTime(a, k[7], -0.1);
    a.editKeyFrame(k[8])->setPose(a.keyframe(k[7])->pose());
    points(a, k[3], 75, 5, {cur, k[20], k[21], k[23]});  // neighbours 1 s apart, IMU initialized, 3 < 4: culled
    setTime(a, k[2], -0.6);
    a.editMap(a.current)->imu_initialized = true;
    for (int i : {3, 5, 8, 10, 12, 14, 16, 18, 24}) connect(a, cur, k[std::size_t(i)], 10 + i);

    local_mapping::LocalMapper m;
    m.inertial = true;
    m.current = cur;
    const int culled = orb_local_mapping::keyFrameCulling(a, m);
    check(culled == 4, "inertial: 4 culled, got " + std::to_string(culled));
    check(bad(a, k[10]) && bad(a, k[12]), "inertial: redundant, neighbours 0.2 s apart: culled");
    check(bad(a, k[3]), "inertial: IMU initialized, before the window, neighbours 1 s apart: culled");
    check(bad(a, k[8]), "inertial: IMU within 2 cm, neighbours 1 s apart: culled");
    check(a.keyframe(k[7])->next == k[9] && a.keyframe(k[2])->next == k[4], "inertial: 8 and 3 unlinked");
    check(!bad(a, k[16]), "inertial: IMU 10 cm away, neighbours 1 s apart: kept");
    check(!bad(a, k[14]), "inertial: 50% redundant: kept");
    check(!bad(a, k[18]), "inertial: neighbours 3.2 s apart: kept");
    check(!bad(a, k[24]), "inertial: last but one: kept");
    check(!bad(a, k[5]) && a.keyframe(k[5])->to_be_erased, "inertial: not_erase: marked");

    // The chain: 9 -> 11, 11 -> 13; 5 out of it too
    check(a.keyframe(k[9])->next == k[11] && a.keyframe(k[11])->prev == k[9], "inertial: 10 unlinked");
    check(a.keyframe(k[11])->next == k[13] && a.keyframe(k[13])->prev == k[11], "inertial: 12 unlinked");
    check(a.keyframe(k[10])->prev == map::kNone && a.keyframe(k[10])->next == map::kNone, "inertial: 10's links");
    check(a.keyframe(k[4])->next == k[6] && a.keyframe(k[5])->next == map::kNone, "inertial: 5 unlinked");
    check(a.keyframe(k[13])->next == k[14], "inertial: 13 still linked");
    // The next keyframe's preintegration covers the culled one's
    check(a.keyframe(k[11])->preintegrated->measurements().size() == 4, "inertial: 11 merged 10's samples");
    check(std::abs(a.keyframe(k[13])->preintegrated->dT - 0.2f) < 1e-6f, "inertial: 13 covers 0.2 s");
    check(a.keyframe(k[10])->preintegrated->measurements().size() == 2, "inertial: 10's own unchanged");
}

void small_inertial_map() {
    // 21 keyframes in the map: none culled
    map::Atlas a;
    std::vector<map::KeyFrameId> k = keyframes(a, 21);
    points(a, k[10], 0, 10, {k[20], k[1], k[2], k[3]});
    connect(a, k[20], k[10], 20);
    local_mapping::LocalMapper m;
    m.inertial = true;
    m.current = k[20];
    check(orb_local_mapping::keyFrameCulling(a, m) == 0 && !bad(a, k[10]), "small map: none culled");
}

void abort_limit() {
    // abort_ba: stops after 20 covisibles; the 21st is counted, the 22nd is not
    map::Atlas a;
    std::vector<map::KeyFrameId> k = keyframes(a, 28);
    const map::KeyFrameId cur = k[27];
    const auto o = {cur, k[23], k[24], k[25]};
    for (int i = 1; i <= 22; ++i) {
        if (i == 21 || i == 22) points(a, k[std::size_t(i)], 10 * (i - 21), 10, o);
        connect(a, cur, k[std::size_t(i)], 100 - i);  // covisibles in order 1..22
    }
    local_mapping::LocalMapper m;
    m.current = cur;
    m.abort_ba = true;
    const int culled = orb_local_mapping::keyFrameCulling(a, m);
    check(culled == 1 && bad(a, k[21]) && !bad(a, k[22]), "abort_ba: stops after the 21st");
}

}  // namespace

int main() {
    stereo();
    inertial();
    small_inertial_map();
    abort_limit();
    if (failures) {
        std::printf("%d failures\n", failures);
        return 1;
    }
    std::printf("keyframe_culling: ok\n");
    return 0;
}
