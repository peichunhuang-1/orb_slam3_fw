// map_point_culling/culling (LocalMapping::MapPointCulling) on synthetic points: each branch of the original's
// if-chain, with the current keyframe 0..3 keyframes after the points' first one.
#include <cstdio>
#include <string>

#include "algorithms/orb_local_mapping/map_point_culling/culling.hpp"
#include "types/map/atlas_ops.hpp"

namespace {

int failures = 0;

void check(bool ok, const std::string& what) {
    if (!ok) {
        ++failures;
        std::printf("FAIL %s\n", what.c_str());
    }
}

std::shared_ptr<const orb::FrameFeatures> features(int n) {
    auto s = std::make_shared<orb::StereoFrame>();
    s->width = 752;
    s->height = 480;
    s->scale_factors = {1.f, 1.2f};
    s->descriptors[0] = cv::Mat(n, 32, CV_8U, cv::Scalar(0));
    s->descriptors[1] = cv::Mat(0, 32, CV_8U);
    for (int i = 0; i < n; ++i) s->keypoints[0].emplace_back(float(i), 10.f, 31.f, -1.f, 0.f, 0);
    s->left_to_right.assign(std::size_t(n), -1);
    s->depth.assign(std::size_t(n), -1.f);
    s->points.assign(std::size_t(n), Eigen::Vector3f::Zero());
    return std::make_shared<const orb::FrameFeatures>(s);
}

map::KeyFrameId keyframe(map::Atlas& a, float x) {
    map::KeyFrame kf;
    auto d = std::make_shared<map::KeyFrameData>();
    d->features = features(20);
    d->Tlr = geometry::SE3f(Eigen::Matrix3f::Identity(), Eigen::Vector3f(0.11f, 0, 0));
    d->Trl = d->Tlr.inverse();
    kf.data = std::move(d);
    kf.setPose(geometry::SE3f(Eigen::Matrix3f::Identity(), Eigen::Vector3f(-x, 0, 0)));
    const map::KeyFrameId id = map::createKeyFrame(a, std::move(kf));
    map::insertKeyFrame(a, id);
    return id;
}

// A point first seen by keyframe `first`, observed at feature `index` of each of `seen` (n_obs = seen.size())
map::MapPointId point(map::Atlas& a, map::KeyFrameId first, std::initializer_list<map::KeyFrameId> seen, int index,
                      int visible, int found) {
    const map::MapPointId p = map::createMapPoint(a, Eigen::Vector3f(0, 0, 5), first);
    for (map::KeyFrameId k : seen) {
        map::addObservation(a, p, k, index);
        a.editKeyFrame(k)->points[std::size_t(index)] = p;
    }
    map::insertMapPoint(a, p);
    map::MapPoint* mp = a.editPoint(p);
    mp->visible = visible;
    mp->found = found;
    return p;
}

bool inList(const local_mapping::LocalMapper& m, map::MapPointId p) {
    for (map::MapPointId q : m.recent_points)
        if (q == p) return true;
    return false;
}

}  // namespace

int main() {
    map::Atlas a;
    std::vector<map::KeyFrameId> k;
    for (int i = 0; i < 6; ++i) k.push_back(keyframe(a, 0.2f * float(i)));

    local_mapping::LocalMapper m;
    // Made at keyframe 0
    const map::MapPointId bad = point(a, k[0], {k[0], k[1], k[2], k[3]}, 0, 4, 4);
    map::setPointBad(a, bad);
    const map::MapPointId low_ratio = point(a, k[0], {k[0], k[1], k[2], k[3]}, 1, 5, 1);  // 0.2 < 0.25
    const map::MapPointId ratio_quarter = point(a, k[0], {k[0], k[1], k[2], k[3]}, 2, 4, 1);  // 0.25: kept
    const map::MapPointId few_obs = point(a, k[0], {k[0], k[1], k[2]}, 3, 1, 1);  // 3 observations: culled at +2
    const map::MapPointId enough_obs = point(a, k[0], {k[0], k[1], k[2], k[3]}, 4, 1, 1);
    // Made at keyframe 1
    const map::MapPointId young = point(a, k[1], {k[1], k[2]}, 5, 1, 1);
    for (map::MapPointId p : {bad, low_ratio, ratio_quarter, few_obs, enough_obs, young}) m.recent_points.push_back(p);

    // Current keyframe 1: one keyframe after keyframe 0
    m.current = k[1];
    orb_local_mapping::mapPointCulling(a, m);
    check(!inList(m, bad), "a bad point leaves the list");
    check(!inList(m, low_ratio) && a.point(low_ratio)->bad, "found ratio < 0.25: culled");
    check(inList(m, ratio_quarter) && !a.point(ratio_quarter)->bad, "found ratio 0.25: kept");
    check(inList(m, few_obs) && !a.point(few_obs)->bad, "few observations, one keyframe after: kept");
    check(m.recent_points.size() == 4, "four left after keyframe 1");

    // Keyframe 2: two after keyframe 0
    m.current = k[2];
    orb_local_mapping::mapPointCulling(a, m);
    check(!inList(m, few_obs) && a.point(few_obs)->bad, "<= 3 observations two keyframes after: culled");
    check(inList(m, enough_obs) && inList(m, ratio_quarter), "4 observations: kept");
    check(inList(m, young) && !a.point(young)->bad, "one keyframe old with 2 observations: kept");

    // Keyframe 3: three after keyframe 0, two after keyframe 1
    m.current = k[3];
    orb_local_mapping::mapPointCulling(a, m);
    check(!inList(m, enough_obs) && !a.point(enough_obs)->bad, "three keyframes old: leaves the list, stays good");
    check(!inList(m, ratio_quarter) && !a.point(ratio_quarter)->bad, "three keyframes old (ratio 0.25): leaves");
    check(!inList(m, young) && a.point(young)->bad, "2 observations two keyframes after: culled");
    check(m.recent_points.empty(), "list empty");
    // The culled points left their keyframes
    check(a.keyframe(k[1])->points[3] == map::kNone && a.keyframe(k[2])->points[5] == map::kNone,
          "culled points erased from the keyframes");

    if (failures) {
        std::printf("%d failure(s)\n", failures);
        return 1;
    }
    std::printf("all passed\n");
    return 0;
}
