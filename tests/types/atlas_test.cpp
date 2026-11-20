// types/map atlas operations on small synthetic keyframes: stereo points and their observation counts, the
// covisibility graph and spanning tree (UpdateConnections, SetBadFlag's re-parenting), point culling through
// EraseObservation, Replace, the representative descriptor, normal and depth, maps and the BoW index.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>
#include <string>

#include "types/map/atlas_ops.hpp"
#include "types/orb/hamming.hpp"

namespace {

int failures = 0;

void check(bool ok, const std::string& what) {
    if (!ok) {
        ++failures;
        std::printf("FAIL %s\n", what.c_str());
    }
}

std::mt19937 rng(7);

// n_left left keypoints, the first n_stereo matched to right keypoints 0.. in order, plus n_right_only more
std::shared_ptr<const orb::FrameFeatures> features(int n_left, int n_stereo, int n_right_only) {
    auto s = std::make_shared<orb::StereoFrame>();
    s->width = 752;
    s->height = 480;
    float f = 1;
    for (int l = 0; l < 8; ++l, f *= 1.2f) s->scale_factors.push_back(f);
    std::uniform_real_distribution<float> x(0, 752), y(0, 480);
    const int n_right = n_stereo + n_right_only;
    for (int v = 0; v < 2; ++v) {
        const int n = v == 0 ? n_left : n_right;
        s->descriptors[v] = cv::Mat(n, 32, CV_8U);
        cv::randu(s->descriptors[v], 0, 256);
        for (int i = 0; i < n; ++i) s->keypoints[v].emplace_back(x(rng), y(rng), 31.f, -1.f, 0.f, i % 8);
    }
    s->left_to_right.assign(std::size_t(n_left), -1);
    s->right_to_left.assign(std::size_t(n_right), -1);
    s->depth.assign(std::size_t(n_left), -1.f);
    s->points.assign(std::size_t(n_left), Eigen::Vector3f::Zero());
    for (int i = 0; i < n_stereo; ++i) {
        s->left_to_right[std::size_t(i)] = i;
        s->right_to_left[std::size_t(i)] = i;
        s->depth[std::size_t(i)] = 2.f;
        s->points[std::size_t(i)] = Eigen::Vector3f(0, 0, 2);
    }
    return std::make_shared<orb::FrameFeatures>(s);
}

map::KeyFrameId keyframe(map::Atlas& a, const Eigen::Vector3f& centre, int n_left = 60) {
    map::KeyFrame kf;
    auto d = std::make_shared<map::KeyFrameData>();
    d->features = features(n_left, n_left / 2, 10);
    d->Tlr = geometry::SE3f(Eigen::Matrix3f::Identity(), Eigen::Vector3f(0.11f, 0, 0));
    d->Trl = d->Tlr.inverse();
    kf.data = std::move(d);
    kf.setPose(geometry::SE3f(Eigen::Matrix3f::Identity(), -centre));  // camera at centre, looking along z
    const map::KeyFrameId id = map::createKeyFrame(a, std::move(kf));
    map::insertKeyFrame(a, id);
    return id;
}

// A point seen at the left feature `index` of each keyframe in kfs (monocular observations)
map::MapPointId seenBy(map::Atlas& a, std::initializer_list<std::pair<map::KeyFrameId, int>> kfs) {
    const map::MapPointId p = map::createMapPoint(a, Eigen::Vector3f(0, 0, 5), kfs.begin()->first);
    for (const auto& k : kfs) {
        map::addObservation(a, p, k.first, k.second);
        a.editKeyFrame(k.first)->points[std::size_t(k.second)] = p;
    }
    map::insertMapPoint(a, p);
    return p;
}

}  // namespace

int main() {
    map::Atlas a;
    const map::MapId m0 = a.currentMap().id;
    check(m0 == 0 && a.currentMap().init_kf_id == 0, "first map");

    // ---- keyframes and stereo points
    const map::KeyFrameId k0 = keyframe(a, Eigen::Vector3f(0, 0, 0));
    const map::KeyFrameId k1 = keyframe(a, Eigen::Vector3f(0.5f, 0, 0));
    const map::KeyFrameId k2 = keyframe(a, Eigen::Vector3f(1.0f, 0, 0));
    check(k0 == 0 && k1 == 1 && k2 == 2, "keyframe ids");
    check(a.currentMap().keyframes.size() == 3 && a.currentMap().max_kf_id == 2 && a.currentMap().lower_kf == 0,
          "map keyframes");
    const map::MapPointId s = map::createStereoPoint(a, k0, 3, Eigen::Vector3f(0.2f, 0.1f, 2.f));
    const map::MapPoint& sp = *a.point(s);
    const int n_left = a.keyframe(k0)->nLeft();
    check(sp.n_obs == 2 && sp.observations.at(k0).left == 3 && sp.observations.at(k0).right == n_left + 3,
          "stereo point observations");
    check(a.keyframe(k0)->points[3] == s && a.keyframe(k0)->points[std::size_t(n_left + 3)] == s,
          "stereo point in the keyframe");
    check(std::equal(sp.descriptor.begin(), sp.descriptor.end(), a.keyframe(k0)->data->features->descriptor(3)) ||
              std::equal(sp.descriptor.begin(), sp.descriptor.end(), a.keyframe(k0)->data->features->descriptor(n_left + 3)),
          "stereo point descriptor is one of its own");
    {
        const std::vector<map::MapPointId> in = map::mapPoints(a, a.current);
        check(std::count(in.begin(), in.end(), s) == 1 && a.currentMap().n_points == in.size(), "stereo point in the map");
    }

    // ---- normal and depth: left camera at the origin, right at (0.11, 0, 0)
    {
        const Eigen::Vector3f v0 = sp.pos.normalized(), v1 = (sp.pos - Eigen::Vector3f(0.11f, 0, 0)).normalized();
        const Eigen::Vector3f want = (v0 + v1) / 2.f;
        const int level = a.keyframe(k0)->data->features->keypoint(3).octave;
        const float max_d = sp.pos.norm() * a.keyframe(k0)->data->features->scale_factors[std::size_t(level)];
        check((sp.normal - want).norm() < 1e-6f, "normal");
        check(std::fabs(sp.max_distance - max_d) < 1e-5f &&
                  std::fabs(sp.min_distance - max_d / a.keyframe(k0)->data->features->scale_factors[7]) < 1e-5f,
              "scale invariance distances");
        check(sp.predictScale(sp.max_distance, std::log(1.2f), 8) == 0 &&
                  sp.predictScale(sp.max_distance / 1.2f / 1.2f, std::log(1.2f), 8) == 2 &&
                  sp.predictScale(sp.max_distance / 100.f, std::log(1.2f), 8) == 7,
              "predictScale");
    }

    // ---- covisibility: 20 points k0-k1, 16 k1-k2, 5 k0-k2
    std::vector<map::MapPointId> p01, p12, p02;
    for (int i = 0; i < 20; ++i) p01.push_back(seenBy(a, {{k0, 10 + i}, {k1, 10 + i}}));
    for (int i = 0; i < 16; ++i) p12.push_back(seenBy(a, {{k1, 40 + i}, {k2, 40 + i}}));
    for (int i = 0; i < 5; ++i) p02.push_back(seenBy(a, {{k0, 50 + i}, {k2, i}}));
    map::updateConnections(a, k0);  // the map's first keyframe: no parent
    map::updateConnections(a, k1);
    map::updateConnections(a, k2);
    const map::KeyFrame& kf0 = *a.keyframe(k0);
    const map::KeyFrame& kf1 = *a.keyframe(k1);
    const map::KeyFrame& kf2 = *a.keyframe(k2);
    check(kf1.connected.size() == 2 && kf1.weight(k0) == 20 && kf1.weight(k2) == 16, "k1 weights");
    check(kf1.ordered_connected == std::vector<map::KeyFrameId>({k0, k2}) &&
              kf1.ordered_weights == std::vector<int>({20, 16}),
          "k1 ordered covisibles");
    // k2: weight 5 with k0 is below 15, so only k1 is ordered; all counts are kept
    check(kf2.ordered_connected == std::vector<map::KeyFrameId>({k1}) && kf2.weight(k0) == 5, "k2 covisibles");
    // k0: only k1 reaches 15; the count 5 with k2 is kept but not ordered
    check(kf0.ordered_connected == std::vector<map::KeyFrameId>({k1}), "k0 ordered covisibles");
    check(kf0.parent == map::kNone && kf1.parent == k0 && kf2.parent == k1 && kf0.children.count(k1) &&
              kf1.children.count(k2),
          "spanning tree");
    check(map::covisiblesByWeight(kf1, 16) == std::vector<map::KeyFrameId>({k0, k2}) &&
              map::covisiblesByWeight(kf1, 17) == std::vector<map::KeyFrameId>({k0}) &&
              map::covisiblesByWeight(kf1, 21).empty(),
          "covisiblesByWeight");
    check(kf1.bestCovisibles(1) == std::vector<map::KeyFrameId>({k0}), "bestCovisibles");
    check(map::trackedMapPoints(a, kf1, 0) == 36 && map::trackedMapPoints(a, kf1, 3) == 0, "trackedMapPoints");

    // ---- two more observations (k2, left and right) keep a point alive when k1 goes (bad at 2 or fewer)
    const int k2_right = a.keyframe(k2)->nLeft() + 5;
    map::addObservation(a, p01[0], k2, 30);
    map::addObservation(a, p01[0], k2, k2_right);
    a.editKeyFrame(k2)->points[30] = p01[0];
    a.editKeyFrame(k2)->points[std::size_t(k2_right)] = p01[0];

    // ---- k1 goes bad: its points with 2 observations go bad too, k2 is re-parented to k0
    {
        auto d = std::make_shared<map::KeyFrameData>(*a.keyframe(k1)->data);
        d->bow = {{5, 0.5}, {9, 0.5}};
        a.editKeyFrame(k1)->data = std::move(d);
    }
    map::addToBowIndex(a, k1);
    map::setKeyFrameBad(a, k1);
    check(kf1.bad && a.currentMap().keyframes.count(k1) == 0, "k1 bad and out of the map");
    check(a.bow_index.keyframes(5).empty() && a.bow_index.keyframes(9).empty(), "k1 out of the BoW index");
    check(kf2.parent == k0 && kf0.children.count(k2) && !kf0.children.count(k1), "k2 re-parented to k0");
    check(kf0.connected.count(k1) == 0 && kf2.connected.count(k1) == 0, "connections to k1 erased");
    bool culled = true;
    for (std::size_t i = 1; i < p01.size(); ++i) culled = culled && a.point(p01[i])->bad;
    for (map::MapPointId p : p12) culled = culled && a.point(p)->bad;
    check(culled, "points left with 1 observation are bad");
    check(!a.point(p01[0])->bad && a.point(p01[0])->n_obs == 3 && a.point(p01[0])->ref_kf == k0,
          "point with 4 observations survives with 3");
    check(a.keyframe(k0)->points[11] == map::kNone && a.keyframe(k2)->points[41] == map::kNone,
          "bad points erased from keyframes");
    map::setKeyFrameBad(a, k0);
    check(!kf0.bad, "the map's first keyframe is never bad");

    // ---- replace: p02[0] by p02[1]; both are in k0 and k2, so the old one's features are cleared
    {
        const map::MapPointId from = p02[0], to = p02[1];
        a.editPoint(from)->found = 4;
        a.editPoint(from)->visible = 6;
        map::replace(a, from, to);
        check(a.point(from)->bad && a.point(from)->replaced == to, "replaced");
        check(a.keyframe(k0)->points[50] == map::kNone && a.keyframe(k2)->points[0] == map::kNone,
              "replaced point's features cleared where the other is seen");
        check(a.point(to)->found == 5 && a.point(to)->visible == 7, "found / visible carried over");
        // in a keyframe that does not see the other point, the observation moves to it
        const map::KeyFrameId kx = keyframe(a, Eigen::Vector3f(0, 1, 0));
        const map::MapPointId lone = seenBy(a, {{kx, 20}});
        map::replace(a, lone, to);
        check(a.keyframe(kx)->points[20] == to && a.point(to)->observations.at(kx).left == 20 &&
                  a.point(to)->n_obs == 3,
              "replace moves the observation");
    }

    // ---- representative descriptor: least median Hamming distance, the first on a tie
    {
        const map::KeyFrameId k3 = keyframe(a, Eigen::Vector3f(2, 0, 0), 4);
        const map::KeyFrameId k4 = keyframe(a, Eigen::Vector3f(3, 0, 0), 4);
        const map::KeyFrameId k5 = keyframe(a, Eigen::Vector3f(4, 0, 0), 4);
        const map::MapPointId p = seenBy(a, {{k3, 1}, {k4, 1}, {k5, 1}});
        // the middle descriptor (k4's) is 8 bits from each of the others, which are 16 apart
        auto set = [&](map::KeyFrameId k, int bits) {
            auto& d = const_cast<cv::Mat&>(a.keyframe(k)->data->features->stereo->descriptors[0]);
            d.row(1).setTo(0);
            for (int b = 0; b < bits; ++b) d.at<std::uint8_t>(1, b / 8) |= std::uint8_t(1 << (b % 8));
        };
        set(k3, 0);
        set(k4, 8);
        set(k5, 16);
        // medians: k3 [0 8 16] -> 8, k4 [0 8 8] -> 8, k5 -> 8: all equal, the first (k3) wins
        map::computeDistinctiveDescriptors(a, p);
        check(a.point(p)->descriptor[0] == 0, "descriptor tie: the first");
        // four descriptors (median index 1), against a brute-force pick
        set(k3, 24);
        const map::KeyFrameId k6 = keyframe(a, Eigen::Vector3f(5, 0, 0), 4);
        set(k6, 16);
        map::addObservation(a, p, k6, 1);
        map::computeDistinctiveDescriptors(a, p);
        int best = -1, best_median = 1 << 30;
        const map::KeyFrameId ks[4] = {k3, k4, k5, k6};
        for (int i = 0; i < 4; ++i) {
            std::vector<int> d;
            for (int j = 0; j < 4; ++j)
                d.push_back(orb::hamming256(a.keyframe(ks[i])->data->features->descriptor(1),
                                            a.keyframe(ks[j])->data->features->descriptor(1)));
            std::sort(d.begin(), d.end());
            if (d[1] < best_median) {
                best_median = d[1];
                best = i;
            }
        }
        check(std::equal(a.point(p)->descriptor.begin(), a.point(p)->descriptor.end(),
                         a.keyframe(ks[best])->data->features->descriptor(1)),
              "descriptor: least median");
    }

    // ---- maps: a new map starts after the current one's last keyframe; clearing detaches its keyframes
    {
        const map::KeyFrameId max_kf = a.currentMap().max_kf_id;
        const map::MapId m1 = map::createNewMap(a);
        check(m1 == 1 && a.current == m1 && a.currentMap().init_kf_id == max_kf + 1 && !a.findMap(m0)->in_use,
              "new map");
        const map::KeyFrameId k = keyframe(a, Eigen::Vector3f(0, 0, 0));
        check(a.keyframe(k)->map == m1 && a.currentMap().init_kf_id == k, "keyframe in the new map");
        a.bow_index.add(k, {{3, 1.0}});
        a.bow_index.add(k0, {{3, 1.0}});
        map::eraseMapFromBowIndex(a, m1);
        map::clearMap(a, m1);
        check(a.bow_index.keyframes(3) == std::vector<map::KeyFrameId>({k0}), "BoW index of the map cleared");
        check(a.keyframe(k)->map == map::kNone && a.currentMap().keyframes.empty(), "map cleared");
        map::clearAtlas(a);
        check(a.keyframes.empty() && a.points.empty() && !a.hasCurrentMap() && a.currentMap().id == 2 &&
                  a.currentMap().init_kf_id == 0,
              "atlas cleared, map ids go on");
    }

    std::printf(failures ? "%d failures\n" : "all passed\n", failures);
    return failures ? 1 : 0;
}
