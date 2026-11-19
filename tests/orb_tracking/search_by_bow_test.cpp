// orb_tracking/shared/search_by_bow and rotation_histogram on hand-made keyframe / frame pairs: the node walk,
// the left ratio test, the right view's quirks (tried only after a left best <= TH_LOW, no ratio test), bad
// points, features matched once, and the orientation check with the original's bins (round(rot / 30)).
#include <cstdio>
#include <memory>
#include <string>

#include "algorithms/orb_tracking/shared/rotation_histogram.hpp"
#include "algorithms/orb_tracking/shared/search_by_bow.hpp"

namespace {

int failures = 0;

void check(bool ok, const std::string& what) {
    if (!ok) {
        ++failures;
        std::printf("FAIL %s\n", what.c_str());
    }
}

// A descriptor with its first `bits` bits set: hamming(desc(a), desc(b)) = |a - b|
void setDescriptor(cv::Mat& m, int row, int bits) {
    for (int b = 0; b < 256; ++b) {
        std::uint8_t& byte = m.at<std::uint8_t>(row, b / 8);
        if (b < bits)
            byte |= std::uint8_t(1u << (b % 8));
        else
            byte &= std::uint8_t(~(1u << (b % 8)));
    }
}

struct View {
    std::vector<int> bits;
    std::vector<float> angles;
};

std::shared_ptr<const orb::FrameFeatures> features(const View& left, const View& right) {
    auto s = std::make_shared<orb::StereoFrame>();
    s->width = 752;
    s->height = 480;
    s->scale_factors = {1.f, 1.2f};
    const View* v[2] = {&left, &right};
    for (int k = 0; k < 2; ++k) {
        s->descriptors[k] = cv::Mat(int(v[k]->bits.size()), 32, CV_8U, cv::Scalar(0));
        for (std::size_t i = 0; i < v[k]->bits.size(); ++i) {
            setDescriptor(s->descriptors[k], int(i), v[k]->bits[i]);
            s->keypoints[k].emplace_back(10.f * float(i), 10.f, 31.f, v[k]->angles[i], 0.f, 0);
        }
    }
    s->left_to_right.assign(left.bits.size(), -1);
    s->right_to_left.assign(right.bits.size(), -1);
    s->depth.assign(left.bits.size(), -1.f);
    s->points.assign(left.bits.size(), Eigen::Vector3f::Zero());
    return std::make_shared<const orb::FrameFeatures>(s);
}

map::MapPointId addPoint(map::Atlas& a, bool bad = false) {
    auto p = std::make_unique<map::MapPoint>();
    p->id = a.points.size();
    p->bad = bad;
    a.points.push_back(std::move(p));
    return a.points.back()->id;
}

void testMatching() {
    map::Atlas atlas;
    // Keyframe: 4 left features, 1 right (index 4)
    map::KeyFrame kf;
    kf.features = features({{0, 100, 200, 30}, {0, 0, 0, 0}}, {{60}, {0}});
    const map::MapPointId p0 = addPoint(atlas), p1 = addPoint(atlas), p2 = addPoint(atlas), p3 = addPoint(atlas, true),
                          p4 = addPoint(atlas);
    kf.points = {p0, p1, p2, p3, p4};
    // Frame: 5 left (0..4), 3 right (5..7)
    tracking::Frame f;
    f.features = features({{2, 10, 103, 140, 30}, {0, 0, 0, 0, 0}}, {{0, 100, 130}, {0, 0, 0}});
    f.points.assign(8, map::kNone);
    f.outlier.assign(8, false);
    // Nodes: the keyframe in 1, 3, 5, 9; the frame in 2, 3, 5, 7, 9
    kf.feat = {{1, {3}}, {3, {0}}, {5, {1, 2}}, {9, {4}}};
    f.feat = {{2, {4}}, {3, {0, 1, 5}}, {5, {2, 3, 6, 7}}, {7, {4}}, {9, {1}}};

    std::vector<map::MapPointId> m;
    const int n = orb_tracking::searchByBoW(kf, atlas, f, 0.7f, false, m);
    // node 3: kf 0 (0 bits) vs left 0 (2), left 1 (10): 2 < 0.7·10 -> left 0; right 5 (0): no ratio -> matched
    check(m[0] == p0 && m[5] == p0, "left ratio + right without ratio");
    // node 5: kf 1 (100) vs left 2 (103), left 3 (140): 3 < 0.7·40 -> left 2; right 6 (100) d 0 -> matched.
    // kf 2 (200): left 2 taken, left 3 at 60 > TH_LOW: its right 7 (130, d 70) is not tried either
    check(m[2] == p1 && m[6] == p1 && m[3] == map::kNone && m[7] == map::kNone, "right only after left <= TH_LOW");
    // node 1 only in the keyframe (p3 bad anyway), node 9: kf 4 (right, 60 bits) vs frame left 1 (10): d 50 <= 50
    check(m[1] == p4, "node walk, right keyframe feature");
    check(n == 5, "match count " + std::to_string(n));

    // Ratio fails on the left (2 vs 2 with ratio 0.7): the left stays unmatched, the right is still matched
    f.feat = {{3, {0, 1, 5}}};
    kf.feat = {{3, {0}}};
    f.features = features({{2, 2, 0, 0, 0}, {0, 0, 0, 0, 0}}, {{40, 0, 0}, {0, 0, 0}});
    const int n2 = orb_tracking::searchByBoW(kf, atlas, f, 0.7f, false, m);
    check(n2 == 1 && m[0] == map::kNone && m[1] == map::kNone && m[5] == p0, "left ratio fails, right kept");

    // Bad point: no match
    kf.feat = {{1, {3}}};
    f.feat = {{1, {0}}};
    check(orb_tracking::searchByBoW(kf, atlas, f, 0.7f, false, m) == 0, "bad point skipped");
}

void testOrientation() {
    // 10 matches at +5° (bin 0), 2 at 350° (bin 12), 1 at 200° (bin 7) and 1 at 100° (bin 3): bins 3 and 7 tie for
    // the third place and the first one seen (3) is kept; 1 >= 10 % of 10
    orb_tracking::RotationHistogram h;
    for (int i = 0; i < 10; ++i) h.add(15.f, 10.f, i);
    h.add(10.f, 20.f, 10);
    h.add(10.f, 20.f, 11);
    h.add(250.f, 50.f, 12);
    h.add(130.f, 30.f, 13);
    int a, b, c;
    h.threeMaxima(a, b, c);
    check(a == 0 && b == 12 && c == 3, "three maxima " + std::to_string(a) + " " + std::to_string(b) + " " + std::to_string(c));
    std::vector<int> dropped;
    h.forEachRejected([&](int i) { dropped.push_back(i); });
    check(dropped == std::vector<int>{12}, "fourth bin dropped");

    // A second bin below 10 % of the first: only the first is kept
    orb_tracking::RotationHistogram g;
    for (int i = 0; i < 20; ++i) g.add(0.f, 0.f, i);
    g.add(100.f, 0.f, 20);
    g.threeMaxima(a, b, c);
    check(a == 0 && b == -1 && c == -1, "small second bin");
    dropped.clear();
    g.forEachRejected([&](int i) { dropped.push_back(i); });
    check(dropped == std::vector<int>{20}, "small second bin dropped");

    // Bins: 359.9° rounds to 12 (359.9 / 30), 7.6° to 0
    orb_tracking::RotationHistogram k;
    k.add(0.f, 0.1f, 0);
    k.add(7.6f, 0.f, 1);
    k.add(7.6f, 0.f, 2);
    k.threeMaxima(a, b, c);
    check(a == 0 && b == 12, "bins of 359.9 and 7.6 degrees");
}

}  // namespace

int main() {
    testMatching();
    testOrientation();
    std::printf(failures ? "%d failures\n" : "all passed\n", failures);
    return failures ? 1 : 0;
}
