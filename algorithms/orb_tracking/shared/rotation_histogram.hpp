#pragma once
// The rotation consistency check of ORB-SLAM3's ORBmatcher searches (rotHist + ComputeThreeMaxima): matches
// are binned by the keypoint angle difference, and those outside the three fullest bins are dropped (the second
// and third bins only count with at least 10 % of the first one's matches).
//
// As the original, the bin is round(rot · 1/30) with rot in [0, 360) degrees (HISTO_LENGTH = 30 used as the bin
// width in degrees, not as the bin count): only bins 0..12 are ever filled, 12 for 345°..360°, and 0 for
// 0°..15°. Reproduced as written, since the matches it keeps depend on it.
#include <cmath>
#include <vector>

namespace orb_tracking {

class RotationHistogram {
public:
    static constexpr int kLength = 30;  // HISTO_LENGTH

    RotationHistogram() {
        for (auto& b : bins_) b.reserve(500);
    }

    // angle_a - angle_b: the reference keypoint's angle minus the matched one's (degrees, cv::KeyPoint::angle)
    void add(float angle_a, float angle_b, int index) {
        float rot = angle_a - angle_b;
        if (rot < 0.0) rot += 360.0f;
        int bin = int(std::round(rot * factor_));
        if (bin == kLength) bin = 0;
        bins_[bin].push_back(index);
    }

    // ComputeThreeMaxima: the fullest bins (-1: none)
    void threeMaxima(int& ind1, int& ind2, int& ind3) const {
        int max1 = 0, max2 = 0, max3 = 0;
        ind1 = ind2 = ind3 = -1;
        for (int i = 0; i < kLength; ++i) {
            const int s = int(bins_[i].size());
            if (s > max1) {
                max3 = max2;
                max2 = max1;
                max1 = s;
                ind3 = ind2;
                ind2 = ind1;
                ind1 = i;
            } else if (s > max2) {
                max3 = max2;
                max2 = s;
                ind3 = ind2;
                ind2 = i;
            } else if (s > max3) {
                max3 = s;
                ind3 = i;
            }
        }
        if (max2 < 0.1f * float(max1)) {
            ind2 = -1;
            ind3 = -1;
        } else if (max3 < 0.1f * float(max1)) {
            ind3 = -1;
        }
    }

    // Calls drop(index) for every match outside the three fullest bins, bin by bin, in insertion order
    template <class F>
    void forEachRejected(F&& drop) const {
        int ind1, ind2, ind3;
        threeMaxima(ind1, ind2, ind3);
        for (int i = 0; i < kLength; ++i) {
            if (i == ind1 || i == ind2 || i == ind3) continue;
            for (int index : bins_[i]) drop(index);
        }
    }

private:
    const float factor_ = 1.0f / kLength;
    std::vector<int> bins_[kLength];
};

}  // namespace orb_tracking
