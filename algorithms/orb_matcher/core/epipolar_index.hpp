#pragma once
// Right keypoints sorted by θ, with their descriptors copied in the same order, so a left
// keypoint's θ window is one binary search plus a sequential scan.
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <numeric>
#include <vector>

#include "algorithms/orb_matcher/core/bearing.hpp"

namespace orb_matcher {

struct EpipolarIndex {
    std::vector<float> theta, phi, sigma, sigma_theta;
    std::vector<int> octave, index;   // index: the keypoint's position in the view
    std::vector<uint8_t> desc;        // 32 bytes per entry, sorted order
    float max_sigma_theta = 0;
    int size = 0;

    void reserve(int n) {
        theta.reserve(n);
        phi.reserve(n);
        sigma.reserve(n);
        sigma_theta.reserve(n);
        octave.reserve(n);
        index.reserve(n);
        desc.reserve(size_t(n) * 32);
        order_.reserve(n);
    }

    void build(const BearingSet& b, const uint8_t* descriptors, size_t stride) {
        order_.clear();
        for (int i = 0; i < int(b.valid.size()); ++i)
            if (b.valid[i]) order_.push_back(i);
        std::sort(order_.begin(), order_.end(), [&](int x, int y) { return b.theta[x] < b.theta[y]; });
        size = int(order_.size());
        theta.resize(size);
        phi.resize(size);
        sigma.resize(size);
        sigma_theta.resize(size);
        octave.resize(size);
        index.resize(size);
        desc.resize(size_t(size) * 32);
        max_sigma_theta = 0;
        for (int k = 0; k < size; ++k) {
            const int i = order_[k];
            theta[k] = b.theta[i];
            phi[k] = b.phi[i];
            sigma[k] = b.sigma[i];
            sigma_theta[k] = b.sigma_theta[i];
            octave[k] = b.octave[i];
            index[k] = i;
            std::memcpy(&desc[size_t(k) * 32], descriptors + size_t(i) * stride, 32);
            max_sigma_theta = std::max(max_sigma_theta, sigma_theta[k]);
        }
    }

    int lower_bound(float t) const { return int(std::lower_bound(theta.begin(), theta.begin() + size, t) - theta.begin()); }

private:
    std::vector<int> order_;
};

}  // namespace orb_matcher
