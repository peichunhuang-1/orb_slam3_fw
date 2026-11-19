#pragma once
// The keypoint grid of one view, for searches in a window (ORB-SLAM3's Frame/KeyFrame mGrid, mGridRight and
// GetFeaturesInArea). 64 x 48 cells over the image [0, width) x [0, height); a keypoint goes to the cell
// round(x · 64 / width), round(y · 48 / height) and is left out if that is outside the grid. Indices are
// positions in that view's keypoints.
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include <opencv2/core/types.hpp>

namespace orb {

class FeatureGrid {
public:
    static constexpr int kCols = 64;  // FRAME_GRID_COLS
    static constexpr int kRows = 48;  // FRAME_GRID_ROWS

    FeatureGrid() = default;
    FeatureGrid(const std::vector<cv::KeyPoint>& keys, int width, int height)
        : inv_w_(float(kCols) / float(width)),
          inv_h_(float(kRows) / float(height)),
          cells_(std::size_t(kCols) * kRows) {
        for (std::size_t i = 0; i < keys.size(); ++i) {
            int cx, cy;
            if (cellOf(keys[i], cx, cy)) cells_[std::size_t(cx) * kRows + cy].push_back(i);
        }
    }

    // keys: the keypoints the grid was built from. Features at |x - kx| < r and |y - ky| < r, cell by cell (x, then y), in cell order. Octaves: at least
    // min_level when min_level > 0, at most max_level when max_level >= 0 (Frame::GetFeaturesInArea; the
    // keyframe's version is this with no level limits).
    std::vector<std::size_t> inArea(const std::vector<cv::KeyPoint>& keys, float x, float y, float r,
                                    int min_level = -1, int max_level = -1) const {
        std::vector<std::size_t> out;
        if (cells_.empty()) return out;
        out.reserve(keys.size());
        const int min_cx = std::max(0, int(std::floor((x - r) * inv_w_)));
        if (min_cx >= kCols) return out;
        const int max_cx = std::min(kCols - 1, int(std::ceil((x + r) * inv_w_)));
        if (max_cx < 0) return out;
        const int min_cy = std::max(0, int(std::floor((y - r) * inv_h_)));
        if (min_cy >= kRows) return out;
        const int max_cy = std::min(kRows - 1, int(std::ceil((y + r) * inv_h_)));
        if (max_cy < 0) return out;
        const bool check_levels = min_level > 0 || max_level >= 0;
        for (int cx = min_cx; cx <= max_cx; ++cx)
            for (int cy = min_cy; cy <= max_cy; ++cy)
                for (std::size_t i : cells_[std::size_t(cx) * kRows + cy]) {
                    const cv::KeyPoint& k = keys[i];
                    if (check_levels) {
                        if (k.octave < min_level) continue;
                        if (max_level >= 0 && k.octave > max_level) continue;
                    }
                    if (std::fabs(k.pt.x - x) < r && std::fabs(k.pt.y - y) < r) out.push_back(i);
                }
        return out;
    }

    bool cellOf(const cv::KeyPoint& k, int& cx, int& cy) const {
        cx = int(std::round(k.pt.x * inv_w_));
        cy = int(std::round(k.pt.y * inv_h_));
        return cx >= 0 && cx < kCols && cy >= 0 && cy < kRows;
    }

private:
    float inv_w_ = 0, inv_h_ = 0;
    std::vector<std::vector<std::size_t>> cells_;
};

}  // namespace orb
