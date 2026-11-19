#include "types/map/bow_index.hpp"

#include <algorithm>

namespace map {

void BowIndex::add(KeyFrameId kf, const BowVector& bow) {
    for (const auto& w : bow) {
        if (w.first >= words_.size()) words_.resize(std::size_t(w.first) + 1);
        words_[w.first].push_back(kf);
    }
}

void BowIndex::erase(KeyFrameId kf, const BowVector& bow) {
    for (const auto& w : bow) {
        if (w.first >= words_.size()) continue;
        std::vector<KeyFrameId>& l = words_[w.first];
        const auto it = std::find(l.begin(), l.end(), kf);
        if (it != l.end()) l.erase(it);
    }
}

void BowIndex::eraseIf(const std::function<bool(KeyFrameId)>& pred) {
    for (std::vector<KeyFrameId>& l : words_) l.erase(std::remove_if(l.begin(), l.end(), pred), l.end());
}

const std::vector<KeyFrameId>& BowIndex::keyframes(WordId w) const {
    static const std::vector<KeyFrameId> none;
    return w < words_.size() ? words_[w] : none;
}

std::size_t BowIndex::entries() const {
    std::size_t n = 0;
    for (const auto& l : words_) n += l.size();
    return n;
}

}  // namespace map
