#include "types/map/bow_index.hpp"

#include <algorithm>

namespace map {

void BowIndex::add(KeyFrameId kf, const BowVector& bow) {
    for (const auto& w : bow) words.edit(w.first)->push_back(kf);
}

void BowIndex::erase(KeyFrameId kf, const BowVector& bow) {
    for (const auto& w : bow) {
        const std::vector<KeyFrameId>* l = words.find(w.first);
        if (!l) continue;
        const auto it = std::find(l->begin(), l->end(), kf);
        if (it == l->end()) continue;
        const auto at = it - l->begin();
        auto e = words.edit(w.first);
        e->erase(e->begin() + at);
    }
}

void BowIndex::eraseIf(const std::function<bool(KeyFrameId)>& pred) {
    std::vector<WordId> hit;  // only the words that change are copied
    words.for_each([&](WordId w, const std::vector<KeyFrameId>& l) {
        if (std::any_of(l.begin(), l.end(), pred)) hit.push_back(w);
    });
    for (WordId w : hit) {
        auto e = words.edit(w);
        e->erase(std::remove_if(e->begin(), e->end(), pred), e->end());
    }
}

const std::vector<KeyFrameId>& BowIndex::keyframes(WordId w) const {
    static const std::vector<KeyFrameId> none;
    const std::vector<KeyFrameId>* l = words.find(w);
    return l ? *l : none;
}

std::size_t BowIndex::entries() const {
    std::size_t n = 0;
    words.for_each([&n](WordId, const std::vector<KeyFrameId>& l) { n += l.size(); });
    return n;
}

}  // namespace map
