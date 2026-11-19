#pragma once
// The keyframes' BoW inverted index (ORB-SLAM3's KeyFrameDatabase, the index only): per word, the keyframes
// whose BowVector has it, in the order they were added. Queries are with their users (relocalization's
// candidates in orb_tracking/relocalize, loop closing's in loop closing).
#include <functional>
#include <vector>

#include "types/map/ids.hpp"
#include "types/map/vocabulary.hpp"

namespace map {

class BowIndex {
public:
    void add(KeyFrameId kf, const BowVector& bow);
    void erase(KeyFrameId kf, const BowVector& bow);  // the first entry per word
    void clear() { words_.clear(); }
    void eraseIf(const std::function<bool(KeyFrameId)>& pred);  // clearMap
    // Keyframes with word w (empty if none)
    const std::vector<KeyFrameId>& keyframes(WordId w) const;
    std::size_t entries() const;

private:
    std::vector<std::vector<KeyFrameId>> words_;  // by word id, grown as needed
};

}  // namespace map
