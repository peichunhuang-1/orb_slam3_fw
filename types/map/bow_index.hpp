#pragma once
// The keyframes' BoW inverted index (ORB-SLAM3's KeyFrameDatabase, the index only): per word, the keyframes
// whose BowVector has it, in the order they were added. Queries are with their users (relocalization's
// candidates in orb_tracking/relocalize, loop closing's in loop closing).
//
// A COW type in the atlas (types/map/docs/DESIGN.md): one tile per word, so adding a keyframe copies only the
// lists of its own words.
#include <functional>
#include <vector>

#include "fw/tiled_map.hpp"
#include "types/map/ids.hpp"
#include "types/map/vocabulary.hpp"

namespace map {

struct BowIndex {
    void add(KeyFrameId kf, const BowVector& bow);
    void erase(KeyFrameId kf, const BowVector& bow);  // the first entry per word
    void clear() { words = {}; }
    void eraseIf(const std::function<bool(KeyFrameId)>& pred);  // clearMap
    // Keyframes with word w (empty if none)
    const std::vector<KeyFrameId>& keyframes(WordId w) const;
    std::size_t entries() const;

    fw::TiledMap<WordId, std::vector<KeyFrameId>> words;
};

}  // namespace map

FW_COW_TYPE(map::BowIndex, words);
