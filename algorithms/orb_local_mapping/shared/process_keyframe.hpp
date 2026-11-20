#pragma once
// LocalMapping::ProcessNewKeyFrame: process_new_keyframe and initialize_imu (which empties the queue the same way).
#include "types/local_mapping/local_mapper.hpp"
#include "types/map/atlas.hpp"
#include "types/map/vocabulary.hpp"

namespace orb_local_mapping {

// Takes the queue's first keyframe as the current one: its BoW, its points' observations (normal, depth,
// descriptor), the covisibility graph, then the map. Keyframes that are gone or out of every map (reset meanwhile)
// are skipped. false when the queue had nothing to process.
bool processNewKeyFrame(map::Atlas& a, local_mapping::LocalMapper& m, const map::Vocabulary& voc);

}  // namespace orb_local_mapping
