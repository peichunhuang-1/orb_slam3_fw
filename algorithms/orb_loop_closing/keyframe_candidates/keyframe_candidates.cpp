#include "algorithms/orb_loop_closing/keyframe_candidates/keyframe_candidates.hpp"

#include <algorithm>
#include <set>
#include <utility>

namespace orb_loop_closing {

void detectNBestCandidates(const map::Atlas& a, PlaceRecognitionScratchMap& scratch, map::KeyFrameId kf_id, int n,
                           std::vector<map::KeyFrameId>& loop_candidates,
                           std::vector<map::KeyFrameId>& merge_candidates) {
    loop_candidates.clear();
    merge_candidates.clear();
    const map::KeyFrame* kf = a.keyframe(kf_id);
    if (!kf) return;

    // Keyframes sharing a word, in the order the index gives them
    std::vector<map::KeyFrameId> sharing;
    for (const auto& [word, weight] : kf->data->bow) {
        for (map::KeyFrameId k : a.bow_index.keyframes(word)) {
            if (k == kf_id || !a.keyframe(k)) continue;
            PlaceRecognitionScratch& s = scratch[k];
            if (s.query != kf_id) {
                s.words = 0;
                if (!kf->connected.count(k)) {
                    s.query = kf_id;
                    sharing.push_back(k);
                }
            }
            ++s.words;
        }
    }
    if (sharing.empty()) return;

    int max_common = 0;
    for (map::KeyFrameId k : sharing) max_common = std::max(max_common, scratch[k].words);
    const int min_common = int(float(max_common) * 0.8f);

    std::vector<std::pair<float, map::KeyFrameId>> scored;
    for (map::KeyFrameId k : sharing) {
        PlaceRecognitionScratch& s = scratch[k];
        if (s.words > min_common) {
            const float si = float(map::Vocabulary::score(kf->data->bow, a.keyframe(k)->data->bow));
            s.score = si;
            scored.emplace_back(si, k);
        }
    }
    if (scored.empty()) return;

    // Accumulated by covisibility: only the neighbours scored for this query (the reference's fix: below
    // min_common a neighbour's score is left over from an earlier query)
    std::vector<std::pair<float, map::KeyFrameId>> accumulated;
    for (const auto& [score, k] : scored) {
        float best = score, acc = score;
        map::KeyFrameId best_kf = k;
        for (map::KeyFrameId nb : a.keyframe(k)->bestCovisibles(10)) {
            const auto it = scratch.find(nb);
            if (it == scratch.end()) continue;
            const PlaceRecognitionScratch& s = it->second;
            if (s.query != kf_id || s.words <= min_common) continue;
            acc += s.score;
            if (s.score > best) {
                best_kf = nb;
                best = s.score;
            }
        }
        accumulated.emplace_back(acc, best_kf);
    }
    // list::sort with a.first > b.first: stable, descending
    std::stable_sort(accumulated.begin(), accumulated.end(),
                     [](const auto& x, const auto& y) { return x.first > y.first; });

    std::set<map::KeyFrameId> added;
    for (std::size_t i = 0; i < accumulated.size() &&
                            (int(loop_candidates.size()) < n || int(merge_candidates.size()) < n);
         ++i) {
        const map::KeyFrameId k = accumulated[i].second;
        const map::KeyFrame* ki = a.keyframe(k);
        if (!ki || ki->bad) continue;
        if (!added.insert(k).second) continue;
        if (ki->map == kf->map) {
            if (int(loop_candidates.size()) < n) loop_candidates.push_back(k);
        } else if (int(merge_candidates.size()) < n && a.findMap(ki->map)) {
            merge_candidates.push_back(k);
        }
    }
}

}  // namespace orb_loop_closing
