#include "algorithms/orb_tracking/relocalize/relocalization_candidates.hpp"

#include <algorithm>
#include <set>
#include <utility>

#include "algorithms/orb_tracking/shared/tracking_ops.hpp"

namespace orb_tracking {

std::vector<map::KeyFrameId> relocalizationCandidates(const map::Atlas& a, tracking::Tracker& t,
                                                      const tracking::Frame& f) {
    // Keyframes sharing a word, in the order the index gives them
    std::vector<map::KeyFrameId> sharing;
    for (const auto& [word, weight] : f.bow) {
        for (map::KeyFrameId k : a.bow_index.keyframes(word)) {
            if (!a.keyframe(k)) continue;
            tracking::RelocScratch& s = relocScratch(t, k);
            if (s.query != f.id) {
                s.words = 0;
                s.query = f.id;
                sharing.push_back(k);
            }
            ++s.words;
        }
    }
    if (sharing.empty()) return {};

    int max_common = 0;
    for (map::KeyFrameId k : sharing) max_common = std::max(max_common, relocScratch(t, k).words);
    const int min_common = int(float(max_common) * 0.8f);

    std::vector<std::pair<float, map::KeyFrameId>> scored;
    for (map::KeyFrameId k : sharing) {
        tracking::RelocScratch& s = relocScratch(t, k);
        if (s.words > min_common) {
            const float si = float(map::Vocabulary::score(f.bow, a.keyframe(k)->bow));
            s.score = si;
            scored.emplace_back(si, k);
        }
    }
    if (scored.empty()) return {};

    // Accumulated by covisibility
    std::vector<std::pair<float, map::KeyFrameId>> accumulated;
    float best_acc = 0;
    for (const auto& [score, k] : scored) {
        float best = score, acc = score;
        map::KeyFrameId best_kf = k;
        for (map::KeyFrameId n : a.keyframe(k)->bestCovisibles(10)) {
            const tracking::RelocScratch& s = relocScratch(t, n);
            if (s.query != f.id) continue;
            acc += s.score;
            if (s.score > best) {
                best_kf = n;
                best = s.score;
            }
        }
        accumulated.emplace_back(acc, best_kf);
        if (acc > best_acc) best_acc = acc;
    }

    const float min_retain = 0.75f * best_acc;
    std::set<map::KeyFrameId> added;
    std::vector<map::KeyFrameId> candidates;
    for (const auto& [acc, k] : accumulated) {
        if (acc <= min_retain) continue;
        const map::KeyFrame* kf = a.keyframe(k);
        if (!kf || kf->map != a.current) continue;
        if (added.insert(k).second) candidates.push_back(k);
    }
    return candidates;
}

}  // namespace orb_tracking
