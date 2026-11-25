#pragma once
// A parallel loop handed from a tree leaf (fw::parallel_for on its THREADS budget) down to the tracking code, which
// does not depend on fw. Empty: sequential.
//
// Loops that sum go over kParallelChunks fixed chunks and add the chunks' sums in chunk order, so a result does not
// depend on the number of threads (the same with THREADS(1) as with THREADS(4)).
#include <functional>

namespace orb_tracking {

using ParallelFor = std::function<void(int begin, int end, const std::function<void(int)>& fn)>;

constexpr int kParallelChunks = 8;

inline void runParallel(const ParallelFor& par, int begin, int end, const std::function<void(int)>& fn) {
    if (end <= begin) return;
    if (par) par(begin, end, fn);
    else
        for (int i = begin; i < end; ++i) fn(i);
}

// [lo, hi) of chunk c of n items in kParallelChunks chunks
inline int chunkBegin(int n, int c) { return int((long long)n * c / kParallelChunks); }

}  // namespace orb_tracking
