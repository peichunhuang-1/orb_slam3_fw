#pragma once
// Ids by which keyframes, map points and maps refer to each other (algorithms/orb_tracking/docs/DESIGN.md,
// "以 id 引用，不用指標"). An id that is no longer found in the atlas is skipped, as ORB-SLAM3's isBad().
#include <cstdint>

namespace map {

using KeyFrameId = std::uint64_t;
using MapPointId = std::uint64_t;
using MapId = std::uint64_t;
constexpr std::uint64_t kNone = ~std::uint64_t(0);

}  // namespace map
