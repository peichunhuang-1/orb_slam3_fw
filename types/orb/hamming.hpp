#pragma once
// Hamming distance of two 256-bit ORB descriptors.
#include <cstdint>
#include <cstring>

#if defined(__ARM_NEON) || defined(__ARM_NEON__)
#include <arm_neon.h>
#endif

namespace orb {

inline int hamming256(const uint8_t* a, const uint8_t* b) {
#if defined(__ARM_NEON) || defined(__ARM_NEON__)
    const uint8x16_t x0 = veorq_u8(vld1q_u8(a), vld1q_u8(b));
    const uint8x16_t x1 = veorq_u8(vld1q_u8(a + 16), vld1q_u8(b + 16));
    // per byte at most 8 + 8; the total (up to 256) needs the widening sum
    return int(vaddlvq_u8(vaddq_u8(vcntq_u8(x0), vcntq_u8(x1))));
#else
    uint64_t x[4], y[4];
    std::memcpy(x, a, 32);
    std::memcpy(y, b, 32);
    return __builtin_popcountll(x[0] ^ y[0]) + __builtin_popcountll(x[1] ^ y[1]) +
           __builtin_popcountll(x[2] ^ y[2]) + __builtin_popcountll(x[3] ^ y[3]);
#endif
}

}  // namespace orb
