#pragma once
// Messages of the GPU engine (warnings, ORB_LIST_PIPELINES-style debug lines): one line each to stderr,
// which the fw container shows in the block's log.
#include <cstdio>
#include <string>

namespace orb_extractor {

enum class log_level { info, warn };

inline void log(log_level level, const std::string& line) {
    std::fprintf(stderr, "%s%s\n", level == log_level::warn ? "[warn] " : "", line.c_str());
}

}  // namespace orb_extractor
