// orb_vocabulary TEXT BINARY
//
// DBoW2's text vocabulary (ORB-SLAM3's ORBvoc.txt) -> the binary file map::Vocabulary loads (fw_load). The
// binary file is loaded back and compared with the text one, node for node and bit for bit, before it is moved
// into place. data/orb_vocabulary/ORBvoc.bin.tar.gz was made with it.
#include <chrono>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <string>

#include "types/map/vocabulary.hpp"

namespace {
using clk = std::chrono::steady_clock;
double since(clk::time_point a) { return std::chrono::duration<double>(clk::now() - a).count(); }
}  // namespace

int main(int argc, char** argv) {
    if (argc != 3) {
        std::fprintf(stderr, "usage: %s TEXT BINARY\n", argv[0]);
        return 2;
    }
    const std::filesystem::path text_path = argv[1], bin_path = argv[2];
    const std::filesystem::path tmp_path = bin_path.string() + ".tmp";
    try {
        map::Vocabulary text, binary;
        auto t = clk::now();
        text.loadText(text_path);
        const double text_s = since(t);
        text.saveBinary(tmp_path);
        t = clk::now();
        binary.loadBinary(tmp_path);
        const double bin_s = since(t);
        std::string why;
        if (!text.sameAs(binary, &why)) {
            std::filesystem::remove(tmp_path);
            std::fprintf(stderr, "the binary vocabulary differs from the text one: %s\n", why.c_str());
            return 1;
        }
        std::filesystem::rename(tmp_path, bin_path);
        std::printf("%s: %zu words, %zu nodes, identical to the text vocabulary; load: text %.1f s, binary %.2f s\n",
                    bin_path.c_str(), text.words(), text.nodes(), text_s, bin_s);
    } catch (const std::exception& e) {
        std::filesystem::remove(tmp_path);
        std::fprintf(stderr, "%s\n", e.what());
        return 1;
    }
    return 0;
}
