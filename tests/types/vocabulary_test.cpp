// map::Vocabulary vs DBoW2 (tests/fixtures/orb_slam3/bow): transform's BowVector and FeatureVector, and the L1
// scores, on ORB-SLAM3's vocabulary. Usage: types_vocabulary_test [ORBvoc.bin] (default: the build's
// orb_vocabulary/ORBvoc.bin).
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

#include "tests/common/orb_slam3_fixture.hpp"
#include "types/map/vocabulary.hpp"

#ifndef ORB_VOCABULARY_BIN
#error "define ORB_VOCABULARY_BIN"
#endif

namespace {

int failures = 0;

void check(bool ok, const std::string& what) {
    if (!ok) {
        ++failures;
        std::printf("FAIL %s\n", what.c_str());
    }
}

cv::Mat descriptors(const Eigen::MatrixXd& m) {
    cv::Mat d(int(m.rows()), 32, CV_8U);
    for (int i = 0; i < m.rows(); ++i)
        for (int j = 0; j < 32; ++j) d.at<std::uint8_t>(i, j) = std::uint8_t(m(i, j));
    return d;
}

// Weights to 1e-15 relative (the same double operations; only the order of the L1 sum could differ)
void compareBow(const map::BowVector& v, const Eigen::MatrixXd& want, const std::string& name) {
    check(v.size() == std::size_t(want.rows()), name + " bow size " + std::to_string(v.size()) + " vs " +
                                                    std::to_string(want.rows()));
    int r = 0;
    double worst = 0;
    for (const auto& e : v) {
        if (r >= want.rows()) break;
        check(e.first == map::WordId(want(r, 0)), name + " bow word at row " + std::to_string(r));
        worst = std::max(worst, std::fabs(e.second - want(r, 1)) / std::fabs(want(r, 1)));
        ++r;
    }
    check(worst < 1e-14, name + " bow weight, relative error " + std::to_string(worst));
}

void compareFeat(const map::FeatureVector& f, const Eigen::MatrixXd& want, const std::string& name) {
    int r = 0;
    bool same = true;
    for (const auto& e : f)
        for (unsigned i : e.second) {
            if (r >= want.rows() || e.first != map::NodeId(want(r, 0)) || i != unsigned(want(r, 1))) same = false;
            ++r;
        }
    check(same && r == want.rows(), name + " feat");
}

}  // namespace

int main(int argc, char** argv) {
    const std::string bin = argc > 1 ? argv[1] : ORB_VOCABULARY_BIN;
    map::Vocabulary voc;
    voc.loadBinary(bin);
    const fixture::File info("bow/vocabulary.txt");
    check(voc.branching() == int(info.scalar("k")) && voc.levels() == int(info.scalar("L")) &&
              voc.words() == std::size_t(info.scalar("words")) &&
              int(voc.weighting()) == int(info.scalar("weighting")) && info.scalar("scoring") == 0,
          "vocabulary parameters");
    std::printf("vocabulary: k %d, L %d, %zu words, %zu nodes\n", voc.branching(), voc.levels(), voc.words(),
                voc.nodes());

    for (const char* c : {"small", "frame", "noisy", "exact_words", "duplicate_words"}) {
        const fixture::File f(std::string("bow/") + c + ".txt");
        map::BowVector bow[3];
        map::FeatureVector feat[3];
        const char* n[3] = {"a", "b", "c"};
        for (int i = 0; i < 3; ++i) {
            voc.transform(descriptors(f[std::string("desc_") + n[i]]), bow[i], feat[i], 4);
            compareBow(bow[i], f[std::string(n[i]) + "_bow"], std::string(c) + " " + n[i]);
            compareFeat(feat[i], f[std::string(n[i]) + "_feat"], std::string(c) + " " + n[i]);
        }
        const double s[4] = {map::Vocabulary::score(bow[0], bow[1]), map::Vocabulary::score(bow[0], bow[2]),
                             map::Vocabulary::score(bow[1], bow[2]), map::Vocabulary::score(bow[0], bow[0])};
        const char* sn[4] = {"score_ab", "score_ac", "score_bc", "score_aa"};
        for (int i = 0; i < 4; ++i)
            check(std::fabs(s[i] - f.scalar(sn[i])) < 1e-14, std::string(c) + " " + sn[i]);
        std::printf("%-16s %4d descriptors, %3zu words, ab %.6f ac %.6f bc %.6f aa %.6f\n", c,
                    int(f["desc_a"].rows()), bow[0].size(), s[0], s[1], s[2], s[3]);
    }

    // Binary round trip
    const std::string tmp = std::string(std::getenv("TMPDIR") ? std::getenv("TMPDIR") : "/tmp") + "/voc_rt.bin";
    voc.saveBinary(tmp);
    map::Vocabulary back;
    back.loadBinary(tmp);
    std::string why;
    check(voc.sameAs(back, &why), "binary round trip: " + why);
    std::remove(tmp.c_str());

    std::printf(failures ? "%d failures\n" : "all passed\n", failures);
    return failures ? 1 : 0;
}
