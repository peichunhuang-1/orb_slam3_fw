#pragma once
// The ORB vocabulary tree (resource type "map/Vocabulary"): DBoW2's TemplatedVocabulary<FORB> as ORB-SLAM3 uses
// it, reduced to what tracking needs (and later loop closing): load, transform a frame's descriptors into a
// BowVector and a FeatureVector, and the L1 score of two BowVectors.
//
// Same results as DBoW2 on the same vocabulary (tests/fixtures/orb_slam3/bow): a descriptor goes down the tree
// to the child at the smallest Hamming distance (the first one on a tie), the word's weight (idf for TF-IDF) is
// added up per word, and the vector is normalized to L1 norm 1.
//
// Files: the binary format of ORB-SLAM3's tools/orb_vocabulary ("DBoW2BIN", version 1; loadBinary), and DBoW2's
// text format (ORBvoc.txt; loadText), which tools/orb_vocabulary converts. As a resource it is loaded from its
// `store:` (the binary file) and never written back.
#include <cstdint>
#include <filesystem>
#include <map>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "fw/fw.hpp"

namespace map {

using WordId = std::uint32_t;
using NodeId = std::uint32_t;
using WordValue = double;

// Word id -> weight, ascending by id (DBoW2::BowVector)
using BowVector = std::map<WordId, WordValue>;
// Node id (levels_up above the leaves) -> indices of the descriptors under it, in the order they were added
// (DBoW2::FeatureVector)
using FeatureVector = std::map<NodeId, std::vector<unsigned int>>;

class Vocabulary {
public:
    enum class Weighting { tf_idf = 0, tf = 1, idf = 2, binary = 3 };  // DBoW2's WeightingType
    static constexpr int kDescriptorBytes = 32;

    bool empty() const { return words_.empty(); }
    std::size_t words() const { return words_.size(); }
    std::size_t nodes() const { return parent_.size(); }  // with the root
    int branching() const { return k_; }
    int levels() const { return L_; }
    Weighting weighting() const { return weighting_; }

    // Descriptors: N x 32 CV_8U, one per row. bow and feat are cleared first; empty if the vocabulary is.
    // levels_up as ComputeBoW (4): feat's nodes are levels() - levels_up below the root (the root if <= 0).
    void transform(const cv::Mat& descriptors, BowVector& bow, FeatureVector& feat, int levels_up) const;
    // One descriptor (32 bytes): its word and that word's weight (0: a stopped word, not added)
    void transform(const std::uint8_t* descriptor, WordId& word, WordValue& weight) const;
    // L1 score in [0, 1] of two L1-normalized vectors (DBoW2's L1Scoring)
    static double score(const BowVector& a, const BowVector& b);
    WordValue wordWeight(WordId w) const { return weight_[words_.at(w)]; }

    // Throw std::runtime_error when the file is missing or malformed, or uses a scoring other than L1
    void loadBinary(const std::filesystem::path& file);
    void loadText(const std::filesystem::path& file);
    void saveBinary(const std::filesystem::path& file) const;
    // Same tree, weights, words and descriptors, bit for bit (what differs, in why)
    bool sameAs(const Vocabulary& o, std::string* why = nullptr) const;

private:
    struct Header {
        int k = 0, L = 0, scoring = 0, weighting = 0;
    };
    void reset(const Header& h);
    // Appends a node under parent (parents come first); is_word makes it the next word
    void addNode(NodeId parent, bool is_word, const std::uint8_t* descriptor, WordValue weight);
    void finish();  // builds the children lists
    // Down the tree; the node at nid_level on the way, if nid
    NodeId descend(const std::uint8_t* descriptor, NodeId* nid, int nid_level) const;

    int k_ = 0, L_ = 0;
    Weighting weighting_ = Weighting::tf_idf;
    // Per node, node 0 the root (no descriptor)
    std::vector<NodeId> parent_;
    std::vector<std::uint8_t> descriptor_;  // kDescriptorBytes per node
    std::vector<WordValue> weight_;
    std::vector<WordId> word_of_;           // the node's word, or kNoWord
    std::vector<NodeId> child_begin_;       // children of n: child_[child_begin_[n] .. child_begin_[n + 1])
    std::vector<NodeId> child_;
    std::vector<NodeId> words_;             // word id -> node
    static constexpr WordId kNoWord = 0xffffffffu;
};

// Resource store: loaded from the binary file; a vocabulary is never written back
void fw_load(Vocabulary& v, const std::filesystem::path& file);
void fw_save(const Vocabulary& v, const std::filesystem::path& file);

}  // namespace map

FW_REGISTER_TYPE(map::Vocabulary, "map/Vocabulary")
