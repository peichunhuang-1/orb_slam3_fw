#include "types/map/vocabulary.hpp"

#include <cmath>
#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include "types/orb/hamming.hpp"

namespace map {

namespace {

constexpr char kMagic[8] = {'D', 'B', 'o', 'W', '2', 'B', 'I', 'N'};
constexpr std::uint32_t kVersion = 1;
constexpr int kL1Scoring = 0;  // DBoW2's L1_NORM

[[noreturn]] void fail(const std::filesystem::path& file, const std::string& what) {
    throw std::runtime_error("vocabulary " + file.string() + ": " + what);
}

}  // namespace

void Vocabulary::reset(const Header& h) {
    k_ = h.k;
    L_ = h.L;
    weighting_ = Weighting(h.weighting);
    parent_.assign(1, 0);
    descriptor_.assign(kDescriptorBytes, 0);
    weight_.assign(1, 0.0);
    word_of_.assign(1, kNoWord);
    child_begin_.clear();
    child_.clear();
    words_.clear();
}

void Vocabulary::addNode(NodeId parent, bool is_word, const std::uint8_t* descriptor, WordValue weight) {
    parent_.push_back(parent);
    descriptor_.insert(descriptor_.end(), descriptor, descriptor + kDescriptorBytes);
    weight_.push_back(weight);
    if (is_word) {
        word_of_.push_back(WordId(words_.size()));
        words_.push_back(NodeId(parent_.size() - 1));
    } else {
        word_of_.push_back(kNoWord);
    }
}

void Vocabulary::finish() {
    // Children in node order, as DBoW2 appends them while reading
    const std::size_t n = parent_.size();
    child_begin_.assign(n + 1, 0);
    for (std::size_t i = 1; i < n; ++i) ++child_begin_[parent_[i] + 1];
    for (std::size_t i = 0; i < n; ++i) child_begin_[i + 1] += child_begin_[i];
    child_.assign(n > 0 ? n - 1 : 0, 0);
    std::vector<NodeId> next(child_begin_.begin(), child_begin_.end() - 1);
    for (std::size_t i = 1; i < n; ++i) child_[next[parent_[i]]++] = NodeId(i);
}

NodeId Vocabulary::descend(const std::uint8_t* d, NodeId* nid, int nid_level) const {
    if (nid_level <= 0 && nid) *nid = 0;  // the root
    NodeId node = 0;
    int level = 0;
    do {
        ++level;
        const NodeId* c = child_.data() + child_begin_[node];
        const NodeId* end = child_.data() + child_begin_[node + 1];
        node = *c;
        int best = orb::hamming256(d, &descriptor_[std::size_t(node) * kDescriptorBytes]);
        for (++c; c != end; ++c) {
            const int dist = orb::hamming256(d, &descriptor_[std::size_t(*c) * kDescriptorBytes]);
            if (dist < best) {
                best = dist;
                node = *c;
            }
        }
        if (nid && level == nid_level) *nid = node;
    } while (child_begin_[node] != child_begin_[node + 1]);
    return node;
}

void Vocabulary::transform(const std::uint8_t* descriptor, WordId& word, WordValue& weight) const {
    const NodeId leaf = descend(descriptor, nullptr, 0);
    // A leaf that is no word (none in ORBvoc) is DBoW2's default word 0, with the node's weight
    word = word_of_[leaf] == kNoWord ? 0 : word_of_[leaf];
    weight = weight_[leaf];
}

void Vocabulary::transform(const cv::Mat& descriptors, BowVector& bow, FeatureVector& feat, int levels_up) const {
    bow.clear();
    feat.clear();
    if (empty()) return;
    if (descriptors.rows > 0 && (descriptors.type() != CV_8U || descriptors.cols != kDescriptorBytes))
        throw std::invalid_argument("Vocabulary::transform: descriptors must be N x 32 CV_8U");
    const int nid_level = L_ - levels_up;
    const bool accumulate = weighting_ == Weighting::tf_idf || weighting_ == Weighting::tf;
    for (int i = 0; i < descriptors.rows; ++i) {
        NodeId nid = 0;
        const NodeId leaf = descend(descriptors.ptr<std::uint8_t>(i), &nid, nid_level);
        const WordId word = word_of_[leaf] == kNoWord ? 0 : word_of_[leaf];
        const WordValue w = weight_[leaf];
        if (!(w > 0)) continue;  // a stopped word
        if (accumulate)
            bow[word] += w;
        else
            bow.emplace(word, w);  // IDF, binary: the first one only
        feat[nid].push_back(unsigned(i));
    }
    // L1 scoring normalizes (so TF's division by the word count never applies)
    double norm = 0;
    for (const auto& e : bow) norm += std::fabs(e.second);
    if (norm > 0)
        for (auto& e : bow) e.second /= norm;
}

double Vocabulary::score(const BowVector& a, const BowVector& b) {
    // ||a - b||_1 = 2 + sum over shared words of (|a_i - b_i| - |a_i| - |b_i|); score = 1 - ||a - b||_1 / 2
    auto i = a.begin(), j = b.begin();
    double s = 0;
    while (i != a.end() && j != b.end()) {
        if (i->first == j->first) {
            s += std::fabs(i->second - j->second) - std::fabs(i->second) - std::fabs(j->second);
            ++i;
            ++j;
        } else if (i->first < j->first) {
            i = a.lower_bound(j->first);
        } else {
            j = b.lower_bound(i->first);
        }
    }
    return -s / 2.0;
}

void Vocabulary::loadBinary(const std::filesystem::path& file) {
    std::ifstream f(file, std::ios::binary);
    if (!f) fail(file, "cannot open");
    std::vector<char> buf((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    const char* p = buf.data();
    const char* end = p + buf.size();
    auto take = [&](void* dst, std::size_t n) {
        if (std::size_t(end - p) < n) fail(file, "truncated");
        std::memcpy(dst, p, n);
        p += n;
    };
    char magic[8];
    take(magic, 8);
    if (std::memcmp(magic, kMagic, 8) != 0) fail(file, "not a binary vocabulary");
    std::uint32_t version = 0, nodes = 0, dbytes = 0;
    std::int32_t k = 0, L = 0, scoring = 0, weighting = 0;
    take(&version, 4);
    take(&k, 4);
    take(&L, 4);
    take(&scoring, 4);
    take(&weighting, 4);
    take(&nodes, 4);
    take(&dbytes, 4);
    if (version != kVersion) fail(file, "unsupported version " + std::to_string(version));
    if (dbytes != kDescriptorBytes || k < 0 || k > 20 || L < 1 || L > 10 || weighting < 0 || weighting > 3)
        fail(file, "unsupported header");
    if (scoring != kL1Scoring) fail(file, "only L1 scoring is supported");
    if (std::size_t(end - p) != std::size_t(nodes) * (4 + 1 + kDescriptorBytes + 8)) fail(file, "wrong size");

    reset({k, L, scoring, weighting});
    parent_.reserve(nodes + 1);
    descriptor_.reserve(std::size_t(nodes + 1) * kDescriptorBytes);
    weight_.reserve(nodes + 1);
    word_of_.reserve(nodes + 1);
    for (std::uint32_t i = 1; i <= nodes; ++i) {
        std::uint32_t parent = 0;
        std::uint8_t is_word = 0, d[kDescriptorBytes];
        double weight = 0;
        take(&parent, 4);
        take(&is_word, 1);
        take(d, kDescriptorBytes);
        take(&weight, 8);
        if (parent >= i) fail(file, "node " + std::to_string(i) + " has parent " + std::to_string(parent));
        addNode(parent, is_word != 0, d, weight);
    }
    finish();
}

void Vocabulary::loadText(const std::filesystem::path& file) {
    std::ifstream f(file);
    if (!f) fail(file, "cannot open");
    std::string line;
    if (!std::getline(f, line)) fail(file, "empty");
    Header h;
    {
        std::istringstream s(line);
        if (!(s >> h.k >> h.L >> h.scoring >> h.weighting)) fail(file, "bad header");
    }
    if (h.k < 0 || h.k > 20 || h.L < 1 || h.L > 10 || h.scoring < 0 || h.scoring > 5 || h.weighting < 0 ||
        h.weighting > 3)
        fail(file, "not a vocabulary text file");
    if (h.scoring != kL1Scoring) fail(file, "only L1 scoring is supported");
    reset(h);
    while (std::getline(f, line)) {
        if (line.empty()) continue;  // the end of the file (DBoW2 made one more node of it)
        std::istringstream s(line);
        long parent = 0;
        int is_leaf = 0;
        std::uint8_t d[kDescriptorBytes];
        double weight = 0;
        if (!(s >> parent >> is_leaf)) fail(file, "bad node " + std::to_string(parent_.size()));
        for (int b = 0; b < kDescriptorBytes; ++b) {
            int v = 0;
            if (!(s >> v)) fail(file, "bad descriptor of node " + std::to_string(parent_.size()));
            d[b] = std::uint8_t(v);
        }
        if (!(s >> weight)) fail(file, "bad weight of node " + std::to_string(parent_.size()));
        if (parent < 0 || std::size_t(parent) >= parent_.size())
            fail(file, "node " + std::to_string(parent_.size()) + " has parent " + std::to_string(parent));
        addNode(NodeId(parent), is_leaf > 0, d, weight);
    }
    finish();
}

void Vocabulary::saveBinary(const std::filesystem::path& file) const {
    std::ofstream f(file, std::ios::binary | std::ios::trunc);
    if (!f) fail(file, "cannot write");
    auto put = [&f](const void* src, std::size_t n) { f.write(static_cast<const char*>(src), std::streamsize(n)); };
    const std::uint32_t version = kVersion, nodes = std::uint32_t(parent_.size() - 1), dbytes = kDescriptorBytes;
    const std::int32_t k = k_, L = L_, scoring = kL1Scoring, weighting = std::int32_t(weighting_);
    put(kMagic, 8);
    put(&version, 4);
    put(&k, 4);
    put(&L, 4);
    put(&scoring, 4);
    put(&weighting, 4);
    put(&nodes, 4);
    put(&dbytes, 4);
    for (std::size_t i = 1; i < parent_.size(); ++i) {
        const std::uint32_t parent = parent_[i];
        const std::uint8_t is_word = word_of_[i] != kNoWord;
        const double weight = weight_[i];
        put(&parent, 4);
        put(&is_word, 1);
        put(&descriptor_[i * kDescriptorBytes], kDescriptorBytes);
        put(&weight, 8);
    }
    if (!f) fail(file, "write failed");
}

bool Vocabulary::sameAs(const Vocabulary& o, std::string* why) const {
    auto differ = [why](const std::string& what) {
        if (why) *why = what;
        return false;
    };
    if (k_ != o.k_ || L_ != o.L_ || weighting_ != o.weighting_) return differ("parameters (k, L, weighting)");
    if (parent_ != o.parent_) return differ("tree");
    if (word_of_ != o.word_of_ || words_ != o.words_) return differ("words");
    if (std::memcmp(weight_.data(), o.weight_.data(), weight_.size() * sizeof(double)) != 0) return differ("weights");
    if (descriptor_ != o.descriptor_) return differ("descriptors");
    return true;
}

}  // namespace map
