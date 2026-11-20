#pragma once
// Sorted-vector containers with the std::map / std::set members the atlas uses, iterated in key order as those
// are. They live inside atlas tiles (types/map/docs/DESIGN.md): a tile is copied whole on its first edit in a
// version, and one contiguous block copies far cheaper than a node-based tree.
#include <algorithm>
#include <cstddef>
#include <initializer_list>
#include <stdexcept>
#include <utility>
#include <vector>

namespace map {

template <class K, class V>
class FlatMap {
public:
    using value_type = std::pair<K, V>;
    using iterator = typename std::vector<value_type>::iterator;
    using const_iterator = typename std::vector<value_type>::const_iterator;

    FlatMap() = default;
    // From pairs in any order; the first of equal keys is kept
    template <class It>
    FlatMap(It first, It last) {
        for (; first != last; ++first) emplace(first->first, first->second);
    }

    iterator begin() { return v_.begin(); }
    iterator end() { return v_.end(); }
    const_iterator begin() const { return v_.begin(); }
    const_iterator end() const { return v_.end(); }
    std::size_t size() const { return v_.size(); }
    bool empty() const { return v_.empty(); }
    void clear() { v_.clear(); }

    iterator find(const K& k) {
        const auto it = lower(k);
        return it != v_.end() && it->first == k ? it : v_.end();
    }
    const_iterator find(const K& k) const {
        const auto it = lower(k);
        return it != v_.end() && it->first == k ? it : v_.end();
    }
    std::size_t count(const K& k) const { return find(k) != end() ? 1 : 0; }
    const V& at(const K& k) const {
        const auto it = find(k);
        if (it == end()) throw std::out_of_range("map::FlatMap::at");
        return it->second;
    }
    V& operator[](const K& k) {
        auto it = lower(k);
        if (it == v_.end() || it->first != k) it = v_.insert(it, value_type(k, V()));
        return it->second;
    }
    // Inserted (true) or already there (false, unchanged)
    std::pair<iterator, bool> emplace(const K& k, const V& val) {
        auto it = lower(k);
        if (it != v_.end() && it->first == k) return {it, false};
        return {v_.insert(it, value_type(k, val)), true};
    }
    std::size_t erase(const K& k) {
        const auto it = find(k);
        if (it == v_.end()) return 0;
        v_.erase(it);
        return 1;
    }
    iterator erase(const_iterator it) { return v_.erase(it); }

private:
    iterator lower(const K& k) {
        return std::lower_bound(v_.begin(), v_.end(), k, [](const value_type& e, const K& x) { return e.first < x; });
    }
    const_iterator lower(const K& k) const {
        return std::lower_bound(v_.begin(), v_.end(), k, [](const value_type& e, const K& x) { return e.first < x; });
    }
    std::vector<value_type> v_;
};

template <class K>
class FlatSet {
public:
    using value_type = K;
    using const_iterator = typename std::vector<K>::const_iterator;
    using iterator = const_iterator;

    FlatSet() = default;
    FlatSet(std::initializer_list<K> l) {
        for (const K& k : l) insert(k);
    }

    const_iterator begin() const { return v_.begin(); }
    const_iterator end() const { return v_.end(); }
    std::size_t size() const { return v_.size(); }
    bool empty() const { return v_.empty(); }
    void clear() { v_.clear(); }

    const_iterator find(const K& k) const {
        const auto it = std::lower_bound(v_.begin(), v_.end(), k);
        return it != v_.end() && *it == k ? it : v_.end();
    }
    std::size_t count(const K& k) const { return find(k) != end() ? 1 : 0; }
    std::pair<const_iterator, bool> insert(const K& k) {
        const auto it = std::lower_bound(v_.begin(), v_.end(), k);
        if (it != v_.end() && *it == k) return {it, false};
        return {v_.insert(it, k), true};
    }
    std::size_t erase(const K& k) {
        const auto it = find(k);
        if (it == v_.end()) return 0;
        v_.erase(it);
        return 1;
    }
    const_iterator erase(const_iterator it) { return v_.erase(it); }

private:
    std::vector<K> v_;
};

}  // namespace map
