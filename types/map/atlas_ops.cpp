#include "types/map/atlas_ops.hpp"

#include <algorithm>
#include <climits>
#include <stdexcept>
#include <utility>

#include "types/orb/hamming.hpp"

namespace map {

namespace {

// Pairs (weight, keyframe) ascending, as std::sort of ORB-SLAM3's pair<int, KeyFrame*> (ties by id here)
void sortedDescending(std::vector<std::pair<int, KeyFrameId>> pairs, std::vector<KeyFrameId>& kfs,
                      std::vector<int>& ws) {
    std::sort(pairs.begin(), pairs.end());
    kfs.clear();
    ws.clear();
    for (auto it = pairs.rbegin(); it != pairs.rend(); ++it) {  // push_front of the ascending list
        kfs.push_back(it->second);
        ws.push_back(it->first);
    }
}

bool sameCamera(const camera::KannalaBrandt8& a, const camera::KannalaBrandt8& b) {
    if (a.fx() != b.fx() || a.fy() != b.fy() || a.cx() != b.cx() || a.cy() != b.cy()) return false;
    for (int i = 0; i < 4; ++i)
        if (a.k(i) != b.k(i)) return false;
    return a.width() == b.width() && a.height() == b.height();
}

}  // namespace

// ---- Atlas

Map& Atlas::currentMap() {
    if (!hasCurrentMap()) createNewMap(*this);
    return maps.at(current);
}

const Map& Atlas::currentMap() const {
    const Map* m = findMap(current);
    if (!m) throw std::logic_error("map::Atlas: no current map");
    return *m;
}

// ---- maps

MapId createNewMap(Atlas& a) {
    if (Map* cur = a.findMap(a.current)) {
        if (!a.maps.empty() && a.last_init_kf_id < cur->max_kf_id) a.last_init_kf_id = cur->max_kf_id + 1;
        cur->in_use = false;  // SetStoredMap
    }
    Map m;  // Map(initKFid)
    m.id = a.next_map_id++;
    m.init_kf_id = a.last_init_kf_id;
    m.max_kf_id = a.last_init_kf_id;
    m.in_use = true;  // SetCurrentMap
    a.current = m.id;
    a.maps.emplace(m.id, std::move(m));
    return a.current;
}

void clearMap(Atlas& a, MapId id) {
    Map* m = a.findMap(id);
    if (!m) return;
    for (KeyFrameId k : m->keyframes)
        if (KeyFrame* kf = a.keyframe(k)) kf->map = kNone;  // UpdateMap(NULL)
    m->points.clear();
    m->keyframes.clear();
    m->max_kf_id = m->init_kf_id;
    m->imu_initialized = false;
    m->reference_points.clear();
    m->keyframe_origins.clear();
    m->imu_ba1 = false;
    m->imu_ba2 = false;
}

void eraseMapFromBowIndex(Atlas& a, MapId m) {
    a.bow_index.eraseIf([&a, m](KeyFrameId k) {
        const KeyFrame* kf = a.keyframe(k);
        return kf && kf->map == m;
    });
}

void clearAtlas(Atlas& a) {
    a.bow_index.clear();
    a.maps.clear();
    a.current = kNone;
    a.last_init_kf_id = 0;
    a.keyframes.clear();  // KeyFrame::nNextId = 0
    a.points.clear();
}

int addRig(Atlas& a, const camera::StereoRig& rig) {
    for (std::size_t i = 0; i < a.rigs.size(); ++i) {
        const camera::StereoRig& r = a.rigs[i];
        if (sameCamera(r.camera(0), rig.camera(0)) && sameCamera(r.camera(1), rig.camera(1)) &&
            r.R_12() == rig.R_12() && r.t_12() == rig.t_12())
            return int(i);
    }
    a.rigs.push_back(rig);
    return int(a.rigs.size() - 1);
}

// ---- keyframes

KeyFrameId createKeyFrame(Atlas& a, KeyFrame kf) {
    const Map& m = a.currentMap();
    kf.id = KeyFrameId(a.keyframes.size());
    kf.map = m.id;
    kf.origin_map = m.id;
    kf.imu = m.imu_initialized;
    if (kf.points.size() != std::size_t(kf.size())) kf.points.assign(std::size_t(kf.size()), kNone);
    a.keyframes.push_back(std::make_unique<KeyFrame>(std::move(kf)));
    return a.keyframes.back()->id;
}

void insertKeyFrame(Atlas& a, KeyFrameId k) {
    KeyFrame* kf = a.keyframe(k);
    if (!kf) return;
    Map* m = a.findMap(kf->map);
    if (!m) return;
    if (m->keyframes.empty()) {
        m->init_kf_id = k;
        m->initial_kf = k;
        m->lower_kf = k;
    }
    m->keyframes.insert(k);
    if (k > m->max_kf_id) m->max_kf_id = k;
    if (k < m->lower_kf) m->lower_kf = k;
}

void computeBoW(KeyFrame& kf, const Vocabulary& voc) {
    if (!kf.bow.empty() && !kf.feat.empty()) return;
    cv::Mat all;  // mDescriptors: left then right
    cv::vconcat(kf.features->stereo->descriptors[0], kf.features->stereo->descriptors[1], all);
    voc.transform(all, kf.bow, kf.feat, 4);
}

void addToBowIndex(Atlas& a, KeyFrameId k) {
    if (const KeyFrame* kf = a.keyframe(k)) a.bow_index.add(k, kf->bow);
}

void addConnection(Atlas& a, KeyFrameId k, KeyFrameId other, int weight) {
    KeyFrame* kf = a.keyframe(k);
    if (!kf) return;
    const auto it = kf->connected.find(other);
    if (it != kf->connected.end() && it->second == weight) return;
    kf->connected[other] = weight;
    updateBestCovisibles(a, k);
}

void eraseConnection(Atlas& a, KeyFrameId k, KeyFrameId other) {
    KeyFrame* kf = a.keyframe(k);
    if (!kf || !kf->connected.erase(other)) return;
    updateBestCovisibles(a, k);
}

void updateBestCovisibles(Atlas& a, KeyFrameId k) {
    KeyFrame* kf = a.keyframe(k);
    if (!kf) return;
    std::vector<std::pair<int, KeyFrameId>> pairs;
    pairs.reserve(kf->connected.size());
    for (const auto& c : kf->connected)
        if (a.goodKeyFrame(c.first)) pairs.emplace_back(c.second, c.first);
    sortedDescending(std::move(pairs), kf->ordered_connected, kf->ordered_weights);
}

void updateConnections(Atlas& a, KeyFrameId k) {
    KeyFrame* kf = a.keyframe(k);
    if (!kf) return;
    std::map<KeyFrameId, int> counter;
    for (MapPointId pid : kf->points) {
        const MapPoint* p = a.goodPoint(pid);
        if (!p) continue;
        for (const auto& o : p->observations) {
            const KeyFrame* other = a.goodKeyFrame(o.first);
            if (o.first == k || !other || other->map != kf->map) continue;
            ++counter[o.first];
        }
    }
    if (counter.empty()) return;

    constexpr int th = 15;
    int nmax = 0;
    KeyFrameId kfmax = kNone;
    std::vector<std::pair<int, KeyFrameId>> pairs;
    pairs.reserve(counter.size());
    for (const auto& c : counter) {
        if (c.second > nmax) {
            nmax = c.second;
            kfmax = c.first;
        }
        if (c.second >= th) {
            pairs.emplace_back(c.second, c.first);
            addConnection(a, c.first, k, c.second);
        }
    }
    if (pairs.empty()) {
        pairs.emplace_back(nmax, kfmax);
        addConnection(a, kfmax, k, nmax);
    }
    kf->connected = counter;
    sortedDescending(std::move(pairs), kf->ordered_connected, kf->ordered_weights);
    const Map* m = a.findMap(kf->map);
    if (kf->first_connection && m && k != m->init_kf_id) {
        kf->parent = kf->ordered_connected.front();
        if (KeyFrame* parent = a.keyframe(kf->parent)) parent->children.insert(k);
        kf->first_connection = false;
    }
}

std::vector<KeyFrameId> covisiblesByWeight(const KeyFrame& kf, int w) {
    if (kf.ordered_connected.empty()) return {};
    // upper_bound with a > b on the descending weights: the first weight below w
    const auto it = std::upper_bound(kf.ordered_weights.begin(), kf.ordered_weights.end(), w,
                                     [](int a, int b) { return a > b; });
    if (it == kf.ordered_weights.end() && kf.ordered_weights.back() < w) return {};
    const std::size_t n = std::size_t(it - kf.ordered_weights.begin());
    return std::vector<KeyFrameId>(kf.ordered_connected.begin(), kf.ordered_connected.begin() + long(n));
}

void changeParent(Atlas& a, KeyFrameId k, KeyFrameId parent) {
    if (k == parent) throw std::invalid_argument("map::changeParent: the parent and child can not be the same");
    KeyFrame* kf = a.keyframe(k);
    if (!kf) return;
    kf->parent = parent;
    if (KeyFrame* p = a.keyframe(parent)) p->children.insert(k);
}

void setKeyFrameBad(Atlas& a, KeyFrameId k) {
    KeyFrame* kf = a.keyframe(k);
    if (!kf) return;
    Map* m = a.findMap(kf->map);
    if (m && k == m->init_kf_id) return;
    if (kf->not_erase) {
        kf->to_be_erased = true;
        return;
    }
    const std::map<KeyFrameId, int> connected = kf->connected;
    for (const auto& c : connected) eraseConnection(a, c.first, k);
    for (MapPointId pid : kf->points)
        if (a.point(pid)) eraseObservation(a, pid, k);

    kf->connected.clear();
    kf->ordered_connected.clear();
    // Update the spanning tree: each child goes to the parent candidate it is most covisible with
    std::set<KeyFrameId> candidates;
    if (kf->parent != kNone) candidates.insert(kf->parent);
    while (!kf->children.empty()) {
        bool found = false;
        int max = -1;
        KeyFrameId child = kNone, parent = kNone;
        for (KeyFrameId c : kf->children) {
            const KeyFrame* ck = a.goodKeyFrame(c);
            if (!ck) continue;
            for (KeyFrameId v : ck->ordered_connected)
                for (KeyFrameId cand : candidates)
                    if (v == cand) {
                        const int w = ck->weight(v);
                        if (w > max) {
                            child = c;
                            parent = v;
                            max = w;
                            found = true;
                        }
                    }
        }
        if (!found) break;
        changeParent(a, child, parent);
        candidates.insert(child);
        kf->children.erase(child);
    }
    // The ones left (no covisibility with a candidate) go to the parent
    if (!kf->children.empty() && kf->parent != kNone)
        for (KeyFrameId c : std::set<KeyFrameId>(kf->children)) changeParent(a, c, kf->parent);
    if (KeyFrame* p = a.keyframe(kf->parent)) {
        p->children.erase(k);
        kf->Tcp = kf->pose() * p->poseInverse();
    }
    kf->bad = true;

    if (m) {  // Map::EraseKeyFrame
        m->keyframes.erase(k);
        if (!m->keyframes.empty()) {
            if (k == m->lower_kf) m->lower_kf = *m->keyframes.begin();
        } else {
            m->lower_kf = kNone;
        }
    }
    a.bow_index.erase(k, kf->bow);
}

int trackedMapPoints(const Atlas& a, const KeyFrame& kf, int min_obs) {
    int n = 0;
    const bool check = min_obs > 0;
    for (int i = 0; i < kf.size(); ++i) {
        const MapPoint* p = a.goodPoint(kf.points[std::size_t(i)]);
        if (!p) continue;
        if (!check || p->n_obs >= min_obs) ++n;
    }
    return n;
}

// ---- map points

MapPointId createMapPoint(Atlas& a, const Eigen::Vector3f& pos, KeyFrameId ref) {
    const KeyFrame* kf = a.keyframe(ref);
    if (!kf) throw std::invalid_argument("map::createMapPoint: no reference keyframe");
    auto p = std::make_unique<MapPoint>();
    p->id = MapPointId(a.points.size());
    p->map = kf->map;
    p->origin_map = kf->map;
    p->first_kf = ref;
    p->first_frame = kf->frame_id;
    p->ref_kf = ref;
    p->pos = pos;
    a.points.push_back(std::move(p));
    return a.points.back()->id;
}

void insertMapPoint(Atlas& a, MapPointId pid) {
    const MapPoint* p = a.point(pid);
    if (!p) return;
    if (Map* m = a.findMap(p->map)) m->points.insert(pid);
}

void addObservation(Atlas& a, MapPointId pid, KeyFrameId k, int index) {
    MapPoint* p = a.point(pid);
    const KeyFrame* kf = a.keyframe(k);
    if (!p || !kf) return;
    Observation& o = p->observations[k];  // (-1, -1) if new
    if (index >= kf->nLeft())
        o.right = index;
    else
        o.left = index;
    ++p->n_obs;  // two cameras: each feature counts one
}

void eraseObservation(Atlas& a, MapPointId pid, KeyFrameId k) {
    MapPoint* p = a.point(pid);
    if (!p) return;
    const auto it = p->observations.find(k);
    if (it == p->observations.end()) return;
    if (it->second.left != -1) --p->n_obs;
    if (it->second.right != -1) --p->n_obs;
    p->observations.erase(it);
    if (p->ref_kf == k) p->ref_kf = p->observations.empty() ? kNone : p->observations.begin()->first;
    if (p->n_obs <= 2) setPointBad(a, pid);
}

void setPointBad(Atlas& a, MapPointId pid) {
    MapPoint* p = a.point(pid);
    if (!p) return;
    p->bad = true;
    const std::map<KeyFrameId, Observation> obs = std::move(p->observations);
    p->observations.clear();
    for (const auto& o : obs) {
        KeyFrame* kf = a.keyframe(o.first);
        if (!kf) continue;
        if (o.second.left != -1) kf->points[std::size_t(o.second.left)] = kNone;
        if (o.second.right != -1) kf->points[std::size_t(o.second.right)] = kNone;
    }
    if (Map* m = a.findMap(p->map)) m->points.erase(pid);
}

void replace(Atlas& a, MapPointId pid, MapPointId by) {
    if (pid == by) return;
    MapPoint* p = a.point(pid);
    MapPoint* q = a.point(by);
    if (!p || !q) return;
    const std::map<KeyFrameId, Observation> obs = std::move(p->observations);
    p->observations.clear();
    p->bad = true;
    const int visible = p->visible, found = p->found;
    p->replaced = by;
    for (const auto& o : obs) {
        KeyFrame* kf = a.keyframe(o.first);
        if (!kf) continue;
        if (!q->inKeyFrame(o.first)) {
            if (o.second.left != -1) {
                kf->points[std::size_t(o.second.left)] = by;
                addObservation(a, by, o.first, o.second.left);
            }
            if (o.second.right != -1) {
                kf->points[std::size_t(o.second.right)] = by;
                addObservation(a, by, o.first, o.second.right);
            }
        } else {
            if (o.second.left != -1) kf->points[std::size_t(o.second.left)] = kNone;
            if (o.second.right != -1) kf->points[std::size_t(o.second.right)] = kNone;
        }
    }
    q->found += found;
    q->visible += visible;
    computeDistinctiveDescriptors(a, by);
    if (Map* m = a.findMap(p->map)) m->points.erase(pid);
}

void computeDistinctiveDescriptors(Atlas& a, MapPointId pid) {
    MapPoint* p = a.point(pid);
    if (!p || p->bad || p->observations.empty()) return;
    std::vector<const std::uint8_t*> d;
    d.reserve(p->observations.size() * 2);
    for (const auto& o : p->observations) {
        const KeyFrame* kf = a.goodKeyFrame(o.first);
        if (!kf) continue;
        if (o.second.left != -1) d.push_back(kf->features->descriptor(o.second.left));
        if (o.second.right != -1) d.push_back(kf->features->descriptor(o.second.right));
    }
    if (d.empty()) return;
    const std::size_t n = d.size();
    std::vector<int> dist(n * n, 0);
    for (std::size_t i = 0; i < n; ++i)
        for (std::size_t j = i + 1; j < n; ++j) dist[i * n + j] = dist[j * n + i] = orb::hamming256(d[i], d[j]);
    // The one with the least median distance to the others
    int best_median = INT_MAX;
    std::size_t best = 0;
    std::vector<int> row(n);
    for (std::size_t i = 0; i < n; ++i) {
        std::copy(dist.begin() + long(i * n), dist.begin() + long((i + 1) * n), row.begin());
        std::sort(row.begin(), row.end());
        const int median = row[std::size_t(0.5 * double(n - 1))];
        if (median < best_median) {
            best_median = median;
            best = i;
        }
    }
    std::copy(d[best], d[best] + 32, p->descriptor.begin());
}

void updateNormalAndDepth(Atlas& a, MapPointId pid) {
    MapPoint* p = a.point(pid);
    if (!p || p->bad || p->observations.empty()) return;
    const KeyFrame* ref = a.keyframe(p->ref_kf);
    if (!ref) return;
    Eigen::Vector3f normal = Eigen::Vector3f::Zero();
    int n = 0;
    for (const auto& o : p->observations) {
        const KeyFrame* kf = a.keyframe(o.first);
        if (!kf) continue;
        if (o.second.left != -1) {
            const Eigen::Vector3f v = p->pos - kf->cameraCenter();
            normal = normal + v / v.norm();
            ++n;
        }
        if (o.second.right != -1) {
            const Eigen::Vector3f v = p->pos - kf->rightCameraCenter();
            normal = normal + v / v.norm();
            ++n;
        }
    }
    const float dist = (p->pos - ref->cameraCenter()).norm();
    const Observation o = p->indexIn(p->ref_kf);
    const int index = o.left != -1 ? o.left : o.right;
    if (index < 0) return;  // the reference keyframe does not see it (ORB-SLAM3 reads out of bounds)
    const int level = ref->features->keypoint(index).octave;
    p->max_distance = dist * ref->features->scale_factors[std::size_t(level)];
    p->min_distance = p->max_distance / ref->features->scale_factors[std::size_t(ref->features->levels - 1)];
    p->normal = normal / float(n);
}

MapPointId createStereoPoint(Atlas& a, KeyFrameId k, int i, const Eigen::Vector3f& pos) {
    const MapPointId pid = createMapPoint(a, pos, k);
    KeyFrame* kf = a.keyframe(k);
    addObservation(a, pid, k, i);
    const int r = kf->features->rightOf(i);
    if (r >= 0) {
        const int ri = kf->nLeft() + r;
        addObservation(a, pid, k, ri);
        kf->points[std::size_t(ri)] = pid;
    }
    kf->points[std::size_t(i)] = pid;
    computeDistinctiveDescriptors(a, pid);
    updateNormalAndDepth(a, pid);
    insertMapPoint(a, pid);
    return pid;
}

}  // namespace map
