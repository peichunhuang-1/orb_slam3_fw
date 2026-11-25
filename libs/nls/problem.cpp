#include "libs/nls/problem.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_map>

#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/SparseCholesky>
#include <Eigen/SparseCore>

namespace nls {

// ---- manifolds and losses

void Euclidean::plus(const double* x, const double* delta, double* x_plus) const {
    for (int i = 0; i < n_; ++i) x_plus[i] = x[i] + delta[i];
}

void SE3Left::plus(const double* x, const double* delta, double* x_plus) const {
    const Eigen::Quaterniond q(x[3], x[0], x[1], x[2]);
    const Eigen::Vector3d t(x[4], x[5], x[6]);
    const Eigen::Vector3d w(delta[0], delta[1], delta[2]), u(delta[3], delta[4], delta[5]);
    const double theta = w.norm();
    Eigen::Matrix3d W;
    W << 0, -w.z(), w.y(), w.z(), 0, -w.x(), -w.y(), w.x(), 0;
    Eigen::Quaterniond qd;
    Eigen::Matrix3d V;
    if (theta < 1e-8) {
        qd = Eigen::Quaterniond(1, 0.5 * w.x(), 0.5 * w.y(), 0.5 * w.z()).normalized();
        V = Eigen::Matrix3d::Identity() + 0.5 * W + W * W / 6;
    } else {
        qd = Eigen::Quaterniond(Eigen::AngleAxisd(theta, w / theta));
        const double t2 = theta * theta;
        V = Eigen::Matrix3d::Identity() + (1 - std::cos(theta)) / t2 * W + (theta - std::sin(theta)) / (t2 * theta) * W * W;
    }
    const Eigen::Quaterniond qn = (qd * q).normalized();
    const Eigen::Vector3d tn = qd * t + V * u;
    x_plus[0] = qn.x();
    x_plus[1] = qn.y();
    x_plus[2] = qn.z();
    x_plus[3] = qn.w();
    x_plus[4] = tn.x();
    x_plus[5] = tn.y();
    x_plus[6] = tn.z();
}

void Huber::evaluate(double s, double rho[2]) const {
    if (s <= delta_sq_) {
        rho[0] = s;
        rho[1] = 1;
    } else {
        const double r = std::sqrt(s);
        rho[0] = 2 * r * delta_ - delta_sq_;
        rho[1] = delta_ / r;
    }
}

// ---- small dense kernels on column-major storage: fixed-size code for the usual dimensions (2, 3, 6, 9: pixel
// and IMU residuals, points, poses), a dynamic fallback otherwise

namespace {

template <int M, int N, int K>
void abtFixed(double* C, const double* A, const double* B, double alpha) {
    Eigen::Map<Eigen::Matrix<double, M, N>>(C).noalias() +=
        alpha * (Eigen::Map<const Eigen::Matrix<double, M, K>>(A) * Eigen::Map<const Eigen::Matrix<double, N, K>>(B).transpose());
}
template <int M, int N, int K>
void abFixed(double* C, const double* A, const double* B) {
    Eigen::Map<Eigen::Matrix<double, M, N>>(C).noalias() =
        Eigen::Map<const Eigen::Matrix<double, M, K>>(A) * Eigen::Map<const Eigen::Matrix<double, K, N>>(B);
}
template <int M, int K>
void axFixed(double* y, const double* A, const double* x, double alpha) {
    Eigen::Map<Eigen::Matrix<double, M, 1>>(y).noalias() +=
        alpha * (Eigen::Map<const Eigen::Matrix<double, M, K>>(A) * Eigen::Map<const Eigen::Matrix<double, K, 1>>(x));
}
template <int M, int K>
void atxFixed(double* y, const double* A, const double* x, double alpha) {
    Eigen::Map<Eigen::Matrix<double, M, 1>>(y).noalias() +=
        alpha * (Eigen::Map<const Eigen::Matrix<double, K, M>>(A).transpose() * Eigen::Map<const Eigen::Matrix<double, K, 1>>(x));
}

constexpr int code3(int m, int n, int k) { return (m << 8) | (n << 4) | k; }
constexpr int code2(int m, int k) { return (m << 4) | k; }
bool small(int d) { return d < 16; }

#define NLS_FOR_K3(F, M, N) F(M, N, 2) F(M, N, 3) F(M, N, 6) F(M, N, 9)
#define NLS_FOR_N3(F, M) NLS_FOR_K3(F, M, 2) NLS_FOR_K3(F, M, 3) NLS_FOR_K3(F, M, 6) NLS_FOR_K3(F, M, 9)
#define NLS_FOR3(F) NLS_FOR_N3(F, 2) NLS_FOR_N3(F, 3) NLS_FOR_N3(F, 6) NLS_FOR_N3(F, 9)
#define NLS_FOR_K2(F, M) F(M, 2) F(M, 3) F(M, 6) F(M, 9)
#define NLS_FOR2(F) NLS_FOR_K2(F, 2) NLS_FOR_K2(F, 3) NLS_FOR_K2(F, 6) NLS_FOR_K2(F, 9)

// C (m x n) += alpha · A (m x k) · Bᵀ (B: n x k)
void gemmABt(double* C, const double* A, const double* B, int m, int n, int k, double alpha) {
    if (small(m) && small(n) && small(k)) switch (code3(m, n, k)) {
#define NLS_CASE(M, N, K) \
    case code3(M, N, K): abtFixed<M, N, K>(C, A, B, alpha); return;
            NLS_FOR3(NLS_CASE)
#undef NLS_CASE
            default: break;
        }
    Eigen::Map<Eigen::MatrixXd>(C, m, n).noalias() +=
        alpha * (Eigen::Map<const Eigen::MatrixXd>(A, m, k) * Eigen::Map<const Eigen::MatrixXd>(B, n, k).transpose());
}

// C (m x n) = A (m x k) · B (k x n)
void gemmAB(double* C, const double* A, const double* B, int m, int n, int k) {
    if (small(m) && small(n) && small(k)) switch (code3(m, n, k)) {
#define NLS_CASE(M, N, K) \
    case code3(M, N, K): abFixed<M, N, K>(C, A, B); return;
            NLS_FOR3(NLS_CASE)
#undef NLS_CASE
            default: break;
        }
    Eigen::Map<Eigen::MatrixXd>(C, m, n).noalias() =
        Eigen::Map<const Eigen::MatrixXd>(A, m, k) * Eigen::Map<const Eigen::MatrixXd>(B, k, n);
}

// y (m) += alpha · A (m x k) · x
void gemv(double* y, const double* A, const double* x, int m, int k, double alpha) {
    if (small(m) && small(k)) switch (code2(m, k)) {
#define NLS_CASE(M, K) \
    case code2(M, K): axFixed<M, K>(y, A, x, alpha); return;
            NLS_FOR2(NLS_CASE)
#undef NLS_CASE
            default: break;
        }
    Eigen::Map<Eigen::VectorXd>(y, m).noalias() +=
        alpha * (Eigen::Map<const Eigen::MatrixXd>(A, m, k) * Eigen::Map<const Eigen::VectorXd>(x, k));
}

// y (m) += alpha · Aᵀ · x with A k x m
void gemvT(double* y, const double* A, const double* x, int m, int k, double alpha) {
    if (small(m) && small(k)) switch (code2(m, k)) {
#define NLS_CASE(M, K) \
    case code2(M, K): atxFixed<M, K>(y, A, x, alpha); return;
            NLS_FOR2(NLS_CASE)
#undef NLS_CASE
            default: break;
        }
    Eigen::Map<Eigen::VectorXd>(y, m).noalias() +=
        alpha * (Eigen::Map<const Eigen::MatrixXd>(A, k, m).transpose() * Eigen::Map<const Eigen::VectorXd>(x, k));
}

// A (n x n) in place: its inverse
void invert(double* A, int n) {
    if (n == 3) {
        Eigen::Map<Eigen::Matrix3d> M(A);
        const Eigen::Matrix3d m = M;
        M = m.inverse();
    } else {
        Eigen::Map<Eigen::MatrixXd> M(A, n, n);
        const Eigen::MatrixXd m = M;
        M = m.inverse();
    }
}

std::uint64_t key(int a, int b) { return (std::uint64_t(std::uint32_t(a)) << 32) | std::uint32_t(b); }

struct Block {
    double* x = nullptr;
    std::shared_ptr<const Manifold> manifold;
    int ambient = 0, tangent = 0;
    bool fixed = false, eliminated = false;
};

struct Res {
    std::shared_ptr<const Residual> owner;
    const Residual* residual = nullptr;
    std::vector<int> blocks;
    Eigen::MatrixXd information;  // empty: info_scale · identity
    double info_scale = 1;
    std::shared_ptr<const Loss> loss;
    bool active = true;
    int rows = 0;
    std::size_t x_off = 0;  // its blocks' values in Impl::xs
    std::size_t e_off = 0;  // its error in Impl::ebuf
};

}  // namespace

// ---- problem
//
// Solve-time layout (setup): free blocks are numbered cameras first (not eliminated), then the eliminated ones.
// The Hessian blocks (lo, hi), lo <= hi in that numbering, live in one buffer: the camera-camera blocks first (so
// the Schur complement starts from a copy of that prefix), then each eliminated block's camera blocks, then the
// eliminated diagonals. A residual's Jacobians are computed into a scratch and accumulated right away.
struct Problem::Impl {
    std::vector<Block> blocks;
    std::vector<Res> res;
    std::vector<const double*> xs;  // per residual block, its values
    std::vector<double> ebuf;       // the residuals' errors
    std::vector<double*> none;      // null Jacobian pointers, as many as the most blocks of a residual

    // ---- solve-time structure
    std::vector<int> active;      // residuals in the solve
    std::vector<int> gid;         // block -> free index (-1: not optimized)
    std::vector<int> free_block;  // free index -> block
    std::vector<int> offset;      // free index -> offset in the tangent vector
    std::vector<int> tangent;     // free index -> tangent size
    int n_cam = 0, cam_dim = 0, total_dim = 0;

    std::vector<int> rgid;                // aligned with xs: free index of the block, -1 if not free
    std::vector<std::size_t> rjoff;       // aligned with xs: the block's Jacobian offset in the scratch
    std::vector<std::size_t> pair_begin;  // per active index, its pairs [pair_begin[i], pair_begin[i + 1])
    struct Pair {
        std::size_t ja, jb;  // Jacobian offsets in the scratch, free index of a <= b
        int ta, tb;
        std::size_t h;       // offset of the (a, b) block in hbuf
    };
    std::vector<Pair> pairs;
    std::vector<double> jbuf, we, scratch;
    std::vector<double*> jptr;

    std::vector<double> hbuf, sbuf;
    std::size_t cc_size = 0;            // the camera-camera prefix of hbuf
    std::vector<std::size_t> diag_off;  // free index -> offset of its diagonal block
    // per eliminated block e (free index - n_cam): its cameras, ascending, and the offsets of (cam, e) in hbuf;
    // the offsets in sbuf of (cams[i], cams[j]), i <= j
    std::vector<std::size_t> nb_begin, cp_begin;
    std::vector<int> nb_cam;
    std::vector<std::size_t> nb_h, cp_s;
    std::vector<double> hinv;
    std::vector<std::size_t> hinv_off;
    std::vector<double> qbuf;
    Eigen::VectorXd b, dx, rhs, hib;

    Eigen::SparseMatrix<double> A;
    std::vector<std::size_t> fill_src;  // sbuf offsets
    std::vector<int> fill_dst;          // A's value indices
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>, Eigen::Upper, Eigen::AMDOrdering<int>> ldlt;
    bool analyzed = false;
    std::vector<double> backup;
    // ebuf holds the active residuals' errors at the current parameters: set by computeActiveErrors, cleared by
    // pop (ebuf then holds the rejected trial's). step() skips its first computeActiveErrors while it is set.
    bool errors_current = false;

    double chi2(const Res& R) const {
        const Eigen::Map<const Eigen::VectorXd> e(ebuf.data() + R.e_off, R.rows);
        if (!R.information.size()) return R.info_scale * e.squaredNorm();
        double c = 0;
        for (int j = 0; j < R.rows; ++j) c += e[j] * R.information.col(j).dot(e);
        return c;
    }

    void setup() {
        const std::size_t nb = blocks.size();
        std::vector<char> used(nb, 0);
        active.clear();
        std::size_t most = 0;
        for (std::size_t r = 0; r < res.size(); ++r) {
            most = std::max(most, res[r].blocks.size());
            if (!res[r].active) continue;
            active.push_back(int(r));
            for (int bi : res[r].blocks)
                if (!blocks[std::size_t(bi)].fixed) used[std::size_t(bi)] = 1;
        }
        none.assign(most, nullptr);
        jptr.assign(most, nullptr);

        gid.assign(nb, -1);
        free_block.clear();
        for (int pass = 0; pass < 2; ++pass)
            for (std::size_t i = 0; i < nb; ++i)
                if (used[i] && blocks[i].eliminated == (pass == 1)) {
                    gid[i] = int(free_block.size());
                    free_block.push_back(int(i));
                }
        const int n_free = int(free_block.size());
        n_cam = 0;
        while (n_cam < n_free && !blocks[std::size_t(free_block[std::size_t(n_cam)])].eliminated) ++n_cam;
        const int n_elim = n_free - n_cam;
        offset.assign(std::size_t(n_free), 0);
        tangent.assign(std::size_t(n_free), 0);
        int o = 0;
        cam_dim = 0;
        for (int g = 0; g < n_free; ++g) {
            offset[std::size_t(g)] = o;
            tangent[std::size_t(g)] = blocks[std::size_t(free_block[std::size_t(g)])].tangent;
            o += tangent[std::size_t(g)];
            if (g + 1 == n_cam) cam_dim = o;
        }
        total_dim = o;

        // The Hessian's blocks
        std::vector<std::uint64_t> cc, ce;
        std::vector<std::vector<int>> cams_of(static_cast<std::size_t>(n_elim));
        rgid.assign(xs.size(), -1);
        rjoff.assign(xs.size(), 0);
        std::size_t jmax = 0;
        for (int r : active) {
            const Res& R = res[std::size_t(r)];
            std::size_t jo = 0;
            int n_elim_here = 0;
            for (std::size_t i = 0; i < R.blocks.size(); ++i) {
                const int g = gid[std::size_t(R.blocks[i])];
                rgid[R.x_off + i] = g;
                if (g < 0) continue;
                rjoff[R.x_off + i] = jo;
                jo += std::size_t(R.rows * tangent[std::size_t(g)]);
                if (g >= n_cam) ++n_elim_here;
            }
            jmax = std::max(jmax, jo);
            if (n_elim_here > 1) throw std::invalid_argument("nls: a residual touches two eliminated blocks");
            for (std::size_t i = 0; i < R.blocks.size(); ++i)
                for (std::size_t j = i + 1; j < R.blocks.size(); ++j) {
                    const int gi = rgid[R.x_off + i], gj = rgid[R.x_off + j];
                    if (gi < 0 || gj < 0) continue;
                    const int lo = std::min(gi, gj), hi = std::max(gi, gj);
                    if (hi < n_cam)
                        cc.push_back(key(lo, hi));
                    else {
                        ce.push_back(key(hi, lo));  // sorted by the eliminated block, then the camera
                        cams_of[std::size_t(hi - n_cam)].push_back(lo);
                    }
                }
        }
        jbuf.assign(jmax, 0);
        for (int g = 0; g < n_cam; ++g) cc.push_back(key(g, g));
        for (auto& cs : cams_of) {
            std::sort(cs.begin(), cs.end());
            cs.erase(std::unique(cs.begin(), cs.end()), cs.end());
            for (std::size_t i = 0; i < cs.size(); ++i)
                for (std::size_t j = i + 1; j < cs.size(); ++j) cc.push_back(key(cs[i], cs[j]));
        }
        auto uniq = [](std::vector<std::uint64_t>& v) {
            std::sort(v.begin(), v.end());
            v.erase(std::unique(v.begin(), v.end()), v.end());
        };
        uniq(cc);
        uniq(ce);
        std::unordered_map<std::uint64_t, std::size_t> h_of;  // key(lo, hi) -> offset in hbuf
        h_of.reserve(cc.size() + ce.size() + std::size_t(n_free));
        std::size_t h = 0;
        for (const std::uint64_t k : cc) {
            const int lo = int(k >> 32), hi = int(k & 0xffffffffu);
            h_of.emplace(k, h);
            h += std::size_t(tangent[std::size_t(lo)] * tangent[std::size_t(hi)]);
        }
        cc_size = h;
        for (const std::uint64_t k : ce) {
            const int e = int(k >> 32), c = int(k & 0xffffffffu);
            h_of.emplace(key(c, e), h);
            h += std::size_t(tangent[std::size_t(c)] * tangent[std::size_t(e)]);
        }
        for (int g = n_cam; g < n_free; ++g) {
            h_of.emplace(key(g, g), h);
            h += std::size_t(tangent[std::size_t(g)] * tangent[std::size_t(g)]);
        }
        hbuf.assign(h, 0);
        sbuf.assign(cc_size, 0);
        diag_off.assign(std::size_t(n_free), 0);
        for (int g = 0; g < n_free; ++g) diag_off[std::size_t(g)] = h_of.at(key(g, g));

        // The residuals' pairs
        pairs.clear();
        pair_begin.assign(active.size() + 1, 0);
        for (std::size_t ai = 0; ai < active.size(); ++ai) {
            pair_begin[ai] = pairs.size();
            const Res& R = res[std::size_t(active[ai])];
            for (std::size_t i = 0; i < R.blocks.size(); ++i)
                for (std::size_t j = i; j < R.blocks.size(); ++j) {
                    std::size_t a = R.x_off + i, c = R.x_off + j;
                    int ga = rgid[a], gc = rgid[c];
                    if (ga < 0 || gc < 0) continue;
                    if (ga > gc) {
                        std::swap(a, c);
                        std::swap(ga, gc);
                    }
                    pairs.push_back({rjoff[a], rjoff[c], tangent[std::size_t(ga)], tangent[std::size_t(gc)],
                                     h_of.at(key(ga, gc))});
                }
        }
        pair_begin[active.size()] = pairs.size();

        // The eliminated blocks' neighbourhoods
        nb_begin.assign(std::size_t(n_elim) + 1, 0);
        cp_begin.assign(std::size_t(n_elim) + 1, 0);
        nb_cam.clear();
        nb_h.clear();
        cp_s.clear();
        hinv_off.assign(std::size_t(n_elim), 0);
        std::size_t hi_size = 0, qmax = 0;
        for (int e = 0; e < n_elim; ++e) {
            const int g = n_cam + e;
            nb_begin[std::size_t(e)] = nb_cam.size();
            cp_begin[std::size_t(e)] = cp_s.size();
            const auto& cs = cams_of[std::size_t(e)];
            std::size_t q = 0;
            for (const int c : cs) {
                nb_cam.push_back(c);
                nb_h.push_back(h_of.at(key(c, g)));
                q += std::size_t(tangent[std::size_t(c)] * tangent[std::size_t(g)]);
            }
            qmax = std::max(qmax, q);
            for (std::size_t i = 0; i < cs.size(); ++i)
                for (std::size_t j = i; j < cs.size(); ++j) cp_s.push_back(h_of.at(key(cs[i], cs[j])));
            hinv_off[std::size_t(e)] = hi_size;
            hi_size += std::size_t(tangent[std::size_t(g)] * tangent[std::size_t(g)]);
        }
        nb_begin[std::size_t(n_elim)] = nb_cam.size();
        cp_begin[std::size_t(n_elim)] = cp_s.size();
        hinv.assign(hi_size, 0);
        qbuf.assign(qmax, 0);

        // The reduced system's pattern (upper triangle) and where each value comes from
        std::vector<Eigen::Triplet<double>> trip;
        fill_src.clear();
        for (const std::uint64_t k : cc) {
            const int lo = int(k >> 32), hi = int(k & 0xffffffffu);
            const std::size_t off = h_of.at(k);
            const int rows = tangent[std::size_t(lo)], cols = tangent[std::size_t(hi)];
            for (int j = 0; j < cols; ++j)
                for (int i = 0; i < rows; ++i) {
                    if (lo == hi && i > j) continue;
                    trip.emplace_back(offset[std::size_t(lo)] + i, offset[std::size_t(hi)] + j, 0.0);
                    fill_src.push_back(off + std::size_t(j * rows + i));
                }
        }
        A.resize(cam_dim, cam_dim);
        A.setFromTriplets(trip.begin(), trip.end());
        A.makeCompressed();
        fill_dst.assign(trip.size(), 0);
        for (std::size_t t = 0; t < trip.size(); ++t) {
            const int col = int(trip[t].col()), row = int(trip[t].row());
            const int* begin = A.innerIndexPtr() + A.outerIndexPtr()[col];
            const int* end = A.innerIndexPtr() + A.outerIndexPtr()[col + 1];
            fill_dst[t] = int(std::lower_bound(begin, end, row) - A.innerIndexPtr());
        }
        analyzed = false;

        b.setZero(total_dim);
        dx.setZero(total_dim);
        std::size_t rmax = 0;
        for (int r : active) rmax = std::max(rmax, std::size_t(res[std::size_t(r)].rows));
        we.assign(rmax, 0);
    }

    void computeError(const Res& R) { R.residual->evaluate(xs.data() + R.x_off, ebuf.data() + R.e_off, none.data()); }

    void computeActiveErrors() {
        for (int r : active) computeError(res[std::size_t(r)]);
        errors_current = true;
    }

    double activeRobustCost() const {
        double c = 0;
        for (int r : active) {
            const Res& R = res[std::size_t(r)];
            const double s = chi2(R);
            if (R.loss) {
                double rho[2];
                R.loss->evaluate(s, rho);
                c += rho[0];
            } else {
                c += s;
            }
        }
        return c;
    }

    // H and b at the current parameters (the errors already computed there). A Jacobian block is written
    // row-major (rows x tangent), which is Jᵀ column-major: H_ab += Jaᵀ W Jb is JaT · (JbT · W)ᵀ.
    void buildSystem() {
        std::fill(hbuf.begin(), hbuf.end(), 0.0);
        b.setZero();
        for (std::size_t ai = 0; ai < active.size(); ++ai) {
            if (pair_begin[ai] == pair_begin[ai + 1]) continue;  // nothing free
            const Res& R = res[std::size_t(active[ai])];
            const std::size_t k = R.blocks.size();
            for (std::size_t i = 0; i < k; ++i)
                jptr[i] = rgid[R.x_off + i] < 0 ? nullptr : jbuf.data() + rjoff[R.x_off + i];
            double* e = ebuf.data() + R.e_off;
            R.residual->evaluate(xs.data() + R.x_off, e, jptr.data());
            double weight = 1;
            if (R.loss) {
                double rho[2];
                R.loss->evaluate(chi2(R), rho);
                weight = rho[1];
            }
            const int rows = R.rows;
            const bool full = R.information.size() != 0;
            if (full) {
                Eigen::Map<Eigen::VectorXd>(we.data(), rows).noalias() =
                    weight * (R.information * Eigen::Map<const Eigen::VectorXd>(e, rows));
            } else {
                weight *= R.info_scale;
                for (int j = 0; j < rows; ++j) we[std::size_t(j)] = weight * e[j];
            }
            for (std::size_t i = 0; i < k; ++i) {
                const int g = rgid[R.x_off + i];
                if (g < 0) continue;
                gemv(b.data() + offset[std::size_t(g)], jptr[i], we.data(), tangent[std::size_t(g)], rows, -1);
            }
            for (std::size_t pi = pair_begin[ai]; pi < pair_begin[ai + 1]; ++pi) {
                const Pair& p = pairs[pi];
                const double* Ja = jbuf.data() + p.ja;
                const double* Jb = jbuf.data() + p.jb;
                if (full) {
                    scratch.resize(std::size_t(p.tb * rows));
                    gemmAB(scratch.data(), Jb, R.information.data(), p.tb, rows, rows);  // (Ω·Jb)ᵀ, Ω symmetric
                    gemmABt(hbuf.data() + p.h, Ja, scratch.data(), p.ta, p.tb, rows, weight);
                } else {
                    gemmABt(hbuf.data() + p.h, Ja, Jb, p.ta, p.tb, rows, weight);
                }
            }
        }
    }

    double maxDiagonal() const {
        double m = 0;
        for (std::size_t g = 0; g < free_block.size(); ++g) {
            const int t = tangent[g];
            for (int i = 0; i < t; ++i) m = std::max(m, std::abs(hbuf[diag_off[g] + std::size_t(i * t + i)]));
        }
        return m;
    }

    // (H + λI) dx = b by the Schur complement on the eliminated blocks
    bool solveLinear(double lambda) {
        const int n_free = int(free_block.size());
        for (int g = n_cam; g < n_free; ++g) {
            const int t = tangent[std::size_t(g)];
            double* Hi = hinv.data() + hinv_off[std::size_t(g - n_cam)];
            std::copy_n(hbuf.data() + diag_off[std::size_t(g)], t * t, Hi);
            for (int i = 0; i < t; ++i) Hi[i * t + i] += lambda;
            invert(Hi, t);
            for (int i = 0; i < t * t; ++i)
                if (!std::isfinite(Hi[i])) return false;
        }
        if (n_cam > 0) {
            std::copy_n(hbuf.data(), cc_size, sbuf.data());
            for (int g = 0; g < n_cam; ++g) {
                const int t = tangent[std::size_t(g)];
                for (int i = 0; i < t; ++i) sbuf[diag_off[std::size_t(g)] + std::size_t(i * t + i)] += lambda;
            }
            rhs = b.head(cam_dim);
            for (int g = n_cam; g < n_free; ++g) {
                const int e = g - n_cam, te = tangent[std::size_t(g)];
                const double* Hi = hinv.data() + hinv_off[std::size_t(e)];
                hib.setZero(te);
                gemv(hib.data(), Hi, b.data() + offset[std::size_t(g)], te, te, 1);
                const std::size_t n0 = nb_begin[std::size_t(e)], n1 = nb_begin[std::size_t(e) + 1];
                std::size_t q = 0;
                for (std::size_t i = n0; i < n1; ++i) {
                    const int c = nb_cam[i], tc = tangent[std::size_t(c)];
                    const double* Hce = hbuf.data() + nb_h[i];
                    gemmAB(qbuf.data() + q, Hce, Hi, tc, te, te);
                    gemv(rhs.data() + offset[std::size_t(c)], Hce, hib.data(), tc, te, -1);
                    q += std::size_t(tc * te);
                }
                std::size_t k = cp_begin[std::size_t(e)], qi = 0;
                for (std::size_t i = n0; i < n1; ++i) {
                    const int tci = tangent[std::size_t(nb_cam[i])];
                    for (std::size_t j = i; j < n1; ++j, ++k)
                        gemmABt(sbuf.data() + cp_s[k], qbuf.data() + qi, hbuf.data() + nb_h[j], tci,
                                tangent[std::size_t(nb_cam[j])], te, -1);
                    qi += std::size_t(tci * te);
                }
            }
            double* v = A.valuePtr();
            for (std::size_t t = 0; t < fill_src.size(); ++t) v[fill_dst[t]] = sbuf[fill_src[t]];
            if (!analyzed) {
                ldlt.analyzePattern(A);
                analyzed = true;
            }
            ldlt.factorize(A);
            if (ldlt.info() != Eigen::Success) return false;
            dx.head(cam_dim) = ldlt.solve(rhs);
            if (ldlt.info() != Eigen::Success || !dx.head(cam_dim).allFinite()) return false;
        }
        for (int g = n_cam; g < n_free; ++g) {
            const int e = g - n_cam, te = tangent[std::size_t(g)];
            hib = b.segment(offset[std::size_t(g)], te);
            for (std::size_t i = nb_begin[std::size_t(e)]; i < nb_begin[std::size_t(e) + 1]; ++i) {
                const int c = nb_cam[i];
                gemvT(hib.data(), hbuf.data() + nb_h[i], dx.data() + offset[std::size_t(c)], te, tangent[std::size_t(c)], -1);
            }
            double* out = dx.data() + offset[std::size_t(g)];
            std::fill_n(out, te, 0.0);
            gemv(out, hinv.data() + hinv_off[std::size_t(e)], hib.data(), te, te, 1);
        }
        return true;
    }

    void push() {
        backup.clear();
        for (int bi : free_block) {
            const Block& B = blocks[std::size_t(bi)];
            backup.insert(backup.end(), B.x, B.x + B.ambient);
        }
    }
    void pop() {
        std::size_t k = 0;
        for (int bi : free_block) {
            const Block& B = blocks[std::size_t(bi)];
            std::copy_n(backup.data() + k, B.ambient, B.x);
            k += std::size_t(B.ambient);
        }
        errors_current = false;
    }
    void update() {
        double tmp[64];
        std::vector<double> big;
        for (std::size_t g = 0; g < free_block.size(); ++g) {
            const Block& B = blocks[std::size_t(free_block[g])];
            double* x0 = tmp;
            if (B.ambient > 64) {
                big.assign(B.x, B.x + B.ambient);
                x0 = big.data();
            } else {
                std::copy_n(B.x, B.ambient, tmp);
            }
            B.manifold->plus(x0, dx.data() + offset[g], B.x);
        }
    }
};

// The LM loop of solve(), one iteration per step(): the state carried between them is the options, λ, ν, the
// small-step count and the summary (g2o keeps the same across its iterations).
struct Problem::Run {
    Options o;
    Summary s;
    double lambda = 0, ni = 2;
    int it = 0, n_small = 0;
    bool done = false;
};

Problem::Problem() : impl_(std::make_unique<Impl>()) {}
Problem::~Problem() = default;

int Problem::addBlock(double* x, int size) { return addBlock(x, std::make_shared<Euclidean>(size)); }

int Problem::addBlock(double* x, std::shared_ptr<const Manifold> manifold) {
    Block B;
    B.x = x;
    B.ambient = manifold->ambientSize();
    B.tangent = manifold->tangentSize();
    B.manifold = std::move(manifold);
    impl_->blocks.push_back(std::move(B));
    return int(impl_->blocks.size()) - 1;
}

void Problem::setFixed(int block, bool fixed) { impl_->blocks.at(std::size_t(block)).fixed = fixed; }
void Problem::setEliminated(int block, bool eliminated) { impl_->blocks.at(std::size_t(block)).eliminated = eliminated; }

int Problem::addResidual(std::shared_ptr<const Residual> residual, std::vector<int> blocks, Eigen::MatrixXd information,
                         std::shared_ptr<const Loss> loss) {
    Impl& m = *impl_;
    for (std::size_t i = 0; i < blocks.size(); ++i) {
        if (blocks[i] < 0 || blocks[i] >= int(m.blocks.size())) throw std::out_of_range("nls: no such block");
        for (std::size_t j = 0; j < i; ++j)
            if (blocks[j] == blocks[i]) throw std::invalid_argument("nls: a residual names a block twice");
    }
    Res R;
    R.rows = residual->size();
    // A multiple of the identity is kept as its scale (the usual 1/σ² of a pixel)
    if (information.size() &&
        information.isApprox(information(0, 0) * Eigen::MatrixXd::Identity(information.rows(), information.cols()), 0.0)) {
        R.info_scale = information(0, 0);
        information.resize(0, 0);
    }
    R.information = std::move(information);
    R.loss = std::move(loss);
    R.residual = residual.get();
    R.owner = std::move(residual);
    R.x_off = m.xs.size();
    for (int b : blocks) m.xs.push_back(m.blocks[std::size_t(b)].x);
    R.e_off = m.ebuf.size();
    m.ebuf.resize(m.ebuf.size() + std::size_t(R.rows), 0.0);
    R.blocks = std::move(blocks);
    if (m.none.size() < R.blocks.size()) m.none.assign(R.blocks.size(), nullptr);
    m.res.push_back(std::move(R));
    return int(m.res.size()) - 1;
}

void Problem::setActive(int residual, bool active) { impl_->res.at(std::size_t(residual)).active = active; }
void Problem::setLoss(int residual, std::shared_ptr<const Loss> loss) {
    impl_->res.at(std::size_t(residual)).loss = std::move(loss);
}
int Problem::numBlocks() const { return int(impl_->blocks.size()); }
int Problem::numResiduals() const { return int(impl_->res.size()); }
void Problem::computeError(int residual) { impl_->computeError(impl_->res.at(std::size_t(residual))); }
Eigen::Map<const Eigen::VectorXd> Problem::error(int residual) const {
    const Res& R = impl_->res.at(std::size_t(residual));
    return Eigen::Map<const Eigen::VectorXd>(impl_->ebuf.data() + R.e_off, R.rows);
}
double Problem::chi2(int residual) const { return impl_->chi2(impl_->res.at(std::size_t(residual))); }

void Problem::start(const Options& o) {
    Impl& m = *impl_;
    run_ = std::make_unique<Run>();
    run_->o = o;
    m.setup();
    m.computeActiveErrors();
    run_->s.initial_cost = run_->s.final_cost = m.activeRobustCost();
    run_->done = m.free_block.empty() || o.max_iterations <= 0;
}

bool Problem::finished() const { return !run_ || run_->done; }

Summary Problem::summary() const { return run_ ? run_->s : Summary{}; }

void Problem::step() {
    if (finished()) return;
    Impl& m = *impl_;
    Run& R = *run_;
    const Options& o = R.o;
    // The parameters are those of start() or of the last accepted trial, whose errors and cost are at hand; a
    // step that ended without accepting (a NaN ρ) left the rejected trial's errors in ebuf.
    double current;
    if (m.errors_current) {
        current = R.s.final_cost;
    } else {
        m.computeActiveErrors();
        current = m.activeRobustCost();
    }
    const double initial = current;
    m.buildSystem();
    if (R.it == 0) {
        R.lambda = o.initial_lambda > 0 ? o.initial_lambda : o.tau * m.maxDiagonal();
        R.ni = 2;
        R.n_small = 0;
    }
    double rho = 0;
    int tries = 0;
    do {
        m.push();
        const bool ok = m.solveLinear(R.lambda);
        double trial;
        if (ok) {
            m.update();
            m.computeActiveErrors();
            trial = m.activeRobustCost();
        } else {
            m.dx.setZero();
            R.s.linear_solver_failed = true;
            trial = std::numeric_limits<double>::max();
        }
        double scale = 0;
        for (int j = 0; j < m.total_dim; ++j) scale += m.dx[j] * (R.lambda * m.dx[j] + m.b[j]);
        scale += 1e-3;
        rho = (current - trial) / scale;
        if (rho > 0 && std::isfinite(trial)) {
            const double alpha = std::min(1. - std::pow(2 * rho - 1, 3), 2. / 3.);
            R.lambda *= std::max(1. / 3., alpha);
            R.ni = 2;
            current = trial;
        } else {
            R.lambda *= R.ni;
            R.ni *= 2;
            m.pop();
        }
        ++tries;
    } while (rho < 0 && tries < o.max_trials);
    ++R.s.iterations;
    ++R.it;
    R.s.final_cost = current;
    if (tries == o.max_trials || rho == 0 || R.it >= o.max_iterations) {
        R.done = true;
        return;
    }
    if (o.small_steps > 0) {
        if ((initial - current) < o.small_step_ratio * initial)
            ++R.n_small;
        else
            R.n_small = 0;
        if (R.n_small >= o.small_steps) R.done = true;
    }
}

Summary Problem::solve(const Options& o) {
    start(o);
    while (!finished()) step();
    return summary();
}

}  // namespace nls
