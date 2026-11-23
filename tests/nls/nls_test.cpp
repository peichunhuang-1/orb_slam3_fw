// libs/nls on synthetic problems:
//   - SE3Left: exp(-δ)·(exp(δ)·T) = T, both branches of the exponential; a small δ moves T by δ to first order
//   - Euclidean curve fit y = a·exp(b·x) from a poor start
//   - pinhole BA (6 cameras, the first and the last fixed against the scale gauge, 120 points): the Schur solve and the plain one take the same
//     steps; noiseless data converge to the truth; fixed blocks do not move
//   - Huber: a gross outlier pulls a robust line fit less than a plain one; the cost is Σρ
#include <cmath>
#include <cstdio>
#include <memory>
#include <random>
#include <vector>

#include <Eigen/Geometry>

#include "libs/nls/problem.hpp"

namespace {

int failures = 0;
void check(bool ok, const char* what, double value = 0) {
    std::printf("%s %s (%.3g)\n", ok ? "ok  " : "FAIL", what, value);
    if (!ok) ++failures;
}

using Pose = std::array<double, 7>;

Eigen::Quaterniond quat(const Pose& x) { return Eigen::Quaterniond(x[3], x[0], x[1], x[2]); }
Eigen::Vector3d trans(const Pose& x) { return Eigen::Vector3d(x[4], x[5], x[6]); }
Pose pose(const Eigen::Quaterniond& q, const Eigen::Vector3d& t) { return {q.x(), q.y(), q.z(), q.w(), t.x(), t.y(), t.z()}; }

void testSE3() {
    nls::SE3Left m;
    const Pose T = pose(Eigen::Quaterniond(Eigen::AngleAxisd(0.7, Eigen::Vector3d(1, 2, 3).normalized())),
                        Eigen::Vector3d(0.3, -1, 2));
    double worst = 0;
    for (double s : {1e-10, 1e-6, 0.3, 2.0}) {
        double d[6] = {s, -2 * s, 0.5 * s, 3 * s, s, -s}, nd[6];
        for (int i = 0; i < 6; ++i) nd[i] = -d[i];
        Pose a, b;
        m.plus(T.data(), d, a.data());
        m.plus(a.data(), nd, b.data());
        worst = std::max(worst, (quat(b).coeffs() - quat(T).coeffs()).norm() + (trans(b) - trans(T)).norm());
    }
    check(worst < 1e-12, "SE3Left exp(-d) exp(d) T = T", worst);
    // first order: exp(δ)·T·p ≈ T·p + ω × (T·p) + υ
    const double e = 1e-7;
    double d[6] = {e, 2 * e, -e, -e, 3 * e, e};
    Pose a;
    m.plus(T.data(), d, a.data());
    const Eigen::Vector3d p(0.5, 1, -2), Tp = quat(T) * p + trans(T);
    const Eigen::Vector3d pred = Tp + Eigen::Vector3d(d[0], d[1], d[2]).cross(Tp) + Eigen::Vector3d(d[3], d[4], d[5]);
    const double err = (quat(a) * p + trans(a) - pred).norm();
    check(err < 1e-12, "SE3Left first order", err);
}

struct ExpModel final : nls::Residual {
    double x, y;
    ExpModel(double x_, double y_) : x(x_), y(y_) {}
    int size() const override { return 1; }
    void evaluate(const double* const* p, double* e, double* const* J) const override {
        const double a = p[0][0], b = p[0][1], v = std::exp(b * x);
        e[0] = a * v - y;
        if (J[0]) {
            J[0][0] = v;
            J[0][1] = a * x * v;
        }
    }
};

void testCurve() {
    double ab[2] = {1, 0};
    nls::Problem prob;
    const int blk = prob.addBlock(ab, 2);
    for (int i = 0; i < 30; ++i) {
        const double x = 0.1 * i;
        prob.addResidual(std::make_shared<ExpModel>(x, 2.5 * std::exp(-0.8 * x)), {blk});
    }
    nls::Options o;
    o.max_iterations = 100;
    o.small_steps = 0;
    const nls::Summary s = prob.solve(o);
    const double err = std::abs(ab[0] - 2.5) + std::abs(ab[1] + 0.8);
    check(err < 1e-6, "curve fit a·exp(b·x)", err);
    check(s.final_cost < 1e-12 && s.initial_cost > 1, "curve fit cost", s.final_cost);
}

// Pinhole projection, e = uv - π(T·X): the same Jacobian structure as the KB8 one
struct Pinhole final : nls::Residual {
    Eigen::Vector2d uv;
    double f = 400, c = 320;
    explicit Pinhole(const Eigen::Vector2d& u) : uv(u) {}
    int size() const override { return 2; }
    void evaluate(const double* const* x, double* e, double* const* J) const override {
        const Eigen::Quaterniond q(x[0][3], x[0][0], x[0][1], x[0][2]);
        const Eigen::Vector3d t(x[0][4], x[0][5], x[0][6]), Xw(x[1][0], x[1][1], x[1][2]);
        const Eigen::Vector3d X = q * Xw + t;
        e[0] = uv.x() - (f * X.x() / X.z() + c);
        e[1] = uv.y() - (f * X.y() / X.z() + c);
        Eigen::Matrix<double, 2, 3> P;
        P << f / X.z(), 0, -f * X.x() / (X.z() * X.z()), 0, f / X.z(), -f * X.y() / (X.z() * X.z());
        if (J[0]) {
            Eigen::Matrix<double, 3, 6> d;
            d << 0, X.z(), -X.y(), 1, 0, 0, -X.z(), 0, X.x(), 0, 1, 0, X.y(), -X.x(), 0, 0, 0, 1;
            Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor>> Jp(J[0]);
            Jp = -P * d;
        }
        if (J[1]) {
            Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> Jx(J[1]);
            Jx = -P * q.toRotationMatrix();
        }
    }
};

struct Scene {
    std::vector<Pose> cams;
    std::vector<Eigen::Vector3d> pts;
    std::vector<std::tuple<int, int, Eigen::Vector2d>> obs;
};

Scene makeScene(std::mt19937& g, double px_noise) {
    std::normal_distribution<double> n(0, 1);
    std::uniform_real_distribution<double> u(-1, 1);
    Scene s;
    for (int c = 0; c < 6; ++c) {
        const Eigen::Quaterniond q(Eigen::AngleAxisd(0.05 * c, Eigen::Vector3d::UnitY()));
        s.cams.push_back(pose(q, Eigen::Vector3d(-0.3 * c, 0.02 * c, 0)));
    }
    for (int i = 0; i < 120; ++i) s.pts.emplace_back(2 * u(g), 1.5 * u(g), 5 + 2 * u(g));
    Pinhole proj(Eigen::Vector2d::Zero());
    for (int c = 0; c < 6; ++c)
        for (int i = 0; i < 120; ++i) {
            const double* x[2] = {s.cams[std::size_t(c)].data(), s.pts[std::size_t(i)].data()};
            double e[2];
            double* none[2] = {nullptr, nullptr};
            proj.evaluate(x, e, none);
            s.obs.emplace_back(c, i, Eigen::Vector2d(-e[0], -e[1]) + px_noise * Eigen::Vector2d(n(g), n(g)));
        }
    return s;
}

void perturb(Scene& s, std::mt19937& g) {
    std::normal_distribution<double> n(0, 1);
    nls::SE3Left m;
    for (std::size_t c = 1; c + 1 < s.cams.size(); ++c) {
        double d[6] = {0.01 * n(g), 0.01 * n(g), 0.01 * n(g), 0.05 * n(g), 0.05 * n(g), 0.05 * n(g)};
        m.plus(s.cams[c].data(), d, s.cams[c].data());
    }
    for (auto& p : s.pts) p += 0.1 * Eigen::Vector3d(n(g), n(g), n(g));
}

nls::Summary solveBA(Scene& s, bool schur, int iterations, std::shared_ptr<const nls::Loss> loss = nullptr,
                     bool stepped = false) {
    nls::Problem prob;
    const auto se3 = std::make_shared<nls::SE3Left>();
    std::vector<int> cb, pb;
    for (auto& c : s.cams) cb.push_back(prob.addBlock(c.data(), se3));
    for (auto& p : s.pts) pb.push_back(prob.addBlock(p.data(), 3));
    prob.setFixed(cb[0]);
    prob.setFixed(cb.back());
    for (int b : pb) prob.setEliminated(b, schur);
    for (const auto& [c, i, uv] : s.obs)
        prob.addResidual(std::make_shared<Pinhole>(uv), {cb[std::size_t(c)], pb[std::size_t(i)]},
                         Eigen::Matrix2d::Identity() * 0.5, loss);
    nls::Options o;
    o.max_iterations = iterations;
    if (!stepped) return prob.solve(o);
    // As the global BA: one iteration per call, the errors read in between
    prob.start(o);
    while (!prob.finished()) {
        prob.step();
        for (int r = 0; r < prob.numResiduals(); r += 50) (void)prob.chi2(r);
    }
    return prob.summary();
}

void testBA() {
    std::mt19937 g(7);
    Scene truth = makeScene(g, 0.0);
    Scene a = truth;
    perturb(a, g);
    Scene b = a;
    const Pose fixed0 = a.cams[0];
    const nls::Summary sa = solveBA(a, true, 3), sb = solveBA(b, false, 3);
    double diff = 0;
    for (std::size_t c = 0; c < a.cams.size(); ++c)
        for (int k = 0; k < 7; ++k) diff = std::max(diff, std::abs(a.cams[c][std::size_t(k)] - b.cams[c][std::size_t(k)]));
    for (std::size_t i = 0; i < a.pts.size(); ++i) diff = std::max(diff, (a.pts[i] - b.pts[i]).cwiseAbs().maxCoeff());
    check(diff < 1e-9, "BA: Schur and plain solve take the same steps", diff);
    check(std::abs(sa.final_cost - sb.final_cost) < 1e-9 * (1 + sa.final_cost), "BA: same cost", sa.final_cost);

    Scene c = truth;
    perturb(c, g);
    const nls::Summary sc = solveBA(c, true, 50);
    double pose_err = 0;
    for (std::size_t k = 0; k < c.cams.size(); ++k)
        pose_err = std::max(pose_err, (trans(c.cams[k]) - trans(truth.cams[k])).norm() +
                                          quat(c.cams[k]).angularDistance(quat(truth.cams[k])));
    check(pose_err < 1e-6, "BA: noiseless data converge to the truth", pose_err);
    check(sc.final_cost < 1e-10 && sc.initial_cost > 1e3, "BA: final cost", sc.final_cost);
    double moved = 0;
    for (int k = 0; k < 7; ++k) moved = std::max(moved, std::abs(a.cams[0][std::size_t(k)] - fixed0[std::size_t(k)]));
    check(moved == 0, "BA: the fixed camera does not move", moved);
}

// solve() in steps (start, step, finished) takes the same iterations and ends where solve() does, with the
// stop rules (max iterations, the small steps) and a robust loss
void testStepped() {
    std::mt19937 g(11);
    for (const int its : {4, 100}) {
        Scene a = makeScene(g, 0.5);
        perturb(a, g);
        Scene b = a;
        const auto huber = std::make_shared<nls::Huber>(std::sqrt(5.991));
        const nls::Summary sa = solveBA(a, true, its, huber), sb = solveBA(b, true, its, huber, true);
        double diff = 0;
        for (std::size_t c = 0; c < a.cams.size(); ++c)
            for (int k = 0; k < 7; ++k)
                diff = std::max(diff, std::abs(a.cams[c][std::size_t(k)] - b.cams[c][std::size_t(k)]));
        for (std::size_t i = 0; i < a.pts.size(); ++i) diff = std::max(diff, (a.pts[i] - b.pts[i]).cwiseAbs().maxCoeff());
        check(diff == 0, "stepped: the same parameters as solve()", diff);
        check(sa.iterations == sb.iterations && sa.final_cost == sb.final_cost && sa.initial_cost == sb.initial_cost,
              "stepped: the same summary", sb.iterations);
        if (its == 100) check(sa.iterations < its, "stepped: the small-step rule stopped it", sa.iterations);
    }
    nls::Problem empty;
    nls::Options o;
    empty.start(o);
    check(empty.finished(), "stepped: nothing to optimize is finished at the start");
}

struct Line final : nls::Residual {
    double x, y;
    Line(double x_, double y_) : x(x_), y(y_) {}
    int size() const override { return 1; }
    void evaluate(const double* const* p, double* e, double* const* J) const override {
        e[0] = p[0][0] * x + p[0][1] - y;
        if (J[0]) {
            J[0][0] = x;
            J[0][1] = 1;
        }
    }
};

void testHuber() {
    auto fit = [](std::shared_ptr<const nls::Loss> loss, double& cost) {
        double ab[2] = {0, 0};
        nls::Problem prob;
        const int blk = prob.addBlock(ab, 2);
        for (int i = 0; i < 20; ++i) {
            const double x = i, y = 2 * x + 1 + (i == 10 ? 100 : 0);
            prob.addResidual(std::make_shared<Line>(x, y), {blk}, Eigen::MatrixXd(), loss);
        }
        nls::Options o;
        o.max_iterations = 200;
        o.small_steps = 0;
        cost = prob.solve(o).final_cost;
        return std::abs(ab[0] - 2) + std::abs(ab[1] - 1);
    };
    double c_plain, c_huber;
    const double plain = fit(nullptr, c_plain), robust = fit(std::make_shared<nls::Huber>(1.0), c_huber);
    check(robust < 0.2 * plain, "Huber: the outlier pulls the fit less", robust / plain);
    // at the robust optimum the outlier costs 2·δ·|e| - δ² (e ≈ 100), the others ~0
    check(std::abs(c_huber - (2 * 100 - 1)) < 5, "Huber: the cost is the sum of rho", c_huber);
}

}  // namespace

int main() {
    testSE3();
    testCurve();
    testBA();
    testHuber();
    testStepped();
    std::printf("%s\n", failures ? "FAILED" : "all passed");
    return failures ? 1 : 0;
}
