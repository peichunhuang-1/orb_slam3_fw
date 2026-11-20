#pragma once
// nls: sparse non-linear least squares, Levenberg-Marquardt with a Schur complement. Pure math: Eigen only, no fw,
// shared by the visual BA (algorithms/ba_solver) and later ones (a lidar BA). The caller owns the residuals
// and their analytic Jacobians; the library owns the parameter blocks' bookkeeping and the solve.
//
//   minimise  Σ_r ρ_r( e_rᵀ Ω_r e_r )   over the free parameter blocks
//
// - A parameter block is caller memory (updated in place) with a manifold: x ⊞ δ, δ in the tangent space.
//   Jacobians are taken with respect to δ. Blocks can be fixed, and marked as eliminated: those are taken out
//   by the Schur complement first (the points of a BA). A residual may touch at most one eliminated block.
// - The step is g2o's OptimizationAlgorithmLevenberg (H + λI, the gain ratio ρ and Nielsen's λ update, up to
//   max_trials tries per iteration), with an optional stop rule: stop after `small_steps` iterations in a row
//   each lowering the cost by less than small_step_ratio of it (ORB-SLAM3's addition to g2o).
// - The robust weight is g2o's: H += Jᵀ ρ'Ω J, b -= Jᵀ ρ'Ω e (ρ'' is not used).
// - After solve(), error(r) / chi2(r) are those last computed: at the last trial step, even when that step
//   was rejected and the parameters went back (g2o leaves the edges' errors so; ORB-SLAM3 classifies outliers
//   with them).
#include <cstdint>
#include <memory>
#include <vector>

#include <Eigen/Core>

namespace nls {

class Manifold {
public:
    virtual ~Manifold() = default;
    virtual int ambientSize() const = 0;  // doubles stored
    virtual int tangentSize() const = 0;  // dimension of δ
    virtual void plus(const double* x, const double* delta, double* x_plus) const = 0;  // may alias x
};

class Euclidean final : public Manifold {
public:
    explicit Euclidean(int n) : n_(n) {}
    int ambientSize() const override { return n_; }
    int tangentSize() const override { return n_; }
    void plus(const double* x, const double* delta, double* x_plus) const override;

private:
    int n_;
};

// SE(3) stored as a unit quaternion (x, y, z, w) then a translation (7 doubles), T = (R, t) acting as R·p + t.
// x ⊞ δ = exp(δ)·T with δ = (ω, υ): rotation first (g2o's VertexSE3Expmap).
class SE3Left final : public Manifold {
public:
    int ambientSize() const override { return 7; }
    int tangentSize() const override { return 6; }
    void plus(const double* x, const double* delta, double* x_plus) const override;
};

class Loss {
public:
    virtual ~Loss() = default;
    // s = eᵀΩe: rho[0] = ρ(s), rho[1] = ρ'(s)
    virtual void evaluate(double s, double rho[2]) const = 0;
};

// ρ(s) = s for s <= δ², 2δ√s - δ² beyond. The square can be given (g2o keeps it in a float).
class Huber final : public Loss {
public:
    explicit Huber(double delta) : delta_(delta), delta_sq_(delta * delta) {}
    Huber(double delta, double delta_sq) : delta_(delta), delta_sq_(delta_sq) {}
    void evaluate(double s, double rho[2]) const override;

private:
    double delta_, delta_sq_;
};

class Residual {
public:
    virtual ~Residual() = default;
    virtual int size() const = 0;
    // e (size()) at the blocks' values, and dr/dδ_i (size() x tangent_i, row-major) for each i whose
    // jacobians[i] is not null (null for fixed blocks)
    virtual void evaluate(const double* const* blocks, double* e, double* const* jacobians) const = 0;
};

struct Options {
    int max_iterations = 10;
    double initial_lambda = 0;  // > 0: used as is (g2o's userLambdaInit); else tau · the largest diagonal of H
    double tau = 1e-5;
    int max_trials = 10;        // tries per iteration before giving up (g2o's maxTrialsAfterFailure)
    int small_steps = 3;        // 0: no small-step stop rule
    double small_step_ratio = 1e-3;
};

struct Summary {
    int iterations = 0;          // iterations run (the last one may have failed)
    double initial_cost = 0;     // Σ ρ at the start
    double final_cost = 0;       // Σ ρ at the parameters returned
    bool linear_solver_failed = false;  // some try could not factorize (treated as a rejected step)
};

class Problem {
public:
    Problem();
    ~Problem();
    Problem(const Problem&) = delete;
    Problem& operator=(const Problem&) = delete;

    int addBlock(double* x, int size);  // Euclidean
    int addBlock(double* x, std::shared_ptr<const Manifold> manifold);
    void setFixed(int block, bool fixed = true);
    void setEliminated(int block, bool eliminated = true);

    // information: size() x size() (symmetric), empty for identity; loss: null for plain least squares. The
    // blocks must differ.
    int addResidual(std::shared_ptr<const Residual> residual, std::vector<int> blocks,
                    Eigen::MatrixXd information = Eigen::MatrixXd(), std::shared_ptr<const Loss> loss = nullptr);
    void setActive(int residual, bool active);  // inactive ones are left out of the solve (g2o's level 1)
    void setLoss(int residual, std::shared_ptr<const Loss> loss);

    int numBlocks() const;
    int numResiduals() const;

    void computeError(int residual);              // at the current parameters
    Eigen::Map<const Eigen::VectorXd> error(int residual) const;
    double chi2(int residual) const;              // eᵀΩe of error(residual)

    Summary solve(const Options& options);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace nls
