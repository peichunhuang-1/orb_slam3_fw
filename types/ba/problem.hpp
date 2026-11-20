#pragma once
// Bundle adjustment problems and their results (message types "ba/Problem", "ba/Result"): local mapping copies
// a problem out of the atlas under the lock, ba_solver solves it without touching any resource, local mapping
// writes the result back by id (algorithms/orb_local_mapping/docs/DESIGN.md, "鎖外計算"). A problem holds no
// pointer into the atlas: the preintegrations and cameras are copies.
//
// The fields are what the four ORB-SLAM3 optimizations need; `local` is solved, the inertial kinds not yet.
#include <cstdint>
#include <memory>
#include <vector>

#include <Eigen/Core>

#include "fw/fw.hpp"
#include "types/camera/stereo_rig.hpp"
#include "types/geometry/se3.hpp"
#include "types/imu/preintegrated.hpp"
#include "types/map/ids.hpp"

namespace ba {

enum class Kind {
    local,           // Optimizer::LocalBundleAdjustment
    local_inertial,  // Optimizer::LocalInertialBA
    inertial_init,   // Optimizer::InertialOptimization (Rwg, scale, biases, velocities; poses fixed)
    full_inertial,   // Optimizer::FullInertialBA
};

struct KeyFrameState {
    map::KeyFrameId id = map::kNone;
    double timestamp = 0;
    int rig = -1;  // index into Problem::rigs
    bool fixed = false;
    geometry::SE3f Tcw;
    geometry::SE3f Trl;  // right camera from left (KeyFrame::GetRelativePoseTrl)
    // Inertial kinds
    bool inertial = false;
    geometry::SE3f T_cb;
    Eigen::Vector3f velocity = Eigen::Vector3f::Zero();
    imu::Bias bias;
    int prev = -1;  // index of the previous keyframe in keyframes, -1: none in the problem
    std::shared_ptr<const imu::Preintegrated> preintegrated;  // a copy, from prev to this one
};

struct PointState {
    map::MapPointId id = map::kNone;
    Eigen::Vector3f pos = Eigen::Vector3f::Zero();
    bool fixed = false;
};

// One feature of a keyframe that sees a point
struct Observation {
    int keyframe = -1, point = -1;  // indices into keyframes, points
    bool right = false;             // the right camera of the rig
    int feature = -1;               // the keyframe's feature index (for erasing the match)
    Eigen::Vector2f uv = Eigen::Vector2f::Zero();
    float inv_sigma2 = 1;           // of the feature's pyramid level
};

struct Problem {
    std::uint64_t seq = 0;
    Kind kind = Kind::local;
    map::MapId map = map::kNone;
    map::KeyFrameId main_keyframe = map::kNone;  // pKF / the current keyframe
    std::vector<camera::StereoRig> rigs;
    std::vector<KeyFrameState> keyframes;
    std::vector<PointState> points;
    std::vector<Observation> observations;
    // Options of the kind
    int iterations = 10;
    double initial_lambda = 0;  // > 0: g2o's setUserLambdaInit; 0: 1e-5 · the largest diagonal
    bool large = false;     // LocalInertialBA bLarge
    bool rec_init = false;  // LocalInertialBA bRecInit
    float prior_g = 0, prior_a = 0;
    // inertial_init: the initial values
    Eigen::Matrix3d Rwg = Eigen::Matrix3d::Identity();
    double scale = 1;
    Eigen::Vector3d bg = Eigen::Vector3d::Zero(), ba = Eigen::Vector3d::Zero();
};

struct Result {
    std::uint64_t seq = 0;
    Kind kind = Kind::local;
    bool ok = false;
    std::vector<KeyFrameState> keyframes;  // id, Tcw, velocity, bias (the preintegration is not sent back)
    std::vector<PointState> points;        // id, pos
    struct Erase {
        map::KeyFrameId keyframe = map::kNone;
        map::MapPointId point = map::kNone;
    };
    std::vector<Erase> outliers;  // observations to erase (EraseMapPointMatch, EraseObservation)
    // inertial_init
    Eigen::Matrix3d Rwg = Eigen::Matrix3d::Identity();
    double scale = 1;
    Eigen::Vector3d bg = Eigen::Vector3d::Zero(), ba = Eigen::Vector3d::Zero();
    int iterations = 0;
    double solve_ms = 0;
};

}  // namespace ba

FW_REGISTER_TYPE(ba::Problem, "ba/Problem")
FW_REGISTER_TYPE(ba::Result, "ba/Result")
