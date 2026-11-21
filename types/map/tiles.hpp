#pragma once
// The atlas's tile types from outside types/map (fw::is_tile_v, fw/tiled_map.hpp §4.4): a fw::TiledMap checks
// that its tiles have value semantics, and a struct is one when declared so in its own namespace. Declared here
// rather than next to each type, so geometry, imu and camera stay free of fw. The map types declare theirs in
// their own headers.
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "fw/tiled_map.hpp"
#include "types/camera/kannala_brandt.hpp"
#include "types/camera/stereo_rig.hpp"
#include "types/geometry/se3.hpp"
#include "types/imu/preintegrated.hpp"

// Eigen matrices and quaternions are values (a dynamic-size one copies its storage). FW_TILE_UNCHECKED takes one
// type, so its hook (found by ADL) is written out for the templates.
namespace Eigen {
template <class S, int R, int C, int O, int MR, int MC>
void fw_tile_type_declared(const Matrix<S, R, C, O, MR, MC>*);
template <class S, int O>
void fw_tile_type_declared(const Quaternion<S, O>*);
}  // namespace Eigen

namespace geometry {
// A quaternion and a translation (private: q_, t_)
template <class S>
void fw_tile_type_declared(const SE3<S>*);
}  // namespace geometry

namespace imu {
FW_TILE_TYPE(Bias, g, a);
}  // namespace imu

namespace camera {
// Both befriend fw_tile_type_declared, so their private members are checked here
FW_TILE_TYPE(KannalaBrandt8, fx_, fy_, cx_, cy_, k_, width_, height_, theta_max_, r_max_);
FW_TILE_TYPE(StereoRig, cam_, R_12_, t_12_);
}  // namespace camera
