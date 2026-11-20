# ORB-SLAM3 golden fixtures

Inputs and outputs of the original ORB-SLAM3 code (the KB8 stereo / stereo-inertial "vslam" refactor), for
checking the fw tracking and local mapping ports (`algorithms/orb_tracking/docs/DESIGN.md`,
`algorithms/orb_local_mapping/docs/DESIGN.md`) against it. The port is a rewrite;
these files are the only link to the original, which is no longer in the repository.

They were made by a one-off generator linked against the original. It built synthetic scenes and called the
original functions on them. Each case is self-contained: the inputs the function saw and what it returned.
Nothing here needs regenerating; a port that agrees with them within tolerance agrees with the original.

## Format

Plain text. `#` lines are comments that say what the following values are. Every other line is one value:

```
name rows cols v(0,0) v(0,1) ... v(rows-1,cols-1)
```

Values are row-major and printed at full precision (`%.17g`). A scalar is `1 1`, and a vector is `n 1`. Flags
and indices are written as numbers. Poses are 4x4 matrices.

Conventions:
- `Tcw` maps world to the left camera.
- `Rwb`, `twb` and `vwb` are the body (IMU) in the world, with z up and gravity (0, 0, -9.81).
- The bias is split into `bg` (gyro) and `ba` (accelerometer).

## rig.txt

Every case uses this rig:
- **Cameras:** the KB8 stereo of `tests/orb_matcher/synth_rigs.hpp` `eurocRig()` (752x480, 11 cm baseline).
  `left_kb8` and `right_kb8` are fx fy cx cy k1 k2 k3 k4.
- **Extrinsics:** `T_left_right` is the right camera in the left one. `T_body_left` is the left camera in the
  body; it is EuRoC's cam0 `T_BS`.
- **IMU noise:** `imu_noise_density` holds gyro noise, accel noise, gyro walk and accel walk, from EuRoC's
  continuous-time values. `imu_calib_discrete` holds the values ORB-SLAM3 actually uses: noise × √hz and
  walk / √hz, as `Tracking` builds `IMU::Calib`.
- **Features:** 8 pyramid levels with scale factor 1.2. The level σ² is the scale factor squared, and its
  information is 1/σ².
- **Depth threshold:** `th_depth_m` is the close/far depth (ThDepth 35 × baseline).

## Synthetic scenes

- **Trajectory:** analytic, a sinusoid on each axis of position and of roll, pitch and yaw. The body rate is
  taken numerically from that rotation.
- **IMU:** samples are the exact specific force and body rate, plus a constant bias and white noise at the rig's
  density, at 200 Hz.
- **Observations:** points are drawn in front of the left camera at the true pose and projected into the KB8
  views. Pixel noise is σ = scale factor of the octave × 1 px.
  - Some points are seen by the left camera only, some by the right only, and some by both.
  - A stereo point has one map point at its left index and its right index.
  - Outliers keep their keypoint but get a map point somewhere else; `outlier_truth` marks them.
  - Some keypoints have no map point.

Observation fields, shared by pose_optimization, pose_inertial and mlpnp:

| field | |
|---|---|
| `kp_left`, `kp_right` | x y octave per keypoint (pixels, distorted, as detected) |
| `n_left`, `n_right` | frame index i < n_left is `kp_left[i]`, else `kp_right[i - n_left]` (Frame's N = n_left + n_right) |
| `mp_of_index` | per frame index: the map point (row of `mp_world`) or -1 |
| `mp_world` | map point world positions |
| `outlier_truth` | 1 where the association was made wrong |
| `left_to_right` | stereo match per left keypoint (-1 none) |
| `Tcw_true` | the pose the observations were made from |

Where map point depth is used (pose_inertial: `mTrackDepth < 10` picks close points), it is z of the map point
at `Tcw_true`.

## preintegration/

Cases cover a 20 Hz frame interval with several sample alignments, zero bias, a large bias step (the
`Reintegrate` regime), 0.5 s and 1.5 s keyframe intervals, and 4 ms intervals: `short_4ms` has one interpolated step, and `short_4ms_empty` has no sample before
`t_cur` − 1 ms. `PreintegrateIMU` makes no preintegration for the empty one, so its file has inputs only.

**Inputs**
- `imu` holds the raw samples, t ax ay az wx wy wz per row. `t_prev`, `t_cur` and `t_next` are frame times.
- `meas_a` (prev → cur) and `meas_b` (cur → next) are what `Tracking::PreintegrateIMU` makes of `imu`: the
  `IntegrateNewMeasurement` arguments ax ay az wx wy wz dt.
  - Sample selection is relative to a 1 ms `mImuPer`.
  - The first and last steps use its interpolation formulas exactly as written. Timestamps are double,
    and each time difference is taken in double and then stored as float. A port that uses the same
    operations reproduces `meas_*` exactly.
- Biases: `b0` integrates, `b1` is the `SetNewBias` value, and `b2` is a query bias.

**Outputs**

Each preintegration prefix holds `dT dR dV dP JRg JVg JVa JPg JPa C avgA avgW bg ba`. `C` is 15x15 in the
order (rotation, velocity, position, gyro walk, acc walk).

| prefix | |
|---|---|
| `a_` | `meas_a` integrated at b0 |
| `a_q_dR/dV/dP` | `GetDeltaRotation/Velocity/Position(b2)` of `a_`: the first-order bias correction |
| `a_u_*` | after `SetNewBias(b1)`: `GetUpdatedDelta*`, and `a_u_db` = `GetDeltaBias()` (dbg, dba) |
| `z_`, `z_u_*` | `meas_a` integrated at zero bias, then `SetNewBias(b1)`: the fw design's upstream preintegration |
| `m_` | `meas_b` at b0, `SetNewBias(b1)`, `MergePrevious(a)`. This re-integrates a's then b's measurements at b1. |
| `r_` | `a_` after `SetNewBias(b1)` and `Reintegrate()`, which is the same as integrating at b1 |

## pose_optimization/

`Optimizer::PoseOptimization`: a single frame, with left and right observations as separate monocular edges.

**Inputs:** the observations, plus `Tcw_init` (the start pose).

**Outputs:**
- `out_Tcw`;
- `out_outlier` (`mvbOutlier` per frame index; meaningful where `mp_of_index >= 0`);
- `out_n_inliers`, the return value.

**Cases:**
- `nominal`;
- `dense`;
- `sparse`;
- `heavy_outliers` (35 %);
- `large_init_error` (6°, 30 cm);
- `few_edges` (8 edges, fewer than 10: the early break after the first round);
- `left_only`;
- `far` (5–30 m).

## pose_inertial/

Each case is a chain: keyframe → frame 1 → frame 2, with 20 Hz frames.
- `<case>_last_keyframe.txt` runs `PoseInertialOptimizationLastKeyFrame` on frame 1.
- `<case>_last_frame.txt` runs `PoseInertialOptimizationLastFrame` on frame 2, whose previous frame is frame 1
  after its optimization.
- Each optimization starts from `Tracking::PredictStateIMU`. Frame 1 is predicted from the keyframe, and frame 2
  from frame 1. That prediction is recorded as `init_*`, so it is tested too.

The keyframe has the true pose, so it agrees with the true map points. Its velocity is off by about 5 mm/s, and
its bias estimate is off from the bias in the IMU samples.

**last_keyframe inputs**
- `kf_Rwb kf_twb kf_vwb kf_bg kf_ba`: the keyframe, which is fixed.
- `imu` and `frame_times`, from the keyframe to frame 1. `meas_kf` is accumulated interval by interval, as
  tracking does.
- `pre_kf_*` is that preintegration at the keyframe's bias.
- The observations.
- `init_Rwb init_twb init_vwb init_bg init_ba init_Tcw`.

**last_frame inputs**
- `prev_*`: frame 1's state.
- `prior_Rwb prior_twb prior_vwb prior_bg prior_ba prior_H`: frame 1's `ConstraintPoseImu`.
- `meas_frame` and `pre_frame_*`: frame 1 → frame 2 at frame 1's bias. This is the inertial edge.
- `meas_kf` and `pre_kf_*`: keyframe → frame 2 at the keyframe's bias.
  - Only its `C` is used: `C(9:12, 9:12)` and `C(12:15, 12:15)`, the information of the bias random-walk edges.
  - The original takes those from the keyframe's preintegration even in this variant.
- The observations, and `init_*`.

**Outputs (both variants):**
- `out_Rwb out_twb out_vwb out_bg out_ba out_Tcw out_outlier out_n_inliers`;
- `out_prior_*`, the `ConstraintPoseImu` left on the frame.

`H` is 15x15. Its order is: rotation, translation, velocity, gyro bias, acc bias.
- Rotation and translation are increments in the body frame: `twb += Rwb·δt`, `Rwb = Rwb·Exp(δr)`. `Exp`
  is orthonormalized (SVD) on every call. The periodic `NormalizeRotation(Rwb)` in `ImuCamPose::Update`
  discards its result, so it does nothing.
- The constraint symmetrizes `H` and zeroes eigenvalues below 1e-12.

**Cases:**
- `nominal`;
- `bias_error` (the keyframe bias is off by about its own size);
- `few_points` (15);
- `outliers` (30 %);
- `fast` (twice the motion frequency, frame 1 three frames after the keyframe);
- `next_frame` (frame 1 right after the keyframe).

## mlpnp/

`MLPnPsolver`, as `Tracking::Relocalization` runs it. Only left keypoints take part.

**`ransac_*`**
- The settings are `SetRansacParameters(0.99, 10, 300, 6, 0.5, 5.991)`, then `iterate(5)` is called until it
  returns a pose or no iterations are left.
- The RANSAC samples come from the original's random generator, so a port draws different ones. Compare
  `ransac_Tcw` within tolerance and the inlier set by overlap, not exactly.
- Fields:
  - `ransac_ok`;
  - `ransac_Tcw`;
  - `ransac_inliers` (per frame index);
  - `ransac_n_inliers`;
  - `ransac_calls`;
  - `ransac_min_inliers`;
  - `ransac_max_error` (per correspondence: 5.991 × σ² of its octave).

**`computePose`, deterministic, without covariance (as `iterate` calls it)**
- `cp_index` lists the true-inlier left indices.
- `cp_bearings` are their unprojected rays, scaled to z = 1. The original does not normalize them.
- `cp6_Tcw` uses the first 6, and `cpall_Tcw` uses all. Both are 3x4 `[Rcw | tcw]`.

**Cases:**
- `nominal` (30 % outliers);
- `noise_free`;
- `heavy_outliers` (60 %);
- `few`;
- `too_few` (below the 10-inlier minimum: no pose). It has no `cp*` values, because its 6-point minimal set is
  degenerate.

`heavy_outliers` fails in the original too. The minimum inlier count is ε·N with ε = 0.5, and only 40 % of its
correspondences are inliers. One `iterate(5)` call already runs all of `ransac_max_iterations`, because its loop
condition is `its < max || current < 5`.

## bow/

`ORBVocabulary::transform(descriptors, bow, feat, 4)`, as `ComputeBoW` calls it, and `score(a, b)`, as the
keyframe database uses it. The vocabulary is ORB-SLAM3's `ORBvoc.txt`; `vocabulary.txt` records its
k, L, word count, weighting (0 = TF-IDF) and scoring (0 = L1).

**Inputs:** `desc_a`, `desc_b` and `desc_c` are N x 32 bytes. They are vocabulary words with random bit flips.
- b shares part of a's words.
- c is unrelated.

**Outputs:**
- `x_bow`: word id and weight, ascending by id.
- `x_feat`: node id (4 levels up from the leaves) and descriptor index, ascending by node, with indices in the
  order they were added.
- `score_ab score_ac score_bc score_aa`.

**Cases:**
- `small`;
- `frame` (1000 descriptors);
- `noisy` (up to 40 flipped bits);
- `exact_words`;
- `duplicate_words` (pairs of features on one word).

## new_map_points/

`LocalMapping::CreateNewMapPoints` on a small atlas, and `ORBmatcher::SearchForTriangulation` between the
current keyframe and each other keyframe before it. Made on 2026-11-20 by calling the original's own
`CreateNewMapPoints` (from a `LocalMapping` subclass) after building the atlas from its own `KeyFrame` and
`MapPoint` objects.

**Scene:** six keyframes along the analytic trajectory at t = 30, 30.25, 30.5, 30.75, 30.98 and 31 s. The last
one is current, and each keyframe's previous one (`mPrevKF`) is the one before it.
- **Tracked points** already have a map point. Each is seen by the current keyframe and one other: 6 by
  keyframe 0, then 18, 26, 34 and 42 by keyframes 1 to 4. So the covisibility order is fixed (4, 3, 2, 1).
  Keyframe 0 is below the 15 of `UpdateConnections`, so only the inertial prev chain reaches it. Keyframe 4 is
  20 ms before the current one, so its baseline is below `mb` and it is skipped.
- **Untracked points** are seen by every keyframe that sees them, in the left view, the right view or both,
  at an octave that fits the distance. Their descriptors are one vocabulary word per point with 0–8 bits
  flipped per view.
- **Distractors:** each keyframe has 40 more untracked features with random words.
- Keypoint angles are -1. The matcher is made with `checkOri = false`, so they are not used.

**Inputs**
- Flags: `inertial`, `recently_lost` (tracking's `mState`), `imu_ba2` (the map's), `far_points` and
  `th_far_points` (`mbFarPoints`, `mThFarPoints`).
- `n_keyframes`.
- `tracked_world`: the existing map points.
- Per keyframe k, under `kfk_`:
  - `Tcw` and `timestamp`;
  - `kp_left` and `kp_right` (x y octave);
  - `desc` (N x 32, left then right, in frame index order);
  - `feat` (node and frame index, `mFeatVec` at 4 levels up, from ORBvoc);
  - `points` (per frame index: the row of `tracked_world`, or -1).

**Outputs**
- `covisibles`: `GetBestCovisibilityKeyFrames(10)` of the current keyframe, as keyframe indices.
- `search_kfk` and `search_kfk_coarse`: the pairs from `SearchForTriangulation(current, kf k, pairs, bCoarse)`
  with bCoarse false and true. Each row is a current index and an other index, in current-index order.
- `created`: the new points in the order of `mlpRecentAddedMapPoints`. Each row is the other keyframe, the
  current index, the other index, then x y z in the world frame.

**Cases:**
- `stereo`;
- `inertial` (the 0.9996 parallax threshold, and the prev chain adds keyframe 0);
- `inertial_coarse` (RECENTLY_LOST with BA2 done, so bCoarse: no epipolar check);
- `far_points` (`mThFarPoints` = 4 m).

## fuse/

`LocalMapping::SearchInNeighbors` (`ORBmatcher::Fuse` both ways) on a small atlas with duplicated points. Made
on 2026-11-20 by calling the original's own `SearchInNeighbors` (from a `LocalMapping` subclass) after building
the atlas from its own `KeyFrame` and `MapPoint` objects.

**Scene:** fourteen keyframes along the analytic trajectory, 0.1 s apart from t = 30 s. The last one is
current, and each keyframe's previous one (`mPrevKF`) is the one before it.
- **Scene points** (450) are in front of the current keyframe and seen by every keyframe that sees them, in
  the left view, the right view or both, at an octave that fits the distance. Their descriptors are one
  vocabulary word per point with 0–8 bits flipped per view.
- **Map points:** 15 % of the scene points have none. For the others, each keyframe 1–13 that sees the point
  is left out (15 %) or put in one of up to three groups at random. Each group becomes its own map point, so
  one scene point is often two or three map points: these duplicates are what `Fuse` merges. Each map point
  is 1 cm (σ) off its scene point, and 5 % are 0.3 m off. Each has a random reference keyframe among its
  observers and random `mnVisible` / `mnFound`. Its descriptor, normal and scale range come from the
  original's `ComputeDistinctiveDescriptors` and `UpdateNormalAndDepth`.
- **Keyframe 0** has features but no map point, so it has no covisibility. Only the inertial prev chain
  reaches it.
- **Distractors:** each keyframe has 30 more features with random words.

**Inputs**
- `inertial` and `n_keyframes`.
- Per keyframe k, under `kfk_`: `Tcw`, `timestamp`, `kp_left` and `kp_right` (x y octave), and `desc`
  (N x 32, left then right, in frame index order).
- `point_ref`: each map point's reference keyframe.
- `before_kfk_points`: per frame index, the map point (a row of `before_points`) or -1.
- `before_points`: per map point: bad, replaced by (-1), `nObs`, visible, found, x y z, normal x y z, then
  `mfMinDistance` and `mfMaxDistance`. `before_point_desc`: the descriptors (32 bytes).
- `before_covisibles`: the current keyframe's covisibles by weight. Each row is a keyframe and its weight.

**Outputs**
- `targets`: the keyframes marked as fuse targets (`mnFuseTargetForKF`), ascending. The order is not
  observable from outside.
- `after_kfk_points`, `after_points`, `after_point_desc` and `after_covisibles`: the same as the inputs, after
  `SearchInNeighbors`. A bad point's normal and descriptor are as `Replace` left them.

**Cases:**
- `stereo`: ten best covisibles plus two second neighbours;
- `inertial`: the prev chain also adds keyframe 0.

**Note:** the original calls `Fuse(pKF, points, true)` for the right camera. That binds `true` to `th`, not to
`bRight`, so the second call is a left-camera fuse with th = 1. These outputs include that.

## Not here

No full-sequence trajectory from the original: the port's trajectory is checked against the recordings' ground
truth (ATE) instead.
