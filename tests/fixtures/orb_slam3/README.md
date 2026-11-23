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

## local_ba/

`Optimizer::LocalBundleAdjustment` (local mapping's visual local BA). Made on 2026-11-20 by calling the
original's own function on an atlas built from its own `KeyFrame` and `MapPoint` objects. The stop flag is
false.

**Scene:** K keyframes along the analytic trajectory, 0.1 s apart from t = 30 s. The last one is current
(`pKF`), and each keyframe's previous one is the one before it. The map's init keyframe is keyframe 0.
- **Scene points** each live over a span of 3–8 consecutive keyframes. They are generated in front of the
  span's middle keyframe, 1.5–8 m away. Each keyframe of the span that projects the point sees it in the left
  view, the right view or both, at an octave that fits the distance. A scene point seen by at least two
  keyframes becomes one map point. Keyframes far from the current one share few points with it, so they are
  not covisible: they become the fixed cameras.
- **Noise:** the initial poses (all but keyframe 0's) are about 0.3° and 1.2 cm (σ per axis) off the true
  ones. The points are 3 cm (σ per axis) off. Pixels have 0.7 σ of the octave's scale of noise, and 4 % of the
  observations are 12–20 px off.
- **Descriptors** are random bytes. Each map point's descriptor, normal and scale range come from the original's
  `ComputeDistinctiveDescriptors` and `UpdateNormalAndDepth`.
- **Distractors:** each keyframe has 20 more features without a point.

**Inputs**
- `inertial`, `n_keyframes` and `init_kf`.
- Per keyframe k, under `kfk_`: `Tcw` (the initial pose), `Tcw_true`, `timestamp`, `kp_left` and `kp_right`
  (x y octave), and `desc` (N x 32, left then right, in frame index order).
- `before_points`: per map point, its reference keyframe and its initial x y z.
- `before_kfk_points`: per frame index, the map point (a row of `before_points`) or -1.

**Outputs**
- `local`: the optimized keyframes (`lLocalKeyFrames`), ascending; an init keyframe among them is held fixed.
  `fixed`: the fixed cameras (`lFixedCameras`), ascending.
- `n_fixed` (`num_fixedKF`: the fixed cameras plus 1 when the init keyframe is local) and `n_edges`.
- `after_kfk_Tcw` and `after_kfk_points`: the poses and the point of each feature after the BA. An erased
  outlier observation shows as -1.
- `after_points`: per map point: bad, `nObs`, x y z, normal x y z, then `mfMinDistance` and `mfMaxDistance`.

**Cases:**
- `stereo`: 16 keyframes; the map is not inertial, so g2o picks the initial lambda (1e-5 · the largest
  diagonal). 7 local keyframes, 7 fixed ones; keyframe 0 is in neither.
- `inertial`: 6 keyframes; the map is inertial, so `setUserLambdaInit(100)`. Every keyframe is local, and
  keyframe 0 is held fixed as the init keyframe.

**Reproducibility:** the original iterates observations in `std::map<KeyFrame*, …>` order, that is by heap
address. Regenerating can change the edge order and so the last bits of the sums. The same goes for `fuse/`,
where a tied descriptor choice can also flip.

## local_inertial_ba/

`Optimizer::LocalInertialBA` (local mapping's local BA once the IMU is initialized). Made on 2026-11-20 by calling
the original's own function on an atlas built from its own `KeyFrame` and `MapPoint` objects. The map is
gravity aligned, inertial and IMU initialized, with BA1 done. BA2 is done except in `rec_init`.

**Scene:** K keyframes along the analytic trajectory, 0.2 s apart from t = 40 s. The last one is current
(`pKF`), and each keyframe's previous one is the one before it. Every keyframe has `bImu`.
- **IMU:** each keyframe has a velocity (true + 2 cm/s σ) and its own bias (true + 0.002 / 0.03 σ). Its
  preintegration from the previous keyframe is integrated from the trajectory's IMU samples at `lin_bias`, near
  the previous keyframe's bias. Then `SetNewBias(bias)` is called, so the preintegration's updated bias is the
  keyframe's own.
- **Scene points** span 3–8 keyframes, as in `local_ba/`, and are 1.5–11 m from the span's middle keyframe. A
  right observation's octave may differ by one from the left one's. 3 % of the observations are 12–20 px off.
- **Noise:** poses are about 0.1° and 5 mm off, points 3 cm off.
- **`mTrackDepth`** (tracking's) is set to the point's distance from the current keyframe's camera centre. The
  port uses the same rule.
- **Addresses:** the keyframes are placement-new'ed into one block, so their addresses ascend with their ids.
  The generator checks this. The original picks a fixed keyframe per point from its observations in
  `KeyFrame*` order, and the port in id order, so the two orders must agree.

**Inputs**
- `large`, `rec_init` (the `bLarge`, `bRecInit` arguments), `n_keyframes`.
- Per keyframe k, under `kfk_`: `Tcw`, `timestamp`, `velocity`, `bias` and `lin_bias` (gyro, acc), `meas`
  (keyframe 0's is empty), `kp_left`, `kp_right`, `desc`.
- `before_points`: per map point, its reference keyframe, x y z and `mTrackDepth`. `before_kfk_points` gives,
  per frame index, the map point or -1.

**Outputs**
- `ok`: whether the function got past its convergence check and wrote back.
- `optimized` and `fixed` (both ascending) and `local_points` (rows of `before_points`). The generator sets
  every keyframe's and point's `mnBALocalForKF` / `mnBAFixedForKF` to a sentinel first, then reads these marks
  back after the call.
- Per keyframe, under `after_kfk_`: `Tcw`, `velocity`, `bias`, and `preint_bias` (its preintegration's
  `GetUpdatedBias`). `after_kfk_points` and `after_points` have the same format as in `local_ba/`.

**Cases:**
- `large`: 14 keyframes, `bLarge`. 12 optimized (min(K − 2, 25)), 2 fixed, 4 iterations from λ = 1e-2.
- `small`: 16 keyframes. 10 optimized, 6 fixed, 10 iterations at most from λ = 1.
- `rec_init`: as `small`, with `bRecInit`, so every inertial edge has the Huber kernel.

None of the cases fails the convergence check. That path, and the `SetNewBias(previous keyframe's bias)` it
leaves on the preintegrations, is not covered.

## Not here

No full-sequence trajectory from the original: the port's trajectory is checked against the recordings' ground
truth (ATE) instead.

## imu_init/

`LocalMapping::InitializeIMU` on a small stereo-inertial map, step by step. Made on 2026-11-20. The optimizers,
`ApplyScaledRotation`, `SetNewBias` and `Reintegrate` are the original's own functions. Two parts of
`InitializeIMU` itself cannot run without a `Tracking`, so the generator holds a copy of their code: the gravity
direction and velocity guess, and the correction carried down the spanning tree.

**Scene:** K keyframes along the analytic trajectory, 0.25 s apart from t = 20 s. The last one is current, and each
keyframe's previous one is the one before it.
- **Preintegration:** each keyframe's runs from the previous keyframe. It is integrated from the trajectory's
  IMU samples (true biases gyro (-0.003, 0.021, 0.078), acc (-0.025, 0.11, 0.06), EuRoC noise) at the keyframe's
  bias.
- **Scene points** span 3–8 keyframes, as in `local_ba/`; 2 % of the observations are 12–20 px off.
- **Covisibility:** observations and `UpdateConnections` are added keyframe by keyframe, so each keyframe's parent
  is an earlier keyframe, as when the keyframes are made.
- **Noise:** poses are about 0.1° and 5 mm off, points 3 cm off, velocities 2 cm/s off.
- **The extra keyframe** (index K) is made after `ApplyScaledRotation`, 0.15 s after the current one, in the new
  frame. It stands for the keyframe tracking queued while `FullInertialBA` ran. Its new points (only it sees them)
  go into the map with it, as tracking's `CreateNewKeyFrame` puts them. It sees the points of the last three
  keyframes' spans too, and gets those observations when it is processed after the full BA.

**Inputs**
- `first`, `prior_g`, `prior_a`, `n_keyframes`.
- Per keyframe k = 0..K, under `kfk_`: `Tcw`, `timestamp`, `has_velocity` and `velocity`, `bias` (gyro, acc: the
  preintegration's linearization point), `meas` (the `IntegrateNewMeasurement` arguments: ax ay az wx wy wz dt per
  row; keyframe 0's preintegration is empty), `kp_left`, `kp_right`, `desc`. For the extra keyframe, use `x_Tcw`,
  `x_velocity` and `x_bias` instead: they are its state as tracking made it.
- `points` (reference keyframe, x y z), `kfk_points` (per frame index, a row of `points` or -1), `parent` (each
  keyframe's parent after the keyframe-by-keyframe build).
- `x_points` (the extra keyframe's new points) and `x_kf_points` (its points: a row of `points`, or `n_points` +
  a row of `x_points`).

**Outputs, per step**
- `guess_Rwg`, `guess_t_init`, `guess_bg`, `guess_ba`, `guess_kfk_velocity`: before `InertialOptimization`. When
  the map is not initialized these are the dirG / velocity guess; else `Rwg` = I and the current keyframe's bias.
- `io_Rwg`, `io_scale`, `io_bg`, `io_ba`: after `InertialOptimization`. Per keyframe, `io_kfk_velocity`,
  `io_kfk_bias`, `io_kfk_reintegrated`, and `io_kfk_lin_bias` (its preintegration's linearization bias after).
- `asr_kfk_Tcw`, `asr_kfk_velocity`, `asr_points` (x y z, normal x y z): after `ApplyScaledRotation(Twg, scale,
  true)`.
- `gba_kfk_Tcw`, `gba_kfk_velocity`, `gba_kfk_bias`, `gba_points` (in the BA, then x y z): `FullInertialBA`'s
  results with nLoopId = the current keyframe (`mTcwGBA`, `mVwbGBA`, `mBiasGBA`, `mPosGBA`).
- `x_parent`: the extra keyframe's parent after it is processed.
- `final_kfk_Tcw`, `final_kfk_velocity`, `final_kfk_bias` (k = 0..K), `final_points` (the extra keyframe's new
  points last): after the correction from the map's origin keyframe down the spanning tree.

**Cases:**
- `init`: the first initialization, `InitializeIMU(1e2, 1e5, true)`. 12 keyframes. The map is in the first
  camera's frame (gravity unknown), the keyframe biases are 0, and there are no velocities.
- `viba1`: `InitializeIMU(1, 1e5, true)` on an initialized map. 14 keyframes, gravity aligned, every keyframe with
  the same bias (the truth plus 2e-3 / 3e-2 of noise), velocities set.
- `viba2`: `InitializeIMU(0, 0, true)`. `InertialOptimization` runs without the user lambda, and `FullInertialBA`
  runs without bInit: a bias per keyframe, random-walk edges, no priors.

**Reproducibility:** the two optimizations end on flat costs or along unobservable directions. `FullInertialBA`
has no fixed vertex, so its translation and yaw are free. Their results therefore depend on the last bits of the
inputs: compare them with tolerances, after aligning the keyframe centres
(`tests/orb_local_mapping/imu_init_test.cpp` explains the numbers).

## loop/

Place recognition (`LoopClosing::NewDetectCommonRegions` and what it calls) with fixed scale, for
`tests/orb_loop_closing/place_recognition_test.cpp`. Made by `fixture_gen`'s `gen_loop.cpp`, which was added
when loop closing was ported.

**Scene:** sixteen keyframes. Keyframes 0..7 are pass A, along the analytic trajectory, 0.1 s apart. Keyframes
8..15 are pass B: nearly the same poses again, with drift. B's poses and points are in a world displaced by D (a
few degrees and centimetres), so the loop's relative pose is not the one the poses give. Each pass has its own
map point per observed scene point (90 %; 5 % of them moved 0.3 m off); these are the duplicates a loop finds.
Descriptors are one vocabulary word per scene point, with a few bits flipped per view; each keyframe also has 30
untracked distractors. The current keyframe is 15; the BoW database holds keyframes 0..14.

**Inputs:** `kfk_Tcw`, `kfk_kp_left` / `kp_right`, `kfk_desc`, `kfk_bow` (word, weight), `kfk_feat` (node,
frame index), `kfk_points`; `points` (reference keyframe, x y z, normal, min / max distance), `point_desc`;
`covisibility` (keyframe, covisible, weight). Ties in weight are ordered by pointer in the original.

**Outputs**
- `loop_candidates`, `merge_candidates`: `DetectNBestCandidates(current, 3)`.
- `bow_kf`, `bow_n`, `bow_matches`: `SearchByBoW(current, first candidate)` with `ORBmatcher(0.9, true)`.
- `ransac_*`: `Sim3Solver(current, bow_kf, bow_matches, fixed scale)` with `SetRansacParameters(0.99, 15, 300)`.
  `true_T12` is the truth.
- `nb_points`, `nb_kfs`: the candidate's neighbourhood. `proj_Tcw`, `proj_pinhole*`, `proj_kb8*`: the two
  `SearchByProjection(KF, Scw, ...)` overloads, (8, 1.5) and (5, 1.0).
- `opt_*`: `OptimizeSim3(current, bow_kf, proj_pinhole, opt_T12_initial, 10, fixed scale, all points)`.
  `mnTrackScaleLevel`, which it reads for points not in the candidate's left view and which this constructor
  leaves uninitialized, is set first to the level predicted from the distance to the candidate.
- `bow_detect_*`: `DetectCommonRegionsFromBoW(candidates)` once, then `bow_detect_detections` out of
  `bow_detect_runs` repeated runs. `true_Tcw` is the current keyframe's true pose in pass A's world.

**Cases:** `loop` (one map) and `merge` (pass A in another map).

**Correction cases** (`correct`, `correct_4dof`; for `tests/orb_loop_closing/correct_loop_test.cpp`):
`LoopClosing::CorrectLoop` on the same kind of scene, in one map, with two changes so the essential graph has work
left after the correction:
- The drift grows along pass B: keyframe k of B is displaced by D^f, with f from 1/8 to 1.
- Each pass B point is seen only by the B keyframes within two of a home keyframe, and is displaced as that
  keyframe is. The current keyframe's covisibles are then only the last few, and the earlier B keyframes are left
  to the essential graph.

The loop is the one `DetectCommonRegionsFromBoW` finds on `loop_candidates`, retried until it detects. `correct`
is a visual map (`OptimizeEssentialGraph`, 6DoF). `correct_4dof` is inertial with every keyframe `bImu`
(`OptimizeEssentialGraph4DoF`). Both have the IMU initialized and a second, empty map in the atlas, so
`CorrectLoop` starts no global BA. Velocities are random. The local mapper is a `LocalMapping` that is always
stopped.

- Extra inputs: `inertial`, `parents` (the spanning tree), `velocity`, `Tcb`; the loop's `loop_matched`,
  `loop_Tcw` (`mg2oLoopScw`), `loop_matched_points` (per current frame index), `loop_points`.
- Outputs: `after_kK_Tcw`, `after_kK_points`, `after_kK_loop_edges` per keyframe; `after_velocity`;
  `after_points` (x y z, bad).

The port matches to float rounding: poses about 1e-5°, points about 1e-6 m, while the graph moves keyframes by
15–18 cm. The fusion (which keyframe feature holds which point, which points went bad) matches exactly.

**Global BA case** (`gba`, generated 2026-11-23; same test): `correct` with the IMU not initialized and only one
map, so the original's `CorrectLoop` starts its global BA thread. That is `RunGlobalBundleAdjustment`: the visual
`GlobalBundleAdjustemnt(map, 10, ..., the current keyframe, bRobust = false)`, then the write-back down the spanning
tree. The generator waits for that thread to finish (`isFinishedGBA`). Two changes from `correct`, both so that
the write-back does what it does in SLAM:
- **A proper spanning tree.** The scene connects all keyframes at once, so `UpdateConnections` can make a later
  keyframe the parent, and the tree has cycles that the walk from the origin never reaches. Here each keyframe's
  parent is instead its strongest covisible among the earlier ones.
- **An origin.** Keyframe 0 is in `mvpKeyFrameOrigins` (Tracking puts it there; nothing else in this scene does).

The local mapper is also `finished`, so the write-back's `RequestStop` does not wait for a `Stop()` that no
thread would make.

- Extra input: `gba_kf` (`nLoopKF`).
- Outputs: the `after_*` tables, now taken after the global BA's write-back.

The port runs the correction, `globalBAProblem`, the `global` kind solved in steps, and `applyGlobalBA`. Poses match
to 4e-5° and 2e-7 m, and points to 5e-5 m, while the global BA moves keyframes by up to 12 cm.

That needed one quirk of the original: the visual BA counts a point's observing keyframes (`nEdges`), not its
edges. A point seen only in right images, whose observations the right-index check all drops, stays in with no
edge and is written back unchanged. `FullInertialBA` leaves such a point out (`bAllFixed`), so there it follows its
reference keyframe instead.

**Inertial merge cases** (`merge_inertial`, `merge_inertial_bias`, generated 2026-11-23; for
`tests/orb_loop_closing/merge_test.cpp`): `LoopClosing::MergeLocal2`, the stereo-inertial merge of pass B's map
into pass A's. The scene differs from `merge` in these ways:
- **Twelve pass B keyframes** (20 in all), because `MergeInertialBA` only runs with at least 10 keyframes in the
  current map.
- **Both passes on the trajectory itself:** A at 30.0–30.7 s, B over 29.8–30.9 s (its timestamps 60 s on), so
  the IMU agrees with the poses.
- **IMU state:** every keyframe is `bImu`, with a velocity, a bias near the truth, and its preintegration from
  the previous keyframe. That preintegration is integrated from the trajectory's IMU samples at a bias near the
  previous keyframe's (a pass's first keyframe has none).
- **Map flags:** both maps are inertial, with the IMU initialized and BA1. A has BA2; B has it too, except in
  `merge_inertial_bias`, where `MergeLocal2` first runs `InertialOptimization(pCurrentMap, bg, ba)`.
- **Drift:** B's drift D is a yaw and a translation, since both maps are gravity aligned. `mSold_new` is D⁻¹
  with a small error, and the matched keyframe is A's middle one (4).
- **The local-BA mark:** the points of B's last three keyframes are marked as taken by the current keyframe's
  last local BA (`mnBALocalForKF`), which `MergeInertialBA` reads.
- **No right-only observations.** For each observation, `MergeInertialBA` reads the left feature, which is −1
  for a right-only one. That is undefined: on a scene that has them, the cost is NaN from the first iteration,
  every LM step is rejected and nothing moves. (The port leaves those observations out.)
- **A spanning tree per pass**, built as SLAM grows it.
- **Allocation order:** keyframes and points are allocated in one block each, so their addresses ascend with
  their ids, the order the port uses.

`MergeLocal2` is copied into the generator with the tracker's calls left out. There is no `Tracking` object:
`UpdateFrameIMU` is tracking's, `GetLastKeyFrame` is the current keyframe, and the sensor is `IMU_STEREO`.

- Extra inputs: `inertial`, `merge_inertial`, `ba2_current`, `parents`, `velocity`, `Tcb`; per keyframe
  `kK_bias`, `kK_lin_bias`, `kK_meas`; `lba_points`; `merge_matched`, `merge_T_old_new`, `merge_last_kf`.
- Outputs, per keyframe: `after_kK_Tcw`, `after_kK_points`, `after_kK_bias`, `after_kK_preint_bias`.
- Outputs, overall: `after_parents`, `after_map`, `after_velocity`, `after_points` (x y z, bad, map),
  `after_merge_connected` (`mvpMergeConnectedKFs`), `after_change_index`, `after_ba2`.

The port runs `alignMergeInertial`, then (when there is no BA2) the `inertial_init` kind with the gravity fixed and
`applyMergeBias`, then `joinMaps`, the `merge_inertial` kind and `applyMergeInertialBA`.
- **Exact matches:** the problem (`merge_inertial`: 12 + 7 optimized keyframes, 1 fixed, 459 points, 8,067
  observations), which map each keyframe and point is in, the spanning tree, `mvpMergeConnectedKFs`, the fused
  points and the bad flags.
- **Numeric agreement:**
  - Poses match to 1e-4° and 2e-6 m.
  - Velocities match to 1e-5 m/s.
  - Biases: pass A's match to 2e-7. The current window's share one offset of 4e-5, while the BA moved them by up
    to 0.13, after 8 LM iterations that stop short of convergence.

The other `loop/` fixtures came out with float-level differences in normals and descriptors when regenerated.
Those come from the original's containers keyed by pointer, which follow the heap layout. They were not replaced.

**Reproducibility:** `Sim3Solver` samples with `rand()`, so its hypothesis, and everything downstream of it, varies
from run to run (the original's `rand()` versus the port's `std::mt19937`). `OptimizeSim3` differentiates
numerically through the KB8 projection's float internals, which makes its path chaotic. The test compares these
with tolerances and detection rates.
