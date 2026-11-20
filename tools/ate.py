#!/usr/bin/env python3
"""ATE of the tracked trajectory against the bag's ground truth, from a slam_job recording.

    python3 tools/ate.py [out/runs/<session> | <file.mcap>]   (default: the newest session in out/runs/)

Reads the taps of graphs/slam_job.graph.yaml: tracking.pose (body in odom, per tracked frame),
tracking.map_to_odom and euroc_mcap_driver.groundtruth (body in world). Each estimate is paired with the ground
truth nearest in time (within --max-dt), the estimate is aligned to it (Umeyama: SE3 for stereo /
stereo-inertial, also Sim3 for reference), and the RMSE of the position error is printed.

--frame odom (default): the continuous pose as sent. --frame map: map_to_odom · pose, the online pose in the
atlas's world as ORB-SLAM3 tracks it, which jumps whenever the map is re-anchored (IMU initialization, BAs): the
poses before the IMU initialization are in another world frame (see --from). Neither is the trajectory
re-expressed through the refined keyframes at the end, which ORB-SLAM3 saves.

The step error is the frame-to-frame motion against the ground truth's (after the alignment): the jumps.
"""
import argparse
import ctypes
import ctypes.util
import glob
import os
import struct
import sys

import numpy as np

POSE_TOPIC = "/fw/tap/main/tracking/pose"
MAP_ODOM_TOPIC = "/fw/tap/main/tracking/map_to_odom"
GT_TOPIC = "/fw/tap/main/euroc_mcap_driver/groundtruth"


def find_mcap(path):
    if path is None:
        sessions = sorted(glob.glob("out/runs/*_slam_job"))
        if not sessions:
            sys.exit("no out/runs/*_slam_job session")
        path = sessions[-1]
    if os.path.isdir(path):
        files = sorted(glob.glob(os.path.join(path, "main-*.mcap")))
        if not files:
            sys.exit(f"no main-*.mcap in {path} (an interrupted .partial: fw rec recover)")
        return files[-1]
    return path


# ---- a minimal mcap reader: the records fw writes (schema, channel, message, zstd / uncompressed chunks)
_zstd = None


def _unzstd(data, size):
    global _zstd
    if _zstd is None:
        _zstd = ctypes.CDLL(ctypes.util.find_library("zstd") or "libzstd.so.1")
        _zstd.ZSTD_decompress.restype = ctypes.c_size_t
        _zstd.ZSTD_isError.restype = ctypes.c_uint
    out = ctypes.create_string_buffer(size)
    n = _zstd.ZSTD_decompress(out, ctypes.c_size_t(size), data, ctypes.c_size_t(len(data)))
    if _zstd.ZSTD_isError(ctypes.c_size_t(n)):
        sys.exit("zstd: cannot decompress a chunk")
    return out.raw[:n]


def _string(b, o):
    (n,) = struct.unpack_from("<I", b, o)
    return b[o + 4:o + 4 + n].decode(), o + 4 + n


def _records(b):
    o = 0
    while o + 9 <= len(b):
        op, n = struct.unpack_from("<BQ", b, o)
        yield op, b[o + 9:o + 9 + n]
        o += 9 + n


def _messages(b, channels):
    """(topic, log time in s, data) of every message in the records b; channels (id -> topic) filled on the way."""
    for op, body in _records(b):
        if op == 0x04:  # Channel: id u16, schema id u16, topic, encoding, metadata
            (cid,) = struct.unpack_from("<H", body, 0)
            channels[cid], _ = _string(body, 4)
        elif op == 0x05:  # Message: channel u16, sequence u32, log time u64, publish time u64, data
            cid, _, log_time = struct.unpack_from("<HIQ", body, 0)
            yield channels.get(cid), 1e-9 * log_time, body[22:]
        elif op == 0x06:  # Chunk: start, end, uncompressed size u64, crc u32, compression, records
            (size,) = struct.unpack_from("<Q", body, 16)
            comp, o = _string(body, 28)
            (n,) = struct.unpack_from("<Q", body, o)
            data = body[o + 8:o + 8 + n]
            if comp == "zstd":
                data = _unzstd(data, size)
            elif comp:
                sys.exit(f"mcap: chunk compression {comp} not supported")
            yield from _messages(data, channels)


def _position(d):
    """geometry_msgs/PoseStamped in CDR: the position (fw leaves the header's stamp 0; the time is the log time)."""
    (n,) = struct.unpack_from("<I", d, 12)
    o = 16 + n
    o = 4 + ((o - 4 + 7) & ~7)  # doubles aligned to 8 from the start of the payload
    return struct.unpack_from("<3d", d, o)


def _transform(d):
    """geometry_msgs/TransformStamped in CDR: translation and rotation (x, y, z, w)."""
    (n,) = struct.unpack_from("<I", d, 12)
    o = 16 + n
    o = 4 + ((o - 4 + 3) & ~3)
    (n,) = struct.unpack_from("<I", d, o)  # child_frame_id
    o += 4 + n
    o = 4 + ((o - 4 + 7) & ~7)
    return struct.unpack_from("<7d", d, o)


def _rotate(q, v):
    x, y, z, w = q
    r = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    return r @ np.asarray(v)


def read_positions(mcap, frame="odom"):
    b = open(mcap, "rb").read()
    if b[:8] != b"\x89MCAP0\r\n":
        sys.exit(f"{mcap}: not an mcap file")
    out = {POSE_TOPIC: [], GT_TOPIC: []}
    map_odom = {}
    for topic, t, data in _messages(b[8:], {}):
        if topic in out:
            out[topic].append((t, *_position(data)))
        elif topic == MAP_ODOM_TOPIC:
            map_odom[t] = _transform(data)
    for topic, v in out.items():
        if not v:
            sys.exit(f"{mcap}: no messages on {topic}")
    if frame == "map":
        if not map_odom:
            sys.exit(f"{mcap}: no messages on {MAP_ODOM_TOPIC}")
        out[POSE_TOPIC] = [(t, *(_rotate(map_odom[t][3:], p) + np.array(map_odom[t][:3])))
                           for t, *p in out[POSE_TOPIC] if t in map_odom]
    return tuple(np.array(sorted(out[t])).reshape(-1, 4) for t in (POSE_TOPIC, GT_TOPIC))


def associate(est, gt, max_dt):
    idx = np.clip(np.searchsorted(gt[:, 0], est[:, 0]), 1, len(gt) - 1)
    prev_closer = np.abs(gt[idx - 1, 0] - est[:, 0]) < np.abs(gt[idx, 0] - est[:, 0])
    idx = np.where(prev_closer, idx - 1, idx)
    ok = np.abs(gt[idx, 0] - est[:, 0]) <= max_dt
    return est[ok, 1:], gt[idx[ok], 1:], est[ok, 0]


def umeyama(x, y, with_scale):
    """s, R, t minimising |y - (s R x + t)|² (x, y: N×3)."""
    mx, my = x.mean(0), y.mean(0)
    xc, yc = x - mx, y - my
    u, d, vt = np.linalg.svd(yc.T @ xc / len(x))
    s_ = np.eye(3)
    if np.linalg.det(u) * np.linalg.det(vt) < 0:
        s_[2, 2] = -1
    r = u @ s_ @ vt
    s = np.trace(np.diag(d) @ s_) / xc.var(0).sum() if with_scale else 1.0
    return s, r, my - s * r @ mx


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("path", nargs="?", help="a session folder or a .mcap (default: the newest slam_job session)")
    ap.add_argument("--frame", choices=("odom", "map"), default="odom",
                    help="odom: the continuous pose as sent (default); map: map_to_odom · pose")
    ap.add_argument("--from", dest="start", type=float, default=0.0,
                    help="skip the poses before this many seconds after the bag's start")
    ap.add_argument("--max-dt", type=float, default=0.01, help="pairing tolerance in seconds (default 0.01)")
    args = ap.parse_args()

    mcap = find_mcap(args.path)
    est, gt = read_positions(mcap, args.frame)
    if len(est) < 3 or len(gt) < 3:
        sys.exit(f"{mcap}: {len(est)} poses, {len(gt)} ground truth samples: too few")
    est = est[est[:, 0] >= gt[0, 0] + args.start]
    x, y, t = associate(est, gt, args.max_dt)
    if len(x) < 3:
        sys.exit(f"{mcap}: only {len(x)} poses paired with the ground truth")

    print(f"{mcap} ({args.frame})")
    print(f"poses {len(est)}, ground truth {len(gt)} ({gt[0, 0]:.2f}–{gt[-1, 0]:.2f} s), paired {len(x)}, "
          f"span {t[0] - gt[0, 0]:.1f}–{t[-1] - gt[0, 0]:.1f} s")
    for name, with_scale in (("SE3", False), ("Sim3", True)):
        s, r, tr = umeyama(x, y, with_scale)
        err = np.linalg.norm(y - (s * (r @ x.T).T + tr), axis=1)
        print(f"ATE {name:4}: rmse {np.sqrt((err ** 2).mean()):.4f} m, mean {err.mean():.4f}, median "
              f"{np.median(err):.4f}, max {err.max():.4f}" + (f", scale {s:.4f}" if with_scale else ""))
        if not with_scale:
            # Frame-to-frame motion against the ground truth's, consecutive frames only
            consecutive = np.diff(t) < 1.5 * np.median(np.diff(t))
            step = np.linalg.norm(np.diff((r @ x.T).T, axis=0) - np.diff(y, axis=0), axis=1)[consecutive]
            print(f"step error: median {1e3 * np.median(step):.1f} mm, p99 {1e3 * np.percentile(step, 99):.1f}, "
                  f"max {1e3 * step.max():.1f} (at {t[1:][consecutive][step.argmax()] - gt[0, 0]:.2f} s); "
                  f"steps > 10 mm: {int((step > 0.01).sum())}, > 20 mm: {int((step > 0.02).sum())}")


if __name__ == "__main__":
    main()
