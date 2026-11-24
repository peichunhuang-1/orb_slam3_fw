#!/usr/bin/env python3
"""TUM-VI (EuRoC-format export, dataset-<seq>_512_16.tar) -> an mcap for drivers/euroc_mcap_driver.

    source /opt/ros/humble/setup.bash
    python3 tools/tumvi_to_mcap.py bags/dataset-room1_512_16.tar bags/tumvi_room1_512.mcap [--no-clahe]

Reads the tar in place (nothing is extracted). The topics are the ones the driver reads from the EuRoC bags:
  /cam0/camera_info, /cam1/camera_info   equidistant (KB, d = k1..k4) from dso/camchain.yaml, at the first stamp
  /tf_static                             body -> cam0, cam1 (inverse of camchain's T_cam_imu), body -> imu0 (identity)
  /imu0                                  sensor_msgs/Imu from mav0/imu0/data.csv
  /cam0, /cam1 image_raw/compressed      PNG, 8-bit: the 16-bit images read as ORB-SLAM3's TUM-VI example does
                                         (cv::IMREAD_GRAYSCALE) and, by default, its CLAHE (clip 3.0, 8x8 tiles)
  /groundtruth/pose                      mav0/mocap0/data.csv: the IMU in the mocap world (= dso/gt_imu.csv)
Every stamp is the sensor's (header, log and publish time alike), messages in time order. The file is plain mcap
(no chunks, no index, no compression): the driver reads it linearly.

The IMU noise is not in the bag: set it in the driver's parameters (dso/imu_config.yaml, printed at the end).
"""
import argparse
import os
import re
import struct
import sys
import tarfile

import cv2
import numpy as np
import yaml
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.serialization import serialize_message
from sensor_msgs.msg import CameraInfo, CompressedImage, Imu
from tf2_msgs.msg import TFMessage

ROS_SHARE = "/opt/ros/humble/share"


# --- ros2msg schema text (the message definition with its dependencies, as rosbag2 writes it) --------------------

def _msg_text(full):
    pkg, name = full.split("/")[0], full.split("/")[-1]
    with open(os.path.join(ROS_SHARE, pkg, "msg", name + ".msg")) as f:
        return f.read()


def _deps(full, text):
    pkg = full.split("/")[0]
    out = []
    for line in text.splitlines():
        line = line.split("#")[0].strip()
        if not line:
            continue
        t = re.sub(r"\[.*\]", "", line.split()[0])
        if t in ("bool", "byte", "char", "float32", "float64", "int8", "uint8", "int16", "uint16", "int32", "uint32",
                 "int64", "uint64", "string", "wstring") or t.startswith("string<") or "=" in line.split()[1:2]:
            continue
        if t == "Header":
            t = "std_msgs/Header"
        out.append(t if "/" in t else pkg + "/" + t)
    return out


def schema_text(full):
    """full: pkg/msg/Name."""
    full = full.replace("/msg/", "/")
    top = _msg_text(full)
    parts, seen, todo = [], {full}, _deps(full, top)
    while todo:
        d = todo.pop(0)
        if d in seen:
            continue
        seen.add(d)
        t = _msg_text(d)
        pkg, name = d.split("/")
        parts.append("=" * 80 + "\nMSG: " + pkg + "/msg/" + name + "\n" + t)
        todo += _deps(d, t)
    return top + ("\n" if parts else "") + "\n".join(parts)


# --- mcap writing (unchunked) ---------------------------------------------------------------------------------------

def _s(x):
    b = x.encode() if isinstance(x, str) else x
    return struct.pack("<I", len(b)) + b


class Mcap:
    def __init__(self, path):
        self.f = open(path, "wb")
        self.f.write(b"\x89MCAP0\r\n")
        self._rec(0x01, _s("ros2") + _s("orb_slam3_fw/tumvi-converter"))
        self.channels, self.seq, self.n = {}, {}, 0

    def _rec(self, op, body):
        self.f.write(struct.pack("<BQ", op, len(body)) + body)

    def channel(self, topic, type_name):
        if topic in self.channels:
            return self.channels[topic]
        sid = cid = len(self.channels) + 1
        self._rec(0x03, struct.pack("<H", sid) + _s(type_name) + _s("ros2msg") + _s(schema_text(type_name)))
        meta = struct.pack("<I", 0)  # empty map
        self._rec(0x04, struct.pack("<HH", cid, sid) + _s(topic) + _s("cdr") + meta)
        self.channels[topic] = cid
        self.seq[cid] = 0
        return cid

    def write(self, topic, type_name, msg, t_ns):
        cid = self.channel(topic, type_name)
        data = serialize_message(msg)
        self._rec(0x05, struct.pack("<HIQQ", cid, self.seq[cid], t_ns, t_ns) + data)
        self.seq[cid] += 1
        self.n += 1

    def close(self):
        self._rec(0x0F, struct.pack("<I", 0))   # DataEnd
        self._rec(0x02, struct.pack("<QQI", 0, 0, 0))  # Footer: no summary
        self.f.write(b"\x89MCAP0\r\n")
        self.f.close()


# --- TUM-VI ----------------------------------------------------------------------------------------------------------

def stamp(ns):
    return Time(sec=int(ns // 1_000_000_000), nanosec=int(ns % 1_000_000_000))


def read_csv(tf, member):
    rows = []
    for line in tf.extractfile(member).read().decode().splitlines():
        if line.startswith("#") or not line.strip():
            continue
        rows.append([x.strip() for x in line.split(",")])
    return rows


def camera_info(cam, frame, ns):
    m = CameraInfo()
    m.header.stamp, m.header.frame_id = stamp(ns), frame
    m.width, m.height = cam["resolution"]
    fx, fy, cx, cy = cam["intrinsics"]
    m.distortion_model = cam["distortion_model"]   # equidistant
    m.d = [float(x) for x in cam["distortion_coeffs"]]
    m.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    m.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    m.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return m


def quat_xyzw(R):
    """Rotation matrix -> unit quaternion (x, y, z, w)."""
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        w, x, y, z = 0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w, x, y, z = (R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w, x, y, z = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w, x, y, z = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def tf_static(T_body_cam0, T_body_cam1, ns):
    m = TFMessage()
    for child, T in (("cam0", T_body_cam0), ("cam1", T_body_cam1), ("imu0", np.eye(4))):
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = stamp(ns), "body", child
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = (float(v) for v in T[:3, 3])
        q = quat_xyzw(T[:3, :3])
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = \
            (float(v) for v in q)
        m.transforms.append(t)
    return m


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("tar")
    ap.add_argument("out")
    ap.add_argument("--no-clahe", action="store_true", help="images as read, without CLAHE")
    args = ap.parse_args()

    tf = tarfile.open(args.tar)
    members = {m.name: m for m in tf.getmembers()}
    root = next(n for n in members if n.endswith("/mav0")).rsplit("/mav0", 1)[0]
    chain = yaml.safe_load(tf.extractfile(members[root + "/dso/camchain.yaml"]).read())
    imu_cfg = yaml.safe_load(tf.extractfile(members[root + "/dso/imu_config.yaml"]).read())

    cams = [read_csv(tf, members[f"{root}/mav0/cam{i}/data.csv"]) for i in (0, 1)]
    imu = read_csv(tf, members[root + "/mav0/imu0/data.csv"])
    gt = read_csv(tf, members[root + "/mav0/mocap0/data.csv"])
    T_body_cam = [np.linalg.inv(np.array(chain[f"cam{i}"]["T_cam_imu"], dtype=float)) for i in (0, 1)]

    events = []   # (t_ns, order, kind, payload)
    for i, rows in enumerate(cams):
        for t, fn in rows:
            events.append((int(t), 2, "img", (i, f"{root}/mav0/cam{i}/data/{fn}")))
    for r in imu:
        events.append((int(r[0]), 1, "imu", [float(x) for x in r[1:7]]))
    for r in gt:
        events.append((int(r[0]), 3, "gt", [float(x) for x in r[1:8]]))
    events.sort(key=lambda e: (e[0], e[1], str(e[3][0]) if e[2] == "img" else ""))
    t0 = min(e[0] for e in events)

    clahe = None if args.no_clahe else cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
    out = Mcap(args.out)
    out.write("/cam0/camera_info", "sensor_msgs/msg/CameraInfo", camera_info(chain["cam0"], "cam0", t0), t0)
    out.write("/cam1/camera_info", "sensor_msgs/msg/CameraInfo", camera_info(chain["cam1"], "cam1", t0), t0)
    out.write("/tf_static", "tf2_msgs/msg/TFMessage", tf_static(T_body_cam[0], T_body_cam[1], t0), t0)

    n_img = 0
    for t, _, kind, p in events:
        if kind == "imu":
            m = Imu()
            m.header.stamp, m.header.frame_id = stamp(t), "imu0"
            m.orientation.w = 1.0
            m.orientation_covariance = [99999.9, 0.0, 0.0, 0.0, 99999.9, 0.0, 0.0, 0.0, 99999.9]
            m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = p[0:3]
            m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = p[3:6]
            out.write("/imu0", "sensor_msgs/msg/Imu", m, t)
        elif kind == "gt":
            m = PoseStamped()
            m.header.stamp, m.header.frame_id = stamp(t), "world"
            m.pose.position.x, m.pose.position.y, m.pose.position.z = p[0:3]
            m.pose.orientation.w, m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z = p[3:7]
            out.write("/groundtruth/pose", "geometry_msgs/msg/PoseStamped", m, t)
        else:
            i, name = p
            raw = np.frombuffer(tf.extractfile(members[name]).read(), np.uint8)
            img = cv2.imdecode(raw, cv2.IMREAD_GRAYSCALE)   # 16 -> 8 bit as cv::imread(IMREAD_GRAYSCALE)
            if clahe is not None:
                img = clahe.apply(img)
            ok, png = cv2.imencode(".png", img)
            if not ok:
                sys.exit(f"cannot encode {name}")
            m = CompressedImage()
            m.header.stamp, m.header.frame_id = stamp(t), f"cam{i}"
            m.format = "png"
            m.data = png.tobytes()
            out.write(f"/cam{i}/image_raw/compressed", "sensor_msgs/msg/CompressedImage", m, t)
            n_img += 1
            if n_img % 1000 == 0:
                print(f"  {n_img} images, t {1e-9 * (t - t0):.1f} s", file=sys.stderr)
    out.close()
    print(f"{args.out}: {out.n} messages ({n_img} images, {len(imu)} IMU, {len(gt)} ground truth), "
          f"{1e-9 * (events[-1][0] - t0):.1f} s, CLAHE {'off' if clahe is None else 'on'}")
    print("IMU noise for the driver's parameters (dso/imu_config.yaml):")
    print(f"  gyro_noise: {imu_cfg['gyroscope_noise_density']}\n  gyro_walk: {imu_cfg['gyroscope_random_walk']}\n"
          f"  accel_noise: {imu_cfg['accelerometer_noise_density']}\n  accel_walk: {imu_cfg['accelerometer_random_walk']}")


if __name__ == "__main__":
    main()
