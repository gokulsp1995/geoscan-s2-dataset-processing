#!/usr/bin/env python3
"""
inspect_onboard.py -- dump the frames, static extrinsics and onboard-SLAM trajectory of a ROS 2 bag

  python inspect_onboard.py ~/.../raw/ros2bag/office_ros2 | tee onboard_report.txt

Prints:
  * every /tf_static transform (parent -> child, xyz, roll/pitch/yaw)  = LiDAR/IMU/camera extrinsics
  * /global/slam_odom: frame ids, first/last pose, path length, start-to-end distance (loop closure
    check), max speed, gaps, and the yaw/pitch/roll at the start
  * the set of parent->child pairs seen on /tf
"""
import sys
from collections import OrderedDict
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore


def rpy(q):
    x, y, z, w = q
    r = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    p = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
    yw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.degrees([r, p, yw])


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    ts = get_typestore(Stores.ROS2_JAZZY)
    static, dyn_pairs = OrderedDict(), set()
    odom_t, odom_p, odom_q, frames = [], [], [], set()
    with AnyReader([Path(sys.argv[1])], default_typestore=ts) as r:
        want = [c for c in r.connections if c.topic in ("/tf_static", "/tf", "/global/slam_odom")]
        for c, _, raw in r.messages(connections=want):
            m = r.deserialize(raw, c.msgtype)
            if c.topic == "/tf_static":
                for t in m.transforms:
                    tr, q = t.transform.translation, t.transform.rotation
                    static[(t.header.frame_id, t.child_frame_id)] = (
                        np.array([tr.x, tr.y, tr.z]), np.array([q.x, q.y, q.z, q.w]))
            elif c.topic == "/tf":
                for t in m.transforms:
                    dyn_pairs.add((t.header.frame_id, t.child_frame_id))
            else:
                p, q = m.pose.pose.position, m.pose.pose.orientation
                odom_t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
                odom_p.append([p.x, p.y, p.z])
                odom_q.append([q.x, q.y, q.z, q.w])
                frames.add((m.header.frame_id, m.child_frame_id))

    print("=== /tf_static (extrinsics) ===")
    for (a, b), (t, q) in static.items():
        print(f"{a:>22s} -> {b:<22s} xyz [{t[0]:+.4f} {t[1]:+.4f} {t[2]:+.4f}] m   "
              f"rpy [{rpy(q)[0]:+7.2f} {rpy(q)[1]:+7.2f} {rpy(q)[2]:+7.2f}] deg")
    if not static:
        print("(none found)")

    print("\n=== /tf (dynamic) parent -> child pairs ===")
    for a, b in sorted(dyn_pairs):
        print(f"{a} -> {b}")

    print("\n=== /global/slam_odom ===")
    if not odom_t:
        print("no odometry messages")
        return
    t, P, Q = np.array(odom_t), np.array(odom_p), np.array(odom_q)
    d = np.linalg.norm(np.diff(P, axis=0), axis=1)
    dt = np.diff(t)
    print(f"frame_id / child_frame_id: {sorted(frames)}")
    print(f"poses {len(t)}   duration {t[-1] - t[0]:.1f} s   median dt {np.median(dt) * 1000:.0f} ms   max gap {dt.max() * 1000:.0f} ms")
    print(f"first pose xyz {np.round(P[0], 3)}  rpy {np.round(rpy(Q[0]), 2)} deg")
    print(f"last  pose xyz {np.round(P[-1], 3)}  rpy {np.round(rpy(Q[-1]), 2)} deg")
    print(f"path length {d.sum():.1f} m   start-to-end distance {np.linalg.norm(P[-1] - P[0]):.2f} m"
          f"   (small if you ended where you started)")
    print(f"max speed {np.max(d / np.maximum(dt, 1e-3)):.2f} m/s   max single jump {d.max():.2f} m")
    print(f"z range {P[:, 2].min():.2f} .. {P[:, 2].max():.2f} m  (big spread on a flat floor = drift or stairs)")
    big = np.where(d > 0.5)[0]
    if len(big):
        print(f"!! {len(big)} jumps > 0.5 m between consecutive poses at t+{np.round(t[big][:5] - t[0], 1)} s "
              "(loop-closure correction or tracking loss)")


if __name__ == "__main__":
    main()
