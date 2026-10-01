#!/usr/bin/env python3
"""
check_bag.py -- sanity-check a converted Geoscan / Livox ROS 2 bag (no ROS needed, uses `rosbags`)

  pip install rosbags numpy
  python check_bag.py ~/Projects/.../ros2bag/data_0          # one split
  python check_bag.py ~/Projects/.../ros2bag/data_*          # all splits in order

Reports per topic: rate from header stamps, gaps, frame_id, stamp-vs-record offset, time going
backwards; IMU units (g vs m/s^2) and stillness; Livox points/scan, scan duration, line ids;
image formats.
"""
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_types_from_msg, get_typestore

LIVOX_POINT = """uint32 offset_time
float32 x
float32 y
float32 z
uint8 reflectivity
uint8 tag
uint8 line
"""
LIVOX_MSG = """std_msgs/Header header
uint64 timebase
uint32 point_num
uint8 lidar_id
uint8[3] rsvd
livox_ros_driver2/CustomPoint[] points
"""


def main():
    paths = [Path(p) for p in sys.argv[1:]]
    if not paths:
        sys.exit(__doc__)
    ts = get_typestore(Stores.ROS2_JAZZY)
    types = {}
    types.update(get_types_from_msg(LIVOX_POINT, "livox_ros_driver2/msg/CustomPoint"))
    types.update(get_types_from_msg(LIVOX_MSG, "livox_ros_driver2/msg/CustomMsg"))
    ts.register(types)

    stamps, rec, frames = defaultdict(list), defaultdict(list), defaultdict(set)
    acc, gyr = [], []
    npts, scan_dur, lines, tags, tb_off = [], [], set(), defaultdict(int), []
    fmts, sizes = defaultdict(set), defaultdict(list)
    errors = defaultdict(int)

    with AnyReader(paths, default_typestore=ts) as r:
        for c, t_rec, raw in r.messages():
            try:
                m = r.deserialize(raw, c.msgtype)
            except Exception as e:  # wrong/unknown msg definition
                errors[c.topic] += 1
                if errors[c.topic] == 1:
                    print(f"!! cannot deserialize {c.topic} ({c.msgtype}): {e}")
                continue
            h = getattr(m, "header", None)
            if h is not None:
                stamps[c.topic].append(h.stamp.sec + h.stamp.nanosec * 1e-9)
                frames[c.topic].add(h.frame_id)
            rec[c.topic].append(t_rec * 1e-9)
            if c.msgtype == "sensor_msgs/msg/Imu":
                a, g = m.linear_acceleration, m.angular_velocity
                acc.append((a.x, a.y, a.z))
                gyr.append((g.x, g.y, g.z))
            elif c.msgtype.endswith("CustomMsg"):
                npts.append(m.point_num)
                if m.points is not None and len(m.points):
                    off = [p.offset_time for p in m.points]
                    scan_dur.append((max(off) - min(off)) * 1e-9)
                    lines.update(p.line for p in m.points[::37])
                    for p in m.points[::37]:
                        tags[p.tag] += 1
                tb_off.append(m.timebase * 1e-9 - stamps[c.topic][-1])
            elif c.msgtype.endswith("CompressedImage"):
                fmts[c.topic].add(m.format)
                sizes[c.topic].append(len(m.data))

    print("\n=== timing (from header stamps) ===")
    for tp in sorted(rec):
        s = np.array(stamps.get(tp) or rec[tp])
        d = np.diff(s)
        rate = (len(s) - 1) / (s[-1] - s[0]) if len(s) > 1 and s[-1] > s[0] else 0
        back = int((d < 0).sum()) if len(d) else 0
        gap = d.max() if len(d) else 0
        offs = np.array(rec[tp][: len(s)]) - s if tp in stamps else np.array([0.0])
        print(f"{tp:38s} n={len(s):6d}  {rate:7.2f} Hz  max gap {gap * 1000:8.1f} ms  "
              f"backwards {back}  frame {sorted(frames[tp]) or '-'}  "
              f"record-stamp offset {np.median(offs):+.3f} s")
        if len(d) and np.median(d) > 0 and gap > 3 * np.median(d):
            big = int((d > 3 * np.median(d)).sum())
            print(f"   !! {big} gaps > 3x the normal period (dropped messages)")

    if acc:
        A, G = np.array(acc), np.array(gyr)
        n = np.linalg.norm(A, axis=1)
        unit = "g  (Livox native)" if np.median(n) < 3 else "m/s^2"
        print("\n=== IMU ===")
        print(f"|acc| median {np.median(n):.3f}  -> units look like {unit}")
        print(f"mean acc {np.round(A.mean(0), 3)}  (gravity direction in the IMU frame)")
        k = min(len(n), 400)  # first ~2 s at 200 Hz
        print(f"first ~2 s: |gyro| mean {np.linalg.norm(G[:k], axis=1).mean():.4f} rad/s, "
              f"|acc| std {n[:k].std():.4f}  -> {'still at start (good for LIO init)' if n[:k].std() < 0.02 * np.median(n) else 'MOVING at start (LIO init may be poor)'}")

    if npts:
        print("\n=== Livox CustomMsg ===")
        print(f"points/scan median {int(np.median(npts))}  min {min(npts)}  max {max(npts)}")
        if scan_dur:
            print(f"scan duration (offset_time span) median {np.median(scan_dur) * 1000:.1f} ms  (10 Hz -> ~100 ms)")
        print(f"line ids seen {sorted(lines)}  (Mid-360 -> 0..3)")
        print(f"tags (sampled) {dict(tags)}")
        print(f"timebase - header.stamp median {np.median(tb_off):+.6f} s  (should be ~0)")

    for tp in sorted(fmts):
        print(f"\n{tp}: format {sorted(fmts[tp])}, median {np.median(sizes[tp]) / 1024:.0f} KiB/frame")
    if errors:
        print(f"\n!! deserialization errors: {dict(errors)}")


if __name__ == "__main__":
    main()