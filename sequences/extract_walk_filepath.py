#!/usr/bin/env bash
#!/usr/bin/env python3
import os
import sys
import csv
import math
import argparse
from pathlib import Path

try:
    import rosbag
except ImportError as e:
    print(f"Missing dependency: {e}. Make sure ROS 1 Noetic is sourced.")
    sys.exit(1)

def latlon_to_enu(lat, lon, alt, ref):
    R = 6371000.0
    east  = R * math.radians(lon - ref[1]) * math.cos(math.radians(ref[0]))
    north = R * math.radians(lat - ref[0])
    up    = alt - ref[2]
    return east, north, up

def timestamp_str(ts):
    return f"{ts.to_sec():.9f}"

def save_image_fast(msg, folder, ts):
    ext = 'jpg' if 'png' not in getattr(msg, 'format', '').lower() else 'png'
    fname = folder / f"{timestamp_str(ts)}.{ext}"
    with open(fname, 'wb') as f:
        f.write(msg.data)

def save_pcd(msg, folder, ts):
    fname = folder / f"{timestamp_str(ts)}.pcd"
    if not hasattr(msg, 'points') or not msg.points:
        return
        
    points = msg.points
    num_points = len(points)
    
    with open(fname, 'w') as f:
        f.write("# .PCD v0.7 - Point Cloud Data\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z intensity\n")
        f.write("SIZE 4 4 4 4\n")
        f.write("TYPE F F F F\n")
        f.write("COUNT 1 1 1 1\n")
        f.write(f"WIDTH {num_points}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {num_points}\n")
        f.write("DATA ascii\n")
        for p in points:
            intensity = getattr(p, 'reflectivity', getattr(p, 'intensity', 0))
            f.write(f"{p.x:.6f} {p.y:.6f} {p.z:.6f} {intensity:.1f}\n")

def process_walk(walk_path):
    input_path = Path(walk_path).resolve()
    
    # 1. ROBUST BASE DIRECTORY RESOLUTION
    # If the user passed raw/, rosbag/, or DataBag_*, trace back to the walk sequence root
    if "raw" in input_path.parts:
        idx = input_path.parts.index("raw")
        base_dir = Path(*input_path.parts[:idx])
    else:
        base_dir = input_path

    # 2. ROBUST RECURSIVE BAG DISCOVERY
    # Search from input_path down for any .bag files
    bags = sorted(list(input_path.rglob("*.bag")))
    if not bags and base_dir != input_path:
        bags = sorted(list(base_dir.rglob("*.bag")))
        
    total_bags = len(bags)
    if total_bags == 0:
        print(f"[SKIP] No .bag files found in {input_path}")
        return

    # 3. CONSTRUCT OUTPUT PATHS IN CANONICAL DATASET HIERARCHY
    out = {
        "cam0":   base_dir / "raw/cam0",
        "cam1":   base_dir / "raw/cam1",
        "cam2":   base_dir / "raw/cam2",
        "lidar":  base_dir / "raw/lidar-clouds",
        "traj":   base_dir / "processed/trajectory",
        "imu":    base_dir / "raw",
    }

    topics = {
        "/front_camera/image/compressed":  "cam0",
        "/left_camera/image/compressed":   "cam1",
        "/right_camera/image/compressed":  "cam2",
        "/imu":                            "imu",
        "/lidar":                          "lidar",
        "/handsfree/rtk/gnss":             "gnss",
    }

    for d in out.values():
        d.mkdir(parents=True, exist_ok=True)

    print("=" * 60)
    print(f" PROCESSING SEQUENCE : {base_dir.name}")
    print(f" Total Bags Located  : {total_bags}")
    print(f" Output Directory    : {base_dir}")
    print("=" * 60)

    imu_file = open(out["imu"] / "imu.csv", 'w', newline='')
    gps_file = open(out["traj"] / "gps-tum.txt", 'w')
    
    imu_writer = csv.writer(imu_file)
    imu_writer.writerow([
        'timestamp',
        'angular_velocity_x', 'angular_velocity_y', 'angular_velocity_z',
        'linear_acceleration_x', 'linear_acceleration_y', 'linear_acceleration_z',
        'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w'
    ])
    
    gps_file.write("# GPS trajectory in TUM format\n")
    gps_file.write("# timestamp tx ty tz qx qy qz qw\n")

    gps_ref = None
    counters = {k: 0 for k in set(topics.values())}

    for idx, bag_path in enumerate(bags, start=1):
        pct = (idx / total_bags) * 100
        print(f"\n[{idx}/{total_bags}] ({pct:.1f}%) Reading {bag_path.name}...")
        
        try:
            with rosbag.Bag(str(bag_path), 'r') as bag:
                for topic, msg, ts in bag.read_messages(topics=list(topics.keys())):
                    cat = topics[topic]
                    
                    if cat in ("cam0", "cam1", "cam2"):
                        save_image_fast(msg, out[cat], ts)
                        counters[cat] += 1
                        if counters[cat] % 300 == 0:
                            print(f"  {cat}: {counters[cat]} images extracted")
                    
                    elif cat == "imu":
                        imu_writer.writerow([
                            timestamp_str(ts),
                            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
                            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
                            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
                        ])
                        counters["imu"] += 1
                    
                    elif cat == "lidar":
                        save_pcd(msg, out[cat], ts)
                        counters["lidar"] += 1
                        if counters["lidar"] % 200 == 0:
                            print(f"  lidar: {counters['lidar']} scans extracted")
                    
                    elif cat == "gnss":
                        lat, lon, alt = msg.latitude, msg.longitude, msg.altitude
                        if gps_ref is None and lat != 0:
                            gps_ref = (lat, lon, alt)
                            print(f"  GPS origin fixed at: lat={lat:.6f}, lon={lon:.6f}, alt={alt:.1f}m")
                            gps_file.write(f"# origin lat={lat} lon={lon} alt={alt}\n")
                        
                        if gps_ref is not None:
                            e, n, u = latlon_to_enu(lat, lon, alt, gps_ref)
                            gps_file.write(f"{timestamp_str(ts)} {e:.6f} {n:.6f} {u:.6f} 0.0 0.0 0.0 1.0\n")
                        counters["gnss"] += 1
        except Exception as e:
            print(f"[Error] Failed processing {bag_path.name}: {e}")

    imu_file.close()
    gps_file.close()

    # Clean up empty GPS file if this was an indoor walk with zero RTK fixes
    if counters["gnss"] == 0:
        (out["traj"] / "gps-tum.txt").unlink(missing_ok=True)

    print("\n" + "=" * 50)
    print(f"EXTRACTION COMPLETE: {base_dir.name}")
    print("=" * 50)
    for k, v in counters.items():
        print(f"  {k:10s}: {v:6d} extracted")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Extract ROS bag datasets.")
    parser.add_argument("walk_path", type=str, help="Path to the walk folder (e.g., wantage_walk_5_north_park)")
    args = parser.parse_args()
    process_walk(args.walk_path)