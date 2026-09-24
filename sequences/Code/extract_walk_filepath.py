
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
    print(f"Missing dependency: {e}")
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
        f.write("# .PCD v0.7 - Point Cloud Data\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n")
        f.write(f"WIDTH {num_points}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {num_points}\nDATA ascii\n")
        for p in points:
            f.write(f"{p.x:.6f} {p.y:.6f} {p.z:.6f} {getattr(p, 'reflectivity', 0):.1f}\n")

def process_walk(walk_dir):
    walk_dir = Path(walk_dir).resolve()
    # Locate rosbag folder
    bag_dir = list(walk_dir.glob("raw/rosbag/*"))
    if not bag_dir or not bag_dir[0].is_dir():
        bag_dir = walk_dir / "raw/rosbag"
    else:
        bag_dir = bag_dir[0]

    print(f"\nTarget Walk Directory: {walk_dir}")
    print(f"Reading bags from:     {bag_dir}")

    out = {
        "cam0":   walk_dir / "raw/cam0",
        "cam1":   walk_dir / "raw/cam1",
        "cam2":   walk_dir / "raw/cam2",
        "lidar":  walk_dir / "raw/lidar-clouds",
        "traj":   walk_dir / "processed/trajectory",
        "imu":    walk_dir / "raw",
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

    bags = sorted(list(bag_dir.glob("*.bag")))
    if not bags:
        print(f"No .bag files found in {bag_dir}")
        return

    print(f"Found {len(bags)} bag files to process.")

    imu_file = open(out["imu"] / "imu.csv", 'w', newline='')
    gps_file = open(out["traj"] / "gps-tum.txt", 'w')
    
    imu_writer = csv.writer(imu_file)
    imu_writer.writerow(['timestamp','angular_velocity_x','angular_velocity_y','angular_velocity_z','linear_acceleration_x','linear_acceleration_y','linear_acceleration_z','orientation_x','orientation_y','orientation_z','orientation_w'])
    gps_file.write("# GPS trajectory in TUM format\n# timestamp tx ty tz qx qy qz qw\n")

    gps_ref = None
    counters = {k: 0 for k in topics.values()}

    for bag_path in bags:
        print(f"Processing {bag_path.name}...")
        try:
            with rosbag.Bag(str(bag_path), 'r') as bag:
                for topic, msg, ts in bag.read_messages(topics=list(topics.keys())):
                    cat = topics[topic]
                    if cat in ("cam0", "cam1", "cam2"):
                        save_image_fast(msg, out[cat], ts)
                        counters[cat] += 1
                    elif cat == "imu":
                        imu_writer.writerow([timestamp_str(ts), msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z, msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z, msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
                        counters["imu"] += 1
                    elif cat == "lidar":
                        save_pcd(msg, out[cat], ts)
                        counters["lidar"] += 1
                    elif cat == "gnss":
                        lat, lon, alt = msg.latitude, msg.longitude, msg.altitude
                        if gps_ref is None and lat != 0:
                            gps_ref = (lat, lon, alt)
                            gps_file.write(f"# origin lat={lat} lon={lon} alt={alt}\n")
                        if gps_ref is not None:
                            e, n, u = latlon_to_enu(lat, lon, alt, gps_ref)
                            gps_file.write(f"{timestamp_str(ts)} {e:.6f} {n:.6f} {u:.6f} 0.0 0.0 0.0 1.0\n")
                        counters["gnss"] += 1
        except Exception as e:
            print(f"[Error] Failed processing {bag_path.name}: {e}")

    imu_file.close()
    gps_file.close()
    print(f"Finished processing sequence: {walk_dir.name}\n")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Extract ROS bag datasets.")
    parser.add_argument("walk_dir", type=str, help="Path to the main walk directory (e.g. wantage_walk_1)")
    args = parser.parse_args()
    process_walk(args.walk_dir)
