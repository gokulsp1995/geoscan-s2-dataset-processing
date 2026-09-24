#!/usr/bin/env bash
#!/usr/bin/env python3
import sys
import csv
import math
import argparse
from pathlib import Path

try:
    from rosbags.rosbag2 import Reader
    from rosbags.serde import deserialize_cdr
    from rosbags.typesys import Stores, get_typestore, get_types_from_msg
except ImportError:
    print("Error: 'rosbags' library not found. Run: pip install rosbags")
    sys.exit(1)

# Register ROS 2 Jazzy Typestore & Custom Livox Definition
typestore = get_typestore(Stores.ROS2_JAZZY)

LIVOX_POINT_DEF = """
uint8 tag
uint8 line
float32 x
float32 y
float32 z
uint8 reflectivity
"""

LIVOX_CUSTOM_MSG_DEF = """
std_msgs/Header header
uint64 timebase
uint32 point_num
uint8 lidar_id
uint8[3] rsvd
livox_ros_driver2/msg/CustomPoint[] points
"""

typestore.register(get_types_from_msg(LIVOX_POINT_DEF, 'livox_ros_driver2/msg/CustomPoint'))
typestore.register(get_types_from_msg(LIVOX_CUSTOM_MSG_DEF, 'livox_ros_driver2/msg/CustomMsg'))

def latlon_to_enu(lat, lon, alt, ref):
    R = 6371000.0
    east  = R * math.radians(lon - ref[1]) * math.cos(math.radians(ref[0]))
    north = R * math.radians(lat - ref[0])
    up    = alt - ref[2]
    return east, north, up

def timestamp_str(nsec):
    return f"{nsec / 1e9:.9f}"

def save_image(msg, folder, ts_str):
    ext = 'jpg' if 'png' not in getattr(msg, 'format', '').lower() else 'png'
    data = msg.data.tobytes() if hasattr(msg.data, 'tobytes') else bytes(msg.data)
    with open(folder / f"{ts_str}.{ext}", 'wb') as f:
        f.write(data)

def save_pcd(msg, folder, ts_str):
    if not hasattr(msg, 'points') or not msg.points:
        return
    points = msg.points
    num_points = len(points)
    with open(folder / f"{ts_str}.pcd", 'w') as f:
        f.write("# .PCD v0.7 - Point Cloud Data\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n")
        f.write(f"WIDTH {num_points}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {num_points}\nDATA ascii\n")
        for p in points:
            intensity = getattr(p, 'reflectivity', 0)
            f.write(f"{p.x:.6f} {p.y:.6f} {p.z:.6f} {intensity:.1f}\n")

def process_sequence(seq_path):
    input_path = Path(seq_path).resolve()
    
    # Trace base directory
    if "ros2bag" in input_path.parts:
        idx = input_path.parts.index("ros2bag")
        base_dir = Path(*input_path.parts[:idx])
    elif "raw" in input_path.parts:
        idx = input_path.parts.index("raw")
        base_dir = Path(*input_path.parts[:idx])
    else:
        base_dir = input_path

    # Output directory scaffolding
    out = {
        "cam0":   base_dir / "raw/cam0",
        "cam1":   base_dir / "raw/cam1",
        "cam2":   base_dir / "raw/cam2",
        "lidar":  base_dir / "raw/lidar-clouds",
        "imu":    base_dir / "raw",
        "traj":   base_dir / "processed/trajectory",
    }
    for d in out.values():
        d.mkdir(parents=True, exist_ok=True)

    cam_topics = {
        "/front_camera/image/compressed": out["cam0"],
        "/left_camera/image/compressed":  out["cam1"],
        "/right_camera/image/compressed": out["cam2"],
    }

    # Discover all ROS 2 bag directories
    bag_dirs = sorted([p.parent for p in input_path.rglob("metadata.yaml")])
    if not bag_dirs:
        bag_dirs = sorted([p.parent for p in base_dir.rglob("metadata.yaml")])
        
    if not bag_dirs:
        print(f"[SKIP] No ROS 2 bag folders (metadata.yaml) found in {input_path}")
        return

    print("=" * 60)
    print(f" EXTRACTING FROM ROS 2 BAGS : {base_dir.name}")
    print(f" Target Bag Folders Located : {len(bag_dirs)}")
    print(f" Output Location            : {base_dir}")
    print("=" * 60)

    imu_file = open(out["imu"] / "imu.csv", 'w', newline='')
    gps_file = open(out["traj"] / "gt-tum.txt", 'w')
    imu_writer = csv.writer(imu_file)
    imu_writer.writerow(['timestamp', 'angular_velocity_x', 'angular_velocity_y', 'angular_velocity_z', 'linear_acceleration_x', 'linear_acceleration_y', 'linear_acceleration_z', 'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w'])
    gps_file.write("# timestamp tx ty tz qx qy qz qw\n")

    gps_ref = None
    counters = {"cam0": 0, "cam1": 0, "cam2": 0, "imu": 0, "lidar": 0, "gnss": 0}

    for b_idx, bag_dir in enumerate(bag_dirs, 1):
        print(f"[{b_idx}/{len(bag_dirs)}] Reading {bag_dir.name}...")
        try:
            with Reader(bag_dir) as reader:
                for connection, timestamp, rawdata in reader.messages():
                    topic = connection.topic
                    msg_type = connection.msgtype
                    ts_str = timestamp_str(timestamp)

                    if topic in cam_topics:
                        msg = deserialize_cdr(rawdata, msg_type, typestore=typestore)
                        save_image(msg, cam_topics[topic], ts_str)
                        counters[cam_topics[topic].name] += 1

                    elif topic in ("/lidar", "/livox/lidar"):
                        msg = deserialize_cdr(rawdata, msg_type, typestore=typestore)
                        save_pcd(msg, out["lidar"], ts_str)
                        counters["lidar"] += 1

                    elif topic in ("/imu", "/livox/imu"):
                        msg = deserialize_cdr(rawdata, msg_type, typestore=typestore)
                        imu_writer.writerow([
                            ts_str,
                            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
                            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
                            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
                        ])
                        counters["imu"] += 1

                    elif topic == "/handsfree/rtk/gnss":
                        msg = deserialize_cdr(rawdata, msg_type, typestore=typestore)
                        lat, lon, alt = msg.latitude, msg.longitude, msg.altitude
                        if gps_ref is None and lat != 0:
                            gps_ref = (lat, lon, alt)
                        if gps_ref is not None:
                            e, n, u = latlon_to_enu(lat, lon, alt, gps_ref)
                            gps_file.write(f"{ts_str} {e:.6f} {n:.6f} {u:.6f} 0.0 0.0 0.0 1.0\n")
                            counters["gnss"] += 1
        except Exception as e:
            print(f"  [Error] {bag_dir.name}: {e}")

    imu_file.close()
    gps_file.close()
    if counters["gnss"] == 0:
        (out["traj"] / "gt-tum.txt").unlink(missing_ok=True)

    print("\nExtraction Summary:")
    for k, v in counters.items():
        print(f"  - {k:10s}: {v} records extracted")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Direct ROS 2 Bag Data Extractor")
    parser.add_argument("sequence_path", type=str, help="Path to sequence or ros2bag directory")
    args = parser.parse_args()
    process_sequence(args.sequence_path)
