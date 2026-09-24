#!/usr/bin/env bash
set -e

# ==============================================================================
# Master Ingestion Script: GeoScan Drop -> Target Dataset Sequence
# Usage: ./build_dataset_sequence.sh <staging_input_folder> <target_dataset_root> <sequence_name>
# ==============================================================================

if [ -z "$1" ] || [ -z "$2" ] || [ -z "$3" ]; then
  echo "Usage: $0 <staging_input_folder> <target_dataset_root> <sequence_name>"
  exit 1
fi

STAGING_DIR="$(cd "$1" && pwd)"
DATASET_ROOT="$(cd "$2" && pwd)"
SEQ_NAME="$3"

SEQ_DIR="${DATASET_ROOT}/${SEQ_NAME}"
RAW_DIR="${SEQ_DIR}/raw"
PROCESSED_DIR="${SEQ_DIR}/processed"
DOCKER_IMAGE="ros:noetic-robot"

echo "=================================================================="
echo "  INGESTING GEOSCAN OUTPUT -> DATASET SEQUENCE"
echo "  Staging Source : $STAGING_DIR"
echo "  Dataset Target : $SEQ_DIR"
echo "=================================================================="

# 1. Create Target Directory Tree on Host
mkdir -p "${RAW_DIR}/rosbag" \
         "${RAW_DIR}/cam0" "${RAW_DIR}/cam1" "${RAW_DIR}/cam2" \
         "${RAW_DIR}/lidar-clouds" \
         "${RAW_DIR}/ros2bag" \
         "${PROCESSED_DIR}/trajectory" \
         "${PROCESSED_DIR}/maps"

# 2. Copy Raw Scanner Drop (Global map copied to processed/maps/ untouched)
echo -e "\n[1/4] Moving raw scanner files..."
cp -r "${STAGING_DIR}"/DataBag_* "${RAW_DIR}/rosbag/" 2>/dev/null || true
cp "${STAGING_DIR}"/*.pcd "${PROCESSED_DIR}/maps/" 2>/dev/null || true

# 3. Extract Images, IMU CSV, and Raw Scans via Transient Docker Container
echo -e "\n[2/4] Extracting Data Streams via Docker ($DOCKER_IMAGE)..."

docker run --rm \
  --user $(id -u):$(id -g) \
  --net=host \
  --ipc=host \
  -v "${SEQ_DIR}:/workspace" \
  "$DOCKER_IMAGE" bash -c '
source /opt/ros/noetic/setup.bash

python3 - << "PYEOF"
import os, sys, csv, math
from pathlib import Path
import rosbag

def latlon_to_enu(lat, lon, alt, ref):
    R = 6371000.0
    east  = R * math.radians(lon - ref[1]) * math.cos(math.radians(ref[0]))
    north = R * math.radians(lat - ref[0])
    up    = alt - ref[2]
    return east, north, up

def timestamp_str(ts):
    return "{:.9f}".format(ts.to_sec())

def save_image(msg, folder, ts):
    ext = "jpg" if "png" not in getattr(msg, "format", "").lower() else "png"
    with open(folder / "{}.{}".format(timestamp_str(ts), ext), "wb") as f:
        f.write(msg.data)

def save_pcd(msg, folder, ts):
    if not hasattr(msg, "points") or not msg.points:
        return
    points = msg.points
    num_points = len(points)
    with open(folder / "{}.pcd".format(timestamp_str(ts)), "w") as f:
        f.write("# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH {}\nHEIGHT 1\nPOINTS {}\nDATA ascii\n".format(num_points, num_points))
        for p in points:
            intensity = getattr(p, "reflectivity", getattr(p, "intensity", 0))
            f.write("{:.6f} {:.6f} {:.6f} {:.1f}\n".format(p.x, p.y, p.z, intensity))

base_dir = Path("/workspace")
raw_dir = base_dir / "raw"
traj_dir = base_dir / "processed/trajectory"
lidar_dir = raw_dir / "lidar-clouds"
bags = sorted(list(raw_dir.glob("rosbag/**/*.bag")))

if not bags:
    print("[SKIP] No .bag files found in /workspace/raw/rosbag")
    sys.exit(0)

cam_map = {
    "/front_camera/image/compressed": raw_dir / "cam0",
    "/left_camera/image/compressed":  raw_dir / "cam1",
    "/right_camera/image/compressed": raw_dir / "cam2",
}

imu_file = open(raw_dir / "imu.csv", "w", newline="")
gps_file = open(traj_dir / "gt-tum.txt", "w")
imu_writer = csv.writer(imu_file)
imu_writer.writerow(["timestamp", "angular_velocity_x", "angular_velocity_y", "angular_velocity_z", "linear_acceleration_x", "linear_acceleration_y", "linear_acceleration_z", "orientation_x", "orientation_y", "orientation_z", "orientation_w"])
gps_file.write("# timestamp tx ty tz qx qy qz qw\n")

gps_ref = None
has_gps = False
total_bags = len(bags)

print("-> Located {} ROS 1 bag chunk(s) for extraction.".format(total_bags))

for idx, bag_path in enumerate(bags, start=1):
    pct_done = (idx / float(total_bags)) * 100.0
    print("\n==================================================")
    print(" [CHUNK {}/{}] ({:.1f}% Processed) Reading: {}".format(idx, total_bags, pct_done, bag_path.name))
    print("==================================================")
    
    counters = {"cam0": 0, "cam1": 0, "cam2": 0, "imu": 0, "lidar": 0, "gnss": 0}
    try:
        with rosbag.Bag(str(bag_path), "r") as bag:
            for topic, msg, ts in bag.read_messages():
                if topic in cam_map:
                    save_image(msg, cam_map[topic], ts)
                    counters[cam_map[topic].name] += 1
                elif topic == "/lidar":
                    save_pcd(msg, lidar_dir, ts)
                    counters["lidar"] += 1
                elif topic == "/imu":
                    imu_writer.writerow([
                        timestamp_str(ts),
                        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
                        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
                        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
                    ])
                    counters["imu"] += 1
                elif topic == "/handsfree/rtk/gnss":
                    lat, lon, alt = msg.latitude, msg.longitude, msg.altitude
                    if gps_ref is None and lat != 0:
                        gps_ref = (lat, lon, alt)
                    if gps_ref is not None:
                        e, n, u = latlon_to_enu(lat, lon, alt, gps_ref)
                        gps_file.write("{} {:.6f} {:.6f} {:.6f} 0.0 0.0 0.0 1.0\n".format(timestamp_str(ts), e, n, u))
                        has_gps = True
                        counters["gnss"] += 1

        print(" Extracted records from chunk:")
        print("   - [CAM0] Front Camera  : {} frames".format(counters["cam0"]))
        print("   - [CAM1] Left Camera   : {} frames".format(counters["cam1"]))
        print("   - [CAM2] Right Camera  : {} frames".format(counters["cam2"]))
        print("   - [LIDAR] Point Clouds : {} scans".format(counters["lidar"]))
        print("   - [IMU] Telemetry      : {} rows".format(counters["imu"]))
        print("   - [GNSS] RTK Poses     : {} poses".format(counters["gnss"]))

    except Exception as e:
        print(" [Error] Failed reading {}: {}".format(bag_path.name, e))

imu_file.close()
gps_file.close()
if not has_gps:
    (traj_dir / "gt-tum.txt").unlink(missing_ok=True)

print("\n-> Extraction completed.")
PYEOF
'

# 4. Batch Convert Bags to ROS 2 (Folder named without .db3 suffix)
echo -e "\n[3/4] Converting ROS 1 bags to ROS 2..."
CONVERT_BIN="rosbags-convert"
if [ -f "$HOME/ros_tools_venv/bin/rosbags-convert" ]; then
  CONVERT_BIN="$HOME/ros_tools_venv/bin/rosbags-convert"
fi

bag_list=($(find "${RAW_DIR}/rosbag" -name "*.bag" | sort))
total_convert=${#bag_list[@]}
current_idx=0

for bag_file in "${bag_list[@]}"; do
  current_idx=$((current_idx + 1))
  pct=$(( (current_idx * 100) / total_convert ))
  bag_name=$(basename "$bag_file" .bag)
  dst_folder="${RAW_DIR}/ros2bag/${bag_name}"
  
  if [ ! -d "$dst_folder" ]; then
    echo "  [${current_idx}/${total_convert}] (${pct}%) Converting: $bag_name.bag -> ${dst_folder}"
    $CONVERT_BIN --src "$bag_file" --dst "$dst_folder"
  fi
done

# 5. Parse External RTK NMEA Logs (if present)
echo -e "\n[4/4] Parsing external RTK NMEA logs..."
find "${STAGING_DIR}" -name "gngga_data_*.txt" | while read -r rtk_file; do
  python3 - << RTKEOF
import math
from pathlib import Path

def latlon_to_enu(lat, lon, alt, ref):
    R = 6371000.0
    east  = R * math.radians(lon - ref[1]) * math.cos(math.radians(ref[0]))
    north = R * math.radians(lat - ref[0])
    up    = alt - ref[2]
    return east, north, up

rtk_path = Path("$rtk_file")
out_path = Path("${PROCESSED_DIR}/trajectory") / "rtk-raw-tum.txt"

ref = None
with open(rtk_path, "r") as fin, open(out_path, "w") as fout:
    fout.write("# timestamp tx ty tz qx qy qz qw\n")
    for line in fin:
        parts = line.strip().split(",")
        if len(parts) >= 10 and parts[0] in ("$GNGGA", "$GPGGA") and parts[2] and parts[4]:
            try:
                lat_raw, lat_dir = float(parts[2]), parts[3]
                lon_raw, lon_dir = float(parts[4]), parts[5]
                alt = float(parts[9]) if parts[9] else 0.0
                lat = int(lat_raw / 100) + (lat_raw % 100) / 60.0
                if lat_dir == "S": lat = -lat
                lon = int(lon_raw / 100) + (lon_raw % 100) / 60.0
                if lon_dir == "W": lon = -lon
                ts = float(parts[1]) if parts[1] else 0.0
                if ref is None: ref = (lat, lon, alt)
                e, n, u = latlon_to_enu(lat, lon, alt, ref)
                fout.write("{:.3f} {:.6f} {:.6f} {:.6f} 0.0 0.0 0.0 1.0\n".format(ts, e, n, u))
            except ValueError:
                continue
RTKEOF
done

# 6. Ownership & permissions safeguard (No sudo prompt required)
chmod -R u+rwX "$SEQ_DIR" 2>/dev/null || true

echo -e "\n=================================================================="
echo "  INGESTION COMPLETE (100%)"
echo "  Output Directory : $SEQ_DIR"
echo "  Raw PCD Scans    : ${RAW_DIR}/lidar-clouds/"
echo "  Untouched Map    : ${PROCESSED_DIR}/maps/"
echo "=================================================================="
