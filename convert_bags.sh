#!/usr/bin/env bash
set -e

# 1. Validate CLI argument
if [ -z "$1" ]; then
  echo "Usage: $0 <path_to_ros1_bag_folder>"
  exit 1
fi

SRC_DIR="${1%/}"

if [ ! -d "$SRC_DIR" ]; then
  echo "Error: Directory '$SRC_DIR' does not exist."
  exit 1
fi

# 2. Extract base raw directory
if [[ "$SRC_DIR" == *"/rosbag"* ]]; then
  RAW_DIR="${SRC_DIR%%/rosbag*}"
else
  RAW_DIR="$(dirname "$SRC_DIR")"
fi

DST_DIR="${RAW_DIR}/ros2bag"
mkdir -p "$DST_DIR"

# 3. Virtual environment check
if [ -f "$HOME/ros_tools_venv/bin/activate" ]; then
  # shellcheck source=/dev/null
  source "$HOME/ros_tools_venv/bin/activate"
fi

if ! command -v rosbags-convert &> /dev/null; then
  echo "Error: 'rosbags-convert' command not found."
  exit 1
fi

echo "=================================================="
echo "Source (.bag files) : $SRC_DIR"
echo "Output Directory    : $DST_DIR"
echo "=================================================="

# 4. Batch convert (folder named $filename without .db3 suffix)
bag_count=0
for file in "$SRC_DIR"/*.bag; do
  [ -e "$file" ] || continue
  
  filename=$(basename "$file" .bag)
  dst_folder="${DST_DIR}/${filename}"
  
  if [ ! -d "$dst_folder" ]; then
    echo "Converting: $filename.bag -> $dst_folder"
    rosbags-convert --src "$file" --dst "$dst_folder"
    bag_count=$((bag_count + 1))
  else
    echo "Skipping existing: $dst_folder"
  fi
done

if [ "$bag_count" -eq 0 ]; then
  echo "No new .bag files to convert in '$SRC_DIR'."
else
  echo "=================================================="
  echo "Done! Converted $bag_count bag(s) directly to: $DST_DIR"
  echo "=================================================="
fi