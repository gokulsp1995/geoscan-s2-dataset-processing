#!/bin/bash

# Arg 1: Target directory (defaults to current directory ".")
TARGET_DIR="${1:-.}"

# Arg 2: Folder matching pattern (defaults to "*", processing all subfolders)
PATTERN="${2:-*}"

# Navigate to target directory
cd "$TARGET_DIR" || { echo "[ERROR] Directory $TARGET_DIR not found"; exit 1; }

echo "=========================================================="
echo " STARTING MASTER EXTRACTION"
echo " Target Directory: $(pwd)"
echo " Matching Pattern: $PATTERN"
echo "=========================================================="

for dir in $PATTERN; do
    # Only iterate over actual directories
    [ -d "$dir" ] || continue

    echo ""
    echo "**********************************************************"
    echo " RUNNING BATCH FOR SEQUENCE: $dir"
    echo "**********************************************************"
    
    # Calls the generalized Python script
    python3 /workspace/wantage_scan/extract_rosbag.py "$dir"
done

echo ""
echo "=========================================================="
echo " BATCH EXTRACTION COMPLETE "
echo "=========================================================="