# Wantage Scan Project (`wantage_scan`)

## Overview
This repository manages 3D spatial scanning datasets collected during the **Wantage Scan** survey campaign. It enforces a standardized, multi-sequence architecture designed to cleanly separate immutable raw field telemetry from processed spatial deliverables (point clouds, meshes, and GIS assets).

---

## Project Repository Architecture

Each sequence capture is cataloged under `sequences/<sequence_name>/` using the canonical Oxford Spires structure:

```text
wantage_scan/
├── README.md                          <-- Project documentation & dataset specifications
├── build_dataset_sequence.sh          <-- Master ingestion, extraction & conversion script
└── sequences/                         <-- Catalog of survey collections
    └── <YYYY-MM-DD-HH-MM-SS-name>/    <-- Sequence Directory (e.g., 2026-07-29-20-27-04-walk3-park)
        ├── raw/                       <-- Unmodified raw sensor telemetry
        │   ├── rosbag/                <-- Original ROS 1 recording (DataBag_*.bag)
        │   ├── ros2bag/               <-- Converted ROS 2 storage databases (*.db3 + metadata.yaml)
        │   ├── cam0/                  <-- Extracted front RGB camera frames (<timestamp>.jpg)
        │   ├── cam1/                  <-- Extracted left RGB camera frames (<timestamp>.jpg)
        │   ├── cam2/                  <-- Extracted right RGB camera frames (<timestamp>.jpg)
        │   ├── lidar-clouds/          <-- Extracted individual LiDAR point clouds (<timestamp>.pcd)
        │   └── imu.csv                <-- Synchronized 200 Hz 6-DOF IMU telemetry table
        └── processed/                 <-- Downstream SLAM & mapping deliverables
            ├── maps/                  <-- Global stitched factory & SLAM point clouds (geomap_*.pcd)
            └── trajectory/            <-- Ground-truth trajectories in TUM format
                ├── gt-tum.txt         <-- ENU Cartesian trajectory from /handsfree/rtk/gnss
                └── rtk-raw-tum.txt    <-- Cartesian trajectory from external GNGGA NMEA logs