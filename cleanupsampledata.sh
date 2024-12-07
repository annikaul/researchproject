#!/bin/bash

if [ "$#" -ne 1 ]; then
  echo "Usage: $0 <directory>; directory e.g. '00000075"
  exit 1
fi

BASE_DIR="noetic-slam/sampledata/raw/$1"
LIDAR_DIR="$BASE_DIR/lidar_00000000"
CAM_DIR="$BASE_DIR/cam_00000000"

if [ ! -d "$LIDAR_DIR" ] || [ ! -d "$CAM_DIR" ]; then
  echo "Directory lidar_00000000 or cam_00000000 does not exist."
  exit 1
fi

# Delete all files in lidar folder that do not exist in cam directory
for LIDAR_SUBDIR in "$LIDAR_DIR"/*; do
  if [ -d "$LIDAR_SUBDIR" ]; then
    SUBDIR_NAME=$(basename "$LIDAR_SUBDIR")
    if [ ! -d "$CAM_DIR/$SUBDIR_NAME" ]; then
      echo "Delete folder: $LIDAR_SUBDIR"
      rm -rf "$LIDAR_SUBDIR"
    fi
  fi
done


# Delete all files in cam folder that do not exist in lidar directory
for CAM_SUBDIR in "$CAM_DIR"/*; do
  if [ -d "$CAM_SUBDIR" ]; then
    SUBDIR_NAME=$(basename "$CAM_SUBDIR")
    if [ ! -d "$LIDAR_DIR/$SUBDIR_NAME" ]; then
      echo "Delete folder: $CAM_SUBDIR"
      rm -rf "$CAM_SUBDIR"
    fi
  fi
done

