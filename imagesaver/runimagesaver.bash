#!/bin/bash

source /opt/ros/humble/setup.bash

echo "installing usb-cam"
sudo apt-get update
sudo apt-get install -y ros-humble-usb-cam

echo "running usb_cam node"
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video4 & # change video device accordingly

sleep 1

echo "building package image_saver"
colcon build
source install/setup.bash
echo "running image_saver node"
ros2 run image_saver image_saver_node