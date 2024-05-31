source /opt/ros/humble/setup.bash
ros2 run usb_cam usb_cam_node_exe

# in different terminal:
source /opt/ros/humble/setup.bash
ros2 run image_view image_view image:=/image_raw


# or:
source /opt/ros/humble/setup.bash
ros2 run camera_calibration cameracalibrator --size 9x6 --square 0.053 --ros-args -r image:=/image_raw -r camera:=/camera
