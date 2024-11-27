#!/bin/bash

source /opt/vulcanexus/iron/setup.bash
cd /home/cervera5R/arise-vulcanexus-enviroment/arise_ws
source install/setup.bash

gnome-terminal -- bash -c "ros2 run fruit_picking detect_fruits; exec bash"
gnome-terminal -- bash -c "ros2 run fruit_picking robot_pick; exec bash"
gnome-terminal -- bash -c "ros2 run battery_disassembly audio_interface; exec bash"
gnome-terminal -- bash -c "ros2 run fruit_picking intel_transforms; exec bash"
gnome-terminal -- bash -c "ros2 run fruit_picking fruits_manager; exec bash"
gnome-terminal -- bash -c "ros2 launch realsense2_camera rs_launch.py camera_namespace:=d435 depth_module.profile:=640x480x30 rgb_camera.profile:=640x480x30 enable_sync:=true align_depth.enable:=true; exec bash"
gnome-terminal -- bash -c "ros2 run fruit_picking arise_ai; exec bash"