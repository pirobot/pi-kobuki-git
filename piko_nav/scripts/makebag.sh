#!/bin/bash

rosparam set use_sim_time false

if [ -z "$1" ]
then
	out_file="rosbag_test"
else
	out_file=$1
fi

rosrun dynamic_reconfigure dynparam set /camera/driver data_skip 3

rosbag record -b 512 -o $out_file \
tf \
scan \
scan_filtered \
odom \
camera/depth_registered/image_raw/compressedDepth \
camera/depth_registered/camera_info \
camera/rgb/image_rect_color/compressed \
camera/rgb/camera_info
