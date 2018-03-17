#!/bin/bash

rosparam set use_sim_time false

if [ -z "$1" ]
then
	out_file="rosbag_test"
else
	out_file=$1
fi

rosbag record -b 512 -o $out_file \
/left_rear_camera/camera_info \
/left_rear_camera/image_color/compressed \
/left_rear_camera/image_mono/compressed \
/left_rear_camera/image_raw/compressed \
/right_rear_camera/camera_info \
/right_rear_camera/image_color/compressed \
/right_rear_camera/image_mono/compressed \
/right_rear_camera/image_raw/compressed \
/sibot/camera_info \
/sibot/depth/compressedDepth \
/sibot/feedback/battery \
/sibot/feedback/configuration \
/sibot/feedback/dynamics \
/sibot/feedback/gps/fix_2d \
/sibot/feedback/gps/fix_3d \
/sibot/feedback/imu \
/sibot/feedback/mag_feild \
/sibot/feedback/propulsion \
/sibot/feedback/status \
/sibot/feedback/twist \
/sibot/filtered_lidar_points2 \
/sibot/filtered_sick_points2 \
/sibot/filtered_sick_scan \
/sibot/ground_object_cloud \
/sibot/histogram \
/sibot/image_points2 \
/sibot/image_points2_color \
/sibot/imu/accelerometer \
/sibot/imu/gyroscope \
/sibot/imu/magnetometer \
/sibot/joint_states \
/sibot/left/camera_info \
/sibot/left/cost/camera_info \
/sibot/left/cost/compressed \
/sibot/left/disparity/camera_info \
/sibot/left/disparity/compressed \
/sibot/left/disparity/compressedDepth \
/sibot/left/image_color/camera_info \
/sibot/left/image_color/compressed \
/sibot/left/image_mono/camera_info \
/sibot/left/image_mono/compressed \
/sibot/left/image_rect/camera_info \
/sibot/left/image_rect/compressed \
/sibot/left/image_rect_color/camera_info \
/sibot/left/image_rect_color/compressed \
/sibot/lidar_points2 \
/sibot/lidar_scan \
/sibot/odometry/local_filtered \
/sibot/organized_image_points2 \
/sibot/organized_image_points2_color \
/sibot/pps \
/sibot/right/camera_info \
/sibot/right/disparity/camera_info \
/sibot/right/disparity/compressed \
/sibot/right/image_mono/camera_info \
/sibot/right/image_mono/compressed \
/sibot/right/image_rect/camera_info \
/sibot/right/image_rect/compressed \
/sibot/sick_scan \
/sibot/stamped_pps \
/tf
