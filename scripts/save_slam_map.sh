#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/foxy/setup.bash
source /home/pi/cartographer_ws2/install/setup.bash
export ROS_DOMAIN_ID=16 ROS_LOCALHOST_ONLY=0

mkdir -p /home/pi/gladis_maps
BASE="${1:-/home/pi/gladis_maps/beep_slam_$(date +%Y%m%d_%H%M%S)}"
sleep 3
ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"
sleep 2
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: \"${BASE}.pbstream\"}"
ros2 run cartographer_ros pbstream_to_ros_map_node \
  -map_filestem="${BASE}" \
  -pbstream_filename="${BASE}.pbstream" \
  -resolution=0.05
printf '%s\n' "$BASE" > /home/pi/gladis_maps/current_base
printf 'SAVED_BASE=%s\n' "$BASE"
