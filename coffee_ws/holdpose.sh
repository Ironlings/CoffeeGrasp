#!/bin/bash


# === 2. Source 所有 ROS 工作空间 ===
source /opt/ros/humble/setup.bash
source /home/agv/armws/moveit_ws/install/setup.bash
source /home/agv/armws/piper_ros/install/setup.bash
source /home/agv/armws/orbbec_ws/install/setup.bash
source /home/agv/armws/trac_ws/install/setup.bash
source /home/agv/armws/coffee_ws/install/setup.bash
export LC_NUMERIC=en_US.UTF-8

ros2 topic pub --once /my_pose_cmd geometry_msgs/msg/PoseStamped "{
  header: {
    stamp: {sec: 0, nanosec: 0},
    frame_id: 'base_link'
  },
  pose: {
    position: {x: -0.019305, y: -0.007057, z: 0.522798},
    orientation: {x: -0.4950316297280355, y: 0.44675858673883606, z: -0.5376808178489426, w: 0.5159939814195661}
  }
}"
