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
    position: {x: 0.411493, y: 0.066035, z: 0.260238},
    orientation: {x: -0.7327915627259012, y: 0.6699915303950059, z: -0.09028338887219962, w: 0.0773096662160723}
  }
}"

# 等待 10 秒
sleep 10

ros2 topic pub --once /my_gripper_cmd std_msgs/msg/Float64 "data: 0.035"

# 等待 5 秒
sleep 5

ros2 topic pub --once /my_pose_cmd geometry_msgs/msg/PoseStamped "{
  header: {
    stamp: {sec: 0, nanosec: 0},
    frame_id: 'base_link'
  },
  pose: {
    position: {x: 0.046216, y: 0.002615, z: 0.478671},
    orientation: {x: -0.0030402966631021544, y: 0.6838194050006842, z: 0.02950326107270329, w: 0.7290482395059921}
  }
}"
