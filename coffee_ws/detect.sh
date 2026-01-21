#!/bin/bash

# === 2. Source 所有 ROS 工作空间 ===
source /opt/ros/humble/setup.bash
source /home/agv/armws/moveit_ws/install/setup.bash
source /home/agv/armws/piper_ros/install/setup.bash
source /home/agv/armws/orbbec_ws/install/setup.bash
source /home/agv/armws/trac_ws/install/setup.bash
source /home/agv/armws/coffee_ws/install/setup.bash
export LC_NUMERIC=en_US.UTF-8

# === 3. 并行启动其他终端（无需等待）===


gnome-terminal --title="Coffee Detect" -- bash -c "
    source ~/anaconda3/bin/activate yolo &&
    ros2 launch coffee_detect coffee.launch.py;
    exec bash
"

