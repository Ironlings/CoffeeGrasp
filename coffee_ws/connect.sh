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

gnome-terminal --title="ORBBEC Camera 335L" -- bash -c "
    ros2 launch orbbec_camera gemini_330_series.launch.py usb_port:=2-8 color_width:=1280 color_height:=800 color_fps:=30 depth_width:=1280 depth_height:=800 depth_fps:=30;
    exec bash
"


echo "🚀 所有 ROS 节点已启动！"
