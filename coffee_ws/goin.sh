#!/bin/bash

# === 2. Source 所有 ROS 工作空间 ===
source /opt/ros/humble/setup.bash
source /home/agv/armws/moveit_ws/install/setup.bash
source /home/agv/armws/piper_ros/install/setup.bash
source /home/agv/armws/orbbec_ws/install/setup.bash
source /home/agv/armws/trac_ws/install/setup.bash
source /home/agv/armws/coffee_ws/install/setup.bash
source /home/agv/ranger_mini_v3_ws/install/setup.sh
source /home/agv/ws_livox/install/setup.bash
export LC_NUMERIC=en_US.UTF-8

# === 3. 并行启动其他终端（无需等待）===

gnome-terminal --title="ORBBEC Camera 335L" -- bash -c "
    ros2 launch orbbec_camera gemini_330_series.launch.py usb_port:=2-8 color_width:=1280 color_height:=800 color_fps:=30 depth_width:=1280 depth_height:=800 depth_fps:=30;
    exec bash
"
gnome-terminal --title="CAN & Ranger" -- bash -c "
  sudo ip link set can_agv down 2>/dev/null;
  sudo ip link set can_agv type can bitrate 500000;
  sudo ip link set can_agv up;
  sudo ip link show can_agv;
  ros2 launch ranger_bringup ranger_mini_v3.launch.py;
  exec bash
"

gnome-terminal --title="mid360" -- bash -c "
    ros2 launch livox_ros_driver2 rviz_MID360_launch.py;
    exec bash
"

gnome-terminal --title="obstacle avoid" -- bash -c "
    ros2 launch coffee_detect avoid.launch.py;
    exec bash
"

gnome-terminal --title="go in lift" -- bash -c "
    ros2 launch coffee_detect goinlift.launch.py;
    exec bash
"
  
echo "🚀 所有 ROS 节点已启动！"
