#!/bin/bash

CAN_IFACE="can_piper"
TARGET_BITRATE="1000000"

# === 1. 检测 can0 是否已激活且配置正确 ===
echo "🔍 检查 $CAN_IFACE 状态..."

# 检查接口是否存在且状态为 UP
if ip link show "$CAN_IFACE" &>/dev/null; then
    # 获取当前状态（UP/DOWN）
    IFACE_STATE=$(ip link show "$CAN_IFACE" | awk 'NR==1{print $9}')
    # 获取当前比特率（bitrate）
    CURRENT_BITRATE=$(ip -details link show "$CAN_IFACE" | grep -o 'bitrate [0-9]*' | awk '{print $2}')

    if [[ "$IFACE_STATE" == "UP" ]] && [[ "$CURRENT_BITRATE" == "$TARGET_BITRATE" ]]; then
        echo "✅ $CAN_IFACE 已激活且比特率为 ${TARGET_BITRATE}，跳过配置。"
        skip_can_setup=true
    else
        echo "⚠️ $CAN_IFACE 已存在但状态或比特率不符（当前状态: $IFACE_STATE, 比特率: $CURRENT_BITRATE），将重新配置。"
        skip_can_setup=false
    fi
else
    echo "⚠️ $CAN_IFACE 未找到，将进行配置。"
    skip_can_setup=false
fi

# === 2. 如果未激活或配置不正确，则执行配置脚本 ===
if [ "$skip_can_setup" = false ]; then
    echo "🔧 配置 CAN 接口 ($CAN_IFACE @ ${TARGET_BITRATE}bps)..."
    sudo bash /home/agv/armws/piper_ros/can_activate.sh "$CAN_IFACE" "$TARGET_BITRATE" "1-1:1.0"

    if [ $? -ne 0 ]; then
        echo "❌ CAN 配置失败，退出。"
        exit 1
    fi
    echo "✅ CAN 配置成功。"
fi

# === 2. Source 所有 ROS 工作空间 ===
source /opt/ros/humble/setup.bash
source /home/agv/armws/moveit_ws/install/setup.bash
source /home/agv/armws/piper_ros/install/setup.bash
source /home/agv/armws/orbbec_ws/install/setup.bash
source /home/agv/armws/trac_ws/install/setup.bash
source /home/agv/armws/coffee_ws/install/setup.bash
source /home/agv/wzb/install/setup.bash
export LC_NUMERIC=en_US.UTF-8

# === 3. 并行启动其他终端（无需等待）===
gnome-terminal --title="ORBBEC Camera 335L" -- bash -c "
    ros2 launch orbbec_camera gemini_330_series.launch.py usb_port:=2-8 color_width:=1280 color_height:=800 color_fps:=30 depth_width:=1280 depth_height:=800 depth_fps:=30;
    exec bash
"

gnome-terminal --title="ORBBEC Camera dabai" -- bash -c "
    ros2 launch orbbec_camera dabai.launch.py;
    exec bash
"

gnome-terminal --title="Yolo" -- bash -c "
    source ~/anaconda3/bin/activate yolo &&
    /home/agv/anaconda3/envs/yolo/bin/python /home/agv/wzb/src/image_saver/image_saver/detect_node.py
    exec bash
"

gnome-terminal --title="Piper Arm" -- bash -c "
    ros2 launch piper start_single_piper.launch.py can_port:=can_piper gripper_val_mutiple:=2;
    exec bash
"

gnome-terminal --title="MoveIt" -- bash -c "
    ros2 launch piper_with_gripper_moveit mydemo.launch.py;
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
gnome-terminal --title="pose for arm" -- bash -c "
    source /opt/ros/humble/setup.bash;
    sleep 5;
    ros2 topic pub --once /my_pose_cmd geometry_msgs/msg/PoseStamped \"{header: {frame_id: 'base_link'}, pose: {position: {x: -0.13984, y: -0.0006, z: 0.436492}, orientation: {x: 0.0071732650039771015, y: 0.637345889369954, z: 0.003499715255773396, w: 0.7705365102093046}}}\";
    echo 'Command finished. Terminal will stay open.';
    exec bash
"

gnome-terminal --title="Panel Align" -- bash -c "
    source ~/anaconda3/bin/activate yolo &&
    ros2 launch coffee_detect m2p.launch.py;
    exec bash
"

gnome-terminal --title="go in lift" -- bash -c "
    ros2 launch coffee_detect goinlift.launch.py;
    exec bash
"
  
echo "🚀 所有 ROS 节点已启动！"
