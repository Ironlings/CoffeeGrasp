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
export LC_NUMERIC=en_US.UTF-8

# === 3. 并行启动其他终端（无需等待）===

gnome-terminal --title="ORBBEC Camera" -- bash -c "
    ros2 launch orbbec_camera dabai.launch.py;
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



echo "🚀 所有 ROS 节点已启动！"
