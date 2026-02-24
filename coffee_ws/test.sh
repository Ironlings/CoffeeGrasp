#!/bin/bash

# === 4. 启动视觉与控制节点 ===
gnome-terminal --title="Elevator Vision" -- bash -c "
    source /home/agv/setup_arm_env.sh;
    ros2 launch elevator_vision elevator_yolo.launch.py params_file:=/home/agv/dev_ws/install/elevator_vision/share/elevator_vision/config/params_floor5_down.yaml;
    exec bash
"
