
## Project Overview

[简体中文](README.md) | English

![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange.svg)
![ROS](https://img.shields.io/badge/ROS-humble-blue.svg)
![Python](https://img.shields.io/badge/Python-3.10-3776ab.svg)

> **Coffee Bag Pose Estimation and Robotic Grasping System Based on RGB-D Point Clouds and SAM³ Semantic Segmentation**

![Mask](coffee_ws/src/coffee_detect/coffeetest/img/mask.jpg)  
*Mask image: coffee bag region (in blue) extracted via semantic segmentation.*

![Depth Image](coffee_ws/src/coffee_detect/coffeetest/img/depth.png)  
*Raw depth map: captured by an Orbbec Dabai camera, used for 3D point cloud reconstruction.*

![Point Cloud](coffee_ws/src/coffee_detect/coffeetest/img/pc.png)  
*3D point cloud: generated from the aligned RGB-D data using the segmentation mask, used for pose estimation and grasp planning.*

![Grasp Demo](coffee_ws/src/coffee_detect/coffeetest/img/VID_20260122_161541.gif)  
*Grasping demonstration (2× speed): the robotic arm autonomously approaches, grasps, and lifts the detected coffee bag based on its estimated pose.*

---

## 📦 Installation

### 1. Install Dependencies

This project relies on the following external repositories. Please ensure they are cloned and installed before building:

| Repository | Description |
|------------|-------------|
| [`agilexrobotics/piper_sdk`](https://github.com/agilexrobotics/piper_sdk) | Low-level CAN communication SDK for the Piper robotic arm |
| [`orbbec/OrbbecSDK_ROS2`](https://github.com/orbbec/OrbbecSDK_ROS2/tree/main) | ROS 2 driver for Orbbec depth cameras |
| [`orbbec/OrbbecSDK`](https://github.com/orbbec/OrbbecSDK) | Core SDK for Orbbec depth cameras |
| [`moveit/moveit2`](https://github.com/moveit/moveit2) | General-purpose motion planning framework for ROS 2 |
| [`binb1nwu/trac_ik`](https://github.com/binb1nwu/trac_ik) | ROS 2 port of the TRAC-IK inverse kinematics solver |
| [`binb1nwu/nlopt`](https://github.com/binb1nwu/nlopt) | NLopt nonlinear optimization library with modified CMake install prefix |

### 2. Install This Project

Clone the repository along with its submodules. The submodule is based on [`agilexrobotics/piper_ros`](https://github.com/agilexrobotics/piper_ros/tree/humble) and extended with `moveit_py` support for arm control:

```bash
git clone --recurse-submodules https://github.com/Ironlings/CoffeeGrasp.git
```

Then build both the main workspace (`coffee_ws`) and the `piper_ros` submodule separately.

### 3. Configure SAM 3

To avoid interference with your ROS 2 environment, we recommend using a dedicated Conda environment:

```bash
conda create -n sam python=3.10
```

SAM 3 is available in Ultralytics v8.3.237 or later. Install or upgrade it via:

```bash
pip install -U ultralytics
```

Download the [SAM 3 model weights](https://huggingface.co/facebook/sam3) and specify the path to the checkpoint in the configuration file:  
`coffee_ws/src/config/params.yaml`.

---

## Running the System

### 1. Activate ROS 2 Environment

Ensure all required ROS 2 environments are sourced, including:
- `/opt/ros/humble/setup.bash`
- Your custom workspace’s `install/setup.bash` (e.g., from `coffee_ws` and `piper_ros`)

### 2. Launch Nodes

Run the following commands in separate terminals to start perception, robotic arm control, and grasping modules:

```bash
# Launch Orbbec Dabai camera driver (with depth-to-color registration enabled)
ros2 launch orbbec_camera dabai.launch.py depth_registration:=true

# Launch Piper robotic arm (specify CAN interface and gripper scaling factor)
ros2 launch piper start_single_piper.launch.py can_port:=can_piper gripper_val_multiple:=2

# Launch MoveIt! for motion planning and end-effector pose control
ros2 launch piper_with_gripper_moveit mydemo.launch.py
```

Then, in a new terminal, activate the `sam` Conda environment and run the coffee detection and grasping node:

```bash
conda activate sam
ros2 launch coffee_detect coffee.launch.py
```

> **Tip**: Run each command in a separate terminal window for easier log monitoring and debugging.

--- 