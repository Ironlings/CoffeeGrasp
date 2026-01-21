# 项目名称

> 基于 RGB-D 点云与 SAM3 语义分割的咖啡袋位姿估计与机械臂抓取系统。

---

## 📦 安装依赖

本项目依赖以下外部仓库，请确保在构建前正确克隆或安装它们：

### 1. 主要依赖仓库

| 仓库 | 说明 |
|------|------|
| [`agilexrobotics/piper_sdk`](https://github.com/agilexrobotics/piper_sdk) | Piper 机械臂的底层 CAN 通信 SDK |
| [`orbbec/OrbbecSDK_ROS2`](https://github.com/orbbec/OrbbecSDK_ROS2/tree/main) | Orbbec 深度相机的 ROS 2 驱动 |
| [`orbbec/OrbbecSDK`](https://github.com/orbbec/OrbbecSDK) | Orbbec 深度相机的底层 SDK |
| [`moveit/moveit2`](https://github.com/moveit/moveit2) | ROS 2 下的通用机械臂运动规划、碰撞检测与执行框架 |
| [`binb1nwu/trac_ik`](https://github.com/binb1nwu/trac_ik) | TRAC-IK 的 ROS 2 移植版，逆运动学（IK）求解器 |
| [`binb1nwu/nlopt`](https://github.com/binb1nwu/nlopt) | 修改了安装前缀的 NLopt 非线性优化库 |

### 2. 
```bash
git clone --recurse-submodules https://github.com/Ironlings/CoffeeGrasp.git
```

