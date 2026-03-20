#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from message_filters import Subscriber, ApproximateTimeSynchronizer
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Twist  # 新增Twist导入
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import cv2
import numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation as R_scipy
import warnings
import time
import sys

warnings.filterwarnings("ignore")


class CoffeeDropNode(Node):
    def __init__(self):
        super().__init__('coffee_drop')
        
        # ===== 声明所有可配置参数 =====
        self.declare_parameter('marker_size_mm', 48.3)
        self.declare_parameter('aruco_dict_type', '6X6_250')
        self.declare_parameter('gemini_fx', 612.593017578125)
        self.declare_parameter('gemini_fy', 612.551513671875)
        self.declare_parameter('gemini_cx', 639.3008422851562)
        self.declare_parameter('gemini_cy', 405.6265869140625)
        self.declare_parameter('required_markers', [0, 1])
        
        # 任务关键参数（全部可配置！）
        self.declare_parameter('basket_offset_m', -0.08)      # 向标记深处偏移/篮子宽度（米）
        self.declare_parameter('height_offset_m', 0.45)       # Z轴高度补偿/袋子高度（米）
        self.declare_parameter('approach_offset_m', 0.08)     # 接近偏移量/末端执行器长度（米）
        self.declare_parameter('gripper_open_gap', 0.035)     # 夹爪开距半宽（米）
        self.declare_parameter('max_processing_retries', 5)   # 最大重试次数
        self.declare_parameter('retry_interval_sec', 0.5)     # 重试间隔（秒）
        self.declare_parameter('enable_visualization', False) # 是否显示检测结果
        
        # 手眼标定矩阵（4x4 齐次变换）
        self.declare_parameter('hand_eye_matrix', [
            0.0, 0.0, 1.0, 0.14,
            -1.0, 0.0, 0.0, 0.0,
            0.0, -1.0, 0.0, -0.25,
            0.0, 0.0, 0.0, 1.0
        ])
        
        # ===== 加载参数 =====
        self.marker_size_mm = self.get_parameter('marker_size_mm').value
        self.aruco_dict_type = self.get_parameter('aruco_dict_type').value
        self.camera_matrix = np.array([
            [self.get_parameter('gemini_fx').value, 0, self.get_parameter('gemini_cx').value],
            [0, self.get_parameter('gemini_fy').value, self.get_parameter('gemini_cy').value],
            [0, 0, 1]
        ], dtype=np.float32)
        self.required_markers = set(self.get_parameter('required_markers').value)
        
        # 任务参数
        self.basket_offset_m = self.get_parameter('basket_offset_m').value
        self.height_offset_m = self.get_parameter('height_offset_m').value
        self.approach_offset_m = self.get_parameter('approach_offset_m').value
        self.gripper_open_gap = self.get_parameter('gripper_open_gap').value
        self.max_retries = self.get_parameter('max_processing_retries').value
        self.retry_interval = self.get_parameter('retry_interval_sec').value
        self.enable_viz = self.get_parameter('enable_visualization').value
        
        # 手眼矩阵
        he_vals = self.get_parameter('hand_eye_matrix').value
        self.T_cam_in_arm = np.array(he_vals, dtype=np.float32).reshape(4, 4)

        self.MAX_ADJUST_TIME = 10.0  # 最大调整时间（秒）
        self.ADJUST_INTERVAL = 0.3   # 调整间隔（秒）
        self.VX_ADJUST = 0.12        # x方向调整速度（m/s）
        self.VY_ADJUST = 0.10        # y方向调整速度（m/s）
        self.VZ_ADJUST = 0.30
        self.X_THRESHOLD = 0.48      # x阈值（米）
        self.Y_THRESHOLD = 0.08      # y阈值（米）
        self.YAW_THRESHOLD = np.radians(5)  # 朝向阈值（弧度）
        self.YAW_TARGET = np.radians(-90)    # 目标朝向（弧度）


        # ===== ROS 接口 =====
        self.bridge = CvBridge()
        self.grasp_pub = self.create_publisher(PoseStamped, '/my_pose_cmd', 10)
        self.gripper_pub = self.create_publisher(Float64, '/my_gripper_cmd', 10)
        # 新增cmd_vel发布器（用于小车平移调整）
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 订阅器
        rgb_sub = Subscriber(self, Image, '/camera/color/image_raw')
        depth_sub = Subscriber(self, Image, '/camera/depth/image_raw')
        pose_sub = Subscriber(self, PoseStamped, '/end_pose_stamped')
        
        self.ts = ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub, pose_sub],
            queue_size=10,
            slop=0.1
        )
        self.ts.registerCallback(self.sync_callback)
        
        # 线程安全帧缓存
        self.latest_frame = None
        
        self.get_logger().info("☕ Coffee drop node initialized. Waiting for synchronized frames...")

    def sync_callback(self, rgb_msg, depth_msg, end_pose_msg):
        """轻量级回调：仅转换并缓存最新帧"""
        try:
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            depth = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
            self.latest_frame = (rgb.copy(), depth.copy(), end_pose_msg)
            self.get_logger().debug("New synchronized frame cached")
        except Exception as e:
            self.get_logger().error(f"Frame conversion failed: {str(e)}")

    def detect_markers(self, image):
        """ArUco 检测封装（带可视化选项）"""
        # 字典选择
        dict_map = {
            '4X4_50': cv2.aruco.DICT_4X4_50,
            '5X5_100': cv2.aruco.DICT_5X5_100,
            '6X6_250': cv2.aruco.DICT_6X6_250,
            '7X7_1000': cv2.aruco.DICT_7X7_1000
        }
        if self.aruco_dict_type not in dict_map:
            raise ValueError(f"Unsupported dict: {self.aruco_dict_type}")
        
        aruco_dict = cv2.aruco.getPredefinedDictionary(dict_map[self.aruco_dict_type])
        detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())
        corners, ids, _ = detector.detectMarkers(image)
        
        # 可视化（调试用）
        if self.enable_viz and ids is not None:
            viz_img = image.copy()
            cv2.aruco.drawDetectedMarkers(viz_img, corners, ids)
            cv2.imshow("ArUco Detection", viz_img)
            cv2.waitKey(1)
        
        return corners, ids

    def process_frame(self, rgb, depth):
        """核心处理逻辑：检测标记 → 计算目标位姿 → 坐标变换"""
        try:
            # ===== 1. 检测 ArUco 标记 =====
            corners, ids = self.detect_markers(rgb)
            if ids is None:
                self.get_logger().warn("❌ 未检测到任何 ArUco 标记")
                return None, None
            
            detected_ids = set(ids.flatten().tolist())
            missing = self.required_markers - detected_ids
            if missing:
                self.get_logger().error(f"❌ 缺少必需标记: {missing} (检测到: {detected_ids})")
                return None, None
            
            # ===== 2. 求解每个标记位姿 =====
            marker_size_m = self.marker_size_mm / 1000.0
            obj_points = np.array([
                [-marker_size_m/2,  marker_size_m/2, 0],
                [ marker_size_m/2,  marker_size_m/2, 0],
                [ marker_size_m/2, -marker_size_m/2, 0],
                [-marker_size_m/2, -marker_size_m/2, 0]
            ], dtype=np.float32)
            
            poses = {}
            for i, marker_id in enumerate(ids.flatten()):
                if marker_id not in self.required_markers:
                    continue
                success, rvec, tvec = cv2.solvePnP(
                    obj_points, corners[i], self.camera_matrix, np.zeros(5),
                    flags=cv2.SOLVEPNP_IPPE_SQUARE
                )
                if not success:
                    self.get_logger().warn(f"ID {marker_id} 位姿求解失败")
                    continue
                R_mat, _ = cv2.Rodrigues(rvec)
                poses[i] = {"tvec": tvec.flatten(), "R": R_mat}
                x, y, z = tvec.flatten() * 100
                self.get_logger().debug(f"[ID {marker_id}] 位置 (cm): x={x:.1f}, y={y:.1f}, z={z:.1f}")
            
            if len(poses) < len(self.required_markers):
                self.get_logger().error("❌ 未获得所有必需标记的位姿")
                return None, None
            
            # ===== 3. 计算目标点（篮子中心） =====
            target_points = {}
            for i, mid in enumerate(self.required_markers):
                tvec = poses[i]["tvec"]
                R = poses[i]["R"]
                z_axis = R @ np.array([0, 0, 1])  # 相机Z轴方向
                target_points[i] = tvec + self.basket_offset_m * z_axis  # 向深处偏移
            
            # 中点位置
            p0, p1 = target_points[0], target_points[1]
            midpoint_cam = (p0 + p1) / 2.0
            
            # 平均旋转（四元数球面插值简化版）
            quat0 = R_scipy.from_matrix(poses[0]["R"]).as_quat()
            quat1 = R_scipy.from_matrix(poses[1]["R"]).as_quat()
            quat_avg = (quat0 + quat1) / 2
            quat_avg /= np.linalg.norm(quat_avg)
            R_avg = R_scipy.from_quat(quat_avg).as_matrix()
            
            # 姿态校正：绕Y/Z轴旋转180°（适配篮子朝向）
            r_correction = R_scipy.from_euler('yz', [180, 180], degrees=True).as_matrix()
            R_avg = R_avg @ r_correction
            
            # ===== 4. 相机 → 机械臂基坐标系变换 =====
            midpoint_hom = np.hstack([midpoint_cam, 1.0])
            midpoint_arm = self.T_cam_in_arm @ midpoint_hom
            R_arm = self.T_cam_in_arm[:3, :3] @ R_avg
            t_arm = midpoint_arm[:3]
            
            # 夹爪朝向校正：绕Y轴旋转30°（使夹爪倾斜朝下）
            r_y_30 = R_scipy.from_euler('y', 30, degrees=True).as_matrix()
            R_arm_corrected = r_y_30 @ R_arm
            quat_arm = R_scipy.from_matrix(R_arm_corrected).as_quat()
            
            # 高度补偿 + 接近偏移
            t_arm[2] += self.height_offset_m
            z_axis_arm = R_scipy.from_quat(quat_arm).as_matrix() @ np.array([0, 0, 1])
            t_arm -= self.approach_offset_m * z_axis_arm

            return t_arm, quat_arm
            
        except Exception as e:
            self.get_logger().error(f"Frame processing failed: {str(e)}", exc_info=True)
            return None, None

    def publish_pose(self, position, quaternion):
        """发布目标位姿"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.pose.position.x = float(position[0])
        msg.pose.position.y = float(position[1])
        msg.pose.position.z = float(position[2])
        msg.pose.orientation.x = float(quaternion[0])
        msg.pose.orientation.y = float(quaternion[1])
        msg.pose.orientation.z = float(quaternion[2])
        msg.pose.orientation.w = float(quaternion[3])
        self.grasp_pub.publish(msg)
        self.get_logger().info(
            f"✅ 目标位姿发布: xyz=({position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}) | "
            f"quat=({quaternion[0]:.3f}, {quaternion[1]:.3f}, {quaternion[2]:.3f}, {quaternion[3]:.3f})"
        )

    def publish_gripper(self, gap):
        """发布夹爪命令"""
        msg = Float64()
        msg.data = float(gap)
        self.gripper_pub.publish(msg)
        self.get_logger().info(f"✅ 夹爪命令发布: 开距 = {gap*2:.4f} m")

    def publish_cmd_vel(self, vx, vy, vz=0.0):
        """发布小车速度命令（仅xy平移，无旋转）"""
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = vz
        self.cmd_vel_pub.publish(msg)

    def wait_for_frames(self, timeout_sec=3.0):
        """非阻塞等待同步帧（使用 spin_once 处理回调）"""
        start = time.time()
        while (time.time() - start) < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)  # 处理一次回调
            if self.latest_frame is not None:
                frame = self.latest_frame
                self.latest_frame = None  # 消费帧
                return frame
        return None

    def adjust_base_position(self, initial_position, yaw):
        """
        调整小车位置使目标点满足: x < 0.48m 且 |y| < 0.03m
        返回调整后的最新位姿 (position, quaternion)
        """
        
        # 调整参数（硬编码，暂不放入参数服务器）
       
        start_time = time.time()
        last_adjust_time = 0.0
        position = initial_position.copy()
        quaternion = None
        
        # 初始帧获取
        frame = self.wait_for_frames(timeout_sec=1.0)
        if frame is None:
            self.get_logger().warn("⚠️ 调整开始时未获取到帧")
            return initial_position, None



        # 调整循环
        while (time.time() - start_time) < self.MAX_ADJUST_TIME:
                        
            rgb, depth, _ = frame
            position, quaternion = self.process_frame(rgb, depth)
            if position is None or quaternion is None:
                self.get_logger().warn("⚠️ 初始位姿计算失败")
                return initial_position, None
            yaw = self.get_base_yaw(quaternion)
            # 检查是否满足条件
            if position[0] < self.X_THRESHOLD and abs(position[1]) < self.Y_THRESHOLD and abs(self.YAW_TARGET - yaw) < self.YAW_THRESHOLD:
                self.get_logger().info(
                    f"✅ 位置调整完成: x={position[0]:.3f}m (<{self.X_THRESHOLD}m), "
                    f"y={position[1]:.3f}m (|y|<{self.Y_THRESHOLD}m), "
                    f"yaw={np.degrees(yaw):.2f}° "
                )
                self.publish_cmd_vel(0.0, 0.0, 0.0)  # 停止小车
                return position, quaternion
            
            # 计算调整速度
            vx = self.VX_ADJUST if position[0] >= self.X_THRESHOLD else 0.0
            vy = self.VY_ADJUST if position[1] > self.Y_THRESHOLD else (-self.VY_ADJUST if position[1] < -self.Y_THRESHOLD else 0.0)
            vz = -self.VZ_ADJUST if self.YAW_TARGET - yaw >= self.YAW_THRESHOLD else (self.VZ_ADJUST if self.YAW_TARGET - yaw <= -self.YAW_THRESHOLD else 0.0)
            
            # 仅当需要调整时才发布速度命令
            if vx != 0.0 or vy != 0.0 or vz != 0.0:
                self.publish_cmd_vel(vx, vy, vz)
                self.get_logger().debug(
                    f"🔧 调整中: x={position[0]:.3f}m, vx={vx:.2f}m/s, "
                    f"y={position[1]:.3f}m, vy={vy:.2f}m/s, "
                    f"yaw={np.degrees(self.YAW_TARGET - yaw):.2f}°, vz={vz:.2f}rad/s"
                )
            
            # 等待调整间隔
            time.sleep(self.ADJUST_INTERVAL)
            
            # 获取新帧并重新计算位姿
            frame = self.wait_for_frames(timeout_sec=1.0)
            if frame is None:
                self.get_logger().warn("⚠️ 调整过程中未获取到新帧")
                continue
                
            rgb, depth, _ = frame
            new_pos, new_quat = self.process_frame(rgb, depth)
            if new_pos is None or new_quat is None:
                self.get_logger().warn("⚠️ 调整过程中位姿计算失败")
                continue
                
            position = new_pos
            quaternion = new_quat
        
        # 超时处理
        self.publish_cmd_vel(0.0, 0.0, 0.0)  # 确保停止小车
        self.get_logger().warn(
            f"⚠️ 小车调整超时 ({self.MAX_ADJUST_TIME}s)，当前位姿: "
            f"x={position[0]:.3f}m, y={position[1]:.3f}m"
        )
        return position, quaternion
    
    def get_base_yaw(self, quaternion):
        """
        从四元数获取小车朝向（yaw，单位：弧度）
        quaternion: [x, y, z, w]
        """
        x, y, z, w = quaternion
        # yaw = arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return yaw

    def execute_drop_sequence(self):
        """完整放置流程（带重试和小车调整）"""
        self.get_logger().info("⏳ 开始咖啡袋放置流程...")
        
        # 初始等待：确保订阅器建立连接并收到首帧
        self.get_logger().info("⏳ 等待传感器数据流初始化（2秒）...")
        
        for attempt in range(1, self.max_retries + 1):
            self.get_logger().info(f"🔄 尝试 #{attempt}/{self.max_retries}")
            
            # 获取同步帧
            frame = self.wait_for_frames(timeout_sec=3.0)
            if frame is None:
                self.get_logger().warn("⚠️ 未收到新帧，重试...")
                time.sleep(self.retry_interval)
                continue
            
            rgb, depth, _ = frame
            position, quaternion = self.process_frame(rgb, depth)
            
            if position is None or quaternion is None:
                self.get_logger().warn("⚠️ 位姿计算失败，重试...")
                time.sleep(self.retry_interval)
                continue

            yaw = self.get_base_yaw(quaternion)
            self.get_logger().info(f"🧭 计算得到小车朝向 (yaw={np.degrees(yaw):.2f}°)")

            # ===== 新增：小车位置调整 =====
            if position[0] >= self.X_THRESHOLD or abs(position[1]) >= self.Y_THRESHOLD or abs(self.YAW_TARGET - yaw) >= self.YAW_THRESHOLD:
                self.get_logger().info(
                    f"⚠️ 目标点超出阈值 (x={position[0]:.3f}m ≥ {self.X_THRESHOLD}m or |y|={abs(position[1]):.3f}m ≥ {self.Y_THRESHOLD}m) |yaw|={abs(self.YAW_TARGET - yaw):.2f} ≥ {self.YAW_THRESHOLD:.2f}，"
                    "启动小车平移调整..."
                )
                position, quaternion = self.adjust_base_position(position, yaw)
                
                # 调整后再次检查位姿有效性
                if position is None or quaternion is None:
                    self.get_logger().warn("⚠️ 调整后位姿无效，重试...")
                    time.sleep(self.retry_interval)
                    continue
            
            # 距离判断（调整后仍需检查）
            distance = np.linalg.norm(position[:2])  # XY平面距离
            if distance > 0.51:
                self.get_logger().warn(f"⚠️ 目标点过远（{distance:.3f} m），重试...")
                time.sleep(self.retry_interval)
                continue
            
            # 成功：执行放置动作
            self.publish_pose(position, quaternion)
            self.get_logger().info("⏳ 等待机械臂移动到位（3秒）...")
            time.sleep(5.0)

            # put pos z -10cm y轴旋转30度
            put_position = position.copy()
            put_position[2] -= 0.10
            r_y_30 = R_scipy.from_euler('y', 30, degrees=True).as_matrix()
            q_rotated = R_scipy.from_matrix(r_y_30 @ R_scipy.from_quat(quaternion).as_matrix()).as_quat()
            self.publish_pose(put_position, q_rotated)
            self.get_logger().info("⏳ 等待机械臂下降（2秒）...")
            time.sleep(2.0)

            # 打开夹爪放置咖啡袋
            self.publish_gripper(self.gripper_open_gap)
            self.get_logger().info("⏳ 等待夹爪动作完成（2秒）...")
            time.sleep(2.0)
            self.get_logger().info("🎉 咖啡袋放置流程完成！")

            # 回到提起物体姿势
            pe = np.array([-0.025252, -0.005776, 0.413126])
            qe = np.array([-0.5042411323427093, 0.5062575028431882, -0.5180797264523359, 0.4701463796604143])
            self.publish_pose(pe, qe)

            # 关闭夹爪
            self.publish_gripper(gap=0.0)
            time.sleep(2.0)

            return True
        
        self.get_logger().error("❌ 所有重试均失败，放弃放置任务")
        return False


def main(args=None):
    rclpy.init(args=args)
    node = CoffeeDropNode()
    
    success = False
    try:
        success = node.execute_drop_sequence()
        
        if not success:
            node.get_logger().fatal("❌ 放置任务失败！请检查：1)标记可见性 2)相机标定 3)手眼矩阵")
            return 1
            
    except KeyboardInterrupt:
        node.get_logger().info("🛑 操作被用户中断")
    except Exception as e:
        node.get_logger().fatal(f"💥 未处理异常: {str(e)}", exc_info=True)
        return 1
    finally:
        # 确保小车停止
        try:
            node.publish_cmd_vel(0.0, 0.0, 0.0)
            time.sleep(0.1)  # 确保停止命令发出
        except:
            pass
            
        node.destroy_node()
        rclpy.shutdown()
        # 清理OpenCV窗口
        try:
            cv2.destroyAllWindows()
        except:
            pass
    
    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())
