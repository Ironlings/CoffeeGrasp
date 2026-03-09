#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool
from sensor_msgs.msg import PointCloud2
import math
import time
import numpy as np
import struct

class MotionCommander(Node):

    def __init__(self):
        super().__init__('motion_commander')

        # ================= ROS 通信 =================
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.sub_yolo = self.create_subscription(
            String, '/yolo/result', self.yolo_callback, 10
        )
        # 新增：雷达订阅
        self.pc_sub = self.create_subscription(
            PointCloud2, '/livox/lidar', self.pc_callback, 10
        )
        self.sub_panel_aligned = self.create_subscription(
            Bool, '/panel_arive', self.pa_callback, 1
        )
        # ================= 状态变量 =================
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_ready = False
        self.yolo_state = ""
        self.panel_aligned = False
        
        # 雷达停车状态
        self.lidar_data = {
            'right_dist': None, 'right_angle': None,
            'front_dist': None, 'front_angle': None
        }
        self.parking_converged = False

        self.parking_mode = False

        # ================= 参数声明 (来自 WallFollower) =================
        # 右墙区域 (Y 轴负方向)
        self.declare_parameter('right_x_min', -1.0)
        self.declare_parameter('right_x_max', 0.0)
        self.declare_parameter('right_y_min', -3.0)
        self.declare_parameter('right_y_max', -0.26)

        # 前墙区域 (X 轴正方向)
        self.declare_parameter('front_x_min', 0.1)
        self.declare_parameter('front_x_max', 3.0)
        self.declare_parameter('front_y_min', -0.3)
        self.declare_parameter('front_y_max', 0.3)

        # 高度
        self.declare_parameter('z_min', -0.50)
        self.declare_parameter('z_max', 1.5)

        # 目标距离
        self.declare_parameter('right_target_dist', 0.36)
        self.declare_parameter('front_target_dist', 0.650)

        # 控制参数
        self.declare_parameter('kp_right_dist', 0.4)
        self.declare_parameter('kp_right_angle', 0.4)
        self.declare_parameter('kp_front_dist', 0.8)
        self.declare_parameter('kp_front_angle', 0.5)

        self.declare_parameter('max_vx', 0.5)
        self.declare_parameter('max_vy', 0.5)
        self.declare_parameter('max_wz', 0.5)

        # 平面拟合
        self.declare_parameter('min_plane_points', 50)
        self.declare_parameter('plane_thresh', 0.03)

        self.get_logger().info("运动控制 +3D 精准停车节点启动 ✨")

    # ================= ODOM 回调 =================
    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny, cosy)
        self.odom_ready = True

    # ================= YOLO 回调 =================
    def yolo_callback(self, msg):
        self.yolo_state = msg.data

    # ================= 面板对齐回调 =================
    def pa_callback(self, msg):
        self.panel_aligned = msg.data

    # ================= 点云回调 (只更新数据，不发布控制) =================
    def pc_callback(self, msg):
        # 如果不在停车模式，不处理以节省资源（可选优化）
        if not self.parking_mode: return 
        
        points = self.pc2_to_xyz(msg)
        if points is None or len(points) == 0:
            return

        min_pts = self.get_parameter('min_plane_points').value
        
        # 提取右墙点
        right_pts = self.extract_region(points, 'right')
        right_plane = self.fit_plane(right_pts) if len(right_pts) > min_pts else None

        # 提取前墙点
        front_pts = self.extract_region(points, 'front')
        front_plane = self.fit_plane(front_pts) if len(front_pts) > min_pts else None

        # 计算右墙信息
        if right_plane is not None:
            a, b, c, d = right_plane
            normal = np.array([a, b, c])
            n_xy = normal[:2] / (np.linalg.norm(normal[:2]) + 1e-8)
            target_n = np.array([0.0, -1.0]) # 右墙法向量指向右侧
            angle_err = self.angle_error(n_xy, target_n)
            dist = abs(d / np.linalg.norm(normal))
            self.lidar_data['right_dist'] = dist
            self.lidar_data['right_angle'] = angle_err
        else:
            self.lidar_data['right_dist'] = None
            self.lidar_data['right_angle'] = None

        # 计算前墙信息
        if front_plane is not None:
            a, b, c, d = front_plane
            normal = np.array([a, b, c])
            n_xy = normal[:2] / (np.linalg.norm(normal[:2]) + 1e-8)
            target_n = np.array([0.0, 1.0]) # 前墙法向量指向正前方
            angle_err = self.angle_error(n_xy, target_n)
            dist = abs(d / np.linalg.norm(normal))
            self.lidar_data['front_dist'] = dist
            self.lidar_data['front_angle'] = angle_err
        else:
            self.lidar_data['front_dist'] = None
            self.lidar_data['front_angle'] = None

    # ================= 核心移动函数 (里程计) =================
    def move(self, direction, speed, distance):
        while not self.odom_ready:
            rclpy.spin_once(self)

        start_x = self.current_x
        start_y = self.current_y
        start_time = time.time()
        kp = 1.5
        
        # 距离取绝对值，方向由 speed 的正负决定
        target_dist = distance
        self.get_logger().info(f"开始移动 方向:{direction} 目标距离:{target_dist:.2f}m 速度:{speed:.2f}")

        # 如果是旋转，直接调用旋转函数，不进入平移循环
        if direction == 'z':
            self.rotate(speed, distance)
            return

        # 平移循环 (X 或 Y)
        while rclpy.ok():
            rclpy.spin_once(self)
            
            dx = self.current_x - start_x
            dy = self.current_y - start_y
            
            error = abs(target_dist) - (dx**2 + dy**2)

            # 到达目标
            if abs(error) <= 0.01: 
                break

            # P 控制
            control_speed = kp * error * math.copysign(1.0, speed)
            
            # 【修复点 1】限制最大速度，但保留符号 (允许负数)
            control_speed = np.clip(control_speed, -abs(speed), abs(speed))
            
            # 【修复点 2】设置最小速度阈值，但保留符号 (允许负数)
            if abs(control_speed) < 0.05:
                control_speed = math.copysign(0.05, control_speed)

            cmd = Twist()
            if direction == 'x':
                cmd.linear.x = control_speed
            elif direction == 'y':
                cmd.linear.y = control_speed
            self.get_logger().info(f"speed: {control_speed} error:{error}")
            self.pub_cmd.publish(cmd)

        # 停止
        self.pub_cmd.publish(Twist())
        duration = time.time() - start_time
        self.get_logger().info(f"移动完成，用时 {duration:.3f} 秒 ")
        time.sleep(0.5)

    def rotate(self, speed, angle_rad):
        start_yaw = self.current_yaw
        target_yaw = start_yaw + angle_rad
        if target_yaw > math.pi:
            target_yaw -= 2 * math.pi
        kp = 1.0
        while rclpy.ok():
            rclpy.spin_once(self)
            # 计算角度差
            current_yaw = self.current_yaw

            dyaw = target_yaw - current_yaw
            # 简单处理，实际需处理 2PI 跳变
            if abs(dyaw) < 0.02:
                break
            
            cmd = Twist()
            cmd.angular.z = math.copysign(min(abs(dyaw)*kp, speed), dyaw)
            self.pub_cmd.publish(cmd)
        self.pub_cmd.publish(Twist())

    # ================= 等待函数 =================
    def wait_for_open(self):
        self.get_logger().info("等待 YOLO 识别到 open ...")
        while rclpy.ok():
            rclpy.spin_once(self)
            if self.yolo_state == "open":
                self.get_logger().info("检测到 open，继续执行 ✨")
                break
            time.sleep(0.05)

    def wait_for_panel_align(self):
        self.get_logger().info("等待面板对齐完成 ...")
        while rclpy.ok():
            rclpy.spin_once(self)
            if self.panel_aligned:
                self.get_logger().info("面板对齐完成，继续执行 ✨")
                break
            time.sleep(0.05)

    # ================= 3D 雷达精准停车 (右墙 + 后墙) =================
    def lidar_park(self):
        self.get_logger().info("开始 3D 雷达精准停车 (右墙) ~")
        self.parking_mode = True
        
        # ================= 阶段控制状态 =================
        phase = 0  # 1=旋转对齐角度，2=平移对齐距离，3=完成
        converge_count = 0
        required_frames = 5
        
        # 角度对齐阈值
        angle_threshold = np.radians(3.0)  # 3 度
        # 距离对齐阈值
        dist_threshold = 0.03  # 3cm
        
        start_time = time.time()
        
        while rclpy.ok():
            rclpy.spin_once(self)  # 触发点云回调更新数据

            rd = self.lidar_data['right_dist']
            ra = self.lidar_data['right_angle']
            fd = self.lidar_data['front_dist']  
            print(f"Right Angle: {ra}")
            # ba = self.lidar_data['back_angle']

            # 如果数据丢失，停止
            if rd is None or ra is None or fd is None:
                self.get_logger().warn("雷达数据丢失，暂停停车")
                self.pub_cmd.publish(Twist())
                time.sleep(0.1)
                converge_count = 0
                continue

            cmd = Twist()
            is_stable = False
            # ================= 阶段 0: 前进对齐距离 =================
            if phase == 0:
                self.get_logger().info(f"阶段 0/2: 平移对齐距离 (当前：{fd:.3f}m, 目标：{self.get_parameter('front_target_dist').value:.3f}m)")
                
                # 右墙距离控制 (Y 轴)
                front_target = self.get_parameter('front_target_dist').value
                front_err = fd - front_target
                
                # 只控制 Y 轴平移，角速度为 0（保持已对齐的角度）
                cmd.linear.x = self.get_parameter('kp_front_dist').value * front_err
                cmd.linear.y = 0.0
                cmd.angular.z = 0.0  # 保持角度，不再旋转
                
                # 限速
                cmd.linear.x = np.clip(cmd.linear.x, 
                                    -self.get_parameter('max_vx').value, 
                                    self.get_parameter('max_vx').value)
                
                # 判断距离是否对齐
                if abs(front_err) < dist_threshold:
                    converge_count += 1
                else:
                    converge_count = 0
                
                # 距离稳定 5 帧后完成
                if converge_count >= required_frames:
                    phase = 1
                    self.get_logger().info('✓ 前墙精准停车完成！')
                    self.move('z', 0.5, 3.14)
                    time.sleep(0.5)

            # ================= 阶段 1: 先旋转对齐角度 =================
            elif phase == 1:
                self.get_logger().info(f"阶段 1/2: 旋转对齐角度 (当前：{ra:.2f}°)")
                
                # 只控制角速度，平移速度为 0
                cmd.linear.x = 0.0
                cmd.linear.y = 0.0
                cmd.angular.z = -self.get_parameter('kp_right_angle').value * ra
                
                # 限速
                cmd.angular.z = np.clip(cmd.angular.z, 
                                    -self.get_parameter('max_wz').value, 
                                        self.get_parameter('max_wz').value)
                
                # 判断角度是否对齐
                if abs(ra) < angle_threshold:
                    converge_count += 1
                else:
                    converge_count = 0
                
                # 角度稳定 5 帧后进入下一阶段
                if converge_count >= required_frames:
                    phase = 2
                    converge_count = 0
                    self.get_logger().info("✓ 角度对齐完成，进入距离对齐阶段 ~")

            # ================= 阶段 2: 再平移对齐距离 =================
            elif phase == 2:
                self.get_logger().info(f"阶段 2/2: 平移对齐距离 (当前：{rd:.3f}m, 目标：{self.get_parameter('right_target_dist').value:.3f}m)")
                
                # 右墙距离控制 (Y 轴)
                right_target = self.get_parameter('right_target_dist').value
                right_err = rd - right_target
                
                # 只控制 Y 轴平移，角速度为 0（保持已对齐的角度）
                cmd.linear.x = 0.0
                cmd.linear.y = -self.get_parameter('kp_right_dist').value * right_err
                cmd.angular.z = 0.0  # 保持角度，不再旋转
                
                # 限速
                cmd.linear.y = np.clip(cmd.linear.y, 
                                    -self.get_parameter('max_vy').value, 
                                    self.get_parameter('max_vy').value)
                
                # 判断距离是否对齐
                if abs(right_err) < dist_threshold:
                    converge_count += 1
                else:
                    converge_count = 0
                
                # 距离稳定 5 帧后完成
                if converge_count >= required_frames:
                    phase = 3
                    self.get_logger().info('✓ 右墙精准停车完成！')
                    break

            # ================= 阶段 3: 完成 =================
            elif phase == 3:
                self.pub_cmd.publish(Twist())
                break

            self.pub_cmd.publish(cmd)
            
            # 超时保护（防止无限循环）
            if time.time() - start_time > 30.0:
                self.get_logger().warn("停车超时，强制结束")
                break

        # 最终停止
        self.pub_cmd.publish(Twist())
        self.parking_mode = False
        duration = time.time() - start_time
        self.get_logger().info(f"停车总用时：{duration:.2f} 秒 ~")

    # =====================================================
    # 工具函数 (来自 WallFollower)
    # =====================================================
    def pc2_to_xyz(self, cloud):
        pts = []
        for i in range(cloud.width * cloud.height):
            off = i * cloud.point_step
            x = struct.unpack_from('f', cloud.data, off + cloud.fields[0].offset)[0]
            y = struct.unpack_from('f', cloud.data, off + cloud.fields[1].offset)[0]
            z = struct.unpack_from('f', cloud.data, off + cloud.fields[2].offset)[0]
            if np.isfinite(x) and np.isfinite(y) and np.isfinite(z):
                pts.append([x, y, z])
        return np.array(pts)

    def extract_region(self, pts, side):
        z_min = self.get_parameter('z_min').value
        z_max = self.get_parameter('z_max').value

        if side == 'right':
            x_min = self.get_parameter('right_x_min').value
            x_max = self.get_parameter('right_x_max').value
            y_min = self.get_parameter('right_y_min').value
            y_max = self.get_parameter('right_y_max').value
        elif side == 'front':
            x_min = self.get_parameter('front_x_min').value
            x_max = self.get_parameter('front_x_max').value
            y_min = self.get_parameter('front_y_min').value
            y_max = self.get_parameter('front_y_max').value


        mask = (
            (pts[:, 0] > x_min) & (pts[:, 0] < x_max) &
            (pts[:, 1] > y_min) & (pts[:, 1] < y_max) &
            (pts[:, 2] > z_min) & (pts[:, 2] < z_max)
        )
        return pts[mask]

    def fit_plane(self, pts):
        thresh = self.get_parameter('plane_thresh').value
        best_inliers = []
        if len(pts) < 3: return None

        for _ in range(100): # 减少迭代次数以提速
            idx = np.random.choice(len(pts), 3, replace=False)
            p1, p2, p3 = pts[idx]
            n = np.cross(p2 - p1, p3 - p1)
            if np.linalg.norm(n) < 1e-6: continue
            n /= np.linalg.norm(n)
            d = -np.dot(n, p1)
            dist = np.abs(np.dot(pts, n) + d)
            inliers = pts[dist < thresh]
            if len(inliers) > len(best_inliers):
                best_inliers = inliers

        if len(best_inliers) < self.get_parameter('min_plane_points').value:
            return None

        centroid = np.mean(best_inliers, axis=0)
        _, _, vh = np.linalg.svd(best_inliers - centroid)
        n = vh[2]
        # 确保法向量方向一致性 (可选，这里简化处理)
        if n[1] > 0:
            n = -n
        d = -np.dot(n, centroid)
        return (*n, d)

    def angle_error(self, n1, n2):
        dot = np.clip(np.dot(n1, n2), -1.0, 1.0)
        ang = np.arccos(dot)
        cross = n1[0] * n2[1] - n1[1] * n2[0]
        return -ang if cross < 0 else ang

def main(args=None):
    rclpy.init(args=args)
    node = MotionCommander()

    try:
        # 0. 等待面板对齐完成
        node.wait_for_panel_align()

        # 1. 基础移动与识别
        node.move('y', 0.5, 0.8)      # 左移0.8
        node.wait_for_open()          # 等待 YOLO
        node.move('x', 0.5, 2.0)      # 前进2.0
        
        # 2. 自转 180 
        # node.move('z', 0.5, 3.14)     

        # 3. 3D 雷达精准停车 (右墙 
        node.lidar_park()

    except KeyboardInterrupt:
        pass
    finally:
        node.pub_cmd.publish(Twist())
        time.sleep(0.1)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()