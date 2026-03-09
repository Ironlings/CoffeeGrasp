#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
from sensor_msgs_py import point_cloud2
import numpy as np

class PointCloudOmniAvoidance(Node):
    def __init__(self):
        super().__init__('pc_avoid')
        
        # ================= 小车尺寸参数 =================
        self.declare_parameter('car_length', 0.72)    # 车长 (米)
        self.declare_parameter('car_width', 0.60)     # 车宽 (米)
        self.declare_parameter('radar_offset_x', 0.26) # 雷达在中心前方距离
        
        # 计算半长半宽
        self.car_length = self.get_parameter('car_length').value
        self.car_width = self.get_parameter('car_width').value
        self.half_length = self.car_length / 2.0      # 0.365m
        self.half_width = self.car_width / 2.0        # 0.265m
        self.radar_offset_x = self.get_parameter('radar_offset_x').value
        
        # ================= 安全距离 (车体边缘 + 缓冲) =================
        self.declare_parameter('safe_buffer_front', 0.1)   # 前方缓冲
        self.declare_parameter('safe_buffer_back', 0.1)    # 后方缓冲
        self.declare_parameter('safe_buffer_side', 0.05)    # 侧方缓冲
        self.declare_parameter('safe_buffer_rotate', -0.08)  # 旋转缓冲
        
        # 计算实际安全距离
        self.safe_front = self.half_length + self.get_parameter('safe_buffer_front').value
        self.safe_back = self.half_length + self.get_parameter('safe_buffer_back').value
        self.safe_left = self.half_width + self.get_parameter('safe_buffer_side').value
        self.safe_right = self.half_width + self.get_parameter('safe_buffer_side').value
        
        # 旋转安全距离 = 对角线半径 + 缓冲
        self.diagonal_radius = np.sqrt(self.half_length**2 + self.half_width**2)
        self.safe_rotate = self.diagonal_radius + self.get_parameter('safe_buffer_rotate').value
        
        # ================= 高度过滤 =================
        self.declare_parameter('min_height', -0.3)
        self.declare_parameter('max_height', 1.0)
        self.min_h = self.get_parameter('min_height').value
        self.max_h = self.get_parameter('max_height').value
        
        # ================= 速度限制 =================
        self.declare_parameter('max_vx', 0.5)
        self.declare_parameter('max_vy', 0.5)
        self.declare_parameter('max_vz', 1.0)
        
        # ================= 订阅与发布 =================
        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel_raw', self.cmd_callback, 10)
        self.sub_cloud = self.create_subscription(PointCloud2, '/livox/lidar', self.cloud_callback, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.latest_cloud = None
        self.input_cmd = Twist()
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info(f"=== 全向避障节点启动 ===")
        self.get_logger().info(f"车体尺寸：{self.car_length}m × {self.car_width}m")
        self.get_logger().info(f"雷达偏移：X+{self.radar_offset_x}m")
        self.get_logger().info(f"安全距离：前{self.safe_front:.2f}m 后{self.safe_back:.2f}m 左{self.safe_left:.2f}m 右{self.safe_right:.2f}m 旋转{self.safe_rotate:.2f}m")

    def cmd_callback(self, msg):
        self.input_cmd = msg

    def cloud_callback(self, msg):
        self.latest_cloud = msg

    def get_obstacle_distances(self):
        """
        使用车体长宽进行矩形分区，计算各方向最小障碍物距离
        返回：(front_dist, back_dist, left_dist, right_dist)
        """
        if self.latest_cloud is None:
            return float('inf'), float('inf'), float('inf'), float('inf')
        
        try:
            # 1. 解析点云为 numpy 数组 (x, y, z)
            points = list(point_cloud2.read_points(
                self.latest_cloud, 
                field_names=("x", "y", "z"), 
                skip_nans=True
            ))
            
            if not points:
                return float('inf'), float('inf'), float('inf'), float('inf')
            
            points_np = np.array([[p[0], p[1], p[2]] for p in points], dtype=np.float32) #shape: (N, 3)

            # 2. 坐标补偿：雷达坐标系 → 小车中心坐标系
            points_np[:, 0] += self.radar_offset_x
            
            # 3. 高度过滤
            z_mask = (points_np[:, 2] > self.min_h) & (points_np[:, 2] < self.max_h)
            filtered_points = points_np[z_mask]
            
            if filtered_points.size == 0:
                return float('inf'), float('inf'), float('inf'), float('inf')
            

            
            # ================= 5. 矩形分区 (基于车体长宽) =================
            
            # --- 前方区域：x > 半长 且 |y| < 半宽 ---
            # 只检测车体前方延伸区域的障碍物
            front_mask = (filtered_points[:, 0] > self.half_length) & \
                         (np.abs(filtered_points[:, 1]) < self.half_width)
            front_points = filtered_points[front_mask]
            front_dist = np.min(np.linalg.norm(front_points[:, :2], axis=1)) if front_points.size > 0 else float('inf')
            
            # --- 后方区域：x < -半长 且 |y| < 半宽 ---
            back_mask = (filtered_points[:, 0] < -self.half_length) & \
                        (np.abs(filtered_points[:, 1]) < self.half_width)
            back_points = filtered_points[back_mask]
            back_dist = np.min(np.linalg.norm(back_points[:, :2], axis=1)) if back_points.size > 0 else float('inf')
            
            # --- 左侧区域：y > 半宽 且 |x| < 半长 ---
            left_mask = (filtered_points[:, 1] > self.half_width) & \
                        (np.abs(filtered_points[:, 0]) < self.half_length)
            left_points = filtered_points[left_mask]
            left_dist = np.min(np.linalg.norm(left_points[:, :2], axis=1)) if left_points.size > 0 else float('inf')
            
            # --- 右侧区域：y < -半宽 且 |x| < 半长 ---
            right_mask = (filtered_points[:, 1] < -self.half_width) & \
                         (np.abs(filtered_points[:, 0]) < self.half_length)
            right_points = filtered_points[right_mask]
            right_dist = np.min(np.linalg.norm(right_points[:, :2], axis=1)) if right_points.size > 0 else float('inf')
            
            return front_dist, back_dist, left_dist, right_dist
            
        except Exception as e:
            self.get_logger().error(f"Point cloud processing error: {e}")
            return float('inf'), float('inf'), float('inf'), float('inf')

    def control_loop(self):
        if self.latest_cloud is None:
            return
        
        # 1. 获取各方向障碍物距离
        dist_front, dist_back, dist_left, dist_right = self.get_obstacle_distances()
        self.get_logger().info(f'F:{dist_front:.2f}, L:{dist_left:.2f}, R:{dist_right:.2f}, B:{dist_back:.2f}')
        # 2. 输出速度初始化
        output_cmd = Twist()
        
        # ================= 避障逻辑 =================
        # --- X 轴 (前进/后退) ---
        if self.input_cmd.linear.x > 0.01:  # 前进
            if dist_front < self.safe_front:
                self.get_logger().warn(f"⚠️  前方障碍：{dist_front:.2f}m < 安全{self.safe_front:.2f}m")
                output_cmd.linear.x = 0.0
            else:
                output_cmd.linear.x = self.input_cmd.linear.x
        elif self.input_cmd.linear.x < -0.01:  # 后退
            if dist_back < self.safe_back:
                self.get_logger().warn(f"⚠️  后方障碍：{dist_back:.2f}m < 安全{self.safe_back:.2f}m")
                output_cmd.linear.x = 0.0
            else:
                output_cmd.linear.x = self.input_cmd.linear.x
        else:
            output_cmd.linear.x = 0.0
        
        # --- Y 轴 (横移) ---
        if self.input_cmd.linear.y > 0.01:  # 向左横移
            if dist_left < self.safe_left:
                self.get_logger().warn(f"⚠️  左侧障碍：{dist_left:.2f}m < 安全{self.safe_left:.2f}m")
                output_cmd.linear.y = 0.0
            else:
                output_cmd.linear.y = self.input_cmd.linear.y
        elif self.input_cmd.linear.y < -0.01:  # 向右横移
            if dist_right < self.safe_right:
                self.get_logger().warn(f"⚠️  右侧障碍：{dist_right:.2f}m < 安全{self.safe_right:.2f}m")
                output_cmd.linear.y = 0.0
            else:
                output_cmd.linear.y = self.input_cmd.linear.y
        else:
            output_cmd.linear.y = 0.0
        
        # --- Z 轴 (自转) ---
        if abs(self.input_cmd.angular.z) > 0.01:
            min_dist = min(dist_front, dist_back, dist_left, dist_right)
            if min_dist < self.safe_rotate:
                self.get_logger().warn(f"⚠️  旋转危险：{min_dist:.2f}m < 安全{self.safe_rotate:.2f}m")
                output_cmd.angular.z = 0.0
            else:
                output_cmd.angular.z = self.input_cmd.angular.z
        else:
            output_cmd.angular.z = 0.0
        
        # ================= 限速保护 =================
        max_vx = self.get_parameter('max_vx').value
        max_vy = self.get_parameter('max_vy').value
        max_vz = self.get_parameter('max_vz').value
        
        output_cmd.linear.x = max(-max_vx, min(output_cmd.linear.x, max_vx))
        output_cmd.linear.y = max(-max_vy, min(output_cmd.linear.y, max_vy))
        output_cmd.angular.z = max(-max_vz, min(output_cmd.angular.z, max_vz))
        
        self.pub_cmd.publish(output_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudOmniAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()