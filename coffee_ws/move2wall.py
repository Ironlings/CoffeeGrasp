import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
import numpy as np
import struct


class WallFollower3DNode(Node):
    def __init__(self):
        super().__init__('wall_follower_3d_node')

        # ================= 参数 =================
        # 右墙区域
        self.declare_parameter('right_x_min', 0.2)
        self.declare_parameter('right_x_max', 2.0)
        self.declare_parameter('right_y_min', -1.5)
        self.declare_parameter('right_y_max', -0.1)

        # 前墙区域
        self.declare_parameter('front_x_min', 0.2)
        self.declare_parameter('front_x_max', 2.0)
        self.declare_parameter('front_y_min', -0.5)
        self.declare_parameter('front_y_max', 0.5)

        # 高度
        self.declare_parameter('z_min', 0.1)
        self.declare_parameter('z_max', 1.5)

        # 目标距离
        self.declare_parameter('right_target_dist', 0.36)
        self.declare_parameter('front_target_dist', 0.50)

        # 控制参数
        self.declare_parameter('kp_right_dist', 1.2)
        self.declare_parameter('kp_right_angle', 2.5)
        self.declare_parameter('kp_front_dist', 0.8)
        self.declare_parameter('kp_front_angle', 0.5)

        self.declare_parameter('max_vx', 0.4)
        self.declare_parameter('max_vy', 0.3)
        self.declare_parameter('max_wz', 0.8)

        # 平面拟合
        self.declare_parameter('min_plane_points', 50)
        self.declare_parameter('plane_thresh', 0.03)

        # ================= ROS =================
        self.pc_sub = self.create_subscription(
            PointCloud2, '/livox/lidar2', self.pc_callback, 10
        )
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ================= 状态 =================
        self.converged = False
        self.converge_count = 0
        self.required_converge_frames = 5

        self.get_logger().info('3D 右墙 + 前墙 精准停车控制器启动喵~')

    # =====================================================
    # 点云回调
    # =====================================================
    def pc_callback(self, msg):
        # 到位后彻底锁死
        if self.converged:
            self.cmd_pub.publish(Twist())
            return

        points = self.pc2_to_xyz(msg)
        if points is None or len(points) == 0:
            self.stop()
            return

        right_pts = self.extract_region(points, 'right')
        front_pts = self.extract_region(points, 'front')

        right_plane = self.fit_plane(right_pts) if len(right_pts) > self.get_parameter('min_plane_points').value else None
        front_plane = self.fit_plane(front_pts) if len(front_pts) > self.get_parameter('min_plane_points').value else None

        cmd = Twist()

        right_dist = None
        right_angle_err = None
        front_dist = None
        front_angle_err = None

        # ================= 右墙控制 =================
        if right_plane is not None:
            a, b, c, d = right_plane
            normal = np.array([a, b, c])

            n_xy = normal[:2] / (np.linalg.norm(normal[:2]) + 1e-8)
            target_n = np.array([0.0, -1.0])

            right_angle_err = self.angle_error(n_xy, target_n)
            right_dist = abs(d / np.linalg.norm(normal))
            dist_err = right_dist - self.get_parameter('right_target_dist').value

            cmd.linear.y = -self.get_parameter('kp_right_dist').value * dist_err
            cmd.angular.z = -self.get_parameter('kp_right_angle').value * right_angle_err

        # ================= 前墙控制 =================
        if front_plane is not None:
            a, b, c, d = front_plane
            normal = np.array([a, b, c])

            n_xy = normal[:2] / (np.linalg.norm(normal[:2]) + 1e-8)
            target_n = np.array([-1.0, 0.0])

            front_angle_err = self.angle_error(n_xy, target_n)
            front_dist = abs(d / np.linalg.norm(normal))
            dist_err = front_dist - self.get_parameter('front_target_dist').value

            cmd.linear.x = -self.get_parameter('kp_front_dist').value * dist_err
            cmd.angular.z += -self.get_parameter('kp_front_angle').value * front_angle_err

        # ================= 限速 =================
        cmd.linear.x = np.clip(cmd.linear.x,
                               -self.get_parameter('max_vx').value,
                                self.get_parameter('max_vx').value)
        cmd.linear.y = np.clip(cmd.linear.y,
                               -self.get_parameter('max_vy').value,
                                self.get_parameter('max_vy').value)
        cmd.angular.z = np.clip(cmd.angular.z,
                               -self.get_parameter('max_wz').value,
                                self.get_parameter('max_wz').value)

        # ================= 到位判定 =================
        if self.is_converged(right_dist, right_angle_err,
                             front_dist, front_angle_err):
            self.converge_count += 1
        else:
            self.converge_count = 0

        if self.converge_count >= self.required_converge_frames:
            self.converged = True
            self.cmd_pub.publish(Twist())
            self.get_logger().info('✓ 已同时对齐右墙和前墙，精准停车完成喵！')
            return

        self.cmd_pub.publish(cmd)

    # =====================================================
    # 工具函数
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
        else:
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

        for _ in range(100):
            idx = np.random.choice(len(pts), 3, replace=False)
            p1, p2, p3 = pts[idx]
            n = np.cross(p2 - p1, p3 - p1)
            if np.linalg.norm(n) < 1e-6:
                continue
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
        if n[1] > 0:
            n = -n
        d = -np.dot(n, centroid)
        return (*n, d)

    def angle_error(self, n1, n2):
        dot = np.clip(np.dot(n1, n2), -1.0, 1.0)
        ang = np.arccos(dot)
        cross = n1[0] * n2[1] - n1[1] * n2[0]
        return -ang if cross < 0 else ang

    def is_converged(self, rd, ra, fd, fa):
        if rd is None or fd is None or ra is None or fa is None:
            return False

        return (
            abs(rd - self.get_parameter('right_target_dist').value) < 0.03 and
            abs(fd - self.get_parameter('front_target_dist').value) < 0.03 and
            abs(ra) < np.radians(2.0) and
            abs(fa) < np.radians(2.0)
        )

    def stop(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = WallFollower3DNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
