#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math
import time


class MotionCommander(Node):

    def __init__(self):
        super().__init__('motion_commander')

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        self.sub_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        # 订阅 YOLO 结果
        self.sub_yolo = self.create_subscription(
            String,
            '/yolo/result',
            self.yolo_callback,
            10
        )
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_ready = False
        self.yolo_state = ""
        self.get_logger().info("运动控制节点启动喵 ✨")

    # ================= ODOM 回调 =================
    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # 四元数转 yaw
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny, cosy)

        self.odom_ready = True

    # ================= YOLO =================
    def yolo_callback(self, msg):
        self.yolo_state = msg.data

    # ================= 等待函数 =================
    def wait_for_open(self):
        self.get_logger().info("等待 YOLO 识别到 open ...")

        while rclpy.ok():
            rclpy.spin_once(self)

            if self.yolo_state == "open":
                self.get_logger().info("检测到 open，继续执行喵 ✨")
                break

            time.sleep(0.05)

    # ================= 核心移动函数 =================
    def move(self, direction, speed, distance):

        while not self.odom_ready:
            rclpy.spin_once(self)

        start_x = self.current_x
        start_y = self.current_y
        start_yaw = self.current_yaw
        start_time = time.time()

        cmd = Twist()

        if direction == 'x':
            cmd.linear.x = speed
        elif direction == 'y':
            cmd.linear.y = speed
        elif direction == 'z':
            cmd.angular.z = speed
        else:
            self.get_logger().error("方向必须是 'x' 'y' 或 'z'")
            return

        self.get_logger().info(f"开始移动 方向:{direction} 距离:{distance}")

        kp = 1.5

        while rclpy.ok():
            rclpy.spin_once(self)

            dx = self.current_x - start_x
            dy = self.current_y - start_y
            moved = math.sqrt(dx*dx + dy*dy)

            error = distance - moved

            if error <= 0.01:  # 误差小于1cm提前刹车
                break

            cmd = Twist()

            control_speed = kp * error
            control_speed = min(control_speed, speed)
            control_speed = max(control_speed, 0.05)

            if direction == 'x':
                cmd.linear.x = control_speed
            elif direction == 'y':
                cmd.linear.y = control_speed

            self.pub_cmd.publish(cmd)

        # 停止
        self.pub_cmd.publish(Twist())

        duration = time.time() - start_time
        self.get_logger().info(f"完成，用时 {duration:.3f} 秒 喵")

        time.sleep(0.5)


def main(args=None):
    rclpy.init(args=args)
    node = MotionCommander()

    # 示例：连续执行多段运动
    node.move('y', 0.5, 0.8)   # 左移 0.8m
    node.wait_for_open()        # 等待 YOLO 识别到 open
    node.move('x', 0.5, 2.2)   # 前进 1m
    node.move('z', 0.5, 3.14)  # 旋转 90度
    node.move('y', -0.5, 0.6)   # 右进 1m

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()