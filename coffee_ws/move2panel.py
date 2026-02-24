#!/home/agv/anaconda3/envs/yolo/bin/python
import rclpy
from rclpy.node import Node
from message_filters import Subscriber, ApproximateTimeSynchronizer
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import numpy as np
import torch
from ultralytics.models.sam import SAM3SemanticPredictor
import time

class ControlPanelFollower(Node):
    def __init__(self):
        super().__init__('control_panel_follower_tf')

        # 参数
        self.declare_parameter('sam_checkpoint', '/home/agv/armws/coffee_ws/src/coffee_detect/coffeetest/sam3.pt')
        self.declare_parameter('camera_fx', 490.2380676269531)
        self.declare_parameter('camera_fy', 490.2380676269531)
        self.declare_parameter('camera_cx', 316.89483642578125)
        self.declare_parameter('camera_cy', 209.28350830078125)
        self.declare_parameter('target_distance', 0.65)
        self.declare_parameter('kp_linear', 1.5)
        self.declare_parameter('kp_y', 0.5)
        self.declare_parameter('kp_angular', 0.3)
        self.declare_parameter('max_linear', 0.3)
        self.declare_parameter('max_angular', 0.3)
        self.declare_parameter('handeye_pos', [-0.07482322180223497, 0.00623968754635619, 0.050192986145776636])
        self.declare_parameter('handeye_quat', [-0.1197419547833481, 0.12550407599676805, -0.7127657428175671, 0.6796142928445386])

        self.fx = self.get_parameter('camera_fx').value
        self.fy = self.get_parameter('camera_fy').value
        self.cx = self.get_parameter('camera_cx').value
        self.cy = self.get_parameter('camera_cy').value
        self.target_distance = self.get_parameter('target_distance').value
        self.kp_linear = self.get_parameter('kp_linear').value
        self.kp_y = self.get_parameter('kp_y').value
        self.kp_angular = self.get_parameter('kp_angular').value
        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value

        self.T_e2c = self.pose_to_matrix(
            self.get_parameter('handeye_pos').value,
            self.get_parameter('handeye_quat').value
        )

        self.bridge = CvBridge()
        rgb_sub = Subscriber(self, Image, '/dabai/color/image_raw')
        depth_sub = Subscriber(self, Image, '/dabai/depth/image_raw')
        pose_sub = Subscriber(self, PoseStamped, '/end_pose_stamped')
        self.ts = ApproximateTimeSynchronizer([rgb_sub, depth_sub, pose_sub], queue_size=5, slop=0.1)
        self.ts.registerCallback(self.frame_callback)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.grasp_pub = self.create_publisher(PoseStamped, '/my_pose_cmd', 10)
        from std_msgs.msg import Bool
        self.arrived_pub = self.create_publisher(Bool, '/panel_arive', 1)


        # SAM3 初始化
        overrides = dict(conf=0.2, task="segment", mode="predict",
                         model=self.get_parameter('sam_checkpoint').value, half=True, save=False)
        self.predictor = SAM3SemanticPredictor(overrides=overrides)

    def pose_to_matrix(self, pos, quat):
        from scipy.spatial.transform import Rotation as R
        T = np.eye(4)
        T[:3,:3] = R.from_quat(quat).as_matrix()
        T[:3,3] = pos
        return T

    def frame_callback(self, rgb_msg, depth_msg, ee_pose_msg):
        rgb = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
        if depth.dtype == np.uint16:
            depth = depth.astype(np.float32)/1000.0

        # SAM3 mask
        self.predictor.set_image(rgb)
        results = self.predictor(text=["control panel"])
        if len(results[0].boxes) == 0:
            cmd = Twist()
            self.cmd_pub.publish(cmd)
            return
        idx = torch.argmax(results[0].boxes.conf).item()
        mask = results[0].masks.data.cpu().numpy()[idx]
        ys, xs = np.where(mask==1)
        if len(xs)==0: return
        zs = depth[ys,xs]; valid = zs>0
        if np.sum(valid)==0: return
        xs, ys, zs = xs[valid], ys[valid], zs[valid]

        X = (xs - self.cx)*zs/self.fx
        Y = (ys - self.cy)*zs/self.fy
        Z = zs
        centroid_cam = np.array([np.median(X), np.median(Y), np.median(Z), 1.0])

        # Transform: camera -> EE -> base
        centroid_ee = self.T_e2c @ centroid_cam
        T_ee_in_base = self.pose_to_matrix(
            [ee_pose_msg.pose.position.x, ee_pose_msg.pose.position.y, ee_pose_msg.pose.position.z],
            [ee_pose_msg.pose.orientation.x, ee_pose_msg.pose.orientation.y,
             ee_pose_msg.pose.orientation.z, ee_pose_msg.pose.orientation.w]
        )
        centroid_base = T_ee_in_base @ centroid_ee
        pos_base = centroid_base[:3]

        # 拟合平面 计算法向量 得到angle_err
        pts_cam = np.stack([X, Y, Z], axis=1)
        pts_mean = np.mean(pts_cam, axis=0)
        pts_centered = pts_cam - pts_mean
        _, _, vh = np.linalg.svd(pts_centered, full_matrices=False)
        normal_cam = vh[-1]

        if normal_cam[2] < 0:
            normal_cam = -normal_cam

        normal_cam_h = np.array([*normal_cam, 0.0])
        normal_ee = self.T_e2c @ normal_cam_h
        normal_base = T_ee_in_base @ normal_ee
        n = normal_base[:3]
        angle_err = np.arctan2(n[1], n[0])

        print(f"Angle error (rad): {angle_err:.3f}")

        # 控制 law
        x_err = pos_base[0] - self.target_distance  # 前进方向
        y_err = pos_base[1]                    # 横向偏差，正向对应小车右侧
        if x_err > 0.03:
            linear_x = np.clip(self.kp_linear * x_err, -self.max_linear, self.max_linear)
        else:
            linear_x = 0.0
        if abs(y_err) > 0.01:
            linear_y = np.clip(self.kp_y * y_err, -self.max_linear, self.max_linear)
        else:
            linear_y = 0.0
        if abs(angle_err) > 0.1:
            angular_z = np.clip(self.kp_angular * angle_err, -self.max_angular, self.max_angular)
        else:
            angular_z = 0.0

        if linear_x != 0.0 or linear_y != 0.0 or angular_z != 0.0:
            cmd = Twist()
            cmd.linear.x = float(linear_x)
            cmd.linear.y = float(linear_y)
            cmd.angular.z = float(angular_z)
            self.cmd_pub.publish(cmd)
            self.get_logger().info(f"Base pos: {pos_base}, cmd: x={linear_x:.2f}, y={linear_y:.2f}, w={angular_z:.2f}")
        else:
            # 停止小车 
            cmd = Twist()
            self.cmd_pub.publish(cmd)
            
            # 启动按按钮
            msg = Bool()
            msg.data = True
            self.arrived_pub.publish(msg)
            self.get_logger().info("Reached target, notified elevator button node.")
            self.create_timer(1.0, rclpy.shutdown)

    def publish_pos(self, position, quaternion):
            # --- Publish ---
            grasp_msg = PoseStamped()
            grasp_msg.header.stamp = self.get_clock().now().to_msg()
            grasp_msg.header.frame_id = "base_link"
            grasp_msg.pose.position.x = float(position[0])
            grasp_msg.pose.position.y = float(position[1])
            grasp_msg.pose.position.z = float(position[2])
            grasp_msg.pose.orientation.x = float(quaternion[0])
            grasp_msg.pose.orientation.y = float(quaternion[1])
            grasp_msg.pose.orientation.z = float(quaternion[2])
            grasp_msg.pose.orientation.w = float(quaternion[3])

            self.grasp_pub.publish(grasp_msg)
            self.get_logger().info(f"Pose published: "
                                   f"xyz = {position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}, "
                                   f"xyzw = {quaternion[0]:.3f}, {quaternion[1]:.3f}, {quaternion[2]:.3f}, {quaternion[3]:.3f}")

def main(args=None):
    rclpy.init(args=args)
    node = ControlPanelFollower()

    p = np.array([-0.13984, -0.0006, 0.436492])
    q = np.array([0.0071732650039771015, 0.637345889369954, 0.003499715255773396, 0.7705365102093046])
 
    node.publish_pos(p,q)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
