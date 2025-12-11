#!/usr/bin/env python3
"""
ROS2 Path Follower (rclpy)

- Subscribes: /global_path  (nav_msgs/Path)
              /amcl_pose    (geometry_msgs/PoseWithCovarianceStamped)
- Publishes: /cmd_vel      (geometry_msgs/Twist or TwistStamped; auto-detect / parameter)
- Controller: rotate-then-go with small heading correction
"""
import os
import math
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from geometry_msgs.msg import Twist, TwistStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')

        # Parameters (tweak these)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('use_stamped', 'auto')   # 'auto', True, or False
        self.declare_parameter('rate', 10.0)            # Hz control loop
        self.declare_parameter('k_lin', 0.8)
        self.declare_parameter('k_ang', 1.5)
        self.declare_parameter('max_lin', 0.22)
        self.declare_parameter('max_ang', 1.0)
        self.declare_parameter('pos_tolerance', 0.06)
        self.declare_parameter('ang_tolerance', 0.12)
        self.declare_parameter('yaw_scale_while_driving', 0.8)  # scale yaw correction while moving forward

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        use_stamped_param = self.get_parameter('use_stamped').get_parameter_value().string_value
        self.rate_hz = float(self.get_parameter('rate').get_parameter_value().double_value)
        self.k_lin = float(self.get_parameter('k_lin').get_parameter_value().double_value)
        self.k_ang = float(self.get_parameter('k_ang').get_parameter_value().double_value)
        self.max_lin = float(self.get_parameter('max_lin').get_parameter_value().double_value)
        self.max_ang = float(self.get_parameter('max_ang').get_parameter_value().double_value)
        self.pos_tol = float(self.get_parameter('pos_tolerance').get_parameter_value().double_value)
        self.ang_tol = float(self.get_parameter('ang_tolerance').get_parameter_value().double_value)
        self.yaw_scale_while_driving = float(self.get_parameter('yaw_scale_while_driving').get_parameter_value().double_value)

        # Decide whether to publish Twist or TwistStamped
        self.use_stamped = None
        if use_stamped_param in ('True', 'true', '1'):
            self.use_stamped = True
        elif use_stamped_param in ('False', 'false', '0'):
            self.use_stamped = False
        else:
            # auto-detect by checking if any topic with this name exists and its type
            topics = self.get_topic_names_and_types()
            match = [t for (n, t) in topics if n == self.cmd_vel_topic]
            if match:
                # match is a list of lists of types; take first if present
                types = match[0]
                if 'geometry_msgs/msg/TwistStamped' in types:
                    self.use_stamped = True
                elif 'geometry_msgs/msg/Twist' in types:
                    self.use_stamped = False
            # fallback based on ROS_DISTRO (common convention used earlier)
            if self.use_stamped is None:
                ros_distro = os.environ.get('ROS_DISTRO', '')
                # Historically some teleop nodes published Twist for humble, and TwistStamped for some newer distros;
                # default to TwistStamped for non-humble to be safe for jazzy.
                self.use_stamped = False if ros_distro == 'humble' else True

        # Publisher
        qos = QoSProfile(depth=10)
        if self.use_stamped:
            self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_vel_topic, qos)
            self.get_logger().info(f'Publishing TwistStamped on {self.cmd_vel_topic}')
        else:
            self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, qos)
            self.get_logger().info(f'Publishing Twist on {self.cmd_vel_topic}')

        # Subscribers
        self.create_subscription(Path, '/global_path', self.path_cb, qos)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_cb, qos)

        # internal state
        self.path: List[Tuple[float, float]] = []
        self.path_index = 0
        self.pose: Optional[Tuple[float, float, float]] = None  # (x, y, yaw)

        # timer for control loop
        self.timer = self.create_timer(1.0 / self.rate_hz, self.control_loop)

        self.get_logger().info('PathFollower ready. Waiting for /global_path and /amcl_pose...')

    # -------------------------
    # Callbacks
    # -------------------------
    def path_cb(self, msg: Path):
        # convert Path -> list of (x,y)
        pts = []
        for p in msg.poses:
            pts.append((p.pose.position.x, p.pose.position.y))
        self.path = pts
        self.path_index = 0
        self.get_logger().info(f'Received path with {len(self.path)} waypoints.')

    def pose_cb(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.pose = (p.x, p.y, yaw)

    # -------------------------
    # Main control loop
    # -------------------------
    def control_loop(self):
        if self.pose is None or not self.path:
            # ensure robot is stopped when no task or no pose
            self.publish_stop()
            return

        # if path_index beyond len -> stop
        if self.path_index >= len(self.path):
            self.get_logger().info('Reached end of path — stopping.')
            self.publish_stop()
            return

        cur_x, cur_y, cur_yaw = self.pose
        tgt_x, tgt_y = self.path[self.path_index]

        dx = tgt_x - cur_x
        dy = tgt_y - cur_y
        dist = math.hypot(dx, dy)
        target_yaw = math.atan2(dy, dx)
        yaw_error = self.angle_diff(target_yaw, cur_yaw)

        # decide action
        cmd_lin = 0.0
        cmd_ang = 0.0

        # big yaw error -> turn in place (but only if not very close)
        if abs(yaw_error) > self.ang_tol and dist > self.pos_tol:
            cmd_lin = 0.0
            cmd_ang = self.k_ang * yaw_error
        elif dist > self.pos_tol:
            # go forward, apply smaller yaw correction while driving
            cmd_lin = self.k_lin * dist
            cmd_ang = self.k_ang * yaw_error * self.yaw_scale_while_driving
        else:
            # waypoint reached
            self.get_logger().debug(f'Waypoint {self.path_index} reached (dist {dist:.3f}). Advancing.')
            self.path_index += 1
            # Immediately stop briefly to let AMCL settle
            self.publish_stop()
            return

        # clamp commands
        cmd_lin = max(-self.max_lin, min(self.max_lin, cmd_lin))
        cmd_ang = max(-self.max_ang, min(self.max_ang, cmd_ang))

        # publish
        self.publish_cmd(cmd_lin, cmd_ang)

    # -------------------------
    # Helpers
    # -------------------------
    def publish_cmd(self, linear_x: float, angular_z: float):
        if self.use_stamped:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = ''
            msg.twist.linear.x = float(linear_x)
            msg.twist.angular.z = float(angular_z)
            self.cmd_pub.publish(msg)
        else:
            msg = Twist()
            msg.linear.x = float(linear_x)
            msg.angular.z = float(angular_z)
            self.cmd_pub.publish(msg)

    def publish_stop(self):
        # publish zero once
        if self.use_stamped:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = ''
            # zeros are default
            self.cmd_pub.publish(msg)
        else:
            msg = Twist()
            self.cmd_pub.publish(msg)

    @staticmethod
    def quaternion_to_yaw(x, y, z, w) -> float:
        # yaw from quaternion
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def angle_diff(a: float, b: float) -> float:
        d = a - b
        while d > math.pi:
            d -= 2.0 * math.pi
        while d < -math.pi:
            d += 2.0 * math.pi
        return d


def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down path_follower — stopping robot.')
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
