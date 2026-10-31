import math
import pickle
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class AIController(Node):
    def __init__(self):
        super().__init__('ai_controller_torii')

        model_path = (
            Path.home() / 'physical_ai_final/scripts/ai_model.pkl'
        )
        with model_path.open('rb') as f:
            self.model = pickle.load(f)

        self.publisher = self.create_publisher(
            Twist, '/cmd_vel', 10
        )

        self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.create_subscription(
            Odometry,
            '/model/ai_robot/odometry',
            self.odom_callback,
            10
        )

        self.x = None
        self.y = None
        self.yaw = None

        # 富士山を右側から迂回し、鳥居を通過する
        self.waypoints = [
            (1.2, -1.2),
            (2.8, -1.2),
            (4.0, 0.0),
            (5.7, 0.0),
            (6.2, 0.0),
        ]
        self.target_index = 0
        self.finished = False

        self.get_logger().info(
            'Torii navigation controller started'
        )

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.yaw = math.atan2(
            2.0 * (q.w*q.z + q.x*q.y),
            1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        )

    def scan_callback(self, msg):
        cmd = Twist()

        if self.finished:
            self.publisher.publish(cmd)
            return

        if None in (self.x, self.y, self.yaw):
            self.publisher.publish(cmd)
            return

        # ウェイポイントに到着したら次へ
        while self.target_index < len(self.waypoints):
            tx, ty = self.waypoints[self.target_index]
            distance = math.hypot(
                tx - self.x, ty - self.y
            )

            if distance > 0.20:
                break

            self.target_index += 1
            self.get_logger().info(
                f'Waypoint reached: {self.target_index}'
            )

        if self.target_index >= len(self.waypoints):
            self.finished = True
            self.publisher.publish(cmd)
            self.get_logger().info(
                'GOAL REACHED! Passed through torii.'
            )
            return

        tx, ty = self.waypoints[self.target_index]

        target_angle = math.atan2(
            ty - self.y, tx - self.x
        )

        error = math.atan2(
            math.sin(target_angle - self.yaw),
            math.cos(target_angle - self.yaw)
        )

        # 旋回を優先し、向きが合ったら前進
        cmd.angular.z = max(
            -0.45, min(0.45, 1.5 * error)
        )

        if abs(error) < 0.25:
            cmd.linear.x = 0.12
        elif abs(error) < 0.6:
            cmd.linear.x = 0.06

        # LiDARによる緊急停止
        front_ranges = []
        for i, value in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment

            if (
                abs(angle) < 0.35
                and math.isfinite(value)
                and msg.range_min <= value <= msg.range_max
            ):
                front_ranges.append(value)

        if front_ranges and min(front_ranges) < 0.45:
            cmd.linear.x = 0.0
            self.get_logger().warn(
                'Obstacle too close: stopping forward motion',
                throttle_duration_sec=2.0
            )

        self.publisher.publish(cmd)


def main():
    rclpy.init()
    node = AIController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
