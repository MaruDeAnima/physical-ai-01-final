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
        super().__init__('ai_controller')

        model_path = (
            Path.home() / 'physical_ai_final/scripts/ai_model.pkl'
        )

        with model_path.open('rb') as f:
            self.model = pickle.load(f)

        self.publisher = self.create_publisher(
            Twist, '/cmd_vel', 10
        )
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )

        self.x = None
        self.y = None
        self.yaw = None

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/model/ai_robot/odometry',
            self.odom_callback,
            10
        )

        # FORWARD -> TURN_OUT -> PASS -> TURN_BACK -> FORWARD
        self.state = 'FORWARD'
        self.state_start = None

        # 最初は右に回避する
        self.direction = -1

        # 動作時間（秒）：実際の動きを見て調整する
        self.turn_time = 3.1
        self.pass_time = 10.0

        self.get_logger().info('AI controller started')

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def scan_callback(self, msg):
        if self.state == 'GOAL':
            self.publisher.publish(Twist())
            return

        if self.x is not None and self.y is not None:
            distance = math.hypot(5.0 - self.x, 0.0 - self.y)

            if distance <= 0.45:
                self.state = 'GOAL'
                self.publisher.publish(Twist())
                self.get_logger().info('GOAL REACHED! Robot stopped.')
                return

        if not msg.ranges:
            return

        def sector_distance(angle_min, angle_max):
            values = []

            for i, distance in enumerate(msg.ranges):
                angle = msg.angle_min + i * msg.angle_increment

                if angle_min <= angle <= angle_max:
                    if (
                        math.isfinite(distance)
                        and msg.range_min <= distance <= msg.range_max
                    ):
                        values.append(distance)

            return min(values) if values else msg.range_max

        right = sector_distance(-math.pi / 2, -math.pi / 6)
        front = sector_distance(-math.pi / 6, math.pi / 6)
        left = sector_distance(math.pi / 6, math.pi / 2)

        try:
            prediction = int(
                self.model.predict([[left, front, right]])[0]
            )
        except Exception as e:
            self.get_logger().error(f'AI prediction failed: {e}')
            self.publisher.publish(Twist())
            return

        now = self.get_clock().now().nanoseconds / 1e9
        cmd = Twist()

        if self.state == 'FORWARD':
            if prediction == 1 or front < 0.8:
                self.state = 'TURN_OUT'
                self.state_start = now
            else:
                if self.x is not None and self.y is not None and self.yaw is not None:
                    goal_angle = math.atan2(-self.y, 5.0 - self.x)
                    angle_error = math.atan2(
                        math.sin(goal_angle - self.yaw),
                        math.cos(goal_angle - self.yaw)
                    )

                    cmd.angular.z = max(
                        -0.5, min(0.5, 1.0 * angle_error)
                    )

                    if abs(angle_error) < 0.5:
                        cmd.linear.x = 0.12
                else:
                    cmd.linear.x = 0.0

        if self.state == 'TURN_OUT':
            elapsed = now - self.state_start

            if elapsed < self.turn_time:
                cmd.angular.z = 0.5 * self.direction
            else:
                self.state = 'PASS'
                self.state_start = now

        elif self.state == 'PASS':
            elapsed = now - self.state_start

            if elapsed < self.pass_time:
                cmd.linear.x = 0.12
            else:
                self.state = 'TURN_BACK'
                self.state_start = now

        elif self.state == 'TURN_BACK':
            elapsed = now - self.state_start

            if elapsed < self.turn_time:
                cmd.angular.z = -0.5 * self.direction
            else:
                self.state = 'FORWARD'
                self.state_start = None
                cmd.linear.x = 0.12

        self.publisher.publish(cmd)

        self.get_logger().info(
            f'L={left:.2f} F={front:.2f} R={right:.2f} '
            f'| {self.state} AI={prediction}'
        )


def main():
    rclpy.init()
    node = None

    try:
        node = AIController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Controller error: {e}')
    finally:
        if node is not None:
            node.publisher.publish(Twist())
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
