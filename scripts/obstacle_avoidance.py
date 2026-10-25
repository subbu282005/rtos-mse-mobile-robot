#!/usr/bin/env python3

#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')

        self.declare_parameter('forward_speed', 0.12)
        self.declare_parameter('turn_speed', 0.5)
        self.declare_parameter('caution_distance', 0.8)
        self.declare_parameter('stop_distance', 0.45)

        self.forward_speed = float(
            self.get_parameter('forward_speed').value)
        self.turn_speed = float(
            self.get_parameter('turn_speed').value)
        self.caution_distance = float(
            self.get_parameter('caution_distance').value)
        self.stop_distance = float(
            self.get_parameter('stop_distance').value)

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.last_scan_time = self.get_clock().now()
        self.turn_direction = 1.0
        self.get_logger().info('Obstacle avoidance controller started')

        self.watchdog = self.create_timer(0.2, self.check_scan_timeout)

    def sector_min(self, scan, start_deg, end_deg):
        """Return the closest valid obstacle in an angular sector."""
        values = []

        for i, distance in enumerate(scan.ranges):
            angle = scan.angle_min + i * scan.angle_increment
            angle_deg = math.degrees(angle)

            if start_deg <= angle_deg <= end_deg:
                if (math.isfinite(distance)
                        and scan.range_min <= distance <= scan.range_max):
                    values.append(distance)

        return min(values) if values else float('inf')

    def publish_stop(self):
        self.cmd_pub.publish(Twist())

    def scan_callback(self, scan):
        self.last_scan_time = self.get_clock().now()

        # Angles assume the conventional LaserScan orientation:
        # 0 degrees is forward, positive angles are to the left.
        front = self.sector_min(scan, -25.0, 25.0)
        left = self.sector_min(scan, 25.0, 90.0)
        right = self.sector_min(scan, -90.0, -25.0)

        cmd = Twist()

        if front <= self.stop_distance:
            # Never command forward motion while too close to an obstacle.
            cmd.linear.x = 0.0

            if left > right:
                self.turn_direction = 1.0
            elif right > left:
                self.turn_direction = -1.0
            else:
                # Both sides appear equally blocked: rotate cautiously
                # and continue checking the scan.
                self.turn_direction = 1.0

            if left <= self.stop_distance and right <= self.stop_distance:
                cmd.angular.z = self.turn_direction * 0.25
            else:
                cmd.angular.z = self.turn_direction * self.turn_speed

        elif front <= self.caution_distance:
            cmd.linear.x = 0.05

            if left > right:
                cmd.angular.z = 0.2
            elif right > left:
                cmd.angular.z = -0.2
            else:
                cmd.angular.z = 0.0

        else:
            cmd.linear.x = self.forward_speed
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

    def check_scan_timeout(self):
        age = (self.get_clock().now() - self.last_scan_time).nanoseconds / 1e9

        if age > 1.0:
            self.get_logger().warn(
                'No recent LiDAR scan; publishing stop command',
                throttle_duration_sec=2.0)
            self.publish_stop()


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
