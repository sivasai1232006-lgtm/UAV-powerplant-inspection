from copy import copy

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


class ScanLayerRelay(Node):
    def __init__(self):
        super().__init__('scan_layer_relay')
        self.declare_parameter('uav_id', '')
        self.declare_parameter('altitude_clearance', 0.9)
        self.uav_id = str(self.get_parameter('uav_id').value)
        self.altitude_clearance = float(
            self.get_parameter('altitude_clearance').value
        )
        self.altitude = None
        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10,
        )
        self.odom_subscription = self.create_subscription(
            Odometry, 'odom', self._odometry_callback, 10
        )
        self.scan_subscription = self.create_subscription(
            LaserScan, 'scan/raw', self._scan_callback, scan_qos
        )
        self.scan_publishers = {
            altitude: self.create_publisher(
                LaserScan,
                f'/planner_{altitude}m/{self.uav_id}/scan',
                scan_qos,
            )
            for altitude in (8, 10)
        }

    def _odometry_callback(self, message):
        self.altitude = float(message.pose.pose.position.z)

    def _scan_callback(self, message):
        if self.altitude is None:
            return
        scan = copy(message)
        scan.header.frame_id = f'{self.uav_id}/lidar_link'
        for altitude, publisher in self.scan_publishers.items():
            if abs(self.altitude - altitude) <= self.altitude_clearance:
                publisher.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = ScanLayerRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
