import math
import time
from typing import Any

import rclpy
from geometry_msgs.msg import AccelStamped, PointStamped
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import (
    BatteryStatus,
    VehicleAttitude,
    VehicleGlobalPosition,
    VehicleLandDetected,
    VehicleLocalPosition,
    VehicleStatus,
)
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float64, Float32
from uav_interfaces.msg import UAVState
from uav_interfaces.msg import TaskStatus
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from uav_info.altitude_monitor import AltitudeMonitor
from uav_info.battery_monitor import BatteryMonitor
from uav_info.coordinate_transform import (
    gps_to_gazebo_enu,
    local_enu_to_map_rotation,
    local_ned_to_map,
    local_origin_to_map,
    multiply_quaternions,
    ned_to_enu,
    px4_attitude_to_enu_flu,
    rotation_matrix_to_quaternion,
    rotate_vector,
)
from uav_info.vehicle_health import VehicleHealth
from uav_info.vehicle_state import VehicleState


class PX4Interface(Node):
    def __init__(self):
        super().__init__('uav_info')
        self._declare_parameters()
        self.uav_id = (
            self._parameter('uav.id') or self.get_namespace().strip('/')
        )
        if not self.uav_id:
            raise ValueError(
                'Set uav.id or launch this node in a UAV namespace'
            )

        self.world_frame = self._parameter('map.frame_id')
        self.local_frame = self._parameter('frames.local')
        self.body_frame = self._parameter('frames.body')
        self.reference = (
            self._parameter('map.reference_latitude'),
            self._parameter('map.reference_longitude'),
            self._parameter('map.reference_altitude'),
        )
        if (
            not all(math.isfinite(float(value)) for value in self.reference)
            or not -90.0 <= self.reference[0] <= 90.0
            or not -180.0 <= self.reference[1] <= 180.0
        ):
            raise ValueError('Map WGS84 reference is invalid')
        self.state = VehicleState(self.uav_id)
        self.battery_monitor = BatteryMonitor(
            self._parameter('battery.initial_percentage'),
            self._parameter('battery.consumption_per_meter'),
            self._parameter('battery.minimum_reserve_percentage'),
        )
        if self._parameter('battery.model') != 'linear_distance':
            raise ValueError(
                'Only the linear_distance battery model is supported'
            )
        self.altitude_monitor = AltitudeMonitor(
            self._parameter('altitude.minimum'),
            self._parameter('altitude.maximum'),
        )
        self.vehicle_health = VehicleHealth(
            self._parameter('health.telemetry_timeout'),
            self._parameter('battery.minimum_reserve_percentage'),
        )

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        reliable_qos = QoSProfile(depth=1)
        self._create_px4_subscription(
            'topics.global_position',
            VehicleGlobalPosition,
            self.global_position_callback,
            sensor_qos,
        )
        self._create_px4_subscription(
            'topics.local_position',
            VehicleLocalPosition,
            self.local_position_callback,
            sensor_qos,
        )
        self._create_px4_subscription(
            'topics.attitude',
            VehicleAttitude,
            self.attitude_callback,
            sensor_qos,
        )
        self._create_px4_subscription(
            'topics.battery_status',
            BatteryStatus,
            self.battery_callback,
            sensor_qos,
        )
        self._create_px4_subscription(
            'topics.vehicle_status',
            VehicleStatus,
            self.vehicle_status_callback,
            sensor_qos,
        )
        self._create_px4_subscription(
            'topics.land_detected',
            VehicleLandDetected,
            self.land_detected_callback,
            sensor_qos,
        )
        self.path_distance_subscription = self.create_subscription(
            Float64,
            self._parameter('topics.path_distance'),
            self.path_distance_callback,
            sensor_qos,
        )

        self.state_publisher = self.create_publisher(
            UAVState, 'uav_state', reliable_qos
        )
        self.task_status_subscription = self.create_subscription(
            TaskStatus, 'task_status', self.task_status_callback, reliable_qos
        )
        self.task_statuses = {}
        mission_task_status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1000,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.mission_task_status_subscription = self.create_subscription(
            TaskStatus,
            '/mission_manager/task_status',
            self.mission_task_status_callback,
            mission_task_status_qos,
        )
        self.odometry_publisher = self.create_publisher(
            Odometry, 'odom', reliable_qos
        )
        self.local_position_publisher = self.create_publisher(
            PointStamped, 'local_position', reliable_qos
        )
        self.acceleration_publisher = self.create_publisher(
            AccelStamped, 'acceleration', reliable_qos
        )
        self.altitude_publisher = self.create_publisher(
            PointStamped, 'altitude', reliable_qos
        )
        self.height_above_ground_publisher = self.create_publisher(
            PointStamped, 'height_above_ground', reliable_qos
        )
        self.gps_position_publisher = self.create_publisher(
            PointStamped, 'gps_position', reliable_qos
        )
        self.measured_battery_publisher = self.create_publisher(
            BatteryState, 'battery/measured', reliable_qos
        )
        self.estimated_battery_publisher = self.create_publisher(
            BatteryState, 'battery/estimated', reliable_qos
        )
        self.estimated_range_publisher = self.create_publisher(
            Float32, 'battery/estimated_range', reliable_qos
        )
        self.transform_broadcaster = TransformBroadcaster(self)
        self.static_transform_broadcaster = StaticTransformBroadcaster(self)
        self._logged_map_pose = False
        self._publish_sensor_transform()

        self.timer = self.create_timer(
            1.0 / self._parameter('telemetry.publish_rate'),
            self.publish_state,
        )
        self.last_operational = None
        self.last_altitude_warning = None
        self.last_path_distance_warning_at = 0.0
        angular_topic = self._parameter('topics.angular_velocity')
        if angular_topic:
            self.get_logger().warning(
                'PX4 does not expose VehicleAngularVelocity in the current '
                'DDS configuration; angular velocity will not be published.'
            )
        self.get_logger().info(
            f"Publishing telemetry for '{self.uav_id}' in namespace "
            f"'{self.get_namespace()}'."
        )

    def _declare_parameters(self):
        defaults = {
            'uav.id': '',
            'frames.local': 'odom',
            'frames.body': 'base_link',
            'topics.global_position': 'fmu/out/vehicle_global_position',
            'topics.local_position': 'fmu/out/vehicle_local_position_v1',
            'topics.attitude': 'fmu/out/vehicle_attitude',
            'topics.angular_velocity': '',
            'topics.battery_status': 'fmu/out/battery_status_v1',
            'topics.vehicle_status': 'fmu/out/vehicle_status_v4',
            'topics.land_detected': 'fmu/out/vehicle_land_detected',
            'topics.path_distance': 'path_distance',
            'map.frame_id': 'map',
            'map.reference_latitude': 0.0,
            'map.reference_longitude': 0.0,
            'map.reference_altitude': 0.0,
            'battery.model': 'linear_distance',
            'battery.initial_percentage': 100.0,
            'battery.consumption_per_meter': 0.05,
            'battery.minimum_reserve_percentage': 20.0,
            'altitude.minimum': 2.0,
            'altitude.maximum': 30.0,
            'health.telemetry_timeout': 2.0,
            'telemetry.publish_rate': 5.0,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)

    def _parameter(self, name: str) -> Any:
        return self.get_parameter(name).value

    def _create_px4_subscription(
        self,
        topic_parameter: str,
        message_type: type,
        callback: Any,
        qos: QoSProfile,
    ):
        topic = self._parameter(topic_parameter)
        return self.create_subscription(message_type, topic, callback, qos)

    def global_position_callback(self, message):
        self.state.update('global_position', message)

    def local_position_callback(self, message):
        self.state.update('local_position', message)

    def attitude_callback(self, message):
        self.state.update('attitude', message)

    def battery_callback(self, message):
        if (
            message.connected
            and math.isfinite(message.remaining)
            and 0.0 <= message.remaining <= 1.0
        ):
            self.battery_monitor.update_measured_percentage(
                float(message.remaining) * 100.0
            )
        self.state.update('battery_status', message)

    def vehicle_status_callback(self, message):
        self.state.update('vehicle_status', message)

    def land_detected_callback(self, message):
        self.state.update('land_detected', message)

    def path_distance_callback(self, message):
        if not self.battery_monitor.update_path_distance(message.data):
            now = time.monotonic()
            if now - self.last_path_distance_warning_at >= 5.0:
                self.get_logger().warning(
                    'Ignored invalid cumulative path distance '
                    f'{message.data!r}; expected a finite non-negative value.'
                )
                self.last_path_distance_warning_at = now

    def publish_state(self):
        connected, operational = self.vehicle_health.evaluate(self.state)
        land_detected = self.state.get('land_detected')
        if (
            self.state.mission_status in (UAVState.FAILED, UAVState.LANDING)
            and land_detected is not None
            and self.state.is_fresh(
                'land_detected', self.vehicle_health.telemetry_timeout_sec
            )
            and land_detected.landed
        ):
            self.state.mission_status = UAVState.LANDED
        self._publish_uav_state(connected, operational)
        self._publish_kinematics()
        self._publish_batteries()

        if operational != self.last_operational:
            self.last_operational = operational
            self.get_logger().info(
                f"UAV '{self.uav_id}' operational={operational}."
            )

    def _publish_uav_state(self, connected, operational):
        message = UAVState()
        message.uav_id = self.uav_id
        message.connected = connected
        message.operational = operational
        land_detected = self.state.get('land_detected')
        message.landed = bool(
            land_detected is not None
            and self.state.is_fresh(
                'land_detected', self.vehicle_health.telemetry_timeout_sec
            )
            and land_detected.landed
        )
        message.mission_status = self.state.mission_status
        message.assigned_task_id = self.state.assigned_task_id
        self.state_publisher.publish(message)

    def _publish_kinematics(self):
        timeout = self.vehicle_health.telemetry_timeout_sec
        global_position = self.state.get('global_position')
        local_position = self.state.get('local_position')
        attitude = self.state.get('attitude')
        if (
            global_position is None
            or local_position is None
            or attitude is None
        ):
            return
        if not all((
            self.state.is_fresh('global_position', timeout),
            self.state.is_fresh('local_position', timeout),
            self.state.is_fresh('attitude', timeout),
        )):
            return
        if not (
            global_position.lat_lon_valid
            and global_position.alt_valid
            and local_position.xy_valid
            and local_position.z_valid
            and local_position.v_xy_valid
            and local_position.v_z_valid
            and local_position.xy_global
            and local_position.z_global
        ):
            return
        local_reference = (
            float(local_position.ref_lat),
            float(local_position.ref_lon),
            float(local_position.ref_alt),
        )
        if (
            not all(math.isfinite(value) for value in local_reference)
            or not -90.0 <= local_reference[0] <= 90.0
            or not -180.0 <= local_reference[1] <= 180.0
        ):
            return
        local_origin_map = tuple(float(value) for value in local_origin_to_map(
            *local_reference, *self.reference
        ))
        enu_map_rotation = local_enu_to_map_rotation(
            local_reference[0], local_reference[1],
            self.reference[0], self.reference[1],
        )
        ned_position = (
            float(local_position.x),
            float(local_position.y),
            float(local_position.z),
        )
        map_position = local_ned_to_map(
            *ned_position, *local_reference, *self.reference
        )
        east, north, up = (float(value) for value in map_position)
        if not self._logged_map_pose:
            self._logged_map_pose = True
            self.get_logger().info(
                f'{self.uav_id} PX4 local NED {ned_position} at '
                f'{local_reference} -> {self.world_frame} ENU '
                f'({east:.3f}, {north:.3f}, {up:.3f})'
            )
        orientation_local = tuple(
            float(value) for value in px4_attitude_to_enu_flu(attitude.q)
        )
        orientation_map = multiply_quaternions(
            rotation_matrix_to_quaternion(enu_map_rotation),
            orientation_local,
        )
        velocity_enu = tuple(float(value) for value in ned_to_enu((
            local_position.vx,
            local_position.vy,
            local_position.vz,
        )))
        inverse_orientation = (
            orientation_local[0],
            -orientation_local[1],
            -orientation_local[2],
            -orientation_local[3],
        )
        velocity_flu = tuple(float(value) for value in rotate_vector(
            inverse_orientation, velocity_enu
        ))

        now = self.get_clock().now().to_msg()
        odometry = Odometry()
        odometry.header.stamp = now
        odometry.header.frame_id = self.world_frame
        odometry.child_frame_id = self.body_frame
        odometry.pose.pose.position.x = east
        odometry.pose.pose.position.y = north
        odometry.pose.pose.position.z = up
        (
            odometry.pose.pose.orientation.w,
            odometry.pose.pose.orientation.x,
            odometry.pose.pose.orientation.y,
            odometry.pose.pose.orientation.z,
        ) = orientation_map
        odometry.twist.twist.linear.x = velocity_flu[0]
        odometry.twist.twist.linear.y = velocity_flu[1]
        odometry.twist.twist.linear.z = velocity_flu[2]
        odometry.pose.covariance[0] = float(global_position.eph**2)
        odometry.pose.covariance[7] = float(global_position.eph**2)
        odometry.pose.covariance[14] = float(global_position.epv**2)
        odometry.twist.covariance[0] = float(local_position.evh**2)
        odometry.twist.covariance[7] = float(local_position.evh**2)
        odometry.twist.covariance[14] = float(local_position.evv**2)
        self.odometry_publisher.publish(odometry)
        map_to_local = TransformStamped()
        map_to_local.header.stamp = now
        map_to_local.header.frame_id = self.world_frame
        map_to_local.child_frame_id = self.local_frame
        (
            map_to_local.transform.translation.x,
            map_to_local.transform.translation.y,
            map_to_local.transform.translation.z,
        ) = local_origin_map
        (
            map_to_local.transform.rotation.w,
            map_to_local.transform.rotation.x,
            map_to_local.transform.rotation.y,
            map_to_local.transform.rotation.z,
        ) = rotation_matrix_to_quaternion(enu_map_rotation)

        local_to_body = TransformStamped()
        local_to_body.header.stamp = now
        local_to_body.header.frame_id = self.local_frame
        local_to_body.child_frame_id = self.body_frame
        local_to_body.transform.translation.x = float(local_position.y)
        local_to_body.transform.translation.y = float(local_position.x)
        local_to_body.transform.translation.z = -float(local_position.z)
        (
            local_to_body.transform.rotation.w,
            local_to_body.transform.rotation.x,
            local_to_body.transform.rotation.y,
            local_to_body.transform.rotation.z,
        ) = orientation_local
        self.transform_broadcaster.sendTransform(
            [map_to_local, local_to_body]
        )

        acceleration_enu = tuple(float(value) for value in ned_to_enu((
            local_position.ax,
            local_position.ay,
            local_position.az,
        )))
        local_enu = tuple(float(value) for value in ned_to_enu((
            local_position.x,
            local_position.y,
            local_position.z,
        )))
        local_point = PointStamped()
        local_point.header.stamp = now
        local_point.header.frame_id = self.local_frame
        local_point.point.x = local_enu[0]
        local_point.point.y = local_enu[1]
        local_point.point.z = local_enu[2]
        self.local_position_publisher.publish(local_point)

        acceleration = AccelStamped()
        acceleration.header.stamp = now
        acceleration_map = tuple(
            float(value) for value in (
                sum(enu_map_rotation[row][column] * acceleration_enu[column]
                    for column in range(3))
                for row in range(3)
            )
        )
        acceleration.header.frame_id = self.world_frame
        acceleration.accel.linear.x = acceleration_map[0]
        acceleration.accel.linear.y = acceleration_map[1]
        acceleration.accel.linear.z = acceleration_map[2]
        self.acceleration_publisher.publish(acceleration)

        altitude = PointStamped()
        altitude.header.stamp = now
        altitude.header.frame_id = self.world_frame
        altitude.point.x = east
        altitude.point.y = north
        altitude.point.z = up
        self.altitude_publisher.publish(altitude)
        self._log_altitude_limit(up)

        gps_position = PointStamped()
        gps_position.header.stamp = now
        gps_position.header.frame_id = self.world_frame
        (
            gps_position.point.x,
            gps_position.point.y,
            gps_position.point.z,
        ) = tuple(float(value) for value in gps_to_gazebo_enu(
            global_position.lat,
            global_position.lon,
            global_position.alt,
            *self.reference,
        ))
        self.gps_position_publisher.publish(gps_position)

        if local_position.dist_bottom_valid:
            height = PointStamped()
            height.header.stamp = now
            height.header.frame_id = self.world_frame
            height.point.x = east
            height.point.y = north
            height.point.z = float(local_position.dist_bottom)
            self.height_above_ground_publisher.publish(height)

    def task_status_callback(self, message):
        if message.uav_id != self.uav_id:
            return
        if message.status not in (
            UAVState.IDLE,
            UAVState.ASSIGNED,
            UAVState.EXECUTING,
            UAVState.INSPECTING,
            UAVState.QUEUED,
            UAVState.COMPLETED,
            UAVState.RETURNING,
            UAVState.FAILED,
            UAVState.LANDING,
            UAVState.LANDED,
            UAVState.RECOVERING,
        ):
            return
        self.state.mission_status = message.status
        self.state.assigned_task_id = (
            '' if message.status in (
                UAVState.IDLE, UAVState.COMPLETED, UAVState.LANDED
            )
            else message.task_id
        )

    def mission_task_status_callback(self, message):
        if message.task_id:
            self.task_statuses[message.task_id] = message

    def _publish_sensor_transform(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.body_frame
        transform.child_frame_id = f'{self.uav_id}/lidar_link'
        transform.transform.translation.x = 0.02
        transform.transform.translation.z = 0.315
        transform.transform.rotation.w = 1.0
        self.static_transform_broadcaster.sendTransform(transform)

    def _log_altitude_limit(self, altitude_m):
        within_limits = self.altitude_monitor.within_limits(altitude_m)
        if within_limits == self.last_altitude_warning:
            return
        self.last_altitude_warning = within_limits
        if not within_limits:
            self.get_logger().warning(
                f'Altitude {altitude_m:.2f} m is outside configured limits.'
            )

    def _publish_batteries(self):
        timeout = self.vehicle_health.telemetry_timeout_sec
        battery = self.state.get('battery_status')
        if (
            battery is not None
            and self.state.is_fresh('battery_status', timeout)
            and battery.connected
            and 0.0 <= battery.remaining <= 1.0
        ):
            measured = BatteryState()
            measured.header.stamp = self.get_clock().now().to_msg()
            measured.header.frame_id = self.body_frame
            measured.voltage = battery.voltage_v
            measured.current = (
                battery.current_a
                if battery.current_a >= 0.0
                else math.nan
            )
            measured.percentage = battery.remaining
            measured.present = True
            measured.power_supply_status = (
                BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            )
            self.measured_battery_publisher.publish(measured)

        estimated = BatteryState()
        estimated.header.stamp = self.get_clock().now().to_msg()
        estimated.header.frame_id = self.body_frame
        estimated.percentage = (
            self.battery_monitor.estimated_percentage() / 100.0
        )
        estimated.present = True
        estimated.power_supply_status = (
            BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        )
        self.estimated_battery_publisher.publish(estimated)

        remaining_range = Float32()
        remaining_range.data = self.battery_monitor.estimated_range_m()
        if math.isfinite(remaining_range.data):
            self.estimated_range_publisher.publish(remaining_range)


def main(args=None):
    rclpy.init(args=args)
    node = PX4Interface()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
