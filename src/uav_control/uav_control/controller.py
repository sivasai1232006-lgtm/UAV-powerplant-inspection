import math
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)
from uav_interfaces.msg import TaskStatus, UAVState
from uav_interfaces.action import GoToWaypoint
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException  # pyright: ignore[reportAttributeAccessIssue]
from uav_info.coordinate_transform import enu_to_ned


def is_emergency_landing_request(task_id, status):
    return (
        task_id == 'EMERGENCY_RETURN'
        and status == UAVState.FAILED
    )


def is_controlled_landing_request(task_id, status):
    return task_id == 'LANDING' and status == UAVState.LANDING


def calculate_velocity_setpoint(
    error,
    velocity_gain,
    active_max_velocity,
    max_velocity_limit,
    horizontal_multiplier,
    forward_speed_multiplier=1.5,
):
    """Scale horizontal speed while preserving direction and vertical speed."""
    horizontal_limit = min(
        active_max_velocity * horizontal_multiplier,
        max_velocity_limit,
    )
    horizontal_speed = math.hypot(error[0], error[1]) * velocity_gain
    if horizontal_speed > horizontal_limit:
        scale = horizontal_limit / horizontal_speed
        horizontal = (
            error[0] * scale,
            error[1] * scale,
        )
    else:
        horizontal = (error[0], error[1])
    horizontal_command = (
        horizontal[0] * velocity_gain * forward_speed_multiplier,
        horizontal[1] * velocity_gain * forward_speed_multiplier,
    )
    command_speed = math.hypot(*horizontal_command)
    if command_speed > max_velocity_limit:
        scale = max_velocity_limit / command_speed
        horizontal_command = tuple(
            value * scale for value in horizontal_command
        )
    return (
        horizontal_command[0],
        horizontal_command[1],
        max(-active_max_velocity, min(active_max_velocity,
            error[2] * velocity_gain)),
    )


class OffboardController(Node):

    def __init__(self):
        super().__init__('offboard_controller')

        # Explicit parameter types prevent ROS 2 parameter type errors.
        self.declare_parameter('target_x', 0.0)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('target_z', -3.0)
        self.declare_parameter('target_yaw', 0.0)
        self.declare_parameter('target_system', 1)
        self.declare_parameter('start_enabled', False)
        self.declare_parameter('map.frame_id', 'map')
        self.declare_parameter('map.reference_latitude', 0.0)
        self.declare_parameter('map.reference_longitude', 0.0)
        self.declare_parameter('map.reference_altitude', 0.0)
        self.declare_parameter('local_frame_id', '')
        self.declare_parameter('body_frame_id', '')
        self.declare_parameter('telemetry_timeout_sec', 2.0)
        self.declare_parameter('goal_timeout_sec', 180.0)
        self.declare_parameter('velocity_gain', 1.0)
        self.declare_parameter('max_velocity_limit', 5.0)
        self.declare_parameter('horizontal_velocity_multiplier', 1.2)
        self.declare_parameter('forward_speed_multiplier', 1.5)
        self.declare_parameter('operating_bounds.enabled', False)
        self.declare_parameter('operating_bounds.x_min', 0.0)
        self.declare_parameter('operating_bounds.x_max', 0.0)
        self.declare_parameter('operating_bounds.y_min', 0.0)
        self.declare_parameter('operating_bounds.y_max', 0.0)
        self.declare_parameter('operating_bounds.z_min', 0.0)
        self.declare_parameter('operating_bounds.z_max', 10.0)

        # Read parameter values and explicitly cast their types.
        self.target_x: float = float(
            self.get_parameter('target_x').get_parameter_value().double_value
        )
        self.target_y: float = float(
            self.get_parameter('target_y').get_parameter_value().double_value
        )
        self.target_z: float = float(
            self.get_parameter('target_z').get_parameter_value().double_value
        )
        self.target_yaw: float = float(
            self.get_parameter('target_yaw').get_parameter_value().double_value
        )
        self.target_system: int = int(
            self.get_parameter(
                'target_system'
            ).get_parameter_value().integer_value
        )
        self.start_enabled: bool = bool(
            self.get_parameter(
                'start_enabled'
            ).get_parameter_value().bool_value
        )
        self.map_frame_id = str(
            self.get_parameter(
                'map.frame_id'
            ).get_parameter_value().string_value
        )
        namespace_id = self.get_namespace().strip('/')
        self.local_frame_id = str(
            self.get_parameter(
                'local_frame_id'
            ).get_parameter_value().string_value
            or f'{namespace_id}/odom'
        )
        self.body_frame_id = str(
            self.get_parameter(
                'body_frame_id'
            ).get_parameter_value().string_value
            or f'{namespace_id}/base_link'
        )
        self.telemetry_timeout_sec = float(
            self.get_parameter(
                'telemetry_timeout_sec'
            ).get_parameter_value().double_value
        )
        if (
            not self.map_frame_id
            or not self.local_frame_id
            or not self.body_frame_id
            or self.telemetry_timeout_sec <= 0.0
        ):
            raise ValueError('Map frame configuration is invalid')
        self.goal_timeout_sec: float = (
            self.get_parameter(
                'goal_timeout_sec'
            ).get_parameter_value().double_value
        )
        self.velocity_gain: float = (
            self.get_parameter(
                'velocity_gain'
            ).get_parameter_value().double_value
        )
        self.max_velocity_limit: float = (
            self.get_parameter(
                'max_velocity_limit'
            ).get_parameter_value().double_value
        )
        self.horizontal_velocity_multiplier = float(
            self.get_parameter(
                'horizontal_velocity_multiplier'
            ).get_parameter_value().double_value
        )
        self.forward_speed_multiplier = float(
            self.get_parameter(
                'forward_speed_multiplier'
            ).get_parameter_value().double_value
        )
        if (
            not math.isfinite(self.horizontal_velocity_multiplier)
            or self.horizontal_velocity_multiplier <= 0.0
            or not math.isfinite(self.forward_speed_multiplier)
            or self.forward_speed_multiplier <= 0.0
        ):
            raise ValueError(
                'Horizontal and forward speed multipliers must be finite '
                'and positive'
            )

        if not all(math.isfinite(v) for v in (
            self.target_x,
            self.target_y,
            self.target_z,
            self.target_yaw,
        )):
            raise ValueError('Target coordinates must be finite numbers')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Publishers use names relative to the UAV namespace.
        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            'fmu/in/offboard_control_mode',
            qos,
        )

        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            'fmu/in/trajectory_setpoint',
            qos,
        )

        self.command_pub = self.create_publisher(
            VehicleCommand,
            'fmu/in/vehicle_command',
            qos,
        )

        self.position: VehicleLocalPosition | None = None
        self.position_received_at: float | None = None
        self.last_map_position = (0.0, 0.0, 0.0)
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.status: VehicleStatus | None = None
        self.active_goal = False
        self.active_target_ned: tuple[float, float, float] | None = None
        self.active_reference_timestamp: int | None = None
        self.active_max_velocity = 0.0
        self.active_yaw_ned = 0.0
        self.idle_target_ned = (
            self.target_x, self.target_y, self.target_z
        )
        self.takeoff_phase = False
        self.goal_lock = threading.Lock()
        self.offboard_request_sent = False
        self.arm_request_sent = False
        self.enabled_ticks = 0
        self.emergency_landing = False
        self.emergency_land_command_sent = False
        self.landing_requested = False
        self.landing_command_sent = False

        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            'fmu/out/vehicle_local_position_v1',
            self.position_callback,
            qos,
        )

        self.status_sub = self.create_subscription(
            VehicleStatus,
            'fmu/out/vehicle_status_v4',
            self.status_callback,
            qos,
        )
        self.task_status_sub = self.create_subscription(
            TaskStatus,
            'task_status',
            self.task_status_callback,
            10,
        )

        self.timer = self.create_timer(0.1, self.control_loop)
        self.action_server = ActionServer(
            self,
            GoToWaypoint,
            'go_to_waypoint',
            execute_callback=self.execute_waypoint,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        self.get_logger().info(
            f"Controller started in namespace '{self.get_namespace()}'. "
            f"Target NED=({self.target_x}, {self.target_y}, "
            f"{self.target_z}), yaw={self.target_yaw}"
        )

        if not self.start_enabled:
            self.get_logger().warn(
                'START DISABLED: no arming or mode commands will be sent.'
            )

    def position_callback(self, msg):
        self.position = msg
        self.position_received_at = time.monotonic()
        if self._has_fresh_global_reference():
            try:
                self.last_map_position = self._map_position()
            except TransformException:
                pass

    def status_callback(self, msg):
        self.status = msg

    def task_status_callback(self, message):
        if is_emergency_landing_request(message.task_id, message.status):
            self.emergency_landing = True
            self.get_logger().fatal(
                f'Emergency landing requested: {message.status_message}'
            )
        elif is_controlled_landing_request(
            message.task_id, message.status
        ):
            self.landing_requested = True

    def publish_offboard_mode(self, velocity_mode=False):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds // 1000)
        msg.position = not velocity_mode
        msg.velocity = velocity_mode
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_pub.publish(msg)

    def publish_setpoint(self):
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds // 1000)
        if self.active_goal and self.active_target_ned is not None:
            target = self.active_target_ned
            if self.takeoff_phase and self.position is not None:
                if abs(self.position.z - target[2]) <= 0.4:
                    self.takeoff_phase = False
                else:
                    target = (self.position.x, self.position.y, target[2])
            error = [
                target[index] - self.position_value(index)
                for index in range(3)
            ]
            msg.position = [float('nan')] * 3
            msg.velocity = list(calculate_velocity_setpoint(
                error,
                self.velocity_gain,
                self.active_max_velocity,
                self.max_velocity_limit,
                self.horizontal_velocity_multiplier,
                self.forward_speed_multiplier,
            ))
            msg.yaw = self.active_yaw_ned
        else:
            msg.position = [float(value) for value in self.idle_target_ned]
            msg.velocity = [float('nan')] * 3
            msg.yaw = float(self.target_yaw)
        msg.acceleration = [float('nan')] * 3
        msg.jerk = [float('nan')] * 3
        msg.yawspeed = float('nan')
        self.setpoint_pub.publish(msg)

    def position_value(self, index):
        if self.position is None:
            return 0.0
        return (self.position.x, self.position.y, self.position.z)[index]

    def send_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds // 1000)
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = int(command)
        msg.target_system = int(self.target_system)
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)

    def control_loop(self):
        if self.emergency_landing:
            if not self.emergency_land_command_sent:
                self.send_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_NAV_LAND
                )
                self.emergency_land_command_sent = True
            return

        if self.landing_requested:
            if not self.landing_command_sent:
                self.send_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_NAV_LAND
                )
                self.landing_command_sent = True
            return

        # PX4 requires a continuous Offboard heartbeat and setpoint stream.
        self.publish_offboard_mode(self.active_goal)
        self.publish_setpoint()

        self.get_logger().info(
            f'pos={self.position is not None} '
            f'status={self.status is not None} '
            f'nav={getattr(self.status, "nav_state", None)} '
            f'arm={getattr(self.status, "arming_state", None)}',
            throttle_duration_sec=2.0,
        )

        if not self.start_enabled:
            return

        if self.position is None or self.status is None:
            return

        # Allow at least 1 second of streaming before requesting Offboard.
        self.enabled_ticks += 1
        if self.enabled_ticks < 12 or self.enabled_ticks % 10 != 0:
            return  # wait ~1 s, then retry about once per second

        if self.status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.send_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        elif self.status.arming_state != VehicleStatus.ARMING_STATE_ARMED:
            self.send_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0)

    def goal_callback(self, goal):
        values = (
            goal.target_pose.pose.position.x,
            goal.target_pose.pose.position.y,
            goal.target_pose.pose.position.z,
            goal.acceptance_radius,
            goal.max_velocity,
        )
        if not all(math.isfinite(value) for value in values):
            return GoalResponse.REJECT
        target = goal.target_pose.pose.position
        if self._operating_bounds_enabled() and (
            not self._inside_operating_bounds(target.x, target.y, target.z)
            or not self._current_position_inside_operating_bounds()
        ):
            self.get_logger().error(
                'Rejected waypoint outside the configured operating bounds.'
            )
            return GoalResponse.REJECT
        if (
            goal.acceptance_radius <= 0.0
            or goal.max_velocity <= 0.0
            or goal.max_velocity > self.max_velocity_limit
            or not self.start_enabled
            or goal.target_pose.header.frame_id != self.map_frame_id
            or not self._has_fresh_global_reference()
        ):
            return GoalResponse.REJECT
        with self.goal_lock:
            if self.active_goal:
                return GoalResponse.REJECT
            self.active_goal = True
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def execute_waypoint(self, goal_handle):
        goal = goal_handle.request
        target = goal.target_pose.pose.position
        position = self.position
        if (
            position is None
            or not self._has_fresh_global_reference()
        ):
            goal_handle.abort()
            with self.goal_lock:
                self.active_goal = False
            return GoToWaypoint.Result(
                success=False,
                message=(
                    'PX4 local position or its global NED reference '
                    'is unavailable or stale.'
                ),
            )

        try:
            target_ned, yaw_local_enu = self._map_to_ned(
                float(target.x), float(target.y), float(target.z),
                goal.target_pose.pose.orientation,
            )
            start_position = self._map_position()
        except TransformException as error:
            goal_handle.abort()
            self._clear_active_goal()
            return GoToWaypoint.Result(
                success=False,
                message=f'Map transform is unavailable or stale: {error}',
            )
        with self.goal_lock:
            self.active_target_ned = target_ned
            self.active_reference_timestamp = int(position.ref_timestamp)
            self.active_max_velocity = float(goal.max_velocity)
            self.active_yaw_ned = math.pi / 2.0 - yaw_local_enu
            self.takeoff_phase = abs(position.z - target_ned[2]) > 0.4

        initial_distance = math.dist(
            start_position, (target.x, target.y, target.z)
        )
        started_at = time.monotonic()
        feedback = GoToWaypoint.Feedback()
        result = GoToWaypoint.Result()

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self._hold_current_position()
                result.success = False
                result.message = 'Waypoint goal was cancelled.'
                result.final_pose = self._current_pose()
                self._clear_active_goal()
                return result
            position = self.position
            if (
                position is None
                or not self._has_fresh_global_reference()
                or int(position.ref_timestamp)
                != self.active_reference_timestamp
            ):
                break
            try:
                current = self._map_position()
            except TransformException:
                break
            if (
                self._operating_bounds_enabled()
                and not self._inside_operating_bounds(*current)
            ):
                result.message = (
                    'UAV left its configured operating bounds during '
                    'waypoint execution.'
                )
                break
            remaining = math.dist(current, (target.x, target.y, target.z))
            feedback.distance_remaining = float(remaining)
            feedback.progress = float(
                1.0 if initial_distance <= 1e-6 else
                max(0.0, min(1.0, 1.0 - remaining / initial_distance))
            )
            goal_handle.publish_feedback(feedback)
            if remaining <= float(goal.acceptance_radius):
                goal_handle.succeed()
                result.success = True
                result.message = 'Waypoint reached.'
                result.final_pose = self._current_pose()
                self._hold_current_position()
                self._clear_active_goal()
                return result
            if time.monotonic() - started_at > self.goal_timeout_sec:
                break
            time.sleep(0.1)

        goal_handle.abort()
        result.success = False
        if not result.message:
            result.message = (
                'Waypoint was not reached before timeout or telemetry loss.'
            )
        result.final_pose = self._current_pose()
        self._hold_current_position()
        self._clear_active_goal()
        return result

    def _operating_bounds_enabled(self):
        return self.get_parameter(
            'operating_bounds.enabled'
        ).get_parameter_value().bool_value

    def _inside_operating_bounds(self, x, y, z):
        values = (float(x), float(y), float(z))
        if not all(math.isfinite(value) for value in values):
            return False
        bounds = {
            name: self.get_parameter(
                f'operating_bounds.{name}'
            ).get_parameter_value().double_value
            for name in ('x_min', 'x_max', 'y_min', 'y_max', 'z_min', 'z_max')
        }
        return (
            bounds['x_min'] <= values[0] <= bounds['x_max']
            and bounds['y_min'] <= values[1] <= bounds['y_max']
            and bounds['z_min'] <= values[2] <= bounds['z_max']
        )

    def _current_position_inside_operating_bounds(self):
        try:
            return self._inside_operating_bounds(*self._map_position())
        except TransformException:
            return False

    def _has_fresh_global_reference(self) -> bool:
        position = self.position
        received_at = self.position_received_at
        return bool(
            position is not None
            and received_at is not None
            and time.monotonic() - received_at <= self.telemetry_timeout_sec
            and position.xy_valid
            and position.z_valid
            and position.xy_global
            and position.z_global
            and all(math.isfinite(float(value)) for value in (
                position.ref_lat, position.ref_lon, position.ref_alt,
                position.x, position.y, position.z,
            ))
            and -90.0 <= float(position.ref_lat) <= 90.0
            and -180.0 <= float(position.ref_lon) <= 180.0
        )

    def _lookup_transform(self, target_frame, source_frame):
        transform = self.tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            Time(),
            timeout=Duration(nanoseconds=int(
                self.telemetry_timeout_sec * 1_000_000_000
            )),
        )
        transform_time = Time.from_msg(transform.header.stamp)
        age_sec = (
            self.get_clock().now().nanoseconds - transform_time.nanoseconds
        ) / 1e9
        if age_sec < -0.1 or age_sec > self.telemetry_timeout_sec:
            raise TransformException(
                f'Transform {source_frame!r} -> {target_frame!r} is stale '
                f'({age_sec:.2f} s old).'
            )
        return transform

    def _map_to_ned(
        self, east, north, up, orientation
    ) -> tuple[tuple[float, float, float], float]:
        position = self.position
        if not self._has_fresh_global_reference() or position is None:
            raise RuntimeError('PX4 local NED reference is unavailable.')
        goal = PoseStamped()
        goal.header.frame_id = self.map_frame_id
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = east
        goal.pose.position.y = north
        goal.pose.position.z = up
        goal.pose.orientation = orientation
        transformed = do_transform_pose(
            goal.pose,
            self._lookup_transform(self.local_frame_id, self.map_frame_id),
        )
        point = transformed.position
        local_enu = (float(point.x), float(point.y), float(point.z))
        yaw_local_enu = self._yaw_from_quaternion(
            transformed.orientation
        )
        ned_position = enu_to_ned(local_enu)
        local_ned = (
            float(ned_position[0]),
            float(ned_position[1]),
            float(ned_position[2]),
        )
        self.get_logger().info(
            f'Waypoint {self.map_frame_id} ENU '
            f'({east:.2f}, {north:.2f}, {up:.2f}) -> '
            f'{self.local_frame_id} NED {local_ned}'
        )
        return local_ned, yaw_local_enu

    def _map_position(self) -> tuple[float, float, float]:
        position = self.position
        if not self._has_fresh_global_reference() or position is None:
            raise RuntimeError(
                'PX4 local position or reference is unavailable.'
            )
        transform = self._lookup_transform(
            self.map_frame_id, self.body_frame_id
        )
        translation = transform.transform.translation
        return (
            float(translation.x),
            float(translation.y),
            float(translation.z),
        )

    def _current_pose(self):
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        position = self.last_map_position
        (
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
        ) = position
        pose.pose.orientation.w = 1.0
        return pose

    @staticmethod
    def _yaw_from_quaternion(quaternion):
        return math.atan2(
            2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
            1.0 - 2.0 * (quaternion.y**2 + quaternion.z**2),
        )

    def _hold_current_position(self):
        if self.position is not None:
            self.idle_target_ned = (
                float(self.position.x),
                float(self.position.y),
                float(self.position.z),
            )

    def _clear_active_goal(self):
        self._hold_current_position()
        with self.goal_lock:
            self.active_target_ned = None
            self.active_reference_timestamp = None
            self.active_goal = False


def main(args=None):
    rclpy.init(args=args)
    node = OffboardController()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
