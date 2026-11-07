import math
import threading
import time

import rclpy
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
from geometry_msgs.msg import PoseStamped
from uav_interfaces.action import GoToWaypoint


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
        self.declare_parameter('home_map_x', 0.0)
        self.declare_parameter('home_map_y', 0.0)
        self.declare_parameter('home_map_z', 0.0)
        self.declare_parameter('goal_timeout_sec', 180.0)
        self.declare_parameter('velocity_gain', 1.0)
        self.declare_parameter('max_velocity_limit', 5.0)

        
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
        self.home_map: tuple[float, float, float] = (
            self.get_parameter(
                'home_map_x'
            ).get_parameter_value().double_value,
            self.get_parameter(
                'home_map_y'
            ).get_parameter_value().double_value,
            self.get_parameter(
                'home_map_z'
            ).get_parameter_value().double_value,
        )
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
        self.status: VehicleStatus | None = None
        self.active_goal = False
        self.active_target_ned: tuple[float, float, float] | None = None
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

    def status_callback(self, msg):
        self.status = msg

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
            horizontal_speed = (
                math.hypot(error[0], error[1]) * self.velocity_gain
            )
            if horizontal_speed > self.active_max_velocity:
                scale = self.active_max_velocity / horizontal_speed
                error[0] *= scale
                error[1] *= scale
            msg.position = [float('nan')] * 3
            msg.velocity = [
                max(-self.active_max_velocity, min(self.active_max_velocity,
                    error[index] * self.velocity_gain))
                for index in range(3)
            ]
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
        if (
            goal.acceptance_radius <= 0.0
            or goal.max_velocity <= 0.0
            or goal.max_velocity > self.max_velocity_limit
            or not self.start_enabled
            or goal.target_pose.header.frame_id not in ('', 'map')
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
        if self.position is None:
            goal_handle.abort()
            with self.goal_lock:
                self.active_goal = False
            return GoToWaypoint.Result(
                success=False,
                message='PX4 local position is unavailable.',
            )

        east = float(target.x) - self.home_map[0]
        north = float(target.y) - self.home_map[1]
        up = float(target.z) - self.home_map[2]
        target_ned = (north, east, -up)
        yaw_enu = self._yaw_from_quaternion(goal.target_pose.pose.orientation)
        with self.goal_lock:
            self.active_target_ned = target_ned
            self.active_max_velocity = float(goal.max_velocity)
            self.active_yaw_ned = math.pi / 2.0 - yaw_enu
            self.takeoff_phase = abs(self.position.z - target_ned[2]) > 0.4

        start_position = self._map_position()
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
            if self.position is None:
                break
            current = self._map_position()
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
        result.message = (
            'Waypoint was not reached before timeout or telemetry loss.'
        )
        result.final_pose = self._current_pose()
        self._hold_current_position()
        self._clear_active_goal()
        return result

    def _map_position(self) -> tuple[float, float, float]:
        position = self.position
        if position is None:
            raise RuntimeError('PX4 local position is unavailable.')
        return (
            self.home_map[0] + position.y,
            self.home_map[1] + position.x,
            self.home_map[2] - position.z,
        )

    def _current_pose(self):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        position = (
            self._map_position()
            if self.position is not None else self.home_map
        )
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
