
from datetime import datetime, timezone
from functools import partial
import csv
import json
import math
from pathlib import Path
import threading
import time

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float64, String
from rcl_interfaces.srv import SetParametersAtomically
from uav_interfaces.action import GoToWaypoint
from uav_interfaces.msg import UAVState
from uav_interfaces.msg import TaskStatus
from uav_interfaces.srv import (
    AddTask,
    AssignTask,
    CancelTask,
    ConfirmRecovery,
    GetUnassignedTasks,
)
from uav_mission.operating_bounds import OperatingBounds

from uav_mission.mission_state import (
    MissionState,
    RecoveryState,
    all_uavs_confirmed_landed,
    completion_return_decision,
    feasible_return_options,
)
from uav_mission.result_writer import (
    default_results_dir,
    write_mission_results,
)
from uav_mission.task_manager import (
    DEFAULT_INSPECTION_ALTITUDES_M,
    TaskManager,
    tasks_from_mission,
)


def _default_mission_config_path():
    source_path = (
        Path(__file__).resolve().parents[1] / 'config' / 'mission.yaml'
    )

    try:
        from ament_index_python.packages import (
            PackageNotFoundError,
            get_package_share_directory,
        )
    except ImportError:
        return source_path

    try:
        package_share = get_package_share_directory('uav_mission')
    except PackageNotFoundError:
        return source_path

    return Path(package_share) / 'config' / 'mission.yaml'


def load_mission_config(config_path=None):
    path = (
        Path(config_path)
        if config_path is not None
        else _default_mission_config_path()
    )

    try:
        with path.open(encoding='utf-8') as mission_file:
            mission_data = yaml.safe_load(mission_file)
    except FileNotFoundError as error:
        raise FileNotFoundError(
            f'Mission configuration file not found: {path}'
        ) from error
    except yaml.YAMLError as error:
        raise ValueError(
            f'Invalid YAML in mission configuration {path}: {error}'
        ) from error

    if not isinstance(mission_data, dict) or not isinstance(
        mission_data.get('mission'), dict
    ):
        raise ValueError(
            f'Mission configuration {path} must contain a top-level '
            '"mission" mapping'
        )

    return mission_data


def cruise_return_waypoint(task, cruise_altitude):
    waypoint = PoseStamped()
    waypoint.header.frame_id = task.frame_id
    waypoint.pose.position.x = float(task.target_position.x)
    waypoint.pose.position.y = float(task.target_position.y)
    waypoint.pose.position.z = float(cruise_altitude)
    waypoint.pose.orientation.w = 1.0
    return waypoint


class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')
        self.mission_data = load_mission_config()
        mission = self.mission_data['mission']
        self.frame_id = str(mission['frame_id'])
        flight_levels = mission['flight_levels']
        recovery_config = mission['recovery']
        navigation_config = mission.get('navigation', {})
        self.operating_region_config = navigation_config.get(
            'operating_region'
        )
        if not isinstance(self.operating_region_config, dict):
            raise ValueError(
                'navigation.operating_region must define the UAV bounds'
            )
        maximum_retries = navigation_config.get('maximum_retries', 2)
        if (
            isinstance(maximum_retries, bool)
            or not isinstance(maximum_retries, int)
            or maximum_retries < 0
        ):
            raise ValueError(
                'navigation.maximum_retries must be a non-negative integer'
            )
        self.maximum_navigation_retries = maximum_retries
        self.standard_flight_altitude = float(
            flight_levels['standard_altitude_m']
        )
        self.inspection_altitudes_m = tuple(
            float(value)
            for value in flight_levels['inspection_altitudes_m']
        )
        if self.inspection_altitudes_m != DEFAULT_INSPECTION_ALTITUDES_M:
            raise ValueError(
                'Mission inspection altitudes must be [3.0, 5.0, 7.0, 10.0]'
            )
        if any(
            not value.is_integer()
            for value in self.inspection_altitudes_m
        ):
            raise ValueError(
                'Inspection planner altitudes must be whole metres'
            )
        self.planner_altitudes = tuple(
            int(value) for value in self.inspection_altitudes_m
        )
        self.low_battery_threshold = float(
            recovery_config['low_battery_threshold_percent']
        )
        self.minimum_safe_battery_reserve = float(
            recovery_config['post_landing_reserve_percent']
        )
        self.battery_consumption_per_meter = float(
            recovery_config['battery_consumption_per_meter']
        )
        self.inspection_consumption_per_minute = float(
            recovery_config.get('inspection_consumption_per_minute', 0.25)
        )
        self.return_distance_safety_factor = float(
            recovery_config['distance_safety_factor']
        )
        if not 1.25 <= self.return_distance_safety_factor <= 1.30:
            raise ValueError(
                'recovery.distance_safety_factor must be between 1.25 '
                'and 1.30'
            )
        self.landing_energy_percentage = float(
            recovery_config['landing_energy_percentage']
        )
        self.landing_altitude_m = float(
            recovery_config['landing_altitude_m']
        )
        self.landing_timeout_sec = float(
            recovery_config['landing_timeout_sec']
        )
        self.ugv_frame_id = str(recovery_config['ugv_frame_id'])
        self.ugv_location = tuple(
            float(value) for value in recovery_config['ugv_location']
        )
        mission_constraints = mission['constraints']
        self.maximum_altitude_m = min(
            float(mission_constraints['maximum_altitude_m']),
            float(self.operating_region_config['z_max_m']),
        )
        z_min = float(self.operating_region_config['z_min_m'])
        if not (
            z_min
            <= self.standard_flight_altitude
            <= self.maximum_altitude_m
        ):
            raise ValueError(
                'Cruise altitude must be inside the configured Z bounds'
            )
        if not z_min <= self.landing_altitude_m <= self.maximum_altitude_m:
            raise ValueError(
                'Landing altitude must be inside the configured Z bounds'
            )
        self.vertical_half_extent_m = float(
            mission_constraints.get('uav_vertical_half_extent_m', 0.25)
        )
        self.vertical_clearance_m = float(
            mission_constraints.get('vertical_obstacle_clearance_m', 0.25)
        )
        self.minimum_separation_m = float(
            mission_constraints['minimum_separation_m']
        )
        self.recovery_state = RecoveryState(
            self.low_battery_threshold,
            self.ugv_location,
            self.ugv_frame_id,
        )
        uav_ids = [uav['id'] for uav in mission['uavs']]
        self.takeoff_positions = {
            uav['id']: tuple(
                float(value) for value in uav['start_position']
            )
            for uav in self.mission_data['mission']['uavs']
        }
        self.home_positions = {
            uav['id']: tuple(
                float(value) for value in uav.get(
                    'home_position', uav['start_position']
                )
            )
            for uav in self.mission_data['mission']['uavs']
        }
        configured_nearby_targets = recovery_config.get(
            'nearby_landing_options', []
        )
        if not isinstance(configured_nearby_targets, list):
            raise ValueError(
                'recovery.nearby_landing_options must be a list'
            )
        self.nearby_landing_targets = []
        for target in configured_nearby_targets:
            if (
                not isinstance(target, dict)
                or target.get('validated') is not True
            ):
                raise ValueError(
                    'Every nearby landing option must be a validated mapping'
                )
            try:
                position = tuple(
                    float(value) for value in target['position']
                )
                name = str(target['name'])
            except (KeyError, TypeError, ValueError) as error:
                raise ValueError(
                    'Nearby landing options require a name and 3D position'
                ) from error
            if (
                not name
                or len(position) != 3
                or not all(math.isfinite(value) for value in position)
                or position[2] < 0.0
            ):
                raise ValueError(
                    'Nearby landing options need a non-empty name and a '
                    'finite, non-negative 3D position'
                )
            self.nearby_landing_targets.append({
                'name': name,
                'position': position,
                'frame_id': str(target.get('frame_id', self.frame_id)),
                'validated': True,
                'recovery': False,
            })
        self.takeoff_position_recorded = set()
        self.operating_bounds_by_uav = {}
        self.operating_bounds_configured = set()
        self.operating_bounds_configuring = set()
        self.tasks = None
        self.task_manager = None
        self.task_assignment_lock = threading.RLock()
        self.deferred_path_fallbacks = {}
        self.path_fallback_attempts = {}
        self.path_fallback_counts = {}
        self.preplanned_routes = {}
        self.wait_reason = None
        self.uav_ids = uav_ids
        self.odometry = {}
        self.odometry_received_at = {}
        self.last_positions = {}
        self.return_minimum_reserve_percentage = {}
        self.priority_returning_uavs = set()
        self.distance_by_uav = {uav_id: 0.0 for uav_id in uav_ids}
        self.initial_battery_percentage = {}
        self.latest_battery_percentage = {}
        self.battery_updated_at = {}
        self.initial_estimated_battery_percentage = {}
        self.latest_estimated_battery_percentage = {}
        self.estimated_battery_updated_at = {}
        self.energy_consumed_wh = {uav_id: 0.0 for uav_id in uav_ids}
        self.energy_samples = {}
        self.energy_interval_counts = {uav_id: 0 for uav_id in uav_ids}
        self.travel_time_sec = 0.0
        self.last_odometry_time = {}
        self.inspection_time_sec = 0.0
        self.planned_distance_m = 0.0
        self.return_distance_m = 0.0
        self.allocator_planning_time_sec = 0.0
        self.mission_planning_time_sec = 0.0
        self.mission_failed_reasons = {}
        self.mission_started_at = None
        self.mission_started_sim_time_sec = None
        self.inspection_completed_sim_time_sec = None
        self.metrics_reported = False
        self.results_saved = False
        self.mission_started_at_utc = None
        self.replan_count = 0
        self.navigation_failures = 0
        self.active_navigation = {}
        self.pending_navigation = []
        self.failed_uavs = set()
        self.last_task_altitude = {}
        self.home_return_started = False
        self.pending_return_uavs = set()
        self.return_navigation = {}
        self.return_targets = {}
        self.return_candidates = {}
        self.return_targets_tried = {}
        self.returned_uavs = set()
        self.return_failures = set()
        self.return_failure_reasons = {}
        self.successful_landed_uavs = set()
        self.shutdown_timer = None
        self.shutdown_delay_elapsed = False
        self.shutdown_initiated = False

        self.declare_parameter(
            'uav_state_topic_template', '/{uav_id}/uav_state'
        )
        self.declare_parameter('state_timeout_sec', 3.0)
        self.declare_parameter('navigation.max_velocity_mps', 2.0)
        self.declare_parameter('navigation.acceptance_radius_m', 0.3)
        self.declare_parameter(
            'navigation.inspection_path_tolerance_m', 1.0
        )
        self.declare_parameter('navigation.replan_distance_m', 5.0)
        self.declare_parameter('results_dir', '')
        self.declare_parameter('save_task_level_results', False)
        self.declare_parameter('simulation_shutdown_delay_sec', 2.0)
        self.declare_parameter('require_algorithm_comparison_results', False)
        configured_results_dir = str(
            self.get_parameter(
                'results_dir'
            ).get_parameter_value().string_value
        )
        self.save_task_level_results = bool(
            self.get_parameter(
                'save_task_level_results'
            ).get_parameter_value().bool_value
        )
        self.results_dir = (
            Path(configured_results_dir).expanduser()
            if configured_results_dir
            else default_results_dir()
        )
        self.declare_parameter('recovery.state_timeout_sec', 3.0)
        topic_template = self.get_parameter(
            'uav_state_topic_template'
        ).get_parameter_value().string_value
        if not isinstance(topic_template, str) or not topic_template:
            raise ValueError(
                'uav_state_topic_template must be a non-empty string'
            )
        state_timeout_sec = self._float_parameter('state_timeout_sec')
        self.max_velocity_mps = self._float_parameter(
            'navigation.max_velocity_mps'
        )
        self.acceptance_radius_m = self._float_parameter(
            'navigation.acceptance_radius_m'
        )
        self.inspection_path_tolerance_m = self._float_parameter(
            'navigation.inspection_path_tolerance_m'
        )
        if self.inspection_path_tolerance_m <= 0.0:
            raise ValueError(
                'navigation.inspection_path_tolerance_m must be positive'
            )
        self.replan_distance_m = self._float_parameter(
            'navigation.replan_distance_m'
        )
        self.simulation_shutdown_delay_sec = self._float_parameter(
            'simulation_shutdown_delay_sec'
        )
        self.require_algorithm_comparison_results = bool(
            self.get_parameter(
                'require_algorithm_comparison_results'
            ).get_parameter_value().bool_value
        )
        if self.simulation_shutdown_delay_sec < 0.0:
            raise ValueError(
                'simulation_shutdown_delay_sec cannot be negative'
            )
        self.mission_state = MissionState(
            uav_ids, state_timeout_sec
        )
        self.planner_clients = {
            uav_id: {
                altitude: ActionClient(
                    self,
                    ComputePathToPose,
                    f'/{uav_id}/planner_{altitude}m/compute_path_to_pose',
                )
                for altitude in self.planner_altitudes
            }
            for uav_id in uav_ids
        }
        self.operating_bounds_parameter_clients = {}
        for uav_id in uav_ids:
            clients = {
                'controller': self.create_client(
                    SetParametersAtomically,
                    f'/{uav_id}/offboard_controller/'
                    'set_parameters_atomically',
                )
            }
            clients.update({
                f'fuser_{altitude}m': self.create_client(
                    SetParametersAtomically,
                    f'/{uav_id}/altitude_map_fuser_{altitude}m/'
                    'set_parameters_atomically',
                )
                for altitude in self.planner_altitudes
            })
            self.operating_bounds_parameter_clients[uav_id] = clients
        self.waypoint_clients = {
            uav_id: ActionClient(
                self, GoToWaypoint, f'/{uav_id}/go_to_waypoint'
            )
            for uav_id in uav_ids
        }
        self.task_status_publishers = {
            uav_id: self.create_publisher(
                TaskStatus, f'/{uav_id}/task_status', 10
            )
            for uav_id in uav_ids
        }
        task_status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1000,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.central_task_status_publisher = self.create_publisher(
            TaskStatus, '/mission_manager/task_status', task_status_qos
        )
        self.path_distance_publishers = {
            uav_id: self.create_publisher(
                Float64, f'/{uav_id}/path_distance', 10
            )
            for uav_id in uav_ids
        }
        self.trajectory_publishers = {
            uav_id: self.create_publisher(
                NavPath, f'/mission_manager/{uav_id}/planned_path', 10
            )
            for uav_id in uav_ids
        }
        self.get_unassigned_tasks_service = self.create_service(
            GetUnassignedTasks,
            '/mission_manager/get_unassigned_tasks',
            self.get_unassigned_tasks_callback,
        )
        self.get_logger().info(
            "GetUnassignedTasks service ready at "
            "'/mission_manager/get_unassigned_tasks'."
        )
        self.assign_task_service = self.create_service(
            AssignTask,
            '/mission_manager/assign_task',
            self.assign_task_callback,
        )
        self.confirm_recovery_service = self.create_service(
            ConfirmRecovery,
            '/mission_manager/confirm_recovery',
            self.confirm_recovery_callback,
        )
        self.add_task_service = self.create_service(
            AddTask,
            '/mission_manager/add_task',
            self.add_task_callback,
        )
        self.cancel_task_service = self.create_service(
            CancelTask,
            '/mission_manager/cancel_task',
            self.cancel_task_callback,
        )
        self.allocation_event_publisher = self.create_publisher(
            String, '/mission_manager/allocation_event', 10
        )
        self.uav_state_subscriptions = []
        self.odometry_subscriptions = []
        self.battery_subscriptions = []
        self.estimated_battery_subscriptions = []
        state_topics = []
        for uav_id in uav_ids:
            topic = topic_template.format(uav_id=uav_id)
            state_topics.append(topic)
            self.uav_state_subscriptions.append(
                self.create_subscription(
                    UAVState,
                    topic,
                    partial(
                        self.uav_state_callback,
                        expected_uav_id=uav_id,
                    ),
                    1,
                )
            )
            self.odometry_subscriptions.append(
                self.create_subscription(
                    Odometry,
                    f'/{uav_id}/odom',
                    partial(self.odometry_callback, uav_id=uav_id),
                    10,
                )
            )
            self.battery_subscriptions.append(
                self.create_subscription(
                    BatteryState,
                    f'/{uav_id}/battery/measured',
                    partial(self.battery_callback, uav_id=uav_id),
                    10,
                )
            )
            self.estimated_battery_subscriptions.append(
                self.create_subscription(
                    BatteryState,
                    f'/{uav_id}/battery/estimated',
                    partial(self.estimated_battery_callback, uav_id=uav_id),
                    10,
                )
            )
        self.path_planning_time_subscription = self.create_subscription(
            Float64,
            '/auction_allocator/path_planning_time',
            self.allocator_planning_time_callback,
            10,
        )
        self.unallocatable_tasks_subscription = self.create_subscription(
            String,
            '/mission_manager/unallocatable_tasks',
            self.unallocatable_tasks_callback,
            10,
        )
        self.uav_state_timeout_sec = float(
            self.get_parameter(
                'recovery.state_timeout_sec'
            ).get_parameter_value().double_value
        )
        self.timer = self.create_timer(1.0, self.prepare_tasks_when_ready)
        self.navigation_timer = self.create_timer(
            0.5, self.start_pending_navigation
        )
        self.return_timer = self.create_timer(
            0.5, self.start_pending_returns
        )
        self.health_timer = self.create_timer(
            1.0, self.check_uav_health
        )

        self.get_logger().info(
            f"Loaded mission '{self.mission_data['mission']['name']}'. "
            f'Waiting for UAV status on {", ".join(state_topics)}.'
        )

    def _float_parameter(self, name):
        result = float(
            self.get_parameter(name).get_parameter_value().double_value
        )
        if not math.isfinite(result):
            raise ValueError(f"Parameter '{name}' must be finite")
        return result

    def uav_state_callback(self, message, expected_uav_id):
        if message.uav_id != expected_uav_id:
            self.get_logger().warning(
                f"Ignoring UAV '{message.uav_id}' status on the "
                f"'{expected_uav_id}' topic."
            )
            return

        previous = self.mission_state.uav_states.get(expected_uav_id)
        self.mission_state.update(message)
        self._complete_landing_if_ready(expected_uav_id, message)
        self._retry_deferred_path_fallbacks()
        if (
            message.connected
            and message.operational
            and message.mission_status in (UAVState.IDLE, UAVState.COMPLETED)
            and expected_uav_id not in self.failed_uavs
            and expected_uav_id not in self.recovery_state.returning
        ):
            if previous is None or previous.mission_status not in (
                UAVState.IDLE, UAVState.COMPLETED
            ):
                self._signal_allocation_event('uav_available')
        elif (
            self.task_manager is not None
            and expected_uav_id not in self.recovery_state.returning
            and self.latest_battery_percentage.get(
                expected_uav_id, 100.0
            ) >= self.low_battery_threshold
            and (not message.connected or not message.operational)
        ):
            self._handle_uav_failure(
                expected_uav_id,
                'UAV reported disconnected or non-operational.',
            )
        self._report_metrics_if_complete()

    def odometry_callback(self, message, uav_id):
        if message.header.frame_id != self.frame_id:
            return
        position = (
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
            float(message.pose.pose.position.z),
        )
        if not all(math.isfinite(value) for value in position):
            return
        previous = self.last_positions.get(uav_id)
        now = time.monotonic()
        if previous is not None and self.mission_started_at is not None:
            distance_increment = math.dist(previous, position)
            updated_distance = (
                self.distance_by_uav[uav_id] + distance_increment
            )
            if math.isfinite(updated_distance):
                self.distance_by_uav[uav_id] = updated_distance
                update = Float64()
                update.data = updated_distance
                self.path_distance_publishers[uav_id].publish(update)
            elapsed = now - self.last_odometry_time.get(uav_id, now)
            velocity = message.twist.twist.linear
            speed = math.sqrt(
                float(velocity.x) ** 2
                + float(velocity.y) ** 2
                + float(velocity.z) ** 2
            )
            if speed > 0.15:
                self.travel_time_sec += min(max(elapsed, 0.0), 1.0)
        self.last_positions[uav_id] = position
        self.odometry[uav_id] = message
        self.last_odometry_time[uav_id] = now
        self.odometry_received_at[uav_id] = now
        self._capture_takeoff_positions((uav_id,))

    def _capture_takeoff_positions(self, uav_ids):
        for uav_id in uav_ids:
            if uav_id in self.takeoff_position_recorded:
                continue
            odometry = self.odometry.get(uav_id)
            if odometry is None:
                continue
            position = odometry.pose.pose.position
            self.takeoff_positions[uav_id] = (
                float(position.x),
                float(position.y),
                float(position.z),
            )
            self.takeoff_position_recorded.add(uav_id)
            try:
                self.operating_bounds_by_uav[uav_id] = (
                    OperatingBounds.from_initial_position(
                        self.takeoff_positions[uav_id],
                        self.operating_region_config,
                    )
                )
            except (KeyError, TypeError, ValueError) as error:
                self.get_logger().error(
                    f"Cannot establish operating bounds for '{uav_id}': "
                    f'{error}'
                )
            self.get_logger().info(
                f"Stored '{uav_id}' launch position in '{self.frame_id}': "
                f'{self.takeoff_positions[uav_id]}.'
            )

    def _configure_operating_bounds(self):
        for uav_id in self.uav_ids:
            if uav_id not in self.operating_bounds_by_uav:
                try:
                    self.operating_bounds_by_uav[uav_id] = (
                        OperatingBounds.from_initial_position(
                            self.takeoff_positions[uav_id],
                            self.operating_region_config,
                        )
                    )
                except (KeyError, TypeError, ValueError) as error:
                    self.get_logger().error(
                        f"Cannot establish operating bounds for '{uav_id}': "
                        f'{error}'
                    )
                    return False

            bounds = self.operating_bounds_by_uav[uav_id].as_parameters()
            client_parameters = {
                'controller': {
                    'operating_bounds.enabled': True,
                    **{
                        f'operating_bounds.{name}': value
                        for name, value in bounds.items()
                    },
                },
                **{
                    f'fuser_{altitude}m': {
                        'operating_bounds.enabled': True,
                        **{
                            f'operating_bounds.{name}': bounds[name]
                            for name in ('x_min', 'x_max', 'y_min', 'y_max')
                        },
                    }
                    for altitude in self.planner_altitudes
                },
            }
            for name, values in client_parameters.items():
                key = f'{uav_id}:{name}'
                if (
                    key in self.operating_bounds_configured
                    or key in self.operating_bounds_configuring
                ):
                    continue
                client = self.operating_bounds_parameter_clients[
                    uav_id
                ][name]
                if not client.service_is_ready():
                    continue
                request = SetParametersAtomically.Request()
                request.parameters = [
                    Parameter(parameter_name, value=value).to_parameter_msg()
                    for parameter_name, value in values.items()
                ]
                self.operating_bounds_configuring.add(key)
                future = client.call_async(request)
                future.add_done_callback(
                    partial(self._operating_bounds_response, key=key)
                )
        expected = len(self.uav_ids) * (len(self.planner_altitudes) + 1)
        return len(self.operating_bounds_configured) == expected

    def _operating_bounds_are_configured(self, uav_id):
        expected = {
            f'{uav_id}:controller',
            *(
                f'{uav_id}:fuser_{altitude}m'
                for altitude in self.planner_altitudes
            ),
        }
        return expected.issubset(self.operating_bounds_configured)

    def _operating_bounds_response(self, future, key):
        self.operating_bounds_configuring.discard(key)
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(
                f'Failed to configure operating bounds for {key}: {error}'
            )
            return
        if not response.result.successful:
            self.get_logger().error(
                f'Operating bounds rejected by {key}: '
                f'{response.result.reason}'
            )
            return
        self.operating_bounds_configured.add(key)

    def _position_is_within_bounds(self, uav_id, position, description):
        bounds = self.operating_bounds_by_uav.get(uav_id)
        if bounds is not None and bounds.contains(position):
            return True
        self.get_logger().error(
            f"Rejected {description} for '{uav_id}': position "
            f'{tuple(position)} is outside its configured operating region.'
        )
        return False

    def _route_is_within_bounds(self, uav_id, route, description):
        if not route:
            return False
        for waypoint in route:
            if waypoint.header.frame_id != self.frame_id:
                self.get_logger().error(
                    f"Rejected {description} for '{uav_id}': waypoint frame "
                    f"'{waypoint.header.frame_id}' is not '{self.frame_id}'."
                )
                return False
            point = waypoint.pose.position
            if not self._position_is_within_bounds(
                uav_id, (point.x, point.y, point.z), description
            ):
                return False
        return True

    def battery_callback(self, message, uav_id):
        if (
            not message.present
            or not math.isfinite(message.percentage)
            or not 0.0 <= message.percentage <= 1.0
        ):
            return
        percentage = float(message.percentage) * 100.0
        self.latest_battery_percentage[uav_id] = percentage
        self.battery_updated_at[uav_id] = time.monotonic()
        if percentage < self.low_battery_threshold:
            self._start_low_battery_return(uav_id, percentage)
        voltage = float(message.voltage)
        current = float(message.current)
        now = time.monotonic()
        previous = self.energy_samples.get(uav_id)
        if (
            previous is not None
            and math.isfinite(voltage)
            and math.isfinite(current)
            and voltage > 0.0
            and current >= 0.0
        ):
            previous_time, previous_voltage, previous_current = previous
            elapsed = now - previous_time
            if elapsed > 0.0:
                average_power = (
                    voltage * current
                    + previous_voltage * previous_current
                ) / 2.0
                self.energy_consumed_wh[uav_id] += (
                    average_power * elapsed / 3600.0
                )
                self.energy_interval_counts[uav_id] += 1
        if (
            math.isfinite(voltage)
            and math.isfinite(current)
            and voltage > 0.0
            and current >= 0.0
        ):
            self.energy_samples[uav_id] = (now, voltage, current)

    def estimated_battery_callback(self, message, uav_id):
        if (
            message.present
            and math.isfinite(message.percentage)
            and 0.0 <= message.percentage <= 1.0
        ):
            self.latest_estimated_battery_percentage[uav_id] = (
                float(message.percentage) * 100.0
            )
            self.estimated_battery_updated_at[uav_id] = time.monotonic()

    def _signal_allocation_event(self, event):
        message = String()
        message.data = str(event)
        self.allocation_event_publisher.publish(message)

    def check_uav_health(self):
        if self.task_manager is None:
            return
        now = time.monotonic()
        for uav_id in self.uav_ids:
            if (
                uav_id in self.failed_uavs
                or uav_id in self.recovery_state.returning
                or not self.task_manager.unfinished_for_uav(uav_id)
            ):
                continue
            age = now - self.mission_state.last_state_update.get(
                uav_id, 0.0
            )
            if age > self.uav_state_timeout_sec:
                self._handle_uav_failure(
                    uav_id,
                    f'UAV state telemetry is stale ({age:.1f} s).',
                )

    def _cancel_uav_goal(self, uav_id):
        navigation = self.active_navigation.get(uav_id, {})
        goal_handle = navigation.get('goal_handle')
        if goal_handle is not None:
            goal_handle.cancel_goal_async()
        return_navigation = self.return_navigation.get(uav_id, {})
        return_goal_handle = return_navigation.get('goal_handle')
        if return_goal_handle is not None:
            return_goal_handle.cancel_goal_async()

    def _interrupt_uav_tasks(self, uav_id):
        self._cancel_uav_goal(uav_id)
        interrupted = []
        task_manager = self.task_manager
        if task_manager is not None:
            interrupted = task_manager.unfinished_for_uav(uav_id)
            for task_id in interrupted:
                task_manager.retry_navigation(
                    task_id,
                    uav_id,
                    self.maximum_navigation_retries,
                    'UAV became unavailable before task completion.',
                )
        self.pending_navigation = [
            (task, assigned_uav)
            for task, assigned_uav in self.pending_navigation
            if assigned_uav != uav_id
        ]
        self.active_navigation.pop(uav_id, None)
        for task_id in interrupted:
            retrying = (
                task_manager is not None
                and task_manager.navigation_retries.get(task_id, 0)
                <= self.maximum_navigation_retries
            )
            self._publish_task_status(
                task_id,
                uav_id,
                UAVState.IDLE if retrying else UAVState.FAILED,
                0.0,
                (
                    'Interrupted; task released for reassignment.'
                    if retrying else
                    'Interrupted; navigation retry limit reached.'
                ),
            )
        self.return_navigation.pop(uav_id, None)
        self.return_targets.pop(uav_id, None)
        self.return_targets_tried.pop(uav_id, None)
        self.pending_return_uavs.discard(uav_id)
        return interrupted

    def _start_low_battery_return(self, uav_id, percentage):
        with self.task_assignment_lock:
            return self._start_low_battery_return_locked(
                uav_id, percentage
            )

    def _start_low_battery_return_locked(self, uav_id, percentage):
        if (
            uav_id in self.return_targets
            or uav_id in self.return_navigation
            or uav_id in self.pending_return_uavs
            or uav_id in self.returned_uavs
        ):
            return
        if (
            uav_id not in self.operating_bounds_by_uav
            or not self._operating_bounds_are_configured(uav_id)
        ):
            self.get_logger().warning(
                f"Deferring low-battery return for '{uav_id}' until its "
                'initial-position operating region is configured.',
                throttle_duration_sec=5.0,
            )
            return
        if not self.recovery_state.begin(uav_id, percentage):
            return
        self.last_task_altitude.setdefault(
            uav_id, int(round(self.standard_flight_altitude))
        )
        interrupted = self._interrupt_uav_tasks(uav_id)
        started = self._queue_safe_return(
            uav_id,
            f'Battery at {percentage:.1f}%; released '
            f'{len(interrupted)} unfinished task(s).',
        )
        if started:
            self.get_logger().warning(
                f"UAV '{uav_id}' crossed the low-battery threshold and is "
                f"returning to '{self.return_targets[uav_id]['name']}'."
            )
        self._signal_allocation_event('low_battery_return')

    def _return_decision_for_uav(self, uav_id):
        odometry = self.odometry.get(uav_id)
        now = time.monotonic()
        battery = self._current_return_battery(uav_id)
        home_position = self.home_positions.get(uav_id)
        if (
            odometry is None
            or now - self.odometry_received_at.get(uav_id, 0.0) > 3.0
            or battery is None
            or home_position is None
        ):
            return None
        bounds = self.operating_bounds_by_uav.get(uav_id)
        if bounds is None:
            return None
        position = (
            float(odometry.pose.pose.position.x),
            float(odometry.pose.pose.position.y),
            float(odometry.pose.pose.position.z),
        )
        if not bounds.contains(position):
            return None
        home_position_valid = bounds.contains((
            home_position[0], home_position[1], self.landing_altitude_m
        ))
        ugv_target = {
            'name': 'UGV',
            'position': self.ugv_location,
            'frame_id': self.ugv_frame_id,
            'recovery': True,
        }
        ugv_target_valid = (
            self.ugv_frame_id == self.frame_id
            and bounds.contains(self.ugv_location)
        )
        nearby_targets = [
            target for target in self.nearby_landing_targets
            if target.get('frame_id', self.frame_id) == self.frame_id
            and bounds.contains(target['position'])
        ]
        if not home_position_valid:
            targets = (
                ([ugv_target] if ugv_target_valid else [])
                + nearby_targets
            )
            options = feasible_return_options(
                position,
                battery,
                targets,
                self.battery_consumption_per_meter,
                self.minimum_safe_battery_reserve,
                self.return_distance_safety_factor,
                self.landing_energy_percentage,
                self.standard_flight_altitude,
            )
            return {
                'mode': 'emergency' if options else 'unsafe',
                'predicted_normal_remaining_percent': None,
                'options': options,
            }
        return completion_return_decision(
            position,
            battery,
            home_position,
            ugv_target if ugv_target_valid else None,
            self.battery_consumption_per_meter,
            self.return_distance_safety_factor,
            self.landing_energy_percentage,
            self.minimum_safe_battery_reserve,
            self.landing_altitude_m,
            self.frame_id,
            nearby_targets,
            self.standard_flight_altitude,
        )

    def _single_feasible_return_target(self, uav_id, target):
        bounds = self.operating_bounds_by_uav.get(uav_id)
        if (
            bounds is None
            or target.get('frame_id', self.frame_id) != self.frame_id
            or not bounds.contains(target.get('position', ()))
        ):
            return None
        odometry = self.odometry.get(uav_id)
        now = time.monotonic()
        battery = self._current_return_battery(uav_id)
        if (
            odometry is None
            or now - self.odometry_received_at.get(uav_id, 0.0) > 3.0
            or battery is None
        ):
            return None
        position = (
            float(odometry.pose.pose.position.x),
            float(odometry.pose.pose.position.y),
            float(odometry.pose.pose.position.z),
        )
        options = feasible_return_options(
            position,
            battery,
            [target],
            self.battery_consumption_per_meter,
            self.minimum_safe_battery_reserve,
            self.return_distance_safety_factor,
            self.landing_energy_percentage,
            self.standard_flight_altitude,
        )
        return options[0] if options else None

    def _current_return_battery(self, uav_id):
        now = time.monotonic()
        if uav_id not in self.latest_battery_percentage:
            return None
        battery_age = now - self.battery_updated_at.get(uav_id, 0.0)
        if not 0.0 <= battery_age <= 3.0:
            return None
        battery = float(self.latest_battery_percentage[uav_id])
        if not math.isfinite(battery) or not 0.0 <= battery <= 100.0:
            return None
        if (
            uav_id in self.latest_estimated_battery_percentage
        ):
            estimated_age = now - self.estimated_battery_updated_at.get(
                uav_id, 0.0
            )
            estimated = float(
                self.latest_estimated_battery_percentage[uav_id]
            )
            if (
                0.0 <= estimated_age <= 3.0
                and math.isfinite(estimated)
                and 0.0 <= estimated <= 100.0
            ):
                battery = min(battery, estimated)
        return battery

    def _queue_safe_return(self, uav_id, reason):
        return self._queue_priority_return(uav_id, reason)

    def _queue_completion_return(self, uav_id, reason):
        return self._queue_priority_return(uav_id, reason)

    def _queue_priority_return(self, uav_id, reason):
        self.return_minimum_reserve_percentage[uav_id] = (
            self.minimum_safe_battery_reserve
        )
        self.priority_returning_uavs.add(uav_id)
        decision = self._return_decision_for_uav(uav_id)
        if decision is None or not decision['options']:
            self._handle_no_safe_return(
                uav_id,
                f'{reason} No fresh battery, odometry, or takeoff position '
                'is available, or neither the takeoff point nor configured '
                'UGV is safely reachable.',
            )
            return False
        options = decision['options']
        target = options[0]
        if target['recovery']:
            self.recovery_state.begin_return(uav_id)
        self.return_candidates[uav_id] = [target]
        self.return_targets[uav_id] = target
        self.return_targets_tried[uav_id] = {target['name']}
        self.pending_return_uavs.add(uav_id)
        predicted = decision['predicted_normal_remaining_percent']
        destination_type = (
            'normal takeoff return'
            if decision['mode'] == 'normal'
            else 'emergency landing'
        )
        self._publish_task_status(
            'RETURN_HOME' if decision['mode'] == 'normal'
            else 'EMERGENCY_LANDING',
            uav_id,
            UAVState.RETURNING,
            0.0,
            f'{reason} Predicted battery after normal return/landing: '
            f'{predicted:.1f}%. Starting {destination_type} at '
            f"'{target['name']}'.",
        )
        self.get_logger().info(
            f"UAV '{uav_id}' starts independent {destination_type}; "
            f'normal-return battery prediction is {predicted:.1f}%.'
        )
        return True

    def _handle_no_safe_return(self, uav_id, reason):
        reserve = self.return_minimum_reserve_percentage.get(
            uav_id, self.minimum_safe_battery_reserve
        )
        self.failed_uavs.add(uav_id)
        self.returned_uavs.add(uav_id)
        self.return_failures.add(uav_id)
        self.pending_return_uavs.discard(uav_id)
        self.return_navigation.pop(uav_id, None)
        self.return_targets.pop(uav_id, None)
        self.return_candidates.pop(uav_id, None)
        self.return_targets_tried.pop(uav_id, None)
        self.return_minimum_reserve_percentage.pop(uav_id, None)
        self.priority_returning_uavs.discard(uav_id)
        message = (
            f'No safe return destination is reachable with the required '
            f'{reserve:.1f}% reserve. '
            f'{reason} No return route was commanded; requesting PX4 '
            'emergency landing.'
        )
        self.return_failure_reasons[uav_id] = message
        self._publish_task_status(
            'EMERGENCY_RETURN', uav_id, UAVState.FAILED, 0.0, message
        )
        self.get_logger().error(f"UAV '{uav_id}': {message}")
        self._report_metrics_if_complete()

    def _handle_uav_failure(self, uav_id, reason):
        with self.task_assignment_lock:
            return self._handle_uav_failure_locked(uav_id, reason)

    def _handle_uav_failure_locked(self, uav_id, reason):
        if (
            uav_id in self.failed_uavs
            or uav_id in self.recovery_state.returning
        ):
            return
        if (
            self.task_manager is None
            or not self.task_manager.unfinished_for_uav(uav_id)
        ):
            return
        self.failed_uavs.add(uav_id)
        interrupted = self._interrupt_uav_tasks(uav_id)
        self.returned_uavs.add(uav_id)
        self.return_failures.add(uav_id)
        self.return_failure_reasons[uav_id] = reason
        self._publish_task_status(
            'UAV_FAILURE', uav_id, UAVState.FAILED, 0.0,
            f'{reason} Released {len(interrupted)} unfinished task(s).',
        )
        self.get_logger().error(
            f"UAV '{uav_id}' failed: {reason} "
            f'Released {len(interrupted)} task(s) for reassessment.'
        )
        self._signal_allocation_event('uav_failure')

    def confirm_recovery_callback(self, request, response):
        uav_id = str(request.uav_id)
        if not request.ready:
            response.success = False
            response.message = 'Recovery readiness must be explicitly true.'
            return response
        state = self.mission_state.uav_states.get(uav_id)
        battery = self.latest_battery_percentage.get(uav_id, 0.0)
        if state is None or not self.recovery_state.confirm_ready(
            uav_id, battery, bool(state.connected and state.operational)
        ):
            response.success = False
            response.message = (
                'UAV must have reached the UGV, report operational telemetry, '
                f'and have at least {self.low_battery_threshold:.1f}% battery.'
            )
            return response
        self._publish_task_status(
            'RECOVERY_CONFIRMED', uav_id, UAVState.IDLE, 0.0,
            'Recovery readiness confirmed; UAV is available.',
        )
        self.return_targets.pop(uav_id, None)
        self.return_candidates.pop(uav_id, None)
        self.return_targets_tried.pop(uav_id, None)
        self.returned_uavs.discard(uav_id)
        response.success = True
        response.message = f"Recovery confirmed for '{uav_id}'."
        self._signal_allocation_event('recovery_confirmed')
        return response

    def add_task_callback(self, request, response):
        with self.task_assignment_lock:
            return self._add_task_locked(request, response)

    def _add_task_locked(self, request, response):
        if self.task_manager is None:
            response.success = False
            response.message = 'Mission task manager is not ready.'
            return response
        response.success, response.message = self.task_manager.add_task(
            request.task
        )
        if response.success:
            self.inspection_completed_sim_time_sec = None
            if self.home_return_started:
                self._resume_mission_for_new_task()
            self._signal_allocation_event('new_task')
        return response

    def _resume_mission_for_new_task(self):
        # New work does not make a returning or landed UAV available.
        # Active returns continue; a confirmed UGV recovery can make a UAV
        # available for later allocation.
        self.home_return_started = False
        self.metrics_reported = False
        self.results_saved = False

    def cancel_task_callback(self, request, response):
        if self.task_manager is None:
            response.success = False
            response.message = 'Mission task manager is not ready.'
            return response
        task_id = str(request.task_id)
        uav_id = self.task_manager.assignments.get(task_id)
        task = self.task_manager.tasks.get(task_id)
        if not self.task_manager.cancel(task_id):
            response.success = False
            response.message = f"Task '{task_id}' cannot be cancelled."
            return response
        if uav_id:
            navigation = self.active_navigation.get(uav_id)
            if (
                navigation is not None
                and navigation['task'].task_id == task_id
            ):
                self._cancel_uav_goal(uav_id)
                self.active_navigation.pop(uav_id, None)
            self.pending_navigation = [
                (queued, assigned_uav)
                for queued, assigned_uav in self.pending_navigation
                if queued.task_id != task_id
            ]
            self._publish_task_status(
                task_id, uav_id, UAVState.FAILED, 0.0,
                'Task cancelled by mission manager.',
            )
            if task is not None and self.task_manager.unfinished_for_uav(
                uav_id
            ):
                self._publish_task_status(
                    task_id, uav_id, UAVState.ASSIGNED, 0.0,
                    'Another committed task is queued.',
                )
            else:
                self._publish_task_status(
                    task_id, uav_id, UAVState.IDLE, 0.0,
                    'Vehicle available for another allocation.',
                )
        response.success = True
        response.message = f"Task '{task_id}' cancelled."
        self._signal_allocation_event('task_cancelled')
        if self._all_tasks_terminal():
            self._begin_return_home()
        return response

    def allocator_planning_time_callback(self, message):
        if math.isfinite(message.data) and message.data >= 0.0:
            self.allocator_planning_time_sec = float(message.data)

    def unallocatable_tasks_callback(self, message):
        if self.task_manager is None:
            return
        try:
            failures = json.loads(message.data)
        except (TypeError, json.JSONDecodeError) as error:
            self.get_logger().error(
                f'Invalid allocator failure report: {error}'
            )
            return
        for failure in failures:
            task_id = str(failure.get('task_id', ''))
            reason = str(failure.get('reason', 'No feasible UAV route.'))
            if self.task_manager.fail(task_id, reason):
                self.mission_failed_reasons[task_id] = reason
                self.get_logger().error(
                    f"Task '{task_id}' marked failed: {reason}"
                )
        if self._all_tasks_terminal():
            self._begin_return_home()

    def _all_tasks_terminal(self):
        if self.task_manager is None:
            return False
        return (
            self.task_manager.completed
            | set(self.task_manager.failed)
            | self.task_manager.cancelled
        ) == set(self.task_manager.tasks)

    def prepare_tasks_when_ready(self):
        if self.tasks is not None:
            return

        now = time.monotonic()
        operational = self.mission_state.operational_uavs()
        available = self.mission_state.available_uavs()
        available_ids = {state.uav_id for state in available}
        if not operational:
            wait_reason = 'No connected, operational UAV status is available.'
        elif not available:
            wait_reason = (
                'Operational UAVs are busy; waiting for one to be ready.'
            )
        elif not all(
            uav_id in self.latest_battery_percentage
            and uav_id in self.odometry
            and now - self.odometry_received_at.get(uav_id, 0.0) <= 3.0
            for uav_id in available_ids
        ):
            wait_reason = (
                'Waiting for fresh odometry and battery telemetry from an '
                'available UAV.'
            )
        elif not all(
            uav_id in self.takeoff_position_recorded
            for uav_id in self.uav_ids
        ):
            self._capture_takeoff_positions(self.uav_ids)
            wait_reason = (
                'Waiting to capture each UAV initial position in the map '
                'frame.'
            )
        elif not self._configure_operating_bounds():
            wait_reason = (
                'Waiting for the controller and altitude map fusers to apply '
                'each UAV operating region.'
            )
        else:
            infeasible_tasks = {}
            self.tasks = tasks_from_mission(
                self.mission_data, infeasible_tasks
            )
            self.task_manager = TaskManager(
                self.tasks,
                self.frame_id,
                maximum_altitude_m=self.maximum_altitude_m,
                vertical_half_extent_m=self.vertical_half_extent_m,
                vertical_clearance_m=self.vertical_clearance_m,
                inspection_altitudes_m=self.inspection_altitudes_m,
                infeasible_tasks=infeasible_tasks,
            )
            self.mission_failed_reasons.update(infeasible_tasks)
            for task_id, reason in infeasible_tasks.items():
                self.get_logger().error(reason)
            self.mission_started_at = time.monotonic()
            self.mission_started_sim_time_sec = self._sim_time_sec()
            self.inspection_completed_sim_time_sec = None
            self.mission_started_at_utc = datetime.now(timezone.utc)
            self.energy_consumed_wh = {
                uav_id: 0.0 for uav_id in self.uav_ids
            }
            self.energy_interval_counts = {
                uav_id: 0 for uav_id in self.uav_ids
            }
            self.energy_samples.clear()
            self.initial_battery_percentage = {
                uav_id: self.latest_battery_percentage[uav_id]
                for uav_id in available_ids
                if uav_id in self.latest_battery_percentage
            }
            self.initial_estimated_battery_percentage = {
                uav_id: self.latest_estimated_battery_percentage[uav_id]
                for uav_id in available_ids
                if uav_id in self.latest_estimated_battery_percentage
            }
            self.get_logger().info(
                f'Prepared {len(self.tasks)} tasks for task management. '
                f'Ready UAVs: {", ".join(sorted(available_ids))}.'
            )
            if self._all_tasks_terminal():
                self._begin_return_home()
            else:
                self._signal_allocation_event('mission_ready')
            self.wait_reason = None
            return

        if wait_reason != self.wait_reason:
            self.get_logger().info(wait_reason)
            self.wait_reason = wait_reason

    def get_unassigned_tasks_callback(self, request, response):
        task_manager = self.task_manager
        if task_manager is None:
            return response
        response.tasks = task_manager.unassigned_tasks()
        now = time.monotonic()
        assigned_ids = {
            uav_id: task_manager.unfinished_for_uav(uav_id)
            for uav_id in self.uav_ids
        }
        ordered_ids = {}
        for uav_id, task_ids in assigned_ids.items():
            navigation = self.active_navigation.get(uav_id)
            current_id = (
                navigation['task'].task_id if navigation else None
            )
            pending_ids = [
                task.task_id for task, queued_uav_id
                in self.pending_navigation if queued_uav_id == uav_id
            ]
            ordered_ids[uav_id] = []
            if current_id in task_ids:
                ordered_ids[uav_id].append(current_id)
            ordered_ids[uav_id].extend(
                task_id for task_id in pending_ids
                if task_id in task_ids and task_id not in ordered_ids[uav_id]
            )
            ordered_ids[uav_id].extend(
                task_id for task_id in task_ids
                if task_id not in ordered_ids[uav_id]
            )
        for uav_id in self.uav_ids:
            for task_id in ordered_ids[uav_id]:
                task = task_manager.tasks[task_id]
                navigation = self.active_navigation.get(uav_id)
                inspecting = bool(
                    navigation
                    and navigation['task'].task_id == task_id
                    and navigation.get('phase') == 'inspection'
                )
                remaining_inspection = (
                    max(0.0, navigation['inspection_end'] - now)
                    if navigation is not None and inspecting
                    else float(task.estimated_duration)
                )
                response.assigned_tasks.append(task)
                response.assigned_uav_ids.append(uav_id)
                response.assigned_tasks_inspecting.append(inspecting)
                response.remaining_inspection_sec.append(
                    remaining_inspection
                )
        return response

    def assign_task_callback(self, request, response):
        with self.task_assignment_lock:
            return self._assign_task_locked(request, response)

    def _assign_task_locked(self, request, response):
        if self.task_manager is None:
            response.success = False
            response.message = 'Mission tasks are not ready for allocation.'
            return response

        state = self.mission_state.uav_states.get(request.uav_id)
        if (
            state is None
            or not state.connected
            or not state.operational
            or request.uav_id in self.failed_uavs
            or self._uav_is_returning(request.uav_id)
        ):
            response.success = False
            response.message = (
                f"UAV '{request.uav_id}' is unavailable for assignments."
            )
            return response

        assigned_to_uav = self.task_manager.unfinished_for_uav(
            request.uav_id
        )
        pending_for_uav = [
            task.task_id for task, uav_id in self.pending_navigation
            if uav_id == request.uav_id
        ]
        if state.mission_status in (UAVState.IDLE, UAVState.COMPLETED):
            eligible = not assigned_to_uav
        elif state.mission_status == UAVState.INSPECTING:
            active = self.active_navigation.get(request.uav_id)
            eligible = bool(
                active
                and active.get('phase') == 'inspection'
                and active['task'].task_id in assigned_to_uav
                and len(assigned_to_uav) == 1
                and not pending_for_uav
            )
        else:
            eligible = False
        if not eligible:
            response.success = False
            response.message = (
                f"UAV '{request.uav_id}' is not idle or inspecting with "
                'an available one-task queue slot.'
            )
            return response
        if len(assigned_to_uav) >= 2:
            response.success = False
            response.message = (
                f"UAV '{request.uav_id}' already has its current task and "
                'one queued task.'
            )
            return response

        task = self.task_manager.tasks.get(request.task_id)
        if task is not None:
            if task.frame_id != self.frame_id:
                response.success = False
                response.message = (
                    f"Task target frame '{task.frame_id}' does not match "
                    f"mission frame '{self.frame_id}'."
                )
                return response
            target = (
                float(task.target_position.x),
                float(task.target_position.y),
                float(task.target_position.z),
            )
            if not self._position_is_within_bounds(
                request.uav_id, target, f"task '{request.task_id}'"
            ):
                response.success = False
                response.message = (
                    'Task target is outside this UAV operating region.'
                )
                return response
            for assigned_task_id, assigned_uav_id in (
                self.task_manager.assignments.items()
            ):
                if (
                    assigned_uav_id == request.uav_id
                    or assigned_task_id in self.task_manager.completed
                    or assigned_task_id in self.task_manager.failed
                    or assigned_task_id in self.task_manager.cancelled
                ):
                    continue
                assigned_task = self.task_manager.tasks[assigned_task_id]
                assigned_target = (
                    float(assigned_task.target_position.x),
                    float(assigned_task.target_position.y),
                    float(assigned_task.target_position.z),
                )
                if math.dist(target, assigned_target) < (
                    self.minimum_separation_m
                ):
                    response.success = False
                    response.message = (
                        'Assignment rejected because its inspection target is '
                        'too close to another UAV task.'
                    )
                    return response

        response.success, response.message = self.task_manager.assign(
            request.task_id,
            request.uav_id,
            request.outbound_distance_m,
            request.return_distance_m,
        )
        if response.success:
            task = self.task_manager.tasks[request.task_id]
            already_busy = len(assigned_to_uav) > 0
            self.last_task_altitude[request.uav_id] = int(
                round(self.standard_flight_altitude)
            )
            self._publish_task_status(
                task.task_id,
                request.uav_id,
                UAVState.QUEUED if already_busy else UAVState.ASSIGNED,
                0.0,
                'Task queued behind existing work.' if already_busy
                else 'Task assignment confirmed.',
                publish_uav=not already_busy,
            )
            self.pending_navigation.append((task, request.uav_id))
            self.get_logger().info(response.message)
        return response

    def start_pending_navigation(self):
        if self.task_manager is None:
            return
        self._retry_deferred_path_fallbacks()
        now = time.monotonic()
        for uav_id, navigation in list(self.active_navigation.items()):
            if (
                navigation.get('phase') == 'inspection'
                and now >= navigation['inspection_end']
            ):
                task = navigation['task']
                climb = cruise_return_waypoint(
                    task, self.standard_flight_altitude
                )
                climb.header.stamp = self.get_clock().now().to_msg()
                navigation['path'].append(climb)
                if not self._route_is_within_bounds(
                    uav_id,
                    navigation['path'],
                    f"climb route '{task.task_id}'",
                ):
                    self._fail_task(
                        task.task_id,
                        uav_id,
                        'Climb route is outside the configured operating '
                        'region.',
                    )
                    continue
                navigation['index'] = len(navigation['path']) - 1
                navigation['phase'] = 'climbing'
                self.planned_distance_m += abs(
                    float(task.target_position.z)
                    - self.standard_flight_altitude
                )
                trajectory = NavPath()
                trajectory.header = climb.header
                trajectory.poses = navigation['path']
                self.trajectory_publishers[uav_id].publish(trajectory)
                self._publish_task_status(
                    task.task_id,
                    uav_id,
                    UAVState.EXECUTING,
                    0.99,
                    'Inspection complete; climbing to cruise altitude.',
                )
                self._send_next_waypoint(task.task_id, uav_id)
                continue
            if navigation.get('phase') != 'climb_complete':
                continue
            task = navigation['task']
            task_id = task.task_id
            with self.task_assignment_lock:
                if not self.task_manager.complete(task_id, uav_id):
                    self._fail_task(
                        task_id,
                        uav_id,
                        'Inspection ended but task ownership was lost; '
                        'completion was not recorded.',
                    )
                    continue
                self.inspection_time_sec += float(
                    task.estimated_duration
                )
                del self.active_navigation[uav_id]
                if (
                    len(self.task_manager.completed)
                    == len(self.task_manager.tasks)
                ):
                    self.inspection_completed_sim_time_sec = (
                        self._sim_time_sec()
                    )
                remaining_tasks = (
                    self.task_manager.unfinished_for_uav(uav_id)
                )
                self._publish_task_status(
                    task_id, uav_id, UAVState.COMPLETED, 1.0,
                    'Inspection duration completed.',
                )
                if remaining_tasks:
                    self._publish_task_status(
                        remaining_tasks[0],
                        uav_id,
                        UAVState.ASSIGNED,
                        0.0,
                        'Another committed task is ready.',
                    )
                elif self.task_manager.unassigned_tasks():
                    # Continue bidding for globally unassigned work. The
                    # allocator will include this UAV only after it is idle,
                    # while an inspecting peer may use its own one available
                    # queue slot independently.
                    self._signal_allocation_event('task_completed')
                elif self._has_deferred_fallback_for(uav_id):
                    # Keep the newly idle peer available for a task whose
                    # original route failed; the state update below lets the
                    # deferred path check run against its idle pose.
                    pass
                else:
                    # A UAV returns as soon as its own committed task queue
                    # is empty and no globally unassigned task remains.
                    self._begin_uav_return_locked(
                        uav_id,
                        'UAV completed all assigned inspection tasks.',
                    )
                if self._all_tasks_terminal():
                    self._begin_return_home_locked()
        if not self.pending_navigation:
            return
        pending = []
        for task, uav_id in self.pending_navigation:
            if uav_id in self.active_navigation:
                pending.append((task, uav_id))
                continue
            odometry = self.odometry.get(uav_id)
            if (
                odometry is None
                or now - self.odometry_received_at.get(uav_id, 0.0) > 3.0
            ):
                pending.append((task, uav_id))
                continue
            current = odometry.pose.pose.position
            if not self._position_is_within_bounds(
                uav_id,
                (current.x, current.y, current.z),
                f"current position before task '{task.task_id}'",
            ):
                self._fail_task(
                    task.task_id,
                    uav_id,
                    'Current position is outside the configured operating '
                    'region.',
                )
                continue
            inspection_altitude = int(round(task.target_position.z))
            if not math.isclose(
                float(task.target_position.z), inspection_altitude,
                abs_tol=1e-6,
            ):
                self._fail_task(
                    task.task_id, uav_id,
                    'Inspection altitude is not a configured planner band.',
                )
                continue
            inspection_planner = self.planner_clients[uav_id].get(
                inspection_altitude
            )
            if inspection_planner is None:
                self._fail_task(
                    task.task_id, uav_id,
                    'No inspection planner map is configured at '
                    f'{inspection_altitude} m.',
                )
                continue
            if not inspection_planner.server_is_ready():
                pending.append((task, uav_id))
                continue
            altitude = inspection_altitude
            preplanned = self.preplanned_routes.get(task.task_id)
            if preplanned is not None:
                route, altitude = preplanned
                self.preplanned_routes.pop(task.task_id, None)
                self.active_navigation[uav_id] = {
                    'task': task,
                    'path': route,
                    'index': 0,
                    'altitude': altitude,
                    'replanning': False,
                    'last_planned_position': (
                        odometry.pose.pose.position.x,
                        odometry.pose.pose.position.y,
                    ),
                }
                self.planned_distance_m += self._path_length(route)
                self._publish_task_status(
                    task.task_id, uav_id, UAVState.EXECUTING, 0.0,
                    'Starting on the validated fallback route.',
                )
                self._send_next_waypoint(task.task_id, uav_id)
                continue
            self.active_navigation[uav_id] = {
                'task': task,
                'path': [],
                'index': 0,
                'altitude': altitude,
                'replanning': False,
                'last_planned_position': (
                    odometry.pose.pose.position.x,
                    odometry.pose.pose.position.y,
                ),
            }
            self._publish_task_status(
                task.task_id, uav_id, UAVState.EXECUTING, 0.0,
                'Planning route.',
            )
            self._request_task_path(task, uav_id, odometry)
        self.pending_navigation = pending

    def _request_task_path(self, task, uav_id, odometry):
        altitude = int(round(float(task.target_position.z)))
        if not math.isclose(
            float(task.target_position.z), altitude, abs_tol=1e-6
        ):
            self._handle_path_failure(
                task.task_id, uav_id,
                'Inspection altitude is not a configured planner band.',
            )
            return
        planner = self.planner_clients[uav_id].get(altitude)
        if planner is None or not planner.server_is_ready():
            self._handle_path_failure(
                task.task_id, uav_id,
                f'No ready planner map is configured at {altitude} m.',
            )
            return
        request = ComputePathToPose.Goal()
        request.start = PoseStamped()
        request.start.header.frame_id = self.frame_id
        request.start.header.stamp = self.get_clock().now().to_msg()
        request.start.pose = odometry.pose.pose
        request.start.pose.position.z = float(altitude)
        request.goal = PoseStamped()
        request.goal.header.frame_id = task.frame_id
        request.goal.header.stamp = self.get_clock().now().to_msg()
        request.goal.pose.position.x = float(task.target_position.x)
        request.goal.pose.position.y = float(task.target_position.y)
        request.goal.pose.position.z = float(altitude)
        request.goal.pose.orientation.w = 1.0
        start = request.start.pose.position
        self.get_logger().info(
            f"Planning task '{task.task_id}' in frame '{self.frame_id}': "
            f'start=({start.x:.2f}, {start.y:.2f}, {start.z:.2f}), '
            f'goal=({request.goal.pose.position.x:.2f}, '
            f'{request.goal.pose.position.y:.2f}, {altitude:.2f})'
        )
        request.use_start = True
        request.planner_id = 'GridBased'
        started_at = time.perf_counter()
        future = planner.send_goal_async(request)
        future.add_done_callback(
            partial(
                self._path_goal_received, task.task_id, uav_id, started_at
            )
        )

    def _path_goal_received(self, task_id, uav_id, started_at, future):
        try:
            goal_handle = future.result()
        except Exception as error:
            self._record_planning_time(started_at)
            self._handle_path_failure(
                task_id, uav_id, f'Path request failed: {error}'
            )
            return
        if not goal_handle.accepted:
            self._record_planning_time(started_at)
            self._handle_path_failure(
                task_id, uav_id, 'Planner rejected the route request.'
            )
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            partial(
                self._path_result_received, task_id, uav_id, started_at
            )
        )

    def _path_result_received(self, task_id, uav_id, started_at, future):
        navigation = self.active_navigation.get(uav_id)
        if navigation is None or navigation['task'].task_id != task_id:
            return
        self._record_planning_time(started_at)
        try:
            wrapped_result = future.result()
        except Exception as error:
            self._handle_path_failure(
                task_id, uav_id, f'Path result failed: {error}'
            )
            return
        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            self._handle_path_failure(
                task_id, uav_id, 'No feasible route was found.'
            )
            return
        path = wrapped_result.result.path
        if not path.poses:
            self._handle_path_failure(
                task_id, uav_id, 'Planner returned an empty path.'
            )
            return
        task_manager = self.task_manager
        if task_manager is None:
            self.get_logger().error(
                f"Cannot handle the planner result for '{task_id}': "
                'task manager is not initialized.'
            )
            return
        task = task_manager.tasks[task_id]
        altitude = float(task.target_position.z)
        route = []
        for pose in path.poses:
            waypoint = PoseStamped()
            waypoint.header = pose.header
            waypoint.pose = pose.pose
            waypoint.pose.position.z = float(altitude)
            route.append(waypoint)
        self._finish_task_path(
            task_id, uav_id, route, path.header
        )

    def _finish_task_path(self, task_id, uav_id, route, path_header):
        navigation = self.active_navigation.get(uav_id)
        if navigation is None or navigation['task'].task_id != task_id:
            return
        task = navigation['task']
        route_endpoint = route[-1].pose.position
        endpoint_error_m = math.hypot(
            float(route_endpoint.x) - float(task.target_position.x),
            float(route_endpoint.y) - float(task.target_position.y),
        )
        if endpoint_error_m > self.inspection_path_tolerance_m:
            self._fail_task(
                task_id,
                uav_id,
                'Planned route endpoint is outside the allowed inspection '
                f'tolerance ({endpoint_error_m:.3f} m > '
                f'{self.inspection_path_tolerance_m:.3f} m).',
            )
            return
        inspection_waypoint = PoseStamped()
        inspection_waypoint.header = route[-1].header
        inspection_waypoint.pose = route[-1].pose
        # Preserve the collision-checked planner endpoint. Do not append an
        # unchecked exact-target waypoint that may lie inside an obstacle.
        inspection_waypoint.pose.position.z = float(task.target_position.z)
        route.append(inspection_waypoint)
        if not self._route_is_within_bounds(
            uav_id, route, f"task route '{task_id}'"
        ):
            self._fail_task(
                task_id,
                uav_id,
                'Planned route contains a waypoint outside the configured '
                'operating region.',
            )
            return
        self.active_navigation[uav_id]['path'] = route
        start_position = self.odometry[uav_id].pose.pose.position
        self.planned_distance_m += (
            abs(float(task.target_position.z) - float(start_position.z))
            + self._path_length(route)
        )
        trajectory = NavPath()
        trajectory.header = path_header
        trajectory.poses = route
        self.trajectory_publishers[uav_id].publish(trajectory)
        odometry = self.odometry.get(uav_id)
        if odometry is not None:
            self.active_navigation[uav_id]['last_planned_position'] = (
                odometry.pose.pose.position.x,
                odometry.pose.pose.position.y,
            )
        self.active_navigation[uav_id]['replanning'] = False
        self.active_navigation[uav_id].pop('resume_index', None)
        self.active_navigation[uav_id]['index'] = 0
        self._send_next_waypoint(task_id, uav_id)

    def _handle_path_failure(self, task_id, uav_id, message):
        navigation = self.active_navigation.get(uav_id)
        if navigation is None or navigation['task'].task_id != task_id:
            return
        if navigation.get('replanning'):
            navigation['replanning'] = False
            navigation['index'] = navigation.pop(
                'resume_index', navigation['index']
            )
            odometry = self.odometry.get(uav_id)
            if odometry is not None:
                navigation['last_planned_position'] = (
                    odometry.pose.pose.position.x,
                    odometry.pose.pose.position.y,
                )
            self.get_logger().warning(
                f"Replan failed for '{task_id}'; continuing the existing "
                f'route: {message}'
            )
            self._send_next_waypoint(task_id, uav_id)
            return
        if navigation.get('fallback_pending'):
            return
        navigation['fallback_pending'] = True
        if not self._start_peer_path_fallback(task_id, uav_id, message):
            self._defer_path_fallback(task_id, uav_id, message)

    def _peer_has_assignment_slot(self, uav_id):
        task_manager = self.task_manager
        if task_manager is None:
            return False
        state = self.mission_state.uav_states.get(uav_id)
        if (
            state is None or not state.connected or not state.operational
            or uav_id in self.failed_uavs
            or self._uav_is_returning(uav_id)
        ):
            return False
        assigned = task_manager.unfinished_for_uav(uav_id)
        pending = [
            task.task_id for task, assigned_uav in self.pending_navigation
            if assigned_uav == uav_id
        ]
        if state.mission_status in (UAVState.IDLE, UAVState.COMPLETED):
            return (
                not assigned and not pending
                and uav_id not in self.active_navigation
            )
        if state.mission_status != UAVState.INSPECTING:
            return False
        navigation = self.active_navigation.get(uav_id)
        return bool(
            navigation
            and navigation.get('phase') == 'inspection'
            and navigation['task'].task_id in assigned
            and len(assigned) == 1
            and not pending
        )

    def _fallback_energy_feasible(self, uav_id, task, outbound_distance_m):
        battery = self._current_return_battery(uav_id)
        configured_home = self.home_positions.get(uav_id)
        if (
            battery is None or configured_home is None
            or not math.isfinite(outbound_distance_m)
            or outbound_distance_m < 0.0
        ):
            return False
        home = (
            configured_home[0],
            configured_home[1],
            self.landing_altitude_m,
        )
        target = (
            float(task.target_position.x),
            float(task.target_position.y),
            float(task.target_position.z),
        )
        current_navigation = self.active_navigation.get(uav_id)
        remaining_inspection = 0.0
        if current_navigation is not None:
            if current_navigation.get('phase') != 'inspection':
                return False
            remaining_inspection = max(
                0.0,
                current_navigation.get('inspection_end', time.monotonic())
                - time.monotonic(),
            )
        inspection_energy = (
            (remaining_inspection + float(task.estimated_duration))
            / 60.0 * self.inspection_consumption_per_minute
        )
        outbound_energy = (
            outbound_distance_m * self.return_distance_safety_factor
            * self.battery_consumption_per_meter
        )

        def battery_after_return(destination):
            return_distance = (
                math.hypot(
                    target[0] - destination[0],
                    target[1] - destination[1],
                )
                + abs(target[2] - self.standard_flight_altitude)
                + abs(destination[2] - self.standard_flight_altitude)
            )
            return battery - (
                outbound_energy
                + inspection_energy
                + return_distance * self.return_distance_safety_factor
                * self.battery_consumption_per_meter
                + self.landing_energy_percentage
            )

        if battery_after_return(home) >= self.minimum_safe_battery_reserve:
            return True
        ugv_remaining = battery_after_return(self.ugv_location)
        if ugv_remaining >= self.minimum_safe_battery_reserve:
            return True
        return any(
            battery_after_return(target['position'])
            >= self.minimum_safe_battery_reserve
            for target in self.nearby_landing_targets
        )

    def _start_peer_path_fallback(self, task_id, source_uav, reason):
        task_manager = self.task_manager
        if task_manager is None:
            return False
        if task_id in self.path_fallback_attempts:
            return True
        peer_uavs = [uav_id for uav_id in self.uav_ids if uav_id != source_uav]
        if not peer_uavs:
            return False
        peer_uav = peer_uavs[0]
        if not self._peer_has_assignment_slot(peer_uav):
            return False
        odometry = self.odometry.get(peer_uav)
        if (
            odometry is None
            or time.monotonic() - self.odometry_received_at.get(
                peer_uav, 0.0
            ) > 3.0
        ):
            return False
        task = task_manager.tasks.get(task_id)
        if task is None:
            return False
        inspection_altitude = int(round(task.target_position.z))
        inspection_planner = self.planner_clients[peer_uav].get(
            inspection_altitude
        )
        if (
            inspection_planner is None
            or not inspection_planner.server_is_ready()
        ):
            return False
        altitude = inspection_altitude
        planner = self.planner_clients[peer_uav].get(altitude)
        if planner is None or not planner.server_is_ready():
            return False
        start = odometry.pose.pose.position
        estimated_distance = (
            math.hypot(
                float(start.x) - float(task.target_position.x),
                float(start.y) - float(task.target_position.y),
            )
            + abs(float(start.z) - float(task.target_position.z))
        )
        if not self._fallback_energy_feasible(
            peer_uav, task, estimated_distance
        ):
            return False

        request = ComputePathToPose.Goal()
        request.start = PoseStamped()
        request.start.header.frame_id = self.frame_id
        request.start.header.stamp = self.get_clock().now().to_msg()
        request.start.pose = odometry.pose.pose
        request.start.pose.position.z = float(altitude)
        request.goal = PoseStamped()
        request.goal.header.frame_id = task.frame_id
        request.goal.header.stamp = self.get_clock().now().to_msg()
        request.goal.pose.position.x = float(task.target_position.x)
        request.goal.pose.position.y = float(task.target_position.y)
        request.goal.pose.position.z = float(altitude)
        request.goal.pose.orientation.w = 1.0
        request.use_start = True
        request.planner_id = 'GridBased'
        with self.task_assignment_lock:
            task_status = task_manager.statuses.get(task_id)
            if task_status == 'ASSIGNED':
                if not task_manager.reserve_for_fallback(
                    task_id, source_uav, reason
                ):
                    return False
            elif task_status != 'RESERVED':
                return False
            self._publish_task_status(
                task_id, source_uav, UAVState.QUEUED, 0.0,
                'Original reservation released; validating peer path.',
                publish_uav=False,
            )
        started_at = time.perf_counter()
        self.path_fallback_attempts[task_id] = {
            'source_uav': source_uav,
            'peer_uav': peer_uav,
            'reason': reason,
            'altitude': altitude,
            'started_at': started_at,
        }
        self.path_fallback_counts[task_id] = (
            self.path_fallback_counts.get(task_id, 0) + 1
        )
        future = planner.send_goal_async(request)
        future.add_done_callback(partial(
            self._fallback_path_goal_received, task_id
        ))
        self.get_logger().warning(
            f"Path planning for '{task_id}' failed on '{source_uav}'; "
            f"checking a feasible path on '{peer_uav}'."
        )
        return True

    def _fallback_path_goal_received(self, task_id, future):
        context = self.path_fallback_attempts.get(task_id)
        if context is None:
            return
        try:
            goal_handle = future.result()
        except Exception as error:
            self._retry_or_finish_path_fallback(
                task_id, f"Peer path request failed: {error}"
            )
            return
        if not goal_handle.accepted:
            self._retry_or_finish_path_fallback(
                task_id, 'Peer planner rejected the route request.'
            )
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(partial(
            self._fallback_path_result_received, task_id
        ))

    def _fallback_path_result_received(self, task_id, future):
        context = self.path_fallback_attempts.get(task_id)
        if context is None:
            return
        task_manager = self.task_manager
        if task_manager is None:
            self.get_logger().error(
                f"Cannot complete fallback for '{task_id}': "
                'task manager is not initialized.'
            )
            return
        self._record_planning_time(context['started_at'])
        try:
            wrapped = future.result()
        except Exception as error:
            self._retry_or_finish_path_fallback(
                task_id, f"Peer path result failed: {error}"
            )
            return
        if (
            wrapped.status != GoalStatus.STATUS_SUCCEEDED
            or not wrapped.result.path.poses
        ):
            self._retry_or_finish_path_fallback(
                task_id, 'Neither UAV produced a valid inspection path.'
            )
            return

        task = task_manager.tasks.get(task_id)
        if task is None:
            self._retry_or_finish_path_fallback(
                task_id, 'Fallback task no longer exists.'
            )
            return
        inspection_altitude = int(round(task.target_position.z))
        endpoint = wrapped.result.path.poses[-1].pose.position
        endpoint_error_m = math.hypot(
            float(endpoint.x) - float(task.target_position.x),
            float(endpoint.y) - float(task.target_position.y),
        )
        if endpoint_error_m > self.inspection_path_tolerance_m:
            self._retry_or_finish_path_fallback(
                task_id,
                'Peer inspection path endpoint is outside the allowed '
                f'tolerance ({endpoint_error_m:.3f} m > '
                f'{self.inspection_path_tolerance_m:.3f} m).',
            )
            return
        source_uav = context['source_uav']
        peer_uav = context['peer_uav']
        if task is None or not self._peer_has_assignment_slot(peer_uav):
            self._defer_path_fallback(
                task_id, source_uav,
                'The peer became unavailable while its path was planned.',
            )
            return

        path = wrapped.result.path
        route = []
        for pose in path.poses:
            waypoint = PoseStamped()
            waypoint.header = pose.header
            waypoint.pose = pose.pose
            waypoint.pose.position.z = float(inspection_altitude)
            route.append(waypoint)
        final_pose = PoseStamped()
        final_pose.header = route[-1].header
        final_pose.pose = route[-1].pose
        # Keep the endpoint returned by the peer planner; only adjust the
        # flight altitude. The exact target may be occupied or inflated.
        final_pose.pose.position.z = float(task.target_position.z)
        route.append(final_pose)
        if not self._route_is_within_bounds(
            peer_uav, route, f"fallback route '{task_id}'"
        ):
            self._mark_path_fallback_unreachable(
                task_id,
                'Peer route is outside its configured operating region.',
            )
            return
        route_start = self.odometry[peer_uav].pose.pose.position
        route_distance = (
            abs(float(route_start.z) - float(task.target_position.z))
            + self._path_length(route)
        )
        if not self._fallback_energy_feasible(
            peer_uav, task, route_distance
        ):
            self._mark_path_fallback_unreachable(
                task_id,
                'Peer path exists, but the task plus safe return exceeds '
                'the available battery reserve.',
            )
            return
        target = (
            float(task.target_position.x),
            float(task.target_position.y),
            float(task.target_position.z),
        )
        home = self.home_positions.get(peer_uav)
        landing_home = (
            (home[0], home[1], self.landing_altitude_m)
            if home is not None else None
        )
        return_distance = (
            math.hypot(
                target[0] - landing_home[0],
                target[1] - landing_home[1],
            )
            + abs(target[2] - self.standard_flight_altitude)
            + abs(landing_home[2] - self.standard_flight_altitude)
            if landing_home is not None else 0.0
        )

        with self.task_assignment_lock:
            if not self._peer_has_assignment_slot(peer_uav):
                self._defer_path_fallback(
                    task_id, source_uav,
                    'The peer queue filled before fallback assignment.',
                )
                return
            if task_manager.statuses.get(task_id) == 'RESERVED':
                assigned = task_manager.assign_reserved(
                    task_id, peer_uav,
                    route_distance * self.return_distance_safety_factor,
                    return_distance * self.return_distance_safety_factor,
                )
            else:
                assigned = task_manager.transfer(
                    task_id, source_uav, peer_uav,
                    route_distance * self.return_distance_safety_factor,
                    return_distance * self.return_distance_safety_factor,
                )
            if not assigned:
                self._mark_path_fallback_unreachable(
                    task_id,
                    'Task ownership changed before fallback assignment.',
                )
                return

            self.path_fallback_attempts.pop(task_id, None)
            self.path_fallback_counts.pop(task_id, None)
            self.deferred_path_fallbacks.pop(task_id, None)
            self.active_navigation.pop(source_uav, None)
            peer_is_inspecting = (
                peer_uav in self.active_navigation
                and self.active_navigation[peer_uav].get('phase')
                == 'inspection'
            )
            if peer_is_inspecting:
                self.preplanned_routes[task_id] = (
                    route, inspection_altitude
                )
                self.pending_navigation.append((task, peer_uav))
                self._publish_task_status(
                    task_id, peer_uav, UAVState.QUEUED, 0.0,
                    'Fallback path validated; queued behind the inspection.',
                    publish_uav=False,
                )
            else:
                self.active_navigation[peer_uav] = {
                    'task': task,
                    'path': route,
                    'index': 0,
                    'altitude': inspection_altitude,
                    'replanning': False,
                    'last_planned_position': (
                        self.odometry[peer_uav].pose.pose.position.x,
                        self.odometry[peer_uav].pose.pose.position.y,
                    ),
                }
                self.planned_distance_m += route_distance
                self._publish_task_status(
                    task_id, peer_uav, UAVState.EXECUTING, 0.0,
                    'Peer path validated; executing fallback task.',
                )
                self._send_next_waypoint(task_id, peer_uav)

            remaining = task_manager.unfinished_for_uav(source_uav)
            if remaining:
                self._publish_task_status(
                    remaining[0], source_uav, UAVState.ASSIGNED, 0.0,
                    'Fallback task transferred; continuing queued work.',
                )
            elif task_manager.unassigned_tasks():
                self._publish_task_status(
                    '', source_uav, UAVState.COMPLETED, 1.0,
                    'Fallback task transferred; UAV is available.',
                )
                self._signal_allocation_event('path_fallback_transferred')
            else:
                self._begin_uav_return_locked(
                    source_uav,
                    'Path-failed task was transferred to the peer UAV.',
                )

    def _defer_path_fallback(self, task_id, source_uav, reason):
        task_manager = self.task_manager
        if task_manager is None:
            self.get_logger().error(
                f"Cannot defer fallback for '{task_id}': "
                'task manager is not initialized.'
            )
            return
        context = self.path_fallback_attempts.pop(task_id, None)
        if context is not None:
            source_uav = context['source_uav']
        task = task_manager.tasks.get(task_id)
        if task is None:
            return
        with self.task_assignment_lock:
            if task_manager.statuses.get(task_id) == 'ASSIGNED':
                if not task_manager.reserve_for_fallback(
                    task_id, source_uav, reason
                ):
                    return
            elif task_manager.statuses.get(task_id) != 'RESERVED':
                return
            self.deferred_path_fallbacks[task_id] = {
                'source_uav': source_uav,
                'reason': str(reason),
                'last_attempt': time.monotonic(),
            }
            self.active_navigation.pop(source_uav, None)
            self._publish_task_status(
                task_id, source_uav, UAVState.QUEUED, 0.0,
                'Path failed; reserved until the peer has a safe queue slot.',
                publish_uav=False,
            )
            remaining = task_manager.unfinished_for_uav(source_uav)
            if remaining:
                self._publish_task_status(
                    remaining[0], source_uav, UAVState.ASSIGNED, 0.0,
                    'A failed path was deferred; continuing queued work.',
                )
            else:
                self._publish_task_status(
                    '', source_uav, UAVState.COMPLETED, 1.0,
                    'Path-failed task is reserved for peer fallback.',
                )
                if task_manager.unassigned_tasks():
                    self._signal_allocation_event('path_fallback_deferred')
                else:
                    self._begin_uav_return_locked(
                        source_uav,
                        'No remaining task is assigned to this UAV.',
                    )
        self.get_logger().warning(
            f"Task '{task_id}' is reserved for peer path fallback: {reason}"
        )

    def _retry_or_finish_path_fallback(self, task_id, reason):
        context = self.path_fallback_attempts.get(task_id)
        if context is None:
            return
        source_uav = context['source_uav']
        attempts = self.path_fallback_counts.get(task_id, 0)
        if attempts < 2:
            self._defer_path_fallback(task_id, source_uav, reason)
        else:
            self._mark_path_fallback_unreachable(
                task_id,
                f'{reason} Peer path attempt limit was reached.',
            )

    def _retry_deferred_path_fallbacks(self):
        task_manager = self.task_manager
        if task_manager is None:
            return
        for task_id, context in list(self.deferred_path_fallbacks.items()):
            if task_id in self.path_fallback_attempts:
                continue
            if time.monotonic() - context['last_attempt'] < 1.0:
                continue
            source_uav = context['source_uav']
            peer_uav = next(
                (uav_id for uav_id in self.uav_ids if uav_id != source_uav),
                None,
            )
            if peer_uav is None:
                self._mark_path_fallback_unreachable(
                    task_id, 'No peer UAV is configured for path fallback.'
                )
                continue
            peer_state = self.mission_state.uav_states.get(peer_uav)
            if peer_state is not None and peer_state.mission_status in (
                UAVState.RETURNING, UAVState.LANDING, UAVState.LANDED,
                UAVState.RECOVERING, UAVState.FAILED,
            ):
                self._mark_path_fallback_unreachable(
                    task_id,
                    f"Peer '{peer_uav}' is unavailable for fallback.",
                )
                continue
            context['last_attempt'] = time.monotonic()
            if self._peer_has_assignment_slot(peer_uav):
                odometry = self.odometry.get(peer_uav)
                task = task_manager.tasks.get(task_id)
                if (
                    odometry is not None and task is not None
                    and time.monotonic() - self.odometry_received_at.get(
                        peer_uav, 0.0
                    ) <= 3.0
                    and self._current_return_battery(peer_uav) is not None
                ):
                    position = odometry.pose.pose.position
                    distance = (
                        math.hypot(
                            float(position.x) - float(task.target_position.x),
                            float(position.y) - float(task.target_position.y),
                        )
                        + abs(
                            float(position.z)
                            - float(task.target_position.z)
                        )
                    )
                    if not self._fallback_energy_feasible(
                        peer_uav, task, distance
                    ):
                        self._mark_path_fallback_unreachable(
                            task_id,
                            'Peer battery cannot cover the task, landing, '
                            'and required return reserve.',
                        )
                        continue
            if self._start_peer_path_fallback(
                task_id, source_uav, context['reason']
            ):
                continue

    def _has_deferred_fallback_for(self, uav_id):
        return any(
            context.get('source_uav') != uav_id
            for context in self.deferred_path_fallbacks.values()
        )

    def _mark_path_fallback_unreachable(self, task_id, reason):
        context = self.path_fallback_attempts.pop(task_id, None)
        if context is not None:
            source_uav = context['source_uav']
        else:
            deferred = self.deferred_path_fallbacks.get(task_id, {})
            source_uav = deferred.get('source_uav')
        if source_uav is None or self.task_manager is None:
            return
        with self.task_assignment_lock:
            if not self.task_manager.fail(task_id, reason):
                return
            self.deferred_path_fallbacks.pop(task_id, None)
            self.path_fallback_counts.pop(task_id, None)
            self.preplanned_routes.pop(task_id, None)
            self.active_navigation.pop(source_uav, None)
            self.pending_navigation = [
                (task, uav_id) for task, uav_id in self.pending_navigation
                if task.task_id != task_id
            ]
            self.mission_failed_reasons[task_id] = str(reason)
            self.navigation_failures += 1
            self._publish_task_status(
                task_id, source_uav, UAVState.FAILED, 0.0,
                f'UNREACHABLE: {reason}',
            )
            remaining = self.task_manager.unfinished_for_uav(source_uav)
            if remaining:
                self._publish_task_status(
                    remaining[0], source_uav, UAVState.ASSIGNED, 0.0,
                    'Unreachable task removed; continuing queued work.',
                )
            elif self.task_manager.unassigned_tasks():
                self._publish_task_status(
                    '', source_uav, UAVState.COMPLETED, 1.0,
                    'Unreachable task removed; UAV is available.',
                )
                self._signal_allocation_event('task_unreachable')
            else:
                self._begin_uav_return_locked(
                    source_uav, 'No remaining task is assigned to this UAV.'
                )
            if self._all_tasks_terminal():
                self._begin_return_home_locked()
        self.get_logger().error(
            f"Task '{task_id}' marked unreachable: {reason}"
        )

    def _send_next_waypoint(self, task_id, uav_id):
        navigation = self.active_navigation[uav_id]
        index = navigation['index']
        route = navigation['path']
        if index >= len(route):
            if navigation.get('phase') == 'climbing':
                navigation['phase'] = 'climb_complete'
                return
            navigation['phase'] = 'inspection'
            duration = float(navigation['task'].estimated_duration)
            navigation['inspection_end'] = time.monotonic() + duration
            self._publish_task_status(
                task_id, uav_id, UAVState.INSPECTING, 0.99,
                'Holding position for inspection.',
            )
            self._signal_allocation_event('inspection_started')
            return
        if not self._route_is_within_bounds(
            uav_id, route, f"task route '{task_id}'"
        ):
            self._fail_task(
                task_id,
                uav_id,
                'Planned route contains a waypoint outside the configured '
                'operating region.',
            )
            return
        if not self.waypoint_clients[uav_id].server_is_ready():
            self._fail_task(
                task_id, uav_id,
                'GoToWaypoint action server is unavailable.',
            )
            return
        request = GoToWaypoint.Goal()
        request.target_pose = route[index]
        request.acceptance_radius = self.acceptance_radius_m
        request.max_velocity = self.max_velocity_mps
        future = self.waypoint_clients[uav_id].send_goal_async(
            request,
            feedback_callback=partial(
                self._waypoint_feedback, task_id, uav_id, index
            ),
        )
        future.add_done_callback(
            partial(self._waypoint_goal_received, task_id, uav_id, index)
        )

    def _begin_return_home(self):
        with self.task_assignment_lock:
            return self._begin_return_home_locked()

    def _begin_return_home_locked(self):
        if not self._all_tasks_terminal():
            return
        self.home_return_started = True
        for uav_id in self.uav_ids:
            self._begin_uav_return_locked(
                uav_id, 'All inspection tasks are terminal.'
            )
        self._report_metrics_if_complete()

    def _begin_uav_return_locked(self, uav_id, reason):
        """Start one UAV's return once its committed task queue is empty."""
        if (
            self.task_manager is None
            or uav_id in self.failed_uavs
            or uav_id in self.recovery_state.returning
            or uav_id in self.return_targets
            or uav_id in self.return_navigation
            or uav_id in self.pending_return_uavs
            or uav_id in self.returned_uavs
            or self.task_manager.unfinished_for_uav(uav_id)
        ):
            return False
        self.last_task_altitude.setdefault(
            uav_id, int(round(self.standard_flight_altitude))
        )
        return self._queue_completion_return(uav_id, reason)

    def _uav_is_returning(self, uav_id):
        return any((
            uav_id in self.recovery_state.returning,
            uav_id in self.return_targets,
            uav_id in self.return_navigation,
            uav_id in self.pending_return_uavs,
            uav_id in self.returned_uavs,
        ))

    def _all_uavs_return_terminal(self):
        return set(self.uav_ids).issubset(self.returned_uavs)

    def _all_uavs_confirmed_landed(self):
        now = time.monotonic()
        if any(
            now - self.mission_state.last_state_update.get(uav_id, 0.0)
            > self.mission_state.state_timeout_sec
            for uav_id in self.uav_ids
        ):
            return False
        return all_uavs_confirmed_landed(
            self.uav_ids,
            self.mission_state.uav_states,
            self.successful_landed_uavs,
            self.failed_uavs | self.return_failures,
            self.recovery_state.returning,
        )

    def _schedule_simulation_shutdown(self):
        if (
            self.shutdown_initiated
            or self.shutdown_timer is not None
            or not self._all_uavs_confirmed_landed()
        ):
            return
        if self.simulation_shutdown_delay_sec == 0.0:
            self._finish_simulation_after_landing()
            return
        self.shutdown_delay_elapsed = False
        self.shutdown_timer = self.create_timer(
            self.simulation_shutdown_delay_sec,
            self._finish_simulation_after_landing,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )
        self.get_logger().info(
            'Both UAVs have confirmed successful landing; saving final '
            f'results and shutting down in '
            f'{self.simulation_shutdown_delay_sec:.1f} seconds.'
        )

    def _finish_simulation_after_landing(self):
        if self.shutdown_timer is not None:
            self.shutdown_timer.cancel()
            self.shutdown_timer = None
        if self.shutdown_initiated:
            return
        if not self._all_uavs_confirmed_landed():
            self.shutdown_delay_elapsed = False
            return

        self.shutdown_delay_elapsed = True
        self._report_metrics_if_complete(finalize=True)
        if not self.results_saved:
            result_path = self._save_mission_results(
                'COMPLETED', self._elapsed_sim_time_sec()
            )
            if result_path is None:
                self.get_logger().error(
                    'Final mission results could not be saved; keeping the '
                    'simulation alive rather than shutting down without them.'
                )
                self.shutdown_delay_elapsed = False
                self._schedule_simulation_shutdown_retry()
                return
        if (
            self.require_algorithm_comparison_results
            and not self._benchmark_comparison_complete()
        ):
            self.get_logger().info(
                'Waiting for the current three-algorithm comparison files '
                'to finish writing before shutdown.'
            )
            self._schedule_simulation_shutdown_retry()
            return

        self.shutdown_initiated = True
        self.get_logger().info(
            'Final results are saved. Shutting down the simulation launch '
            'gracefully.'
        )
        if rclpy.ok():
            rclpy.shutdown()

    def _schedule_simulation_shutdown_retry(self):
        if self.shutdown_initiated or self.shutdown_timer is not None:
            return
        self.shutdown_timer = self.create_timer(
            1.0,
            self._finish_simulation_after_landing,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )

    def _benchmark_comparison_complete(self):
        expected_algorithms = {'existing_algorithm', 'milp', 'greedy'}
        run_directories = sorted(
            self.results_dir.glob('*tasks*waypoints/run_*'),
            key=lambda path: path.stat().st_mtime,
            reverse=True,
        )
        if not run_directories:
            return False
        run_dir = run_directories[0]
        required_files = (
            'algorithm_comparison.csv',
            'algorithm_comparison.json',
            'comparison_summary.txt',
            'comparison.csv',
            'comparison.json',
            'summary.txt',
        )
        if any(
            not (run_dir / filename).is_file()
            or (run_dir / filename).stat().st_size == 0
            for filename in required_files
        ):
            return False
        try:
            with (run_dir / 'comparison.json').open(
                encoding='utf-8'
            ) as stream:
                comparison = json.load(stream)
            with (run_dir / 'algorithm_comparison.json').open(
                encoding='utf-8'
            ) as stream:
                aggregate = json.load(stream)
            with (run_dir / 'comparison.csv').open(
                newline='', encoding='utf-8'
            ) as stream:
                legacy_rows = list(csv.DictReader(stream))
            with (run_dir / 'algorithm_comparison.csv').open(
                newline='', encoding='utf-8'
            ) as stream:
                aggregate_rows = list(csv.DictReader(stream))
        except (OSError, ValueError, csv.Error):
            return False
        if set(comparison.get('algorithms', {})) != expected_algorithms:
            return False
        if len(aggregate.get('algorithms', [])) != len(expected_algorithms):
            return False
        if len(legacy_rows) != len(expected_algorithms):
            return False
        if len(aggregate_rows) != len(expected_algorithms):
            return False
        return all(
            (run_dir / algorithm / filename).is_file()
            and (run_dir / algorithm / filename).stat().st_size > 0
            for algorithm in expected_algorithms
            for filename in ('metrics.json', 'summary.txt')
        )

    def start_pending_returns(self):
        self._check_return_landings()
        if not self.pending_return_uavs:
            return
        now = time.monotonic()
        for uav_id in list(self.pending_return_uavs):
            odometry = self.odometry.get(uav_id)
            if (
                odometry is None
                or now - self.odometry_received_at.get(uav_id, 0.0) > 3.0
                or not self._operating_bounds_are_configured(uav_id)
            ):
                continue
            altitude = self.last_task_altitude[uav_id]
            planner = self.planner_clients[uav_id].get(altitude)
            if planner is None or not planner.server_is_ready():
                continue
            return_target = self.return_targets[uav_id]
            target_position = return_target['position']
            self.pending_return_uavs.remove(uav_id)
            self.return_navigation[uav_id] = {
                'planning': True,
                'target': return_target,
            }
            request = ComputePathToPose.Goal()
            request.start = PoseStamped()
            request.start.header.frame_id = odometry.header.frame_id
            request.start.header.stamp = self.get_clock().now().to_msg()
            request.start.pose = odometry.pose.pose
            request.start.pose.position.z = float(altitude)
            request.goal = PoseStamped()
            request.goal.header.frame_id = odometry.header.frame_id
            request.goal.header.stamp = self.get_clock().now().to_msg()
            request.goal.header.frame_id = return_target.get(
                'frame_id', odometry.header.frame_id
            )
            request.goal.pose.position.x = target_position[0]
            request.goal.pose.position.y = target_position[1]
            request.goal.pose.position.z = float(altitude)
            request.goal.pose.orientation.w = 1.0
            request.use_start = True
            request.planner_id = 'GridBased'
            started_at = time.perf_counter()
            future = planner.send_goal_async(request)
            future.add_done_callback(
                partial(
                    self._return_path_goal_received,
                    uav_id, altitude, started_at,
                )
            )
            self._publish_task_status(
                'RETURN_HOME', uav_id, UAVState.RETURNING, 0.0,
                f"Planning return to '{return_target['name']}'.",
            )

    def _return_path_goal_received(
        self, uav_id, altitude, started_at, future
    ):
        if uav_id not in self.return_navigation:
            return
        try:
            goal_handle = future.result()
        except Exception as error:
            self._record_planning_time(started_at)
            self._try_alternate_return_target(
                uav_id, f'Return planning failed: {error}'
            )
            return
        if not goal_handle.accepted:
            self._record_planning_time(started_at)
            self._try_alternate_return_target(
                uav_id, 'Planner rejected the return path.'
            )
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            partial(
                self._return_path_result_received,
                uav_id, altitude, started_at,
            )
        )

    def _return_path_result_received(
        self, uav_id, altitude, started_at, future
    ):
        navigation = self.return_navigation.get(uav_id)
        if navigation is None:
            return
        self._record_planning_time(started_at)
        try:
            wrapped_result = future.result()
        except Exception as error:
            self._try_alternate_return_target(
                uav_id, f'Return path failed: {error}'
            )
            return
        path = wrapped_result.result.path
        if (
            wrapped_result.status != GoalStatus.STATUS_SUCCEEDED
            or not path.poses
        ):
            self._try_alternate_return_target(
                uav_id, 'No feasible return path was found.'
            )
            return
        route = []
        for pose in path.poses:
            waypoint = PoseStamped()
            waypoint.header = pose.header
            waypoint.pose = pose.pose
            waypoint.pose.position.z = float(altitude)
            route.append(waypoint)
        target = navigation['target']
        target_position = target['position']
        final_pose = PoseStamped()
        final_pose.header = path.header
        final_pose.pose.position.x = target_position[0]
        final_pose.pose.position.y = target_position[1]
        final_pose.pose.position.z = target_position[2]
        final_pose.pose.orientation.w = 1.0
        route.append(final_pose)
        if not self._route_is_within_bounds(
            uav_id, route, 'return or landing route'
        ):
            self._try_alternate_return_target(
                uav_id,
                'The planned return route is outside the configured '
                'operating region.',
            )
            return
        current_position = self.odometry[uav_id].pose.pose.position
        route_distance_m = (
            abs(float(current_position.z) - float(altitude))
            + self._path_length(route)
        )
        battery = self._current_return_battery(uav_id)
        energy_required = (
            route_distance_m
            * self.return_distance_safety_factor
            * self.battery_consumption_per_meter
            + self.landing_energy_percentage
        )
        minimum_remaining = float(target.get(
            'minimum_remaining_battery_percentage',
            self.return_minimum_reserve_percentage.get(
                uav_id, self.minimum_safe_battery_reserve
            ),
        ))
        battery_after_landing = (
            battery - energy_required if battery is not None else None
        )
        target_is_infeasible = (
            battery_after_landing is None
            or battery_after_landing < minimum_remaining
        )
        if target_is_infeasible:
            self._try_alternate_return_target(
                uav_id,
                'The planned route would violate the battery safety reserve.',
            )
            return
        self.return_distance_m += route_distance_m
        self.planned_distance_m += route_distance_m
        navigation.update({'path': route, 'index': 0})
        trajectory = NavPath()
        trajectory.header = path.header
        trajectory.poses = route
        self.trajectory_publishers[uav_id].publish(trajectory)
        self._send_next_return_waypoint(uav_id)

    def _try_alternate_return_target(self, uav_id, reason):
        navigation = self.return_navigation.get(uav_id, {})
        attempted = navigation.get('target', {}).get('name')
        if attempted is None:
            attempted = self.return_targets.get(uav_id, {}).get('name')

        options = []
        tried = self.return_targets_tried.setdefault(uav_id, set())
        if uav_id in self.priority_returning_uavs:
            candidates = []
            if attempted == 'home_position':
                candidates.append({
                    'name': 'UGV',
                    'position': self.ugv_location,
                    'frame_id': self.ugv_frame_id,
                    'recovery': True,
                })
            candidates.extend(self.nearby_landing_targets)
            for candidate in candidates:
                if candidate.get('name') in tried:
                    continue
                option = self._single_feasible_return_target(
                    uav_id, candidate
                )
                if option is not None:
                    options = [option]
                    break

        if not options:
            self._finish_return_failed(
                uav_id,
                f'{reason} The configured home, UGV, and validated nearby '
                'landing options were exhausted or not battery-safe.',
            )
            return
        self.return_navigation.pop(uav_id, None)
        self.return_targets[uav_id] = options[0]
        self.return_candidates[uav_id] = [options[0]]
        if attempted is not None:
            tried.add(attempted)
        tried.add(options[0]['name'])
        self.pending_return_uavs.add(uav_id)
        if options[0].get('recovery', False):
            self.recovery_state.begin_return(uav_id)
        self.get_logger().warning(
            f"UAV '{uav_id}' cannot safely continue to '{attempted}'; "
            f"trying the next landing priority '{options[0]['name']}'. "
            f'{reason}'
        )
        self._publish_task_status(
            'EMERGENCY_LANDING', uav_id, UAVState.RETURNING, 0.0,
            f'{reason} Proceeding to {options[0]["name"]}.',
        )

    def _send_next_return_waypoint(self, uav_id):
        navigation = self.return_navigation[uav_id]
        if not self._route_is_within_bounds(
            uav_id, navigation['path'], 'return or landing route'
        ):
            self._finish_return_failed(
                uav_id,
                'Return route contains a waypoint outside the configured '
                'operating region.',
            )
            return
        index = navigation['index']
        if index >= len(navigation['path']):
            navigation['phase'] = 'landing'
            navigation['landing_started_at'] = time.monotonic()
            self._publish_task_status(
                'LANDING', uav_id, UAVState.LANDING, 1.0,
                f"Reached '{navigation['target']['name']}' touchdown point; "
                'waiting for PX4 landed confirmation.',
            )
            state = self.mission_state.uav_states.get(uav_id)
            if state is not None:
                self._complete_landing_if_ready(uav_id, state)
            return
        if not self.waypoint_clients[uav_id].server_is_ready():
            self._finish_return_failed(
                uav_id, 'GoToWaypoint action server is unavailable.'
            )
            return
        goal = GoToWaypoint.Goal()
        goal.target_pose = navigation['path'][index]
        goal.acceptance_radius = self.acceptance_radius_m
        goal.max_velocity = self.max_velocity_mps
        future = self.waypoint_clients[uav_id].send_goal_async(goal)
        future.add_done_callback(
            partial(self._return_waypoint_goal_received, uav_id, index)
        )

    def _return_waypoint_goal_received(self, uav_id, index, future):
        navigation = self.return_navigation.get(uav_id)
        if navigation is None or navigation.get('index') != index:
            return
        try:
            goal_handle = future.result()
        except Exception as error:
            self._try_alternate_return_target(
                uav_id, f'Return waypoint failed: {error}'
            )
            return
        if not goal_handle.accepted:
            self._try_alternate_return_target(
                uav_id, 'UAV rejected a return waypoint.'
            )
            return
        self.return_navigation[uav_id]['goal_handle'] = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            partial(self._return_waypoint_result_received, uav_id, index)
        )

    def _return_waypoint_result_received(self, uav_id, index, future):
        navigation = self.return_navigation.get(uav_id)
        if navigation is None or navigation.get('index') != index:
            return
        try:
            wrapped_result = future.result()
        except Exception as error:
            self._try_alternate_return_target(
                uav_id, f'Return result failed: {error}'
            )
            return
        if (
            wrapped_result.status != GoalStatus.STATUS_SUCCEEDED
            or not wrapped_result.result.success
        ):
            self._try_alternate_return_target(
                uav_id,
                wrapped_result.result.message or 'UAV failed to return home.',
            )
            return
        navigation['index'] += 1
        self._send_next_return_waypoint(uav_id)

    def _complete_landing_if_ready(self, uav_id, state):
        navigation = self.return_navigation.get(uav_id)
        if (
            navigation is None
            or navigation.get('phase') != 'landing'
            or not bool(state.landed)
        ):
            return False
        self.return_navigation.pop(uav_id, None)
        self.pending_return_uavs.discard(uav_id)
        self.return_targets_tried.pop(uav_id, None)
        self.return_minimum_reserve_percentage.pop(uav_id, None)
        self.priority_returning_uavs.discard(uav_id)
        self.returned_uavs.add(uav_id)
        target = navigation['target']
        if target['recovery']:
            self.recovery_state.mark_arrived(uav_id)
            status = UAVState.RECOVERING
            task_id = 'LOW_BATTERY_RETURN'
            message = 'Landed at the UGV; awaiting recovery confirmation.'
        else:
            status = UAVState.LANDED
            task_id = 'RETURN_HOME'
            message = f"Landed safely at '{target['name']}'."
            self.successful_landed_uavs.add(uav_id)
        self._publish_task_status(task_id, uav_id, status, 1.0, message)
        self._report_metrics_if_complete()
        return True

    def _check_return_landings(self):
        now = time.monotonic()
        for uav_id, navigation in list(self.return_navigation.items()):
            if navigation.get('phase') != 'landing':
                continue
            state = self.mission_state.uav_states.get(uav_id)
            if state is not None and self._complete_landing_if_ready(
                uav_id, state
            ):
                continue
            if (
                now - navigation.get('landing_started_at', now)
                > self.landing_timeout_sec
            ):
                self._try_alternate_return_target(
                    uav_id,
                    'PX4 did not confirm landing at the selected destination.',
                )

    def _finish_return_failed(self, uav_id, message):
        self.navigation_failures += 1
        self.return_navigation.pop(uav_id, None)
        self.pending_return_uavs.discard(uav_id)
        self.return_minimum_reserve_percentage.pop(uav_id, None)
        self.priority_returning_uavs.discard(uav_id)
        self.returned_uavs.add(uav_id)
        self.return_failures.add(uav_id)
        self.return_failure_reasons[uav_id] = message
        self._publish_task_status(
            'EMERGENCY_RETURN', uav_id, UAVState.FAILED, 0.0,
            f'{message} Requesting PX4 emergency landing.',
        )
        self.get_logger().error(f"UAV '{uav_id}' return failed: {message}")
        self._report_metrics_if_complete()

    def _waypoint_feedback(self, task_id, uav_id, index, feedback_message):
        navigation = self.active_navigation.get(uav_id)
        if navigation is None or navigation['task'].task_id != task_id:
            return
        count = max(1, len(navigation['path']))
        progress = (index + feedback_message.feedback.progress) / count
        self._publish_task_status(
            task_id, uav_id, UAVState.EXECUTING, progress,
            'Following planned path.',
        )

    def _waypoint_goal_received(self, task_id, uav_id, index, future):
        try:
            goal_handle = future.result()
        except Exception as error:
            navigation = self.active_navigation.get(uav_id)
            if navigation is None or navigation['task'].task_id != task_id:
                return
            self._fail_task(task_id, uav_id, f'Waypoint goal failed: {error}')
            return
        if not goal_handle.accepted:
            navigation = self.active_navigation.get(uav_id)
            if navigation is None or navigation['task'].task_id != task_id:
                return
            self._fail_task(task_id, uav_id, 'UAV rejected a waypoint goal.')
            return
        navigation = self.active_navigation.get(uav_id)
        if navigation is None or navigation['task'].task_id != task_id:
            goal_handle.cancel_goal_async()
            return
        navigation['goal_handle'] = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            partial(self._waypoint_result_received, task_id, uav_id, index)
        )

    def _waypoint_result_received(self, task_id, uav_id, index, future):
        navigation = self.active_navigation.get(uav_id)
        if (
            navigation is None
            or navigation['task'].task_id != task_id
            or navigation['index'] != index
        ):
            return
        try:
            wrapped_result = future.result()
        except Exception as error:
            self._fail_task(
                task_id, uav_id, f'Waypoint result failed: {error}'
            )
            return
        if (
            wrapped_result.status != GoalStatus.STATUS_SUCCEEDED
            or not wrapped_result.result.success
        ):
            self._fail_task(
                task_id, uav_id, wrapped_result.result.message or
                'UAV failed to reach a waypoint.'
            )
            return
        odometry = self.odometry.get(uav_id)
        planned_position = navigation['last_planned_position']
        if odometry is not None and math.hypot(
            odometry.pose.pose.position.x - planned_position[0],
            odometry.pose.pose.position.y - planned_position[1],
        ) >= self.replan_distance_m:
            self.replan_count += 1
            navigation['replanning'] = True
            navigation['resume_index'] = index + 1
            self._request_task_path(navigation['task'], uav_id, odometry)
            return
        navigation['index'] += 1
        self._send_next_waypoint(task_id, uav_id)

    def _publish_task_status(
        self, task_id, uav_id, status, progress, message,
        publish_uav=True,
    ):
        update = TaskStatus()
        update.task_id = task_id
        update.uav_id = uav_id
        update.status = status
        update.progress = float(max(0.0, min(1.0, progress)))
        update.status_message = message
        if publish_uav:
            self.task_status_publishers[uav_id].publish(update)
        self.central_task_status_publisher.publish(update)

    def _fail_task(self, task_id, uav_id, message):
        self.navigation_failures += 1
        self.mission_failed_reasons[task_id] = str(message)
        retry_outcome = 'UNCHANGED'
        task_manager = self.task_manager
        if task_manager is not None:
            retry_outcome = task_manager.retry_navigation(
                task_id,
                uav_id,
                self.maximum_navigation_retries,
                message,
            )
        retrying = retry_outcome == 'RETRY'
        if retry_outcome == 'UNCHANGED':
            self.get_logger().warning(
                f"Task '{task_id}' could not be released for retry; its "
                'assignment or status changed before failure handling.'
            )
        retry_count = (
            task_manager.navigation_retries.get(task_id, 0)
            if task_manager is not None else 0
        )
        self._publish_task_status(
            task_id,
            uav_id,
            UAVState.IDLE if retrying else UAVState.FAILED,
            0.0,
            (
                f'{message} Task released for retry '
                f'({retry_count}/'
                f'{self.maximum_navigation_retries}).'
                if retrying else message
            ),
        )
        self.active_navigation.pop(uav_id, None)
        self.pending_navigation = [
            (task, assigned_uav)
            for task, assigned_uav in self.pending_navigation
            if task.task_id != task_id
        ]
        remaining = (
            task_manager.unfinished_for_uav(uav_id)
            if task_manager is not None else []
        )
        if remaining:
            self._publish_task_status(
                remaining[0],
                uav_id,
                UAVState.ASSIGNED,
                0.0,
                'Task failed; another committed task remains queued.',
            )
        else:
            if retrying:
                self._signal_allocation_event('task_retry')
            else:
                self._publish_task_status(
                    '', uav_id, UAVState.IDLE, 0.0,
                    'Vehicle available for another allocation.',
                )
                self._signal_allocation_event('task_failed')
                if self._all_tasks_terminal():
                    with self.task_assignment_lock:
                        self._begin_return_home_locked()
        self.get_logger().error(
            f"Task '{task_id}' failed for '{uav_id}': {message}"
        )

    def _record_planning_time(self, started_at):
        self.mission_planning_time_sec += max(
            0.0, time.perf_counter() - started_at
        )

    @staticmethod
    def _path_length(path):
        return sum(
            math.dist(
                (
                    first.pose.position.x,
                    first.pose.position.y,
                    first.pose.position.z,
                ),
                (
                    second.pose.position.x,
                    second.pose.position.y,
                    second.pose.position.z,
                ),
            )
            for first, second in zip(path, path[1:])
        )

    def _report_metrics_if_complete(self, finalize=False):
        if (
            self.metrics_reported
            or self.task_manager is None
            or not self._all_tasks_terminal()
            or self.mission_started_at is None
            or not self._all_uavs_return_terminal()
        ):
            return
        if set(self.uav_ids).issubset(self.successful_landed_uavs):
            if not self._all_uavs_confirmed_landed():
                return
            if not finalize:
                self._schedule_simulation_shutdown()
                return
        mission_time = (
            self._elapsed_sim_time_sec()
            if not self.return_failures else None
        )
        total_distance = sum(self.distance_by_uav.values())
        total_tasks = len(self.task_manager.tasks)
        completed_tasks = len(self.task_manager.completed)
        failed_tasks = len(self.task_manager.failed)
        success_rate = (
            completed_tasks / total_tasks * 100.0 if total_tasks else 0.0
        )
        if completed_tasks == total_tasks and not self.return_failures:
            completion_status = 'COMPLETED'
        elif completed_tasks:
            completion_status = 'PARTIALLY COMPLETED'
        else:
            completion_status = 'FAILED'
        if all(
            uav_id in self.initial_battery_percentage
            and uav_id in self.latest_battery_percentage
            for uav_id in self.uav_ids
        ):
            battery_consumed = sum(
                max(
                    0.0,
                    self.initial_battery_percentage[uav_id]
                    - self.latest_battery_percentage[uav_id],
                )
                for uav_id in self.uav_ids
            )
            measured_battery_metric = (
                f'{battery_consumed:.2f} percentage points (PX4 telemetry)'
            )
        else:
            measured_battery_metric = (
                'N/A (initial or final PX4 battery percentage unavailable)'
            )
        if all(
            uav_id in self.initial_estimated_battery_percentage
            and uav_id in self.latest_estimated_battery_percentage
            for uav_id in self.uav_ids
        ):
            estimated_consumed = sum(
                max(
                    0.0,
                    self.initial_estimated_battery_percentage[uav_id]
                    - self.latest_estimated_battery_percentage[uav_id],
                )
                for uav_id in self.uav_ids
            )
            estimated_battery_metric = (
                f'{estimated_consumed:.2f} percentage points '
                '(linear distance model)'
            )
        else:
            estimated_battery_metric = (
                'N/A (initial or final distance-based estimate unavailable)'
            )
        if all(
            self.energy_interval_counts[uav_id] > 0
            for uav_id in self.uav_ids
        ):
            energy_metric = (
                f'{sum(self.energy_consumed_wh.values()):.3f} Wh '
                '(PX4 voltage/current integration)'
            )
        else:
            energy_metric = 'N/A (PX4 voltage/current telemetry unavailable)'
        planning_time = (
            self.allocator_planning_time_sec
            + self.mission_planning_time_sec
        )
        self.metrics_reported = True
        self._save_mission_results(completion_status, mission_time)
        mission_time_text = (
            f'{mission_time:.1f} s' if mission_time is not None else 'N/A'
        )
        self.get_logger().info(
            '\n========================================\n'
            '       MULTI-UAV MISSION REPORT\n'
            '========================================\n'
            f'Mission completion status: {completion_status}\n'
            f'Total mission time: {mission_time_text}\n'
            f'Total tasks: {total_tasks}\n'
            f'Completed tasks: {completed_tasks}\n'
            f'Failed tasks: {failed_tasks}\n'
            f'Mission success rate: {success_rate:.1f}%\n'
            f'Total actual distance travelled: {total_distance:.2f} m\n'
            f'Total planned distance: {self.planned_distance_m:.2f} m\n'
            f'Total return distance: {self.return_distance_m:.2f} m\n'
            f'Total battery consumed: measured {measured_battery_metric}; '
            f'estimated {estimated_battery_metric}\n'
            f'Total energy consumed: {energy_metric}\n'
            f'Total travel time: {self.travel_time_sec:.1f} s '
            '(odometry velocity > 0.15 m/s)\n'
            f'Total inspection time: {self.inspection_time_sec:.1f} s '
            '(configured intervals actually completed)\n'
            f'Total path-planning computation time: {planning_time:.3f} s '
            '(allocator candidate paths plus mission and return paths)\n'
            f'Total replanning count: {self.replan_count}\n'
            'Total collisions: N/A (no collision telemetry configured)\n'
            '========================================'
        )

    def _save_mission_results(self, run_status, mission_time=None):
        if (
            self.results_saved
            or self.task_manager is None
            or self.mission_started_at is None
        ):
            return None
        if mission_time is None and run_status == 'INTERRUPTED':
            mission_time = self._elapsed_sim_time_sec()
        total_tasks = len(self.task_manager.tasks)
        completed = len(self.task_manager.completed)
        failed = len(self.task_manager.failed)
        cancelled = len(self.task_manager.cancelled)
        unfinished = max(0, total_tasks - completed - failed - cancelled)
        if run_status == 'INTERRUPTED':
            mission_status = 'INCOMPLETE'
        elif completed == total_tasks and not self.return_failures:
            mission_status = 'COMPLETED'
        elif completed:
            mission_status = 'PARTIALLY COMPLETED'
        else:
            mission_status = 'FAILED'

        task_rows = []
        for task_id, task in self.task_manager.tasks.items():
            assigned_uav = self.task_manager.assignments.get(task_id, '')
            task_rows.append({
                'task_id': task_id,
                'status': self.task_manager.statuses.get(task_id, 'UNKNOWN'),
                'assigned_uav': assigned_uav,
                'priority': int(task.priority),
                'estimated_duration_sec': float(task.estimated_duration),
                'x_m': float(task.target_position.x),
                'y_m': float(task.target_position.y),
                'z_m': float(task.target_position.z),
                'failure_reason': self.task_manager.failed.get(task_id, ''),
            })

        uav_results = {}
        for uav_id in self.uav_ids:
            position = self.last_positions.get(uav_id)
            uav_results[uav_id] = {
                'takeoff_position_m': (
                    list(self.takeoff_positions[uav_id])
                    if uav_id in self.takeoff_positions else None
                ),
                'distance_m': float(self.distance_by_uav.get(uav_id, 0.0)),
                'final_position_m': list(position) if position else None,
                'initial_battery_percent': self.initial_battery_percentage.get(
                    uav_id
                ),
                'final_battery_percent': self.latest_battery_percentage.get(
                    uav_id
                ),
                'initial_estimated_battery_percent': (
                    self.initial_estimated_battery_percentage.get(uav_id)
                ),
                'final_estimated_battery_percent': (
                    self.latest_estimated_battery_percentage.get(uav_id)
                ),
                'energy_consumed_wh': float(
                    self.energy_consumed_wh.get(uav_id, 0.0)
                ),
                'return_status': (
                    'RETURN_FAILED' if uav_id in self.return_failures
                    else 'LANDED' if uav_id in self.returned_uavs
                    else 'NOT_RETURNED'
                ),
                'return_failure_reason': self.return_failure_reasons.get(
                    uav_id, ''
                ),
            }

        payload = {
            'schema_version': 1,
            'run_status': run_status,
            'mission_status': mission_status,
            'started_at_utc': (
                self.mission_started_at_utc.isoformat()
                if self.mission_started_at_utc else None
            ),
            'saved_at_utc': datetime.now(timezone.utc).isoformat(),
            'mission_time_sec': (
                float(mission_time) if mission_time is not None else None
            ),
            'inspection_completion_time_sec': (
                max(
                    0.0,
                    self.inspection_completed_sim_time_sec
                    - self.mission_started_sim_time_sec,
                )
                if (
                    self.inspection_completed_sim_time_sec is not None
                    and self.mission_started_sim_time_sec is not None
                ) else None
            ),
            'task_counts': {
                'total': total_tasks,
                'completed': completed,
                'failed': failed,
                'cancelled': cancelled,
                'unfinished': unfinished,
            },
            'tasks': task_rows,
            'uavs': uav_results,
            'total_distance_m': float(sum(self.distance_by_uav.values())),
            'planned_distance_m': float(self.planned_distance_m),
            'return_distance_m': float(self.return_distance_m),
            'travel_time_sec': float(self.travel_time_sec),
            'inspection_time_sec': float(self.inspection_time_sec),
            'planning_time_sec': float(
                self.allocator_planning_time_sec
                + self.mission_planning_time_sec
            ),
            'replan_count': int(self.replan_count),
            'navigation_failures': int(self.navigation_failures),
            'mission_failed_reasons': dict(self.mission_failed_reasons),
        }
        try:
            result_path = write_mission_results(
                self.results_dir,
                payload,
                include_task_details=self.save_task_level_results,
            )
        except OSError as error:
            self.get_logger().error(
                f'Could not save mission results under {self.results_dir}: '
                f'{error}'
            )
            return None
        self.results_saved = True
        self.get_logger().info(f'Mission results saved to {result_path}')
        return result_path

    def _sim_time_sec(self):
        """Get the node clock in simulation seconds (use_sim_time launch)."""
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def _elapsed_sim_time_sec(self):
        if self.mission_started_sim_time_sec is None:
            return None
        elapsed = self._sim_time_sec() - self.mission_started_sim_time_sec
        return max(0.0, elapsed) if math.isfinite(elapsed) else None


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if not node.results_saved:
            node._save_mission_results(
                'COMPLETED' if node.metrics_reported else 'INTERRUPTED'
            )
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
