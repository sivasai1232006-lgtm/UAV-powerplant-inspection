
from functools import partial
import json
import math
from pathlib import Path
import time

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float64, String
from uav_interfaces.action import GoToWaypoint
from uav_interfaces.msg import UAVState
from uav_interfaces.msg import TaskStatus
from uav_interfaces.srv import AssignTask, GetUnassignedTasks

from uav_mission.mission_state import MissionState
from uav_mission.task_manager import TaskManager, tasks_from_mission


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


class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')
        self.mission_data = load_mission_config()
        uav_ids = [uav['id'] for uav in self.mission_data['mission']['uavs']]
        self.home_positions = {
            uav['id']: tuple(
                float(value) for value in uav['home_position']
            )
            for uav in self.mission_data['mission']['uavs']
        }
        self.tasks = None
        self.task_manager = None
        self.wait_reason = None
        self.uav_ids = uav_ids
        self.odometry = {}
        self.odometry_received_at = {}
        self.last_positions = {}
        self.distance_by_uav = {uav_id: 0.0 for uav_id in uav_ids}
        self.initial_battery_percentage = {}
        self.latest_battery_percentage = {}
        self.initial_estimated_battery_percentage = {}
        self.latest_estimated_battery_percentage = {}
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
        self.metrics_reported = False
        self.replan_count = 0
        self.navigation_failures = 0
        self.active_navigation = {}
        self.pending_navigation = []
        self.last_task_altitude = {}
        self.home_return_started = False
        self.pending_return_uavs = set()
        self.return_navigation = {}
        self.returned_uavs = set()
        self.return_failures = set()

        self.declare_parameter(
            'uav_state_topic_template', '/{uav_id}/uav_state'
        )
        self.declare_parameter('state_timeout_sec', 3.0)
        self.declare_parameter('navigation.max_velocity_mps', 2.0)
        self.declare_parameter('navigation.acceptance_radius_m', 0.3)
        self.declare_parameter('navigation.replan_distance_m', 5.0)
        topic_template = self.get_parameter(
            'uav_state_topic_template'
        ).value
        state_timeout_sec = float(
            self.get_parameter('state_timeout_sec').value
        )
        self.max_velocity_mps = float(
            self.get_parameter('navigation.max_velocity_mps').value
        )
        self.acceptance_radius_m = float(
            self.get_parameter('navigation.acceptance_radius_m').value
        )
        self.replan_distance_m = float(
            self.get_parameter('navigation.replan_distance_m').value
        )
        self.mission_state = MissionState(uav_ids, state_timeout_sec)
        self.planner_clients = {
            altitude: ActionClient(
                self,
                ComputePathToPose,
                f'/planner_{altitude}m/compute_path_to_pose',
            )
            for altitude in (8, 10)
        }
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
        self.assign_task_service = self.create_service(
            AssignTask,
            '/mission_manager/assign_task',
            self.assign_task_callback,
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
        self.timer = self.create_timer(1.0, self.prepare_tasks_when_ready)
        self.navigation_timer = self.create_timer(
            0.5, self.start_pending_navigation
        )
        self.return_timer = self.create_timer(
            0.5, self.start_pending_returns
        )

        self.get_logger().info(
            f"Loaded mission '{self.mission_data['mission']['name']}'. "
            f'Waiting for UAV status on {", ".join(state_topics)}.'
        )

    def uav_state_callback(self, message, expected_uav_id):
        if message.uav_id != expected_uav_id:
            self.get_logger().warning(
                f"Ignoring UAV '{message.uav_id}' status on the "
                f"'{expected_uav_id}' topic."
            )
            return

        self.mission_state.update(message)

    def odometry_callback(self, message, uav_id):
        if message.header.frame_id != self.mission_data['mission']['frame_id']:
            return
        position = (
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
            float(message.pose.pose.position.z),
        )
        previous = self.last_positions.get(uav_id)
        now = time.monotonic()
        if previous is not None and self.mission_started_at is not None:
            self.distance_by_uav[uav_id] += math.dist(previous, position)
            elapsed = now - self.last_odometry_time.get(uav_id, now)
            velocity = message.twist.twist.linear
            speed = math.sqrt(
                float(velocity.x) ** 2
                + float(velocity.y) ** 2
                + float(velocity.z) ** 2
            )
            if speed > 0.15:
                self.travel_time_sec += min(max(elapsed, 0.0), 1.0)
            update = Float64()
            update.data = self.distance_by_uav[uav_id]
            self.path_distance_publishers[uav_id].publish(update)
        self.last_positions[uav_id] = position
        self.odometry[uav_id] = message
        self.last_odometry_time[uav_id] = now
        self.odometry_received_at[uav_id] = now

    def battery_callback(self, message, uav_id):
        if (
            not message.present
            or not math.isfinite(message.percentage)
            or not 0.0 <= message.percentage <= 1.0
        ):
            return
        percentage = float(message.percentage) * 100.0
        self.latest_battery_percentage[uav_id] = percentage
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
            self.task_manager.completed | set(self.task_manager.failed)
        ) == set(self.task_manager.tasks)

    def prepare_tasks_when_ready(self):
        if self.tasks is not None:
            return

        now = time.monotonic()
        if not self.mission_state.operational_uavs():
            wait_reason = 'No connected, operational UAV status is available.'
        elif not self.mission_state.available_uavs():
            wait_reason = (
                'Operational UAVs are busy; waiting for one to be ready.'
            )
        elif not all(
            uav_id in self.latest_battery_percentage
            and uav_id in self.odometry
            and now - self.odometry_received_at.get(uav_id, 0.0) <= 3.0
            for uav_id in self.uav_ids
        ):
            wait_reason = 'Waiting for fresh odometry and battery telemetry from both UAVs.'
        else:
            self.tasks = tasks_from_mission(self.mission_data)
            self.task_manager = TaskManager(self.tasks)
            self.mission_started_at = time.monotonic()
            self.energy_consumed_wh = {
                uav_id: 0.0 for uav_id in self.uav_ids
            }
            self.energy_interval_counts = {
                uav_id: 0 for uav_id in self.uav_ids
            }
            self.energy_samples.clear()
            self.initial_battery_percentage = dict(
                self.latest_battery_percentage
            )
            self.initial_estimated_battery_percentage = dict(
                self.latest_estimated_battery_percentage
            )
            ready_ids = [
                state.uav_id for state in self.mission_state.available_uavs()
            ]
            self.get_logger().info(
                f'Prepared {len(self.tasks)} tasks for task management. '
                f'Ready UAVs: {", ".join(ready_ids)}.'
            )
            self.wait_reason = None
            return

        if wait_reason != self.wait_reason:
            self.get_logger().info(wait_reason)
            self.wait_reason = wait_reason

    def get_unassigned_tasks_callback(self, request, response):
        if self.task_manager is not None:
            response.tasks = self.task_manager.unassigned_tasks()
        return response

    def assign_task_callback(self, request, response):
        if self.task_manager is None:
            response.success = False
            response.message = 'Mission tasks are not ready for allocation.'
            return response

        available_uav_ids = {
            state.uav_id for state in self.mission_state.available_uavs()
        }
        if request.uav_id not in available_uav_ids:
            response.success = False
            response.message = (
                f"UAV '{request.uav_id}' is not currently available."
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
            self.last_task_altitude[request.uav_id] = int(
                round(task.target_position.z)
            )
            self._publish_task_status(
                task.task_id, request.uav_id, UAVState.ASSIGNED, 0.0,
                'Task assignment confirmed.',
            )
            self.pending_navigation.append((task, request.uav_id))
            self.get_logger().info(response.message)
        return response

    def start_pending_navigation(self):
        if self.task_manager is None:
            return
        now = time.monotonic()
        for uav_id, navigation in list(self.active_navigation.items()):
            if (
                navigation.get('phase') == 'inspection'
                and now >= navigation['inspection_end']
            ):
                task = navigation['task']
                task_id = task.task_id
                self.task_manager.complete(task_id, uav_id)
                self.inspection_time_sec += float(task.estimated_duration)
                self._publish_task_status(
                    task_id, uav_id, UAVState.COMPLETED, 1.0,
                    'Inspection duration completed.',
                )
                del self.active_navigation[uav_id]
                if self._all_tasks_terminal():
                    self._begin_return_home()
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
            altitude = int(round(task.target_position.z))
            planner = self.planner_clients.get(altitude)
            if planner is None:
                self._fail_task(
                    task.task_id, uav_id,
                    'No planner map is configured at '
                    f'{task.target_position.z:g} m.',
                )
                continue
            if not planner.server_is_ready():
                pending.append((task, uav_id))
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
            self._request_task_path(task, uav_id, odometry, altitude)
        self.pending_navigation = pending

    def _request_task_path(self, task, uav_id, odometry, altitude):
        request = ComputePathToPose.Goal()
        request.start = PoseStamped()
        request.start.header.frame_id = odometry.header.frame_id
        request.start.header.stamp = self.get_clock().now().to_msg()
        request.start.pose = odometry.pose.pose
        request.start.pose.position.z = float(altitude)
        request.goal = PoseStamped()
        request.goal.header.frame_id = odometry.header.frame_id
        request.goal.header.stamp = self.get_clock().now().to_msg()
        request.goal.pose.position.x = float(task.target_position.x)
        request.goal.pose.position.y = float(task.target_position.y)
        request.goal.pose.position.z = float(altitude)
        request.goal.pose.orientation.w = 1.0
        request.use_start = True
        request.planner_id = 'GridBased'
        started_at = time.perf_counter()
        future = self.planner_clients[altitude].send_goal_async(request)
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
        task = self.task_manager.tasks[task_id]
        route = []
        for pose in path.poses:
            waypoint = PoseStamped()
            waypoint.header = pose.header
            waypoint.pose = pose.pose
            waypoint.pose.position.z = float(task.target_position.z)
            route.append(waypoint)
        self.active_navigation[uav_id]['path'] = route
        start_position = self.odometry[uav_id].pose.pose.position
        self.planned_distance_m += (
            abs(float(task.target_position.z) - float(start_position.z))
            + self._path_length(route)
        )
        trajectory = NavPath()
        trajectory.header = path.header
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
        self._fail_task(task_id, uav_id, message)

    def _send_next_waypoint(self, task_id, uav_id):
        navigation = self.active_navigation[uav_id]
        index = navigation['index']
        route = navigation['path']
        if index >= len(route):
            navigation['phase'] = 'inspection'
            duration = float(navigation['task'].estimated_duration)
            navigation['inspection_end'] = time.monotonic() + duration
            self._publish_task_status(
                task_id, uav_id, UAVState.EXECUTING, 0.99,
                'Holding position for inspection.',
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
        if self.home_return_started:
            return
        self.home_return_started = True
        self.pending_return_uavs = set(self.last_task_altitude)
        if not self.pending_return_uavs:
            self._report_metrics_if_complete()

    def start_pending_returns(self):
        if (
            not self.home_return_started
            or not self.pending_return_uavs
        ):
            return
        now = time.monotonic()
        for uav_id in list(self.pending_return_uavs):
            odometry = self.odometry.get(uav_id)
            if (
                odometry is None
                or now - self.odometry_received_at.get(uav_id, 0.0) > 3.0
            ):
                continue
            altitude = self.last_task_altitude[uav_id]
            planner = self.planner_clients.get(altitude)
            if planner is None or not planner.server_is_ready():
                continue
            self.pending_return_uavs.remove(uav_id)
            self.return_navigation[uav_id] = {'planning': True}
            request = ComputePathToPose.Goal()
            request.start = PoseStamped()
            request.start.header.frame_id = odometry.header.frame_id
            request.start.header.stamp = self.get_clock().now().to_msg()
            request.start.pose = odometry.pose.pose
            request.start.pose.position.z = float(altitude)
            request.goal = PoseStamped()
            request.goal.header.frame_id = odometry.header.frame_id
            request.goal.header.stamp = self.get_clock().now().to_msg()
            home = self.home_positions[uav_id]
            request.goal.pose.position.x = home[0]
            request.goal.pose.position.y = home[1]
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
                'Planning return to the home staging point.',
            )

    def _return_path_goal_received(
        self, uav_id, altitude, started_at, future
    ):
        try:
            goal_handle = future.result()
        except Exception as error:
            self._record_planning_time(started_at)
            self._finish_return_failed(
                uav_id, f'Return planning failed: {error}'
            )
            return
        if not goal_handle.accepted:
            self._record_planning_time(started_at)
            self._finish_return_failed(
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
        self._record_planning_time(started_at)
        try:
            wrapped_result = future.result()
        except Exception as error:
            self._finish_return_failed(uav_id, f'Return path failed: {error}')
            return
        path = wrapped_result.result.path
        if (
            wrapped_result.status != GoalStatus.STATUS_SUCCEEDED
            or not path.poses
        ):
            self._finish_return_failed(
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
        home = self.home_positions[uav_id]
        final_pose = PoseStamped()
        final_pose.header = path.header
        final_pose.pose.position.x = home[0]
        final_pose.pose.position.y = home[1]
        final_pose.pose.position.z = home[2]
        final_pose.pose.orientation.w = 1.0
        route.append(final_pose)
        self.return_distance_m += self._path_length(route)
        self.planned_distance_m += self._path_length(route)
        self.return_navigation[uav_id] = {'path': route, 'index': 0}
        trajectory = NavPath()
        trajectory.header = path.header
        trajectory.poses = route
        self.trajectory_publishers[uav_id].publish(trajectory)
        self._send_next_return_waypoint(uav_id)

    def _send_next_return_waypoint(self, uav_id):
        navigation = self.return_navigation[uav_id]
        index = navigation['index']
        if index >= len(navigation['path']):
            del self.return_navigation[uav_id]
            self.returned_uavs.add(uav_id)
            self._publish_task_status(
                '', uav_id, UAVState.COMPLETED, 1.0,
                'Returned to the home staging point.',
            )
            self._report_metrics_if_complete()
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
        try:
            goal_handle = future.result()
        except Exception as error:
            self._finish_return_failed(
                uav_id, f'Return waypoint failed: {error}'
            )
            return
        if not goal_handle.accepted:
            self._finish_return_failed(
                uav_id, 'UAV rejected a return waypoint.'
            )
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            partial(self._return_waypoint_result_received, uav_id, index)
        )

    def _return_waypoint_result_received(self, uav_id, index, future):
        try:
            wrapped_result = future.result()
        except Exception as error:
            self._finish_return_failed(
                uav_id, f'Return result failed: {error}'
            )
            return
        if (
            wrapped_result.status != GoalStatus.STATUS_SUCCEEDED
            or not wrapped_result.result.success
        ):
            self._finish_return_failed(
                uav_id,
                wrapped_result.result.message or 'UAV failed to return home.',
            )
            return
        navigation = self.return_navigation.get(uav_id)
        if navigation is None or navigation['index'] != index:
            return
        navigation['index'] += 1
        self._send_next_return_waypoint(uav_id)

    def _finish_return_failed(self, uav_id, message):
        self.navigation_failures += 1
        self.return_navigation.pop(uav_id, None)
        self.returned_uavs.add(uav_id)
        self.return_failures.add(uav_id)
        self._publish_task_status(
            'RETURN_HOME', uav_id, UAVState.FAILED, 0.0, message
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
            self._fail_task(task_id, uav_id, f'Waypoint goal failed: {error}')
            return
        if not goal_handle.accepted:
            self._fail_task(task_id, uav_id, 'UAV rejected a waypoint goal.')
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            partial(self._waypoint_result_received, task_id, uav_id, index)
        )

    def _waypoint_result_received(self, task_id, uav_id, index, future):
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
        navigation = self.active_navigation.get(uav_id)
        if navigation is None or navigation['index'] != index:
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
            self._request_task_path(
                navigation['task'], uav_id, odometry,
                navigation['altitude'],
            )
            return
        navigation['index'] += 1
        self._send_next_waypoint(task_id, uav_id)

    def _publish_task_status(self, task_id, uav_id, status, progress, message):
        update = TaskStatus()
        update.task_id = task_id
        update.uav_id = uav_id
        update.status = status
        update.progress = float(max(0.0, min(1.0, progress)))
        update.status_message = message
        self.task_status_publishers[uav_id].publish(update)

    def _fail_task(self, task_id, uav_id, message):
        self.navigation_failures += 1
        if self.task_manager is not None:
            self.task_manager.release(task_id, uav_id)
        self._publish_task_status(
            task_id, uav_id, UAVState.FAILED, 0.0, message
        )
        self._publish_task_status(
            task_id, uav_id, UAVState.IDLE, 0.0,
            'Vehicle available for another allocation.',
        )
        self.active_navigation.pop(uav_id, None)
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

    def _report_metrics_if_complete(self):
        if (
            self.metrics_reported
            or self.task_manager is None
            or not self._all_tasks_terminal()
            or self.mission_started_at is None
            or not set(self.last_task_altitude).issubset(self.returned_uavs)
        ):
            return
        mission_time = time.monotonic() - self.mission_started_at
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
        self.get_logger().info(
            '\n========================================\n'
            '       MULTI-UAV MISSION REPORT\n'
            '========================================\n'
            f'Mission completion status: {completion_status}\n'
            f'Total mission time: {mission_time:.1f} s\n'
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


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
