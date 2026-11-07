import math
import json
from dataclasses import dataclass, field
from functools import partial
import time
from typing import Any, Literal, cast

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float64, String
from uav_interfaces.msg import UAVState
from uav_interfaces.srv import AssignTask, GetUnassignedTasks

from uav_allocator.battery_feasibility import BatteryFeasibility
from uav_allocator.cost_function import CostFunction
from uav_allocator.path_cost_client import PathCostClient
from uav_allocator.workload_estimator import WorkloadEstimator


Position = tuple[float, float, float]


@dataclass(frozen=True)
class InspectionTask:
    task_id: str
    target_position: Position
    priority: int
    estimated_duration_sec: float


@dataclass(frozen=True)
class UavSnapshot:
    uav_id: str
    position: Position
    battery_percentage: float
    connected: bool
    operational: bool
    mission_status: int
    state_age_sec: float
    yaw: float = 0.0


@dataclass(frozen=True)
class AllocationDecision:
    task_id: str
    uav_id: str | None
    bid: float | None
    reason: str
    outbound_distance_m: float | None = None
    return_distance_m: float | None = None


@dataclass
class UavTelemetry:
    state: UAVState | None = None
    odometry: Odometry | None = None
    battery: BatteryState | None = None
    received_at: dict[str, float] = field(default_factory=dict)


UavTelemetryKey = Literal['state', 'odometry', 'battery']
TelemetryMessage = UAVState | Odometry | BatteryState


class AuctionAllocator:
    def __init__(
        self,
        path_cost,
        workload_estimator,
        battery_feasibility,
        cost_function,
        state_timeout_sec,
    ):
        if state_timeout_sec <= 0.0:
            raise ValueError('State timeout must be greater than zero')
        self.path_cost = path_cost
        self.workload_estimator = workload_estimator
        self.battery_feasibility = battery_feasibility
        self.cost_function = cost_function
        self.state_timeout_sec = float(state_timeout_sec)

    def allocate(
        self,
        tasks,
        uavs,
        base_positions,
        already_assigned_task_ids=(),
    ):
        tasks = list(tasks)
        uavs = list(uavs)
        assigned_task_ids = set(already_assigned_task_ids)
        reserved_uav_ids = set()
        decisions = []

        for task in tasks:
            if not task.task_id or task.task_id in assigned_task_ids:
                decisions.append(AllocationDecision(
                    task.task_id,
                    None,
                    None,
                    'task is invalid or already assigned',
                ))
                continue
            if not self._valid_task(task):
                decisions.append(AllocationDecision(
                    task.task_id, None, None, 'task data is invalid'
                ))
                continue

            candidates = []
            rejection_reasons = []
            for uav in uavs:
                if uav.uav_id in reserved_uav_ids:
                    rejection_reasons.append(
                        f'{uav.uav_id}: reserved for another task'
                    )
                    continue
                eligible, reason = self._eligible(uav)
                if not eligible:
                    rejection_reasons.append(f'{uav.uav_id}: {reason}')
                    continue

                home = base_positions.get(uav.uav_id)
                if home is None or not self._valid_position(home):
                    rejection_reasons.append(
                        f'{uav.uav_id}: home position is unavailable'
                    )
                    continue

                outbound = self.path_cost.estimate_distance(
                    uav.position, task.target_position, uav.yaw, 0.0
                )
                return_distance = self.path_cost.estimate_distance(
                    task.target_position, home, 0.0, 0.0
                )
                if outbound is None or return_distance is None:
                    rejection_reasons.append(
                        f'{uav.uav_id}: path distance is unavailable'
                    )
                    continue

                workload = self.workload_estimator.estimate(
                    outbound,
                    task.estimated_duration_sec,
                    return_distance,
                )
                feasibility = self.battery_feasibility.evaluate(
                    uav.battery_percentage,
                    outbound,
                    return_distance,
                    task.estimated_duration_sec,
                )
                if not feasibility.feasible:
                    rejection_reasons.append(
                        f'{uav.uav_id}: {feasibility.reason}'
                    )
                    continue

                distance_energy = (
                    outbound * self.battery_feasibility.consumption_per_meter
                )
                inspection_rate = (
                    self.battery_feasibility.inspection_consumption_per_minute
                )
                inspection_energy = (
                    task.estimated_duration_sec / 60.0
                    * inspection_rate
                )
                return_energy = (
                    return_distance
                    * self.battery_feasibility.consumption_per_meter
                )
                estimated_energy = (
                    distance_energy + return_energy + inspection_energy
                )
                bid = self.cost_function.calculate(
                    outbound,
                    workload.total_task_sec + workload.return_travel_sec,
                    estimated_energy,
                    task.priority,
                )
                candidates.append((
                    bid,
                    uav.uav_id,
                    outbound,
                    return_distance,
                ))

            if not candidates:
                reason = '; '.join(rejection_reasons) or 'no UAVs available'
                decisions.append(AllocationDecision(
                    task.task_id, None, None, reason
                ))
                continue

            bid, selected_uav_id, outbound, return_distance = min(
                candidates, key=lambda candidate: (candidate[0], candidate[1])
            )
            assigned_task_ids.add(task.task_id)
            reserved_uav_ids.add(selected_uav_id)
            decisions.append(AllocationDecision(
                task.task_id,
                selected_uav_id,
                bid,
                'lowest feasible bid',
                outbound,
                return_distance,
            ))

        return decisions

    def _eligible(self, uav):
        if not uav.uav_id:
            return False, 'UAV id is empty'
        if not uav.connected:
            return False, 'UAV is disconnected'
        if not uav.operational:
            return False, 'UAV is unhealthy or non-operational'
        if uav.mission_status not in (UAVState.IDLE, UAVState.COMPLETED):
            return False, 'UAV is not ready for another task'
        if (
            not math.isfinite(uav.state_age_sec)
            or uav.state_age_sec < 0.0
            or uav.state_age_sec > self.state_timeout_sec
        ):
            return False, 'UAV state is stale or invalid'
        if not self._valid_position(uav.position):
            return False, 'UAV position is invalid'
        if (
            not math.isfinite(uav.battery_percentage)
            or not 0.0 <= uav.battery_percentage <= 100.0
        ):
            return False, 'battery percentage is invalid'
        return True, ''

    @staticmethod
    def _valid_position(position):
        try:
            return (
                len(position) == 3
                and all(math.isfinite(value) for value in position)
            )
        except (TypeError, ValueError):
            return False

    @classmethod
    def _valid_task(cls, task):
        return (
            cls._valid_position(task.target_position)
            and task.priority > 0
            and math.isfinite(task.estimated_duration_sec)
            and task.estimated_duration_sec >= 0.0
        )


def allocator_from_config(config, allow_straight_line=True):
    if 'allocator' in config:
        config = config['allocator']['ros__parameters']
    cost_config = config['cost']
    return AuctionAllocator(
        path_cost=PathCostClient(allow_straight_line),
        workload_estimator=WorkloadEstimator(
            config['nominal_speed_mps']
        ),
        battery_feasibility=BatteryFeasibility(
            config['battery_consumption_per_meter'],
            config['inspection_consumption_per_minute'],
            config['minimum_battery_reserve_percentage'],
        ),
        cost_function=CostFunction(
            cost_config['weights'],
            cost_config['distance_scale_m'],
            cost_config['time_scale_sec'],
            cost_config['energy_scale_percentage'],
            cost_config['minimum_priority'],
            cost_config['maximum_priority'],
        ),
        state_timeout_sec=config['state_timeout_sec'],
    )


class AuctionAllocatorNode(Node):
    def __init__(self):
        super().__init__('auction_allocator')
        self._declare_parameters()
        self.uav_ids: list[str] = [
            str(uav_id) for uav_id in self._parameter('uav_ids')
        ]
        self.base_positions: dict[str, Position] = {}
        for uav_id in self.uav_ids:
            position = self._parameter(f'base_positions.{uav_id}')
            if len(position) != 3:
                raise ValueError(
                    f"Base position for '{uav_id}' must have 3 values"
                )
            self.base_positions[uav_id] = (
                float(position[0]),
                float(position[1]),
                float(position[2]),
            )
        self.allocator = allocator_from_config(
            self._allocator_config(), allow_straight_line=False
        )
        self.path_action_clients = {
            altitude: ActionClient(
                self,
                ComputePathToPose,
                f'/planner_{altitude}m/compute_path_to_pose',
            )
            for altitude in (8, 10)
        }
        self.planning_frame = str(self._parameter('planning_frame'))
        self.state_timeout_sec = float(
            self._parameter('state_timeout_sec')
        )
        self.uav_data: dict[str, UavTelemetry] = {
            uav_id: UavTelemetry()
            for uav_id in self.uav_ids
        }
        self.get_tasks_client = self.create_client(
            GetUnassignedTasks,
            str(self._parameter('get_unassigned_tasks_service')),
        )
        self.assign_task_client = self.create_client(
            AssignTask,
            str(self._parameter('assign_task_service')),
        )
        self._create_uav_subscriptions()
        self._allocation_active = False
        self._assignment_decisions: list[AllocationDecision] = []
        self._assignment_index = 0
        self._next_cycle_time = 0.0
        self._path_planning_time_sec = 0.0
        self._path_request_started_at = None
        self._unallocatable_cycles: dict[str, int] = {}
        self.path_planning_time_publisher = self.create_publisher(
            Float64, '/auction_allocator/path_planning_time', 10
        )
        self.unallocatable_tasks_publisher = self.create_publisher(
            String, '/mission_manager/unallocatable_tasks', 10
        )
        self.timer = self.create_timer(
            float(self._parameter('auction_retry_interval_sec')),
            self._start_allocation_cycle,
        )

    def _declare_parameters(self):
        defaults = {
            'uav_ids': ['uav_1', 'uav_2'],
            'topics.uav_state': '/{uav_id}/uav_state',
            'topics.odometry': '/{uav_id}/odom',
            'topics.battery': '/{uav_id}/battery/measured',
            'get_unassigned_tasks_service': (
                '/mission_manager/get_unassigned_tasks'
            ),
            'assign_task_service': '/mission_manager/assign_task',
            'planning_frame': 'map',
            'nominal_speed_mps': 3.0,
            'state_timeout_sec': 3.0,
            'auction_retry_interval_sec': 1.0,
            'unallocatable_cycles_before_failure': 5,
            'battery_consumption_per_meter': 0.05,
            'inspection_consumption_per_minute': 0.25,
            'minimum_battery_reserve_percentage': 20.0,
            'cost.weights.distance': 0.4,
            'cost.weights.time': 0.25,
            'cost.weights.energy': 0.25,
            'cost.weights.priority': 0.1,
            'cost.distance_scale_m': 100.0,
            'cost.time_scale_sec': 300.0,
            'cost.energy_scale_percentage': 10.0,
            'cost.minimum_priority': 1,
            'cost.maximum_priority': 5,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        for uav_id, position in (
            ('uav_1', [-5.0, 0.0, 0.0]),
            ('uav_2', [5.0, 0.0, 0.0]),
        ):
            self.declare_parameter(f'base_positions.{uav_id}', position)

    def _parameter(self, name: str) -> Any:
        return self.get_parameter(name).value

    def _allocator_config(self):
        return {
            'nominal_speed_mps': float(
                self._parameter('nominal_speed_mps')
            ),
            'state_timeout_sec': float(
                self._parameter('state_timeout_sec')
            ),
            'battery_consumption_per_meter': float(
                self._parameter('battery_consumption_per_meter')
            ),
            'inspection_consumption_per_minute': float(
                self._parameter('inspection_consumption_per_minute')
            ),
            'minimum_battery_reserve_percentage': float(
                self._parameter('minimum_battery_reserve_percentage')
            ),
            'cost': {
                'weights': {
                    name: float(self._parameter(f'cost.weights.{name}'))
                    for name in ('distance', 'time', 'energy', 'priority')
                },
                'distance_scale_m': float(
                    self._parameter('cost.distance_scale_m')
                ),
                'time_scale_sec': float(
                    self._parameter('cost.time_scale_sec')
                ),
                'energy_scale_percentage': float(
                    self._parameter('cost.energy_scale_percentage')
                ),
                'minimum_priority': int(
                    self._parameter('cost.minimum_priority')
                ),
                'maximum_priority': int(
                    self._parameter('cost.maximum_priority')
                ),
            },
        }

    def _create_uav_subscriptions(self):
        self.uav_subscriptions = []
        for uav_id in self.uav_ids:
            topics: tuple[tuple[str, type[Any], UavTelemetryKey], ...] = (
                ('topics.uav_state', UAVState, 'state'),
                ('topics.odometry', Odometry, 'odometry'),
                ('topics.battery', BatteryState, 'battery'),
            )
            for parameter, message_type, key in topics:
                topic = str(self._parameter(parameter)).format(
                    uav_id=uav_id
                )
                self.uav_subscriptions.append(
                    self.create_subscription(
                        message_type,
                        topic,
                        partial(self._uav_data_callback, uav_id, key),
                        10,
                    )
                )

    def _uav_data_callback(
        self,
        uav_id: str,
        key: UavTelemetryKey,
        message: TelemetryMessage,
    ) -> None:
        data = self.uav_data[uav_id]
        setattr(data, key, message)
        data.received_at[key] = time.monotonic()

    def _start_allocation_cycle(self):
        if self._allocation_active or time.monotonic() < self._next_cycle_time:
            return
        if not self.get_tasks_client.service_is_ready():
            self.get_logger().warning(
                'Waiting for task manager GetUnassignedTasks service.'
            )
            return

        self._allocation_active = True
        future = self.get_tasks_client.call_async(
            GetUnassignedTasks.Request()
        )
        future.add_done_callback(self._unassigned_tasks_received)

    def _unassigned_tasks_received(
        self, future: Future
    ) -> None:
        try:
            result = future.result()
        except Exception as error:
            self.get_logger().error(
                f'Failed to get unassigned tasks: {error}'
            )
            self._finish_allocation_cycle()
            return
        if result is None:
            self.get_logger().error(
                'Task manager returned an empty GetUnassignedTasks response.'
            )
            self._finish_allocation_cycle()
            return
        response = cast(GetUnassignedTasks.Response, result)

        if not response.tasks:
            self._finish_allocation_cycle()
            return
        tasks = [
            InspectionTask(
                task_id=str(task.task_id),
                target_position=(
                    float(task.target_position.x),
                    float(task.target_position.y),
                    float(task.target_position.z),
                ),
                priority=int(task.priority),
                estimated_duration_sec=float(task.estimated_duration),
            )
            for task in response.tasks
        ]
        now = time.monotonic()
        self._cycle_tasks = tasks
        self._cycle_uavs = self._current_uav_snapshots(now)
        self.allocator.path_cost._distances.clear()
        self._path_queries = []
        unique_queries = set()
        for uav in self._cycle_uavs:
            home = self.base_positions.get(uav.uav_id)
            if home is None:
                continue
            for task in tasks:
                for start, goal in (
                    (uav.position, task.target_position),
                    (task.target_position, home),
                ):
                    altitude = int(round(task.target_position[2]))
                    if altitude not in self.path_action_clients:
                        continue
                    start_yaw = uav.yaw if start == uav.position else 0.0
                    goal_yaw = 0.0
                    key = (
                        PathCostClient._key(
                            start, goal, start_yaw, goal_yaw
                        ),
                        altitude,
                    )
                    if key not in unique_queries:
                        unique_queries.add(key)
                        self._path_queries.append((
                            start, goal, altitude, start_yaw, goal_yaw
                        ))
        self._path_query_index = 0
        self._request_next_path()

    def _request_next_path(self):
        if self._path_query_index >= len(self._path_queries):
            self._allocate_from_path_results()
            return
        start, goal, altitude, start_yaw, goal_yaw = self._path_queries[
            self._path_query_index
        ]
        action_client = self.path_action_clients[altitude]
        if not action_client.server_is_ready():
            self.get_logger().warning(
                f'Waiting for the {altitude} m Nav2 planner action server.'
            )
            self._finish_allocation_cycle()
            return
        request = ComputePathToPose.Goal()
        request.start = self._planning_pose(start, altitude, start_yaw)
        request.goal = self._planning_pose(goal, altitude, goal_yaw)
        request.use_start = True
        request.planner_id = 'GridBased'
        self._path_request_started_at = time.perf_counter()
        future = action_client.send_goal_async(request)
        future.add_done_callback(
            partial(
                self._path_goal_received, start, goal, altitude,
                start_yaw, goal_yaw,
            )
        )

    def _planning_pose(self, position, altitude, yaw):
        pose = PoseStamped()
        pose.header.frame_id = self.planning_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(position[0])
        pose.pose.position.y = float(position[1])
        pose.pose.position.z = float(altitude)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _path_goal_received(
        self, start, goal, altitude, start_yaw, goal_yaw, future
    ):
        try:
            goal_handle = future.result()
        except Exception as error:
            self.get_logger().warning(f'Path request failed: {error}')
            self._record_path_planning_time()
            self.allocator.path_cost.set_distance(
                start, goal, None, start_yaw, goal_yaw
            )
            self._path_query_index += 1
            self._request_next_path()
            return
        if not goal_handle.accepted:
            self._record_path_planning_time()
            self.allocator.path_cost.set_distance(
                start, goal, None, start_yaw, goal_yaw
            )
            self._path_query_index += 1
            self._request_next_path()
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            partial(
                self._path_result_received, start, goal, altitude,
                start_yaw, goal_yaw,
            )
        )

    def _path_result_received(
        self, start, goal, altitude, start_yaw, goal_yaw, future
    ):
        distance = None
        try:
            wrapped_result = future.result()
            if wrapped_result.status == GoalStatus.STATUS_SUCCEEDED:
                path = wrapped_result.result.path
                if path.poses:
                    points = [
                        (
                            pose.pose.position.x,
                            pose.pose.position.y,
                            pose.pose.position.z,
                        )
                        for pose in path.poses
                    ]
                    distance = sum(
                        math.hypot(
                            second[0] - first[0],
                            second[1] - first[1],
                        )
                        for first, second in zip(points, points[1:])
                    )
                    if len(points) == 1:
                        distance = math.hypot(
                            goal[0] - start[0], goal[1] - start[1]
                        )
                    distance += (
                        abs(start[2] - altitude)
                        + abs(goal[2] - altitude)
                    )
        except Exception as error:
            self.get_logger().warning(f'Path result failed: {error}')
        self._record_path_planning_time()
        self.allocator.path_cost.set_distance(
            start, goal, distance, start_yaw, goal_yaw
        )
        self._path_query_index += 1
        self._request_next_path()

    def _allocate_from_path_results(self):
        self._assignment_decisions = self.allocator.allocate(
            self._cycle_tasks,
            self._cycle_uavs,
            self.base_positions,
        )
        self._report_unallocatable_tasks()
        for decision in self._assignment_decisions:
            if decision.uav_id is None:
                self.get_logger().warning(
                    f"Task '{decision.task_id}' remains unassigned: "
                    f'{decision.reason}'
                )
        self._assignment_decisions = [
            decision for decision in self._assignment_decisions
            if decision.uav_id is not None
        ]
        self._assignment_index = 0
        self._path_queries = []
        self._path_query_index = 0
        self._cycle_tasks = []
        self._cycle_uavs = []
        self._submit_next_assignment()

    def _record_path_planning_time(self):
        if self._path_request_started_at is None:
            return
        self._path_planning_time_sec += (
            time.perf_counter() - self._path_request_started_at
        )
        self._path_request_started_at = None
        message = Float64()
        message.data = self._path_planning_time_sec
        self.path_planning_time_publisher.publish(message)

    def _report_unallocatable_tasks(self):
        all_uavs_available = (
            len(self._cycle_uavs) == len(self.uav_ids)
            and all(
                uav.connected
                and uav.operational
                and uav.mission_status in (
                    UAVState.IDLE, UAVState.COMPLETED
                )
                for uav in self._cycle_uavs
            )
        )
        if not all_uavs_available:
            return
        limit = int(self._parameter(
            'unallocatable_cycles_before_failure'
        ))
        terminal = []
        for decision in self._assignment_decisions:
            if decision.uav_id is not None:
                self._unallocatable_cycles.pop(decision.task_id, None)
                continue
            if 'reserved for another task' in decision.reason:
                continue
            attempts = self._unallocatable_cycles.get(decision.task_id, 0) + 1
            self._unallocatable_cycles[decision.task_id] = attempts
            if attempts >= limit:
                terminal.append({
                    'task_id': decision.task_id,
                    'reason': decision.reason,
                })
                self._unallocatable_cycles.pop(decision.task_id, None)
        if terminal:
            message = String()
            message.data = json.dumps(terminal)
            self.unallocatable_tasks_publisher.publish(message)

    def _current_uav_snapshots(self, now):
        snapshots = []
        for uav_id, data in self.uav_data.items():
            state = data.state
            odometry = data.odometry
            battery = data.battery
            receive_times = data.received_at
            if (
                state is None
                or odometry is None
                or battery is None
                or not bool(battery.present)
            ):
                continue
            battery_percentage = float(battery.percentage)
            if not math.isfinite(battery_percentage):
                continue
            orientation = odometry.pose.pose.orientation
            yaw = math.atan2(
                2.0 * (
                    orientation.w * orientation.z
                    + orientation.x * orientation.y
                ),
                1.0 - 2.0 * (
                    orientation.y * orientation.y
                    + orientation.z * orientation.z
                ),
            )
            ages = [
                now - receive_times.get(key, float('inf'))
                for key in ('state', 'odometry', 'battery')
            ]
            snapshots.append(UavSnapshot(
                uav_id=uav_id,
                position=(
                    float(odometry.pose.pose.position.x),
                    float(odometry.pose.pose.position.y),
                    float(odometry.pose.pose.position.z),
                ),
                battery_percentage=battery_percentage * 100.0,
                connected=bool(state.connected),
                operational=bool(state.operational),
                mission_status=int(state.mission_status),
                state_age_sec=max(ages),
                yaw=yaw,
            ))
        return snapshots

    def _submit_next_assignment(self):
        if self._assignment_index >= len(self._assignment_decisions):
            self._finish_allocation_cycle()
            return
        if not self.assign_task_client.service_is_ready():
            self.get_logger().warning(
                'Waiting for task manager AssignTask service.'
            )
            self._finish_allocation_cycle()
            return

        decision = self._assignment_decisions[self._assignment_index]
        if decision.uav_id is None or decision.bid is None:
            self._assignment_index += 1
            self._submit_next_assignment()
            return
        request = AssignTask.Request()
        request.task_id = decision.task_id
        request.uav_id = decision.uav_id
        request.outbound_distance_m = float(
            decision.outbound_distance_m or 0.0
        )
        request.return_distance_m = float(
            decision.return_distance_m or 0.0
        )
        future = self.assign_task_client.call_async(request)
        future.add_done_callback(
            partial(self._assignment_response_received, decision)
        )

    def _assignment_response_received(
        self,
        decision: AllocationDecision,
        future: Future,
    ) -> None:
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(
                f"Assignment confirmation failed for task "
                f"'{decision.task_id}': {error}"
            )
            response = None

        if response is not None and response.success:
            self.get_logger().info(
                f"Assigned task '{decision.task_id}' to "
                f"'{decision.uav_id}' with bid {decision.bid:.3f}."
            )
        elif response is not None:
            self.get_logger().warning(
                f"Task manager rejected '{decision.task_id}' for "
                f"'{decision.uav_id}': {response.message}"
            )

        self._assignment_index += 1
        self._submit_next_assignment()

    def _finish_allocation_cycle(self):
        self._allocation_active = False
        self._next_cycle_time = (
            time.monotonic()
            + float(self._parameter('auction_retry_interval_sec'))
        )


def main(args=None):
    rclpy.init(args=args)
    node = AuctionAllocatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
