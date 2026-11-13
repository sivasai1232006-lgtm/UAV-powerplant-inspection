import math
import json
from dataclasses import dataclass, field
from functools import partial
import time
from typing import Any, Literal, cast
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.task import Future
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float64, String
from uav_interfaces.msg import TaskStatus, UAVState
from uav_interfaces.srv import AssignTask, GetUnassignedTasks
from uav_allocator.battery_feasibility import (BatteryFeasibility,FeasibilityResult,)
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
    current_task: InspectionTask | None = None
    queued_tasks: tuple[InspectionTask, ...] = ()
    current_task_inspecting: bool = False
    remaining_inspection_sec: float = 0.0
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
    completion_return_emergency_threshold_percentage: float
    def __init__(
        self,
        path_cost, workload_estimator, battery_feasibility, cost_function, state_timeout_sec,
        planning_batch_size=4, maximum_workload_sec=1800.0, workload_warning_sec=900.0, battery_warning_margin_percentage=15.0,
        future_lookahead_tasks=3, minimum_separation_m=3.0, recovery_position=None, landing_energy_percentage=2.0, landing_altitude_m=0.2,
        post_landing_reserve_percentage=None, cruise_altitude_m=7.0,):
        if state_timeout_sec <= 0.0:
            raise ValueError('State timeout must be greater than zero')
        self.path_cost = path_cost
        self.workload_estimator = workload_estimator
        self.battery_feasibility = battery_feasibility
        self.cost_function = cost_function
        self.state_timeout_sec = float(state_timeout_sec)
        if planning_batch_size < 1:
            raise ValueError('Planning batch size must be positive')
        self.planning_batch_size = int(planning_batch_size)
        self.maximum_workload_sec = float(maximum_workload_sec)
        self.workload_warning_sec = float(workload_warning_sec)
        self.battery_warning_margin_percentage = float(battery_warning_margin_percentage)
        self.future_lookahead_tasks = int(future_lookahead_tasks)
        self.minimum_separation_m = float(minimum_separation_m)
        self.recovery_position = recovery_position
        self.landing_energy_percentage = float(landing_energy_percentage)
        self.landing_altitude_m = float(landing_altitude_m)
        self.completion_return_emergency_threshold_percentage = 5.0
        self.post_landing_reserve_percentage = (
            self.battery_feasibility.minimum_reserve_percentage
            if post_landing_reserve_percentage is None
            else float(post_landing_reserve_percentage)
        )
        if not math.isclose(
            self.post_landing_reserve_percentage,
            self.battery_feasibility.minimum_reserve_percentage,
        ):
            raise ValueError(
                'Allocator and battery feasibility reserve percentages '
                'must match'
            )
        if not math.isclose(
            self.landing_energy_percentage,
            self.battery_feasibility.landing_energy_percentage,
        ):
            raise ValueError(
                'Allocator and battery feasibility landing energy '
                'percentages must match'
            )
        self.cruise_altitude_m = float(cruise_altitude_m)

    def allocate(self,tasks,uavs,base_positions,already_assigned_task_ids=(),allowed_uav_ids_by_task=None,):
        tasks = sorted(tasks,key=lambda task: (int(task.priority), str(task.task_id)),)[:self.planning_batch_size]
        uavs = list(uavs)
        assigned_task_ids = set(already_assigned_task_ids)
        decisions = []
        landing_homes = {
            uav_id: (
                float(position[0]),
                float(position[1]),
                self.landing_altitude_m,
            )
            if self._valid_position(position) else position
            for uav_id, position in base_positions.items()
        }
        self._allocation_homes = landing_homes
        routes = {uav.uav_id: self._existing_workload(uav) for uav in uavs}
        snapshots = {uav.uav_id: uav for uav in uavs}
        committed_uav_ids = set()
        pending_tasks = []
        seen_task_ids = set(assigned_task_ids)
        for task in tasks:
            if not task.task_id or task.task_id in seen_task_ids:
                decisions.append(AllocationDecision(
                    task.task_id, None, None,
                    'task is invalid, duplicated, or already assigned',
                ))
            elif not self._valid_task(task):
                decisions.append(AllocationDecision(
                    task.task_id, None, None, 'task data is invalid'
                ))
                seen_task_ids.add(task.task_id)
            else:
                pending_tasks.append(task)
                seen_task_ids.add(task.task_id)

        # Select the cheapest feasible task/UAV pair at each step. Rebuild
        # bids after each selection so projected routes and return energy are
        # evaluated against the assignments already made in this cycle.
        rejection_reasons = {task.task_id: [] for task in pending_tasks}
        while pending_tasks:
            candidates = []
            for task in pending_tasks:
                for uav in uavs:
                    if uav.uav_id in committed_uav_ids:
                        continue
                    if (
                        allowed_uav_ids_by_task is not None
                        and uav.uav_id not in allowed_uav_ids_by_task.get(
                            task.task_id, ()
                        )
                    ):
                        rejection_reasons[task.task_id].append(
                            f'{uav.uav_id}: task goal is outside operating '
                            'bounds'
                        )
                        continue
                    eligible, reason = self._eligible(uav)
                    if not eligible:
                        rejection_reasons[task.task_id].append(
                            f'{uav.uav_id}: {reason}'
                        )
                        continue

                    home = landing_homes.get(uav.uav_id)
                    if home is None or not self._valid_position(home):
                        rejection_reasons[task.task_id].append(
                            f'{uav.uav_id}: home position is unavailable'
                        )
                        continue

                    candidate_route = list(routes[uav.uav_id])
                    candidate_route.append(task)
                    if self._conflicts_with_other_routes(
                        task, routes, uav.uav_id
                    ):
                        rejection_reasons[task.task_id].append(
                            f'{uav.uav_id}: task is too close to another '
                            'UAV route'
                        )
                        continue
                    metrics = self._route_metrics(uav, candidate_route, home)
                    if metrics is None:
                        rejection_reasons[task.task_id].append(
                            f'{uav.uav_id}: route estimate is unavailable'
                        )
                        continue
                    feasibility = self._route_feasibility(uav, metrics)
                    if not feasibility.feasible:
                        rejection_reasons[task.task_id].append(
                            f'{uav.uav_id}: {feasibility.reason}'
                        )
                        continue
                    old_metrics = self._route_metrics(
                        uav, routes[uav.uav_id], home
                    )
                    if old_metrics is None:
                        rejection_reasons[task.task_id].append(
                            f'{uav.uav_id}: existing route estimate is '
                            'unavailable'
                        )
                        continue
                    remaining = [
                        candidate for candidate in pending_tasks
                        if candidate.task_id != task.task_id
                    ][:self.future_lookahead_tasks]
                    future_penalty = self._future_cost(
                        routes, uav.uav_id, candidate_route, uav,
                        remaining, snapshots,
                    )
                    marginal = self._marginal_cost(
                        uav, task, metrics, old_metrics,
                        future_penalty=future_penalty,
                    )
                    candidates.append((
                        marginal['cost'], task.task_id, uav.uav_id,
                        max(
                            0.0,
                            metrics['outbound_distance_m']
                            - old_metrics['outbound_distance_m'],
                        ),
                        metrics['return_distance_m'], candidate_route,
                        task,
                    ))

            if not candidates:
                break
            (
                bid, _, selected_uav_id, outbound, return_distance, route,
                selected_task,
            ) = min(candidates, key=lambda candidate: (
                candidate[0], candidate[1], candidate[2]
            ))
            assigned_task_ids.add(selected_task.task_id)
            committed_uav_ids.add(selected_uav_id)
            routes[selected_uav_id] = route
            pending_tasks.remove(selected_task)
            decisions.append(AllocationDecision(
                selected_task.task_id, selected_uav_id, bid,
                'lowest feasible marginal bid', outbound, return_distance,
            ))

        for task in pending_tasks:
            reasons = rejection_reasons.get(task.task_id, [])
            more_eligible_uavs = any(
                uav.uav_id not in committed_uav_ids
                and self._eligible(uav)[0]
                for uav in uavs
            )
            if (
                committed_uav_ids and not more_eligible_uavs
                and not reasons
            ):
                reason = (
                    'all eligible UAVs already received one task in this '
                    'allocation cycle'
                )
            else:
                reason = (
                    '; '.join(dict.fromkeys(reasons))
                    or 'no UAVs available'
                )
            decisions.append(AllocationDecision(
                task.task_id, None, None, reason,
            ))

        route_order = {
            (uav_id, task.task_id): index
            for uav_id, route in routes.items()
            for index, task in enumerate(route)
        }
        decisions.sort(key=lambda decision: (
            decision.uav_id is None,
            decision.uav_id or '',
            route_order.get((decision.uav_id, decision.task_id), 10**6),
        ))
        return decisions

    def _route_metrics(self, uav, route, home):
        position = uav.position
        outbound_distance = 0.0
        inspection_duration = 0.0
        for index, task in enumerate(route):
            current_inspection = (
                index == 0
                and uav.current_task is not None
                and task.task_id == uav.current_task.task_id
                and uav.current_task_inspecting
            )
            if current_inspection:
                # The UAV is already hovering at this task. Keep its latest
                # odometry position as the origin for the next queued task;
                # the task's target position is only a planned waypoint and
                # may differ slightly from the UAV's actual position.
                leg_distance = 0.0
                task_duration = max(
                    0.0, float(uav.remaining_inspection_sec)
                )
            else:
                leg_distance = self.path_cost.estimate_distance(
                    position, task.target_position,
                    uav.yaw if position == uav.position else 0.0, 0.0,
                )
                task_duration = float(task.estimated_duration_sec)
            if (
                leg_distance is None
                or not math.isfinite(leg_distance)
                or leg_distance < 0.0
            ):
                return None
            outbound_distance += leg_distance
            inspection_duration += task_duration
            # For an active inspection, position deliberately remains the
            # current odometry position. For a future route leg, advance to
            # the waypoint just evaluated.
            if not current_inspection:
                position = task.target_position

        def recovery_choice(recovery_position):
            leg = self.path_cost.estimate_distance(
                position, recovery_position, 0.0, 0.0
            )
            if leg is None or not math.isfinite(leg) or leg < 0.0:
                return None
            energy = (
                (outbound_distance + leg)
                * self.battery_feasibility.consumption_per_meter
                + inspection_duration / 60.0
                * self.battery_feasibility.inspection_consumption_per_minute
                + self.landing_energy_percentage
            )
            remaining = uav.battery_percentage - energy
            return {
                'position': recovery_position,
                'distance': leg,
                'energy': energy,
                'margin': (
                    remaining
                    - self.post_landing_reserve_percentage
                ),
            }

        normal = (
            recovery_choice(home)
            if home is not None and self._valid_position(home) else None
        )
        if normal is None:
            return None
        recovery = normal
        if (
            normal['margin'] < 0.0
            and self.recovery_position is not None
            and self._valid_position(self.recovery_position)
        ):
            ugv = recovery_choice(self.recovery_position)
            if ugv is not None and ugv['margin'] >= 0.0:
                recovery = ugv
        return_distance = recovery['distance']
        travel_time = (
            outbound_distance + return_distance
        ) / self.workload_estimator.nominal_speed_mps
        energy = recovery['energy']
        return {
            'outbound_distance_m': outbound_distance,
            'return_distance_m': return_distance,
            'inspection_duration_sec': inspection_duration,
            'travel_time_sec': travel_time,
            'workload_sec': travel_time + inspection_duration,
            'energy_percentage': energy,
            'return_position': recovery['position'],
            'recovery_feasible': recovery['margin'] >= 0.0,
            'battery_margin_percentage': recovery['margin'],
        }

    @staticmethod
    def _existing_workload(uav):
        workload = []
        if uav.current_task is not None:
            workload.append(uav.current_task)
        workload.extend(uav.queued_tasks)
        return workload

    def _route_feasibility(self, uav, metrics):
        result = self.battery_feasibility.evaluate(
            uav.battery_percentage,
            metrics['outbound_distance_m'],
            metrics['return_distance_m'],
            metrics['inspection_duration_sec'],
        )
        if metrics['recovery_feasible']:
            return result
        return FeasibilityResult(
            False,
            result.required_percentage,
            result.available_percentage,
            'battery reserve makes both UGV and base return unsafe',
        )

    def _marginal_cost(
        self, uav, task, metrics, old_metrics, future_penalty=0.0
    ):
        marginal_distance = max(
            0.0,
            metrics['outbound_distance_m']
            + metrics['return_distance_m']
            - old_metrics['outbound_distance_m']
            - old_metrics['return_distance_m'],
        )
        marginal_time = max(
            0.0,
            metrics['workload_sec'] - old_metrics['workload_sec'],
        )
        marginal_energy = max(
            0.0,
            metrics['energy_percentage']
            - old_metrics['energy_percentage'],
        )
        remaining_after_route = (
            uav.battery_percentage
            - metrics['energy_percentage']
            - self.battery_feasibility.minimum_reserve_percentage
        )
        battery_margin = max(self.battery_warning_margin_percentage, 1.0)
        battery_penalty = max(
            0.0,
            (battery_margin - remaining_after_route) / battery_margin,
        )
        workload_penalty = max(
            0.0,
            metrics['workload_sec'] / max(self.maximum_workload_sec, 1.0)
            - self.workload_warning_sec / max(self.maximum_workload_sec, 1.0),
        )
        return {
            'cost': self.cost_function.calculate(
                marginal_distance,
                marginal_time,
                marginal_energy,
                task.priority,
                battery_penalty=battery_penalty,
                workload_penalty=workload_penalty,
                future_penalty=future_penalty,
            ),
            'battery_penalty': battery_penalty,
        }

    def _future_cost(
        self, routes, changed_uav_id, changed_route, changed_uav,
        future_tasks, snapshots,
    ):
        if not future_tasks:
            return 0.0
        projected_routes = {
            uav_id: list(route) for uav_id, route in routes.items()
        }
        projected_routes[changed_uav_id] = list(changed_route)
        projected_cost = 0.0
        for future_task in future_tasks:
            choices = []
            for uav_id, route in projected_routes.items():
                uav = snapshots[uav_id]
                if not self._eligible(uav)[0]:
                    continue
                home = getattr(self, '_allocation_homes', {}).get(uav_id)
                if home is None:
                    continue
                candidate_route = list(route)
                candidate_route.append(future_task)
                if self._conflicts_with_other_routes(
                    future_task, projected_routes, uav_id
                ):
                    continue
                metrics = self._route_metrics(uav, candidate_route, home)
                old_metrics = self._route_metrics(uav, route, home)
                if metrics is None or old_metrics is None:
                    continue
                if not self._route_feasibility(uav, metrics).feasible:
                    continue
                marginal = self._marginal_cost(
                    uav, future_task, metrics, old_metrics
                )
                choices.append((marginal['cost'], uav_id, candidate_route))
            if not choices:
                projected_cost += 1.0
                continue
            cost, uav_id, route = min(
                choices, key=lambda item: (item[0], item[1])
            )
            projected_cost += cost
            projected_routes[uav_id] = route
        return projected_cost

    def _conflicts_with_other_routes(self, task, routes, candidate_uav_id):
        for uav_id, route in routes.items():
            if uav_id == candidate_uav_id:
                continue
            for assigned_task in route:
                if math.dist(
                    task.target_position, assigned_task.target_position
                ) < self.minimum_separation_m:
                    return True
        return False

    def _eligible(self, uav):
        if not uav.uav_id:
            return False, 'UAV id is empty'
        if not uav.connected:
            return False, 'UAV is disconnected'
        if not uav.operational:
            return False, 'UAV is unhealthy or non-operational'
        if uav.mission_status in (UAVState.IDLE, UAVState.COMPLETED):
            if uav.current_task is not None or uav.queued_tasks:
                return False, 'idle UAV has an assigned workload'
        elif uav.mission_status == UAVState.INSPECTING:
            if (
                uav.current_task is None
                or not uav.current_task_inspecting
                or uav.queued_tasks
            ):
                return False, (
                    'inspecting UAV has no active inspection or its queue '
                    'slot is occupied'
                )
        else:
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


def allocator_from_config(config):
    if 'allocator' in config:
        config = config['allocator']['ros__parameters']
    cost_config = config['cost']
    return AuctionAllocator(
        path_cost=PathCostClient(
            config.get('distance_safety_factor', 1.3),
            cruise_altitude_m=config.get('cruise_altitude_m', 7.0),
        ),
        workload_estimator=WorkloadEstimator(
            config['nominal_speed_mps']
        ),
        battery_feasibility=BatteryFeasibility(
            config['battery_consumption_per_meter'],
            config['inspection_consumption_per_minute'],
            config.get('post_landing_reserve_percentage', 10.0),
            config.get('landing_energy_percentage', 2.0),
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
        planning_batch_size=config.get('planning_batch_size', 4),
        maximum_workload_sec=config.get('maximum_workload_sec', 1800.0),
        workload_warning_sec=config.get('workload_warning_sec', 900.0),
        battery_warning_margin_percentage=config.get(
            'battery_warning_margin_percentage', 15.0
        ),
        future_lookahead_tasks=config.get('future_lookahead_tasks', 3),
        minimum_separation_m=config.get('minimum_separation_m', 3.0),
        recovery_position=(
            tuple(float(value) for value in config['recovery_position'])
            if 'recovery_position' in config else None
        ),
        landing_energy_percentage=config.get(
            'landing_energy_percentage', 2.0
        ),
        landing_altitude_m=config.get('landing_altitude_m', 0.2),
        post_landing_reserve_percentage=config.get(
            'post_landing_reserve_percentage', 10.0
        ),
        cruise_altitude_m=config.get('cruise_altitude_m', 7.0),
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
        self.allocator = allocator_from_config(self._allocator_config())
        self.uav_data: dict[str, UavTelemetry] = {
            uav_id: UavTelemetry()
            for uav_id in self.uav_ids
        }
        self.get_tasks_client = self.create_client(
            GetUnassignedTasks,
            str(self._parameter('get_unassigned_tasks_service')),
        )
        self.get_tasks_service_name = str(
            self._parameter('get_unassigned_tasks_service')
        )
        self.assign_task_client = self.create_client(
            AssignTask,
            str(self._parameter('assign_task_service')),
        )
        self._create_uav_subscriptions()
        self._allocation_active = False
        self._allocation_requested = False
        self._task_service_connected = False
        self._last_service_wait_log = 0.0
        self._assignment_decisions: list[AllocationDecision] = []
        self._assignment_index = 0
        self._next_cycle_time = 0.0
        self._unallocatable_cycles: dict[str, int] = {}
        self.task_statuses: dict[str, int] = {}
        self.completed_task_ids: set[str] = set()
        task_status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1000,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.task_status_subscription = self.create_subscription(
            TaskStatus,
            '/mission_manager/task_status',
            self._task_status_callback,
            task_status_qos,
        )
        self.path_planning_time_publisher = self.create_publisher(
            Float64, '/auction_allocator/path_planning_time', 10
        )
        self.unallocatable_tasks_publisher = self.create_publisher(
            String, '/mission_manager/unallocatable_tasks', 10
        )
        self.timer = self.create_timer(
            float(self._parameter('allocation_poll_sec')),
            self._start_allocation_cycle,
        )
        self.service_discovery_timer = self.create_timer(
            1.0, self._check_task_manager_service
        )
        self.allocation_event_subscription = self.create_subscription(
            String,
            '/mission_manager/allocation_event',
            self._allocation_event_callback,
            10,
        )

    def _declare_parameters(self):
        defaults = {
            'uav_ids': ['uav_1', 'uav_2'],
            'planning_frame': 'map',
            'topics.uav_state': '/{uav_id}/uav_state',
            'topics.odometry': '/{uav_id}/odom',
            'topics.battery': '/{uav_id}/battery/measured',
            'get_unassigned_tasks_service': (
                '/mission_manager/get_unassigned_tasks'
            ),
            'assign_task_service': '/mission_manager/assign_task',
            'planning_batch_size': 4,
            'maximum_workload_sec': 1800.0,
            'workload_warning_sec': 900.0,
            'battery_warning_margin_percentage': 15.0,
            'future_lookahead_tasks': 3,
            'minimum_separation_m': 3.0,
            'distance_safety_factor': 1.3,
            'cruise_altitude_m': 7.0,
            'recovery_position': [-5.0, 0.0, 0.2],
            'landing_energy_percentage': 2.0,
            'landing_altitude_m': 0.2,
            'post_landing_reserve_percentage': 10.0,
            'nominal_speed_mps': 3.6,
            'state_timeout_sec': 3.0,
            'auction_retry_interval_sec': 1.0,
            'allocation_poll_sec': 10.0,
            'unallocatable_cycles_before_failure': 5,
            'battery_consumption_per_meter': 0.05,
            'inspection_consumption_per_minute': 0.25,
            'cost.weights.distance': 0.4,
            'cost.weights.time': 0.25,
            'cost.weights.energy': 0.25,
            'cost.weights.priority': 0.1,
            'cost.weights.battery': 0.2,
            'cost.weights.workload': 0.15,
            'cost.weights.future': 0.25,
            'cost.distance_scale_m': 100.0,
            'cost.time_scale_sec': 300.0,
            'cost.energy_scale_percentage': 10.0,
            'cost.minimum_priority': 1,
            'cost.maximum_priority': 5,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        for uav_id, position in (
            ('uav_1', [-5.0, 0.0, 3.2]),
            ('uav_2', [5.0, 0.0, 3.2]),
        ):
            self.declare_parameter(f'base_positions.{uav_id}', position)

    def _parameter(self, name: str) -> Any:
        return self.get_parameter(name).value

    def _allocator_config(self):
        return {
            'nominal_speed_mps': float(
                self._parameter('nominal_speed_mps')
            ),
            'planning_batch_size': int(
                self._parameter('planning_batch_size')
            ),
            'maximum_workload_sec': float(
                self._parameter('maximum_workload_sec')
            ),
            'workload_warning_sec': float(
                self._parameter('workload_warning_sec')
            ),
            'battery_warning_margin_percentage': float(
                self._parameter('battery_warning_margin_percentage')
            ),
            'future_lookahead_tasks': int(
                self._parameter('future_lookahead_tasks')
            ),
            'distance_safety_factor': float(
                self._parameter('distance_safety_factor')
            ),
            'cruise_altitude_m': float(
                self._parameter('cruise_altitude_m')
            ),
            'recovery_position': tuple(
                float(value)
                for value in self._parameter('recovery_position')
            ),
            'landing_energy_percentage': float(
                self._parameter('landing_energy_percentage')
            ),
            'landing_altitude_m': float(
                self._parameter('landing_altitude_m')
            ),
            'post_landing_reserve_percentage': float(
                self._parameter('post_landing_reserve_percentage')
            ),
            'minimum_separation_m': float(
                self._parameter('minimum_separation_m')
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
            'cost': {
                'weights': {
                    name: float(self._parameter(f'cost.weights.{name}'))
                    for name in (
                        'distance', 'time', 'energy', 'priority',
                        'battery', 'workload', 'future',
                    )
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
        if not self._has_available_uav():
            return
        if not self.get_tasks_client.service_is_ready():
            self._task_service_connected = False
            return

        self._task_service_connected = True
        self._allocation_active = True
        future = self.get_tasks_client.call_async(
            GetUnassignedTasks.Request()
        )
        future.add_done_callback(self._unassigned_tasks_received)

    def _has_available_uav(self):
        now = time.monotonic()
        for uav_id, data in self.uav_data.items():
            state = data.state
            odometry = data.odometry
            battery = data.battery
            if (
                state is None or odometry is None or battery is None
                or not bool(battery.present)
                or not state.connected or not state.operational
                or state.mission_status not in (
                    UAVState.IDLE, UAVState.COMPLETED, UAVState.INSPECTING
                )
            ):
                continue
            ages = [
                now - data.received_at.get(key, float('inf'))
                for key in ('state', 'odometry', 'battery')
            ]
            if (
                all(0.0 <= age <= self.allocator.state_timeout_sec
                    for age in ages)
                and self.allocator._valid_position((
                    float(odometry.pose.pose.position.x),
                    float(odometry.pose.pose.position.y),
                    float(odometry.pose.pose.position.z),
                ))
                and math.isfinite(float(battery.percentage))
                and 0.0 <= float(battery.percentage) <= 1.0
            ):
                # The service response supplies current and queued task
                # details; the allocator applies the exact eligibility rule
                # after it receives that workload snapshot.
                return True
        return False

    def _check_task_manager_service(self):
        if self.get_tasks_client.service_is_ready():
            if not self._task_service_connected:
                self._task_service_connected = True
                self.get_logger().info(
                    f"Connected to GetUnassignedTasks service "
                    f"'{self.get_tasks_service_name}'."
                )
                self._next_cycle_time = 0.0
                self._start_allocation_cycle()
            return

        self._task_service_connected = False
        now = time.monotonic()
        if now - self._last_service_wait_log >= 5.0:
            self.get_logger().warning(
                f"Waiting for GetUnassignedTasks service "
                f"'{self.get_tasks_service_name}'. Verify that "
                "mission_manager is running in the same ROS domain."
            )
            self._last_service_wait_log = now

    def _allocation_event_callback(self, _message):
        if self._allocation_active:
            self._allocation_requested = True
            return
        self._next_cycle_time = 0.0
        self._start_allocation_cycle()

    def _unassigned_tasks_received(
        self, future: Future
    ) -> None:
        try:
            result = future.result()
        except Exception as error:
            self._task_service_connected = False
            self.get_logger().error(
                f"GetUnassignedTasks call to "
                f"'{self.get_tasks_service_name}' failed: {error}"
            )
            self._finish_allocation_cycle()
            return
        if result is None:
            self._task_service_connected = False
            self.get_logger().error(
                f"Service '{self.get_tasks_service_name}' returned no "
                'response; discovery will retry.'
            )
            self._finish_allocation_cycle()
            return
        response = cast(GetUnassignedTasks.Response, result)

        if not response.tasks:
            self._finish_allocation_cycle()
            return
        planning_frame = str(self._parameter('planning_frame'))
        mismatched_frames = sorted({
            str(task.frame_id)
            for task in response.tasks
            if str(task.frame_id) != planning_frame
        })
        if mismatched_frames:
            self.get_logger().error(
                f'Refusing allocation: task frames {mismatched_frames} '
                f'do not match planning frame {planning_frame!r}.'
            )
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
            if str(task.task_id) not in self.completed_task_ids
        ]
        tasks.sort(key=lambda task: (task.priority, task.task_id))
        if not tasks:
            self._finish_allocation_cycle()
            return
        tasks = tasks[:int(self._parameter('planning_batch_size'))]
        now = time.monotonic()
        self._cycle_tasks = tasks
        workload_by_uav = {uav_id: [] for uav_id in self.uav_ids}
        for task, uav_id, inspecting, remaining_sec in zip(
            response.assigned_tasks,
            response.assigned_uav_ids,
            response.assigned_tasks_inspecting,
            response.remaining_inspection_sec,
        ):
            if uav_id not in workload_by_uav:
                continue
            workload_by_uav[uav_id].append((
                InspectionTask(
                    task_id=str(task.task_id),
                    target_position=(
                        float(task.target_position.x),
                        float(task.target_position.y),
                        float(task.target_position.z),
                    ),
                    priority=int(task.priority),
                    estimated_duration_sec=float(task.estimated_duration),
                ),
                bool(inspecting),
                max(0.0, float(remaining_sec)),
            ))
        self._cycle_uavs = self._current_uav_snapshots(
            now, workload_by_uav
        )
        if not any(
            self.allocator._eligible(uav)[0] for uav in self._cycle_uavs
        ):
            self._finish_allocation_cycle()
            return
        self._allocate_from_euclidean_estimates()

    def _task_status_callback(self, message: TaskStatus) -> None:
        task_id = str(message.task_id)
        if not task_id:
            return
        self.task_statuses[task_id] = int(message.status)
        if message.status == UAVState.COMPLETED:
            self.completed_task_ids.add(task_id)

    def _allocate_from_euclidean_estimates(self):
        started_at = time.perf_counter()
        self._assignment_decisions = self.allocator.allocate(
            self._cycle_tasks,
            self._cycle_uavs,
            self.base_positions,
        )
        allocation_duration = time.perf_counter() - started_at
        planning_time = Float64()
        planning_time.data = allocation_duration
        self.path_planning_time_publisher.publish(planning_time)
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
        self._cycle_tasks = []
        self._cycle_uavs = []
        self._submit_next_assignment()

    def _report_unallocatable_tasks(self):
        some_uav_available = any(
            self.allocator._eligible(uav)[0] for uav in self._cycle_uavs
        )
        if not some_uav_available:
            return
        limit = int(self._parameter(
            'unallocatable_cycles_before_failure'
        ))
        terminal = []
        for decision in self._assignment_decisions:
            if decision.uav_id is not None:
                self._unallocatable_cycles.pop(decision.task_id, None)
                continue
            if (
                'not ready for another task' in decision.reason
                or 'disconnected' in decision.reason
                or 'non-operational' in decision.reason
                or 'already received one task in this allocation cycle'
                in decision.reason
            ):
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

    def _current_uav_snapshots(self, now, workload_by_uav=None):
        workload_by_uav = workload_by_uav or {}
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
            if odometry.header.frame_id != str(
                self._parameter('planning_frame')
            ):
                continue
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
            workload = workload_by_uav.get(uav_id, [])
            current = workload[0] if workload else None
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
                current_task=current[0] if current else None,
                queued_tasks=tuple(item[0] for item in workload[1:]),
                current_task_inspecting=(
                    current[1] if current else False
                ),
                remaining_inspection_sec=(
                    current[2] if current else 0.0
                ),
            ))
        return snapshots

    def _submit_next_assignment(self):
        if self._assignment_index >= len(self._assignment_decisions):
            self._finish_allocation_cycle()
            return
        if not self.assign_task_client.service_is_ready():
            self.get_logger().warning(
                "Waiting for task manager AssignTask service "
                f"'{self._parameter('assign_task_service')}'."
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
        if self._allocation_requested:
            self._allocation_requested = False
            self._next_cycle_time = 0.0
            self._start_allocation_cycle()
            return
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