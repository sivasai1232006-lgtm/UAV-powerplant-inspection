import math
import threading

from uav_interfaces.msg import Task


DEFAULT_INSPECTION_ALTITUDES_M = (3.0, 5.0, 7.0, 10.0)


def select_inspection_altitude(
    object_height_m,
    altitudes_m=DEFAULT_INSPECTION_ALTITUDES_M,
    maximum_altitude_m=None,
):
    """Choose the first configured altitude above the object's height."""
    try:
        object_height = float(object_height_m)
        altitudes = tuple(float(value) for value in altitudes_m)
        maximum_altitude = (
            float(maximum_altitude_m)
            if maximum_altitude_m is not None else math.inf
        )
    except (TypeError, ValueError):
        raise ValueError('Inspection altitude settings must be numeric')

    if (
        not math.isfinite(object_height)
        or math.isnan(maximum_altitude)
        or object_height < 0.0
        or altitudes != DEFAULT_INSPECTION_ALTITUDES_M
    ):
        raise ValueError('Inspection altitude settings are invalid')

    preferred = next(
        (
            candidate
            for candidate in altitudes
            if candidate > object_height and candidate <= maximum_altitude
        ),
        None,
    )
    if preferred is not None:
        return preferred
    if altitudes and altitudes[-1] <= maximum_altitude:
        return altitudes[-1]

    raise ValueError(
        f'No available inspection altitude is within the configured maximum '
        f'of {maximum_altitude:.2f} m for an object {object_height:.2f} m high'
    )


def tasks_from_mission(mission_data, infeasible_tasks=None):
    """Load tasks with explicit waypoint Z, or legacy object-height bands."""
    mission = mission_data['mission']
    frame_id = str(mission.get('frame_id', ''))
    if not frame_id:
        raise ValueError('Mission must specify a coordinate frame_id')
    flight_levels = mission['flight_levels']
    altitude = float(flight_levels['standard_altitude_m'])
    inspection_altitudes = tuple(
        float(value)
        for value in flight_levels.get(
            'inspection_altitudes_m', DEFAULT_INSPECTION_ALTITUDES_M
        )
    )
    maximum_altitude = float(mission['constraints']['maximum_altitude_m'])
    vertical_half_extent = float(
        mission['constraints'].get('uav_vertical_half_extent_m', 0.25)
    )
    vertical_clearance = float(
        mission['constraints'].get('vertical_obstacle_clearance_m', 0.25)
    )
    if any(not math.isfinite(value) for value in (
        altitude, maximum_altitude, vertical_half_extent, vertical_clearance,
        *inspection_altitudes,
    )):
        raise ValueError('Mission altitude settings must be finite')
    if vertical_half_extent < 0.0 or vertical_clearance < 0.0:
        raise ValueError('Mission vertical clearances cannot be negative')
    if (
        altitude - vertical_half_extent < vertical_clearance
        or altitude + vertical_half_extent + vertical_clearance
        > maximum_altitude
    ):
        raise ValueError(
            'Configured cruise altitude is outside the safe altitude range'
        )
    if altitude not in inspection_altitudes:
        raise ValueError(
            'Configured cruise altitude must be one of the inspection bands'
        )
    tasks = []

    for task_data in mission['tasks']:
        task_type = str(task_data.get('type', 'inspection')).lower()
        if task_type != 'inspection':
            raise ValueError(
                f"Task '{task_data.get('id', '')}' has unsupported type "
                f"'{task_type}'; only simulated inspection is supported"
            )
        task = Task()
        task.task_id = task_data['id']
        task.frame_id = frame_id
        waypoint = task_data.get('waypoint')
        if not isinstance(waypoint, (list, tuple)) or len(waypoint) not in (
            2, 3
        ):
            raise ValueError(
                f"Task '{task.task_id}' waypoint must contain X/Y and "
                'optionally a target altitude.'
            )
        target_x = float(waypoint[0])
        target_y = float(waypoint[1])
        has_object_height = 'object_height_m' in task_data
        if has_object_height:
            object_height = float(task_data['object_height_m'])
        elif len(waypoint) == 3:
            object_height = 0.0
        else:
            raise ValueError(
                f"Task '{task.task_id}' must specify object_height_m when "
                'waypoint does not include a target altitude.'
            )
        priority = int(task_data['priority'])
        duration = float(task_data['estimated_duration_s'])
        explicit_target_altitude = (
            float(waypoint[2]) if len(waypoint) == 3 else None
        )
        if (
            any(not math.isfinite(value) for value in (
                target_x, target_y, object_height, duration,
            ))
            or (
                explicit_target_altitude is not None
                and not math.isfinite(explicit_target_altitude)
            )
            or priority <= 0 or duration < 0.0
        ):
            raise ValueError(
                f"Task '{task_data.get('id', '')}' has invalid coordinates, "
                'priority, object height, or inspection duration'
            )
        if explicit_target_altitude is not None:
            inspection_altitude = explicit_target_altitude
            if (
                not any(
                    math.isclose(
                        inspection_altitude, altitude, abs_tol=1e-6
                    )
                    for altitude in inspection_altitudes
                )
                or inspection_altitude > maximum_altitude
            ):
                message = (
                    f"Task '{task.task_id}' is infeasible: target altitude "
                    f'{inspection_altitude:g} m must match a configured '
                    'inspection altitude and be within the mission ceiling.'
                )
                if infeasible_tasks is None:
                    raise ValueError(message)
                infeasible_tasks[task.task_id] = message
                inspection_altitude = inspection_altitudes[-1]
        else:
            try:
                inspection_altitude = select_inspection_altitude(
                    object_height,
                    inspection_altitudes,
                    maximum_altitude,
                )
            except ValueError as error:
                message = f"Task '{task.task_id}' is infeasible: {error}"
                if infeasible_tasks is None:
                    raise ValueError(message) from error
                infeasible_tasks[task.task_id] = message
                inspection_altitude = inspection_altitudes[-1]
        task.target_position.x = target_x
        task.target_position.y = target_y
        task.target_position.z = inspection_altitude
        task.object_height_m = object_height
        task.priority = priority
        task.estimated_duration = duration
        tasks.append(task)

    return tasks


class TaskManager:
    def __init__(
        self, tasks, frame_id='map', maximum_altitude_m=None,
        vertical_half_extent_m=0.25, vertical_clearance_m=0.25,
        inspection_altitudes_m=DEFAULT_INSPECTION_ALTITUDES_M,
        infeasible_tasks=None,
    ):
        self.tasks = {task.task_id: task for task in tasks}
        self.frame_id = str(frame_id)
        self.maximum_altitude_m = (
            float(maximum_altitude_m)
            if maximum_altitude_m is not None else None
        )
        self.vertical_half_extent_m = float(vertical_half_extent_m)
        self.vertical_clearance_m = float(vertical_clearance_m)
        self.inspection_altitudes_m = tuple(
            float(value) for value in inspection_altitudes_m
        )
        self.failed = dict(infeasible_tasks or {})
        if not set(self.failed).issubset(self.tasks):
            raise ValueError('Infeasible task ids must exist in the task list')
        if (
            self.vertical_half_extent_m < 0.0
            or self.vertical_clearance_m < 0.0
            or not math.isfinite(self.vertical_half_extent_m)
            or not math.isfinite(self.vertical_clearance_m)
            or (
                self.maximum_altitude_m is not None
                and (
                    not math.isfinite(self.maximum_altitude_m)
                    or self.maximum_altitude_m <= 0.0
                )
            )
        ):
            raise ValueError('Task manager altitude limits are invalid')
        self.assignments = {}
        self.assignment_distances = {}
        self.statuses = {
            task_id: (
                'FAILED' if task_id in self.failed else 'UNASSIGNED'
            )
            for task_id in self.tasks
        }
        self.completed = set()
        self.cancelled = set()
        self.reserved_reasons = {}
        self.navigation_retries = {}
        self._lock = threading.RLock()

    def add_task(self, task):
        if not task.task_id:
            return False, 'Task id cannot be empty.'
        if task.frame_id != self.frame_id:
            return False, (
                f"Task frame '{task.frame_id}' does not match mission "
                f"frame '{self.frame_id}'."
            )
        if task.task_id in self.tasks:
            return False, f"Task '{task.task_id}' already exists."
        values = (
            task.target_position.x,
            task.target_position.y,
            task.target_position.z,
            task.estimated_duration,
            task.object_height_m,
        )
        if (
            task.priority <= 0
            or any(not math.isfinite(float(value)) for value in values)
            or task.estimated_duration < 0.0
        ):
            return False, (
                'Task data must have finite coordinates and duration.'
            )
        if self.maximum_altitude_m is not None and (
            task.target_position.z - self.vertical_half_extent_m
            < self.vertical_clearance_m
            or task.target_position.z > self.maximum_altitude_m
        ):
            return False, (
                'Task altitude is outside the configured safe vertical range.'
            )
        try:
            expected_altitude = select_inspection_altitude(
                task.object_height_m,
                self.inspection_altitudes_m,
                self.maximum_altitude_m,
            )
        except ValueError as error:
            return False, f'Inspection task is infeasible: {error}'
        if not math.isclose(
            float(task.target_position.z), expected_altitude, abs_tol=1e-6
        ):
            return False, (
                'Task altitude does not match the safe preferred inspection '
                f'band of {expected_altitude:g} m.'
            )
        with self._lock:
            if task.task_id in self.tasks:
                return False, f"Task '{task.task_id}' already exists."
            self.tasks[task.task_id] = task
            self.statuses[task.task_id] = 'UNASSIGNED'
        return True, f"Task '{task.task_id}' added."

    def unassigned_tasks(self):
        with self._lock:
            return [
                task for task_id, task in self.tasks.items()
                if self.statuses.get(task_id) == 'UNASSIGNED'
            ]

    def assign(
        self, task_id, uav_id, outbound_distance_m=0.0,
        return_distance_m=0.0,
    ):
        with self._lock:
            if task_id not in self.tasks:
                return False, f"Task '{task_id}' does not exist."
            if self.statuses.get(task_id) in (
                'COMPLETED', 'FAILED', 'CANCELLED'
            ):
                return False, f"Task '{task_id}' is already terminal."
            if self.statuses.get(task_id) == 'ASSIGNED':
                return False, f"Task '{task_id}' is already assigned."
            if self.statuses.get(task_id) != 'UNASSIGNED':
                return False, (
                    f"Task '{task_id}' is not available for allocation."
                )
            distances = (outbound_distance_m, return_distance_m)
            if any(
                not math.isfinite(value) or value < 0.0
                for value in distances
            ):
                return False, (
                    'Planned distances must be finite and non-negative.'
                )

            self.assignments[task_id] = uav_id
            self.assignment_distances[task_id] = tuple(
                float(value) for value in distances
            )
            self.statuses[task_id] = 'ASSIGNED'
            return True, f"Task '{task_id}' assigned to UAV '{uav_id}'."

    def complete(self, task_id, uav_id):
        with self._lock:
            if (
                self.assignments.get(task_id) != uav_id
                or self.statuses.get(task_id) != 'ASSIGNED'
            ):
                return False
            self.completed.add(task_id)
            self.statuses[task_id] = 'COMPLETED'
            return True

    def fail(self, task_id, reason):
        with self._lock:
            if (
                task_id not in self.tasks
                or self.statuses.get(task_id) in (
                    'COMPLETED', 'FAILED', 'CANCELLED'
                )
            ):
                return False
            self.assignments.pop(task_id, None)
            self.assignment_distances.pop(task_id, None)
            self.failed[task_id] = str(reason)
            self.statuses[task_id] = 'FAILED'
            self.reserved_reasons.pop(task_id, None)
            return True

    def retry_navigation(self, task_id, uav_id, maximum_retries, reason):
        """Release a failed route for reassignment, or fail after its limit."""
        with self._lock:
            if (
                self.assignments.get(task_id) != uav_id
                or self.statuses.get(task_id) != 'ASSIGNED'
            ):
                return 'UNCHANGED'
            retries = self.navigation_retries.get(task_id, 0) + 1
            self.navigation_retries[task_id] = retries
            self.assignments.pop(task_id, None)
            self.assignment_distances.pop(task_id, None)
            if retries <= maximum_retries:
                self.statuses[task_id] = 'UNASSIGNED'
                return 'RETRY'
            self.failed[task_id] = str(reason)
            self.statuses[task_id] = 'FAILED'
            self.reserved_reasons.pop(task_id, None)
            return 'FAILED'

    def release(self, task_id, uav_id):
        with self._lock:
            if (
                self.assignments.get(task_id) != uav_id
                or self.statuses.get(task_id) != 'ASSIGNED'
            ):
                return False
            del self.assignments[task_id]
            self.assignment_distances.pop(task_id, None)
            self.statuses[task_id] = 'UNASSIGNED'
            return True

    def reserve_for_fallback(self, task_id, uav_id, reason):
        """Hold a path-failed task while its peer becomes available."""
        with self._lock:
            if (
                self.assignments.get(task_id) != uav_id
                or self.statuses.get(task_id) != 'ASSIGNED'
            ):
                return False
            self.assignments.pop(task_id, None)
            self.assignment_distances.pop(task_id, None)
            self.statuses[task_id] = 'RESERVED'
            self.reserved_reasons[task_id] = str(reason)
            return True

    def transfer(self, task_id, from_uav, to_uav, outbound=0.0, returning=0.0):
        """Atomically transfer an assigned task after peer path validation."""
        with self._lock:
            if (
                self.assignments.get(task_id) != from_uav
                or self.statuses.get(task_id) != 'ASSIGNED'
            ):
                return False
            return self._set_assignment(
                task_id, to_uav, outbound, returning
            )

    def assign_reserved(self, task_id, uav_id, outbound=0.0, returning=0.0):
        with self._lock:
            if self.statuses.get(task_id) != 'RESERVED':
                return False
            success = self._set_assignment(
                task_id, uav_id, outbound, returning
            )
            if success:
                self.reserved_reasons.pop(task_id, None)
            return success

    def _set_assignment(self, task_id, uav_id, outbound, returning):
        if task_id not in self.tasks:
            return False
        distances = (float(outbound), float(returning))
        if any(not math.isfinite(value) or value < 0.0 for value in distances):
            return False
        self.assignments[task_id] = uav_id
        self.assignment_distances[task_id] = distances
        self.statuses[task_id] = 'ASSIGNED'
        return True

    def release_unfinished_for_uav(self, uav_id):
        with self._lock:
            released = []
            for task_id, assigned_uav in list(self.assignments.items()):
                if assigned_uav != uav_id:
                    continue
                if self.statuses.get(task_id) != 'ASSIGNED':
                    continue
                del self.assignments[task_id]
                self.assignment_distances.pop(task_id, None)
                self.statuses[task_id] = 'UNASSIGNED'
                released.append(task_id)
            return released

    def unfinished_for_uav(self, uav_id):
        with self._lock:
            return [
                task_id for task_id, assigned_uav in self.assignments.items()
                if assigned_uav == uav_id
                and self.statuses.get(task_id) == 'ASSIGNED'
            ]

    def cancel(self, task_id):
        with self._lock:
            if task_id not in self.tasks or task_id in self.completed:
                return False
            self.assignments.pop(task_id, None)
            self.assignment_distances.pop(task_id, None)
            self.cancelled.add(task_id)
            self.statuses[task_id] = 'CANCELLED'
            return True
