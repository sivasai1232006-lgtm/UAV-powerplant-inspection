from dataclasses import asdict, dataclass, field
import math
import time
from uav_allocator.auction_allocator import (AuctionAllocator,UavSnapshot,)
from uav_interfaces.msg import UAVState

@dataclass
class Evaluation:
    name: str
    status: str
    allocation_time_sec: float
    assignments: list
    reasons: dict
    solver: dict
    distance_by_uav_m: dict
    energy_by_uav_percent: dict
    remaining_battery_by_uav_percent: dict
    workload_by_uav_sec: dict
    violations: list
    return_choices: dict = field(default_factory=dict)

def _initial_metrics(uavs):
    return {
        'distance': {u.uav_id: 0.0 for u in uavs},
        'energy': {u.uav_id: 0.0 for u in uavs},
        'workload': {u.uav_id: 0.0 for u in uavs},
        'position': {u.uav_id: tuple(u.position) for u in uavs},
        'battery': {u.uav_id: float(u.battery_percentage) for u in uavs},
    }

def _eligible(allocator, uavs, operating_bounds=None):
    return [
        u for u in uavs
        if allocator._eligible(u)[0]and (operating_bounds is None or u.uav_id not in operating_bounds or operating_bounds[u.uav_id].contains(u.position))]

def _objective_distance_bound(allocator, tasks, uavs, homes):
    points = [tuple(uav.position) for uav in uavs]
    points.extend(tuple(task.target_position) for task in tasks)
    points.extend(tuple(position) for position in homes.values() if position is not None)
    if allocator.recovery_position is not None:
        points.append(tuple(allocator.recovery_position))
    valid_distances = []
    for start in points:
        for goal in points:
            distance = allocator.path_cost.estimate_distance(start, goal)
            if (distance is not None and math.isfinite(distance)and distance >= 0.0):
                valid_distances.append(distance)
    maximum_leg = max(valid_distances, default=0.0)
    return maximum_leg * (len(tasks) + len(uavs))


def _plan_metrics(allocator, tasks, uavs, homes, assignments, max_tasks_per_uav=None,operating_bounds=None,):
    task_by_id = {task.task_id: task for task in tasks}
    uav_by_id = {uav.uav_id: uav for uav in uavs}
    capacity = max_tasks_per_uav or max(len(tasks), 1)
    assigned_ids = set()
    route_rows = {uav.uav_id: [] for uav in uavs}
    violations = []
    normalized = []

    for source in assignments:
        row = dict(source)
        task_id = row.get('task_id')
        uav_id = row.get('uav_id')
        if task_id not in task_by_id:
            violations.append(f'unknown task: {task_id}')
            continue
        if task_id in assigned_ids:
            violations.append(f'duplicate task assignment: {task_id}')
            continue
        if uav_id not in uav_by_id:
            violations.append(f'unknown UAV for task {task_id}: {uav_id}')
            continue
        if not allocator._eligible(uav_by_id[uav_id])[0]:
            violations.append(f'{uav_id} is unavailable for task {task_id}')
            continue
        bounds = (operating_bounds.get(uav_id)if operating_bounds is not None else None)
        if bounds is not None and not bounds.contains(task_by_id[task_id].target_position):
            violations.append(f'{uav_id} task goal for {task_id} is outside operating bounds')
            continue
        row['priority'] = int(task_by_id[task_id].priority)
        row['status'] = 'assigned_not_executed'
        normalized.append(row)
        route_rows[uav_id].append(row)
        assigned_ids.add(task_id)

    for uav_id, route in route_rows.items():
        if len(route) > capacity:
            violations.append(f'{uav_id} exceeds max_tasks_per_uav ({len(route)} > {capacity})')

    for first_index, first in enumerate(normalized):
        for second in normalized[first_index + 1:]:
            if first['uav_id'] == second['uav_id']:
                continue
            a = task_by_id[first['task_id']].target_position
            b = task_by_id[second['task_id']].target_position
            if math.dist(a, b) < allocator.minimum_separation_m:
                violations.append('minimum separation violated by tasks 'f"{first['task_id']} and {second['task_id']}")

    distance_by_uav = {u.uav_id: 0.0 for u in uavs}
    energy_by_uav = {u.uav_id: 0.0 for u in uavs}
    remaining_by_uav = {u.uav_id: float(u.battery_percentage) for u in uavs}
    workload_by_uav = {u.uav_id: 0.0 for u in uavs}
    inspection_by_uav = {u.uav_id: 0.0 for u in uavs}
    return_choices = {}
    enriched_rows = {row['task_id']: row for row in normalized}
    min_reserve = allocator.battery_feasibility.minimum_reserve_percentage
    consumption_per_meter = allocator.battery_feasibility.consumption_per_meter
    inspection_per_minute = (allocator.battery_feasibility.inspection_consumption_per_minute)
    landing_energy = float(getattr(allocator, 'landing_energy_percentage', 0.0))
    speed = allocator.workload_estimator.nominal_speed_mps

    for uav in uavs:
        route = route_rows[uav.uav_id]
        if not route:
            continue
        position = tuple(uav.position)
        outbound_distance = 0.0
        inspection_duration = 0.0
        for row in route:
            task = task_by_id[row['task_id']]
            leg = allocator.path_cost.estimate_distance(
                position, task.target_position,
                uav.yaw if position == tuple(uav.position) else 0.0,
                0.0,
            )
            if leg is None or not math.isfinite(leg) or leg < 0.0:
                violations.append(f'route distance unavailable for {uav.uav_id}/{task.task_id}')
                leg = 0.0
            outbound_distance += leg
            inspection_duration += float(task.estimated_duration_sec)
            row['estimated_distance_m'] = float(leg)
            row['estimated_energy_percent'] = (
                float(leg) * consumption_per_meter
                + float(task.estimated_duration_sec) / 60.0
                * inspection_per_minute
            )
            row['estimated_workload_sec'] = (leg / speed + float(task.estimated_duration_sec))
            position = tuple(task.target_position)

        service_energy = (inspection_duration / 60.0 * inspection_per_minute)
        base = homes.get(uav.uav_id)
        bounds = (operating_bounds.get(uav.uav_id)if operating_bounds is not None else None)
        base_in_bounds = (bounds is None or bounds.contains(base or ()))
        base_distance = (allocator.path_cost.estimate_distance(position, base)if base is not None and base_in_bounds else None)
        base_remaining = None
        if base_distance is not None and math.isfinite(base_distance):
            base_energy = (
                (outbound_distance + base_distance) * consumption_per_meter
                + service_energy + landing_energy
            )
            base_remaining = float(uav.battery_percentage) - base_energy

        selected_name = 'takeoff_position'
        selected_distance = base_distance
        if base_remaining is None or base_remaining < min_reserve:
            recovery = allocator.recovery_position
            recovery_in_bounds = (recovery is not None and (bounds is None or bounds.contains(recovery)))
            recovery_distance = (allocator.path_cost.estimate_distance(position, recovery)if recovery_in_bounds else None)
            recovery_remaining = None
            if recovery_distance is not None and math.isfinite(recovery_distance):
                recovery_energy = (
                    (outbound_distance + recovery_distance)
                    * consumption_per_meter
                    + service_energy + landing_energy
                )
                recovery_remaining = (float(uav.battery_percentage) - recovery_energy)
            required_reserve = max(min_reserve,float(getattr(allocator,'completion_return_emergency_threshold_percentage',20.0,)),
            )
            if (recovery_remaining is not None and recovery_remaining >= required_reserve):
                selected_name = 'UGV'
                selected_distance = recovery_distance
            else:
                violations.append(f'battery: {uav.uav_id} cannot safely return to base ''or the configured UGV')
                # Retain the preferred base estimate for diagnostics only.
                selected_name = 'takeoff_position'
                selected_distance = base_distance or 0.0
        return_position = (allocator.recovery_position if selected_name == 'UGV'else homes.get(uav.uav_id))
        if bounds is not None and (not bounds.contains(uav.position)or not bounds.contains(return_position or ())):
            violations.append(f'{uav.uav_id} route start or return goal is outside ''operating bounds')

        if selected_distance is None or not math.isfinite(selected_distance):
            violations.append(f'return distance unavailable for {uav.uav_id}')
            selected_distance = 0.0
        full_distance = outbound_distance + selected_distance
        total_energy = full_distance * consumption_per_meter + service_energy + landing_energy
        final_battery = float(uav.battery_percentage) - total_energy
        if final_battery < min_reserve - 1e-8:
            violations.append(
                f'battery reserve violated by {uav.uav_id}: '
                f'{final_battery:.3f}% < {min_reserve:.3f}%'
            )

        if route:
            final_task_row = enriched_rows[route[-1]['task_id']]
            final_task_row['estimated_distance_m'] += float(selected_distance)
            final_task_row['estimated_energy_percent'] += (float(selected_distance) * consumption_per_meter)
            final_task_row['estimated_workload_sec'] += (float(selected_distance) / speed)

        workload = sum(float(row.get('estimated_workload_sec', 0.0)) for row in route)
        if workload > allocator.maximum_workload_sec + 1e-8:
            violations.append(
                f'workload limit violated by {uav.uav_id}: '
                f'{workload:.3f}s > {allocator.maximum_workload_sec:.3f}s'
            )

        distance_by_uav[uav.uav_id] = full_distance
        energy_by_uav[uav.uav_id] = total_energy
        remaining_by_uav[uav.uav_id] = final_battery
        workload_by_uav[uav.uav_id] = workload
        inspection_by_uav[uav.uav_id] = inspection_duration
        return_choices[uav.uav_id] = selected_name
    return {
        'distance': distance_by_uav,
        'energy': energy_by_uav,
        'remaining_battery': remaining_by_uav,
        'workload': workload_by_uav,
        'inspection_time': inspection_by_uav,
        'makespan': max(workload_by_uav.values(), default=0.0),
        'total_distance': sum(distance_by_uav.values()),
        'total_energy': sum(energy_by_uav.values()),
        'return_choices': return_choices,
        'assignments': normalized,
        'violations': violations,
        'assigned_task_ids': assigned_ids,
        'uavs': uavs,
        'homes': homes,
    }


def _make_evaluation(allocator, name, started, tasks, uavs, homes, assignments, reasons,solver, max_tasks_per_uav=None, status='completed',allocation_time_sec=None, operating_bounds=None):
    metrics = _plan_metrics(allocator, tasks, uavs, homes, assignments, max_tasks_per_uav,operating_bounds,)
    return Evaluation(
        name=name,
        status=status,
        allocation_time_sec=(time.perf_counter() - started if allocation_time_sec is None else allocation_time_sec),
        assignments=metrics['assignments'],
        reasons=reasons,
        solver=solver,
        distance_by_uav_m=metrics['distance'],
        energy_by_uav_percent=metrics['energy'],
        remaining_battery_by_uav_percent=metrics['remaining_battery'],
        workload_by_uav_sec=metrics['workload'],
        violations=metrics['violations'],
        return_choices=metrics['return_choices'],
    )


def run_existing(allocator: AuctionAllocator, tasks, uavs, homes, max_tasks_per_uav=None,operating_bounds=None,):
    started = time.perf_counter()
    remaining = list(tasks)
    task_by_id = {task.task_id: task for task in tasks}
    states = {u.uav_id: u for u in uavs}
    task_counts = {u.uav_id: 0 for u in uavs}
    capacity = max_tasks_per_uav or max(len(tasks), 1)
    assignments, reasons = [], {}

    for _ in range(len(tasks) + 1):
        if not remaining:
            break
        allowed_uav_ids_by_task = None
        if operating_bounds is not None:
            allowed_uav_ids_by_task = {
                task.task_id: {
                    uav_id for uav_id, bounds in operating_bounds.items()
                    if (
                        uav_id in states
                        and bounds.contains(states[uav_id].position)
                        and bounds.contains(task.target_position)
                        and (bounds.contains(homes.get(uav_id, ()))
                            or (allocator.recovery_position is not None and bounds.contains(allocator.recovery_position)
                            )
                        )
                    )
                }
                for task in remaining
            }
        decisions = allocator.allocate(
            remaining,
            list(states.values()),
            homes,
            allowed_uav_ids_by_task=allowed_uav_ids_by_task,
        )
        accepted = [decision for decision in decisions if decision.uav_id]
        for decision in decisions:
            if not decision.uav_id:
                reasons[decision.task_id] = decision.reason
        if not accepted:
            break
        for decision in accepted:
            task = task_by_id[decision.task_id]
            uav = states[decision.uav_id]
            travel_distance = allocator.path_cost.estimate_distance(uav.position, task.target_position, uav.yaw, 0.0)
            energy_used = (
                travel_distance* allocator.battery_feasibility.consumption_per_meter+ task.estimated_duration_sec / 60.0* allocator.battery_feasibility.inspection_consumption_per_minute)
            assignments.append({
                'task_id': task.task_id,
                'uav_id': uav.uav_id,
                'priority': task.priority,
                'bid': decision.bid,
            })
            task_counts[uav.uav_id] += 1
            states[uav.uav_id] = UavSnapshot(
                uav_id=uav.uav_id,
                position=tuple(task.target_position),
                battery_percentage=float(uav.battery_percentage - energy_used),
                connected=uav.connected,
                operational=uav.operational,
                mission_status=(UAVState.ASSIGNED if task_counts[uav.uav_id] >= capacity else UAVState.IDLE),
                state_age_sec=0.0,
                yaw=uav.yaw,
            )
            reasons.pop(task.task_id, None)
            remaining = [candidate for candidate in remaining if candidate.task_id != task.task_id]

    for task in remaining:
        reasons.setdefault(task.task_id, 'existing allocator left task unassigned')
    allocation_time_sec = time.perf_counter() - started
    return _make_evaluation(
        allocator, 'existing_algorithm', started, tasks, uavs, homes,
        assignments, reasons,
        solver={'name': 'project AuctionAllocator', 'status': 'not_applicable'},
        max_tasks_per_uav=max_tasks_per_uav,
        allocation_time_sec=allocation_time_sec,
        operating_bounds=operating_bounds,
    )


def run_greedy(allocator: AuctionAllocator, tasks, uavs, homes, max_tasks_per_uav=None,operating_bounds=None,):
    started = time.perf_counter()
    capacity = max_tasks_per_uav or max(len(tasks), 1)
    eligible = _eligible(allocator, uavs, operating_bounds)
    assignments, reasons = [], {}
    counts = {u.uav_id: 0 for u in uavs}

    ordered_tasks = sorted(tasks, key=lambda task: (-int(task.priority), str(task.task_id)))
    current_metrics = _plan_metrics(allocator, tasks, uavs, homes, assignments, capacity,operating_bounds,)

    for task in ordered_tasks:
        choices = []
        rejected = []
        for uav in eligible:
            uav_id = uav.uav_id
            if counts[uav_id] >= capacity:
                continue
            proposal = assignments + [{
                'task_id': task.task_id,
                'uav_id': uav_id,
                'priority': task.priority,
                'bid': 0.0,
            }]
            candidate_metrics = _plan_metrics(allocator, tasks, uavs, homes, proposal, capacity,operating_bounds,)
            if candidate_metrics['violations']:
                rejected.append(candidate_metrics['violations'][0])
                continue

            projected_makespan = candidate_metrics['makespan']
            # Balance mission durations without forcing equal task counts.
            projected_workloads = candidate_metrics['workload']
            workload_imbalance = sum(float(workload_sec) ** 2for workload_sec in projected_workloads.values())
            # Distance and energy remain important secondary criteria.
            choices.append((projected_makespan,workload_imbalance, candidate_metrics['total_distance'],candidate_metrics['total_energy'],str(uav_id),uav_id,
                proposal,candidate_metrics,))

        if not choices:
            reasons[task.task_id] = (rejected[0] if rejected else 'no available UAV satisfies the shared route constraints')
            continue
        (_projected_makespan, _workload_imbalance, _total_distance,
         _total_energy, _uav_sort_key, selected_uav_id,assignments, current_metrics,) = min(choices, key=lambda choice: choice[:5])
        counts[selected_uav_id] += 1

    allocation_time_sec = time.perf_counter() - started
    return _make_evaluation(
        allocator, 'greedy', started, tasks, uavs, homes, assignments,
        reasons,
        solver={
            'name': 'priority-ordered mission-time-balanced greedy',
            'status': 'not_applicable',
        },
        max_tasks_per_uav=max_tasks_per_uav,
        allocation_time_sec=allocation_time_sec,
        operating_bounds=operating_bounds,
    )


def run_milp(allocator: AuctionAllocator, tasks, uavs, homes,time_limit_sec=10.0, max_tasks_per_uav=None,operating_bounds=None,):
    started = time.perf_counter()
    try:
        import pulp
    except ImportError as error:
        return _failed('milp', started, tasks, uavs, 'solver_unavailable', str(error))

    eligible = _eligible(allocator, uavs, operating_bounds)
    task_count = len(tasks)
    if not tasks:
        return _make_evaluation(
            allocator, 'milp', started, tasks, uavs, homes, [], {},
            {'name': 'PuLP MILP', 'incumbent_available': True},
            max_tasks_per_uav,status='completed',operating_bounds=operating_bounds,)
    if not eligible:
        return _failed('milp', started, tasks, uavs, 'infeasible','no eligible UAV is available',)

    capacity = max_tasks_per_uav or task_count
    uav_index = {uav.uav_id: index for index, uav in enumerate(eligible)}
    n, k = task_count, len(eligible)
    recovery = allocator.recovery_position
    has_ugv = recovery is not None
    speed = allocator.workload_estimator.nominal_speed_mps
    rate = allocator.battery_feasibility.consumption_per_meter
    inspection_rate = (allocator.battery_feasibility.inspection_consumption_per_minute)
    min_reserve = allocator.battery_feasibility.minimum_reserve_percentage
    threshold = float(getattr(allocator, 'completion_return_emergency_threshold_percentage', 5.0))
    landing_energy = float(getattr(allocator, 'landing_energy_percentage', 0.0))

    start_distance, pair_distance, base_return_distance = {}, {}, {}
    ugv_return_distance = {}
    for uav in eligible:
        ui = uav_index[uav.uav_id]
        for i, task in enumerate(tasks):
            start_distance[(ui, i)] = allocator.path_cost.estimate_distance(uav.position, task.target_position, uav.yaw, 0.0)
            base_return_distance[(ui, i)] = allocator.path_cost.estimate_distance(task.target_position, homes[uav.uav_id])
            if has_ugv:
                ugv_return_distance[(ui, i)] = allocator.path_cost.estimate_distance(task.target_position, recovery)
            for j, other in enumerate(tasks):
                if i != j:
                    pair_distance[(ui, i, j)] = allocator.path_cost.estimate_distance(task.target_position, other.target_position)
    base_valid = {
        uav.uav_id: (
            operating_bounds is None
            or uav.uav_id not in operating_bounds
            or operating_bounds[uav.uav_id].contains(homes[uav.uav_id])
        )
        for uav in eligible
    }
    ugv_valid = {
        uav.uav_id: (
            recovery is not None
            and (
                operating_bounds is None
                or uav.uav_id not in operating_bounds
                or operating_bounds[uav.uav_id].contains(recovery)
            )
        )
        for uav in eligible
    }

    if any(
        value is None or not math.isfinite(value) or value < 0.0
        for mapping in (start_distance, pair_distance, base_return_distance,ugv_return_distance,)
        for value in mapping.values()
    ):
        return _failed('milp', started, tasks, uavs, 'failed','route distance matrix contains invalid values',)

    problem = pulp.LpProblem('uav_route_assignment', pulp.LpMinimize)
    assigned = {
        (i, ui): pulp.LpVariable(f'x_{i}_{ui}', cat=pulp.LpBinary)
        for i in range(n) for ui in range(k)
    }
    unassigned = {
        i: pulp.LpVariable(f'unassigned_{i}', cat=pulp.LpBinary)
        for i in range(n)
    }
    active = {
        ui: pulp.LpVariable(f'active_{ui}', cat=pulp.LpBinary)
        for ui in range(k)
    }
    starts = {
        (i, ui): pulp.LpVariable(f'start_{i}_{ui}', cat=pulp.LpBinary)
        for i in range(n) for ui in range(k)
    }
    ends_base = {
        (i, ui): pulp.LpVariable(f'end_base_{i}_{ui}', cat=pulp.LpBinary)
        for i in range(n) for ui in range(k)
    }
    ends_ugv = ({
        (i, ui): pulp.LpVariable(f'end_ugv_{i}_{ui}', cat=pulp.LpBinary)
        for i in range(n) for ui in range(k)
    } if has_ugv else {})
    arcs = {
        (i, j, ui): pulp.LpVariable(f'arc_{i}_{j}_{ui}', cat=pulp.LpBinary)
        for i in range(n) for j in range(n) if i != j
        for ui in range(k)
    }
    order = {
        (i, ui): pulp.LpVariable(f'order_{i}_{ui}', lowBound=0, upBound=n, cat=pulp.LpContinuous)
        for i in range(n) for ui in range(k)
    }
    makespan = pulp.LpVariable('estimated_mission_makespan_sec', lowBound=0, upBound=allocator.maximum_workload_sec,)

    max_priority = allocator.cost_function.maximum_priority
    min_priority = allocator.cost_function.minimum_priority
    priority_range = max_priority - min_priority
    priority_scale = allocator.maximum_workload_sec + 1.0
    coverage_scale = (n * priority_range * priority_scale + allocator.maximum_workload_sec + 3.0)
    route_distance_expressions = []
    route_time_expressions = []
    route_energy_expressions = []

    for i in range(n):
        problem += (
            pulp.lpSum(assigned[(i, ui)] for ui in range(k)) + unassigned[i] == 1,f'assign_or_unassign_{i}',)
        for ui, uav in enumerate(eligible):
            bounds = (operating_bounds.get(uav.uav_id) if operating_bounds is not None else None)
            if (bounds is not None and not bounds.contains(tasks[i].target_position)):
                problem += assigned[(i, ui)] == 0
    for ui, uav in enumerate(eligible):
        problem += pulp.lpSum(assigned[(i, ui)] for i in range(n)) <= capacity
        problem += pulp.lpSum(starts[(i, ui)] for i in range(n)) == active[ui]
        problem += (
            pulp.lpSum(ends_base[(i, ui)] + (ends_ugv[(i, ui)] if has_ugv else 0) for i in range(n)) == active[ui]        )
        if not base_valid[uav.uav_id]:
            for i in range(n):
                problem += ends_base[(i, ui)] == 0
        if has_ugv and not ugv_valid[uav.uav_id]:
            for i in range(n):
                problem += ends_ugv[(i, ui)] == 0

        for i in range(n):
            incoming = starts[(i, ui)] + pulp.lpSum(arcs[(j, i, ui)] for j in range(n) if j != i)
            outgoing = (
                ends_base[(i, ui)]
                + (ends_ugv[(i, ui)] if has_ugv else 0)
                + pulp.lpSum(arcs[(i, j, ui)] for j in range(n) if j != i)
            )
            problem += incoming == assigned[(i, ui)]
            problem += outgoing == assigned[(i, ui)]
            problem += order[(i, ui)] >= assigned[(i, ui)]
            problem += order[(i, ui)] <= n * assigned[(i, ui)]
            for j in range(n):
                if i == j:
                    continue
                problem += ( order[(j, ui)] >= order[(i, ui)] + 1- n * (1 - arcs[(i, j, ui)]))

        outbound_distance = (
            pulp.lpSum(
                start_distance[(ui, i)] * starts[(i, ui)]
                for i in range(n)
            )
            + pulp.lpSum(
                pair_distance[(ui, i, j)] * arcs[(i, j, ui)]
                for i in range(n) for j in range(n) if i != j
            )
        )
        service_duration = pulp.lpSum(
            float(task.estimated_duration_sec) * assigned[(i, ui)]
            for i, task in enumerate(tasks)
        )
        service_energy = service_duration / 60.0 * inspection_rate
        base_distance = outbound_distance + pulp.lpSum(
            base_return_distance[(ui, i)]
            * (ends_base[(i, ui)] + (ends_ugv[(i, ui)] if has_ugv else 0))
            for i in range(n)
        )
        selected_route_distance = outbound_distance + pulp.lpSum(
            base_return_distance[(ui, i)] * ends_base[(i, ui)]
            + (ugv_return_distance[(ui, i)] * ends_ugv[(i, ui)]
               if has_ugv else 0.0)
            for i in range(n)
        )
        selected_route_energy = (selected_route_distance * rate + service_energy+ landing_energy * active[ui])
        route_time = outbound_distance / speed + service_duration + pulp.lpSum(
            base_return_distance[(ui, i)] / speed * ends_base[(i, ui)]
            + (ugv_return_distance[(ui, i)] / speed * ends_ugv[(i, ui)]
               if has_ugv else 0.0)
            for i in range(n)
        )
        problem += selected_route_energy <= max(
            0.0, float(uav.battery_percentage) - min_reserve
        )
        problem += route_time <= allocator.maximum_workload_sec
        problem += makespan >= route_time
        route_distance_expressions.append(selected_route_distance)
        route_time_expressions.append(route_time)
        route_energy_expressions.append(selected_route_energy)

        base_energy_if_used = (base_distance * rate + service_energy + landing_energy * active[ui])
        base_remaining = float(uav.battery_percentage) - base_energy_if_used
        base_active = pulp.lpSum(ends_base[(i, ui)] for i in range(n))
        ugv_active = (pulp.lpSum(ends_ugv[(i, ui)] for i in range(n))if has_ugv else 0)
        max_energy = (
            max(start_distance.values(), default=0.0)
            + max(pair_distance.values(), default=0.0) * max(n - 1, 0)
            + max(base_return_distance.values(), default=0.0)
            + max(ugv_return_distance.values(), default=0.0)) * rate + sum(max(0.0, float(task.estimated_duration_sec)) for task in tasks) / 60.0 * inspection_rate + landing_energy + 100.0
        big_m = max(1000.0, max_energy)
        if base_valid[uav.uav_id]:
            problem += (base_remaining >= min_reserve - 1e-6- big_m * (1 - base_active))
        if has_ugv and ugv_valid[uav.uav_id]:
            if base_valid[uav.uav_id]:
                problem += (base_remaining <= min_reserve - 1e-6+ big_m * (1 - ugv_active))
            problem += selected_route_energy <= max(0.0,float(uav.battery_percentage)- max(min_reserve, threshold),) + big_m * (1 - ugv_active)

    # Apply the project's cross-UAV minimum-separation constraint.
    for i, first in enumerate(tasks):
        for j in range(i + 1, n):
            second = tasks[j]
            if math.dist(first.target_position, second.target_position) >= allocator.minimum_separation_m:
                continue
            for ua in range(k):
                for ub in range(k):
                    if ua == ub:
                        continue
                    problem += assigned[(i, ua)] + assigned[(j, ub)] <= 1

    distance_bound = _objective_distance_bound(allocator, tasks, uavs, homes)
    energy_bound = (distance_bound * rate + sum(float(task.estimated_duration_sec) for task in tasks) / 60.0 * inspection_rate + len(uavs) * landing_energy + 1.0)
    total_priority_penalty = pulp.lpSum(
        (max_priority - min(max_priority, max(min_priority, int(task.priority)))
        ) * assigned[(i, ui)] for i, task in enumerate(tasks) for ui in range(k))
    problem += (coverage_scale * pulp.lpSum(unassigned.values()) + priority_scale * total_priority_penalty + makespan
        + 0.001 * pulp.lpSum(route_distance_expressions) / (distance_bound + 1.0) + 0.001 * pulp.lpSum(route_energy_expressions) / (energy_bound + 1.0))

    solver = None
    solver_names = ('COIN_CMD', 'PULP_CBC_CMD', 'HiGHS_CMD', 'HiGHS','GLPK_CMD', 'CPLEX_CMD', 'GUROBI_CMD', 'GUROBI',)
    try:
        available = set(pulp.listSolvers(onlyAvailable=True))
        for solver_name in solver_names:
            if solver_name not in available:
                continue
            solver_class = getattr(pulp, solver_name, None)
            if solver_class is None:
                continue
            try:
                solver = solver_class(msg=False, timeLimit=float(time_limit_sec))
            except TypeError:
                solver = solver_class(msg=False)
            if solver.available():
                break
            solver = None
        if solver is None:
            return _failed('milp', started, tasks, uavs, 'solver_unavailable', 'PuLP is installed, but no supported solver backend is available',)
        problem.solve(solver)
    except Exception as error:
        return _failed('milp', started, tasks, uavs, 'solver_error', str(error))

    solver_status = pulp.LpStatus.get(problem.status, str(problem.status))
    all_variables = problem.variables()
    has_values = all(variable.value() is not None for variable in all_variables)
    integral = has_values and all(
        variable.cat != pulp.LpInteger
        or abs(variable.value() - round(variable.value())) <= 1e-5
        for variable in all_variables
    )
    feasible_incumbent = (integral and (not hasattr(problem, 'valid') or problem.valid(1e-5)))
    if not feasible_incumbent:
        return _failed('milp', started, tasks, uavs, solver_status.lower(),'solver returned no feasible incumbent',)

    route_order = []
    reasons = {}
    for i, task in enumerate(tasks):
        chosen = None
        for ui, uav in enumerate(eligible):
            assigned_value = assigned[(i, ui)].value()
            if assigned_value is not None and assigned_value > 0.5:
                chosen = uav
                break
        if chosen is None:
            reasons[task.task_id] = 'MILP left task unassigned under constraints'
            continue
        ui = uav_index[chosen.uav_id]
        order_value = order[(i, ui)].value()
        if order_value is None:
            reasons[task.task_id] = ('MILP returned no route order for an assigned task')
            continue
        route_order.append((float(order_value), task.task_id, chosen.uav_id,task.priority,))
    route_order.sort(key=lambda item: (item[2], item[0], item[1]))
    assignments = [{'task_id': task_id, 'uav_id': uav_id, 'priority': priority,'bid': 0.0,
    } for _order, task_id, uav_id, priority in route_order]

    solver_info = {
        'name': f'{solver.__class__.__name__} via PuLP',
        'time_limit_sec': float(time_limit_sec),
        'incumbent_available': feasible_incumbent,
    }
    allocation_time_sec = time.perf_counter() - started
    return _make_evaluation(
        allocator, 'milp', started, tasks, uavs, homes, assignments, reasons,
        solver_info, max_tasks_per_uav,
        status='completed',
        allocation_time_sec=allocation_time_sec,
        operating_bounds=operating_bounds,
    )


def _failed(name, started, tasks, uavs, status, detail):
    return Evaluation(
        name=name,
        status=status,
        allocation_time_sec=time.perf_counter() - started,
        assignments=[],
        reasons={task.task_id: detail for task in tasks},
        solver={'name': 'PuLP MILP solver','detail': detail,'incumbent_available': False,},
        distance_by_uav_m={u.uav_id: 0.0 for u in uavs},
        energy_by_uav_percent={u.uav_id: 0.0 for u in uavs},
        remaining_battery_by_uav_percent={u.uav_id: u.battery_percentage for u in uavs},
        workload_by_uav_sec={u.uav_id: 0.0 for u in uavs},
        violations=[],
    )


def evaluation_dict(evaluation):
    return asdict(evaluation)