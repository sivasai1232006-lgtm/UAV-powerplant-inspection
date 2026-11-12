"""Command line entry point for isolated, non-actuating allocator benchmarks."""

import argparse
import csv
from datetime import datetime
import json
import math
import os
from pathlib import Path
import sys

import yaml

from uav_allocator.auction_allocator import (
    InspectionTask,
    UavSnapshot,
    allocator_from_config,
)
from uav_interfaces.msg import UAVState
from uav_mission.operating_bounds import OperatingBounds
from uav_mission.task_manager import tasks_from_mission

from benchmarks.algorithms import (
    Evaluation,
    run_existing,
    run_greedy,
    run_milp,
)


def _package_share(package):
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory(package))


def _default_config(package, file_name):
    try:
        return _package_share(package) / 'config' / file_name
    except Exception:
        root = Path(__file__).resolve().parents[2]
        return root / package / 'config' / file_name


def _source_workspace_root():
    """Find the source tree even when this entry point is installed."""
    module_root = Path(__file__).resolve().parents[2]
    if (module_root / 'uav_mission').is_dir():
        return module_root
    prefixes = os.environ.get('COLCON_PREFIX_PATH', '').split(os.pathsep)
    prefixes += os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep)
    for prefix in prefixes:
        if not prefix:
            continue
        candidate = Path(prefix).expanduser().resolve().parent / 'src'
        if (candidate / 'benchmarks').is_dir() and (candidate / 'uav_mission').is_dir():
            return candidate
    current = Path.cwd().resolve()
    if (current / 'benchmarks').is_dir() and (current / 'uav_mission').is_dir():
        return current
    return module_root


def _read_yaml(path):
    with Path(path).open(encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError(f'Expected YAML mapping in {path}')
    return data


def _normalise_allocator(data):
    if 'allocator' in data:
        return data['allocator'].get('ros__parameters', data['allocator'])
    for value in data.values():
        if isinstance(value, dict) and 'ros__parameters' in value:
            return value['ros__parameters']
    return data


def _make_inputs(mission_data, infeasible_tasks=None):
    mission = mission_data['mission']
    landing_altitude = float(
        mission.get('recovery', {}).get('landing_altitude_m', 0.2)
    )
    mission_infeasible_tasks = {}
    task_messages = tasks_from_mission(
        mission_data, infeasible_tasks=mission_infeasible_tasks
    )
    if infeasible_tasks is not None:
        infeasible_tasks.update(mission_infeasible_tasks)
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
        for task in task_messages
        if task.task_id not in mission_infeasible_tasks
    ]
    uavs = []
    homes = {}
    for config in mission['uavs']:
        uav_id = str(config['id'])
        start = tuple(float(value) for value in config['start_position'])
        configured_home = tuple(
            float(value) for value in config['home_position']
        )
        if len(start) != 3 or len(configured_home) != 3:
            raise ValueError(f'{uav_id} positions must have three coordinates')
        # Completion-return policy uses the UAV's original takeoff point.
        homes[uav_id] = (start[0], start[1], landing_altitude)
        uavs.append(UavSnapshot(
            uav_id=uav_id,
            position=start,
            battery_percentage=float(config['battery_remaining']),
            connected=True,
            operational=True,
            mission_status=UAVState.IDLE,
            state_age_sec=0.0,
        ))
    return tasks, uavs, homes


def _make_operating_bounds(mission):
    region = mission.get('navigation', {}).get('operating_region')
    if region is None:
        return None
    return {
        str(config['id']): OperatingBounds.from_initial_position(
            config['start_position'], region
        )
        for config in mission['uavs']
    }


def _assignment_rows(evaluation, tasks):
    by_id = {row['task_id']: row for row in evaluation.assignments}
    for task in tasks:
        row = by_id.get(task.task_id)
        if row:
            yield row
        else:
            yield {
                'task_id': task.task_id, 'uav_id': '',
                'priority': task.priority, 'estimated_distance_m': '',
                'estimated_energy_percent': '',
                'estimated_workload_sec': '', 'bid': '',
                'status': 'unassigned',
                'reason': evaluation.reasons.get(task.task_id, ''),
            }


def _aggregate_metrics(evaluation, task_count, run_info):
    assigned_count = len(evaluation.assignments)
    infeasible_tasks = run_info.get('infeasible_tasks', {})
    total_task_count = task_count + len(infeasible_tasks)
    unassigned_count = total_task_count - assigned_count
    estimated_completion_by_uav = {
        uav_id: float(workload_sec)
        for uav_id, workload_sec in evaluation.workload_by_uav_sec.items()
    }
    assigned_tasks_by_uav = {
        uav_id: [
            row['task_id'] for row in evaluation.assignments
            if row['uav_id'] == uav_id
        ]
        for uav_id in evaluation.distance_by_uav_m
    }
    task_ids = list(run_info.get('task_ids', ()))
    if not task_ids:
        task_ids = [
            row['task_id'] for row in evaluation.assignments
        ] + list(infeasible_tasks)
    assigned_task_ids = {
        row['task_id'] for row in evaluation.assignments
    }
    unassigned_reasons = {
        task_id: infeasible_tasks.get(
            task_id,
            evaluation.reasons.get(
                task_id, 'No feasible assignment was produced.'
            ),
        )
        for task_id in task_ids
        if task_id not in assigned_task_ids
    }
    assignment_by_task = {
        row['task_id']: row for row in evaluation.assignments
    }
    task_assignment_results = {}
    for task_id in task_ids:
        assignment = assignment_by_task.get(task_id)
        if assignment is not None:
            task_assignment_results[task_id] = {
                'status': 'assigned_not_executed',
                'uav_id': assignment['uav_id'],
                'priority': int(assignment['priority']),
                'estimated_distance_m': assignment.get(
                    'estimated_distance_m'
                ),
                'estimated_workload_sec': assignment.get(
                    'estimated_workload_sec'
                ),
                'estimated_energy_percent': assignment.get(
                    'estimated_energy_percent'
                ),
            }
        else:
            task_assignment_results[task_id] = {
                'status': (
                    'infeasible' if task_id in infeasible_tasks
                    else 'unassigned'
                ),
                'uav_id': None,
                'reason': unassigned_reasons.get(task_id, ''),
            }
    inspection_duration_by_task = run_info.get(
        'inspection_duration_sec_by_task', {}
    )
    inspection_time_by_uav = {
        uav_id: float(sum(
            inspection_duration_by_task.get(task_id, 0.0)
            for task_id in assigned_tasks
        ))
        for uav_id, assigned_tasks in assigned_tasks_by_uav.items()
    }
    recovery_position = run_info.get('recovery_position')
    return_destination_by_uav = {}
    for uav_id, choice in evaluation.return_choices.items():
        initial = run_info.get('uav_initial_conditions', {}).get(uav_id, {})
        if choice == 'UGV':
            return_destination_by_uav[uav_id] = recovery_position
        else:
            return_destination_by_uav[uav_id] = initial.get(
                'normal_landing_target'
            )
    # Total mission time is the sum of all UAV mission durations. Each
    # per-UAV duration includes takeoff-to-task travel, inspection time, and
    # the return leg to its selected landing destination.
    estimated_completion_sec = sum(estimated_completion_by_uav.values())
    violations = evaluation.violations
    status = evaluation.status
    model_valid = status == 'completed' and not violations
    feasible = model_valid and unassigned_count == 0
    if not model_valid:
        feasibility_status = (
            'SOLVER_UNAVAILABLE' if status == 'solver_unavailable'
            else 'INFEASIBLE' if status in ('failed', 'infeasible')
            else 'UNKNOWN'
        )
    elif unassigned_count and assigned_count:
        feasibility_status = 'PARTIAL_FEASIBLE'
    elif unassigned_count:
        feasibility_status = 'INFEASIBLE'
    else:
        feasibility_status = 'FEASIBLE'
    battery_violations = sum(
        'battery' in str(item).lower() for item in violations
    )
    safety_violations = sum(
        any(word in str(item).lower() for word in (
            'safety', 'separation', 'reserve', 'collision',
        ))
        for item in violations
    )
    return {
        'algorithm_name': evaluation.name,
        'status': status,
        'number_of_tasks': int(total_task_count),
        'number_of_waypoints': int(run_info['waypoint_count']),
        'allocation_computation_time_sec': float(
            evaluation.allocation_time_sec
        ),
        'estimated_total_mission_time_sec': estimated_completion_sec,
        'estimated_mission_time_by_uav_sec': (
            estimated_completion_by_uav
        ),
        'inspection_time_by_uav_sec': inspection_time_by_uav,
        'assigned_tasks_by_uav': assigned_tasks_by_uav,
        'return_destination_by_uav': return_destination_by_uav,
        'estimated_total_distance_m': float(
            sum(evaluation.distance_by_uav_m.values())
        ),
        'estimated_distance_by_uav_m': {
            key: float(value)
            for key, value in evaluation.distance_by_uav_m.items()
        },
        'estimated_energy_consumption_percent': float(
            sum(evaluation.energy_by_uav_percent.values())
        ),
        'estimated_energy_by_uav_percent': {
            key: float(value)
            for key, value in evaluation.energy_by_uav_percent.items()
        },
        'remaining_battery_by_uav_percent': {
            key: float(value)
            for key, value in evaluation.remaining_battery_by_uav_percent.items()
        },
        'task_assignment_coverage_percentage': (
            100.0 * assigned_count / total_task_count
            if total_task_count else 100.0
        ),
        'assigned_task_count': assigned_count,
        'unassigned_task_count': unassigned_count,
        'infeasible_task_count': len(infeasible_tasks),
        'infeasible_tasks': dict(infeasible_tasks),
        'unassigned_task_reasons': unassigned_reasons,
        'task_assignment_results': task_assignment_results,
        'feasibility_status': feasibility_status,
        'feasible': bool(feasible),
        'battery_violation_count': int(battery_violations),
        'safety_violation_count': int(safety_violations),
        'constraint_violation_count': len(violations),
        'constraint_violations': [str(item) for item in violations],
        'battery_or_safety_violation_count': sum(
            any(word in str(item).lower() for word in (
                'battery', 'safety', 'separation', 'reserve', 'collision',
            ))
            for item in violations
        ),
        'solver': evaluation.solver,
    }


def _write_evaluation(
    directory, evaluation, tasks, run_info, debug_task_details=False
):
    directory.mkdir(parents=True, exist_ok=False)
    metrics = _aggregate_metrics(evaluation, len(tasks), run_info)
    metrics['configuration'] = run_info['configuration']
    metrics['uav_initial_conditions'] = run_info['uav_initial_conditions']
    with (directory / 'metrics.json').open('w', encoding='utf-8') as stream:
        json.dump(metrics, stream, indent=2, sort_keys=True)
        stream.write('\n')
    if debug_task_details:
        fields = [
            'task_id', 'uav_id', 'priority', 'estimated_distance_m',
            'estimated_energy_percent',
            'estimated_workload_sec', 'bid', 'status', 'reason',
        ]
        with (directory / 'debug_task_assignments.csv').open(
            'w', newline='', encoding='utf-8'
        ) as stream:
            writer = csv.DictWriter(
                stream, fieldnames=fields, extrasaction='ignore'
            )
            writer.writeheader()
            rows = list(_assignment_rows(evaluation, tasks))
            rows.extend({
                'task_id': task_id,
                'uav_id': '',
                'priority': '',
                'estimated_distance_m': '',
                'estimated_energy_percent': '',
                'estimated_workload_sec': '',
                'bid': '',
                'status': 'infeasible',
                'reason': reason,
            } for task_id, reason in run_info.get(
                'infeasible_tasks', {}
            ).items())
            writer.writerows(rows)
    summary = [
        f"Algorithm: {metrics['algorithm_name']}",
        f"Feasibility: {metrics['feasibility_status']}",
        f"Tasks assigned/unassigned: {metrics['assigned_task_count']}/"
        f"{metrics['unassigned_task_count']} of {metrics['number_of_tasks']}",
        f"Estimated total mission time: "
        f"{metrics['estimated_total_mission_time_sec']:.2f} s; "
        'offline route estimate only',
        f"Allocation computation: {metrics['allocation_computation_time_sec']:.6f} s",
        f"Estimated travel distance: {metrics['estimated_total_distance_m']:.2f} m",
        f"Estimated energy: {metrics['estimated_energy_consumption_percent']:.2f}% battery",
        f"Battery/safety violations: {metrics['battery_violation_count']}/"
        f"{metrics['safety_violation_count']}",
        *(
            f"Assigned to {uav_id}: {', '.join(task_ids) or '(none)'}; "
            f"inspection: {metrics['inspection_time_by_uav_sec'].get(uav_id, 0.0):.2f} s; "
            f"return: {metrics['return_destination_by_uav'].get(uav_id)}"
            for uav_id, task_ids in metrics['assigned_tasks_by_uav'].items()
        ),
        *(
            f"Unassigned {task_id}: {reason}"
            for task_id, reason in metrics['unassigned_task_reasons'].items()
        ),
    ]
    (directory / 'summary.txt').write_text('\n'.join(summary) + '\n', encoding='utf-8')
    return metrics


def _comparison_records(metrics_rows, run_info):
    uav_ids = list(run_info['uav_initial_conditions'])
    records = []
    for metrics in metrics_rows:
        estimated_distances = metrics['estimated_distance_by_uav_m']
        estimated_battery = metrics['estimated_energy_by_uav_percent']
        violations = int(metrics['battery_or_safety_violation_count'])
        records.append({
            'Algorithm': {
                'existing_algorithm': 'Existing',
                'milp': 'MILP',
                'greedy': 'Greedy',
            }.get(metrics['algorithm_name'], metrics['algorithm_name']),
            'Allocation Time (s)': metrics[
                'allocation_computation_time_sec'
            ],
            'Estimated Mission Time (s)': metrics[
                'estimated_total_mission_time_sec'
            ],
            'Estimated Total Distance (m)': metrics[
                'estimated_total_distance_m'
            ],
            'Estimated UAV 1 Distance (m)': (
                estimated_distances.get(uav_ids[0], 'N/A')
                if uav_ids else 'N/A'
            ),
            'Estimated UAV 2 Distance (m)': (
                estimated_distances.get(uav_ids[1], 'N/A')
                if len(uav_ids) > 1 else 'N/A'
            ),
            'Estimated UAV 1 Battery Used (pp)': (
                estimated_battery.get(uav_ids[0], 'N/A')
                if uav_ids else 'N/A'
            ),
            'Estimated UAV 2 Battery Used (pp)': (
                estimated_battery.get(uav_ids[1], 'N/A')
                if len(uav_ids) > 1 else 'N/A'
            ),
            'Estimated UAV 1 Remaining Battery (%)': (
                metrics['remaining_battery_by_uav_percent'].get(
                    uav_ids[0], 'N/A'
                ) if uav_ids else 'N/A'
            ),
            'Estimated UAV 2 Remaining Battery (%)': (
                metrics['remaining_battery_by_uav_percent'].get(
                    uav_ids[1], 'N/A'
                ) if len(uav_ids) > 1 else 'N/A'
            ),
            'Estimated Energy Consumed (pp)': metrics[
                'estimated_energy_consumption_percent'
            ],
            'Tasks Assigned': metrics['assigned_task_count'],
            'Task Assignment Coverage (%)': metrics[
                'task_assignment_coverage_percentage'
            ],
            'Unassigned Tasks': metrics['unassigned_task_count'],
            'Feasible': metrics['feasible'],
            'Constraint Violations': violations,
        })
    return records


def _write_algorithm_comparison(run_dir, metric_rows, run_info):
    records = _comparison_records(metric_rows, run_info)
    csv_path = run_dir / 'algorithm_comparison.csv'
    fields = list(records[0]) if records else ['Algorithm']
    with csv_path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(records)

    scenario = (
        f"{run_info['task_count']} Tasks, "
        f"{run_info['waypoint_count']} Waypoints"
    )
    comparison = {
        'scenario': scenario,
        'comparison_model_version': 'coverage_bounds_v4',
        'metric_basis': (
            'Offline estimate-only comparison. All algorithms use the same '
            'route evaluator, distance model, task set, operating bounds, '
            'return policy, battery model, workload limit, capacity, and '
            'separation constraints. Mission time includes each UAV return '
            'leg and is the sum of the individual UAV takeoff-to-landing times.'
        ),
        'algorithms': records,
    }
    (run_dir / 'algorithm_comparison.json').write_text(
        json.dumps(comparison, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )

    names = [record['Algorithm'] for record in records]
    table_metrics = (
        ('Allocation Time (s)', 'Allocation Time (s)'),
        ('Estimated Mission Time (s)', 'Estimated Mission Time (s)'),
        ('Estimated Total Distance (m)', 'Estimated Total Distance (m)'),
        ('Estimated UAV 1 Distance (m)', 'Estimated UAV 1 Distance (m)'),
        ('Estimated UAV 2 Distance (m)', 'Estimated UAV 2 Distance (m)'),
        ('Estimated UAV 1 Battery Used (pp)', 'Estimated UAV 1 Battery Used (pp)'),
        ('Estimated UAV 2 Battery Used (pp)', 'Estimated UAV 2 Battery Used (pp)'),
        ('Estimated UAV 1 Remaining Battery (%)', 'Estimated UAV 1 Remaining Battery (%)'),
        ('Estimated UAV 2 Remaining Battery (%)', 'Estimated UAV 2 Remaining Battery (%)'),
        ('Estimated Energy Consumed (pp)', 'Estimated Total Battery Used (pp)'),
        ('Tasks Assigned', 'Tasks Assigned'),
        ('Task Assignment Coverage (%)', 'Task Assignment Coverage (%)'),
        ('Unassigned Tasks', 'Unassigned Tasks'),
        ('Feasible', 'Feasible'),
        ('Constraint Violations', 'Constraint Violations'),
    )
    lines = [
        'UAV ALGORITHM PERFORMANCE COMPARISON',
        '=====================================',
        '',
        f'Scenario: {scenario}',
        'All mission, distance, and battery figures are estimates; no flight data is used.',
        '',
        f"{'Metric':42} " + ' '.join(f'{name:>12}' for name in names),
    ]
    for key, label in table_metrics:
        values = []
        for record in records:
            value = record[key]
            if isinstance(value, bool):
                rendered = str(value)
            elif isinstance(value, int):
                rendered = str(value)
            elif isinstance(value, float):
                rendered = (
                    f'{value:.6f}' if key == 'Allocation Time (s)'
                    else f'{value:.2f}'
                )
            else:
                rendered = str(value)
            values.append(rendered)
        lines.append(f'{label:42} ' + ' '.join(
            f'{value:>12}' for value in values
        ))

    def winner(key, direction=min):
        choices = [
            (record[key], record['Algorithm']) for record in records
            if isinstance(record[key], (int, float))
            and not isinstance(record[key], bool)
        ]
        if not choices:
            return None
        best_value = direction(value for value, _name in choices)
        best_names = [
            name for value, name in choices
            if math.isclose(value, best_value, rel_tol=1e-9, abs_tol=1e-9)
        ]
        return ', '.join(best_names)

    lines.extend(['', 'BEST ESTIMATES'])
    lines.append(
        'Fastest allocation computation: '
        f"{winner('Allocation Time (s)') or 'N/A'}"
    )
    lines.append(
        'Lowest estimated mission time: '
        f"{winner('Estimated Mission Time (s)') or 'N/A'}"
    )
    lines.append(
        'Lowest estimated total distance: '
        f"{winner('Estimated Total Distance (m)') or 'N/A'}"
    )
    lines.append(
        'Lowest estimated battery use: '
        f"{winner('Estimated Energy Consumed (pp)') or 'N/A'}"
    )
    (run_dir / 'comparison_summary.txt').write_text(
        '\n'.join(lines) + '\n', encoding='utf-8'
    )


def _write_results(
    root, tasks, results, run_info, debug_task_details=False
):
    infeasible_tasks = run_info.get('infeasible_tasks', {})
    task_count = len(tasks) + len(infeasible_tasks)
    folder = root / f"{task_count}tasks{run_info['waypoint_count']}waypoints"
    folder.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    run_dir = folder / f'run_{timestamp}'
    run_dir.mkdir()
    comparison = {
        'run_id': run_dir.name,
        'comparison_model_version': 'coverage_bounds_v4',
        'task_count': task_count,
        'waypoint_count': run_info['waypoint_count'],
        'evaluation_basis': 'estimate_only_shared_route_and_constraint_model',
        'configuration': run_info['configuration'],
        'uav_initial_conditions': run_info['uav_initial_conditions'],
        'infeasible_tasks': infeasible_tasks,
        'algorithms': {},
    }
    metric_rows = []
    for result in results:
        metrics = _write_evaluation(
            run_dir / result.name, result, tasks, run_info,
            debug_task_details,
        )
        comparison['algorithms'][result.name] = metrics
        metric_rows.append(metrics)
    comparison_info = dict(run_info)
    comparison_info['task_count'] = task_count
    _write_algorithm_comparison(run_dir, metric_rows, comparison_info)
    csv_fields = list(metric_rows[0]) if metric_rows else []
    with (run_dir / 'comparison.csv').open(
        'w', newline='', encoding='utf-8'
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=csv_fields)
        writer.writeheader()
        for row in metric_rows:
            writer.writerow({
                key: json.dumps(value, sort_keys=True)
                if isinstance(value, (dict, list)) else value
                for key, value in row.items()
            })
    (run_dir / 'comparison.json').write_text(
        json.dumps(comparison, indent=2, sort_keys=True) + '\n', encoding='utf-8'
    )
    summary = [
        'Offline estimated allocation and route comparison; no mission was executed.',
        f"Tasks: {task_count} "
        f"(pre-marked infeasible: {len(infeasible_tasks)}); "
        f"waypoints: {run_info['waypoint_count']}",
        *(
            f"Infeasible task '{task_id}': {reason}"
            for task_id, reason in infeasible_tasks.items()
        ),
        'All algorithms use the same task set, route evaluator, battery model, '
        'workload/capacity/separation and operating-bound constraints, and '
        'base/UGV return policy.',
        '',
        'algorithm | feasibility | assigned | unassigned | allocation sec | '
        'estimated mission sec | estimated distance m | estimated energy %',
        '-' * 118,
    ]
    for row in metric_rows:
        summary.append(
            f"{row['algorithm_name']} | {row['feasibility_status']} | "
            f"{row['assigned_task_count']} | {row['unassigned_task_count']} | "
            f"{row['allocation_computation_time_sec']:.6f} | "
            f"{row['estimated_total_mission_time_sec']:.2f} | "
            f"{row['estimated_total_distance_m']:.2f} | "
            f"{row['estimated_energy_consumption_percent']:.2f}"
        )
    (run_dir / 'summary.txt').write_text(
        '\n'.join(summary) + '\n', encoding='utf-8'
    )
    return run_dir


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Compare three allocation algorithms offline without commanding UAVs.'
    )
    parser.add_argument(
        '--mission-config',
        default=str(_default_config('uav_mission', 'mission.yaml')),
    )
    parser.add_argument(
        '--allocator-config',
        default=str(_default_config('uav_allocator', 'allocator.yaml')),
    )
    parser.add_argument('--results-dir', default='')
    parser.add_argument('--milp-time-limit-sec', type=float, default=10.0)
    parser.add_argument(
        '--max-tasks-per-uav', type=int, default=0,
        help='Per-UAV assignment capacity; 0 means the task-set size.',
    )
    parser.add_argument(
        '--write-debug-task-details', action='store_true',
        help='Write optional per-task assignment CSVs under algorithm folders.',
    )
    raw_args = list(sys.argv[1:] if argv is None else argv)
    if '--ros-args' in raw_args:
        raw_args = raw_args[:raw_args.index('--ros-args')]
    args = parser.parse_args(raw_args)
    mission_data = _read_yaml(args.mission_config)
    allocator_data = _read_yaml(args.allocator_config)
    config = dict(_normalise_allocator(allocator_data))
    config['future_lookahead_tasks'] = 0
    cost_config = dict(config.get('cost', {}))
    cost_weights = dict(cost_config.get('weights', {}))
    # Hard battery/workload limits are enforced identically by all methods.
    # Remove the auction's per-bid warning penalties so the MILP's internal
    # optimization objective can represent the same route trade-offs.
    for name in ('battery', 'workload', 'future'):
        if name in cost_weights:
            cost_weights[name] = 0.0
    cost_config['weights'] = cost_weights
    config['cost'] = cost_config
    if args.milp_time_limit_sec <= 0.0:
        parser.error('--milp-time-limit-sec must be positive')
    if args.max_tasks_per_uav < 0:
        parser.error('--max-tasks-per-uav cannot be negative')
    infeasible_tasks = {}
    tasks, uavs, homes = _make_inputs(mission_data, infeasible_tasks)
    operating_bounds = _make_operating_bounds(mission_data['mission'])
    max_tasks_per_uav = args.max_tasks_per_uav or max(len(tasks), 1)
    allocator = allocator_from_config(config)
    nominal_speed_mps = float(allocator.workload_estimator.nominal_speed_mps)
    allocator.landing_energy_percentage = float(
        mission_data['mission'].get('recovery', {}).get(
            'landing_energy_percentage', 0.0
        )
    )
    allocator.landing_altitude_m = float(
        mission_data['mission'].get('recovery', {}).get(
            'landing_altitude_m', 0.2
        )
    )
    allocator.completion_return_emergency_threshold_percentage = 5.0
    source_root = _source_workspace_root()
    waypoint_count = sum(
        len(task['waypoints']) if 'waypoints' in task else 1
        for task in mission_data['mission']['tasks']
    )
    run_info = {
        'waypoint_count': waypoint_count,
        'infeasible_tasks': infeasible_tasks,
        'configuration': {
            'nominal_speed_mps': nominal_speed_mps,
            'distance_safety_factor': config.get('distance_safety_factor', 1.3),
            'battery_consumption_per_meter': config['battery_consumption_per_meter'],
            'inspection_consumption_per_minute': config['inspection_consumption_per_minute'],
            'minimum_battery_reserve_percentage': (
                allocator.battery_feasibility.minimum_reserve_percentage
            ),
            'maximum_workload_sec': config.get('maximum_workload_sec', 1800.0),
            'minimum_separation_m': config.get('minimum_separation_m', 3.0),
            'milp_time_limit_sec': args.milp_time_limit_sec,
            'max_tasks_per_uav': max_tasks_per_uav,
            'cost_weights': cost_weights,
            'future_lookahead_tasks': 0,
            'operating_bounds': {
                uav_id: bounds.as_parameters()
                for uav_id, bounds in (operating_bounds or {}).items()
            },
        },
        'task_ids': [
            task.task_id for task in tasks
        ] + list(infeasible_tasks),
        'inspection_duration_sec_by_task': {
            task.task_id: float(task.estimated_duration_sec)
            for task in tasks
        },
        'recovery_position': config.get('recovery_position'),
        'uav_initial_conditions': {
            u.uav_id: {
                'position': list(u.position),
                'battery_percentage': u.battery_percentage,
                'connected': u.connected,
                'operational': u.operational,
                'mission_status': u.mission_status,
                'takeoff_position': list(u.position),
                'normal_landing_target': list(homes[u.uav_id]),
                'initial_workload_sec': 0.0,
            }
            for u in uavs
        },
    }
    # Independent copies guarantee that hypothetical state updates from one
    # evaluator never leak into another evaluator or the live mission.
    outcomes = []
    for function in (
        lambda: run_existing(
            allocator, list(tasks), list(uavs), dict(homes),
            max_tasks_per_uav, operating_bounds,
        ),
        lambda: run_milp(
            allocator, list(tasks), list(uavs), dict(homes),
            args.milp_time_limit_sec, max_tasks_per_uav, operating_bounds,
        ),
        lambda: run_greedy(
            allocator, list(tasks), list(uavs), dict(homes),
            max_tasks_per_uav, operating_bounds,
        ),
    ):
        try:
            outcomes.append(function())
        except Exception as error:  # isolate algorithm failures and preserve the others
            name = ('existing_algorithm' if len(outcomes) == 0 else
                    'milp' if len(outcomes) == 1 else 'greedy')
            outcomes.append(Evaluation(
                name=name, status='failed', allocation_time_sec=0.0,
                assignments=[], reasons={t.task_id: str(error) for t in tasks},
                solver={'status': 'failed', 'detail': str(error)},
                distance_by_uav_m={u.uav_id: 0.0 for u in uavs},
                energy_by_uav_percent={u.uav_id: 0.0 for u in uavs},
                remaining_battery_by_uav_percent={u.uav_id: u.battery_percentage for u in uavs},
                workload_by_uav_sec={u.uav_id: 0.0 for u in uavs},
                violations=[],
            ))
    result_root = (
        Path(args.results_dir).expanduser()
        if args.results_dir else source_root / 'sim_results'
    )
    result_path = _write_results(
        result_root, tasks, outcomes, run_info,
        debug_task_details=args.write_debug_task_details,
    )
    print(f'Benchmark results saved to {result_path}')
    # Algorithm-level failures are captured in metrics.json; successful result
    # persistence means the benchmark process itself completed normally.
    return 0


if __name__ == '__main__':
    raise SystemExit(main())