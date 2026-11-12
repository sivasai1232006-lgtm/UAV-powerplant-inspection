import json

import pytest
import yaml

from uav_allocator.auction_allocator import (
    InspectionTask,
    UavSnapshot,
    allocator_from_config,
)
from uav_interfaces.msg import UAVState
from uav_mission.operating_bounds import OperatingBounds

from benchmarks.algorithms import (
    Evaluation,
    run_existing,
    run_greedy,
    run_milp,
)
from benchmarks.compare import collect
from benchmarks.runner import (
    _aggregate_metrics,
    _make_inputs,
    main as run_benchmark,
)


def _allocator():
    return allocator_from_config({
        'planning_batch_size': 8,
        'maximum_workload_sec': 1800.0,
        'workload_warning_sec': 900.0,
        'battery_warning_margin_percentage': 15.0,
        'future_lookahead_tasks': 0,
        'minimum_separation_m': 3.0,
        'distance_safety_factor': 1.3,
        'recovery_position': [0.0, 0.0, 0.0],
        'nominal_speed_mps': 2.4,
        'state_timeout_sec': 3.0,
        'battery_consumption_per_meter': 0.05,
        'inspection_consumption_per_minute': 0.25,
        'minimum_battery_reserve_percentage': 5.0,
        'cost': {
            'weights': {
                'distance': 0.4, 'time': 0.25, 'energy': 0.25,
                'priority': 0.1, 'battery': 0.0, 'workload': 0.0,
                'future': 0.0,
            },
            'distance_scale_m': 100.0, 'time_scale_sec': 300.0,
            'energy_scale_percentage': 10.0, 'minimum_priority': 1,
            'maximum_priority': 5,
        },
    })


def _inputs(battery=100.0, connected=True):
    tasks = [
        InspectionTask('north', (10.0, 0.0, 5.0), 5, 30.0),
        InspectionTask('south', (0.0, 10.0, 5.0), 2, 30.0),
    ]
    uavs = [
        UavSnapshot('uav_1', (0.0, 0.0, 5.0), battery, connected, True,
                    UAVState.IDLE, 0.0),
        UavSnapshot('uav_2', (20.0, 0.0, 5.0), battery, connected, True,
                    UAVState.IDLE, 0.0),
    ]
    return tasks, uavs, {'uav_1': (0.0, 0.0, 0.0), 'uav_2': (20.0, 0.0, 0.0)}


def test_greedy_is_unique_deterministic_and_respects_priority():
    allocator = _allocator()
    tasks, uavs, homes = _inputs()
    first = run_greedy(allocator, tasks, uavs, homes)
    second = run_greedy(allocator, tasks, uavs, homes)
    assert [row['task_id'] for row in first.assignments] == ['north', 'south']
    assert len({row['task_id'] for row in first.assignments}) == 2
    assert [(r['task_id'], r['uav_id']) for r in first.assignments] == [
        (r['task_id'], r['uav_id']) for r in second.assignments
    ]


def test_greedy_respects_battery_return_reserve():
    allocator = _allocator()
    tasks, uavs, homes = _inputs(battery=5.0)
    result = run_greedy(allocator, tasks, uavs, homes)
    assert result.assignments == []
    assert set(result.reasons) == {'north', 'south'}


def test_benchmark_does_not_treat_task_endpoint_as_landing_site():
    allocator = _allocator()
    allocator.recovery_position = (100.0, 0.0, 0.2)
    task = InspectionTask('near', (10.0, 0.0, 5.0), 3, 0.0)
    uav = UavSnapshot(
        'uav_1', (0.0, 0.0, 5.0), 8.0, True, True,
        UAVState.IDLE, 0.0,
    )

    result = run_greedy(
        allocator, [task], [uav], {'uav_1': (0.0, 0.0, 0.2)}
    )

    assert result.assignments == []


def test_greedy_respects_availability_and_per_uav_capacity():
    allocator = _allocator()
    tasks, uavs, homes = _inputs(connected=False)
    unavailable = run_greedy(allocator, tasks, uavs, homes)
    assert unavailable.assignments == []
    tasks, uavs, homes = _inputs()
    capped = run_greedy(allocator, tasks, uavs, homes, max_tasks_per_uav=1)
    counts = {}
    for row in capped.assignments:
        counts[row['uav_id']] = counts.get(row['uav_id'], 0) + 1
    assert all(count <= 1 for count in counts.values())


def test_existing_baseline_invokes_project_allocator():
    allocator = _allocator()
    tasks, uavs, homes = _inputs()
    result = run_existing(allocator, tasks, uavs, homes)
    assert result.name == 'existing_algorithm'
    assert result.assignments
    assert len({row['task_id'] for row in result.assignments}) == len(result.assignments)


def test_existing_baseline_repeats_cycles_after_simulated_task_completion():
    allocator = _allocator()
    tasks = [
        InspectionTask('first', (5.0, 0.0, 5.0), 3, 5.0),
        InspectionTask('second', (10.0, 0.0, 5.0), 2, 5.0),
        InspectionTask('third', (15.0, 0.0, 5.0), 1, 5.0),
    ]
    uav = UavSnapshot(
        'uav_1', (0.0, 0.0, 5.0), 100.0, True, True,
        UAVState.IDLE, 0.0,
    )

    result = run_existing(
        allocator, tasks, [uav], {'uav_1': (0.0, 0.0, 0.0)},
        max_tasks_per_uav=3,
    )

    assert len(result.assignments) == 3
    assert result.violations == []


def test_mission_messages_convert_to_project_task_dataclasses():
    mission = {
        'mission': {
            'frame_id': 'map',
            'flight_levels': {
                'standard_altitude_m': 7.0,
                'inspection_altitudes_m': [3.0, 5.0, 7.0, 10.0],
            },
            'constraints': {'maximum_altitude_m': 30.0},
            'tasks': [{
                'id': 'test_task', 'waypoint': [1.0, 2.0, 7.0],
                'object_height_m': 7.0, 'priority': 4,
                'estimated_duration_s': 20.0,
            }],
            'uavs': [{
                'id': 'uav_1', 'start_position': [0.0, 0.0, 0.0],
                'home_position': [0.0, 0.0, 0.2],
                'battery_remaining': 90.0,
            }],
        },
    }
    tasks, uavs, homes = _make_inputs(mission)
    assert tasks == [
        InspectionTask('test_task', (1.0, 2.0, 7.0), 4, 20.0)
    ]
    assert uavs[0].uav_id == 'uav_1'
    assert homes['uav_1'] == (0.0, 0.0, 0.2)


def test_benchmark_accepts_explicit_waypoint_altitude_without_object_height():
    mission = {
        'mission': {
            'frame_id': 'map',
            'flight_levels': {
                'standard_altitude_m': 7.0,
                'inspection_altitudes_m': [3.0, 5.0, 7.0, 10.0],
            },
            'constraints': {'maximum_altitude_m': 15.0},
            'tasks': [{
                'id': 'target-altitude',
                'waypoint': [1.0, 2.0, 5.0],
                'priority': 1,
                'estimated_duration_s': 5.0,
            }],
            'uavs': [{
                'id': 'uav_1',
                'start_position': [0.0, 0.0, 0.0],
                'home_position': [0.0, 0.0, 0.2],
                'battery_remaining': 90.0,
            }],
        },
    }

    tasks, _, _ = _make_inputs(mission)

    assert len(tasks) == 1
    assert tasks[0].target_position == (1.0, 2.0, 5.0)


def test_safe_ten_meter_inspection_is_not_misclassified_as_infeasible():
    mission = {
        'mission': {
            'frame_id': 'map',
            'flight_levels': {
                'standard_altitude_m': 7.0,
                'inspection_altitudes_m': [3.0, 5.0, 7.0, 10.0],
            },
            'constraints': {
                'maximum_altitude_m': 10.0,
                'uav_vertical_half_extent_m': 0.25,
                'vertical_obstacle_clearance_m': 0.25,
            },
            'tasks': [{
                'id': 'building',
                'waypoint': [1.0, 2.0, 10.0],
                'object_height_m': 7.86,
                'priority': 1,
                'estimated_duration_s': 5.0,
            }],
            'uavs': [{
                'id': 'uav_1',
                'start_position': [0.0, 0.0, 0.0],
                'home_position': [0.0, 0.0, 0.2],
                'battery_remaining': 90.0,
            }],
        },
    }
    infeasible = {}

    tasks, _, _ = _make_inputs(mission, infeasible)

    assert len(tasks) == 1
    assert tasks[0].target_position[2] == 10.0
    assert infeasible == {}


def test_explicit_target_altitude_is_not_rejected_by_object_height():
    mission = {
        'mission': {
            'frame_id': 'map',
            'flight_levels': {
                'standard_altitude_m': 7.0,
                'inspection_altitudes_m': [3.0, 5.0, 7.0, 10.0],
            },
            'constraints': {
                'maximum_altitude_m': 10.0,
                'uav_vertical_half_extent_m': 0.25,
                'vertical_obstacle_clearance_m': 0.25,
            },
            'tasks': [{
                'id': 'building',
                'waypoint': [1.0, 2.0, 10.0],
                'object_height_m': 8.1,
                'priority': 1,
                'estimated_duration_s': 5.0,
            }],
            'uavs': [{
                'id': 'uav_1',
                'start_position': [0.0, 0.0, 0.0],
                'home_position': [0.0, 0.0, 0.2],
                'battery_remaining': 90.0,
            }],
        },
    }
    infeasible = {}

    tasks, _, _ = _make_inputs(mission, infeasible)

    assert len(tasks) == 1
    assert tasks[0].target_position[2] == 10.0
    assert infeasible == {}


def test_infeasible_tasks_count_as_unassigned_in_benchmark_metrics():
    evaluation = Evaluation(
        name='greedy',
        status='completed',
        allocation_time_sec=0.0,
        assignments=[],
        reasons={},
        solver={'status': 'not_applicable'},
        distance_by_uav_m={},
        energy_by_uav_percent={},
        remaining_battery_by_uav_percent={},
        workload_by_uav_sec={},
        violations=[],
    )
    run_info = {
        'waypoint_count': 2,
        'infeasible_tasks': {'building': 'No safe altitude.'},
    }

    metrics = _aggregate_metrics(evaluation, task_count=1, run_info=run_info)

    assert metrics['number_of_tasks'] == 2
    assert metrics['unassigned_task_count'] == 2
    assert metrics['infeasible_task_count'] == 1
    assert metrics['task_assignment_coverage_percentage'] == 0.0
    assert metrics['feasible'] is False
    assert metrics['task_assignment_results']['building'] == {
        'status': 'infeasible',
        'uav_id': None,
        'reason': 'No safe altitude.',
    }
    assert 'uav_velocity_multiplier' not in metrics


def test_milp_reports_missing_solver_instead_of_fabricating_assignments(monkeypatch):
    allocator = _allocator()
    tasks, uavs, homes = _inputs()
    import builtins
    original_import = builtins.__import__

    def fail_pulp(name, *args, **kwargs):
        if name == 'pulp':
            raise ImportError('test solver absent')
        return original_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, '__import__', fail_pulp)
    result = run_milp(allocator, tasks, uavs, homes, 1.0)
    assert result.status == 'solver_unavailable'
    assert not result.assignments
    assert not result.solver['incumbent_available']


def test_results_use_shared_estimates_and_omit_unavailable_actual_metrics(tmp_path):
    allocator = _allocator()
    tasks, uavs, homes = _inputs()
    results = [
        run_existing(allocator, tasks, uavs, homes),
        run_milp(allocator, tasks, uavs, homes, 1.0),
        run_greedy(allocator, tasks, uavs, homes),
    ]
    assert results[1].status in ('optimal', 'feasible_incumbent')
    assert results[1].solver['name'].endswith('via PuLP')
    assert results[1].solver['status'] == 'Optimal'
    run_info = {
        'waypoint_count': 2,
        'configuration': {},
        'uav_initial_conditions': {},
    }
    from benchmarks.runner import _write_results
    first = _write_results(tmp_path, tasks, results, run_info)
    second = _write_results(tmp_path, tasks, results, run_info)
    assert first != second
    assert (first / 'comparison.csv').is_file()
    assert (first / 'algorithm_comparison.csv').is_file()
    assert (first / 'algorithm_comparison.json').is_file()
    assert (first / 'comparison_summary.txt').is_file()
    assert not (first / 'greedy/task_assignments.csv').exists()
    import json
    metrics = json.loads((first / 'greedy/metrics.json').read_text())
    assert 'actual_total_distance_m' not in metrics
    assert 'actual_energy_consumption_wh' not in metrics
    assert 'completed_task_count' not in metrics
    assert metrics['estimated_mission_completion_time_sec'] >= 0.0
    assert metrics['task_assignment_coverage_percentage'] == 100.0
    assert metrics['task_assignment_results']['north'][
        'status'
    ] == 'assigned_not_executed'
    assert metrics['task_assignment_results']['north'][
        'estimated_energy_percent'
    ] > 0.0
    rows = collect(tmp_path)
    assert len(rows) == 6
    assert 'mission_execution_status' not in rows[0]
    legacy = tmp_path / '2tasks2waypoints/run_legacy'
    legacy.mkdir(parents=True)
    (legacy / 'comparison.json').write_text(
        json.dumps({'algorithms': {'milp': {'status': 'optimal'}}})
    )
    assert len(collect(tmp_path)) == 6
    with (first / 'comparison.csv').open(encoding='utf-8') as stream:
        import csv
        assert len(list(csv.DictReader(stream))) == 3
    with (first / 'algorithm_comparison.csv').open(
        encoding='utf-8'
    ) as stream:
        import csv
        comparison_rows = list(csv.DictReader(stream))
    assert len(comparison_rows) == 3
    assert [row['Algorithm'] for row in comparison_rows] == [
        'Existing', 'MILP', 'Greedy'
    ]
    required_columns = {
        'Allocation Time (s)', 'Estimated Mission Time (s)',
        'Estimated Total Distance (m)', 'Estimated Energy Consumed (pp)',
        'Tasks Assigned', 'Task Assignment Coverage (%)',
        'Unassigned Tasks', 'Feasible', 'Constraint Violations',
        'Solver Status', 'Optimality Gap (%)',
    }
    assert required_columns.issubset(comparison_rows[0])
    assert not any('Actual' in column for column in comparison_rows[0])
    assert not any('Completed' in column for column in comparison_rows[0])
    comparison = json.loads((first / 'comparison.json').read_text())
    assert set(comparison['algorithms']) == {
        'existing_algorithm', 'milp', 'greedy'
    }
    assert (first / 'summary.txt').is_file()
    algorithm_comparison = json.loads(
        (first / 'algorithm_comparison.json').read_text()
    )
    assert [row['Algorithm'] for row in algorithm_comparison['algorithms']] == [
        'Existing', 'MILP', 'Greedy'
    ]
    summary = (first / 'comparison_summary.txt').read_text()
    assert 'All mission, distance, and battery figures are estimates' in summary
    assert 'Fastest allocation computation:' in summary
    assert 'Lowest estimated mission time:' in summary


def test_milp_optimizes_the_shared_feasible_plan_cost():
    allocator = _allocator()
    tasks, uavs, homes = _inputs()
    methods = [
        run_existing(allocator, tasks, uavs, homes),
        run_milp(allocator, tasks, uavs, homes, 1.0),
        run_greedy(allocator, tasks, uavs, homes),
    ]
    assert all(result.violations == [] for result in methods)
    assert all(len(result.assignments) == len(tasks) for result in methods)
    # MILP and both heuristics use the same weighted route cost. An optimal
    # MILP must therefore score no worse than either feasible heuristic.
    assert methods[1].objective_value <= methods[0].objective_value + 1e-5
    assert methods[1].objective_value <= methods[2].objective_value + 1e-5


def test_all_algorithms_apply_cross_uav_separation_constraint():
    allocator = _allocator()
    tasks = [
        InspectionTask('close_a', (10.0, 0.0, 5.0), 3, 10.0),
        InspectionTask('close_b', (11.0, 0.0, 5.0), 3, 10.0),
    ]
    uavs = [
        UavSnapshot('uav_1', (0.0, 0.0, 5.0), 100.0, True, True,
                    UAVState.IDLE, 0.0),
        UavSnapshot('uav_2', (20.0, 0.0, 5.0), 100.0, True, True,
                    UAVState.IDLE, 0.0),
    ]
    homes = {'uav_1': (0.0, 0.0, 0.0), 'uav_2': (20.0, 0.0, 0.0)}
    results = [
        run_existing(allocator, tasks, uavs, homes),
        run_milp(allocator, tasks, uavs, homes, 1.0),
        run_greedy(allocator, tasks, uavs, homes),
    ]
    for result in results:
        assert result.violations == []
        assert len(result.assignments) == 2
        assert len({row['uav_id'] for row in result.assignments}) == 1


@pytest.mark.parametrize(
    'target',
    [
        (-30.0, 0.0, 5.0),
        (10.0, 0.0, 5.0),
        (0.0, -70.0, 5.0),
        (0.0, 70.0, 5.0),
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 10.0),
    ],
)
def test_allocation_algorithms_allow_goals_on_operating_bounds(target):
    allocator = _allocator()
    task = InspectionTask('edge', target, 3, 5.0)
    uav = UavSnapshot(
        'uav_1', (0.0, 0.0, 5.0), 100.0, True, True,
        UAVState.IDLE, 0.0,
    )
    region = {
        'x_min_offset_m': -30.0,
        'x_max_offset_m': 10.0,
        'y_min_offset_m': -70.0,
        'y_max_offset_m': 70.0,
        'z_min_m': 0.0,
        'z_max_m': 10.0,
    }
    bounds = {
        'uav_1': OperatingBounds.from_initial_position(uav.position, region)
    }
    homes = {'uav_1': (0.0, 0.0, 0.2)}

    results = [
        run_existing(allocator, [task], [uav], homes, operating_bounds=bounds),
        run_greedy(allocator, [task], [uav], homes, operating_bounds=bounds),
        run_milp(
            allocator, [task], [uav], homes, 1.0,
            operating_bounds=bounds,
        ),
    ]

    assert all(len(result.assignments) == 1 for result in results)
    assert all(result.violations == [] for result in results)


@pytest.mark.parametrize(
    'target',
    [
        (-30.001, 0.0, 5.0),
        (10.001, 0.0, 5.0),
        (0.0, -70.001, 5.0),
        (0.0, 70.001, 5.0),
        (0.0, 0.0, -0.001),
        (0.0, 0.0, 10.001),
    ],
)
def test_allocation_algorithms_reject_goals_outside_operating_bounds(target):
    allocator = _allocator()
    task = InspectionTask('outside', target, 3, 5.0)
    uav = UavSnapshot(
        'uav_1', (0.0, 0.0, 5.0), 100.0, True, True,
        UAVState.IDLE, 0.0,
    )
    region = {
        'x_min_offset_m': -30.0,
        'x_max_offset_m': 10.0,
        'y_min_offset_m': -70.0,
        'y_max_offset_m': 70.0,
        'z_min_m': 0.0,
        'z_max_m': 10.0,
    }
    bounds = {
        'uav_1': OperatingBounds.from_initial_position(uav.position, region)
    }
    homes = {'uav_1': (0.0, 0.0, 0.2)}

    results = [
        run_existing(allocator, [task], [uav], homes, operating_bounds=bounds),
        run_greedy(allocator, [task], [uav], homes, operating_bounds=bounds),
        run_milp(
            allocator, [task], [uav], homes, 1.0,
            operating_bounds=bounds,
        ),
    ]

    assert all(result.assignments == [] for result in results)
    assert all('outside' in result.reasons for result in results)


def test_runner_uses_effective_post_landing_reserve_from_allocator_config(
    tmp_path,
):
    mission_config = {
        'mission': {
            'frame_id': 'map',
            'flight_levels': {
                'standard_altitude_m': 7.0,
                'inspection_altitudes_m': [3.0, 5.0, 7.0, 10.0],
            },
            'constraints': {'maximum_altitude_m': 20.0},
            'recovery': {'landing_altitude_m': 0.2},
            'tasks': [{
                'id': 'single_task',
                'waypoint': [5.0, 0.0, 7.0],
                'object_height_m': 1.0,
                'priority': 3,
                'estimated_duration_s': 5.0,
            }],
            'uavs': [{
                'id': 'uav_1',
                'start_position': [0.0, 0.0, 7.0],
                'home_position': [0.0, 0.0, 0.2],
                'battery_remaining': 90.0,
            }],
        },
    }
    allocator_config = {
        '/**': {
            'ros__parameters': {
                'nominal_speed_mps': 2.4,
                'state_timeout_sec': 3.0,
                'battery_consumption_per_meter': 0.05,
                'inspection_consumption_per_minute': 0.25,
                'post_landing_reserve_percentage': 10.0,
                'cost': {
                    'weights': {
                        'distance': 0.4,
                        'time': 0.25,
                        'energy': 0.25,
                        'priority': 0.1,
                    },
                    'distance_scale_m': 100.0,
                    'time_scale_sec': 300.0,
                    'energy_scale_percentage': 10.0,
                    'minimum_priority': 1,
                    'maximum_priority': 5,
                },
            },
        },
    }
    mission_path = tmp_path / 'mission.yaml'
    allocator_path = tmp_path / 'allocator.yaml'
    results_path = tmp_path / 'results'
    mission_path.write_text(yaml.safe_dump(mission_config), encoding='utf-8')
    allocator_path.write_text(
        yaml.safe_dump(allocator_config), encoding='utf-8'
    )

    assert run_benchmark([
        '--mission-config', str(mission_path),
        '--allocator-config', str(allocator_path),
        '--results-dir', str(results_path),
        '--milp-time-limit-sec', '1.0',
    ]) == 0

    comparison_path = next(results_path.glob('*/run_*/comparison.json'))
    comparison = json.loads(comparison_path.read_text(encoding='utf-8'))
    assert (
        comparison['configuration']['minimum_battery_reserve_percentage']
        == 10.0
    )
