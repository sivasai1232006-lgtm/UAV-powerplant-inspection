import json
from pathlib import Path
from types import SimpleNamespace

from uav_mission.mission_manager import MissionManager
from uav_mission.result_writer import (
    default_results_dir,
    write_mission_results,
)


def test_default_results_directory_is_in_source_workspace():
    source_file = Path(__file__).resolve().parents[1] / 'uav_mission' / 'result_writer.py'
    expected = Path(__file__).resolve().parents[1].parent / 'sim_results'
    assert default_results_dir(module_file=source_file) == expected


def test_mission_results_write_aggregate_json_and_summary_by_default(tmp_path):
    payload = {
        'run_status': 'INTERRUPTED',
        'mission_status': 'INCOMPLETE',
        'mission_time_sec': 12.5,
        'inspection_completion_time_sec': None,
        'task_counts': {
            'total': 1, 'completed': 0, 'failed': 0,
            'cancelled': 0, 'unfinished': 1,
        },
        'tasks': [{
            'task_id': 'inspect_1', 'status': 'ASSIGNED',
            'assigned_uav': 'uav_1', 'priority': 2,
            'estimated_duration_sec': 5.0, 'x_m': 1.0, 'y_m': 2.0,
            'z_m': 8.0, 'failure_reason': '',
        }],
    }

    result_dir = write_mission_results(tmp_path, payload)

    assert result_dir.parent == tmp_path / 'mission_runs'
    saved_metrics = json.loads(
        (result_dir / 'mission_metrics.json').read_text(encoding='utf-8')
    )
    assert 'tasks' not in saved_metrics
    assert saved_metrics['task_counts'] == payload['task_counts']
    assert not (result_dir / 'task_results.csv').exists()
    assert 'INTERRUPTED' in (result_dir / 'summary.txt').read_text(
        encoding='utf-8'
    )
    summary = (result_dir / 'summary.txt').read_text(encoding='utf-8')
    assert 'Inspection completion time: N/A' in summary


def test_task_level_mission_logs_are_opt_in(tmp_path):
    result_dir = write_mission_results(
        tmp_path,
        {
            'mission_time_sec': None,
            'inspection_completion_time_sec': None,
            'task_counts': {},
            'tasks': [{'task_id': 'detail-task', 'status': 'COMPLETED'}],
        },
        include_task_details=True,
    )

    assert 'detail-task' in (result_dir / 'task_results.csv').read_text(
        encoding='utf-8'
    )
    assert 'Total mission time: N/A' in (
        result_dir / 'summary.txt'
    ).read_text(encoding='utf-8')


def test_shutdown_waits_for_complete_three_algorithm_comparison(tmp_path):
    run_dir = tmp_path / '4tasks2waypoints' / 'run_20261108_100000_000000'
    run_dir.mkdir(parents=True)
    algorithms = ['existing_algorithm', 'milp', 'greedy']
    for filename in (
        'algorithm_comparison.csv', 'comparison.csv',
    ):
        (run_dir / filename).write_text(
            'Algorithm\nExisting\nMILP\nGreedy\n', encoding='utf-8'
        )
    for filename in ('comparison_summary.txt', 'summary.txt'):
        (run_dir / filename).write_text('complete\n', encoding='utf-8')
    (run_dir / 'algorithm_comparison.json').write_text(json.dumps({
        'algorithms': [{'Algorithm': name} for name in algorithms],
    }), encoding='utf-8')
    (run_dir / 'comparison.json').write_text(json.dumps({
        'algorithms': {name: {} for name in algorithms},
    }), encoding='utf-8')
    for algorithm in algorithms:
        directory = run_dir / algorithm
        directory.mkdir()
        (directory / 'metrics.json').write_text('{}\n', encoding='utf-8')
        (directory / 'summary.txt').write_text('complete\n', encoding='utf-8')

    manager = SimpleNamespace(results_dir=tmp_path)
    assert MissionManager._benchmark_comparison_complete(manager)

    (run_dir / 'comparison_summary.txt').unlink()
    assert not MissionManager._benchmark_comparison_complete(manager)
