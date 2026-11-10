"""Persist actual mission execution results under a stable workspace folder."""

import csv
from datetime import datetime, timezone
import json
import os
from pathlib import Path


def default_results_dir(module_file=None, prefixes=None, cwd=None):
    """Return ``<workspace source>/sim_results`` when the workspace is known."""
    module_file = Path(module_file or __file__).resolve()
    source_root = module_file.resolve().parents[2]
    if (source_root / 'uav_mission').is_dir():
        return source_root / 'sim_results'

    prefixes = prefixes
    if prefixes is None:
        prefixes = os.environ.get('COLCON_PREFIX_PATH', '').split(os.pathsep)
        prefixes += os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep)
    for prefix in prefixes:
        if not prefix:
            continue
        workspace_source = Path(prefix).expanduser().resolve().parent / 'src'
        if (workspace_source / 'uav_mission').is_dir():
            return workspace_source / 'sim_results'

    cwd = Path(cwd or Path.cwd()).expanduser().resolve()
    for candidate in (cwd, cwd / 'src'):
        if (candidate / 'uav_mission').is_dir():
            return candidate / 'sim_results'
    return cwd / 'sim_results'


def write_mission_results(
    results_dir, payload, include_task_details=False
):
    """Write JSON, CSV, and a readable summary into a unique run directory."""
    root = Path(results_dir).expanduser()
    timestamp = datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S_%f')
    run_dir = root / 'mission_runs' / f'run_{timestamp}'
    run_dir.mkdir(parents=True, exist_ok=False)

    metrics_path = run_dir / 'mission_metrics.json'
    temporary_path = run_dir / '.mission_metrics.json.tmp'
    aggregate_payload = dict(payload)
    if not include_task_details:
        aggregate_payload.pop('tasks', None)
    with temporary_path.open('w', encoding='utf-8') as stream:
        json.dump(aggregate_payload, stream, indent=2, sort_keys=True)
        stream.write('\n')
    temporary_path.replace(metrics_path)

    if include_task_details:
        fields = (
            'task_id', 'status', 'assigned_uav', 'priority',
            'estimated_duration_sec', 'x_m', 'y_m', 'z_m', 'failure_reason',
        )
        with (run_dir / 'task_results.csv').open(
            'w', newline='', encoding='utf-8'
        ) as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(payload.get('tasks', []))

    counts = payload.get('task_counts', {})
    mission_time = payload.get('mission_time_sec')
    inspection_time = payload.get('inspection_completion_time_sec')
    mission_time_text = (
        f'{mission_time:.2f} s' if mission_time is not None else 'N/A'
    )
    inspection_time_text = (
        f'{inspection_time:.2f} s'
        if inspection_time is not None else 'N/A'
    )
    summary = [
        f"Mission status: {payload.get('mission_status', 'UNKNOWN')}",
        f"Run status: {payload.get('run_status', 'UNKNOWN')}",
        f'Total mission time: {mission_time_text}',
        f'Inspection completion time: {inspection_time_text}',
        f"Tasks: {counts.get('total', 0)} total, "
        f"{counts.get('completed', 0)} completed, "
        f"{counts.get('failed', 0)} failed, "
        f"{counts.get('cancelled', 0)} cancelled, "
        f"{counts.get('unfinished', 0)} unfinished",
        f"Results: {metrics_path.name}"
        + (', task_results.csv' if include_task_details else ''),
    ]
    (run_dir / 'summary.txt').write_text(
        '\n'.join(summary) + '\n', encoding='utf-8'
    )
    return run_dir
