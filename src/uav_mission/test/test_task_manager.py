from concurrent.futures import ThreadPoolExecutor

import pytest
from uav_interfaces.msg import Task

from uav_mission.task_manager import (
    TaskManager,
    select_inspection_altitude,
    tasks_from_mission,
)
from uav_mission.mission_manager import (
    cruise_return_waypoint,
    load_mission_config,
)


def make_task(task_id):
    task = Task()
    task.task_id = task_id
    task.frame_id = 'map'
    task.target_position.z = 3.0
    task.object_height_m = 0.0
    return task


def test_assigned_tasks_are_removed_from_unassigned_list():
    manager = TaskManager([make_task('task-a'), make_task('task-b')])

    success, _ = manager.assign('task-a', 'uav1')

    assert success
    assert [task.task_id for task in manager.unassigned_tasks()] == ['task-b']


def test_uav_can_reserve_a_task_behind_its_current_task():
    manager = TaskManager([make_task('task-a'), make_task('task-b')])

    assert manager.assign('task-a', 'uav1')[0]
    assert not manager.assign('task-a', 'uav2')[0]
    assert manager.assign('task-b', 'uav1')[0]
    assert manager.unfinished_for_uav('uav1') == ['task-a', 'task-b']
    assert manager.complete('task-a', 'uav1')
    assert manager.unfinished_for_uav('uav1') == ['task-b']


def test_unknown_task_is_rejected():
    manager = TaskManager([make_task('task-a')])

    success, message = manager.assign('unknown', 'uav1')

    assert not success
    assert 'does not exist' in message


def test_completed_task_is_not_reallocated_and_uav_can_take_another_task():
    manager = TaskManager([make_task('task-a'), make_task('task-b')])

    assert manager.assign('task-a', 'uav_1')[0]
    assert manager.complete('task-a', 'uav_1')
    assert [task.task_id for task in manager.unassigned_tasks()] == ['task-b']
    assert manager.assign('task-b', 'uav_1')[0]


def test_completed_task_status_is_persistent_and_visible_to_peer_uav():
    manager = TaskManager([make_task('task-a')])
    assert manager.assign('task-a', 'uav_1')[0]
    assert manager.statuses['task-a'] == 'ASSIGNED'

    assert manager.complete('task-a', 'uav_1')

    assert manager.statuses['task-a'] == 'COMPLETED'
    assert manager.unassigned_tasks() == []
    assert not manager.assign('task-a', 'uav_2')[0]


def test_simultaneous_uavs_cannot_duplicate_one_task_assignment():
    manager = TaskManager([make_task('task-a')])
    with ThreadPoolExecutor(max_workers=2) as executor:
        results = list(executor.map(
            lambda uav_id: manager.assign('task-a', uav_id),
            ('uav_1', 'uav_2'),
        ))

    assert sum(success for success, _ in results) == 1
    assert len(manager.assignments) == 1
    assert manager.statuses['task-a'] == 'ASSIGNED'


def test_assignment_keeps_planned_outbound_and_return_distances():
    manager = TaskManager([make_task('task-a')])

    success, _ = manager.assign('task-a', 'uav_1', 12.5, 8.25)

    assert success
    assert manager.assignment_distances['task-a'] == (12.5, 8.25)


def test_failed_or_cancelled_tasks_are_not_returned_for_allocation():
    failed = make_task('failed')
    cancelled = make_task('cancelled')
    available = make_task('available')
    manager = TaskManager([failed, cancelled, available])

    assert manager.fail('failed', 'unreachable')
    assert manager.cancel('cancelled')

    assert [task.task_id for task in manager.unassigned_tasks()] == [
        'available'
    ]


def test_release_unfinished_for_failed_uav_makes_tasks_reassignable():
    manager = TaskManager([
        make_task('active'), make_task('other'),
    ])
    manager.assign('active', 'uav1')
    manager.assign('other', 'uav2')

    released = manager.release_unfinished_for_uav('uav1')

    assert released == ['active']
    assert {task.task_id for task in manager.unassigned_tasks()} == {
        'active'
    }
    assert manager.assignments['other'] == 'uav2'


def test_interrupted_task_can_be_reassigned_to_the_other_uav():
    manager = TaskManager([make_task('interrupted')])
    assert manager.assign('interrupted', 'uav_1')[0]

    assert manager.release_unfinished_for_uav('uav_1') == ['interrupted']
    assert manager.statuses['interrupted'] == 'UNASSIGNED'
    assert manager.assign('interrupted', 'uav_2')[0]


def test_path_failed_task_is_reserved_until_peer_path_is_validated():
    manager = TaskManager([make_task('path-failed')])
    assert manager.assign('path-failed', 'uav_2')[0]

    assert manager.reserve_for_fallback(
        'path-failed', 'uav_2', 'No route on uav_2.'
    )
    assert manager.statuses['path-failed'] == 'RESERVED'
    assert manager.unassigned_tasks() == []
    assert not manager.assign('path-failed', 'uav_1')[0]

    assert manager.assign_reserved(
        'path-failed', 'uav_1', outbound=12.0, returning=8.0
    )
    assert manager.statuses['path-failed'] == 'ASSIGNED'
    assert manager.assignments['path-failed'] == 'uav_1'
    assert manager.assignment_distances['path-failed'] == (12.0, 8.0)


def test_failed_peer_fallback_marks_task_terminal_not_completed():
    manager = TaskManager([make_task('unreachable')])
    assert manager.assign('unreachable', 'uav_2')[0]
    assert manager.reserve_for_fallback(
        'unreachable', 'uav_2', 'Original path failed.'
    )

    assert manager.fail('unreachable', 'Neither UAV can reach the task.')
    assert manager.statuses['unreachable'] == 'FAILED'
    assert manager.failed['unreachable'] == 'Neither UAV can reach the task.'
    assert manager.completed == set()
    assert manager.unassigned_tasks() == []


def test_new_task_is_added_and_duplicate_id_is_rejected():
    manager = TaskManager([])
    task = make_task('new')
    task.priority = 2
    task.estimated_duration = 30.0

    assert manager.add_task(task)[0]
    assert not manager.add_task(task)[0]
    assert [item.task_id for item in manager.unassigned_tasks()] == ['new']


def test_task_with_a_different_frame_is_rejected():
    manager = TaskManager([])
    task = make_task('wrong-frame')
    task.frame_id = 'odom'
    task.priority = 1
    task.estimated_duration = 5.0

    accepted, message = manager.add_task(task)

    assert not accepted
    assert 'does not match mission frame' in message


def test_added_task_must_satisfy_configured_vertical_clearance():
    manager = TaskManager([], 'map', maximum_altitude_m=30.0)
    task = make_task('too-high')
    task.priority = 1
    task.estimated_duration = 5.0
    task.target_position.z = 30.1

    accepted, message = manager.add_task(task)

    assert not accepted
    assert 'safe vertical range' in message


@pytest.mark.parametrize(
    ('object_height', 'expected_altitude'),
    [
        (0.5, 3.0),
        (3.0, 5.0),
        (5.0, 7.0),
        (7.0, 10.0),
    ],
)
def test_preferred_inspection_altitude_bands_at_boundaries(
    object_height, expected_altitude,
):
    assert select_inspection_altitude(object_height) == expected_altitude


@pytest.mark.parametrize(
    ('object_height', 'expected_altitude'),
    [(2.9, 3.0), (4.9, 5.0), (6.9, 7.0)],
)
def test_inspection_altitude_does_not_add_an_object_height_offset(
    object_height, expected_altitude,
):
    assert select_inspection_altitude(object_height) == expected_altitude


def test_tall_object_uses_highest_configured_inspection_altitude():
    assert select_inspection_altitude(8.1) == 10.0


def test_ten_meter_band_is_safe_at_the_ten_meter_center_altitude_limit():
    assert select_inspection_altitude(
        7.86, maximum_altitude_m=10.0
    ) == 10.0
    assert select_inspection_altitude(
        8.0, maximum_altitude_m=10.0
    ) == 10.0


def test_inspection_is_infeasible_when_ten_meters_exceeds_ceiling():
    with pytest.raises(ValueError, match='No available inspection altitude'):
        select_inspection_altitude(7.0, maximum_altitude_m=9.0)


def test_tall_mission_task_is_not_rejected_by_old_height_clearance_rule():
    mission_data = {
        'mission': {
            'frame_id': 'map',
            'flight_levels': {
                'standard_altitude_m': 7.0,
                'inspection_altitudes_m': [3.0, 5.0, 7.0, 10.0],
            },
            'constraints': {'maximum_altitude_m': 30.0},
            'tasks': [{
                'id': 'too-high', 'waypoint': [1.0, 2.0],
                'object_height_m': 8.1, 'priority': 1,
                'estimated_duration_s': 5.0,
            }],
        },
    }
    infeasible = {}
    tasks = tasks_from_mission(mission_data, infeasible)
    manager = TaskManager(
        tasks, maximum_altitude_m=30.0, infeasible_tasks=infeasible
    )

    assert not infeasible
    assert manager.statuses['too-high'] == 'UNASSIGNED'
    assert manager.tasks['too-high'].target_position.z == 10.0
    assert [
        task.task_id for task in manager.unassigned_tasks()
    ] == ['too-high']


def test_inspection_return_waypoint_uses_seven_meter_cruise():
    task = make_task('task')
    task.target_position.z = 10.0
    mission = load_mission_config()['mission']
    altitude = mission['flight_levels']['standard_altitude_m']
    waypoint = cruise_return_waypoint(task, altitude)

    assert altitude == 7.0
    assert waypoint.pose.position.z == altitude


def test_added_task_must_use_its_safe_selected_altitude():
    manager = TaskManager([make_task('existing')])
    task = make_task('wrong-band')
    task.priority = 1
    task.estimated_duration = 5.0
    task.target_position.z = 10.0

    accepted, message = manager.add_task(task)

    assert not accepted
    assert 'safe preferred inspection band' in message


def test_task_manager_accepts_safe_ten_meter_inspection_at_ceiling():
    manager = TaskManager([], 'map', maximum_altitude_m=10.0)
    task = make_task('building')
    task.priority = 1
    task.estimated_duration = 5.0
    task.object_height_m = 7.86
    task.target_position.z = 10.0

    accepted, _ = manager.add_task(task)

    assert accepted


def test_navigation_retries_are_bounded_and_terminal_after_limit():
    manager = TaskManager([make_task('task')])
    assert manager.assign('task', 'uav_1')[0]

    assert manager.retry_navigation('task', 'uav_1', 2, 'first') == 'RETRY'
    assert manager.statuses['task'] == 'UNASSIGNED'
    assert manager.assign('task', 'uav_2')[0]
    assert manager.retry_navigation('task', 'uav_2', 2, 'second') == 'RETRY'
    assert manager.assign('task', 'uav_1')[0]
    assert manager.retry_navigation('task', 'uav_1', 2, 'third') == 'FAILED'
    assert manager.statuses['task'] == 'FAILED'
    assert manager.failed['task'] == 'third'
    assert manager.navigation_retries['task'] == 3


def test_mission_tasks_keep_explicit_xyz_and_declared_map_frame():
    mission_data = {
        'mission': {
            'frame_id': 'map',
            'flight_levels': {
                'standard_altitude_m': 7.0,
                'inspection_altitudes_m': [3.0, 5.0, 7.0, 10.0],
            },
            'constraints': {'maximum_altitude_m': 30.0},
            'tasks': [
                {
                    'id': 'short', 'waypoint': [1.25, -3.5, 7.0],
                    'object_height_m': 7.9, 'priority': 1,
                    'estimated_duration_s': 10.0,
                },
                {
                    'id': 'threshold', 'waypoint': [4.0, 6.0, 7.0],
                    'object_height_m': 8.0, 'priority': 2,
                    'estimated_duration_s': 10.0,
                },
            ],
        },
    }

    tasks = tasks_from_mission(mission_data)

    assert [task.frame_id for task in tasks] == ['map', 'map']
    assert [
        (
            task.target_position.x,
            task.target_position.y,
            task.target_position.z,
            task.object_height_m,
        )
        for task in tasks
    ] == [(1.25, -3.5, 7.0, 7.9), (4.0, 6.0, 7.0, 8.0)]


def test_tall_mission_object_uses_highest_inspection_altitude():
    mission_data = {
        'mission': {
            'frame_id': 'map',
            'flight_levels': {
                'standard_altitude_m': 7.0,
                'inspection_altitudes_m': [3.0, 5.0, 7.0, 10.0],
            },
            'constraints': {'maximum_altitude_m': 30.0},
            'tasks': [{
                'id': 'too-high', 'type': 'inspection',
                'waypoint': [1.0, 2.0], 'object_height_m': 30.0,
                'priority': 1, 'estimated_duration_s': 5.0,
            }],
        },
    }

    tasks = tasks_from_mission(mission_data)

    assert len(tasks) == 1
    assert tasks[0].target_position.z == 10.0


@pytest.mark.parametrize(
    ('target_altitude', 'expected_altitude'),
    [(3.0, 3.0), (5.0, 5.0), (7.0, 7.0), (10.0, 10.0)],
)
def test_explicit_waypoint_altitude_is_used_without_object_height(
    target_altitude, expected_altitude,
):
    mission_data = {
        'mission': {
            'frame_id': 'map',
            'flight_levels': {
                'standard_altitude_m': 7.0,
                'inspection_altitudes_m': [3.0, 5.0, 7.0, 10.0],
            },
            'constraints': {'maximum_altitude_m': 15.0},
            'tasks': [{
                'id': 'target-altitude',
                'waypoint': [1.0, 2.0, target_altitude],
                'priority': 1,
                'estimated_duration_s': 5.0,
            }],
        },
    }

    tasks = tasks_from_mission(mission_data)

    assert len(tasks) == 1
    assert tasks[0].target_position.z == expected_altitude
    assert tasks[0].object_height_m == 0.0


def test_explicit_waypoint_altitude_must_match_a_planner_band():
    mission_data = {
        'mission': {
            'frame_id': 'map',
            'flight_levels': {
                'standard_altitude_m': 7.0,
                'inspection_altitudes_m': [3.0, 5.0, 7.0, 10.0],
            },
            'constraints': {'maximum_altitude_m': 15.0},
            'tasks': [{
                'id': 'unsupported-target-altitude',
                'waypoint': [1.0, 2.0, 4.0],
                'priority': 1,
                'estimated_duration_s': 5.0,
            }],
        },
    }
    infeasible = {}

    tasks = tasks_from_mission(mission_data, infeasible)

    assert infeasible['unsupported-target-altitude'].startswith(
        "Task 'unsupported-target-altitude' is infeasible"
    )
    manager = TaskManager(
        tasks, maximum_altitude_m=15.0, infeasible_tasks=infeasible
    )
    assert manager.statuses['unsupported-target-altitude'] == 'FAILED'


def test_two_coordinate_waypoint_requires_object_height_for_legacy_selection():
    mission_data = {
        'mission': {
            'frame_id': 'map',
            'flight_levels': {
                'standard_altitude_m': 7.0,
                'inspection_altitudes_m': [3.0, 5.0, 7.0, 10.0],
            },
            'constraints': {'maximum_altitude_m': 15.0},
            'tasks': [{
                'id': 'missing-altitude',
                'waypoint': [1.0, 2.0],
                'priority': 1,
                'estimated_duration_s': 5.0,
            }],
        },
    }

    with pytest.raises(ValueError, match='must specify object_height_m'):
        tasks_from_mission(mission_data)


def test_mission_rejects_unsupported_task_types():
    mission_data = {
        'mission': {
            'frame_id': 'map',
            'flight_levels': {
                'standard_altitude_m': 7.0,
                'inspection_altitudes_m': [3.0, 5.0, 7.0, 10.0],
            },
            'constraints': {'maximum_altitude_m': 30.0},
            'tasks': [{
                'id': 'unsupported', 'type': 'repair',
                'waypoint': [1.0, 2.0], 'object_height_m': 5.0,
                'priority': 1, 'estimated_duration_s': 5.0,
            }],
        },
    }

    with pytest.raises(ValueError, match='only simulated inspection'):
        tasks_from_mission(mission_data)
