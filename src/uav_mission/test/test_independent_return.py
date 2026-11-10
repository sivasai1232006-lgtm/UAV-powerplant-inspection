from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import time
import threading

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as NavPath
from sensor_msgs.msg import BatteryState
from uav_interfaces.msg import Task, UAVState
import yaml
from uav_mission.mission_manager import MissionManager
from uav_mission.mission_manager import load_mission_config
from uav_mission.task_manager import TaskManager


def _manager_for_return(tasks=()):
    task_manager = TaskManager(list(tasks), 'map')
    manager = SimpleNamespace(
        task_manager=task_manager,
        failed_uavs=set(),
        recovery_state=SimpleNamespace(returning=set()),
        return_targets={},
        return_navigation={},
        pending_return_uavs=set(),
        returned_uavs=set(),
        last_task_altitude={},
        standard_flight_altitude=7.0,
        _queue_completion_return=Mock(return_value=True),
    )
    return manager


def _task(task_id):
    task = Task()
    task.task_id = task_id
    task.frame_id = 'map'
    return task


def test_task_path_request_uses_target_altitude_map_and_z():
    task = _task('inspection')
    task.target_position.x = 2.0
    task.target_position.y = 3.0
    task.target_position.z = 5.0
    target_planner = Mock()
    target_planner.server_is_ready.return_value = True
    request_future = Mock()
    target_planner.send_goal_async.return_value = request_future
    other_planner = Mock()
    clock = Mock()
    clock.now.return_value.to_msg.return_value = Time()
    manager = SimpleNamespace(
        planner_clients={'uav_1': {5: target_planner, 7: other_planner}},
        frame_id='map',
        get_clock=Mock(return_value=clock),
        get_logger=Mock(return_value=Mock()),
        _path_goal_received=Mock(),
    )

    MissionManager._request_task_path(
        manager, task, 'uav_1', Odometry()
    )

    request = target_planner.send_goal_async.call_args.args[0]
    assert request.start.pose.position.z == 5.0
    assert request.goal.pose.position.z == 5.0
    target_planner.send_goal_async.assert_called_once()
    other_planner.send_goal_async.assert_not_called()


def test_planned_task_waypoints_keep_target_altitude():
    task = _task('inspection')
    task.target_position.x = 2.0
    task.target_position.y = 3.0
    task.target_position.z = 10.0
    path = NavPath()
    waypoint = PoseStamped()
    waypoint.pose.position.x = 2.0
    waypoint.pose.position.y = 3.0
    waypoint.pose.position.z = 7.0
    path.poses = [waypoint]
    manager = SimpleNamespace(
        active_navigation={'uav_1': {'task': task}},
        task_manager=SimpleNamespace(tasks={'inspection': task}),
        _record_planning_time=Mock(),
        _handle_path_failure=Mock(),
        _finish_task_path=Mock(),
    )
    future = Mock()
    future.result.return_value = SimpleNamespace(
        status=GoalStatus.STATUS_SUCCEEDED,
        result=SimpleNamespace(path=path),
    )

    MissionManager._path_result_received(
        manager, 'inspection', 'uav_1', time.perf_counter(), future
    )

    route = manager._finish_task_path.call_args.args[2]
    assert route
    assert all(pose.pose.position.z == 10.0 for pose in route)


def test_return_failure_reason_is_retained_for_mission_results():
    manager = SimpleNamespace(
        navigation_failures=0,
        return_navigation={'uav_1': {'phase': 'route'}},
        pending_return_uavs={'uav_1'},
        return_minimum_reserve_percentage={'uav_1': 10.0},
        priority_returning_uavs={'uav_1'},
        returned_uavs=set(),
        return_failures=set(),
        return_failure_reasons={},
        _publish_task_status=Mock(),
        get_logger=Mock(return_value=Mock()),
        _report_metrics_if_complete=Mock(),
    )

    MissionManager._finish_return_failed(
        manager, 'uav_1', 'Planner rejected the return path.'
    )

    assert manager.return_failure_reasons == {
        'uav_1': 'Planner rejected the return path.'
    }


def _assignment_manager(state, assigned=(), active=None, pending=()):
    task_manager = TaskManager(
        [_task(task_id) for task_id in assigned], 'map'
    )
    for task_id in assigned:
        task_manager.assign(task_id, 'uav_1')
    manager = SimpleNamespace(
        task_manager=task_manager,
        mission_state=SimpleNamespace(uav_states={'uav_1': SimpleNamespace(
            connected=True, operational=True, mission_status=state
        )}),
        failed_uavs=set(),
        recovery_state=SimpleNamespace(returning=set()),
        return_targets={},
        return_navigation={},
        pending_return_uavs=set(),
        returned_uavs=set(),
        active_navigation=active or {},
        pending_navigation=list(pending),
    )
    manager._uav_is_returning = lambda uav_id: (
        MissionManager._uav_is_returning(manager, uav_id)
    )
    return manager


def test_uav_with_empty_queue_can_start_return_before_global_completion():
    manager = _manager_for_return()

    started = MissionManager._begin_uav_return_locked(
        manager, 'uav_2', 'UAV completed its assigned tasks.'
    )

    assert started
    manager._queue_completion_return.assert_called_once_with(
        'uav_2', 'UAV completed its assigned tasks.'
    )
    assert manager.last_task_altitude == {'uav_2': 7}


def test_uav_with_queued_task_does_not_start_return():
    manager = _manager_for_return([_task('queued')])
    assert manager.task_manager.assign('queued', 'uav_2')[0]

    started = MissionManager._begin_uav_return_locked(
        manager, 'uav_2', 'queue should still be active'
    )

    assert not started
    manager._queue_completion_return.assert_not_called()


def test_mission_return_completion_waits_for_each_uav_independently():
    manager = SimpleNamespace(
        uav_ids=['uav_1', 'uav_2'], returned_uavs={'uav_2'}
    )
    assert not MissionManager._all_uavs_return_terminal(manager)

    manager.returned_uavs.add('uav_1')
    assert MissionManager._all_uavs_return_terminal(manager)


def test_returning_landing_or_landed_uav_is_unavailable_for_assignment():
    manager = SimpleNamespace(
        recovery_state=SimpleNamespace(returning=set()),
        return_targets={},
        return_navigation={},
        pending_return_uavs=set(),
        returned_uavs=set(),
    )
    assert not MissionManager._uav_is_returning(manager, 'uav_2')
    manager.return_targets['uav_2'] = {'name': 'base'}
    assert MissionManager._uav_is_returning(manager, 'uav_2')
    manager.return_targets.clear()
    manager.returned_uavs.add('uav_2')
    assert MissionManager._uav_is_returning(manager, 'uav_2')


def test_idle_uav_is_eligible_only_without_an_assigned_task():
    idle = _assignment_manager(UAVState.IDLE)
    assert MissionManager._peer_has_assignment_slot(idle, 'uav_1')

    with_task = _assignment_manager(UAVState.IDLE, ('current',))
    assert not MissionManager._peer_has_assignment_slot(with_task, 'uav_1')


def test_inspecting_uav_has_exactly_one_additional_task_slot():
    task = _task('current')
    inspecting = _assignment_manager(
        UAVState.INSPECTING,
        ('current',),
        active={'uav_1': {'task': task, 'phase': 'inspection'}},
    )
    assert MissionManager._peer_has_assignment_slot(inspecting, 'uav_1')
    assert inspecting.task_manager.assign('queued', 'uav_1')[0] is False

    queued_task = _task('queued')
    queued_task.priority = 1
    inspecting.task_manager.add_task(queued_task)
    inspecting.task_manager.assign('queued', 'uav_1')
    inspecting.pending_navigation.append((queued_task, 'uav_1'))
    assert not MissionManager._peer_has_assignment_slot(
        inspecting, 'uav_1'
    )


def test_travelling_uav_cannot_receive_another_assignment():
    travelling = _assignment_manager(UAVState.EXECUTING, ('current',))
    assert not MissionManager._peer_has_assignment_slot(
        travelling, 'uav_1'
    )


def test_path_fallback_battery_model_includes_task_return_and_landing():
    task = _task('fallback')
    task.target_position.x = 10.0
    task.target_position.y = 0.0
    task.target_position.z = 10.0
    manager = SimpleNamespace(
        latest_battery_percentage={'uav_1': 50.0},
        battery_updated_at={'uav_1': time.monotonic()},
        latest_estimated_battery_percentage={},
        estimated_battery_updated_at={},
        takeoff_positions={'uav_1': (0.0, 0.0, 0.2)},
        home_positions={'uav_1': (0.0, 0.0, 3.2)},
        standard_flight_altitude=7.0,
        active_navigation={},
        return_distance_safety_factor=1.3,
        battery_consumption_per_meter=0.05,
        inspection_consumption_per_minute=0.25,
        landing_energy_percentage=2.0,
        landing_altitude_m=0.2,
        minimum_safe_battery_reserve=10.0,
        ugv_location=(-5.0, 0.0, 0.2),
        nearby_landing_targets=[],
    )
    manager._current_return_battery = lambda uav_id: (
        MissionManager._current_return_battery(manager, uav_id)
    )

    assert MissionManager._fallback_energy_feasible(
        manager, 'uav_1', task, 10.0
    )
    manager.latest_battery_percentage['uav_1'] = 5.0
    manager.battery_updated_at['uav_1'] = time.monotonic()
    assert not MissionManager._fallback_energy_feasible(
        manager, 'uav_1', task, 10.0
    )
    manager.latest_battery_percentage['uav_1'] = 50.0
    manager.battery_updated_at['uav_1'] = time.monotonic() - 4.0
    assert not MissionManager._fallback_energy_feasible(
        manager, 'uav_1', task, 10.0
    )


def test_uav2_path_failure_checks_uav1_slot_battery_and_planner():
    task = _task('fallback')
    task.priority = 1
    task.target_position.x = 4.0
    task.target_position.y = 2.0
    task.target_position.z = 10.0
    task_manager = TaskManager([task], 'map')
    task_manager.assign(task.task_id, 'uav_2')
    odometry = Odometry()
    odometry.pose.pose.position.x = 1.0
    odometry.pose.pose.position.y = 2.0
    future = Mock()
    planner = Mock()
    planner.server_is_ready.return_value = True
    planner.send_goal_async.return_value = future
    manager = SimpleNamespace(
        path_fallback_attempts={},
        path_fallback_counts={},
        task_assignment_lock=threading.RLock(),
        task_manager=task_manager,
        uav_ids=['uav_1', 'uav_2'],
        active_navigation={
            'uav_2': {'task': task, 'altitude': 7},
        },
        odometry={'uav_1': odometry},
        odometry_received_at={'uav_1': time.monotonic()},
        planner_clients={'uav_1': {7: planner, 10: planner}},
        standard_flight_altitude=7.0,
        frame_id='map',
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            to_msg=lambda: Time()
        )),
        get_logger=lambda: Mock(),
        _publish_task_status=Mock(),
        _fallback_path_goal_received=Mock(),
        _peer_has_assignment_slot=lambda uav_id: uav_id == 'uav_1',
        _fallback_energy_feasible=lambda uav_id, candidate, distance: (
            uav_id == 'uav_1' and distance < 14.0
        ),
    )

    started = MissionManager._start_peer_path_fallback(
        manager, task.task_id, 'uav_2', 'No path on uav_2.'
    )

    assert started
    assert manager.path_fallback_attempts[task.task_id]['peer_uav'] == 'uav_1'
    assert task_manager.statuses[task.task_id] == 'RESERVED'
    assert task.task_id not in task_manager.assignments
    planner.send_goal_async.assert_called_once()
    future.add_done_callback.assert_called_once()


def test_path_fallback_does_not_plan_when_peer_battery_is_infeasible():
    task = _task('fallback')
    task.priority = 1
    task.target_position.x = 4.0
    task.target_position.y = 2.0
    task.target_position.z = 10.0
    task_manager = TaskManager([task], 'map')
    odometry = Odometry()
    planner = Mock()
    planner.server_is_ready.return_value = True
    manager = SimpleNamespace(
        path_fallback_attempts={},
        task_manager=task_manager,
        uav_ids=['uav_1', 'uav_2'],
        active_navigation={},
        odometry={'uav_1': odometry},
        odometry_received_at={'uav_1': time.monotonic()},
        planner_clients={'uav_1': {7: planner, 10: planner}},
        standard_flight_altitude=7.0,
        frame_id='map',
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            to_msg=lambda: Time()
        )),
        get_logger=lambda: Mock(),
        _peer_has_assignment_slot=lambda _uav_id: True,
        _fallback_energy_feasible=lambda _uav_id, _candidate, _distance: (
            False
        ),
    )

    assert not MissionManager._start_peer_path_fallback(
        manager, task.task_id, 'uav_2', 'No path on uav_2.'
    )
    planner.send_goal_async.assert_not_called()


def test_global_return_sweep_requests_each_remaining_uav():
    manager = SimpleNamespace(
        home_return_started=False,
        uav_ids=['uav_1', 'uav_2'],
        _all_tasks_terminal=Mock(return_value=True),
        _begin_uav_return_locked=Mock(return_value=True),
        _report_metrics_if_complete=Mock(),
    )

    MissionManager._begin_return_home_locked(manager)

    assert manager.home_return_started
    assert manager._begin_uav_return_locked.call_count == 2
    manager._report_metrics_if_complete.assert_called_once_with()


def test_unreachable_base_tries_configured_ugv_before_current_position():
    ugv_target = {
        'name': 'UGV',
        'position': (-5.0, 0.0, 0.2),
        'frame_id': 'map',
        'recovery': True,
    }
    manager = SimpleNamespace(
        return_navigation={
            'uav_1': {'target': {'name': 'home_position'}},
        },
        return_targets={'uav_1': {'name': 'home_position'}},
        return_targets_tried={'uav_1': {'home_position'}},
        return_candidates={},
        pending_return_uavs=set(),
        priority_returning_uavs={'uav_1'},
        nearby_landing_targets=[],
        ugv_location=(-5.0, 0.0, 0.2),
        ugv_frame_id='map',
        _single_feasible_return_target=Mock(return_value=ugv_target),
        _finish_return_failed=Mock(),
        recovery_state=SimpleNamespace(begin_return=Mock()),
        get_logger=Mock(return_value=Mock()),
        _publish_task_status=Mock(),
    )

    MissionManager._try_alternate_return_target(
        manager, 'uav_1', 'planner failed at base'
    )

    assert manager.return_targets['uav_1'] == ugv_target
    manager.recovery_state.begin_return.assert_called_once_with('uav_1')
    manager._finish_return_failed.assert_not_called()


def test_ten_percent_actual_route_fallback_uses_only_configured_ugv():
    ugv_target = {
        'name': 'UGV',
        'position': (-5.0, 0.0, 0.2),
        'frame_id': 'map',
        'recovery': True,
    }
    manager = SimpleNamespace(
        return_navigation={
            'uav_1': {'target': {'name': 'home_position'}},
        },
        return_targets={'uav_1': {'name': 'home_position'}},
        return_targets_tried={'uav_1': {'home_position'}},
        return_candidates={},
        pending_return_uavs=set(),
        priority_returning_uavs={'uav_1'},
        nearby_landing_targets=[],
        ugv_location=(-5.0, 0.0, 0.2),
        ugv_frame_id='map',
        _single_feasible_return_target=Mock(return_value=ugv_target),
        _finish_return_failed=Mock(),
        recovery_state=SimpleNamespace(begin_return=Mock()),
        get_logger=Mock(return_value=Mock()),
        _publish_task_status=Mock(),
    )

    MissionManager._try_alternate_return_target(
        manager,
        'uav_1',
        'planned base route leaves exactly five percent',
    )

    selected = manager._single_feasible_return_target.call_args.args[1]
    assert selected == ugv_target
    assert selected['name'] != 'current_position'
    assert manager.return_targets['uav_1'] == ugv_target
    manager._finish_return_failed.assert_not_called()


def test_unreachable_ugv_without_validated_alternative_fails_safely():
    manager = SimpleNamespace(
        return_navigation={'uav_1': {'target': {'name': 'UGV'}}},
        return_targets={'uav_1': {'name': 'UGV'}},
        return_targets_tried={'uav_1': {'home_position', 'UGV'}},
        return_candidates={},
        pending_return_uavs=set(),
        priority_returning_uavs={'uav_1'},
        nearby_landing_targets=[],
        _finish_return_failed=Mock(),
        recovery_state=SimpleNamespace(begin_return=Mock()),
        get_logger=Mock(return_value=Mock()),
        _publish_task_status=Mock(),
    )

    MissionManager._try_alternate_return_target(
        manager, 'uav_1', 'planner failed at UGV'
    )

    manager._finish_return_failed.assert_called_once()
    manager.recovery_state.begin_return.assert_not_called()


def test_navigation_failure_retries_then_keeps_terminal_failure_status():
    task_manager = TaskManager([_task('retry')], 'map')
    assert task_manager.assign('retry', 'uav_1')[0]
    manager = SimpleNamespace(
        navigation_failures=0,
        mission_failed_reasons={},
        task_manager=task_manager,
        maximum_navigation_retries=1,
        active_navigation={'uav_1': {'task': task_manager.tasks['retry']}},
        pending_navigation=[],
        task_assignment_lock=threading.RLock(),
        _publish_task_status=Mock(),
        _signal_allocation_event=Mock(),
        _all_tasks_terminal=lambda: True,
        _begin_return_home_locked=Mock(),
        get_logger=lambda: Mock(),
    )

    MissionManager._fail_task(
        manager, 'retry', 'uav_1', 'first route failed'
    )
    assert manager.mission_failed_reasons == {
        'retry': 'first route failed'
    }

    assert task_manager.statuses['retry'] == 'UNASSIGNED'
    assert manager._publish_task_status.call_args_list[0].args[2] == (
        UAVState.IDLE
    )
    assert manager._signal_allocation_event.call_args.args == ('task_retry',)

    assert task_manager.assign('retry', 'uav_2')[0]
    manager.active_navigation['uav_2'] = {
        'task': task_manager.tasks['retry'],
    }
    MissionManager._fail_task(
        manager, 'retry', 'uav_2', 'second route failed'
    )

    assert task_manager.statuses['retry'] == 'FAILED'
    assert task_manager.failed['retry'] == 'second route failed'
    task_status_calls = manager._publish_task_status.call_args_list
    assert task_status_calls[-2].args[2] == UAVState.FAILED
    assert task_status_calls[-1].args[0] == ''
    manager._begin_return_home_locked.assert_called_once()


def test_inspection_route_requires_climb_completion_before_task_completion():
    task = _task('altitude')
    task.estimated_duration = 1.0
    navigation = {'task': task, 'path': [], 'index': 0}
    manager = SimpleNamespace(
        active_navigation={'uav_1': navigation},
        _publish_task_status=Mock(),
        _signal_allocation_event=Mock(),
    )

    MissionManager._send_next_waypoint(manager, task.task_id, 'uav_1')
    assert navigation['phase'] == 'inspection'
    assert navigation['inspection_end'] > time.monotonic()

    navigation.update({'phase': 'climbing', 'index': 0})
    MissionManager._send_next_waypoint(manager, task.task_id, 'uav_1')
    assert navigation['phase'] == 'climb_complete'


def test_ugv_location_is_loaded_from_mission_recovery_configuration():
    config = load_mission_config()
    mission = config['mission']
    recovery = mission['recovery']
    allocator_path = (
        Path(__file__).resolve().parents[2]
        / 'uav_allocator' / 'config' / 'allocator.yaml'
    )
    with allocator_path.open(encoding='utf-8') as allocator_file:
        allocator_parameters = yaml.safe_load(allocator_file)[
            '/**'
        ]['ros__parameters']

    assert tuple(recovery['ugv_location']) == (-5.0, 0.0, 0.2)
    assert (
        allocator_parameters['post_landing_reserve_percentage']
        == recovery['post_landing_reserve_percent']
    )
    assert tuple(allocator_parameters['recovery_position']) == tuple(
        recovery['ugv_location']
    )
    assert allocator_parameters['base_positions'] == {
        uav['id']: uav['home_position'] for uav in mission['uavs']
    }


def test_battery_callback_keeps_each_uav_reading_separate():
    manager = SimpleNamespace(
        latest_battery_percentage={},
        battery_updated_at={},
        energy_samples={},
        energy_consumed_wh={},
        energy_interval_counts={},
        low_battery_threshold=30.0,
        _start_low_battery_return=Mock(),
    )
    battery = BatteryState()
    battery.present = True
    battery.percentage = 0.86

    MissionManager.battery_callback(manager, battery, 'uav_2')

    assert manager.latest_battery_percentage == {'uav_2': 86.0}
    assert 'uav_1' not in manager.latest_battery_percentage


def test_stale_measured_battery_is_not_used_even_with_fresh_estimate(
    monkeypatch,
):
    from uav_mission import mission_manager as mission_manager_module

    monkeypatch.setattr(
        mission_manager_module.time, 'monotonic', lambda: 100.0
    )
    manager = SimpleNamespace(
        latest_battery_percentage={'uav_1': 87.0, 'uav_2': 91.0},
        battery_updated_at={'uav_1': 99.5, 'uav_2': 96.0},
        latest_estimated_battery_percentage={
            'uav_1': 30.0, 'uav_2': 20.0,
        },
        estimated_battery_updated_at={
            'uav_1': 96.0, 'uav_2': 99.8,
        },
    )

    assert MissionManager._current_return_battery(manager, 'uav_1') == 87.0
    assert MissionManager._current_return_battery(manager, 'uav_2') is None
