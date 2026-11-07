from uav_interfaces.msg import Task

from uav_mission.task_manager import TaskManager


def make_task(task_id):
    task = Task()
    task.task_id = task_id
    return task


def test_assigned_tasks_are_removed_from_unassigned_list():
    manager = TaskManager([make_task('task-a'), make_task('task-b')])

    success, _ = manager.assign('task-a', 'uav1')

    assert success
    assert [task.task_id for task in manager.unassigned_tasks()] == ['task-b']


def test_task_and_uav_cannot_receive_duplicate_assignments():
    manager = TaskManager([make_task('task-a'), make_task('task-b')])

    assert manager.assign('task-a', 'uav1')[0]
    assert not manager.assign('task-a', 'uav2')[0]
    assert not manager.assign('task-b', 'uav1')[0]


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


def test_assignment_keeps_planned_outbound_and_return_distances():
    manager = TaskManager([make_task('task-a')])

    success, _ = manager.assign('task-a', 'uav_1', 12.5, 8.25)

    assert success
    assert manager.assignment_distances['task-a'] == (12.5, 8.25)
