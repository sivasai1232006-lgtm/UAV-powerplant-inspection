import math

from uav_interfaces.msg import Task


def tasks_from_mission(mission_data):
    mission = mission_data['mission']
    tasks = []

    for task_data in mission['tasks']:
        task = Task()
        task.task_id = task_data['id']
        task.target_position.x = float(task_data['waypoint'][0])
        task.target_position.y = float(task_data['waypoint'][1])
        task.target_position.z = float(task_data['waypoint'][2])
        task.priority = int(task_data['priority'])
        task.estimated_duration = float(task_data['estimated_duration_s'])
        tasks.append(task)

    return tasks


class TaskManager:
    def __init__(self, tasks):
        self.tasks = {task.task_id: task for task in tasks}
        self.assignments = {}
        self.assignment_distances = {}
        self.completed = set()
        self.failed = {}

    def unassigned_tasks(self):
        return [
            task for task_id, task in self.tasks.items()
            if task_id not in self.assignments
            and task_id not in self.completed
            and task_id not in self.failed
        ]

    def assign(
        self, task_id, uav_id, outbound_distance_m=0.0,
        return_distance_m=0.0,
    ):
        if task_id not in self.tasks:
            return False, f"Task '{task_id}' does not exist."
        if task_id in self.assignments:
            return False, f"Task '{task_id}' is already assigned."
        if any(
            assigned_uav == uav_id and assigned_task not in self.completed
            for assigned_task, assigned_uav in self.assignments.items()
        ):
            return False, f"UAV '{uav_id}' already has an assigned task."
        distances = (outbound_distance_m, return_distance_m)
        if any(
            not math.isfinite(value) or value < 0.0 for value in distances
        ):
            return False, 'Planned distances must be finite and non-negative.'

        self.assignments[task_id] = uav_id
        self.assignment_distances[task_id] = tuple(
            float(value) for value in distances
        )
        return True, f"Task '{task_id}' assigned to UAV '{uav_id}'."

    def complete(self, task_id, uav_id):
        if self.assignments.get(task_id) != uav_id:
            return False
        self.completed.add(task_id)
        return True

    def fail(self, task_id, reason):
        if task_id not in self.tasks or task_id in self.completed:
            return False
        self.assignments.pop(task_id, None)
        self.assignment_distances.pop(task_id, None)
        self.failed[task_id] = str(reason)
        return True

    def release(self, task_id, uav_id):
        if self.assignments.get(task_id) != uav_id:
            return False
        del self.assignments[task_id]
        self.assignment_distances.pop(task_id, None)
        return True
