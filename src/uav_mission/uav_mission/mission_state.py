import time

from uav_interfaces.msg import UAVState


class MissionState:
    def __init__(self, uav_ids, state_timeout_sec=3.0):
        if state_timeout_sec <= 0.0:
            raise ValueError('State timeout must be greater than zero')

        self.uav_ids = set(uav_ids)
        self.state_timeout_sec = float(state_timeout_sec)
        self.uav_states = {}
        self.last_state_update = {}

    def update(self, uav_state):
        if uav_state.uav_id not in self.uav_ids:
            return False

        self.uav_states[uav_state.uav_id] = uav_state
        self.last_state_update[uav_state.uav_id] = time.monotonic()
        return True

    def operational_uavs(self):
        now = time.monotonic()
        return [
            state
            for uav_id, state in self.uav_states.items()
            if now - self.last_state_update[uav_id] <= self.state_timeout_sec
            and state.connected
            and state.operational
        ]

    def available_uavs(self):
        ready_statuses = (UAVState.IDLE, UAVState.COMPLETED)
        return [
            state
            for state in self.operational_uavs()
            if state.mission_status in ready_statuses
        ]
