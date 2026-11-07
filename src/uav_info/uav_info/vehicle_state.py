import threading
import time
from typing import Any, Literal, overload

from px4_msgs.msg import (
    BatteryStatus,
    VehicleAttitude,
    VehicleGlobalPosition,
    VehicleLandDetected,
    VehicleLocalPosition,
    VehicleStatus,
)
from uav_interfaces.msg import UAVState


class VehicleState:
    def __init__(self, uav_id):
        self.uav_id = uav_id
        self.mission_status = UAVState.IDLE
        self.assigned_task_id = ''
        self._values: dict[str, Any] = {}
        self._updated_at = {}
        self._lock = threading.RLock()

    def update(self, field, value):
        with self._lock:
            self._values[field] = value
            self._updated_at[field] = time.monotonic()

    @overload
    def get(
        self, field: Literal['global_position']
    ) -> VehicleGlobalPosition | None: ...

    @overload
    def get(
        self, field: Literal['local_position']
    ) -> VehicleLocalPosition | None: ...

    @overload
    def get(
        self, field: Literal['attitude']
    ) -> VehicleAttitude | None: ...

    @overload
    def get(
        self, field: Literal['battery_status']
    ) -> BatteryStatus | None: ...

    @overload
    def get(
        self, field: Literal['vehicle_status']
    ) -> VehicleStatus | None: ...

    @overload
    def get(
        self, field: Literal['land_detected']
    ) -> VehicleLandDetected | None: ...

    def get(self, field: str) -> Any:
        with self._lock:
            return self._values.get(field)

    def is_fresh(self, field, timeout_sec, now=None):
        now = time.monotonic() if now is None else now
        with self._lock:
            received_at = self._updated_at.get(field)
        return received_at is not None and now - received_at <= timeout_sec

    def snapshot(self):
        with self._lock:
            return dict(self._values)
