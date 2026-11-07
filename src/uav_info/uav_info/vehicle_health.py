from px4_msgs.msg import BatteryStatus


class VehicleHealth:
    def __init__(self, telemetry_timeout_sec, minimum_battery_percentage):
        self.telemetry_timeout_sec = float(telemetry_timeout_sec)
        self.minimum_battery_percentage = float(minimum_battery_percentage)

    def evaluate(self, vehicle_state):
        connected = vehicle_state.is_fresh(
            'vehicle_status', self.telemetry_timeout_sec
        )
        global_position = vehicle_state.get('global_position')
        local_position = vehicle_state.get('local_position')
        battery = vehicle_state.get('battery_status')
        vehicle_status = vehicle_state.get('vehicle_status')

        sensors_fresh = all(
            vehicle_state.is_fresh(name, self.telemetry_timeout_sec)
            for name in (
                'global_position',
                'local_position',
                'attitude',
                'battery_status',
            )
        )
        position_valid = (
            global_position is not None
            and global_position.lat_lon_valid
            and global_position.alt_valid
            and local_position is not None
            and local_position.xy_valid
            and local_position.z_valid
            and local_position.v_xy_valid
            and local_position.v_z_valid
        )
        battery_valid = (
            battery is not None
            and battery.connected
            and 0.0 <= battery.remaining <= 1.0
            and battery.remaining * 100.0
            >= self.minimum_battery_percentage
            and battery.warning not in (
                BatteryStatus.WARNING_CRITICAL,
                BatteryStatus.WARNING_EMERGENCY,
                BatteryStatus.WARNING_FAILED,
                BatteryStatus.WARNING_UNHEALTHY,
            )
        )
        no_failsafe = (
            vehicle_status is not None and not vehicle_status.failsafe
        )
        operational = (
            connected
            and sensors_fresh
            and position_valid
            and battery_valid
            and no_failsafe
        )
        return connected, operational
