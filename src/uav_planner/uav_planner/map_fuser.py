import math
from copy import deepcopy
import rclpy
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy,HistoryPolicy,QoSProfile,ReliabilityPolicy,)
from uav_planner.map_generator import fuse_occupancy_grids

def apply_xy_bounds(data, width, height, resolution, origin, bounds):
    bounded = list(data)
    origin_x, origin_y = origin
    for row in range(height):
        y = origin_y + (row + 0.5) * resolution
        for column in range(width):
            x = origin_x + (column + 0.5) * resolution
            if not (
                bounds['x_min'] <= x <= bounds['x_max']
                and bounds['y_min'] <= y <= bounds['y_max']
            ):
                bounded[row * width + column] = 100
    return bounded

class AltitudeMapFuser(Node):
    def __init__(self):
        super().__init__('altitude_map_fuser')
        self.declare_parameter('structural_map_topic', '')
        self.declare_parameter('base_map_topic', '/map')
        self.declare_parameter('output_map_topic', '')
        self.declare_parameter('operating_bounds.enabled', False)
        self.declare_parameter('operating_bounds.x_min', 0.0)
        self.declare_parameter('operating_bounds.x_max', 0.0)
        self.declare_parameter('operating_bounds.y_min', 0.0)
        self.declare_parameter('operating_bounds.y_max', 0.0)
        structural_topic = self.get_parameter(
            'structural_map_topic'
        ).get_parameter_value().string_value
        base_topic = self.get_parameter(
            'base_map_topic'
        ).get_parameter_value().string_value
        output_topic = self.get_parameter(
            'output_map_topic'
        ).get_parameter_value().string_value
        if not structural_topic or not output_topic:
            raise ValueError('Structural and output map topics are required')

        transient_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        input_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.structural_map = None
        self.base_map = None
        self.publisher = self.create_publisher(
            OccupancyGrid, output_topic, transient_qos
        )
        self.structural_subscription = self.create_subscription(
            OccupancyGrid, structural_topic, self._structural_callback,
            transient_qos,
        )
        self.base_subscription = self.create_subscription(
            OccupancyGrid, base_topic, self._base_callback, input_qos
        )
        self.add_on_set_parameters_callback(
            self._validate_operating_bounds_parameters
        )

    @staticmethod
    def _grid(message):
        expected = int(message.info.width) * int(message.info.height)
        if message.info.width <= 0 or message.info.height <= 0:
            raise ValueError('OccupancyGrid dimensions must be positive')
        if len(message.data) != expected:
            raise ValueError(
                f'OccupancyGrid data length {len(message.data)} does not match '
                f'{message.info.width}x{message.info.height}'
            )
        if message.info.resolution <= 0.0:
            raise ValueError('OccupancyGrid resolution must be positive')
        return {
            'frame_id': message.header.frame_id,
            'resolution': message.info.resolution,
            'width': message.info.width,
            'height': message.info.height,
            'origin': [
                message.info.origin.position.x,
                message.info.origin.position.y,
                message.info.origin.position.z,
            ],
            'data': list(message.data),
        }

    def _structural_callback(self, message):
        self.structural_map = message
        self._publish_fused()

    def _base_callback(self, message):
        self.base_map = message
        self._publish_fused()

    def _publish_fused(self):
        if self.structural_map is None:
            return
        output = deepcopy(self.base_map or self.structural_map)
        try:
            if (
                self.base_map is not None
                and self.base_map.header.frame_id
                != self.structural_map.header.frame_id
            ):
                raise ValueError(
                    'SLAM and structural map frame IDs do not match'
                )
            merged = fuse_occupancy_grids(
                self._grid(self.base_map) if self.base_map is not None else None,
                self._grid(self.structural_map),
            )
        except (KeyError, ValueError, IndexError) as error:
            self.get_logger().error(f'Could not fuse navigation maps: {error}')
            return
        if self.get_parameter(
            'operating_bounds.enabled'
        ).get_parameter_value().bool_value:
            bounds = {
                name: self.get_parameter(
                    f'operating_bounds.{name}'
                ).get_parameter_value().double_value
                for name in ('x_min', 'x_max', 'y_min', 'y_max')
            }
            merged['data'] = apply_xy_bounds(
                merged['data'],
                merged['width'],
                merged['height'],
                merged['resolution'],
                merged['origin'][:2],
                bounds,
            )
        output.data = merged['data']
        output.info.width = merged['width']
        output.info.height = merged['height']
        output.info.resolution = merged['resolution']
        output.info.origin.position.x = merged['origin'][0]
        output.info.origin.position.y = merged['origin'][1]
        output.info.origin.position.z = merged['origin'][2]
        output.header.frame_id = merged.get(
            'frame_id', output.header.frame_id
        )
        output.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(output)

    def _validate_operating_bounds_parameters(self, parameters):
        names = ('x_min', 'x_max', 'y_min', 'y_max')
        bounds = {
            name: self.get_parameter(
                f'operating_bounds.{name}'
            ).get_parameter_value().double_value
            for name in names
        }
        enabled = self.get_parameter(
            'operating_bounds.enabled'
        ).get_parameter_value().bool_value
        for parameter in parameters:
            prefix = 'operating_bounds.'
            if parameter.name == f'{prefix}enabled':
                enabled = parameter.value
            elif parameter.name.startswith(prefix):
                name = parameter.name[len(prefix):]
                if name in bounds:
                    bounds[name] = parameter.value
        if enabled and (
            not all(math.isfinite(value) for value in bounds.values())
            or bounds['x_min'] >= bounds['x_max']
            or bounds['y_min'] >= bounds['y_max']
        ):
            return SetParametersResult(
                successful=False,
                reason='Operating-region minima must be below maxima',
            )
        if any(
            parameter.name.startswith('operating_bounds.')
            for parameter in parameters
        ):
            timer_holder = {}

            def republish():
                timer = timer_holder['timer']
                timer.cancel()
                self.destroy_timer(timer)
                self._publish_fused()

            timer_holder['timer'] = self.create_timer(0.01, republish)
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = AltitudeMapFuser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
