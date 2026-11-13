import math

def _mesh_bins(triangles, x_min, x_max, y_min, y_max, bin_size=1.0):
    bins = {}
    for triangle in triangles:
        xs = [point[0] for point in triangle]
        ys = [point[1] for point in triangle]
        x0 = max(x_min, min(xs))
        x1 = min(x_max, max(xs))
        y0 = max(y_min, min(ys))
        y1 = min(y_max, max(ys))
        if x0 > x1 or y0 > y1:
            continue
        for bx in range(math.floor(x0 / bin_size),
                        math.floor(x1 / bin_size) + 1):
            for by in range(math.floor(y0 / bin_size),
                            math.floor(y1 / bin_size) + 1):
                bins.setdefault((bx, by), []).append(triangle)
    return bins

def _triangle_height(triangle, x, y):
    (x1, y1, z1), (x2, y2, z2), (x3, y3, z3) = triangle
    denominator = (y2 - y3) * (x1 - x3) + (x3 - x2) * (y1 - y3)
    if abs(denominator) < 1e-10:
        return None
    a = ((y2 - y3) * (x - x3) + (x3 - x2) * (y - y3)) / denominator
    b = ((y3 - y1) * (x - x3) + (x1 - x3) * (y - y3)) / denominator
    c = 1.0 - a - b
    if min(a, b, c) < -1e-7:
        return None
    return a * z1 + b * z2 + c * z3

def _mesh_column_intersects(bins, x, y, lower_z, upper_z):
    hits = []
    for triangle in bins.get((math.floor(x), math.floor(y)), ()):
        height = _triangle_height(triangle, x, y)
        if height is not None:
            hits.append(height)
    if not hits:
        return False

    hits.sort()
    unique = []
    for height in hits:
        if not unique or abs(height - unique[-1]) > 1e-4:
            unique.append(height)

    for index in range(0, len(unique) - 1, 2):
        if unique[index] <= upper_z and unique[index + 1] >= lower_z:
            return True
    return False

def _clip_triangle(triangle, lower_z, upper_z):
    polygon = list(triangle)
    for boundary, keep_above in ((lower_z, True), (upper_z, False)):
        if not polygon:
            break
        clipped = []
        previous = polygon[-1]
        previous_inside = (
            previous[2] >= boundary if keep_above
            else previous[2] <= boundary
        )
        for current in polygon:
            current_inside = (
                current[2] >= boundary if keep_above
                else current[2] <= boundary
            )
            if current_inside != previous_inside:
                fraction = (boundary - previous[2]) / (
                    current[2] - previous[2]
                )
                clipped.append(tuple(
                    previous[axis]
                    + fraction * (current[axis] - previous[axis])
                    for axis in range(3)
                ))
            if current_inside:
                clipped.append(current)
            previous, previous_inside = current, current_inside
        polygon = clipped
    return polygon

def _projected_mesh_bins(triangles, lower_z, upper_z, bin_size=1.0,
                         raster_margin=0.0):
    bins = {}
    for triangle in triangles:
        polygon = _clip_triangle(triangle, lower_z, upper_z)
        if not polygon:
            continue
 
        x0 = min(p[0] for p in polygon) - raster_margin
        x1 = max(p[0] for p in polygon) + raster_margin
        y0 = min(p[1] for p in polygon) - raster_margin
        y1 = max(p[1] for p in polygon) + raster_margin
        for bx in range(math.floor(x0 / bin_size),
                        math.floor(x1 / bin_size) + 1):
            for by in range(math.floor(y0 / bin_size),
                            math.floor(y1 / bin_size) + 1):
                bins.setdefault((bx, by), []).append(polygon)
    return bins

def _point_segment_distance(x, y, first, second):
    dx, dy = second[0] - first[0], second[1] - first[1]
    length_squared = dx * dx + dy * dy
    if length_squared <= 1e-12:
        return math.hypot(x - first[0], y - first[1])
    fraction = max(0.0, min(1.0, (
        (x - first[0]) * dx + (y - first[1]) * dy
    ) / length_squared))
    return math.hypot(
        x - first[0] - fraction * dx,
        y - first[1] - fraction * dy,
    )

def _inside_projected_polygon(x, y, polygon, raster_margin):
    inside = False
    for index, first in enumerate(polygon):
        second = polygon[(index + 1) % len(polygon)]
        if _point_segment_distance(x, y, first, second) <= raster_margin:
            return True
        if (first[1] > y) != (second[1] > y):
            crossing_x = (
                (second[0] - first[0]) * (y - first[1])
                / (second[1] - first[1]) + first[0]
            )
            if x < crossing_x:
                inside = not inside
    return inside

def generate_grid(obstacles, config, base_grid=None):
    resolution = float(config['resolution'])
    x_min, x_max = float(config['x_min']), float(config['x_max'])
    y_min, y_max = float(config['y_min']), float(config['y_max'])
    if resolution <= 0.0 or x_max <= x_min or y_max <= y_min:
        raise ValueError('Map bounds and resolution are invalid')
    width = math.ceil((x_max - x_min) / resolution)
    height = math.ceil((y_max - y_min) / resolution)
    if width <= 0 or height <= 0:
        raise ValueError('Generated map dimensions must be positive')
    if width * height > 100_000_000:
        raise ValueError('Requested map is too large to rasterize safely')
    grid = [int(config['free_value'])] * (width * height)
    altitude = float(config['flight_altitude'])
    vertical_extent = float(config.get('uav_vertical_half_extent', 0.25))
    vertical_clearance = float(config.get('vertical_clearance',
                                            config.get('additional_clearance', 0.25)))
    if vertical_extent < 0.0 or vertical_clearance < 0.0:
        raise ValueError('Vertical UAV extent and clearance cannot be negative')
    if not math.isfinite(altitude + vertical_extent + vertical_clearance):
        raise ValueError('Altitude and vertical clearances must be finite')
    lower_z = altitude - vertical_extent - vertical_clearance
    upper_z = altitude + vertical_extent + vertical_clearance
    occupied = int(config['occupied_value'])

    for obstacle in obstacles:
        xmin, ymin, zmin, xmax, ymax, zmax = obstacle['bounds']
        if zmax < lower_z or zmin > upper_z:
            continue
        first_x = max(0, math.floor((xmin - x_min) / resolution))
        last_x = min(width - 1, math.floor((xmax - x_min) / resolution))
        first_y = max(0, math.floor((ymin - y_min) / resolution))
        last_y = min(height - 1, math.floor((ymax - y_min) / resolution))
        if last_x < first_x or last_y < first_y:
            continue

        triangles = obstacle.get('triangles')
        if not triangles:

            for row in range(first_y, last_y + 1):
                start = row * width + first_x
                grid[start:start + last_x - first_x + 1] = [
                    occupied
                ] * (last_x - first_x + 1)
            continue

        raster_margin = resolution * math.sqrt(2.0) / 2.0
        bins = _projected_mesh_bins(
            triangles, lower_z, upper_z, raster_margin=raster_margin
        )
        ray_bins = _mesh_bins(triangles, xmin, xmax, ymin, ymax)
        for row in range(first_y, last_y + 1):
            y = y_min + (row + 0.5) * resolution
            for column in range(first_x, last_x + 1):
                x = x_min + (column + 0.5) * resolution
                polygons = bins.get((math.floor(x), math.floor(y)), ())
                if (
                    _mesh_column_intersects(
                        ray_bins, x, y, lower_z, upper_z
                    )
                    or any(
                        _inside_projected_polygon(
                            x, y, polygon, raster_margin
                        )
                        for polygon in polygons
                    )
                ):
                    grid[row * width + column] = occupied

    if base_grid is not None:
        _merge_base_occupancy(grid, width, height, resolution,
                              (x_min, y_min), base_grid, occupied,
                              int(config['free_value']),
                              int(config.get('unknown_value', -1)))

    return {
        'frame_id': str(config['world_frame']),
        'resolution': resolution,
        'width': width,
        'height': height,
        'origin': [x_min, y_min, 0.0],
        'flight_altitude': altitude,
        'vertical_band': [lower_z, upper_z],
        'data': grid,
    }

def _merge_base_occupancy(target, width, height, resolution, origin,base_grid, occupied, free_value, unknown_value):
    if (base_grid['width'] != width or base_grid['height'] != height
            or abs(float(base_grid['resolution']) - resolution) > 1e-9
            or tuple(base_grid['origin'][:2]) != tuple(origin)):
        raise ValueError('Base map metadata must match the structural grid')
    for index, value in enumerate(base_grid['data']):
        if value < 0:
            if target[index] == free_value:
                target[index] = unknown_value
        elif value >= 50:
            target[index] = occupied

def fuse_occupancy_grids(base_grid, structural_grid):
    if base_grid is None:
        return dict(structural_grid)
    resolution = float(base_grid['resolution'])
    structure_resolution = float(structural_grid['resolution'])
    base_origin = base_grid['origin']
    structure_origin = structural_grid['origin']
    base_max_x = base_origin[0] + base_grid['width'] * resolution
    base_max_y = base_origin[1] + base_grid['height'] * resolution
    structure_max_x = (structure_origin[0] + structural_grid['width'] * structure_resolution)
    structure_max_y = (structure_origin[1] + structural_grid['height'] * structure_resolution)
    origin_x = min(base_origin[0], structure_origin[0])
    origin_y = min(base_origin[1], structure_origin[1])
    width = math.ceil((max(base_max_x, structure_max_x) - origin_x) / resolution)
    height = math.ceil((max(base_max_y, structure_max_y) - origin_y) / resolution)
    unknown = -1
    data = [unknown] * (width * height)

    def overlay(source, lethal_only=False):
        source_resolution = float(source['resolution'])
        for row in range(source['height']):
            y0 = source['origin'][1] + row * source_resolution
            y1 = y0 + source_resolution
            first_row = math.floor((y0 - origin_y) / resolution)
            last_row = math.floor((y1 - origin_y - 1e-10) / resolution)
            for column in range(source['width']):
                value = source['data'][row * source['width'] + column]
                if lethal_only and value < 50:
                    continue
                x0 = source['origin'][0] + column * source_resolution
                x1 = x0 + source_resolution
                first_column = math.floor((x0 - origin_x) / resolution)
                last_column = math.floor((x1 - origin_x - 1e-10) / resolution)
                for target_row in range(first_row, last_row + 1):
                    if not 0 <= target_row < height:
                        continue
                    for target_column in range(first_column,
                                               last_column + 1):
                        if not 0 <= target_column < width:
                            continue
                        index = target_row * width + target_column
                        if value >= 50:
                            data[index] = 100
                        elif not lethal_only and data[index] != 100:
                            if value >= 0 or data[index] == unknown:
                                data[index] = value

    overlay(base_grid)
    overlay(structural_grid, lethal_only=True)
    result = dict(base_grid)
    result.update({'width': width,'height': height,'resolution': resolution,'origin': [origin_x, origin_y, base_origin[2]],})
    result['data'] = data
    return result