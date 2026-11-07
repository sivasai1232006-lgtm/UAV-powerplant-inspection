import math


def generate_grid(obstacles, config):
    resolution = float(config['resolution'])
    x_min, x_max = float(config['x_min']), float(config['x_max'])
    y_min, y_max = float(config['y_min']), float(config['y_max'])
    if resolution <= 0.0 or x_max <= x_min or y_max <= y_min:
        raise ValueError('Map bounds and resolution are invalid')
    width = math.ceil((x_max - x_min) / resolution)
    height = math.ceil((y_max - y_min) / resolution)
    grid = [int(config['free_value'])] * (width * height)
    altitude = float(config['flight_altitude'])
    vertical_clearance = float(config['uav_safety_radius']) + float(
        config['additional_clearance']
    )
    horizontal_inflation = vertical_clearance

    for obstacle in obstacles:
        xmin, ymin, zmin, xmax, ymax, zmax = obstacle['bounds']
        if zmax < altitude - vertical_clearance or zmin > altitude + vertical_clearance:
            continue
        xmin -= horizontal_inflation
        ymin -= horizontal_inflation
        xmax += horizontal_inflation
        ymax += horizontal_inflation
        first_x = max(0, math.floor((xmin - x_min) / resolution))
        last_x = min(width - 1, math.floor((xmax - x_min) / resolution))
        first_y = max(0, math.floor((ymin - y_min) / resolution))
        last_y = min(height - 1, math.floor((ymax - y_min) / resolution))
        if last_x < first_x or last_y < first_y:
            continue
        for row in range(first_y, last_y + 1):
            start = row * width + first_x
            grid[start:start + last_x - first_x + 1] = [
                int(config['occupied_value'])
            ] * (last_x - first_x + 1)

    return {
        'frame_id': str(config['world_frame']),
        'resolution': resolution,
        'width': width,
        'height': height,
        'origin': [x_min, y_min, 0.0],
        'flight_altitude': altitude,
        'data': grid,
    }
