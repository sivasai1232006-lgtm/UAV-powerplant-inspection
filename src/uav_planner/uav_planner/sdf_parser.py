import math
import struct
import xml.etree.ElementTree as ET
from pathlib import Path


def _values(element, default=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)):
    if element is None or not (element.text or '').strip():
        return default
    values = tuple(float(value) for value in element.text.split())
    return values + (0.0,) * (6 - len(values))


def _rotate(point, pose):
    x, y, z = point
    _, _, _, roll, pitch, yaw = pose
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp) * x + (cy * sp * sr - sy * cr) * y + (cy * sp * cr + sy * sr) * z,
        (sy * cp) * x + (sy * sp * sr + cy * cr) * y + (sy * sp * cr - cy * sr) * z,
        (-sp) * x + cp * sr * y + cp * cr * z,
    )


def _transform(point, pose):
    rotated = _rotate(point, pose)
    return tuple(rotated[index] + pose[index] for index in range(3))


def _compose(parent, child):
    translated = _transform(child[:3], parent)
    def matrix(pose):
        roll, pitch, yaw = pose[3:]
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        return (
            (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr),
        )

    first, second = matrix(parent), matrix(child)
    rotation = tuple(
        tuple(sum(first[row][axis] * second[axis][column] for axis in range(3))
              for column in range(3))
        for row in range(3)
    )
    pitch = math.asin(max(-1.0, min(1.0, -rotation[2][0])))
    if abs(math.cos(pitch)) > 1e-8:
        roll = math.atan2(rotation[2][1], rotation[2][2])
        yaw = math.atan2(rotation[1][0], rotation[0][0])
    else:
        roll = 0.0
        yaw = math.atan2(-rotation[0][1], rotation[1][1])
    return translated + (roll, pitch, yaw)


def _mesh_vertices(path):
    content = path.read_bytes()
    if len(content) >= 84:
        count = struct.unpack_from('<I', content, 80)[0]
        if 84 + count * 50 == len(content):
            vertices = []
            offset = 84
            for _ in range(count):
                offset += 12
                for _ in range(3):
                    vertices.append(struct.unpack_from('<3f', content, offset))
                    offset += 12
                offset += 2
            return vertices
    text = content.decode('ascii', errors='ignore')
    vertices = []
    for line in text.splitlines():
        parts = line.split()
        if len(parts) == 4 and parts[0] == 'vertex':
            vertices.append(tuple(float(value) for value in parts[1:]))
    if not vertices:
        raise ValueError(f'Unable to read STL mesh vertices: {path}')
    return vertices


def _geometry_points(geometry, model_dir):
    box = geometry.find('box/size')
    if box is not None:
        size = tuple(float(value) / 2.0 for value in box.text.split())
        return [(x, y, z) for x in (-size[0], size[0])
                for y in (-size[1], size[1]) for z in (-size[2], size[2])]
    cylinder = geometry.find('cylinder')
    if cylinder is not None:
        radius = float(cylinder.findtext('radius'))
        half_length = float(cylinder.findtext('length')) / 2.0
        return [(x, y, z) for x in (-radius, radius)
                for y in (-radius, radius) for z in (-half_length, half_length)]
    sphere = geometry.find('sphere')
    if sphere is not None:
        radius = float(sphere.findtext('radius'))
        return [(x, y, z) for x in (-radius, radius)
                for y in (-radius, radius) for z in (-radius, radius)]
    mesh = geometry.find('mesh')
    if mesh is not None:
        uri = (mesh.findtext('uri') or '').strip()
        mesh_path = model_dir / uri
        if not mesh_path.is_file():
            raise FileNotFoundError(f'Mesh not found: {mesh_path}')
        scale = tuple(float(value) for value in (mesh.findtext('scale') or '1 1 1').split())
        return [tuple(point[index] * scale[index] for index in range(3))
                for point in _mesh_vertices(mesh_path)]
    raise ValueError('Unsupported collision geometry in SDF')


def load_world_collisions(world_path, models_dir):
    world_path = Path(world_path)
    models_dir = Path(models_dir)
    root = ET.parse(world_path).getroot()
    world = root.find('world')
    obstacles = []
    for include in world.findall('include'):
        uri = (include.findtext('uri') or '').strip()
        if not uri.startswith('model://uav_sim/models/'):
            continue
        model_dir = models_dir / uri.rsplit('/', 1)[-1]
        model_root = ET.parse(model_dir / 'model.sdf').getroot()
        model = model_root.find('model')
        include_pose = _values(include.find('pose'))
        model_pose = _compose(include_pose, _values(model.find('pose')))
        for link in model.findall('link'):
            link_pose = _compose(model_pose, _values(link.find('pose')))
            for collision in link.findall('collision'):
                collision_pose = _compose(link_pose, _values(collision.find('pose')))
                points = _geometry_points(collision.find('geometry'), model_dir)
                world_points = [_transform(point, collision_pose) for point in points]
                obstacles.append({
                    'name': include.findtext('name') or model.get('name'),
                    'bounds': [
                        min(point[axis] for point in world_points)
                        for axis in range(3)
                    ] + [
                        max(point[axis] for point in world_points)
                        for axis in range(3)
                    ],
                })
    return obstacles
