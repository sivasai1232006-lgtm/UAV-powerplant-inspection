import math
import struct
import xml.etree.ElementTree as ET
from pathlib import Path


def _values(element, default=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)):
    if element is None or not (element.text or '').strip():
        return default
    values = tuple(float(value) for value in element.text.split())
    return values + (0.0,) * (6 - len(values))


def _required_float(element, path, context):
    text = element.findtext(path)
    if text is None or not text.strip():
        raise ValueError(f'Missing {path} value in {context}')
    return float(text)


def _rotate(point, pose):
    x, y, z = point
    _, _, _, roll, pitch, yaw = pose
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp) * x
        + (cy * sp * sr - sy * cr) * y
        + (cy * sp * cr + sy * sr) * z,
        (sy * cp) * x
        + (sy * sp * sr + cy * cr) * y
        + (sy * sp * cr - cy * sr) * z,
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


def _mesh_bounds(path):
    """Read mesh extents without retaining every triangle vertex."""
    content = path.read_bytes()
    bounds = [[math.inf, -math.inf] for _ in range(3)]

    def include(point):
        for axis, value in enumerate(point):
            bounds[axis][0] = min(bounds[axis][0], value)
            bounds[axis][1] = max(bounds[axis][1], value)

    if len(content) >= 84:
        count = struct.unpack_from('<I', content, 80)[0]
        if 84 + count * 50 == len(content):
            for triangle in struct.iter_unpack('<12fH', content[84:]):
                include(triangle[3:6])
                include(triangle[6:9])
                include(triangle[9:12])
    if any(not math.isfinite(axis[0]) for axis in bounds):
        for line in content.decode('ascii', errors='ignore').splitlines():
            parts = line.split()
            if len(parts) == 4 and parts[0] == 'vertex':
                include(tuple(float(value) for value in parts[1:]))
    if any(not math.isfinite(axis[0]) for axis in bounds):
        raise ValueError(f'Unable to read STL mesh vertices: {path}')
    return [
        (x, y, z)
        for x in bounds[0]
        for y in bounds[1]
        for z in bounds[2]
    ]


def _mesh_triangles(path):
    """Load collision-mesh triangles from binary or ASCII STL."""
    content = path.read_bytes()
    if len(content) >= 84:
        count = struct.unpack_from('<I', content, 80)[0]
        if 84 + count * 50 == len(content):
            return [
                (triangle[3:6], triangle[6:9], triangle[9:12])
                for triangle in struct.iter_unpack('<12fH', content[84:])
            ]

    triangles = []
    vertices = []
    for line in content.decode('ascii', errors='ignore').splitlines():
        parts = line.split()
        if len(parts) == 4 and parts[0] == 'vertex':
            vertices.append(tuple(float(value) for value in parts[1:]))
            if len(vertices) == 3:
                triangles.append(tuple(vertices))
                vertices = []
    if not triangles:
        raise ValueError(f'Unable to read STL mesh triangles: {path}')
    return triangles


def _geometry_points(geometry, model_dir):
    if geometry is None:
        raise ValueError('Collision is missing its geometry element')
    box = geometry.find('box/size')
    if box is not None:
        if box.text is None:
            raise ValueError('Box geometry is missing its size')
        size = tuple(float(value) / 2.0 for value in box.text.split())
        if len(size) != 3:
            raise ValueError('Box size must contain three coordinates')
        return [(x, y, z) for x in (-size[0], size[0])
                for y in (-size[1], size[1]) for z in (-size[2], size[2])]
    cylinder = geometry.find('cylinder')
    if cylinder is not None:
        radius = _required_float(cylinder, 'radius', 'cylinder geometry')
        half_length = (
            _required_float(cylinder, 'length', 'cylinder geometry') / 2.0
        )
        return [(x, y, z) for x in (-radius, radius)
                for y in (-radius, radius)
                for z in (-half_length, half_length)]
    sphere = geometry.find('sphere')
    if sphere is not None:
        radius = _required_float(sphere, 'radius', 'sphere geometry')
        return [(x, y, z) for x in (-radius, radius)
                for y in (-radius, radius) for z in (-radius, radius)]
    mesh = geometry.find('mesh')
    if mesh is not None:
        uri = (mesh.findtext('uri') or '').strip()
        mesh_path = model_dir / uri
        if not mesh_path.is_file():
            raise FileNotFoundError(f'Mesh not found: {mesh_path}')
        scale = tuple(
            float(value)
            for value in (mesh.findtext('scale') or '1 1 1').split()
        )
        if len(scale) != 3:
            raise ValueError('Mesh scale must contain three coordinates')
        return [
            tuple(point[index] * scale[index] for index in range(3))
            for point in _mesh_bounds(mesh_path)
        ]
    raise ValueError('Unsupported collision geometry in SDF')



def _box_triangles(size):
    """Return a closed, triangulated box centered at the local origin."""
    hx, hy, hz = (float(value) / 2.0 for value in size)
    vertices = [(x, y, z) for x in (-hx, hx) for y in (-hy, hy) for z in (-hz, hz)]
    faces = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1),
             (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    return [(vertices[a], vertices[b], vertices[c])
            for face in faces
            for a, b, c in ((face[0], face[1], face[2]),
                            (face[0], face[2], face[3]))]


def _cylinder_triangles(radius, length, segments=48):
    """Closed triangulated cylinder; SDF cylinders have their axis on local Z."""
    radius, half = float(radius), float(length) / 2.0
    if radius <= 0.0 or half <= 0.0:
        raise ValueError('Cylinder radius and length must be positive')
    bottom, top = [], []
    for index in range(segments):
        angle = 2.0 * math.pi * index / segments
        x, y = radius * math.cos(angle), radius * math.sin(angle)
        bottom.append((x, y, -half))
        top.append((x, y, half))
    center_bottom, center_top = (0.0, 0.0, -half), (0.0, 0.0, half)
    triangles = []
    for index in range(segments):
        nxt = (index + 1) % segments
        triangles.extend(((bottom[index], bottom[nxt], top[nxt]),
                          (bottom[index], top[nxt], top[index]),
                          (center_bottom, bottom[nxt], bottom[index]),
                          (center_top, top[index], top[nxt])))
    return triangles


def _sphere_triangles(radius, rings=16, segments=32):
    """Closed UV sphere mesh used to preserve its circular XY footprint."""
    radius = float(radius)
    if radius <= 0.0:
        raise ValueError('Sphere radius must be positive')
    vertices = []
    for ring in range(rings + 1):
        latitude = -math.pi / 2.0 + math.pi * ring / rings
        z, radial = radius * math.sin(latitude), radius * math.cos(latitude)
        vertices.append([(radial * math.cos(2.0 * math.pi * i / segments),
                          radial * math.sin(2.0 * math.pi * i / segments), z)
                         for i in range(segments)])
    triangles = []
    bottom_pole, top_pole = (0.0, 0.0, -radius), (0.0, 0.0, radius)
    for index in range(segments):
        nxt = (index + 1) % segments
        triangles.append((bottom_pole, vertices[1][nxt], vertices[1][index]))
        triangles.append((top_pole, vertices[-2][index], vertices[-2][nxt]))
    for ring in range(1, rings - 1):
        for index in range(segments):
            nxt = (index + 1) % segments
            a, b = vertices[ring][index], vertices[ring][nxt]
            c, d = vertices[ring + 1][nxt], vertices[ring + 1][index]
            triangles.extend(((a, b, c), (a, c, d)))
    return triangles


def _geometry_triangles(geometry, model_dir):
    """Return local-coordinate triangles for every supported SDF collision."""
    if geometry is None:
        raise ValueError('Collision is missing its geometry element')
    box = geometry.find('box')
    if box is not None:
        text = box.findtext('size')
        if not text:
            raise ValueError('Box geometry is missing its size')
        size = tuple(float(value) for value in text.split())
        if len(size) != 3:
            raise ValueError('Box size must contain three coordinates')
        return _box_triangles(size)
    cylinder = geometry.find('cylinder')
    if cylinder is not None:
        return _cylinder_triangles(_required_float(cylinder, 'radius', 'cylinder geometry'),
                                   _required_float(cylinder, 'length', 'cylinder geometry'))
    sphere = geometry.find('sphere')
    if sphere is not None:
        return _sphere_triangles(_required_float(sphere, 'radius', 'sphere geometry'))
    mesh = geometry.find('mesh')
    if mesh is not None:
        uri = (mesh.findtext('uri') or '').strip()
        mesh_path = model_dir / uri
        if not mesh_path.is_file():
            raise FileNotFoundError(f'Mesh not found: {mesh_path}')
        scale = tuple(float(value) for value in (mesh.findtext('scale') or '1 1 1').split())
        if len(scale) != 3:
            raise ValueError(f'Mesh scale must contain three coordinates: {mesh_path}')
        return [tuple(tuple(vertex[axis] * scale[axis] for axis in range(3))
                      for vertex in triangle) for triangle in _mesh_triangles(mesh_path)]
    raise ValueError('Unsupported collision geometry in SDF')


def _triangle_vertices(triangles):
    return [point for triangle in triangles for point in triangle]


def load_world_collisions(world_path, models_dir):
    world_path = Path(world_path)
    models_dir = Path(models_dir)
    root = ET.parse(world_path).getroot()
    world = root.find('world')
    if world is None:
        raise ValueError(f'No world element found in SDF: {world_path}')
    obstacles = []
    for include in world.findall('include'):
        uri = (include.findtext('uri') or '').strip()
        if not uri.startswith('model://uav_sim/models/'):
            continue
        model_dir = models_dir / uri.rsplit('/', 1)[-1]
        model_root = ET.parse(model_dir / 'model.sdf').getroot()
        model = model_root.find('model')
        if model is None:
            raise ValueError(
                f'No model element found in SDF: {model_dir / "model.sdf"}'
            )
        include_pose = _values(include.find('pose'))
        model_pose = _compose(include_pose, _values(model.find('pose')))
        for link in model.findall('link'):
            link_pose = _compose(model_pose, _values(link.find('pose')))
            for collision in link.findall('collision'):
                collision_pose = _compose(
                    link_pose, _values(collision.find('pose'))
                )
                geometry = collision.find('geometry')
                local_triangles = _geometry_triangles(geometry, model_dir)
                world_triangles = [
                    tuple(_transform(vertex, collision_pose) for vertex in triangle)
                    for triangle in local_triangles
                ]
                world_points = _triangle_vertices(world_triangles)
                item = {
                    'name': (include.findtext('name') or model.get('name') or model_dir.name),
                    'bounds': [min(point[axis] for point in world_points) for axis in range(3)]
                              + [max(point[axis] for point in world_points) for axis in range(3)],
                    'triangles': world_triangles,
                }
                obstacles.append(item)
    return obstacles