#!/usr/bin/env python3
"""Host-side URDF/COLLADA visuals to Geometry TriangleMesh assets (requires NumPy).

No ROS subscriber, robot kinematics or renderer. Fixed visual offsets are baked
into observed-link coordinates; moving link poses remain canonical IR observations.
Only static Z_UP COLLADA triangles with normals are admitted. Material diffuse colors become separate mesh assets with shared link poses.
Texture-bearing COLLADA remains unsupported. IDs hash the final immutable bytes.
"""
import argparse
import hashlib
import json
import math
import warnings
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np


class RobotMesh:
    MAX_BYTES = 16 * 1024 * 1024

    @staticmethod
    def xml(path):
        with Path(path).open('rb') as source:
            data = source.read(RobotMesh.MAX_BYTES + 1)
        if len(data) > RobotMesh.MAX_BYTES or b'<!DOCTYPE' in data or b'<!ENTITY' in data:
            raise ValueError('XML exceeds limit or contains DTD/entities')
        root = ET.fromstring(data)
        for element in root.iter():
            element.tag = element.tag.rsplit('}', 1)[-1]
        return root

    @staticmethod
    def numbers(text, count):
        values = np.array([float(x) for x in text.split()])
        if len(values) != count or not np.isfinite(values).all():
            raise ValueError('Invalid numeric vector')
        return values

    @staticmethod
    def rotation(axis, angle):
        axis = np.asarray(axis, dtype=float)
        length = np.linalg.norm(axis)
        if not np.isfinite(length) or length < 1e-12:
            raise ValueError('Invalid rotation axis')
        x, y, z = axis / length
        cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
        result = np.eye(4)
        result[:3, :3] = np.eye(3) + math.sin(angle) * cross + (1 - math.cos(angle)) * cross @ cross
        return result

    @classmethod
    def origin(cls, element):
        if element is None:
            return np.eye(4)
        angles = cls.numbers(element.get('rpy', '0 0 0'), 3)
        result = np.eye(4)
        for axis, angle in zip(np.eye(3), angles):
            result = cls.rotation(axis, angle) @ result
        result[:3, 3] = cls.numbers(element.get('xyz', '0 0 0'), 3)
        return result

    @classmethod
    def node_transform(cls, node):
        result = np.eye(4)
        for child in node:
            transform = np.eye(4)
            if child.tag == 'matrix':
                # COLLADA matrices operate on columns but XML writes rows in order.
                transform = cls.numbers(child.text, 16).reshape(4, 4)
            elif child.tag == 'translate':
                transform[:3, 3] = cls.numbers(child.text, 3)
            elif child.tag == 'scale':
                transform[:3, :3] = np.diag(cls.numbers(child.text, 3))
            elif child.tag == 'rotate':
                value = cls.numbers(child.text, 4)
                transform = cls.rotation(value[:3], math.radians(value[3]))
            elif child.tag in ('lookat', 'skew', 'instance_controller', 'instance_node'):
                raise ValueError('Unsupported COLLADA transform/instance')
            else:
                continue
            result = result @ transform
        if not np.allclose(result[3], [0, 0, 0, 1]) or abs(np.linalg.det(result[:3, :3])) < 1e-12:
            raise ValueError('Non-affine or singular visual transform')
        return result

    @classmethod
    def collada(cls, path, placed):
        root = cls.xml(path)
        if root.findtext('asset/up_axis') != 'Z_UP' or root.find('library_animations') is not None:
            raise ValueError('Only static Z_UP COLLADA is supported')
        unit = root.find('asset/unit')
        meter = float(unit.get('meter', '1')) if unit is not None else 1.
        if not math.isfinite(meter) or meter <= 0:
            raise ValueError('Invalid COLLADA units')
        unit_transform = np.diag([meter, meter, meter, 1.])
        geometries = {g.get('id'): g.find('mesh') for g in root.findall('library_geometries/geometry')}
        scene_id = root.find('scene/instance_visual_scene').get('url').removeprefix('#')
        scene = next(s for s in root.findall('library_visual_scenes/visual_scene') if s.get('id') == scene_id)
        effects = {e.get('id'): e for e in root.findall('library_effects/effect')}
        materials = {m.get('id'): m.find('instance_effect').get('url').removeprefix('#') for m in root.findall('library_materials/material')}
        groups = {}
        total_vertices = total_indices = 0
        stack = [(node, placed @ unit_transform, 0) for node in scene.findall('node')]
        visited = 0
        while stack:
            node, parent, depth = stack.pop()
            visited += 1
            if visited > 4096 or depth > 64:
                raise ValueError('COLLADA node budget exceeded')
            world = parent @ cls.node_transform(node)
            stack.extend((child, world, depth + 1) for child in node.findall('node'))
            for instance in node.findall('instance_geometry'):
                mesh = geometries[instance.get('url').removeprefix('#')]
                if any(child.tag in ('polygons', 'trifans', 'tristrips') for child in mesh):
                    raise ValueError('Only explicit triangle primitives are supported')
                if any(child.tag in ('lines', 'linestrips') for child in mesh):
                    warnings.warn(f'{path.name}: line primitives omitted from surface mesh', stacklevel=2)
                sources = {}
                for source in mesh.findall('source'):
                    accessor = source.find('technique_common/accessor')
                    array = next(a for a in source.findall('float_array') if '#' + a.get('id') == accessor.get('source'))
                    values = cls.numbers(array.text, int(array.get('count')))
                    count, stride, offset = int(accessor.get('count')), int(accessor.get('stride', '1')), int(accessor.get('offset', '0'))
                    if stride < 3 or count > 1_000_000 or offset < 0 or offset + count * stride > len(values):
                        raise ValueError('Invalid COLLADA accessor')
                    sources[source.get('id')] = values[offset:offset + count * stride].reshape(count, stride)[:, :3]
                vertices = {v.get('id'): v.find("input[@semantic='POSITION']").get('source').removeprefix('#') for v in mesh.findall('vertices')}
                normal_matrix = np.linalg.inv(world[:3, :3]).T
                bindings = {m.get('symbol'): m.get('target').removeprefix('#') for m in instance.findall('bind_material/technique_common/instance_material')}
                for triangles in (child for child in mesh if child.tag in ('triangles', 'polylist')):
                    if triangles.tag == 'polylist':
                        counts = [int(v) for v in triangles.findtext('vcount', '').split()]
                        if len(counts) != int(triangles.get('count')) or any(v != 3 for v in counts):
                            raise ValueError('Only triangular polylist faces are supported')
                    color = (0.72, 0.72, 0.72, 1.)
                    symbol = triangles.get('material')
                    if symbol:
                        effect = effects[materials[bindings[symbol]]]
                        diffuse = effect.find('.//diffuse')
                        if diffuse is not None and diffuse.find('texture') is not None:
                            raise ValueError('Textured COLLADA requires the texture import adapter')
                        rgba = diffuse.find('color') if diffuse is not None else None
                        if rgba is not None:
                            color = tuple(cls.numbers(rgba.text, 4))
                            if any(v < 0 or v > 1 for v in color):
                                raise ValueError('Material color outside [0,1]')
                    if color not in groups and len(groups) >= 32:
                        raise ValueError('Material part budget exceeded')
                    positions, normals, indices = groups.setdefault(color, ([], [], []))
                    inputs = {i.get('semantic'): i for i in triangles.findall('input')}
                    stride = max(int(i.get('offset', '0')) for i in inputs.values()) + 1
                    count = int(triangles.get('count')) * 3
                    if count > 6_000_000 - total_indices:
                        raise ValueError('Triangle budget exceeded')
                    raw = np.array([int(x) for x in triangles.findtext('p').split()], dtype=np.int64).reshape(count, stride)
                    chunks = []
                    for semantic in ['VERTEX', 'NORMAL']:
                        item = inputs[semantic]
                        source = item.get('source').removeprefix('#')
                        if semantic == 'VERTEX':
                            source = vertices[source]
                        data = sources[source]
                        slots = raw[:, int(item.get('offset', '0'))]
                        if slots.min() < 0 or slots.max() >= len(data):
                            raise ValueError('COLLADA index out of bounds')
                        chunks.append(data[slots])
                    pos = chunks[0] @ world[:3, :3].T + world[:3, 3]
                    normal = chunks[1] @ normal_matrix.T
                    lengths = np.linalg.norm(normal, axis=1)
                    if np.any(lengths < 1e-12):
                        raise ValueError('Zero vertex normal')
                    packed, remap = np.unique(np.round(np.concatenate([pos, normal / lengths[:, None]], axis=1), 7), axis=0, return_inverse=True)
                    if total_vertices + len(packed) > 1_000_000:
                        raise ValueError('Vertex budget exceeded')
                    local = remap.reshape(-1, 3)
                    if np.linalg.det(world[:3, :3]) < 0:
                        local = local[:, [0, 2, 1]]
                    total_vertices += len(packed)
                    total_indices += count
                    indices.extend((local.reshape(-1) + len(positions)).tolist())
                    positions.extend(packed[:, :3].tolist())
                    normals.extend(packed[:, 3:].tolist())
        if not total_vertices:
            raise ValueError('No supported mesh geometry')
        return [dict(rgba=list(color), mesh=dict(positions=p, normals=n, lods=[dict(indices=i, error=0)], uv=[], tangents=[]))
                for color, (p, n, i) in groups.items()]

    @classmethod
    def export(cls, urdf, frames, output, package_root, visual_links):
        root = cls.xml(urdf)
        parents = {j.find('child').get('link'): j for j in root.findall('joint')}
        output.mkdir(parents=True, exist_ok=True)
        manifest = []
        links = {link.get('name'): link for link in root.findall('link')}
        if not visual_links.issubset(links):
            raise ValueError('Requested visual link absent from robot_description')
        for name in sorted(visual_links):
            link = links[name]
            name, frame, fixed, seen = link.get('name'), link.get('name'), np.eye(4), set()
            if not link.findall('visual'):
                continue
            while frame not in frames:
                joint = parents.get(frame)
                if frame in seen or joint is None or joint.get('type') != 'fixed':
                    raise ValueError(f'Visual {name} has no fixed path to an observed frame')
                seen.add(frame)
                fixed = cls.origin(joint.find('origin')) @ fixed
                frame = joint.find('parent').get('link')
            visuals = link.findall('visual')
            if len(visuals) != 1:
                raise ValueError('This adapter requires one mesh visual per link')
            visual = visuals[0]
            mesh = visual.find('geometry/mesh')
            if mesh is None:
                raise ValueError('URDF primitive visuals require the primitive adapter')
            uri = mesh.get('filename')
            if uri.startswith('file://'):
                path = Path(uri[7:])
            elif uri.startswith('package://'):
                path = package_root / uri[10:]
            else:
                path = Path(urdf).parent / uri
            path = path.resolve(strict=True)
            if not path.is_relative_to(package_root.resolve()) or path.suffix.lower() != '.dae':
                raise ValueError('Visual file outside explicit package root or not COLLADA')
            scale = np.diag([*cls.numbers(mesh.get('scale', '1 1 1'), 3), 1.])
            parts = cls.collada(path, fixed @ cls.origin(visual.find('origin')) @ scale)
            material = visual.find('material')
            if material is not None and material.find('color') is None:
                material = next((m for m in root.findall('material') if m.get('name') == material.get('name')), material)
            override = material.find('color') if material is not None else None
            for part_index, part in enumerate(parts):
                data = part['mesh']
                rgba = cls.numbers(override.get('rgba'), 4).tolist() if override is not None else part['rgba']
                if any(v < 0 or v > 1 for v in rgba):
                    raise ValueError('URDF material color outside [0,1]')
                encoded = json.dumps(data, separators=(',', ':'), allow_nan=False).encode()
                if len(encoded) > 8 * 1024 * 1024:
                    raise ValueError('Export exceeds HIVE asset byte limit')
                asset = hashlib.sha256(encoded).hexdigest() + '.json'
                (output / asset).write_bytes(encoded)
                manifest.append(dict(frame=frame, visual_link=name, part=part_index, rgba=rgba, mesh_asset=asset, source=str(path),
                                     triangles=len(data['lods'][0]['indices']) // 3, bytes=len(encoded),
                                     bounds=[np.min(data['positions'], axis=0).tolist(), np.max(data['positions'], axis=0).tolist()]))
        return manifest


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf', type=Path, required=True)
    parser.add_argument('--frames', nargs='+', required=True)
    parser.add_argument('--visual-links', nargs='+', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--package-root', type=Path, required=True)
    args = parser.parse_args()
    args.manifest.write_text(json.dumps(RobotMesh.export(args.urdf, set(args.frames), args.output, args.package_root, set(args.visual_links)), indent=2))
