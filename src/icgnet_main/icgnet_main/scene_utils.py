"""Shared utilities for scene setup: model discovery, random placement, and Gazebo spawning."""
import glob
import math
import os
import random
import re
import subprocess

import yaml

from icgnet_msgs.msg import SceneObject


def half_height_from_sdf(sdf_path: str) -> float:
    """Return half the vertical extent of the first collision geometry in the SDF, or 0.05."""
    try:
        with open(sdf_path) as f:
            content = f.read()
        m = re.search(r'<length>([\d.eE+-]+)</length>', content)
        if m:
            return float(m.group(1)) / 2.0
        m = re.search(r'<size>([\d.eE+-]+)\s+([\d.eE+-]+)\s+([\d.eE+-]+)</size>', content)
        if m:
            return float(m.group(3)) / 2.0
        m = re.search(r'<radius>([\d.eE+-]+)</radius>', content)
        if m:
            return float(m.group(1))
    except (OSError, ValueError):
        pass
    return 0.05


def visual_geometry_from_sdf(sdf_path: str) -> dict | None:
    """Parse the first <visual> geometry block from an SDF model file.

    Returns a spec dict with kind-specific keys plus shared keys:
      offset_xyz, offset_rpy  — visual-local pose offset (default zeros)
      color_rgba              — (r, g, b, a) from <diffuse>, else neutral gray
    Kind-specific keys:
      'mesh':     uri (package://icgnet_main/models/...), scale (sx, sy, sz)
      'cylinder': radius, length
      'box':      dims (x, y, z)
      'sphere':   radius
    Returns None on parse failure or unknown geometry.
    """
    try:
        with open(sdf_path) as f:
            content = f.read()
    except OSError:
        return None

    vm = re.search(r'<visual[^>]*>(.*?)</visual>', content, re.DOTALL)
    if not vm:
        return None
    block = vm.group(1)

    # Optional visual-local pose: x y z roll pitch yaw.
    offset_xyz = (0.0, 0.0, 0.0)
    offset_rpy = (0.0, 0.0, 0.0)
    pm = re.search(r'<pose>(.*?)</pose>', block, re.DOTALL)
    if pm:
        vals = [float(v) for v in pm.group(1).split()]
        if len(vals) >= 6:
            offset_xyz = (vals[0], vals[1], vals[2])
            offset_rpy = (vals[3], vals[4], vals[5])

    # Optional diffuse color: r g b a.
    color_rgba = (0.7, 0.7, 0.7, 1.0)
    dm = re.search(r'<diffuse>([\d.\s]+)</diffuse>', block)
    if dm:
        vals = [float(v) for v in dm.group(1).split()]
        if len(vals) >= 4:
            color_rgba = (vals[0], vals[1], vals[2], vals[3])
        elif len(vals) == 3:
            color_rgba = (vals[0], vals[1], vals[2], 1.0)

    base = {'offset_xyz': offset_xyz, 'offset_rpy': offset_rpy, 'color_rgba': color_rgba}

    # Mesh: build a package:// URI RViz can open.
    #  - model://<head>/...  (flat models, e.g. coke_can) → package://icgnet_main/models/<head>/...
    #  - relative URI (GSO models, e.g. 'meshes/model.obj') → resolve against the SDF location
    #    and rebuild from the 'models/' segment, so any class-subdir layout works too.
    mm = re.search(r'<mesh>.*?<uri>(.*?)</uri>.*?</mesh>', block, re.DOTALL)
    if mm:
        raw_uri = mm.group(1).strip()
        if raw_uri.startswith('model://'):
            uri = re.sub(r'^model://([^/]+)/(.*)', r'package://icgnet_main/models/\1/\2', raw_uri)
        else:
            abs_mesh = os.path.normpath(os.path.join(os.path.dirname(sdf_path), raw_uri))
            idx = abs_mesh.find(os.sep + 'models' + os.sep)
            uri = ('package://icgnet_main' + abs_mesh[idx:].replace(os.sep, '/')
                   if idx != -1 else raw_uri)
        scale = (1.0, 1.0, 1.0)
        sm = re.search(r'<scale>([\d.\s]+)</scale>', block)
        if sm:
            vals = [float(v) for v in sm.group(1).split()]
            if len(vals) >= 3:
                scale = (vals[0], vals[1], vals[2])
        return {**base, 'kind': 'mesh', 'uri': uri, 'scale': scale}

    # Cylinder.
    cym = re.search(
        r'<cylinder>.*?<radius>([\d.eE+-]+)</radius>.*?<length>([\d.eE+-]+)</length>.*?</cylinder>',
        block, re.DOTALL,
    )
    if cym:
        return {**base, 'kind': 'cylinder',
                'radius': float(cym.group(1)), 'length': float(cym.group(2))}

    # Box.
    bm = re.search(
        r'<box>.*?<size>([\d.eE+-]+)\s+([\d.eE+-]+)\s+([\d.eE+-]+)</size>.*?</box>',
        block, re.DOTALL,
    )
    if bm:
        return {**base, 'kind': 'box',
                'dims': (float(bm.group(1)), float(bm.group(2)), float(bm.group(3)))}

    # Sphere.
    spm = re.search(r'<sphere>.*?<radius>([\d.eE+-]+)</radius>.*?</sphere>', block, re.DOTALL)
    if spm:
        return {**base, 'kind': 'sphere', 'radius': float(spm.group(1))}

    return None


def load_catalog(models_dir: str) -> dict:
    """Load and return models/catalog.yaml as a dict."""
    catalog_path = os.path.join(models_dir, 'catalog.yaml')
    with open(catalog_path) as f:
        return yaml.safe_load(f)


def find_model_sdf(models_dir: str, model_name: str) -> str | None:
    """Return the absolute path to model.sdf for a named model, or None if not found.

    Searches class-subdir layout (models/<class>/<name>/model.sdf) first,
    then flat layout (models/<name>/model.sdf).
    """
    matches = glob.glob(os.path.join(models_dir, '*', model_name, 'model.sdf'))
    if matches:
        return matches[0]
    direct = os.path.join(models_dir, model_name, 'model.sdf')
    if os.path.isfile(direct):
        return direct
    return None


def get_random_pose(
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    reach_min: float,
    reach_max: float,
    min_dist: float,
    existing_poses: list,
) -> tuple[float | None, float | None]:
    """Return a random (x, y) satisfying reach and min-distance constraints, or (None, None)."""
    for _ in range(500):
        x = random.uniform(x_min, x_max)
        y = random.uniform(y_min, y_max)
        reach = math.sqrt(x ** 2 + y ** 2)
        if reach > reach_max or reach < reach_min:
            continue
        if all(math.sqrt((x - ex) ** 2 + (y - ey) ** 2) >= min_dist for ex, ey in existing_poses):
            return x, y
    return None, None


def spawn_gz_entity(
    entity_name: str,
    sdf_path: str,
    x: float,
    y: float,
    z: float,
    yaw: float,
    world: str = 'icgnet_world',
    logger=None,
) -> bool:
    """Spawn a model in Gazebo via ros_gz_sim create. Returns True on success."""
    cmd = [
        'ros2', 'run', 'ros_gz_sim', 'create',
        '-world', world,
        '-name', entity_name,
        '-x', f'{x:.3f}',
        '-y', f'{y:.3f}',
        '-z', f'{z:.4f}',
        '-Y', f'{yaw:.3f}',
        '-file', sdf_path,
    ]
    try:
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        for line in proc.stdout:
            if logger:
                logger.info(f'[{entity_name}] {line.rstrip()}')
        proc.wait()
        return proc.returncode == 0
    except Exception as e:
        if logger:
            logger.error(f'[{entity_name}] Spawn exception: {e}')
        return False


def yaw_to_quat_zw(yaw: float) -> tuple[float, float]:
    """Return the (z, w) quaternion components for a rotation of `yaw` about +Z."""
    half = yaw / 2.0
    return math.sin(half), math.cos(half)


def scene_object_from_entry(entry: dict) -> SceneObject:
    """Build a SceneObject from a spawn-registry dict.

    Expected keys: entity_name, model_name, semantic_class, x, y, z, yaw.
    """
    obj = SceneObject()
    obj.entity_name = entry['entity_name']
    obj.model_name = entry['model_name']
    obj.semantic_class = entry['semantic_class']
    obj.pose.position.x = float(entry['x'])
    obj.pose.position.y = float(entry['y'])
    obj.pose.position.z = float(entry['z'])
    obj.pose.orientation.z, obj.pose.orientation.w = yaw_to_quat_zw(float(entry['yaw']))
    return obj
