"""Shared utilities for scene setup: model discovery, random placement, and Gazebo spawning."""
import glob
import math
import os
import random
import re
import subprocess

import yaml


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
    except Exception:
        pass
    return 0.05


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
