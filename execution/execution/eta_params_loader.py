"""
ETA parameter loader for launch wiring.

Source of truth: execution/config/eta_robot_params.yaml
Physical profile (footprint/vicinity): execution/config/config.yaml

Attach from launch with:
  from execution.eta_params_loader import load_eta_launch_config
"""

from __future__ import annotations

import os
from typing import Any, Dict, Tuple

try:
    import yaml
except ImportError:
    yaml = None


def load_eta_motion_traits(
    eta_params_path: str,
    config_yaml_path: str,
) -> Dict[str, Any]:
    """Load fleet-adapter vehicle traits from ETA params (+ footprint from config.yaml)."""
    traits: Dict[str, Any] = {
        'linear_velocity': 0.5,
        'linear_acceleration': 0.5,
        'angular_velocity': 0.85,
        'angular_acceleration': 1.0,
        'linear_deceleration': 0.6,
        'footprint_radius': 0.3,
        'vicinity_radius': 0.5,
        'reversible': True,
    }

    if yaml is not None and os.path.isfile(config_yaml_path):
        try:
            with open(config_yaml_path, 'r', encoding='utf-8') as f:
                cfg = yaml.safe_load(f) or {}
            fleet = (cfg.get('rmf_fleet') or {})
            profile = fleet.get('profile') or {}
            if 'footprint' in profile:
                traits['footprint_radius'] = float(profile['footprint'])
            if 'vicinity' in profile:
                traits['vicinity_radius'] = float(profile['vicinity'])
            if 'reversible' in fleet:
                traits['reversible'] = bool(fleet['reversible'])
        except Exception:
            pass

    if yaml is not None and os.path.isfile(eta_params_path):
        try:
            with open(eta_params_path, 'r', encoding='utf-8') as f:
                eta_root = yaml.safe_load(f) or {}
            eta = eta_root.get('eta_robot') or eta_root
            traits['linear_velocity'] = float(
                eta.get('operating_linear_speed_mps', traits['linear_velocity']))
            traits['linear_acceleration'] = float(
                eta.get('linear_accel_mps2', traits['linear_acceleration']))
            traits['angular_velocity'] = float(
                eta.get('angular_speed_max_rps', traits['angular_velocity']))
            traits['angular_acceleration'] = float(
                eta.get('angular_accel_rps2', traits['angular_acceleration']))
            traits['linear_deceleration'] = float(
                eta.get('linear_decel_mps2', 0.6))
        except Exception:
            pass

    return traits


def find_eta_params_file(execution_pkg_share: str = '') -> str:
    """Prefer the source-tree YAML so an edit applies on the next launch."""
    starts = [os.getcwd(), os.path.dirname(os.path.abspath(__file__))]
    prefix = os.environ.get('COLCON_PREFIX_PATH', '')
    starts.extend(part for part in prefix.split(os.pathsep) if part)

    seen = set()
    for start in starts:
        directory = os.path.abspath(start)
        for _ in range(8):
            if directory in seen:
                break
            seen.add(directory)
            candidate = os.path.join(
                directory, 'src', 'execution', 'config', 'eta_robot_params.yaml')
            if os.path.isfile(candidate):
                return candidate
            parent = os.path.dirname(directory)
            if parent == directory:
                break
            directory = parent

    if execution_pkg_share:
        installed = os.path.join(
            execution_pkg_share, 'config', 'eta_robot_params.yaml')
        if os.path.isfile(installed):
            return installed
    return ''


def resolve_eta_paths(execution_pkg_share: str) -> Tuple[str, str]:
    """Return (eta_params.yaml path, config.yaml path)."""
    eta_params_file = find_eta_params_file(execution_pkg_share)
    if not eta_params_file:
        eta_params_file = os.path.join(
            execution_pkg_share, 'config', 'eta_robot_params.yaml')
    config_file = os.path.join(execution_pkg_share, 'config', 'config.yaml')
    return eta_params_file, config_file


def load_eta_launch_config(execution_pkg_share: str) -> Dict[str, Any]:
    """
    One-call helper for launch files.

    Returns dict with:
      eta_params_file, config_file, traits,
      fleet_adapter_vehicle_params, zone_monitor_launch_arguments
    """
    eta_params_file, config_file = resolve_eta_paths(execution_pkg_share)
    traits = load_eta_motion_traits(eta_params_file, config_file)

    # Core RMF curve timing params (DifferentialDrivePlanner).
    curve_params: Dict[str, Any] = {
        'curve_heading_change_threshold_deg': 20.0,
        'curve_heading_min_deg': 3.0,
        'curve_heading_max_deg': 25.0,
        'curve_linear_speed_mps': 0.45,
    }
    if yaml is not None and os.path.isfile(eta_params_file):
        try:
            with open(eta_params_file, 'r', encoding='utf-8') as f:
                eta_root = yaml.safe_load(f) or {}
            eta = eta_root.get('eta_robot') or eta_root
            cm = eta.get('curve_motion') or {}
            ap = eta.get('curve_approach') or {}
            if 'heading_change_threshold_deg' in cm:
                curve_params['curve_heading_change_threshold_deg'] = float(
                    cm['heading_change_threshold_deg'])
            if 'heading_min_deg' in cm:
                curve_params['curve_heading_min_deg'] = float(
                    cm['heading_min_deg'])
            if 'heading_max_deg' in cm:
                curve_params['curve_heading_max_deg'] = float(
                    cm['heading_max_deg'])
            if 'curve_linear_speed_mps' in cm:
                curve_params['curve_linear_speed_mps'] = float(
                    cm['curve_linear_speed_mps'])
            elif 'required_curve_speed_mps' in ap:
                curve_params['curve_linear_speed_mps'] = float(
                    ap['required_curve_speed_mps'])
            if 'approach_start_distance_m' in ap:
                curve_params['approach_start_distance_m'] = float(
                    ap['approach_start_distance_m'])
            stop_speeds = []
            stop_distances = []
            for row in eta.get('stop_decel_table') or []:
                if not isinstance(row, dict):
                    continue
                if 'speed_mps' in row and 'distance_m' in row:
                    stop_speeds.append(float(row['speed_mps']))
                    stop_distances.append(float(row['distance_m']))
            if stop_speeds and stop_distances:
                curve_params['stop_speed_mps'] = stop_speeds
                curve_params['stop_distance_m'] = stop_distances
        except Exception:
            pass

    curve_params.setdefault('approach_start_distance_m', 5.0)
    curve_params.setdefault('stop_speed_mps', [0.5, 1.0, 1.5, 2.0])
    curve_params.setdefault('stop_distance_m', [3.0, 4.0, 5.0, 7.0])

    return {
        'eta_params_file': eta_params_file,
        'config_file': config_file,
        'traits': traits,
        'curve_params': curve_params,
        'fleet_adapter_vehicle_params': {
            'linear_velocity': traits['linear_velocity'],
            'linear_acceleration': traits['linear_acceleration'],
            'angular_velocity': traits['angular_velocity'],
            'angular_acceleration': traits['angular_acceleration'],
            'footprint_radius': traits['footprint_radius'],
            'vicinity_radius': traits['vicinity_radius'],
            'reversible': traits['reversible'],
            'curve_heading_change_threshold_deg':
                curve_params['curve_heading_change_threshold_deg'],
            'curve_heading_min_deg':
                curve_params['curve_heading_min_deg'],
            'curve_heading_max_deg':
                curve_params['curve_heading_max_deg'],
            'curve_linear_speed_mps': curve_params['curve_linear_speed_mps'],
            'linear_deceleration': traits['linear_deceleration'],
            'approach_start_distance_m':
                curve_params['approach_start_distance_m'],
            'stop_speed_mps': curve_params['stop_speed_mps'],
            'stop_distance_m': curve_params['stop_distance_m'],
        },
        'zone_monitor_launch_arguments': {},
    }


def _stop_distance_at_speed(speed: float, speeds, distances) -> float:
    if not speeds or not distances:
        return 0.0
    pairs = sorted(zip(speeds, distances), key=lambda row: row[0])
    if speed <= pairs[0][0]:
        base = max(pairs[0][0], 1e-6)
        return pairs[0][1] * (speed / base)
    for index in range(len(pairs) - 1):
        speed_lo, dist_lo = pairs[index]
        speed_hi, dist_hi = pairs[index + 1]
        if speed <= speed_hi and speed_hi > speed_lo:
            span = (speed - speed_lo) / (speed_hi - speed_lo)
            return dist_lo + span * (dist_hi - dist_lo)
    return pairs[-1][1]


def load_eta_timing_values(execution_pkg_share: str = '') -> Dict[str, Any]:
    """Motion numbers from eta_robot_params.yaml for FMS and Flatland."""
    if not execution_pkg_share:
        try:
            from ament_index_python.packages import get_package_share_directory
            execution_pkg_share = get_package_share_directory('execution')
        except Exception:
            execution_pkg_share = ''

    cfg = load_eta_launch_config(execution_pkg_share)
    traits = cfg['traits']
    curve = cfg['curve_params']
    return {
        'linear_speed_mps': float(traits['linear_velocity']),
        'curve_linear_speed_mps': float(curve['curve_linear_speed_mps']),
        'linear_accel_mps2': float(traits['linear_acceleration']),
        'linear_decel_mps2': float(traits['linear_deceleration']),
        'angular_speed_rps': float(traits['angular_velocity']),
        'angular_accel_rps2': float(traits['angular_acceleration']),
        'approach_start_distance_m': float(curve['approach_start_distance_m']),
        'heading_min_deg': float(curve['curve_heading_min_deg']),
        'heading_max_deg': float(curve['curve_heading_max_deg']),
        'heading_change_threshold_deg': float(
            curve['curve_heading_change_threshold_deg']),
        'stop_speed_mps': list(curve['stop_speed_mps']),
        'stop_distance_m': list(curve['stop_distance_m']),
        'stop_distance_at_speed_m': _stop_distance_at_speed(
            float(traits['linear_velocity']),
            curve['stop_speed_mps'],
            curve['stop_distance_m'],
        ),
        'eta_params_file': cfg['eta_params_file'],
    }
