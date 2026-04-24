"""
Offline magnetometer calibration script.

Reads a ROS2 bag file, fits hard-iron + soft-iron ellipsoid calibration,
and prints the values ready to paste into mag_heading_params.yaml.

Usage (only --bag is required):
    ros2 run mag_heading calibrate_mag --bag /path/to/rosbag2_dir

The topic is read from mag_heading_params.yaml automatically.
Override with --topic if needed.

Requires: rosbag2_py, numpy
"""

import argparse
import math
import os
import sys

import numpy as np
import yaml

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import MagneticField
    from ament_index_python.packages import get_package_share_directory
except ImportError as e:
    print(f'ERROR: Missing ROS2 dependency: {e}')
    print('Run this script inside the ROS2 devcontainer environment.')
    sys.exit(1)


def load_topic_from_config(config_path: str) -> str:
    with open(config_path, 'r') as f:
        cfg = yaml.safe_load(f)
    return cfg['mag_heading_node']['ros__parameters']['mag_topic']


def read_bag(bag_path: str, topic: str) -> np.ndarray:
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr',
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    if topic not in type_map:
        print(f'ERROR: Topic "{topic}" not in bag. Available topics:')
        for t in topic_types:
            print(f'  {t.name}')
        sys.exit(1)

    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))

    readings = []
    while reader.has_next():
        _, data, _ = reader.read_next()
        msg = deserialize_message(data, MagneticField)
        readings.append([msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z])

    if not readings:
        print(f'ERROR: No messages found on topic "{topic}".')
        sys.exit(1)

    arr = np.array(readings, dtype=float)
    print(f'Read {len(arr)} samples from "{topic}".')
    return arr


def fit_ellipsoid(data: np.ndarray):
    """
    Algebraic least-squares ellipsoid fit.
    Solves: Ax² + By² + Cz² + 2Dxy + 2Exz + 2Fyz + 2Gx + 2Hy + 2Iz = 1

    Returns hard_iron (3,), soft_iron (3,3), radius_sq (float).
    """
    x, y, z = data[:, 0], data[:, 1], data[:, 2]
    D = np.column_stack([
        x**2, y**2, z**2,
        2*x*y, 2*x*z, 2*y*z,
        2*x, 2*y, 2*z,
    ])
    v, _, _, _ = np.linalg.lstsq(D, np.ones(len(data)), rcond=None)

    A, B, C, Dv, E, F, G, H, I = v
    M = np.array([[A, Dv, E], [Dv, B, F], [E, F, C]])
    b = np.array([G, H, I])

    hard_iron = -np.linalg.inv(M) @ b
    inner = hard_iron @ M @ hard_iron - 1.0
    radius_sq = max(-inner, 1e-30)

    W = M / radius_sq
    eigvals, eigvecs = np.linalg.eigh(W)
    eigvals = np.maximum(eigvals, 1e-12)
    soft_iron = eigvecs @ np.diag(np.sqrt(eigvals)) @ eigvecs.T

    return hard_iron, soft_iron, radius_sq


def main():
    default_config = os.path.join(
        get_package_share_directory('mag_heading'), 'config', 'mag_heading_params.yaml'
    )

    parser = argparse.ArgumentParser(
        description='Fit magnetometer calibration from a ROS2 bag.'
    )
    parser.add_argument('--bag', required=True,
                        help='Path to rosbag2 directory')
    parser.add_argument('--config', default=default_config,
                        help=f'Path to mag_heading_params.yaml (default: package config)')
    parser.add_argument('--topic', default=None,
                        help='Override mag topic (default: read from --config)')
    parser.add_argument('--min-samples', type=int, default=200,
                        help='Warn if fewer samples than this (default: 200)')
    args = parser.parse_args()

    topic = args.topic or load_topic_from_config(args.config)
    print(f'Using topic: {topic}  (config: {args.config})')

    data = read_bag(args.bag, topic)

    if len(data) < args.min_samples:
        print(f'WARNING: Only {len(data)} samples. Rotate through multiple full 360° turns.')

    print('Fitting ellipsoid...')
    hard_iron, soft_iron, radius_sq = fit_ellipsoid(data)

    corrected = (soft_iron @ (data - hard_iron).T).T
    norms = np.linalg.norm(corrected, axis=1)
    mean_n, std_n = float(np.mean(norms)), float(np.std(norms))
    quality = std_n / mean_n if mean_n > 0 else 999

    print(f'\n--- Calibration result ---')
    print(f'Samples : {len(data)}')
    print(f'Radius  : {math.sqrt(radius_sq):.6f} T')
    print(f'Residual: mean={mean_n:.5f}  std={std_n:.5f}  ({quality:.1%})', end='  ')
    if quality < 0.05:
        print('GOOD')
    elif quality < 0.10:
        print('ACCEPTABLE')
    else:
        print('POOR — collect more data with full 360° rotations')

    hi = hard_iron.tolist()
    si = soft_iron.flatten().tolist()

    print(f'\n--- Paste into mag_heading_params.yaml ---')
    print(f'    hard_iron: [{hi[0]:.8f}, {hi[1]:.8f}, {hi[2]:.8f}]')
    si_fmt = ', '.join(f'{v:.8f}' for v in si)
    print(f'    soft_iron: [{si_fmt}]')
    print(f'------------------------------------------')


if __name__ == '__main__':
    main()
