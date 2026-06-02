#!/usr/bin/env python3
"""Extract joint range statistics from a rosbag file.

Usage:
    ros2 run dog_tool js_range_extractor --bag /path/to/bag.db3 --output ranges.txt

Output:
    ranges.txt: per-joint min/max/mean/std grouped by joint number
"""

import argparse
import math
import sys
from pathlib import Path
from typing import Dict, List, Optional

try:
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
except ImportError as exc:
    print("rosbag2_py not found. Install: pip install rosbag2_py", file=sys.stderr)
    raise exc

from sensor_msgs.msg import JointState
import rclpy
from rclpy.serialization import deserialize_message


def extract_ranges(bag_path: str, topic: str) -> Dict:
    """Read bag and compute per-joint statistics."""
    
    rclpy.init()
    
    reader = SequentialReader()
    storage_opts = StorageOptions(uri=str(Path(bag_path).resolve()), storage_id='sqlite3')
    converter_opts = ConverterOptions('', 'cdr')
    reader.open(storage_opts, converter_opts)

    joint_data: Dict[str, List[float]] = {}
    timestamps: List[float] = []
    msg_count = 0
    first_stamp: Optional[float] = None

    while reader.has_next():
        (topic_name, data, stamp) = reader.read_next()
        if topic_name != topic:
            continue
        
        msg = deserialize_message(data, JointState())
        msg_count += 1
        
        if first_stamp is None:
            first_stamp = float(stamp) * 1e-9
        
        rel_time = float(stamp) * 1e-9 - first_stamp
        timestamps.append(rel_time)
        
        for i, name in enumerate(msg.name):
            if i >= len(msg.position):
                continue
            val = float(msg.position[i])
            if name not in joint_data:
                joint_data[name] = []
            joint_data[name].append(val)

    print(f"Processed {msg_count} messages, {len(joint_data)} joints")
    
    leg_order = ['LF_', 'RF_', 'RB_', 'LB_']
    jnum_order = ['Joint_1', 'Joint_2', 'Joint_3']
    
    result = {}
    for jnum in jnum_order:
        for leg in leg_order:
            name = leg + jnum
            if name not in joint_data:
                continue
            values = joint_data[name]
            if not values:
                continue
            n = len(values)
            min_val = min(values)
            max_val = max(values)
            mean_val = sum(values) / n
            variance = sum((v - mean_val) ** 2 for v in values) / n
            std_val = math.sqrt(variance)
            
            result[name] = {
                "min": round(min_val, 6),
                "max": round(max_val, 6),
                "mean": round(mean_val, 6),
                "std": round(std_val, 6),
            }
    
    return {
        "bag_name": Path(bag_path).name,
        "duration_sec": round(timestamps[-1], 2) if timestamps else 0.0,
        "msg_count": msg_count,
        "joints": result
    }


def generate_report(data: Dict) -> str:
    """Generate text report grouped by joint number."""
    joints = data['joints']
    lines = []
    
    lines.append("=" * 60)
    lines.append(f"Joint Range Summary ({data['bag_name']})")
    lines.append(f"Duration: {data['duration_sec']:.2f}s, {data['msg_count']} msgs")
    lines.append("=" * 60)
    
    labels = {'Joint_1': 'J1 (髋外摆)', 'Joint_2': 'J2 (髋俯仰)', 'Joint_3': 'J3 (膝关节)'}
    leg_order = ['LF_', 'RF_', 'RB_', 'LB_']
    jnum_order = ['Joint_1', 'Joint_2', 'Joint_3']
    
    for jnum in jnum_order:
        joints_in_group = {k: v for k, v in joints.items() if k.endswith(jnum)}
        if not joints_in_group:
            continue
        
        lines.append("")
        lines.append("-" * 60)
        lines.append(labels[jnum])
        lines.append("-" * 60)
        lines.append(f"{'':18} {'min':>10} {'max':>10} {'mean':>10} {'std':>10} {'range':>10}")
        lines.append("-" * 60)
        
        for name in leg_order:
            full_name = name + jnum
            if full_name not in joints_in_group:
                continue
            s = joints_in_group[full_name]
            lines.append(f"{full_name:18} {s['min']:>+10.4f} {s['max']:>+10.4f} "
                         f"{s['mean']:>+10.4f} {s['std']:>10.4f} {s['max']-s['min']:>10.4f}")
    
    lines.append("")
    lines.append("=" * 60)
    lines.append("")
    lines.append("Compact:")
    lines.append("-" * 70)
    for name in leg_order:
        for jnum in jnum_order:
            full_name = name + jnum
            if full_name not in joints:
                continue
            s = joints[full_name]
            lines.append(f"{full_name:15} min={s['min']:+.4f}  max={s['max']:+.4f}  mean={s['mean']:+.4f}±{s['std']:.4f}")
    
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description='Extract joint range statistics from rosbag')
    parser.add_argument('--bag', '-b', required=True, help='Path to rosbag')
    parser.add_argument('--topic', '-t', default='/joint_states', help='Topic to analyze')
    parser.add_argument('--output', '-o', default='ranges.txt', help='Output txt file')
    args = parser.parse_args()

    if not Path(args.bag).exists():
        print(f"ERROR: Bag file not found: {args.bag}", file=sys.stderr)
        sys.exit(1)

    print(f"Reading bag: {args.bag}")
    data = extract_ranges(args.bag, args.topic)
    
    report = generate_report(data)
    
    with open(args.output, 'w') as f:
        f.write(report)
    
    print(f"\nSaved to: {args.output}")
    print(report)


if __name__ == '__main__':
    main()
