#!/usr/bin/env python3
"""
Simple script to extract LiDAR points from a ROS bag to a CSV file.
Requires: pip install rosbags numpy

Usage:
    python3 bag_to_xyz.py <bag_file> <topic> <output_file> [max_points]

Example:
    python3 bag_to_xyz.py IndoorOffice1.bag /avia/livox/lidar points.csv
"""

import sys
import struct
import argparse
from pathlib import Path

try:
    from rosbags.rosbag1 import Reader
    from rosbags.serde import deserialize_cdr, ros1_to_cdr
except ImportError:
    print("Error: 'rosbags' library not found. Please install it using:")
    print("  pip install rosbags")
    sys.exit(1)

def get_field_offset(fields, name):
    offset = 0
    for field in fields:
        if field.name == name:
            return offset, field.datatype
        # 4 bytes for FLOAT32, etc. Simplified for common PointCloud2
        if field.datatype == 7: # FLOAT32
            offset += 4
        elif field.datatype == 4: # UINT16
            offset += 2
        elif field.datatype == 2: # UINT8
            offset += 1
        else:
            offset += 4 # Assume 4 for others for now
    return None, None

def parse_pointcloud2(msg, max_points_per_msg=10000):
    # Simplified parser for PointCloud2
    # This is fragile and assumes standard float32 fields for x,y,z
    
    width = msg.width
    height = msg.height
    point_step = msg.point_step
    row_step = msg.row_step
    data = msg.data
    is_bigendian = msg.is_bigendian
    fields = msg.fields
    
    # Find offsets
    x_off = -1
    y_off = -1
    z_off = -1
    i_off = -1
    
    offset = 0
    for f in fields:
        if f.name == 'x': x_off = f.offset
        elif f.name == 'y': y_off = f.offset
        elif f.name == 'z': z_off = f.offset
        elif f.name == 'intensity': i_off = f.offset
    
    if x_off == -1 or y_off == -1 or z_off == -1:
        return []

    points = []
    
    fmt = '<f' if not is_bigendian else '>f'
    
    # Limit number of points to avoid massive files for POC
    total_points = width * height
    step = max(1, total_points // max_points_per_msg)
    
    for i in range(0, total_points, step):
        base = i * point_step
        if base + point_step > len(data):
            break
            
        x = struct.unpack_from(fmt, data, base + x_off)[0]
        y = struct.unpack_from(fmt, data, base + y_off)[0]
        z = struct.unpack_from(fmt, data, base + z_off)[0]
        
        intensity = 0.5
        if i_off != -1:
            # intensity often float or uint8
            # try float first
            try:
                intensity = struct.unpack_from(fmt, data, base + i_off)[0]
                intensity /= 255.0 # Normalize roughly
            except:
                pass
                
        points.append((x, y, z, intensity))
        
    return points

def main():
    parser = argparse.ArgumentParser(description='Convert ROS bag PointCloud2 to CSV')
    parser.add_argument('bag', help='Input ROS bag file')
    parser.add_argument('topic', help='Lidar topic name')
    parser.add_argument('output', help='Output CSV file')
    parser.add_argument('--limit', type=int, default=100000, help='Total points limit')
    
    args = parser.parse_args()
    
    out_path = Path(args.output)
    count = 0
    
    with Reader(args.bag) as reader:
        connections = [x for x in reader.connections if x.topic == args.topic]
        if not connections:
            print(f"Topic {args.topic} not found in bag.")
            return

        print(f"Reading {args.topic} from {args.bag}...")
        
        with open(out_path, 'w') as f:
            f.write("x,y,z,intensity\n")
            
            for connection, timestamp, rawdata in reader.messages(connections=connections):
                if connection.msgtype == 'sensor_msgs/PointCloud2':
                    msg = deserialize_cdr(ros1_to_cdr(rawdata, connection.msgtype), connection.msgtype)
                    points = parse_pointcloud2(msg)
                    
                    for p in points:
                        f.write(f"{p[0]:.4f},{p[1]:.4f},{p[2]:.4f},{p[3]:.4f}\n")
                        count += 1
                        
                    if count >= args.limit:
                        print(f"Reached limit of {args.limit} points.")
                        break
                else:
                    # livox custom msg handling would go here, skipping for standard PC2
                    print(f"Skipping msg type {connection.msgtype} (only sensor_msgs/PointCloud2 supported in this basic script)")

    print(f"Wrote {count} points to {out_path}")

if __name__ == '__main__':
    main()

