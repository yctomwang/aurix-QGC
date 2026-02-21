#!/usr/bin/env python3
"""
Convert .las or .laz LiDAR files to CSV (x,y,z,intensity) for QGroundControl POC.
Requires: pip install laspy numpy

Note for .laz support: You may need 'lazrs' or 'laszip'.
    pip install "laspy[lazrs]" 
    OR
    pip install "laspy[laszip]"

Usage:
    python3 laz_to_xyz.py <input_file.laz> <output_file.csv> [limit]
"""

import sys
import argparse
import numpy as np

try:
    import laspy
except ImportError:
    print("Error: 'laspy' library not found. Please install it:")
    print("  pip install laspy")
    sys.exit(1)

def main():
    parser = argparse.ArgumentParser(description='Convert LAS/LAZ to CSV')
    parser.add_argument('input', help='Input .las or .laz file')
    parser.add_argument('output', help='Output .csv file')
    parser.add_argument('--limit', type=int, default=50000, help='Max points to output')
    
    args = parser.parse_args()
    
    print(f"Opening {args.input}...")
    try:
        las = laspy.read(args.input)
    except Exception as e:
        print(f"Error reading file: {e}")
        print("If this is a .laz file, ensure you have lazrs or laszip installed:")
        print("  pip install \"laspy[lazrs]\"")
        return

    # Extract coordinates
    # Scale and offset are applied automatically by laspy getters usually, 
    # but accessing .x .y .z directly is safest high-level API.
    
    points = np.vstack((las.x, las.y, las.z)).transpose()
    
    # Try to get intensity
    if hasattr(las, 'intensity'):
        intensities = las.intensity
        # Normalize intensity to 0-1 range if it looks like 16-bit or 8-bit
        max_i = np.max(intensities)
        if max_i > 0:
            intensities = intensities.astype(float) / max_i
    else:
        intensities = np.full(len(points), 0.5)

    total_points = len(points)
    print(f"File contains {total_points} points.")
    
    # Downsample if needed
    if total_points > args.limit:
        print(f"Downsampling to {args.limit} points...")
        indices = np.linspace(0, total_points-1, args.limit, dtype=int)
        points = points[indices]
        intensities = intensities[indices]
    
    print(f"Writing to {args.output}...")
    with open(args.output, 'w') as f:
        f.write("x,y,z,intensity\n")
        for i in range(len(points)):
            p = points[i]
            # Center the cloud? For now, keep original coords but maybe 
            # the viewer expects local coords. 
            # The previous POC used small local coords. 
            # Real world data might be huge UTM.
            # We might need to subtract the mean to center it for the viewer.
            
            # Simple centering for POC viewing comfort:
            # x = p[0] - las.header.x_offset
            # But let's just write raw first, or maybe center based on first point?
            
            # Let's subtract the mean of the subset to center it at 0,0,0
            # This makes it visible in the orbit view which looks at 0,0,0
            
            pass

    # Actually let's do the writing with centering
    center = np.mean(points, axis=0)
    print(f"Centering data by subtracting: {center}")
    
    points = points - center
    
    with open(args.output, 'w') as f:
        f.write("x,y,z,intensity\n")
        for i in range(len(points)):
            x, y, z = points[i]
            inten = intensities[i]
            f.write(f"{x:.4f},{y:.4f},{z:.4f},{inten:.4f}\n")

    print("Done.")

if __name__ == '__main__':
    main()

