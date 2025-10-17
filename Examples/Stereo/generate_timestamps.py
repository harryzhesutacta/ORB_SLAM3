#!/usr/bin/env python3
"""
Generate timestamps.txt file for stereo offline processing.

This script helps create a timestamps file from existing PNG files in a directory.
You can provide either:
1. A fixed frame rate (e.g., 30 fps)
2. A CSV file with actual timestamps

Usage:
    python3 generate_timestamps.py --fps 30 --left-dir ./ir_left --output timestamps.txt
    python3 generate_timestamps.py --csv-file times.csv --left-dir ./ir_left --output timestamps.txt
"""

import argparse
import os
import glob
import sys


def get_sorted_images(directory):
    """Get sorted list of PNG files from directory."""
    pattern = os.path.join(directory, "*.png")
    images = glob.glob(pattern)
    images.sort()
    return [os.path.basename(img) for img in images]


def generate_from_fps(fps, left_dir, output_file):
    """Generate timestamps assuming a fixed frame rate."""
    images = get_sorted_images(left_dir)
    
    if not images:
        print(f"Error: No PNG files found in {left_dir}")
        sys.exit(1)
    
    dt = 1.0 / fps  # Time between frames
    
    with open(output_file, 'w') as f:
        f.write(f"# Generated timestamps for {len(images)} frames at {fps} fps\n")
        f.write("# timestamp(s) filename\n")
        
        for idx, filename in enumerate(images):
            timestamp = idx * dt
            f.write(f"{timestamp:.6f} {filename}\n")
    
    print(f"Generated {output_file} with {len(images)} timestamps at {fps} fps")


def generate_from_csv(csv_file, left_dir, output_file, time_column=0, filename_column=None):
    """Generate timestamps from a CSV file."""
    import csv
    
    images = get_sorted_images(left_dir)
    
    if not images:
        print(f"Error: No PNG files found in {left_dir}")
        sys.exit(1)
    
    timestamps = []
    filenames_from_csv = []
    
    # Read CSV file
    with open(csv_file, 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            if row and not row[0].startswith('#'):
                try:
                    timestamp = float(row[time_column])
                    timestamps.append(timestamp)
                    
                    if filename_column is not None:
                        filenames_from_csv.append(row[filename_column])
                except (ValueError, IndexError):
                    continue
    
    if not timestamps:
        print(f"Error: No valid timestamps found in {csv_file}")
        sys.exit(1)
    
    # Use filenames from CSV if available, otherwise use sorted image list
    if filenames_from_csv and len(filenames_from_csv) == len(timestamps):
        output_list = list(zip(timestamps, filenames_from_csv))
    elif len(timestamps) == len(images):
        output_list = list(zip(timestamps, images))
    else:
        print(f"Error: Number of timestamps ({len(timestamps)}) doesn't match number of images ({len(images)})")
        sys.exit(1)
    
    # Write output
    with open(output_file, 'w') as f:
        f.write(f"# Generated from {csv_file}\n")
        f.write("# timestamp(s) filename\n")
        
        for timestamp, filename in output_list:
            f.write(f"{timestamp:.6f} {filename}\n")
    
    print(f"Generated {output_file} with {len(output_list)} timestamps from {csv_file}")


def main():
    parser = argparse.ArgumentParser(
        description='Generate timestamps.txt for stereo offline processing',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    
    parser.add_argument('--fps', type=float, 
                        help='Frame rate (e.g., 30.0) - generates evenly spaced timestamps')
    parser.add_argument('--csv-file', type=str,
                        help='CSV file containing timestamps')
    parser.add_argument('--time-column', type=int, default=0,
                        help='Column index for timestamps in CSV (default: 0)')
    parser.add_argument('--filename-column', type=int, default=None,
                        help='Column index for filenames in CSV (default: None, use sorted PNG files)')
    parser.add_argument('--left-dir', type=str, required=True,
                        help='Directory containing left IR images (used to get list of files)')
    parser.add_argument('--output', type=str, default='timestamps.txt',
                        help='Output timestamps file (default: timestamps.txt)')
    
    args = parser.parse_args()
    
    # Validate arguments
    if not args.fps and not args.csv_file:
        parser.error("Either --fps or --csv-file must be specified")
    
    if args.fps and args.csv_file:
        parser.error("Cannot specify both --fps and --csv-file")
    
    if not os.path.isdir(args.left_dir):
        print(f"Error: Directory not found: {args.left_dir}")
        sys.exit(1)
    
    # Generate timestamps
    if args.fps:
        generate_from_fps(args.fps, args.left_dir, args.output)
    else:
        generate_from_csv(args.csv_file, args.left_dir, args.output, 
                         args.time_column, args.filename_column)


if __name__ == '__main__':
    main()
