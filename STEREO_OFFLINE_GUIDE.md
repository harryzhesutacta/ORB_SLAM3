# Stereo RealSense D435i Offline Processing Guide

This guide explains how to run ORB-SLAM3 with stereo IR images stored as PNG files.

## Overview

The `stereo_realsense_D435i_offline` executable allows you to run ORB-SLAM3 using pre-recorded stereo infrared (IR) images stored as PNG files in separate directories.

## Directory Structure

Your data should be organized as follows:

```
dataset/
├── ir_left/
│   ├── 0000000000.png
│   ├── 0000000001.png
│   ├── 0000000002.png
│   └── ...
├── ir_right/
│   ├── 0000000000.png
│   ├── 0000000001.png
│   ├── 0000000002.png
│   └── ...
└── timestamps.txt
```

## Timestamps File Format

The `timestamps.txt` file should contain one line per image pair with the following format:

```
# timestamp filename
1234567890.123456 0000000000.png
1234567890.156789 0000000001.png
1234567890.190123 0000000002.png
...
```

- **First column**: Timestamp in seconds (can be floating point)
- **Second column**: Filename (should match files in both ir_left and ir_right folders)
- Lines starting with `#` are treated as comments and ignored

### Example timestamps.txt:
```
# RealSense D435i Stereo IR Recording
# timestamp(s) filename
0.000000 0000000000.png
0.033333 0000000001.png
0.066667 0000000002.png
0.100000 0000000003.png
```

## Building

1. Build ORB-SLAM3 normally:
```bash
cd ORB_SLAM3
./build.sh
```

2. The executable will be created at:
```
Examples/Stereo/stereo_realsense_D435i_offline
```

## Running

Basic usage:
```bash
./Examples/Stereo/stereo_realsense_D435i_offline \
    Vocabulary/ORBvoc.txt \
    Examples/Stereo/RealSense_D435i.yaml \
    /path/to/ir_left \
    /path/to/ir_right \
    /path/to/timestamps.txt
```

With trajectory output:
```bash
./Examples/Stereo/stereo_realsense_D435i_offline \
    Vocabulary/ORBvoc.txt \
    Examples/Stereo/RealSense_D435i.yaml \
    /path/to/ir_left \
    /path/to/ir_right \
    /path/to/timestamps.txt \
    trajectory_output.txt
```

## Arguments

1. **path_to_vocabulary**: Path to ORB vocabulary file (e.g., `Vocabulary/ORBvoc.txt`)
2. **path_to_settings**: Path to camera calibration YAML file (e.g., `Examples/Stereo/RealSense_D435i.yaml`)
3. **path_to_left_folder**: Directory containing left IR images
4. **path_to_right_folder**: Directory containing right IR images
5. **path_to_times_file**: Path to timestamps file
6. **trajectory_file_name** (optional): Output trajectory filename

## Camera Calibration

Make sure your YAML settings file contains the correct calibration parameters for your RealSense D435i camera. You can use the existing `Examples/Stereo/RealSense_D435i.yaml` or create your own.

Key parameters to check:
- Camera intrinsics (fx, fy, cx, cy)
- Distortion coefficients
- Stereo baseline
- Image resolution

## Recording Data from RealSense Camera

If you need to record data from a live RealSense camera, you can use this Python script:

```python
import pyrealsense2 as rs
import numpy as np
import cv2
import time
import os

# Create output directories
os.makedirs('ir_left', exist_ok=True)
os.makedirs('ir_right', exist_ok=True)

# Configure RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)
config.enable_stream(rs.stream.infrared, 2, 640, 480, rs.format.y8, 30)

# Start streaming
profile = pipeline.start(config)

# Disable emitter
sensor = profile.get_device().first_depth_sensor()
sensor.set_option(rs.option.emitter_enabled, 0)

frame_count = 0
timestamps_file = open('timestamps.txt', 'w')
timestamps_file.write('# timestamp(s) filename\n')

try:
    while True:
        # Wait for frames
        frames = pipeline.wait_for_frames()
        
        # Get IR frames
        ir_left = frames.get_infrared_frame(1)
        ir_right = frames.get_infrared_frame(2)
        
        if not ir_left or not ir_right:
            continue
        
        # Convert to numpy arrays
        ir_left_image = np.asanyarray(ir_left.get_data())
        ir_right_image = np.asanyarray(ir_right.get_data())
        
        # Get timestamp
        timestamp = ir_left.get_timestamp() / 1000.0  # Convert ms to seconds
        
        # Save images
        filename = f'{frame_count:010d}.png'
        cv2.imwrite(f'ir_left/{filename}', ir_left_image)
        cv2.imwrite(f'ir_right/{filename}', ir_right_image)
        
        # Write timestamp
        timestamps_file.write(f'{timestamp:.6f} {filename}\n')
        
        frame_count += 1
        
        if frame_count % 100 == 0:
            print(f'Recorded {frame_count} frames')
            
except KeyboardInterrupt:
    print(f'Recording stopped. Total frames: {frame_count}')
finally:
    timestamps_file.close()
    pipeline.stop()
```

## Output

The SLAM system will:
- Display the current tracking status in a viewer window
- Print tracking statistics (median/mean tracking time)
- Save the camera trajectory to `CameraTrajectory.txt` (or specified filename) in KITTI format

## Troubleshooting

### "Failed to load image" error
- Check that image filenames in `timestamps.txt` match the actual files in the directories
- Verify that image paths are correct
- Ensure PNG files are valid grayscale images

### "Different number of left and right images" error
- Ensure both `ir_left` and `ir_right` folders have the same number of images
- Verify that filenames match between the two folders

### Poor tracking performance
- Check camera calibration parameters
- Verify that images are actually stereo pairs (synchronized)
- Ensure sufficient texture in the scene
- Check that the emitter was turned off during recording (IR images should not have the dot pattern)

## Example

```bash
# Navigate to ORB_SLAM3 directory
cd ORB_SLAM3

# Run with example data
./Examples/Stereo/stereo_realsense_D435i_offline \
    Vocabulary/ORBvoc.txt \
    Examples/Stereo/RealSense_D435i.yaml \
    /home/user/dataset/ir_left \
    /home/user/dataset/ir_right \
    /home/user/dataset/timestamps.txt \
    my_trajectory.txt
```

The trajectory will be saved to `my_trajectory.txt` and `CameraTrajectory.txt`.
