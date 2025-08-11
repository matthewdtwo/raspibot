#!/usr/bin/env python3

import os
import sys

# Set environment variable to suppress libcamera logs
os.environ['LIBCAMERA_LOG_LEVELS'] = 'ERROR'

sys.path.append('../src')
from camera import Camera

print("Testing camera with suppressed logging...")
camera = Camera()
print("Camera initialized successfully!")

image_path = camera.take_snapshot()
print(f"Snapshot taken: {image_path}")
