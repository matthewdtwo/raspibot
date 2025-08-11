#!/usr/bin/env python3

import sys
sys.path.append('../src')

from camera import Camera
from llm import LLMs

# Test the camera and vision pipeline
camera = Camera()
llm = LLMs()

print("Taking snapshot...")
image_path = camera.take_snapshot()
print(f"Snapshot saved to: {image_path}")

print("Analyzing image...")
description = llm._get_image_description(image_path)
print(f"Image description: {description}")
