from picamera2 import Picamera2
from libcamera import Transform
import tempfile
import time
import os
import logging
from datetime import datetime

# Suppress libcamera logging
os.environ['LIBCAMERA_LOG_LEVELS'] = 'ERROR'
logging.getLogger('picamera2').setLevel(logging.WARNING)

class Camera:
    def __init__(self, persistent_snapshots=True):
        self._picam2 = Picamera2()
        self._config = self._picam2.create_still_configuration(transform=Transform(vflip=1, hflip=1))
        self._picam2.configure(self._config)
        self._persistent_snapshots = persistent_snapshots
        
        # Create snapshots directory if persistent snapshots are enabled
        if self._persistent_snapshots:
            os.makedirs('../snapshots', exist_ok=True)

        self._picam2.start()
        time.sleep(1)
        print("Camera initialized")

    def take_snapshot(self) -> str:
        """Take a snapshot of what's in front of the robot and return the file path"""
        if self._persistent_snapshots:
            # Create a persistent file with timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]  # Include milliseconds
            snapshot_path = f'../snapshots/robot_snapshot_{timestamp}.jpg'
            
            # Capture directly to the persistent file
            self._picam2.capture_file(snapshot_path)
            
            return snapshot_path
        else:
            # Create a temporary file with .jpg extension (original behavior)
            temp_fd, temp_path = tempfile.mkstemp(suffix='.jpg', prefix='robot_snapshot_')
            os.close(temp_fd)  # Close the file descriptor since we'll use the path
            
            # Capture directly to the temporary file
            self._picam2.capture_file(temp_path)
            
            return temp_path