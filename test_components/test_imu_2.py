import sys

sys.path.append("../src")

from imu import IMU
import time

imu = IMU(alpha=0.25)  # Almost pure magnetometer
print(f"Gyro bias: {imu.g_bias}")
for i in range(15):
    h = imu.get_heading()
    dbg = getattr(imu, "_dbg", {})
    print(
        f"{i+1:2d}: "
        f"yaw={h:7.2f}°, "
        f"yaw_mag={dbg.get('yaw_mag_deg', float('nan')):7.2f}°, "
        f"B≈{dbg.get('B_raw', float('nan')):6.1f} raw, "
        f"trust={dbg.get('trust', 'na'):.1f}, "
        f"gz_raw={dbg.get('gz_raw', float('nan')):8.3f}, "
        f"gz_after_bias={dbg.get('gz_after_bias', float('nan')):8.3f}"
    )
    time.sleep(0.5)
