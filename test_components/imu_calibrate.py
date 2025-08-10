#!/usr/bin/env python3

import sys
import time
import math

# Allow imports from src
sys.path.append('../src')

try:
    import qwiic_icm20948
except Exception:
    qwiic_icm20948 = None

# Read current sign settings so calibration matches your orientation
try:
    from config import MAG_SIGN_X, MAG_SIGN_Y, MAG_SIGN_Z
except Exception:
    MAG_SIGN_X = 1.0
    MAG_SIGN_Y = 1.0
    MAG_SIGN_Z = 1.0


def _read_mag(imu):
    """Return magnetometer tuple (mx, my, mz). Falls back across attribute names."""
    # Ensure fresh sample
    try:
        if hasattr(imu, 'dataReady') and imu.dataReady():
            imu.getAgmt()
        elif hasattr(imu, 'getAgmt'):
            imu.getAgmt()
    except Exception:
        pass

    def gx(primary, fallback):
        return getattr(imu, primary, getattr(imu, fallback, 0.0))

    mx = gx('mX', 'mxRaw')
    my = gx('mY', 'myRaw')
    mz = gx('mZ', 'mzRaw')
    return mx, my, mz


def calibrate(duration_s: float = 30.0, apply_signs: bool = True):
    if qwiic_icm20948 is None:
        print('ICM-20948 library not available in this environment.', file=sys.stderr)
        sys.exit(1)

    imu = qwiic_icm20948.QwiicIcm20948()
    if not getattr(imu, 'connected', False):
        print("IMU not connected. Check wiring and I2C.", file=sys.stderr)
        sys.exit(2)

    imu.begin()
    time.sleep(0.25)

    mnx = mny = mnz = float('inf')
    mxx = mxy = mxz = float('-inf')
    count = 0
    start = time.time()

    print(f'Collecting magnetometer samples for {duration_s:.1f}s...')
    print('Move the robot in a slow figure-8 and rotate it through all orientations.')

    while time.time() - start < duration_s:
        mx, my, mz = _read_mag(imu)
        if apply_signs:
            mx *= MAG_SIGN_X
            my *= MAG_SIGN_Y
            mz *= MAG_SIGN_Z

        mnx = min(mnx, mx)
        mny = min(mny, my)
        mnz = min(mnz, mz)
        mxx = max(mxx, mx)
        mxy = max(mxy, my)
        mxz = max(mxz, mz)
        count += 1

        # Light pacing to avoid spamming I2C; adjust as needed
        time.sleep(0.02)

    # Hard-iron offsets
    off_x = (mnx + mxx) / 2.0
    off_y = (mny + mxy) / 2.0
    off_z = (mnz + mxz) / 2.0

    # Soft-iron scale factors (optional)
    rad_x = (mxx - mnx) / 2.0
    rad_y = (mxy - mny) / 2.0
    rad_z = (mxz - mnz) / 2.0
    avg_rad = (rad_x + rad_y + rad_z) / 3.0 if count > 0 else 1.0
    scl_x = avg_rad / rad_x if rad_x else 1.0
    scl_y = avg_rad / rad_y if rad_y else 1.0
    scl_z = avg_rad / rad_z if rad_z else 1.0

    print('\nSamples:', count)
    print('Raw ranges (after sign application if enabled):')
    print(f'  X: min={mnx:.2f} max={mxx:.2f} span={mxx - mnx:.2f}')
    print(f'  Y: min={mny:.2f} max={mxy:.2f} span={mxy - mny:.2f}')
    print(f'  Z: min={mnz:.2f} max={mxz:.2f} span={mxz - mnz:.2f}')

    print('\nRecommended hard-iron offsets (paste into src/config.py):')
    print(f'  MAG_OFFSET_X = {off_x:.2f}')
    print(f'  MAG_OFFSET_Y = {off_y:.2f}')
    print(f'  MAG_OFFSET_Z = {off_z:.2f}')

    print('\nOptional soft-iron scale factors (apply in code if needed):')
    print(f'  MAG_SCALE_X = {scl_x:.3f}')
    print(f'  MAG_SCALE_Y = {scl_y:.3f}')
    print(f'  MAG_SCALE_Z = {scl_z:.3f}')
    print('\nNote: If CCW yaw is negative, try flipping MAG_SIGN_Y (or X) in config and recalibrate.')


if __name__ == '__main__':
    try:
        dur = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
    except ValueError:
        print('Usage: imu_calibrate.py [duration_seconds]', file=sys.stderr)
        sys.exit(64)
    calibrate(dur, apply_signs=True)
