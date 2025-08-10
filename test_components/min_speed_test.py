#!/usr/bin/env python3

import time
import sys
from typing import Tuple

# fix path to import componets from ../src
sys.path.append('../src')

from motor_controller import MotorController
from encoders import Encoders
from config import MIN_SPEED, MAX_SPEED, L_MTR, R_MTR, FWD





def read_counts(enc: Encoders) -> Tuple[int, int]:
    try:
        return enc.get_counts()
    except Exception as e:
        print(f"Failed to read encoders: {e}", file=sys.stderr)
        return 0, 0


def run_min_speed_test(duration_s: float = 2.0, speed: int = MIN_SPEED, right_offset: int = 25) -> None:
    print(f"Min-speed test: duration={duration_s}s speed={speed} right_offset={right_offset}")

    motors = MotorController()
    enc = Encoders()

    # Reset and settle
    enc.reset_counts()
    time.sleep(0.1)

    l0, r0 = read_counts(enc)
    t0 = time.time()

    # Command both motors same direction, with +offset on right to overcome deadband
    right_speed = min(MAX_SPEED, speed + max(0, int(right_offset)))
    motors.set_motor(L_MTR, FWD, speed)
    motors.set_motor(R_MTR, FWD, right_speed)
    print(f"Commanded speeds -> left:{speed} right:{right_speed}")

    # Optional mid-run sample for rough rates
    time.sleep(max(0.0, duration_s / 2))
    l_mid, r_mid = read_counts(enc)
    t_mid = time.time()

    # Finish
    time.sleep(max(0.0, duration_s - (t_mid - t0)))
    motors.stop_motors()
    time.sleep(0.1)

    l1, r1 = read_counts(enc)
    t1 = time.time()

    # Compute deltas and rates
    dl = l1 - l0
    dr = r1 - r0
    dt_total = max(1e-3, t1 - t0)
    dl_mid = l_mid - l0
    dr_mid = r_mid - r0
    dt_mid = max(1e-3, t_mid - t0)

    print("Results:")
    print(f"  Left:  delta={dl}  rate={dl/dt_total:.1f} cnt/s  mid_rate={dl_mid/dt_mid:.1f} cnt/s")
    print(f"  Right: delta={dr}  rate={dr/dt_total:.1f} cnt/s  mid_rate={dr_mid/dt_mid:.1f} cnt/s")
    if abs(dr) < abs(dl) * 0.8:
        print("  Note: Right side significantly lower. Consider raising MIN_SPEED or adding right trim.")


if __name__ == "__main__":
    try:
        dur = 2.0
        spd = MIN_SPEED
        if len(sys.argv) > 1:
            dur = float(sys.argv[1])
        if len(sys.argv) > 2:
            spd = int(sys.argv[2])
        roff = 15
        if len(sys.argv) > 3:
            roff = int(sys.argv[3])
        run_min_speed_test(dur, spd, roff)
    except KeyboardInterrupt:
        print("Interrupted.")
        sys.exit(0)
