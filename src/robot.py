import time
import math

from llm import LLM
from motor_controller import MotorController
from encoders import Encoders
from imu import IMU
from camera import Camera

from config import (
    WHEEL_DIAMETER,
    MIN_SPEED,
    MAX_SPEED,
    TURN_MIN_SPEED,
    TURN_MAX_SPEED,
    L_MTR,
    R_MTR,
    FWD,
    RWD,
    PULSES_PER_ROTATION_LEFT,
    PULSES_PER_ROTATION_RIGHT,
    ENCODER_LEFT_SIGN,
    ENCODER_RIGHT_SIGN,
)


class Robot:
    def __init__(self):
        self.motors = MotorController()
        self.encoders = Encoders()
        self.imu = IMU()
        self.camera = Camera()
        self.llm = LLM()

    def _move_forward(self, mm: int):
        """Drive forward a distance in millimeters using a simple PID on encoder counts.

        Controls overall progress using average mm while keeping wheels aligned.
        Uses per-wheel encoder calibration and normalized signs.
        """
        # Conversion from mm to encoder pulses for each wheel
        wheel_circumference_mm = math.pi * WHEEL_DIAMETER
        pulses_per_mm_L = PULSES_PER_ROTATION_LEFT / wheel_circumference_mm
        pulses_per_mm_R = PULSES_PER_ROTATION_RIGHT / wheel_circumference_mm
        target_pulses_L = int(mm * pulses_per_mm_L)
        target_pulses_R = int(mm * pulses_per_mm_R)

        # PID gains (tune on hardware)
        Kp = 0.25
        Ki = 0.0
        Kd = 0.02

        # Steering gain
        K_steer = 0.035
        # Loop/tolerance
        dt = 0.05
        tolerance_mm = 3.0
        tolerance_pulses_L = max(2, int(tolerance_mm * pulses_per_mm_L))
        tolerance_pulses_R = max(2, int(tolerance_mm * pulses_per_mm_R))
        integral = 0.0
        prev_error_mm = mm
        integral_limit = 4 * MAX_SPEED

        # Timeout based on distance
        est_mm_per_s = 80.0
        timeout_s = max(2.0, mm / est_mm_per_s + 1.0)
        start_time = time.time()

        # Reset encoders
        self.encoders.reset_counts()

        def clamp(val, lo, hi):
            return max(lo, min(hi, val))

        try:
            while True:
                if time.time() - start_time > timeout_s:
                    break

                # Read encoders and normalize sign so forward is positive
                raw_left_count, raw_right_count = self.encoders.get_counts()
                left_count = ENCODER_LEFT_SIGN * raw_left_count
                right_count = ENCODER_RIGHT_SIGN * raw_right_count

                # Per-wheel pulse errors
                err_L_pulses = target_pulses_L - left_count
                err_R_pulses = target_pulses_R - right_count

                # Overall error in mm (average of both wheels)
                avg_mm_done = 0.5 * (left_count / pulses_per_mm_L + right_count / pulses_per_mm_R)
                error_mm = mm - avg_mm_done

                # Completion when both wheels within their tolerances
                if abs(err_L_pulses) <= tolerance_pulses_L and abs(err_R_pulses) <= tolerance_pulses_R:
                    break

                # PID core
                integral += error_mm * dt
                integral = clamp(integral, -integral_limit, integral_limit)
                derivative = (error_mm - prev_error_mm) / dt
                prev_error_mm = error_mm
                pid_output = Kp * error_mm + Ki * integral + Kd * derivative

                # Base speed (forward only)
                speed_cmd = int(abs(pid_output))
                if speed_cmd < MIN_SPEED:
                    speed_cmd = MIN_SPEED
                speed_cmd = clamp(speed_cmd, 0, MAX_SPEED)

                # Steering correction in mm space
                left_mm = left_count / pulses_per_mm_L
                right_mm = right_count / pulses_per_mm_R
                diff_mm = left_mm - right_mm
                steer = int(K_steer * diff_mm)

                left_speed = clamp(speed_cmd - steer, 0, MAX_SPEED)
                right_speed = clamp(speed_cmd + steer, 0, MAX_SPEED)

                self.motors.set_motor(L_MTR, FWD, int(left_speed))
                self.motors.set_motor(R_MTR, FWD, int(right_speed))

                time.sleep(dt)
        finally:
            self.motors.stop_motors()



    def _move_backward(self, mm: int):
        pass

    def _rotate_cw(self, deg: int, debug: bool = False):
        # Clockwise is negative heading (with our IMU: CCW positive)
        self._rotate_by_deg(-abs(deg), debug=debug)

    def _rotate_ccw(self, deg: int, debug: bool = False):
        # Counter-clockwise is positive heading
        self._rotate_by_deg(abs(deg), debug=debug)

    def _rotate_by_deg(self, deg: float, debug: bool = False):
        """Rotate in place by a signed angle in degrees using IMU feedback.

        Positive deg => CCW, Negative => CW.
        """
        # Zero heading to current orientation for a relative rotation
        try:
            self.imu.calibrate_zero(samples=15)
        except Exception:
            # If IMU not available, do nothing
            return

        target_rad = math.radians(deg)

        # Control parameters - much more conservative
        dt = 0.06  # Slower loop for stability
        tol_deg = 1.8  # Reasonable tolerance
        deadband_deg = 1.0  # Moderate deadband
        hold_time = 0.35  # Reasonable hold time
        tol_rad = math.radians(tol_deg)
        deadband_rad = math.radians(deadband_deg)
        max_turn_speed = min(TURN_MAX_SPEED, MAX_SPEED)

        # Simple oscillation damping
        prev_error = 0.0
        error_trend = 0.0
        damping_active = False

        # Timeout guard proportional to requested rotation  
        timeout_s = max(4.0, 0.1 * abs(deg) + 2.0)
        start = time.time()

        def clamp(v, lo, hi):
            return max(lo, min(hi, v))

        last_within = None
        loop_count = 0
        try:
            while True:
                if time.time() - start > timeout_s:
                    if debug:
                        print(f"Rotation timeout after {timeout_s:.1f}s")
                    break

                heading = self.imu.get_orientation()  # radians, CCW positive
                # Small unwrap: compute shortest signed difference
                err = target_rad - heading
                # Wrap to [-pi, pi]
                err = (err + math.pi) % (2 * math.pi) - math.pi

                # Calculate error trend for damping
                if loop_count > 0:
                    error_trend = (err - prev_error) / dt
                prev_error = err

                # Detect rapid oscillation
                if abs(error_trend) > math.radians(30):  # More than 30°/s change rate
                    damping_active = True
                elif abs(err) > math.radians(5):  # Reset damping when far from target
                    damping_active = False

                # Apply deadband to prevent tiny corrections that cause hunting
                effective_deadband = deadband_rad * (2.0 if damping_active else 1.0)
                if abs(err) <= effective_deadband:
                    if debug and loop_count % 8 == 0:
                        status = "damped" if damping_active else "normal"
                        print(f"In deadband ({status}): err={math.degrees(err):.2f}°")
                    time.sleep(dt)
                    loop_count += 1
                    continue

                if abs(err) <= tol_rad:
                    # Use simple tolerance-based stopping
                    if last_within is None:
                        last_within = time.time()
                        if debug:
                            print(f"Entered tolerance: err={math.degrees(err):.2f}°")
                    elif time.time() - last_within >= hold_time:
                        if debug:
                            print(f"Hold complete after {time.time() - last_within:.3f}s, stopping")
                        break
                else:
                    if last_within is not None and debug:
                        print(f"Left tolerance band: err={math.degrees(err):.2f}°")
                    last_within = None

                # Simplified speed control with heavy damping
                err_deg = abs(math.degrees(err))
                
                if err_deg < 2.0:
                    speed = TURN_MIN_SPEED
                elif err_deg < 10.0:
                    # Gentle linear ramp
                    ratio = (err_deg - 2.0) / 8.0
                    speed_range = max_turn_speed - TURN_MIN_SPEED
                    speed = int(TURN_MIN_SPEED + ratio * speed_range * 0.6)
                else:
                    # Cap at moderate speed
                    speed = int(TURN_MIN_SPEED + (max_turn_speed - TURN_MIN_SPEED) * 0.8)

                # Apply additional damping when oscillating
                if damping_active:
                    speed = max(TURN_MIN_SPEED, int(speed * 0.75))

                speed = clamp(speed, TURN_MIN_SPEED, max_turn_speed)

                # Debug every 8 loops
                if debug and loop_count % 8 == 0:
                    elapsed = time.time() - start
                    in_tol = "TOL" if abs(err) <= tol_rad else "TURN"
                    damp_status = "DAMP" if damping_active else "----"
                    hold_time_str = f"hold:{time.time() - last_within:.2f}s" if last_within else "---"
                    print(f"{elapsed:.2f}s {in_tol} err:{math.degrees(err):+6.2f}° rate:{math.degrees(error_trend):+5.0f}°/s spd:{speed:3d} {damp_status} {hold_time_str}")

                # Determine directions for in-place rotation
                if err > 0:  # need to rotate CCW (positive heading direction)
                    # CCW: left wheel forward, right wheel backward (SWAPPED TO FIX DIRECTION)
                    self.motors.set_motor(L_MTR, FWD, speed)
                    self.motors.set_motor(R_MTR, RWD, speed)
                else:       # need to rotate CW (negative heading direction)
                    # CW: left wheel backward, right wheel forward (SWAPPED TO FIX DIRECTION)
                    self.motors.set_motor(L_MTR, RWD, speed)
                    self.motors.set_motor(R_MTR, FWD, speed)

                time.sleep(dt)
                loop_count += 1
        finally:
            self.motors.stop_motors()
            if debug:
                final_heading = self.imu.get_orientation()
                final_err = math.degrees(target_rad - final_heading)
                print(f"Final error: {final_err:+.2f}°, loops: {loop_count}")

    def _take_snapshot(self):
        return self.camera.take_snapshot()
    
    def explore(self):
        # self._move_forward(100)
        self._rotate_ccw(90)