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
        # Get current heading as our starting point for relative rotation
        try:
            start_heading = self.imu.get_heading()
        except Exception:
            # If IMU not available, do nothing
            return

        target_heading = start_heading + deg

        # Control parameters - balanced for accuracy
        dt = 0.05  # Moderate updates
        tol_deg = 1.5  # Tighter tolerance for better accuracy
        deadband_deg = 0.5  # Smaller deadband for less oscillation
        hold_time = 0.8  # Longer hold time for stability
        max_turn_speed = min(TURN_MAX_SPEED, MAX_SPEED)

        # PID parameters - very conservative for stability
        Kp = 2.0   # Much lower proportional gain
        Kd = 25.0  # Very high derivative for strong damping
        Ki = 0.005 # Minimal integral
        
        # PID state
        prev_error = 0.0
        integral = 0.0
        integral_limit = 20.0
        
        # Oscillation detection
        error_history = []
        max_history = 6

        # Timeout guard proportional to requested rotation  
        timeout_s = max(8.0, 0.12 * abs(deg) + 3.0)  # Shorter timeout
        start = time.time()

        def clamp(v, lo, hi):
            return max(lo, min(hi, v))

        def angle_diff(target, current):
            """Calculate shortest angular difference between two angles in degrees."""
            diff = target - current
            # Wrap to [-180, 180]
            while diff > 180:
                diff -= 360
            while diff <= -180:
                diff += 360
            return diff

        last_within = None
        loop_count = 0
        try:
            while True:
                if time.time() - start > timeout_s:
                    if debug:
                        print(f"Rotation timeout after {timeout_s:.1f}s")
                    break

                current_heading = self.imu.get_heading()  # degrees
                # Calculate shortest signed difference
                err = angle_diff(target_heading, current_heading)

                # Check for oscillation pattern
                error_history.append(err)
                if len(error_history) > max_history:
                    error_history.pop(0)
                
                # Detect oscillation: sign changes in recent history
                oscillating = False
                if len(error_history) >= 4:
                    sign_changes = 0
                    for i in range(1, len(error_history)):
                        if (error_history[i] > 0) != (error_history[i-1] > 0):
                            sign_changes += 1
                    oscillating = sign_changes >= 2  # 2+ sign changes = oscillating

                # Calculate PID terms
                if loop_count > 0:
                    derivative = (err - prev_error) / dt
                else:
                    derivative = 0.0
                
                integral += err * dt
                integral = max(-integral_limit, min(integral_limit, integral))  # Clamp integral
                
                # PID output in radians
                pid_output = Kp * err + Ki * integral + Kd * derivative
                prev_error = err

                # Apply deadband to prevent tiny corrections that cause hunting
                if abs(err) <= deadband_deg:
                    if debug and loop_count % 8 == 0:
                        print(f"In deadband: err={err:.2f}°")
                    time.sleep(dt)
                    loop_count += 1
                    continue

                if abs(err) <= tol_deg:
                    # Use simple tolerance-based stopping
                    if last_within is None:
                        last_within = time.time()
                        if debug:
                            print(f"Entered tolerance: err={err:.2f}°")
                    elif time.time() - last_within >= hold_time:
                        if debug:
                            print(f"Hold complete after {time.time() - last_within:.3f}s, stopping")
                        break
                else:
                    if last_within is not None and debug:
                        print(f"Left tolerance band: err={err:.2f}°")
                    last_within = None

                # Adaptive speed profile based on error and approach rate
                err_deg = abs(err)
                approaching = (err > 0 and derivative < 0) or (err < 0 and derivative > 0)
                
                # Base speed from error magnitude
                if err_deg < 1.0:
                    base_speed = TURN_MIN_SPEED  # 90 PWM - minimum
                elif err_deg < 3.0:
                    base_speed = TURN_MIN_SPEED + 3  # 93 PWM - crawl
                elif err_deg < 8.0:
                    base_speed = TURN_MIN_SPEED + 6  # 96 PWM - slow
                elif err_deg < 20.0:
                    base_speed = TURN_MIN_SPEED + 9  # 99 PWM - moderate
                else:
                    base_speed = TURN_MIN_SPEED + 12  # 102 PWM - max
                
                # Rate-based adjustment for smooth approach
                rate_deg_s = abs(derivative) if derivative else 0
                if approaching and rate_deg_s > 10:  # Fast approach - slow down
                    speed = max(TURN_MIN_SPEED, base_speed - 5)
                elif rate_deg_s > 20:  # Very fast rate - emergency slow
                    speed = TURN_MIN_SPEED
                else:
                    speed = base_speed
                
                # Oscillation detection and mitigation
                if oscillating:
                    speed = TURN_MIN_SPEED  # Drop to absolute minimum
                
                speed = clamp(speed, TURN_MIN_SPEED, max_turn_speed)

                # Debug every 8 loops
                if debug and loop_count % 8 == 0:
                    elapsed = time.time() - start
                    in_tol = "TOL" if abs(err) <= tol_deg else "TURN"
                    hold_time_str = f"hold:{time.time() - last_within:.2f}s" if last_within else "---"
                    deriv_rate = derivative if derivative else 0
                    hz = loop_count / elapsed if elapsed > 0 else 0
                    osc_status = "OSC" if oscillating else "---"
                    print(f"{elapsed:.2f}s {in_tol} err:{err:+6.2f}° rate:{deriv_rate:+5.0f}°/s spd:{speed:3d} {osc_status} {hold_time_str} ({hz:.0f}Hz)")

                # Determine directions for in-place rotation
                if err > 0:  # need to rotate CCW (positive heading direction)
                    # CCW: left wheel backward, right wheel forward (FIXED DIRECTION)
                    self.motors.set_motor(L_MTR, RWD, speed)
                    self.motors.set_motor(R_MTR, FWD, speed)
                else:       # need to rotate CW (negative heading direction)
                    # CW: left wheel forward, right wheel backward (FIXED DIRECTION)
                    self.motors.set_motor(L_MTR, FWD, speed)
                    self.motors.set_motor(R_MTR, RWD, speed)

                time.sleep(dt)
                loop_count += 1
        finally:
            self.motors.stop_motors()
            if debug:
                final_heading = self.imu.get_heading()
                final_err = angle_diff(target_heading, final_heading)
                print(f"Final error: {final_err:+.2f}°, loops: {loop_count}")

    def _take_snapshot(self):
        return self.camera.take_snapshot()
    
    def explore(self):
        # self._move_forward(100)
        self._rotate_ccw(90)