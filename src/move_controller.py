import time
import math
import signal
import atexit
from typing import Tuple, NamedTuple

from qwiic_otos import QwiicOTOS

from motor_controller import MotorController
from encoders import Encoders
from imu import IMU

from models import ActualMovement, MovementParams, MovementResult, PIDState

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


class MoveController:
    def __init__(self):
        self.motors = MotorController()
        self.encoders = Encoders()
        self.imu = IMU()
        self.otos = QwiicOTOS()

        print("Calibrating OTOS IMU...")
        time.sleep(1)
        self.otos.calibrateImu()
        


        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        atexit.register(self.cleanup)
        print("Move controller initialized")

    def _signal_handler(self, signum, frame):
        print(f"\nReceived signal {signum}, stopping motors...")
        self.cleanup()
        exit(0)

    def cleanup(self):
        try:
            self.motors.stop_motors()
            print("Motors stopped safely")
        except Exception as e:
            print(f"Error stopping motors: {e}")

    def __enter__(self):
        return self

    def __exit__(self):
        self.cleanup()

    def _calculate_movement_params(self, mm: int, direction: int = 1) -> MovementParams:

        wheel_circumference_mm = math.pi * WHEEL_DIAMETER
        pulses_per_mm_L = PULSES_PER_ROTATION_LEFT / wheel_circumference_mm
        pulses_per_mm_R = PULSES_PER_ROTATION_RIGHT / wheel_circumference_mm
        target_pulses_L = int(direction * mm * pulses_per_mm_L)
        target_pulses_R = int(direction * mm * pulses_per_mm_R)
        
        return MovementParams(
            wheel_circumference_mm=wheel_circumference_mm,
            pulses_per_mm_L=pulses_per_mm_L,
            pulses_per_mm_R=pulses_per_mm_R,
            target_pulses_L=target_pulses_L,
            target_pulses_R=target_pulses_R
        )

    def _calculate_movement_progress(self, movement_params: MovementParams, is_backward: bool = False) -> Tuple[float, int, int, int, int]:
        raw_left_count, raw_right_count = self.encoders.get_counts()
        left_count = ENCODER_LEFT_SIGN * raw_left_count
        right_count = ENCODER_RIGHT_SIGN * raw_right_count

        err_L_pulses = movement_params.target_pulses_L - left_count
        err_R_pulses = movement_params.target_pulses_R - right_count

        if is_backward:
            avg_mm_done = 0.5 * (abs(left_count) / movement_params.pulses_per_mm_L + 
                               abs(right_count) / movement_params.pulses_per_mm_R)
        else:
            avg_mm_done = 0.5 * (left_count / movement_params.pulses_per_mm_L + 
                               right_count / movement_params.pulses_per_mm_R)

        return avg_mm_done, left_count, right_count, err_L_pulses, err_R_pulses

    def _calculate_movement_speeds(self, error_mm: float, left_count: int, right_count: int, 
                                 movement_params: MovementParams, pid_state: PIDState, 
                                 integral: float, prev_error_mm: float, is_backward: bool = False) -> Tuple[int, int, float, float]:
        def clamp(val, lo, hi):
            return max(lo, min(hi, val))

        new_integral = integral + error_mm * pid_state.dt
        new_integral = clamp(new_integral, -pid_state.integral_limit_factor * MAX_SPEED, 
                           pid_state.integral_limit_factor * MAX_SPEED)
        derivative = (error_mm - prev_error_mm) / pid_state.dt
        pid_output = pid_state.Kp * error_mm + pid_state.Ki * new_integral + pid_state.Kd * derivative


        speed_cmd = int(abs(pid_output))
        
        if error_mm < 20:
            speed_cmd = int(speed_cmd * max(0.3, error_mm / 20.0))
        
        if speed_cmd < MIN_SPEED and error_mm > 5:
            speed_cmd = MIN_SPEED
        speed_cmd = clamp(speed_cmd, 0, MAX_SPEED)

        if is_backward:
            left_mm = abs(left_count) / movement_params.pulses_per_mm_L
            right_mm = abs(right_count) / movement_params.pulses_per_mm_R
        else:
            left_mm = left_count / movement_params.pulses_per_mm_L
            right_mm = right_count / movement_params.pulses_per_mm_R
            
        diff_mm = left_mm - right_mm
        steer = int(pid_state.K_steer * diff_mm)

        left_speed = clamp(speed_cmd - steer, 0, MAX_SPEED)
        right_speed = clamp(speed_cmd + steer, 0, MAX_SPEED)

        if error_mm < 2:
            left_speed = right_speed = 0

        return left_speed, right_speed, new_integral, derivative

    def _log_linear_movement_completion(self, direction: str, mm: int, movement_params: MovementParams, is_backward: bool = False) -> MovementResult:

        raw_left_final, raw_right_final = self.encoders.get_counts()
        left_final = ENCODER_LEFT_SIGN * raw_left_final
        right_final = ENCODER_RIGHT_SIGN * raw_right_final
        
        if is_backward:
            final_mm = 0.5 * (abs(left_final) / movement_params.pulses_per_mm_L + 
                            abs(right_final) / movement_params.pulses_per_mm_R)
        else:
            final_mm = 0.5 * (left_final / movement_params.pulses_per_mm_L + 
                            right_final / movement_params.pulses_per_mm_R)
            
        error_mm = final_mm - mm

        otos_pos_y = self.otos.getPosition().y * 25.4

        return MovementResult(
            type="linear",
            target=mm,
            actual=ActualMovement(
                measured=int(final_mm),
                optical=int(abs(otos_pos_y)),
                warning="Optical telemetry deviates significantly from encoder readings. You are possibly stuck with wheel slippage." if abs(abs(otos_pos_y) - final_mm) > (mm * 0.25) else None
            ),
            unit="mm"
        )

    @staticmethod
    def _clamp(val, lo, hi):
        return max(lo, min(hi, val))

    def move_forward(self, mm: int, debug: bool = False) -> MovementResult:
        return self._execute_linear_movement(mm, "forward", FWD, debug=debug)

    def move_backward(self, mm: int, debug: bool = False) -> MovementResult:
        return self._execute_linear_movement(mm, "backward", RWD, debug=debug)

    def _execute_linear_movement(self, mm: int, direction_name: str, motor_direction: int, debug: bool = False) -> MovementResult:
        is_backward = motor_direction == RWD
        direction_multiplier = -1 if is_backward else 1
        
        movement_params = self._calculate_movement_params(mm, direction_multiplier)
        
        pid_state = PIDState()
        tolerance_pulses_L = max(2, int(pid_state.tolerance_mm * movement_params.pulses_per_mm_L))
        tolerance_pulses_R = max(2, int(pid_state.tolerance_mm * movement_params.pulses_per_mm_R))
        integral = 0.0
        prev_error_mm = mm

        est_mm_per_s = 80.0
        timeout_s = max(2.0, mm / est_mm_per_s + 1.0)
        start_time = time.time()
        self.encoders.reset_counts()

        self.otos.resetTracking()

        try:
            loop_count = 0
            while True:
                loop_count += 1
                if time.time() - start_time > timeout_s:
                    if debug:
                        print("  Timeout reached!")
                    break

                avg_mm_done, left_count, right_count, err_L_pulses, err_R_pulses = \
                    self._calculate_movement_progress(movement_params, is_backward)

                error_mm = mm - avg_mm_done

                if abs(err_L_pulses) <= tolerance_pulses_L and abs(err_R_pulses) <= tolerance_pulses_R:
                    break

                if avg_mm_done > mm + 20:
                    break

                # Calculate motor speeds
                left_speed, right_speed, integral, derivative = \
                    self._calculate_movement_speeds(error_mm, left_count, right_count, 
                                                  movement_params, pid_state, integral, prev_error_mm, is_backward)
                
                prev_error_mm = error_mm

                # Set motor speeds
                self.motors.set_motor(L_MTR, motor_direction, int(left_speed))
                self.motors.set_motor(R_MTR, motor_direction, int(right_speed))

                time.sleep(pid_state.dt)
        finally:
            self.motors.stop_motors()
            return self._log_linear_movement_completion(direction_name, mm, movement_params, is_backward)

    def rotate_cw(self, deg: int) -> MovementResult:
        return self._execute_rotation(-abs(deg))

    def rotate_ccw(self, deg: int) -> MovementResult:
        return self._execute_rotation(abs(deg))

    @staticmethod
    def _normalize_angle(angle):
        while angle > 180:
            angle -= 360
        while angle <= -180:
            angle += 360
        return angle
    
    @staticmethod
    def _angle_diff(target, current):
        diff = MoveController._normalize_angle(target - current)
        return diff
    
    @staticmethod
    def _average_angles(angle1, angle2):
        """Average two angles using circular statistics to handle wraparound correctly."""
        # Convert to radians
        rad1 = math.radians(angle1)
        rad2 = math.radians(angle2)
        
        # Convert to unit vectors and average
        x = (math.cos(rad1) + math.cos(rad2)) / 2
        y = (math.sin(rad1) + math.sin(rad2)) / 2
        
        # Convert back to degrees
        avg_rad = math.atan2(y, x)
        return math.degrees(avg_rad)

    def _execute_rotation(self, target_rotation_deg: float) -> MovementResult:
        requested_deg = abs(target_rotation_deg)

        start_heading = self.imu.get_heading() # absolute

        start_otos_heading = self.otos.getPosition().h # relative

        target_heading = self._normalize_angle(start_heading + target_rotation_deg)

        tolerance = 2.0
        max_turn_time = abs(target_rotation_deg) / 45.0 + 2.0
        dt = 0.05

        turn_direction = 1 if target_rotation_deg > 0 else -1
        base_speed = TURN_MIN_SPEED + int(0.3 * (TURN_MAX_SPEED - TURN_MIN_SPEED))
        
        start_time = time.time()
        
        try:
            while time.time() - start_time < max_turn_time:
                current_heading = self.imu.get_heading()

                error = self._angle_diff(target_heading, current_heading)

                # Check if we've reached the target
                if abs(error) <= tolerance:
                    break

                speed_factor = min(1.0, abs(error) / 20.0)
                turn_speed = int(TURN_MIN_SPEED + speed_factor * (base_speed - TURN_MIN_SPEED))

                if turn_direction > 0:
                    self.motors.set_motor(L_MTR, RWD, turn_speed)
                    self.motors.set_motor(R_MTR, FWD, turn_speed)
                else:
                    self.motors.set_motor(L_MTR, FWD, turn_speed)
                    self.motors.set_motor(R_MTR, RWD, turn_speed)
                
                time.sleep(dt)
            
            # Timeout reached
            if time.time() - start_time >= max_turn_time:
                final_heading_imu = self.imu.get_heading()
                final_otos_heading = self.otos.getPosition().h
                print(f"Warning: Rotation timeout reached. Target: {target_heading:.1f}°, Final(IMU): {final_heading_imu:.1f} deg, Final(OTOS): {final_otos_heading:.1f} deg")
        finally:
            self.motors.stop_motors()
            
            # Calculate actual rotation for return value
            final_heading_imu = self.imu.get_heading()
            final_otos_heading = abs(self.otos.getPosition().h)
            
            raw_rotation_imu = self._angle_diff(final_heading_imu, start_heading)

            raw_rotation_otos = self._angle_diff(final_otos_heading, start_otos_heading)

            actual_rotation = abs(self._average_angles(raw_rotation_imu, raw_rotation_otos))

            error_deg = abs(self._angle_diff(actual_rotation, requested_deg))

            warning = f"Large rotation error detected: {error_deg:+.1f} deg. You may be stuck" if abs(error_deg) > tolerance else None

            if warning:
                print(warning)

            return MovementResult(
                type="rotation",
                target=int(requested_deg),
                actual=ActualMovement(
                    measured=int(actual_rotation),
                    optical=int(final_otos_heading),
                    warning=warning
                ),
                unit="deg"
            )


if __name__ == "__main__":
    mc = MoveController()
    print(mc.move_backward(200))
    print(mc.rotate_cw(120))
    print(mc.move_forward(250))
    mc.cleanup()