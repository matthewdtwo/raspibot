import qwiic_scmd
import time

from config import FWD, RWD, L_MTR, R_MTR



class MotorController:
    def __init__(self):
        self.motorController = qwiic_scmd.QwiicScmd()

        if self.motorController.connected == False:
            print("Motor is not connected")
            return

        self.motorController.begin()
        time.sleep(0.25)
        self.motorController.set_drive(R_MTR, FWD, 0)
        self.motorController.set_drive(L_MTR, RWD, 0)

        self.motorController.enable()
        time.sleep(.25)

    def set_motor(self, motor, direction, speed):
        self.motorController.set_drive(motor, direction, speed)

    def stop_motors(self):
        self.motorController.set_drive(R_MTR, FWD, 0)
        self.motorController.set_drive(L_MTR, FWD, 0)

        

if __name__ == "__main__":
    motor_controller = MotorController()
    
    # set both motors to forward for 5 seconds
    motor_controller.set_motor(L_MTR, FWD, 100)
    motor_controller.set_motor(R_MTR, FWD, 100)
    time.sleep(2)
    motor_controller.stop_motors()