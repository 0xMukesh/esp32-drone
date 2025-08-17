from controller import Robot, Motor
import serial
from typing import cast


class DriveDroneController:
    def __init__(self) -> None:
        self.robot = Robot()
        self.keyboard = self.robot.getKeyboard()
        self.ser = serial.Serial(port="/dev/ttyACM0", baudrate=9600, timeout=5)

        self.timestep = 64
        self.sim_step = 0

        self.motor_defs = ["m1_motor", "m2_motor", "m3_motor", "m4_motor"]
        self.motors: list[Motor] = []

        for motor in self.motor_defs:
            self.motors.append(cast(Motor, self.robot.getDevice(motor)))

        for motor in self.motors:
            motor.setPosition(float("inf"))
            motor.setVelocity(0.0)

    def run(self):
        while self.robot.step(self.timestep) != -1:
            if self.sim_step < 20:
                self.ser.write(bytes("hello\n", "UTF-8"))

            self.sim_step += 1


def main():
    controller = DriveDroneController()
    controller.run()


if __name__ == "__main__":
    main()
