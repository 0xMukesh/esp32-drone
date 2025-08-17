from controller import Robot, Motor, Keyboard
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

        self._setup()

    def _setup(self):
        # enable listening to keyboard inputs
        self.keyboard.enable(self.timestep)

        # initialize virtual motor components
        for motor in self.motor_defs:
            self.motors.append(cast(Motor, self.robot.getDevice(motor)))

        for motor in self.motors:
            motor.setPosition(float("inf"))
            motor.setVelocity(0.0)

    def _send_to_esp32(self, command: str):
        self.ser.write(bytes(f"{command}\n", "UTF-8"))

    def _handle_recieved_data(self):
        if self.ser.in_waiting:
            line = self.ser.readline().decode()

            if line.startswith("DATA:"):
                speeds = [int(v) for v in line.split("DATA:")[1].split(",")]

                for i, motor in enumerate(self.motors):
                    motor.setVelocity(speeds[i])

    def _handle_keyboard_input(self):
        key = self.keyboard.getKey()

        while key != -1:
            if key == Keyboard.UP:
                self._send_to_esp32("UP")

            key = self.keyboard.getKey()

    def run(self):
        while self.robot.step(self.timestep) != -1:
            self._handle_recieved_data()
            self._handle_keyboard_input()

            self.sim_step += 1


def main():
    controller = DriveDroneController()
    controller.run()


if __name__ == "__main__":
    main()
