from controller import Robot, Motor, Keyboard, GPS
import serial
from typing import cast


class DriveDroneController:
    def __init__(self) -> None:
        self.robot = Robot()
        self.keyboard = self.robot.getKeyboard()
        self.ser = serial.Serial(port="/dev/ttyACM0", baudrate=9600, timeout=5)

        self.motor_defs = ["m1_motor", "m2_motor", "m3_motor", "m4_motor"]
        self.motors: list[Motor] = []

        self.timestep = 64
        self.sim_step = 0

        self.current_altitude = 0
        self.target_altitude = 0

        self.altitude_step = 0.5

        self._setup()

    def _setup(self):
        # start listening to keyboard inputs
        self.keyboard.enable(self.timestep)

        # initialize motor components
        for motor in self.motor_defs:
            self.motors.append(cast(Motor, self.robot.getDevice(motor)))

        for motor in self.motors:
            motor.setPosition(float("inf"))
            motor.setVelocity(0.0)

        # initialize gps component
        self.gps = cast(GPS, self.robot.getDevice("gps"))
        self.gps.enable(self.timestep)

    def _send_to_esp32(self, command: str):
        self.ser.write(bytes(f"{command}\n", "UTF-8"))

    def _update_altitude(self):
        gps_values = self.gps.getValues()

        if gps_values and len(gps_values) >= 2:
            self.current_altitude = gps_values[2]
            self._send_to_esp32(f"ALT:{self.current_altitude:.4f}")

    def _handle_recieved_data(self):
        if self.ser.in_waiting:
            line = self.ser.readline().decode()

            if line.startswith("MOTOR:"):
                speeds = [float(v) for v in line.split(":")[1].split(",")]

                print(speeds)

                for i, motor in enumerate(self.motors):
                    motor.setVelocity(speeds[i])

    def _handle_keyboard_input(self):
        key = self.keyboard.getKey()

        while key != -1:
            if key == Keyboard.UP:
                self.target_altitude += self.altitude_step
                self._send_to_esp32(f"SET:{self.target_altitude:.4f}")
            elif key == Keyboard.DOWN:
                self.target_altitude -= self.altitude_step
                self._send_to_esp32(f"SET:{self.target_altitude:.4f}")

            key = self.keyboard.getKey()

    def run(self):
        self.up_start = None
        while self.robot.step(self.timestep) != -1:
            self._update_altitude()
            self._handle_recieved_data()
            self._handle_keyboard_input()

            # if self.up_start is not None and self.sim_step - self.up_start > 50:
            #     print("hovering triggered")
            #     self._send_to_esp32("HOVER")
            #     self.up_start = None

            self.sim_step += 1


def main():
    controller = DriveDroneController()
    controller.run()


if __name__ == "__main__":
    main()
