from controller import Robot, Motor, Keyboard
from typing import cast


class DriveRobotController:
    def __init__(self) -> None:
        self.robot = Robot()
        self.timestep = 64

        self.keyboard = self.robot.getKeyboard()
        self.keyboard.enable(self.timestep)

        self.left_motor = cast(Motor, self.robot.getDevice("motor_1"))
        self.right_motor = cast(Motor, self.robot.getDevice("motor_2"))

        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

    def _handle_keyboard_input(self) -> bool:
        keys_down = set()

        key = self.keyboard.getKey()
        while key >= 0:
            keys_down.add(key)
            key = self.keyboard.getKey()

        left_speed, right_speed = 0.0, 0.0

        if ord("W") in keys_down:
            left_speed += 5.0
            right_speed += 5.0
        if ord("S") in keys_down:
            left_speed -= 5.0
            right_speed -= 5.0
        if ord("A") in keys_down:
            left_speed -= 2.5
            right_speed += 2.5
        if ord("D") in keys_down:
            left_speed += 2.5
            right_speed -= 2.5
        if ord(" ") in keys_down:
            left_speed, right_speed = 0.0, 0.0
            print("exiting...")
            return False

        self.left_motor.setVelocity(left_speed)
        self.right_motor.setVelocity(right_speed)

        return True

    def run(self):
        print("use WASD or arrow keys to control, SPACE to stop, ESC to exit")

        while self.robot.step(self.timestep) != -1:
            if not self._handle_keyboard_input():
                break


def main():
    controller = DriveRobotController()
    controller.run()


if __name__ == "__main__":
    main()
