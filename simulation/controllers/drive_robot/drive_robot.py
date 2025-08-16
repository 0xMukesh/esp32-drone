from controller import Robot, Motor
from typing import cast

robot = Robot()
timestep = 64

left_motor = cast(Motor, robot.getDevice("motor_1"))
right_motor = cast(Motor, robot.getDevice("motor_2"))

left_motor.setPosition(float("inf"))  # Enable velocity control
right_motor.setPosition(float("inf"))  # Enable velocity control

# Set initial velocity to 0
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

while robot.step(timestep) != -1:
    left_motor.setVelocity(0.25 * 6)
    right_motor.setVelocity(0.5 * 6)
