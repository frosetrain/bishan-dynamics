from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from usys import stdin
from uselect import poll

hub = PrimeHub()
third_motor = Motor(Port.E, positive_direction=Direction.CLOCKWISE)
left_motor = Motor(Port.C, positive_direction=Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.A, positive_direction=Direction.CLOCKWISE)
db = DriveBase(left_motor, right_motor, 56, 160)
db.settings(straight_acceleration=1500, turn_rate=300, turn_acceleration=1000)

keyboard = poll()
keyboard.register(stdin)

start_heading = hub.imu.heading()
start_motor_angle = left_motor.angle()
p = None

while True:
    if keyboard.poll(0):
        key = stdin.read(1)
        if key == " ":
            db.stop()
            if p == "w" or p == "s":
                print((left_motor.angle() - start_motor_angle) / 360 * 56 * 3.14159)
            if p == "a" or p == "d":
                print(hub.imu.heading() - start_heading)
        if key == "w":
            p = "w"
            start_motor_angle = left_motor.angle()
            db.drive(100, 0)
        if key == "s":
            p = "s"
            start_motor_angle = left_motor.angle()
            db.drive(-100, 0)
        if key == "a":
            p = "a"
            start_heading = hub.imu.heading()
            db.drive(0, -20)
        if key == "d":
            p = "d"
            start_heading = hub.imu.heading()
            db.drive(0, 20)
