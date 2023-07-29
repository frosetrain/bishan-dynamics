from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

hub = PrimeHub()
l_ref = 0
r_ref = 0
left_motor = Motor(Port.B, positive_direction=Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.F, positive_direction=Direction.CLOCKWISE)
# third_motor = Motor(Port.D, positive_direction=Direction.CLOCKWISE)
left_sensor = ColorSensor(Port.A)
right_sensor = ColorSensor(Port.E)
# side_sensor = ColorSensor(Port.C)
# side_sensor.detectable_colors((Color.WHITE, Color.BLACK, Color.NONE))
db = DriveBase(left_motor, right_motor, 56, 160)
db.settings(straight_acceleration=1500, turn_rate=300, turn_acceleration=1000)
stopwatch = StopWatch()
print(db.settings())

WHITE = 93
BLACK = 10


def drive_to_inter(speed, direction, turn_angle=0, delay=0):
    stopwatch.reset()
    while True:
        l_ref = (left_sensor.reflection() - BLACK) / (WHITE - BLACK)
        r_ref = (right_sensor.reflection() - BLACK) / (WHITE - BLACK)
        # if direction == "left":
        # db.drive(speed, (0.5-r_ref) * 100)
        # elif direction == "right":
        # db.drive(speed, (l_ref-0.5) * 100)
        if True:
            db.drive(speed, (l_ref - r_ref) * 50)

        l_ref = (left_sensor.reflection() - BLACK) / (WHITE - BLACK)
        r_ref = (right_sensor.reflection() - BLACK) / (WHITE - BLACK)
        # print(l_ref, r_ref)
        if direction == "left":
            if l_ref < 0.2 and r_ref > 0.5 and stopwatch.time() > delay:
                # if l_ref < 0.2:
                print(stopwatch.time())
                print("we hit left")
                db.straight(69)
                if turn_angle != 0:
                    db.turn(turn_angle)
                else:
                    db.turn(-90)
                return
        elif direction == "right":
            if l_ref > 0.5 and r_ref < 0.2 and stopwatch.time() > delay:
                print("we hit right")
                db.straight(69)
                if turn_angle != 0:
                    db.turn(turn_angle)
                else:
                    db.turn(90)
                return
        else:  # double black
            if l_ref < 0.1 and r_ref < 0.1 and stopwatch.time() > delay:
                print("pov: you saw black", l_ref, r_ref)
                db.straight(69)
                db.turn(turn_angle)
                return


if __name__ == "__main__":
    db.straight(100, then=Stop.NONE)  # start

    while True:
        drive_to_inter(350, "left")  # go to blue port
        drive_to_inter(250, "", 180)  # go to first checkpoint and about turn
        drive_to_inter(250, "", 90)  # face the start
        drive_to_inter(350, "", 180)  # return to start

        drive_to_inter(350, "right", delay=4500)
        drive_to_inter(250, "right", 180)
        drive_to_inter(250, "left")
        drive_to_inter(350, "", 180)

        drive_to_inter(350, "right", delay=4500)
        drive_to_inter(250, "right")
        drive_to_inter(250, "left")
        drive_to_inter(250, "", 180)
        drive_to_inter(250, "right")
        drive_to_inter(250, "left")
        drive_to_inter(250, "left")
        drive_to_inter(350, "", 180)
