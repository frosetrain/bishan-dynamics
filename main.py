from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait

hub = PrimeHub()

left_motor = Motor(Port.B, positive_direction=Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.F, positive_direction=Direction.CLOCKWISE)
third_motor = Motor(Port.C, positive_direction=Direction.COUNTERCLOCKWISE)
left_sensor = ColorSensor(Port.A)
right_sensor = ColorSensor(Port.E)
side_sensor = ColorSensor(Port.D)
side_sensor.detectable_colors((Color.RED, Color.NONE))
db = DriveBase(left_motor, right_motor, 56, 160)
db.settings(straight_acceleration=1500, turn_rate=300, turn_acceleration=1000)

WHITE = 93
BLACK = 10

_SLOT_TIMES = [
    [443, 697],
    [784, 1035],
    [1104, 1364],
    [1439, 1699],
    [2289, 2553],
    [2615, 2869],
    [2938, 3167],
    [3258, 3500],
]

# Positions of slots
SLOT_ANGLES = [
    [202, 278],
    [302, 378],
    [402, 478],
    [502, 578],
    [758, 834],
    [858, 934],
    [958, 1034],
    [1058, 1134],
]

tallies = {
    # NONE, BLACK, WHITE
    0: [0, 0, 0],
    1: [0, 0, 0],
    2: [0, 0, 0],
    3: [0, 0, 0],
    4: [0, 0, 0],
    5: [0, 0, 0],
    6: [0, 0, 0],
    7: [0, 0, 0],
}


def attempt():
    left_motor.reset_angle(0)
    slot = 0
    pthing = Color.NONE

    while True:
        rl = (right_sensor.reflection() - BLACK) / (WHITE - BLACK)
        thing = side_sensor.color()
        if thing == Color.RED and pthing == Color.NONE:
            pthing = thing
            print("on", left_motor.angle())
        elif thing == Color.NONE and pthing == Color.RED:
            pthing = thing
            print("off", left_motor.angle())
        db.drive(150, (rl - 0.5) * 100)


def detect():
    left_motor.reset_angle(0)
    slot = 0

    while True:
        # print(slot)
        # ll = (left_sensor.reflection() - BLACK) / (WHITE - BLACK)
        rl = (right_sensor.reflection() - BLACK) / (WHITE - BLACK)
        # sc = side_sensor.color()
        sr = side_sensor.reflection()
        left_motor_angle = left_motor.angle()
        if sr <= 3:
            sc = Color.NONE
        elif sr <= 30:
            sc = Color.BLACK
        else:
            sc = Color.WHITE

        if (
            left_motor_angle > SLOT_ANGLES[slot][0]
            and left_motor_angle < SLOT_ANGLES[slot][1]
        ):
            if sc == Color.NONE:
                tallies[slot][0] += 1
            if sc == Color.BLACK:
                tallies[slot][1] += 1
            if sc == Color.WHITE:
                tallies[slot][2] += 3

        if left_motor_angle > SLOT_ANGLES[slot][1]:
            slot += 1
            if slot == 8:
                break
        db.drive(200, (rl - 0.5) * 100)
        # db.drive(100, (ll - rl) * 69)
    print(tallies)
    for i in range(8):
        if max(tallies[i]) == tallies[i][0]:
            print("NONE")
        if max(tallies[i]) == tallies[i][1]:
            print("BLACK")
        if max(tallies[i]) == tallies[i][2]:
            print("WHITE")


if __name__ == "__main__":
    third_motor.reset_angle()
    third_motor.run_target(200, 0, wait=False)
    left_motor.run_angle(360, 270)
    right_motor.run_angle(360, 270)
    detect()
    db.straight(75)
    db.turn(-25)
    db.straight(130)
    third_motor.run_angle(200, 45)
    db.straight(-130)
    db.turn(45)
    third_motor.run_angle(200, -45)
    db.straight(70)
    third_motor.run_angle(200, 45)
    db.straight(-140)
    db.turn(80)

    while True:
        ll = (left_sensor.reflection() - BLACK) / (WHITE - BLACK)
        rl = (right_sensor.reflection() - BLACK) / (WHITE - BLACK)
        db.drive(150, (ll - rl) * 100)
