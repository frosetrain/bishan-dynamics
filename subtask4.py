from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import GyroDriveBase
from pybricks.tools import wait

hub = PrimeHub()

left_motor = Motor(Port.B, positive_direction=Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.F, positive_direction=Direction.CLOCKWISE)
third_motor = Motor(Port.C, positive_direction=Direction.COUNTERCLOCKWISE)
left_sensor = ColorSensor(Port.A)
right_sensor = ColorSensor(Port.E)
side_sensor = ColorSensor(Port.D)
side_sensor.detectable_colors((Color.RED, Color.NONE))
db = GyroDriveBase(left_motor, right_motor, 56, 160)
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

pick_angle = 10000

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
        db.drive(300, (rl - 0.5) * 100)
        # db.drive(100, (ll - rl) * 69)
    print(tallies)
    for i in range(8):
        if max(tallies[i]) == tallies[i][0]:
            print("NONE")
        if max(tallies[i]) == tallies[i][1]:
            print("BLACK")
        if max(tallies[i]) == tallies[i][2]:
            print("WHITE")


def deposit():
    db.curve(200, -60, Stop.HOLD)
    third_motor.run_angle(200, -45)
    db.curve(-200, -60, Stop.HOLD)
    db.turn(185)


def pickup():
    while left_sensor.color() != Color.RED:
        db.drive(300, 0)

    db.straight(-84)
    db.turn(-38)
    db.straight(69)
    third_motor.run_angle(200, 45)
    db.turn(162)
    db.straight(210)
    db.turn(-25)

    pick_angle = left_motor.angle()


if __name__ == "__main__":
    third_motor.reset_angle()
    third_motor.run_target(200, 0, wait=False)
    left_motor.run_angle(360, 270)
    right_motor.run_angle(360, 270)
    detect()

    while left_sensor.color() != Color.RED:
        db.drive(300, 0)

    db.stop()
    db.straight(-84)
    db.turn(-38)
    db.straight(60)
    third_motor.run_angle(200, 45)
    db.turn(162)
    db.straight(210)
    db.turn(-25)

    pick_angle = left_motor.angle()

    for i in range(1000):
        ll = (left_sensor.reflection() - BLACK) / (WHITE - BLACK)
        rl = (right_sensor.reflection() - BLACK) / (WHITE - BLACK)
        db.drive(150, (ll - rl) * 100)

    print(left_motor.angle())  # 2027

    # while False:
    #     ll = (left_sensor.reflection() - BLACK) / (WHITE - BLACK)
    #     rl = (right_sensor.reflection() - BLACK) / (WHITE - BLACK)
    #     db.drive(300, (ll - rl) * 60)
    #     #print(ll, rl)
    #     if ll < 0.1 and rl > 0.5 and left_motor.angle() > 3000: #funny left turn
    #         break

    # deposit()

    while True:
        ll = (left_sensor.reflection() - BLACK) / (WHITE - BLACK)
        rl = (right_sensor.reflection() - BLACK) / (WHITE - BLACK)
        db.drive(300, (ll - rl) * 60)
        print(pick_angle)
        if (
            ll < 0.1 and rl > 0.7 and left_motor.angle() > pick_angle + 700
        ):  # funny left turn
            deposit()

        if ll < 0.1 and rl < 0.1:  # funny left turn
            db.straight(69)
            db.turn(90)
            pickup()
            for i in range(1000):
                ll = (left_sensor.reflection() - BLACK) / (WHITE - BLACK)
                rl = (right_sensor.reflection() - BLACK) / (WHITE - BLACK)
                db.drive(150, (ll - rl) * 100)
