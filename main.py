from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

hub = PrimeHub()

left_motor = Motor(Port.B, positive_direction=Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.F, positive_direction=Direction.CLOCKWISE)
third_motor = Motor(Port.D, positive_direction=Direction.CLOCKWISE)
left_sensor = ColorSensor(Port.A)
right_sensor = ColorSensor(Port.E)
side_sensor = ColorSensor(Port.C)
side_sensor.detectable_colors((Color.WHITE, Color.BLACK, Color.NONE))
db = DriveBase(left_motor, right_motor, 56, 160)

WHITE = 93
BLACK = 10

SLOT_TIMES = [
    [443, 697],
    [784, 1035],
    [1104, 1364],
    [1439, 1699],
    [2289, 2553],
    [2615, 2869],
    [2938, 3167],
    [3258, 3500],
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


def detect():
    stopwatch = StopWatch()
    slot = 0

    while True:
        # print(slot)
        # ll = (left_sensor.reflection() - BLACK) / (WHITE - BLACK)
        rl = (right_sensor.reflection() - BLACK) / (WHITE - BLACK)
        # sc = side_sensor.color()
        sr = side_sensor.reflection()
        if sr <= 3:
            sc = Color.NONE
        elif sr <= 30:
            sc = Color.BLACK
        else:
            sc = Color.WHITE

        if (
            stopwatch.time() > SLOT_TIMES[slot][0]
            and stopwatch.time() < SLOT_TIMES[slot][1]
        ):
            if sc == Color.NONE:
                tallies[slot][0] += 1
            if sc == Color.BLACK:
                tallies[slot][1] += 1
            if sc == Color.WHITE:
                tallies[slot][2] += 3
        if stopwatch.time() > SLOT_TIMES[slot][1]:
            slot += 1
            if slot == 8:
                break
        db.drive(150, (rl - 0.5) * 100)
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
    third_motor.run_target(0, 0)
    left_motor.run_angle(360, 300)
    right_motor.run_angle(360, 300)
    detect()
    db.straight(125)
    db.turn(-90)
    db.straight(100)
    db.turn(90)
    db.straight(200)
    db.turn(180)

    # while True:
        # ll = (left_sensor.reflection() - BLACK) / (WHITE - BLACK)
        # rl = (right_sensor.reflection() - BLACK) / (WHITE - BLACK)
        # db.drive(150, (ll - rl) * 100)
