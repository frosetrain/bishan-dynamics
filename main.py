from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import GyroDriveBase
from pybricks.tools import wait, StopWatch

hub = PrimeHub()
third_motor = Motor(Port.E, positive_direction=Direction.COUNTERCLOCKWISE)
left_motor = Motor(Port.C, positive_direction=Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.A, positive_direction=Direction.CLOCKWISE)
side_sensor = ColorSensor(Port.F)
left_sensor = ColorSensor(Port.D)
right_sensor = ColorSensor(Port.B)
db = GyroDriveBase(left_motor, right_motor, 56, 160)
db.settings(straight_acceleration=1500, turn_rate=300, turn_acceleration=1000)
stopwatch = StopWatch()

WHITE = 93
BLACK = 10

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


def detect():
    # Total reflection for the entire block window
    tallies = [0] * 8
    left_motor.reset_angle(0)
    slot = 0

    while True:
        right_ref = (right_sensor.reflection() - BLACK) / (WHITE - BLACK)
        side_ref = side_sensor.reflection()
        left_motor_angle = left_motor.angle()
        if (
            left_motor_angle > SLOT_ANGLES[slot][0]
            and left_motor_angle < SLOT_ANGLES[slot][1]
        ):
            tallies[slot] += side_ref
        if left_motor_angle > SLOT_ANGLES[slot][1]:
            slot += 1
            if slot == 8:
                break
        db.drive(300, (right_ref - 0.5) * 100)

    south_numed = [[val, i + 1] for i, val in enumerate(tallies[:4])]
    north_numed = [[val, i + 1] for i, val in enumerate(tallies[4:])]
    south_sorted = sorted(south_numed, reverse=True)
    north_sorted = sorted(north_numed, reverse=True)
    return {
        "south": {
            "white": south_sorted[0][1],
            "black": south_sorted[1][1]
        },
        "north": {
            "white": north_sorted[0][1],
            "black": north_sorted[1][1]
        }
    }


def drive_to_inter(speed, direction, delay=0):
    stopwatch.reset()
    db.settings(straight_speed=speed)
    while True:
        l_ref = (left_sensor.reflection() - BLACK) / (WHITE - BLACK)
        r_ref = (right_sensor.reflection() - BLACK) / (WHITE - BLACK)
        db.drive(speed, (l_ref - r_ref) * 50)
        l_ref = (left_sensor.reflection() - BLACK) / (WHITE - BLACK)
        r_ref = (right_sensor.reflection() - BLACK) / (WHITE - BLACK)
        if direction == "left":
            hit = l_ref < 0.2 and r_ref > 0.5
        elif direction == "right":
            hit = l_ref > 0.5 and r_ref < 0.2
        elif direction == "double":
            hit = l_ref < 0.2 and r_ref < 0.2
            
        if hit and stopwatch.time() > delay:
            db.straight(69)
            return


third_motor.reset_angle()
third_motor.run_target(200, 0, wait=False)
left_motor.run_angle(360, 270)
right_motor.run_angle(360, 270)
positions = detect()
print(positions)
db.stop()
db.turn(30)
db.straight(165)
third_motor.run_target(200, 45)
db.straight(-165)
db.turn(-35)
third_motor.run_target(200, -5)
db.straight(105)
third_motor.run_target(200, 45)
db.turn(150)
db.straight(235)
db.turn(-55)

drive_to_inter(350, "left", 200)
db.straight(70)
db.curve(160 * 2 / 3, -90)
db.straight(50)
db.curve(160 * 2 / 3, 180)
db.straight(80)
db.turn(90)
drive_to_inter(350, "right", 500)
db.straight(70)
db.curve(160 * 2 / 3, 90)
db.straight(50)
db.curve(160 * 2 / 3, -180)
db.straight(80)
db.turn(90)
drive_to_inter(350, "right", 1000)
