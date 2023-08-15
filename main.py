from pybricks.hubs import PrimeHub
from pybricks.parameters import Direction, Port
from pybricks.pupdevices import ColorSensor, Motor
from pybricks.robotics import GyroDriveBase
from pybricks.tools import StopWatch, wait

hub = PrimeHub()
left_motor = Motor(Port.E, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.A, Direction.CLOCKWISE)
main_motor = Motor(Port.C, Direction.CLOCKWISE)
left_sensor = ColorSensor(Port.D)
right_sensor = ColorSensor(Port.F)
mech_motor = Motor(Port.B, Direction.COUNTERCLOCKWISE)

db = GyroDriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=160)
db.settings(straight_acceleration=450, turn_rate=300, turn_acceleration=400)
# left_sensor.detectable_colors([Color.RED, Color.NONE, Color.WHITE])
# right_sensor.detectable_colors([Color.NONE, Color.WHITE, Color.BLACK])
stopwatch = StopWatch()

WHITE = 100
BLACK = 0


def gen_slot_distances(begin, slot_width, slot_interval, mid_gap) -> list[int]:
    """This algorithm is very goofy."""
    distances = []
    for i in range(9):  # NOTE: Why 9? There are 8 slots
        if i < 4:
            distances.append(
                [
                    begin + (i * slot_interval) + (i * slot_width),
                    begin + (i * slot_interval) + (i * slot_width) + slot_width,
                ]
            )
        else:
            distances.append(
                [
                    begin
                    + ((i - 1) * slot_interval)  # NOTE: Why do we need to subtract 1?
                    + (i * slot_width)
                    + mid_gap,
                    begin
                    + ((i - 1) * slot_interval)
                    + (i * slot_width)
                    + mid_gap
                    + slot_width,
                ]
            )
    return distances


def linetrack_by_distance(distance, speed=350):
    db.reset()
    while db.distance() < distance:
        l_ref = (left_sensor.reflection() - BLACK) / (WHITE - BLACK)
        r_ref = (right_sensor.reflection() - BLACK) / (WHITE - BLACK)
        db.drive(speed, (l_ref - r_ref) * 50)
    db.stop()


def linetrack(direction, speed=350, min_distance=0):
    db.reset()
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

        if hit and db.distance() > min_distance:
            return


def racing_line_turn(angle):
    db.stop()
    initial_heading = hub.imu.heading()
    print(initial_heading)
    target_heading = initial_heading + angle
    print(target_heading)
    if angle <= 0:
        while 3 < abs(target_heading - hub.imu.heading()):
            right_motor.run(500)
    else:
        while 3 < abs(target_heading - hub.imu.heading()):
            left_motor.run(500)
    db.stop()


def ferris_wheel_turn(desired_cart):
    """Turn the ferris wheel (does not lower it)"""
    ferris_angle = mech_motor.angle()
    if desired_cart == "NONE":
        desired_ferris_angle = 450
    elif desired_cart == "WHITE":
        desired_ferris_angle = 900
    elif desired_cart == "BLACK":
        desired_ferris_angle = 1350
    else:
        desired_ferris_angle = 0

    ferris_angle_diff = desired_ferris_angle - ferris_angle

    if abs(ferris_angle_diff) > 900:
        ferris_angle_diff = 1800 - abs(ferris_angle_diff)

    mech_motor.run_angle(500, ferris_angle_diff)


def ferris_wheel_placement(desired_cart):
    """Turn the ferris wheel and lower the mechanism.
    Why does this function encapsulate another?"""
    ferris_wheel_turn(desired_cart)
    while main_motor.angle() < 0:
        # Default 0 is when sensor is at side and mechanism motor is lowered
        main_motor.run(360)


def ferris_wheel_default(desired_cart):
    """Another goofy function."""
    while main_motor.angle() > -290:
        main_motor.run(-360)
    ferris_wheel_turn(desired_cart)


def voting(slot_distances) -> list[str]:
    slot_averages = [0] * 8
    slot_no = -1
    current_tally = 0
    reading_count = 0
    db.reset()
    while True:
        reflection = left_sensor.reflection()
        distance = db.distance()
        if (
            distance > slot_distances[slot_no][0] + 10
            and distance < slot_distances[slot_no][1] - 10
        ):
            current_tally += reflection
            reading_count += 1
        if db.distance() >= slot_distances[slot_no + 1][0] - 10:
            slot_averages[slot_no] = current_tally / reading_count
            slot_no += 1
            if slot_no == 8:
                break
        db.drive(300, 0)

    south_numed = [[val, i] for i, val in enumerate(slot_averages[:4])]
    north_numed = [[val, i] for i, val in enumerate(slot_averages[4:])]
    south_sorted = sorted(south_numed, reverse=True)
    north_sorted = sorted(north_numed, reverse=True)

    slot_colors = ["NONE"] * 8
    slot_colors[south_sorted[0][1]] = "WHITE"
    slot_colors[south_sorted[1][1]] = "BLACK"
    slot_colors[north_sorted[0][1]] = "WHITE"
    slot_colors[north_sorted[1][1]] = "BLACK"
    return slot_colors


if __name__ == "__main__":
    wait(300)
    main_motor.reset_angle(0)
    mech_motor.reset_angle(0)
    # mech_motor.run_target(360,0)
    # main_motor.run_angle(100,300)
    # mech_motor.run_angle(360, (mech_motor.angle() % 360) - FERRIS_ANGLES["red"])
    db.curve(2 / 3 * 162, 48)
    db.curve(2 / 3 * 162, -48)
    slot_distances = gen_slot_distances(5, 32, 17, 94)
    slot_colors = voting(slot_distances)
    print(slot_colors)
    main_motor.run_angle(200, -300)
    db.curve(-100, 25)
    db.curve(-100, -25)
    linetrack_by_distance(80)
    db.turn(180)
    main_motor.run_angle(300, 290)
    db.curve(2 / 3 * -70, -85)
    db.curve(2 / 3 * -70, 85)
    ferris_wheel_placement("WHITE")
    db.straight(-135)
    ferris_wheel_placement("BLACK")
    db.straight(135)
    db.turn(90)
    db.straight(165)
    db.turn(-90)
    db.straight(-150)
    ferris_wheel_default("fk u min sen")
    db.curve(2 / 3 * 60, -90)
    db.curve(2 / 3 * 60, 90)

    # INSERT LINE TRACK TO MIDDLE
    db.settings(straight_acceleration=300)
    linetrack("left", min_distance=15)
    racing_line_turn("left")
    linetrack("left", min_distance=200)
    db.straight(-15)
    db.curve(-100, -90)

    if slot_colors[0] != "NONE":
        ferris_wheel_placement(slot_colors[0])
        db.straight(25)
        ferris_wheel_default("fk u min sen")

    print("Min Sen is retarded")

    # linetracingtocorner(-1,-90)
    # linetracingtocorner(-1,-90)
    # SWEEPING THE RED CUBE

    # db.straight(-30)
    # db.curve(-150, -180, Stop.HOLD)
    # db.straight(-230)
    # db.curve(-150, -90, Stop.HOLD)
    # db.straight(-70)
    # test_voting()
    # while True:
    #     print(left_sensor.reflection())
    # db.straight(1)
