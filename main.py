"""Bishan Dynamics BD23"""

from pybricks.hubs import PrimeHub
from pybricks.parameters import Color, Direction, Port, Stop
from pybricks.pupdevices import ColorSensor, Motor
from pybricks.robotics import GyroDriveBase
from pybricks.tools import wait

hub = PrimeHub()
left_motor = Motor(Port.E, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.A)
main_motor = Motor(Port.C)
left_sensor = ColorSensor(Port.D)
right_sensor = ColorSensor(Port.F)
mech_motor = Motor(Port.B, Direction.COUNTERCLOCKWISE)

db = GyroDriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=160)
db.settings(straight_acceleration=450, turn_rate=300, turn_acceleration=300)
left_sensor.detectable_colors([Color.RED, Color.NONE, Color.WHITE])
right_sensor.detectable_colors([Color.NONE, Color.WHITE, Color.BLACK])

WHITE = 100
BLACK = 0

slot_colors: list[str] = []


def gen_slot_distances(
    begin: int, slot_width: int, slot_interval: int, mid_gap: int
) -> list[list[int]]:
    """Generate the list of distances where slots are positioned

    Args:
        begin (int): Begin
        slot_width (int): Width of slot
        slot_interval (int): Gap between slots
        mid_gap (int): Gap between Pacific and Caribbean slots

    Returns:
        list[list[int]]: List of slot distances
    """
    slot_distances = []
    for i in range(9):  # NOTE: Why 9? There are 8 slots
        if i < 4:
            slot_distances.append(
                [
                    begin + (i * slot_interval) + (i * slot_width),
                    begin + (i * slot_interval) + (i * slot_width) + slot_width,
                ]
            )
        else:
            slot_distances.append(
                [
                    begin + ((i - 1) * slot_interval) + (i * slot_width) + mid_gap,
                    begin
                    + ((i - 1) * slot_interval)
                    + (i * slot_width)
                    + mid_gap
                    + slot_width,
                ]
            )
    return slot_distances


def linetrack_by_distance(distance: int) -> None:
    """Linetrack a specific distance

    Args:
        distance (int): Distance in mm
    """
    db.reset()
    while db.distance() < distance:
        db.drive(350, (left_sensor.reflection() - right_sensor.reflection()) * 2.2)
    db.stop()


def linetrack_to_corner(
    turn_direction: str,
    turn_angle: int = 0,
    min_distance: int = 30,
    backwards: bool = False,
    speed: int = 350,
) -> None:
    """Linetrack to a corner, then do a smooth turn.

    Args:
        turn_direction (str): The direction of the turn - left, double or right
        turn_angle (int, optional): The angle of the turn. Defaults to 0.
        min_distance (int, optional): The minimum distance driven before turning. Defaults to 30.
        backwards (bool, optional): Whether the bot drives backwards. Defaults to False.
        speed (int, optional): Speed of robot. Defaults to 350.
    """
    db.reset()
    if backwards:
        speed = -speed

    while True:
        l_ref = left_sensor.reflection()
        r_ref = right_sensor.reflection()
        db.drive(speed, (l_ref - r_ref) * 1.2)

        l_ref = left_sensor.reflection()
        r_ref = right_sensor.reflection()
        if turn_direction == "left":
            hit = l_ref < 0.2 and r_ref > 0.5
        elif turn_direction == "right":
            hit = l_ref > 0.5 and r_ref < 0.2
        elif turn_direction == "double":
            hit = l_ref < 0.2 and r_ref < 0.2

        if hit and db.distance() > min_distance:
            break

    db.straight(0, Stop.BRAKE)
    initial_heading = hub.imu.heading()
    target_heading = initial_heading + turn_angle
    print(initial_heading, target_heading)
    if turn_angle <= 0:
        while 3 < abs(target_heading - hub.imu.heading()):
            right_motor.run(500)
    else:
        while 3 < abs(target_heading - hub.imu.heading()):
            left_motor.run(500)
    db.straight(0, Stop.BRAKE)


def ferris_wheel_turn(desired_cart: str) -> None:
    """Turn the ferris wheel without lowering.

    Args:
        desired_cart (str): Desired cart
    """
    ferris_angle = mech_motor.angle()
    print("FERRIS: ", ferris_angle)
    if desired_cart == "RED":
        desired_ferris_angle = 450
    elif desired_cart == "WHITE":
        desired_ferris_angle = 900
    elif desired_cart == "BLACK":
        desired_ferris_angle = 1350
    else:
        desired_ferris_angle = 0
    ferris_angle_diff = desired_ferris_angle - ferris_angle

    if abs(ferris_angle_diff) < 20:
        return
    if abs(ferris_angle_diff) > 900:
        ferris_angle_diff = (
            (1800 - abs(ferris_angle_diff))
            * -1
            * ferris_angle_diff
            / abs(ferris_angle_diff)
        )
    ferris_angle = desired_ferris_angle
    print("DESIRED FERRIS ANGLE:", desired_ferris_angle)
    print("FERRIS ANGLE DIFF:", ferris_angle_diff)
    mech_motor.run_angle(500, ferris_angle_diff)


def ferris_wheel_turn_and_down(
    desired_cart: str,
) -> None:
    """Turn the ferris wheel and lower the mechanism.

    Args:
        desired_cart (str): Desired cart
    """
    if desired_cart == "RED":
        print("you screwed up")
        return
    ferris_wheel_turn(desired_cart)
    print("MAIN MOTOR: ", main_motor.angle())
    while (
        main_motor.angle() < 0
    ):  # "default" 0 is when sensor is at side and mechanism motor is lowered
        main_motor.run(360)


def ferris_wheel_up_and_turn(desired_cart: str) -> None:
    """Raise the mechanism and turn the ferris wheel.

    Args:
        desired_cart (str): Desired cart
    """
    print(main_motor.angle())
    while main_motor.angle() > -280:
        main_motor.run(-360)
    ferris_wheel_turn(desired_cart)


def voting(slot_distances: list[list[int]]) -> list[str]:
    """Improved Kenneth voting to scan cubes.
    Highest average reflection is white, second highest is black.

    Args:
        slot_distances (list[list[int]]): List of distances for the slots

    Returns:
        list[str]: The result after scanning each slot
    """
    slot_averages = [0] * 8
    slot_no = 0
    current_tally = 0
    reading_count = 0
    db.reset()
    while True:
        reflection = left_sensor.reflection()
        distance = db.distance()
        if slot_distances[slot_no][0] + 10 < distance < slot_distances[slot_no][1] - 10:
            current_tally += reflection
            reading_count += 1
        if distance >= slot_distances[slot_no + 1][0] - 10:
            print(current_tally, reading_count)
            slot_averages[slot_no] = current_tally / reading_count
            current_tally = 0
            reading_count = 0
            slot_no += 1
            if slot_no == 8:
                break
        db.drive(350, 0)

    south_numed = [[val, i] for i, val in enumerate(slot_averages[:4])]
    north_numed = [[val, i] for i, val in enumerate(slot_averages[4:])]
    south_sorted = sorted(south_numed, reverse=True)
    north_sorted = sorted(north_numed, reverse=True)

    colors = ["NONE"] * 8
    colors[south_sorted[0][1]] = "WHITE"
    colors[south_sorted[1][1]] = "BLACK"
    colors[north_sorted[0][1] + 4] = "WHITE"
    colors[north_sorted[1][1] + 4] = "BLACK"
    return colors


def deposit(slot: int) -> None:
    """Deposit an object.

    Args:
        slot (int): The slot number.
    """
    if slot_colors[slot] == "NONE":
        print("deposit('NONE') doesn't really make sense")
    if slot_colors[slot] == "WHITE" or slot_colors[slot] == "BLACK":
        ferris_wheel_turn_and_down(slot_colors[slot])
        db.straight(30)
        ferris_wheel_up_and_turn("fk u min sen")
        db.straight(-30)


if __name__ == "__main__":
    # Interesting blob of function calls.

    # mech_motor.run_target(100, 100)
    # mech_motor.run(300)
    wait(300)
    # main_motor.run_angle(100,-300)
    main_motor.reset_angle(0)

    # mech_motor.run_target(100, -225)
    # main_motor.run_angle(100,-300)
    # mech_motor.run_angle(360, (mech_motor.angle() % 360) - FERRIS_ANGLES["red"])
    db.curve(2 / 3 * 162, 48)
    db.curve(2 / 3 * 162, -48)
    db.reset()  # necessary for voting

    distances = gen_slot_distances(5, 32, 17, 94)

    # slot_colors = ["NONE", "WHITE", "BLACK", "NONE", "NONE", "WHITE", "NONE", "BLACK"]
    slot_colors = voting(distances)

    main_motor.run_angle(200, -300)

    # reset ferris wheel from compromised starting position
    mech_motor.run_angle(500, -210)
    mech_motor.reset_angle(0)

    db.curve(-100, 25)
    db.curve(-100, -25)

    linetrack_by_distance(85)
    # ensure possible cube at last slot is not collected
    db.turn(180)
    # main_motor.run_angle(300,-290)
    # linetrack_by_distance(180)

    db.curve(2 / 3 * -70, -85)
    db.curve(2 / 3 * -70, 85)
    # db.straight(50)
    ferris_wheel_turn_and_down(
        "WHITE"
    )  # by right we should lift it back up after collecting but only for this position its fine
    db.straight(-135)
    ferris_wheel_turn_and_down("BLACK")
    db.straight(135)

    db.turn(90)
    db.straight(155)
    db.turn(-90)

    db.straight(-150)

    ferris_wheel_up_and_turn("fk u min sen")

    db.curve(2 / 3 * 60, -90)
    db.curve(2 / 3 * 60, 90)

    # INSERT LINE TRACK TO MIDDLE
    db.settings(straight_acceleration=300, turn_acceleration=500)
    linetrack_to_corner("left", -90, min_distance=100)
    linetrack_to_corner("left", 0, min_distance=500)

    # SWEEPING THE RED CUBE

    db.straight(0)
    db.straight(-30, Stop.NONE)
    db.curve(-150, -90, Stop.NONE)
    db.curve(-152, -90, Stop.NONE)
    db.straight(-300, Stop.NONE)
    db.curve(-95, -90, Stop.NONE)
    db.straight(-40, Stop.NONE)
    db.curve(-160, 90, Stop.NONE)
    db.straight(-125, Stop.NONE)

    # DRIVE BACK TO INTERSECTION

    # db.turn(-90)

    # db.straight(50)
    # db.turn(-90)
    # db.straight(50)
    # db.turn(90)

    linetrack_to_corner("right", 0, min_distance=100)

    if slot_colors[4] == "NONE" and slot_colors[5] == "NONE":
        db.straight(25)
        db.turn(90)
    elif slot_colors[4] != "NONE":
        if slot_colors[5] != "NONE":
            db.turn(90)
            linetrack_to_corner("double", -90, min_distance=100)
            linetrack_to_corner("double", 0, min_distance=30)
            db.curve(100, -25)
            db.curve(100, 25)
            db.straight(200)
            deposit(5)
            db.curve(100, -180)
            db.straight(20)
            db.turn(90)
            db.straight(50)
            deposit(4)
            db.straight(20)
            db.turn(-90)
        else:
            db.straight(250)
            db.curve(78, 90)
            db.turn(180)
            db.straight(-20)
            deposit(4)
            db.straight(80)
            db.turn(-90)
        linetrack_to_corner("left", -90, min_distance=50)
    else:
        db.turn(90)
        linetrack_to_corner("double", -90, min_distance=100)
        linetrack_to_corner("double", 0, min_distance=30)
        db.curve(100, -25)
        db.curve(100, 25)
        db.straight(200)
        deposit(5)
        db.curve(100, -180)
        db.straight(20)
        db.turn(90)
        db.straight(90)
        db.turn(-90)
        linetrack_to_corner("left", -90, min_distance=50)
    if slot_colors[6] == "NONE" and slot_colors[7] == "NONE":
        pass
    elif slot_colors[6] != "NONE":
        if slot_colors[7] != "NONE":
            linetrack_to_corner("double", 90, min_distance=100)
            linetrack_to_corner("double", 0, min_distance=30)
            db.curve(100, 25)
            db.curve(100, -25)
            db.straight(200)
            deposit(7)
            db.curve(100, 180)
            db.straight(20)
            db.turn(-90)
            db.straight(80)
            deposit(6)
            db.straight(20)
            db.turn(-90)
        else:
            db.turn(90)
            db.straight(250)
            db.curve(78, -90)
            db.turn(180)
            db.straight(-20)
            deposit(6)
            db.straight(80)
            db.turn(-90)
    else:
        linetrack_to_corner("double", 90, min_distance=100)
        linetrack_to_corner("double", 0, min_distance=50)
        db.curve(100, 25)
        db.curve(100, -25)
        db.straight(200)
        deposit(7)
        db.curve(100, 180)
        db.straight(20)
        db.turn(-90)
        db.straight(70)
        db.turn(-90)
    ferris_wheel_up_and_turn("o")

    # START DEPOSITION

    # db.straight(250)
    # db.curve(78, 90)
    # db.turn(180)
    # db.straight(-30)

    # SLOT ONE
    # if slot_colors[4] != "NONE":
    #     ferris_wheel_turn_and_down(slot_colors[4])
    #     db.straight(30)
    #     ferris_wheel_up_and_turn("fk u min sen")
    # else:
    #     db.straight(30)

    # SLOT TWO
    # db.straight(-40)
    # db.curve(45, 92)
    # db.straight(70)

    # if slot_colors[5] != "NONE":
    #     ferris_wheel_turn_and_down(slot_colors[5])
    #     db.straight(30)
    #     ferris_wheel_up_and_turn("fk u min sen")
    # else:
    #     db.straight(30)

    # db.straight(425)

    # if slot_colors[6] != "NONE":
    #     ferris_wheel_turn_and_down(slot_colors[6])
    #     db.straight(30)
    #     ferris_wheel_up_and_turn("fk u min sen")
    # else:
    #     db.straight(30)

    # db.curve(90, 180)
    # db.straight(110)
    # db.turn(-90)

    # db.straight(70)

    # if slot_colors[7] != "NONE":
    #     ferris_wheel_turn_and_down(slot_colors[7])
    #     db.straight(30)
    #     ferris_wheel_up_and_turn("fk u min sen")
    # else:
    #     db.straight(30)

    # db.curve(150, 90)

    # linetrack_to_corner("left", 0, min_distance=150, backwards=True)
    # db.curve(-90, -90)
    # linetrack_to_corner("left", 0, min_distance=150, backwards=True)

    # db.straight(-250)

    # RED OBJECT IS NOW DEPOSITED

    # db.curve(200, -90)

    # linetrack_to_corner("left", 0, min_distance=100)

    # ferris_wheel_up_and_turn("o")

    # START DEPOSITION

    # db.straight(-80)
    # db.curve(78, 90)
    # db.straight(150)

    # SLOT ONE
    # if slot_colors[0] != "NONE":
    #     ferris_wheel_turn_and_down(slot_colors[0])
    #     db.straight(30)
    #     #ferris_wheel_default("fk u min sen")
    # else:
    #     db.straight(30)

    # SLOT TWO
    # db.curve(40, 92)
    # db.straight(70)

    # if slot_colors[1] != "NONE":
    #     print("AT 0")
    #     ferris_wheel_turn_and_down(slot_colors[1])
    #     print("AT 1")
    #     db.straight(30)
    #     print("AT 2")
    #     ferris_wheel_up_and_turn("fk u min sen")
    #     print("AT 3")
    # else:
    #     db.straight(30)

    # db.straight(425)

    # if slot_colors[2] != "NONE":
    #     ferris_wheel_turn_and_down(slot_colors[2])
    #     db.straight(30)
    #     ferris_wheel_up_and_turn("fk u min sen")
    # else:
    #     db.straight(30)

    # db.curve(90, 180)
    # db.straight(110)
    # db.turn(-90)

    # db.straight(70)

    # if slot_colors[3] != "NONE":
    #     ferris_wheel_turn_and_down(slot_colors[3])
    #     db.straight(30)
    #     ferris_wheel_up_and_turn("fk u min sen")
    # else:
    #     db.straight(30)

    # db.curve(150, 90)

    # db.straight(500)

    # RED SSSS (sibilant aliteration)
    # BACK TO INTERSECTION

    # linetrack_to_corner("left", -90)
    # linetrack_to_corner("left", -90)
