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
ferris_motor = Motor(Port.B, Direction.COUNTERCLOCKWISE)

db = GyroDriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=160)
db.settings(
    straight_speed=200, straight_acceleration=300, turn_rate=200, turn_acceleration=400
)
left_sensor.detectable_colors([Color.RED, Color.NONE, Color.WHITE])
right_sensor.detectable_colors([Color.NONE, Color.WHITE, Color.BLACK])

WHITE = 93
BLACK = 9

slot_colors: list[str] = []
prevent_right = True
# ferris_angle: int


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
    for i in range(9):
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


def straight_to_double() -> None:
    hit = False
    while not hit:
        l_ref = (left_sensor.reflection() - BLACK) / (WHITE - BLACK)
        r_ref = (right_sensor.reflection() - BLACK) / (WHITE - BLACK)
        hit = l_ref < 0.2 and r_ref < 0.2 #might need to make this more lenient
        db.straight(1, Stop.NONE)

    db.straight(0, Stop.HOLD)
    return


def to_angle(angle: int) -> None:
    print(main_motor.angle(), "to", angle)
    if angle >= -20 or angle < -285:
        print("you screwed up")
        return

    if main_motor.angle() > angle:
        while main_motor.angle() > angle:
            main_motor.run(-600)
    elif main_motor.angle() < angle:
        while main_motor.angle() < angle:
            main_motor.run(600)


def linetrack_by_distance(distance, sensitivity=1.7):
    """Linetrack a specific distance.

    Args:
        distance (int): Distance in mm
    """
    db.reset()
    while db.distance() < distance:
        diff = left_sensor.reflection() - right_sensor.reflection()
        db.drive(
            150 - abs(diff),
            diff * sensitivity
        )
    db.stop()


def linetrack_to_corner(
    turn_direction: str,
    turn_angle: int = 0,
    min_distance: int = 30,
    speed: int = 350,
) -> None:

    """Linetrack to a corner, then do a smooth turn.

    Args:
        turn_direction (str): The direction of the turn - left, double or right
        turn_angle (int, optional): The angle of the turn. Defaults to 0.
        min_distance (int, optional): The minimum distance driven before turning. Defaults to 30.
        speed (int, optional): Speed of robot. Defaults to 350.
    """
    db.reset()
    while True:
        l_ref = (left_sensor.reflection() - BLACK) / (WHITE - BLACK)
        r_ref = (right_sensor.reflection() - BLACK) / (WHITE - BLACK)
        db.drive(350, (l_ref - r_ref) * 53) #og 60
        l_ref = (left_sensor.reflection() - BLACK) / (WHITE - BLACK)
        r_ref = (right_sensor.reflection() - BLACK) / (WHITE - BLACK)

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


def turn_to_line(turn_direction: str) -> None:
    db.straight(90)

    if turn_direction == "left":
        db.turn(-90)
    elif turn_direction == "right":
        db.turn(90)
    elif turn_direction == "left_back":
        db.turn(-180)
    else:  # turn_direction == "right_back"
        db.turn(180)


def ferris_wheel_turn(desired_cart: str, wait=True) -> None:
    """Turn the ferris wheel without lowering.

    Args:
        desired_cart (str): Desired cart
    """
    ferris_angle = ferris_motor.angle()

    print("FERRIS: ", ferris_angle)
    if desired_cart == "WHITE":
        desired_ferris_angle = 450
    elif desired_cart == "BLACK":
        desired_ferris_angle = 1350
    elif desired_cart == "DEF":
        desired_ferris_angle = 1000
    elif desired_cart == "EXTRAHAND":
        desired_ferris_angle = 900
    elif desired_cart == "SUS":
        desired_ferris_angle = 1700
    elif desired_cart == "RED":
        desired_ferris_angle = 800
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
    if abs(ferris_angle_diff) == 900:
        if (prevent_right == True and ferris_angle == 1350) or (
            prevent_right == False and ferris_angle == 450
        ):
            ferris_angle_diff = abs(ferris_angle_diff) * -1
        else:
            ferris_angle_diff = abs(ferris_angle_diff)

    print("DESIRED FERRIS ANGLE:", desired_ferris_angle)
    print("FERRIS ANGLE DIFF:", ferris_angle_diff)
    ferris_motor.run_angle(1000, ferris_angle_diff, wait=wait)
    ferris_angle = desired_ferris_angle


def ferris_wheel_turn_and_down(desired_cart: str, wait=True) -> None:
    """Turn the ferris wheel and lower the mechanism.

    Args:
        desired_cart (str): Desired cart
    """
    if desired_cart == "EXTRAHAND":
        print("you screwed up")
        return
    print("down and turn")
    ferris_wheel_turn(desired_cart, wait=wait)
    if main_motor.angle() < -30:
        main_motor.run_angle(600, 285)


def ferris_wheel_up_and_turn(desired_cart: str, wait=True) -> None:
    """Raise the mechanism and turn the ferris wheel.

    Args:
        desired_cart (str): Desired cart
    """
    print("up and turn")
    ferris_wheel_turn(desired_cart, wait=wait)
    if main_motor.angle() > -250:
        main_motor.run_angle(600, -285)
    
    print("DONE with up and turn")


def align_to_line():
    db.straight(-150)
    linetrack_by_distance(60, 2.2)
    db.straight(90)


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
        if slot_distances[slot_no][0] + 15 < distance < slot_distances[slot_no][1] - 15:
            current_tally += reflection
            reading_count += 1
        if distance >= slot_distances[slot_no + 1][0] + 15:
            # print(current_tally, reading_count)
            slot_averages[slot_no] = current_tally / reading_count
            current_tally = 0
            reading_count = 0
            slot_no += 1
            if slot_no == 8:
                print(distance)
                db.straight(0)
                break
        db.drive(200, 0)

    south_numed = [[val, i] for i, val in enumerate(slot_averages[:4])]
    north_numed = [[val, i] for i, val in enumerate(slot_averages[4:])]
    south_sorted = sorted(south_numed, reverse=True)
    north_sorted = sorted(north_numed, reverse=True)

    colors = ["NONE"] * 8
    colors[south_sorted[0][1]] = "WHITE"
    colors[south_sorted[1][1]] = "BLACK"
    colors[north_sorted[0][1] + 4] = "WHITE"
    colors[north_sorted[1][1] + 4] = "BLACK"
    print(slot_averages)
    try:
        hub.system.storage(0, write = slot_averages + slot_colors)
    except Exception:
        pass
    return colors


def deposit(slot: int) -> None:
    """Deposit an object.

    Args:
        slot (int): The slot number.
    """

    if slot == 2 or slot == 6:
        prevent_right = True
    else:
        prevent_right = False

    if slot_colors[slot] == "NONE":
        print("deposit('NONE') doesn't really make sense")
    if slot_colors[slot] == "WHITE" or slot_colors[slot] == "BLACK":
        ferris_wheel_turn_and_down(slot_colors[slot])
        if slot_colors[slot] == "BLACK":
            if slot < 4:
                db.straight(28)
            else:
                db.straight(43)

            if slot == 0 or slot == 3 or slot == 4 or slot == 7:
                ferris_wheel_turn("DEF")

            else:
                main_motor.run_angle(600, -285)

            if slot < 4:
                db.straight(-28)
            else:
                db.straight(-43)

        elif slot_colors[slot] == "WHITE":
            if slot < 4:
                db.straight(20)
            else:
                db.straight(36)
            if slot == 0 or slot == 3 or slot == 4 or slot == 7:
                ferris_wheel_turn("DEF")

            else:
                main_motor.run_angle(600, -285)

            if slot < 4:
                db.straight(-20)
            else:
                db.straight(-34)

    prevent_right = False


def go_to_slot(region: int, slot: int) -> None:
    if slot == 0:
        db.settings(straight_speed=450, straight_acceleration=500)
        db.straight(350)
        db.settings(straight_speed=200, straight_acceleration=300)
        initial_heading = hub.imu.heading()
        target_heading = initial_heading - 90
        print(initial_heading, target_heading)
        while 3 < abs(target_heading - hub.imu.heading()):
            left_motor.run(-500)
        db.straight(0, Stop.HOLD)

        db.settings(straight_speed=450, straight_acceleration=500)
        if region == 1:
            db.straight(45)

        deposit(region * 4 + slot)

        db.straight(-38)
        ferris_wheel_up_and_turn("EXTRAHAND")

        if region == 0:
            db.straight(32)

        initial_heading = hub.imu.heading()
        target_heading = initial_heading - 90
        print(initial_heading, target_heading)
        while 3 < abs(target_heading - hub.imu.heading()):
            right_motor.run(500)
        db.straight(0, Stop.HOLD)
        db.straight(-35)
        db.settings(straight_speed=200, straight_acceleration=300)
        linetrack_to_corner("left", 0, min_distance=10)
        turn_to_line("right_back")

    elif slot == 1:

        db.turn(90)
        linetrack_to_corner("double", 0, min_distance=45 - (region * 35))  # 10
        ferris_wheel_up_and_turn(slot_colors[region * 4 + 1])
        turn_to_line("right_back")
        db.settings(straight_speed=450, straight_acceleration=500)
        db.straight(-92)
        db.turn(-75)
        db.straight(-143)
        deposit(region * 4 + 1)
        db.straight(143)
        db.turn(75)
        db.settings(straight_speed=200, straight_acceleration=300)
        if region == 1:
            linetrack_to_corner("double", 0, min_distance=65 - (region * 35))  # 30
        else:
            linetrack_to_corner("right", 0, min_distance=65 - (region * 35))  # 30

        turn_to_line("right")

    elif slot == 2:

        db.turn(90)
        linetrack_to_corner("double", 0, min_distance=45 - (region * 35))  # 10
        ferris_wheel_up_and_turn(slot_colors[region * 4 + 2])
        turn_to_line("right_back")
        db.settings(straight_speed=450, straight_acceleration=500)
        db.straight(-92)
        
        db.turn(75)
        db.straight(-147)
        deposit(region * 4 + 2)
        db.straight(147)  # (110+3 +- 2 basic math)
        db.turn(-75)
        db.settings(straight_speed=200, straight_acceleration=300)
        if region == 1:
            linetrack_to_corner("double", 0, min_distance=65 - (region * 35))  # 30
        else:
            linetrack_to_corner("right", 0, min_distance=65 - (region * 35))  # 30
        turn_to_line("right")

    elif slot == 3:
        db.settings(straight_speed=450, straight_acceleration=500)
        db.straight(-175)
        initial_heading = hub.imu.heading()
        target_heading = initial_heading - 90
        print(initial_heading, target_heading)
        while 3 < abs(target_heading - hub.imu.heading()):
            left_motor.run(-500)
        db.straight(0, Stop.HOLD)
        if region == 1:
            db.straight(45)
        deposit(region * 4 + slot)
        db.straight(-38)
        ferris_wheel_up_and_turn("EXTRAHAND")
        if region == 0:
            db.straight(47)

        initial_heading = hub.imu.heading()
        target_heading = initial_heading + 90
        print(initial_heading, target_heading)
        while 3 < abs(target_heading - hub.imu.heading()):
            left_motor.run(500)
        db.straight(0, Stop.HOLD)
        db.straight(-35)

        if region == 1:
            db.settings(straight_speed=200, straight_acceleration=300)
            linetrack_to_corner("right", 0, min_distance=5)
        else:
            db.settings(straight_speed=450)
            db.straight(82)

        db.settings(straight_speed=200, straight_acceleration=300)
        db.straight(90)


def regional_deposit(dist=0, region=0) -> None:
    if slot_colors[1] != "NONE" and slot_colors[2] != "NONE" and region == 0:
        db.turn(90)
        linetrack_to_corner("double", 0, min_distance=45 - (region * 35))  # 10
        ferris_wheel_up_and_turn(slot_colors[region * 4 + 1])
        turn_to_line("right_back")
        db.straight(-90)
        db.turn(-75)
        db.straight(-143)
        deposit(region * 4 + 1)
        db.straight(143)  
        db.turn(75)
        ferris_wheel_up_and_turn(slot_colors[region * 4 + 2])
        db.turn(77)
        db.straight(-150)
        deposit(region * 4 + 2)
        db.straight(150) 
        db.turn(-77)
        if region == 1:
            linetrack_to_corner("double", 0, min_distance=65 - (region * 35))  # 30
        else:
            linetrack_to_corner("right", 0, min_distance=65 - (region * 35))  # 30
        turn_to_line("right")

    else:
        if region == 1:
            if slot_colors[region * 4 + 1] == "BLACK":
                go_to_slot(region, 1)
            
            elif slot_colors[region * 4 + 2] == "BLACK":
                go_to_slot(region, 2)
            
            elif slot_colors[region * 4 + 3] == "BLACK":
                go_to_slot(region, 3)
                
            elif slot_colors[region * 4 + 0] == "BLACK":
                go_to_slot(region, 0)
        else:
            if slot_colors[region * 4 + 1] != "NONE":
                go_to_slot(region, 1)

            if slot_colors[region * 4 + 2] != "NONE":
                go_to_slot(region, 2)
                
            if slot_colors[region * 4 + 3] != "NONE":
                go_to_slot(region, 3)
                
            if slot_colors[region * 4 + 0] != "NONE":
                go_to_slot(region, 0)
            

if __name__ == "__main__":
    db.settings(straight_speed=450, straight_acceleration=500, turn_acceleration=500)
    distances = gen_slot_distances(5, 32, 17, 94)
    main_motor.reset_angle(0)

    db.curve(2 / 3 * 162, 48)
    db.curve(2 / 3 * 162, -48)
    # slot_colors = ["NONE", "WHITE", "BLACK", "NONE", "NONE", "WHITE", "NONE", "BLACK"]
    slot_colors = voting(distances)
    print(slot_colors)
    ferris_motor.run_angle(1000, 210, wait=False)
    main_motor.run_angle(600, -310)

    ferris_motor.reset_angle(0)
    db.turn(170)
    #db.curve()

    linetrack_by_distance(220)
    db.straight(-65)
    ferris_wheel_turn_and_down("WHITE", wait=False)
    db.turn(12)
    db.straight(-360)
    ferris_wheel_turn_and_down("BLACK")
    db.straight(360)
    db.turn(-20)
    db.straight(-360)

    prevent_right = False
    main_motor.run_angle(600, -285)
    ferris_wheel_up_and_turn("BLACK", wait=False)
    db.settings(straight_acceleration=300, turn_acceleration=300)
    db.curve(2 / 3 * 45, -82)
    db.curve(2 / 3 * 45, 90)

    # Line track to red disposal area
    db.settings(turn_acceleration=500)
    linetrack_to_corner("left", -90, min_distance=25)
    db.settings(straight_speed=450, straight_acceleration=500, turn_acceleration=500)
    linetrack_to_corner("right", 90, min_distance=500)
    db.settings(straight_speed=250, straight_acceleration=200, turn_acceleration=400)
    linetrack_to_corner("right", 0, min_distance=90)
    
    # Dispose red
    db.settings(straight_acceleration=300, turn_acceleration=300)
    db.straight(330)
    db.turn(160)
    ferris_wheel_turn_and_down("WHITE")
    db.straight(20)
    ferris_wheel_up_and_turn("DEF")
    db.straight(-20)
    
    db.turn(20)
    
    db.straight(240)
    ferris_wheel_turn("BLACK", wait=False)
    db.turn(-90)

    # Line track to South Port
    db.settings(straight_speed=300, straight_acceleration=200)
    linetrack_to_corner("left", 0, min_distance=100)
    turn_to_line("right_turn")

    db.settings(straight_speed=200, straight_acceleration=300, turn_rate = 200, turn_acceleration=200)
    regional_deposit(0, 0) # Deposit South
    db.settings(straight_speed=450, straight_acceleration=500, turn_rate = 500, turn_acceleration=500)
    db.straight(60)

    ferris_wheel_turn_and_down("BLACK")
    db.straight(-30)
    ferris_wheel_up_and_turn("EXTRAHAND", wait=False)

    # Head to North solar panel
    # Driving to solar farm 1 (right)
    linetrack_to_corner("left", -90, min_distance=50)
    linetrack_to_corner("left", 0, min_distance=5)
    ferris_wheel_turn("DEF", wait=False)
    straight_to_double() #drives until double black NOTE NEEDS TO BE MADE LENIENT
    db.straight(-15)

    # Deploy the North solar panel
    initial_heading = hub.imu.heading()
    target_heading = initial_heading - 90
    print(initial_heading, target_heading)
    main_motor.run_angle(600, 285, wait=False)
    while 3 < abs(target_heading - hub.imu.heading()):
        right_motor.run(500)
    db.straight(0, Stop.HOLD)
    
    db.straight(-30)
    ferris_wheel_turn("BLACK") #Knocks the solar panel
    main_motor.run_angle(600, -285) #returns to normal

    initial_heading = hub.imu.heading() #arc to line
    target_heading = initial_heading - 90
    print(initial_heading, target_heading)
    while 3 < abs(target_heading - hub.imu.heading()):
        right_motor.run(500)
    db.straight(0, Stop.HOLD)
    db.straight(70)

    initial_heading = hub.imu.heading() #arc back on line ready for line tracking
    target_heading = initial_heading + 90
    print(initial_heading, target_heading)
    while 3 < abs(target_heading - hub.imu.heading()):
        left_motor.run(500)
    db.straight(0, Stop.HOLD)

    # North Port
    linetrack_to_corner("right", 0, min_distance=400)
    db.straight(90)
    regional_deposit(0, 1)

    db.settings(straight_speed=450, straight_acceleration=500, turn_acceleration=400)
    linetrack_to_corner("double", -90, min_distance=200)
    
    db.straight(380)
    initial_heading = hub.imu.heading
    target_heading = hub.imu.heading() + 90
    print(initial_heading, target_heading)
    while 3 < abs(target_heading - hub.imu.heading()):
        right_motor.run(500)
    db.straight(0, Stop.HOLD)
    ferris_wheel_turn("SUS")
    db.straight(-420)


 
    ferris_wheel_turn("BLACK")
    main_motor.run_angle(600, -285)

    db.straight(300)  # Back to start
