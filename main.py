from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import GyroDriveBase
from pybricks.tools import wait, StopWatch

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
stopwatch = StopWatch()

WHITE = 100
BLACK = 0

# Slot Width = 32 mm
# Slot Interval = 17 mm
# Mid Gap = 94 mm

slot_angles = []
slot_colours = []
slot_votes = [0] * 8

ferris_angle = 0


def gen_slot_angle_list(begin, slot_width, slot_interval, mid_gap):
    for i in range(9):
        if i < 4:
            slot_angles.append(
                [
                    begin + (i * slot_interval) + (i * slot_width),
                    begin + (i * slot_interval) + (i * slot_width) + slot_width,
                ]
            )
        else:
            slot_angles.append(
                [
                    begin + ((i - 1) * slot_interval) + (i * slot_width) + mid_gap,
                    begin
                    + ((i - 1) * slot_interval)
                    + (i * slot_width)
                    + mid_gap
                    + slot_width,
                ]
            )
    print(slot_angles)


def detect():
    count = 0
    while True:
        db.straight(1, Stop.NONE)
        print(ultra_sensor.distance(), count)
        count += 1


def sus():
    count = 0
    while True:
        # db.drive(200, 0)
        side_col_1 = right_sensor.color()
        side_col_2 = left_sensor.color()
        print(ultra_sensor.distance(), side_col_1, side_col_2, count)
        count += 1


def linetracingtodistance(distance_covered):
    db.reset()
    # DEBUG = 0
    while db.distance() < distance_covered:
        # DEBUG += 1
        # print(DEBUG)
        left_sensor_reflect = left_sensor.reflection()
        right_sensor_reflect = right_sensor.reflection()
        current_sensor_diff = left_sensor_reflect - right_sensor_reflect
        total_sensor_value = left_sensor_reflect + right_sensor_reflect
        db.drive(100, current_sensor_diff * 2.2)
    db.straight(0)


def linetracing(past_junctions=1, racing_line_turn=0):
    junctions_covered = 0
    while junctions_covered < past_junctions:
        left_sensor_reflect = left_sensor.reflection()
        right_sensor_reflect = right_sensor.reflection()
        current_sensor_diff = left_sensor_reflect - right_sensor_reflect
        total_sensor_value = left_sensor_reflect + right_sensor_reflect
        if total_sensor_value > 40:
            db.drive(400, current_sensor_diff * 1.5)
        else:
            print(total_sensor_value)
            junctions_covered += 1
    db.straight(0, Stop.BRAKE)  # include this for braking
    initial_heading = hub.imu.heading()
    print(initial_heading)
    target_heading = initial_heading + racing_line_turn
    print(target_heading)
    if racing_line_turn <= 0:
        while 3 < abs(target_heading - hub.imu.heading()):
            right_motor.run(500)
    else:
        while 3 < abs(target_heading - hub.imu.heading()):
            left_motor.run(500)
    db.straight(0, Stop.BRAKE)  # include this for braking


def linetracingtocorner(corner_heading, racing_line_turn=0, leeway=30):
    left_sensor_reflect = left_sensor.reflection()
    right_sensor_reflect = right_sensor.reflection()
    current_sensor_diff = left_sensor_reflect - right_sensor_reflect
    total_sensor_value = left_sensor_reflect + right_sensor_reflect
    db.reset()
    while db.distance() < leeway:
        left_sensor_reflect = left_sensor.reflection()
        right_sensor_reflect = right_sensor.reflection()
        current_sensor_diff = left_sensor_reflect - right_sensor_reflect
        total_sensor_value = left_sensor_reflect + right_sensor_reflect
        db.drive(220, current_sensor_diff * 1.2)
    
    if corner_heading == 0:
        while total_sensor_value > 0.15:
            left_sensor_reflect = left_sensor.reflection()
            right_sensor_reflect = right_sensor.reflection()
            current_sensor_diff = left_sensor_reflect - right_sensor_reflect
            total_sensor_value = left_sensor_reflect + right_sensor_reflect
            db.drive(300, current_sensor_diff * 1.35)
    else:
        while current_sensor_diff / corner_heading < 75:
            left_sensor_reflect = left_sensor.reflection()
            right_sensor_reflect = right_sensor.reflection()
            current_sensor_diff = left_sensor_reflect - right_sensor_reflect
            total_sensor_value = left_sensor_reflect + right_sensor_reflect
            db.drive(300, current_sensor_diff * 1.35)

    db.straight(0, Stop.BRAKE)  # include this for motor setting Stop.brake
    initial_heading = hub.imu.heading()
    print(initial_heading)
    target_heading = initial_heading + racing_line_turn
    print(target_heading)
    if racing_line_turn <= 0:
        while 3 < abs(target_heading - hub.imu.heading()):
            right_motor.run(500)
    else:
        while 3 < abs(target_heading - hub.imu.heading()):
            left_motor.run(500)
    db.straight(0, Stop.BRAKE)  # include this for braking


def ferris_wheel_turn(desired_cart):  # *** ONLY turns ferris wheel, does not lower
    ferris_angle = mech_motor.angle()
    print("FERRIS: ", ferris_angle)
    if desired_cart == "NONE":
        desired_ferris_angle = 0
    elif desired_cart == "WHITE":
        desired_ferris_angle = 450
    elif desired_cart == "BLACK":
        desired_ferris_angle = 900
    else:
        desired_ferris_angle = 1350
    ferris_angle_diff = desired_ferris_angle - ferris_angle
    if abs(ferris_angle_diff) > 900:
        ferris_angle_diff = 1800 - abs(ferris_angle_diff)
    mech_motor.run_angle(350, ferris_angle_diff)


def ferris_wheel_placement(desired_cart):  # turns ferris wheel plus lowers mechanism
    ferris_wheel_turn(desired_cart)
    print("MAIN MOTOR: ", main_motor.angle())
    while (
        main_motor.angle() < 0
    ):  # "default" 0 is when sensor is at side and mechanism motor is lowered
        main_motor.run(360)


def ferris_wheel_default(desired_cart):
    print(main_motor.angle())
    while main_motor.angle() > -290:
        main_motor.run(-360)
    ferris_wheel_turn(desired_cart)


def ferris_wheel_up():
    while main_motor.angle() > -290:
        main_motor.run(-360)


def test_voting():  # during voting, LEFT sensor will detect the cubes
    slot_no = -1
    onSlot = False
    while True:
        # print(left_sensor.reflection())
        if left_sensor.color() != Color.NONE:
            if onSlot == False:
                onSlot = True
                slot_no += 1
                print(slot_no, " ON: ", db.distance())
        else:
            if onSlot == True:
                onSlot = False
                print(" | OFF: ", db.distance())
        db.drive(100, 0)


def voting():
    slot_no = -1
    onSlot = False
    slot_readings = 0
    slot_low = 100
    slot_high = -1
    first_group_white = 0
    first_group_black = -1
    second_group_white = 4
    second_group_black = -1

    while True:
        if db.distance() - 10 >= slot_angles[slot_no + 1][0] and onSlot == False:
            onSlot = True
            slot_no += 1
        elif db.distance() + 9 >= slot_angles[slot_no][1] and onSlot == True:
            onSlot = False
            slot_avg = slot_votes[slot_no] / slot_readings
            if slot_avg < 5:
                print(slot_no, ":", "NONE", slot_avg, "HIGHEST:", slot_high)
            elif slot_avg < 18:
                print(slot_no, ":", "BLACK", slot_avg, "LOWEST:", slot_low)
            else:
                print(slot_no, ":", "WHITE", slot_avg)
            slot_readings = 0
            slot_low = 100
            slot_high = -1
            if slot_no == 7:
                break
            # db.stop()
            # wait(1000)

        if onSlot == True:
            slot_votes[slot_no] += left_sensor.reflection()
            slot_readings += 1
            if left_sensor.reflection() < slot_low:
                slot_low = left_sensor.reflection()

            if left_sensor.reflection() > slot_high:
                slot_high = left_sensor.reflection()
            # hub.speaker.beep(440, 10)
        db.drive(150, 0)

    for i in range(8):
        if i < 4:
            if slot_votes[i] > slot_votes[first_group_white]:
                first_group_black = first_group_white
                first_group_white = i
            elif (
                first_group_black == -1 or slot_votes[i] > slot_votes[first_group_black]
            ):
                first_group_black = i
        else:
            if slot_votes[i] > slot_votes[second_group_white]:
                second_group_black = second_group_white
                second_group_white = i
            elif (
                second_group_black == -1
                or slot_votes[i] > slot_votes[second_group_black]
            ):
                second_group_black = i

    for i in range(8):
        if i == first_group_white or i == second_group_white:
            slot_colours.append("WHITE")
        elif i == first_group_black or i == second_group_black:
            slot_colours.append("BLACK")
        else:
            slot_colours.append("NONE")

    print("VOTING RESULTS:", slot_colours)
    db.straight(0)


if __name__ == "__main__":
    # mech_motor.run_target(100, 100)
    while True:
        mech_motor.run(-300)
    wait(300)
    main_motor.reset_angle(0)
    mech_motor.reset_angle(0)
    # mech_motor.run_target(100, -100)
    # mech_motor.run_target(360,0)
    #main_motor.run_angle(100,300)
    # mech_motor.run_angle(360, (mech_motor.angle() % 360) - FERRIS_ANGLES["red"])
    db.curve(2 / 3 * 162, 48)
    db.curve(2 / 3 * 162, -48)
    db.reset()  # necessary for voting

    gen_slot_angle_list(5, 32, 17, 94)

    # slot_colours = ["NONE", "WHITE", "BLACK", "NONE", "NONE", "NONE", "NONE", "NONE"]
    voting()

    main_motor.run_angle(200, -300)

    db.curve(-100, 25)
    db.curve(-100, -25)

    linetracingtodistance(80)
    # db.straight(105)
    # wait(3000)
    # linetracingtocorner(-1, 0)
    # ensure possible cube at last slot is not collected
    db.turn(180)
    # main_motor.run_angle(300,-290)
    # linetracingtodistance(180)
    main_motor.run_angle(300, 290)
    db.curve(2 / 3 * -70, -85)
    db.curve(2 / 3 * -70, 85)
    # db.straight(50)
    ferris_wheel_placement("WHITE")
    db.straight(-135)
    ferris_wheel_placement("BLACK")
    db.straight(135)

    db.turn(90)
    db.straight(165)
    db.turn(-90)

    db.straight(-150)

    ferris_wheel_up()
    db.curve(2 / 3 * 60, -90)
    db.curve(2 / 3 * 60, 90)
    ferris_wheel_default("fk u min sen")

    # INSERT LINE TRACK TO MIDDLE
    db.settings(straight_acceleration=300)
    linetracingtocorner(-1, -90, 10)
    linetracingtocorner(-1, 0, 500)

    # SWEEPING THE RED CUBE

    db.straight(-30)
    db.curve(-150, -180, Stop.HOLD)
    db.straight(-250)
    db.curve(-150, -90, Stop.HOLD)
    db.straight(-110)

    #DRIVE BACK TO INTERSECTION

    db.turn(-90)
    
    linetracingtocorner(1, 0, 10)


    ##START DEPOSITION

    
    db.straight(100)
    db.curve(78, 90)
    db.straight(150)

    # SLOT ONE
    if slot_colours[4] != "NONE":
        ferris_wheel_placement(slot_colours[4])
        db.straight(35)
        #ferris_wheel_default("fk u min sen")
    else:
        db.straight(35)

    # SLOT TWO
    db.curve(45, 92)
    db.straight(20)

    if slot_colours[5] != "NONE":
        print("AT 0")
        ferris_wheel_placement(slot_colours[5])
        print("AT 1")
        db.straight(35)
        print("AT 2")
        ferris_wheel_default("fk u min sen")
        print("AT 3")
    else:
        db.straight(35)

    db.straight(500)

    if slot_colours[6] != "NONE":
        ferris_wheel_placement(slot_colours[6])
        db.straight(35)
        ferris_wheel_default("fk u min sen")
    else:
        db.straight(35)

    db.curve(90, 180)
    db.straight(110)
    db.turn(-90)

    db.straight(70)

    if slot_colours[7] != "NONE":
        ferris_wheel_placement(slot_colours[7])
        db.straight(35)
        ferris_wheel_default("fk u min sen")
    else:
        db.straight(35)

    # RED SSSS
    # BACK TO INTERSECTION

    # linetracingtocorner(-1,-90)
    # linetracingtocorner(-1,-90)
