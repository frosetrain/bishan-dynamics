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
left_sensor.detectable_colors([Color.RED, Color.NONE, Color.YELLOW, Color.GREEN, Color.BLUE])
right_sensor.detectable_colors([Color.RED, Color.NONE, Color.WHITE, Color.BLACK, Color.YELLOW, Color.GREEN, Color.BLUE])

WHITE = 93
BLACK = 9

slot_colors: list[str] = []
slot_votes = [0] * 8
slot_angles = []
prevent_right = True

long_hand = ["NONE", "NONE"] # [outside, inside]
short_hand = ["NONE", "NONE"] # [outside, inside]

position = {
    "region": -1,
    "slot": -1
}

hands = {
    "LONG": ["NONE", "NONE"],
    "SHORT": ["NONE", "NONE"]
}



def gen_slot_distances(
    begin: int, slot_width: int, slot_interval: int, mid_gap: int
) -> list[list[int]]:
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
    return slot_angles


def gyro_turn(angle: int, motor: str):
    if motor == "LEFT":
        if angle > 0:
            initial_heading = hub.imu.heading
            target_heading = hub.imu.heading() + angle
            print(initial_heading, target_heading)
            while 3 < abs(target_heading - hub.imu.heading()):
                left_motor.run(500)
            db.straight(0, Stop.HOLD)
        
        else:
            initial_heading = hub.imu.heading
            target_heading = hub.imu.heading() + angle
            print(initial_heading, target_heading)
            while 3 < abs(target_heading - hub.imu.heading()):
                left_motor.run(-500)
            db.straight(0, Stop.HOLD)
    
    elif motor == "RIGHT":
        if angle > 0:
            initial_heading = hub.imu.heading
            target_heading = hub.imu.heading() + angle
            print(initial_heading, target_heading)
            while 3 < abs(target_heading - hub.imu.heading()):
                right_motor.run(-500)
            db.straight(0, Stop.HOLD)
        
        else:
            initial_heading = hub.imu.heading
            target_heading = hub.imu.heading() + angle
            print(initial_heading, target_heading)
            while 3 < abs(target_heading - hub.imu.heading()):
                right_motor.run(500)
            db.straight(0, Stop.HOLD)





def straight_to_double(reverse = False, brake=False) -> None:
    hit = False
    while not hit:
        l_ref = (left_sensor.reflection() - BLACK) / (WHITE - BLACK)
        r_ref = (right_sensor.reflection() - BLACK) / (WHITE - BLACK)
        hit = l_ref < 0.2 and r_ref < 0.2 #might need to make this more lenient
        if(reverse):
            db.straight(-1, Stop.NONE)
        else:
            db.straight(1, Stop.NONE)

    if(brake == True):
        db.straight(0, Stop.BRAKE)
    else:
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
    speed_multiplier: float = 1.0,
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
        db.drive((500-((l_ref - r_ref)*3))*speed_multiplier, (l_ref - r_ref) * 53) #og 60
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

def linetrack_to_color(color, speed_multiplier):
    while True:
        l_ref = (left_sensor.reflection() - BLACK) / (WHITE - BLACK)
        r_ref = (right_sensor.reflection() - BLACK) / (WHITE - BLACK)
        db.drive((500-((l_ref - r_ref)*3))*speed_multiplier, (l_ref - r_ref) * 53) #og 60
        l_ref = (left_sensor.reflection() - BLACK) / (WHITE - BLACK)
        r_ref = (right_sensor.reflection() - BLACK) / (WHITE - BLACK)

        if left_sensor.color() == color and right_sensor.color() == color:
            db.straight(0, Stop.HOLD)
            break


def turn_to_line(turn_direction: str) -> None:
    db.straight(95)

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
    if desired_cart == "LONG":
        desired_ferris_angle = 450
    elif desired_cart == "SHORT":
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
    if 880 < abs(ferris_angle_diff) < 920:
        if (prevent_right == True and 1330 < ferris_angle%1800 < 1370) or (prevent_right == False and 430 < ferris_angle%1800 < 470) or (prevent_right == True and -470 < ferris_angle%1800 < -430) or (prevent_right == False and -1370 < ferris_angle%1800 < -1330):
            ferris_angle_diff = abs(ferris_angle_diff) * -1
        else:
            ferris_angle_diff = abs(ferris_angle_diff)

    print("DESIRED FERRIS ANGLE:", desired_ferris_angle)
    print("FERRIS ANGLE DIFF:", ferris_angle_diff)
    ferris_motor.run_angle(1000, ferris_angle_diff)
    ferris_angle = desired_ferris_angle


def ferris_wheel_turn_and_down(desired_cart: str) -> None:
    """Turn the ferris wheel and lower the mechanism.

    Args:
        desired_cart (str): Desired cart
    """

    print("down and turn")
    ferris_wheel_turn(desired_cart)
    if main_motor.angle() < -50:
         main_motor.run_angle(600, 285)






def ferris_wheel_up_and_turn(desired_cart: str) -> None:
    """Raise the mechanism and turn the ferris wheel.

    Args:
        desired_cart (str): Desired cart
    """
    print("up and turn")
    while main_motor.angle() > -290:
        main_motor.run(-500)
    ferris_wheel_turn(desired_cart, wait=wait)
    print(main_motor.control.stall_tolerances())

    

    
    print("DONE with up and turn")


def align_to_line():
    db.straight(-150)
    linetrack_by_distance(60, 2.2)
    db.straight(90)


def voting(slot_distances: list[list[int]]) -> list[str]:
    slot_no = -1
    onSlot = False
    slot_readings = 0
    slot_low = 100
    slot_high = -1
    color_hp = 3
    scanning_white = True
    is_confirmed = False
    result_color = "NONE"

    right_sensor.lights.off()
    while True:
        #print(slot_no)
        if db.distance() - 7 >= slot_distances[slot_no + 1][0] and onSlot == False:
            onSlot = True
            is_confirmed = False
            scanning_white = True
            slot_no += 1
            #hub.speaker.beep(440)

        elif db.distance() + 10 >= slot_distances[slot_no][1] and onSlot == True:
            onSlot = False
            if is_confirmed == False:
                slot_avg = slot_votes[slot_no] / slot_readings
                if slot_avg < 1:
                    #print(slot_no, ":", "BLACK", slot_avg)
                    slot_colors.append("BLACK")
                
                else:
                    #print(slot_no, ":", "NONE", slot_avg)
                    slot_colors.append("NONE")
            else:
                slot_colors.append(result_color)
                #print(slot_no, ":", result_color)
            
            slot_readings = 0
            slot_low = 100
            slot_high = -1
            color_hp = 2
            #hub.speaker.beep(532)
            is_confirmed = False
            result_color = "NONE"
            if slot_no == 7:
                print(slot_colors)
                break


        if onSlot == True and is_confirmed == False:
            
            #print(left_sensor.reflection())
            if left_sensor.color() != Color.NONE and left_sensor.color() != Color.WHITE and left_sensor.color() != Color.BLACK and is_confirmed == False:
                color_hp -= 1
                if color_hp == 0:
                    result_color = str(left_sensor.color())[6:]
                    is_confirmed = True
            
            elif left_sensor.reflection() > 70:
                
                result_color = "WHITE"
                is_confirmed = True
                scanning_white = False
                    

            else:
                #print("SUSSY BAKA")
                left_sensor.lights.off()
                slot_votes[slot_no] += left_sensor.ambient()
                slot_readings += 1
                if left_sensor.ambient() < slot_low:
                    slot_low = left_sensor.ambient()
                    

                if left_sensor.ambient() > slot_high:
                    slot_high = left_sensor.ambient()
                
                left_sensor.lights.on()
            # hub.speaker.beep(440, 10)
        
        #print(db.distance())
        db.drive(100, 0)

def mono_voting(slot_distances: list[list[int]]) -> list[str]:
    slot_no = -1
    onSlot = False
    slot_readings = 0
    slot_low = 100
    slot_high = -1
    is_confirmed = False
    result_color = "NONE"

    right_sensor.lights.off()
    while True:
        #print(slot_no)
        if db.distance() - 7 >= slot_distances[slot_no + 1][0] and onSlot == False:
            onSlot = True
            is_confirmed = False
            scanning_white = True
            slot_no += 1
            #hub.speaker.beep(440)

        elif db.distance() + 10 >= slot_distances[slot_no][1] and onSlot == True:
            onSlot = False
            if is_confirmed == False:
                slot_avg = slot_votes[slot_no] / slot_readings
                if slot_avg < 1:
                    print(slot_no, ":", "BLACK", slot_avg)
                    slot_colors.append("BLACK")
                
                else:
                    print(slot_no, ":", "NONE", slot_avg)
                    slot_colors.append("NONE")
            else:
                slot_colors.append(result_color)
                #print(slot_no, ":", result_color)
            
            slot_readings = 0
            slot_low = 100
            slot_high = -1
            color_hp = 2
            #hub.speaker.beep(532)
            is_confirmed = False
            result_color = "NONE"
            if slot_no == 7:
                print(slot_colors)
                break


        if onSlot == True and is_confirmed == False:
            
            #print(left_sensor.reflection())
            if left_sensor.reflection() > 30:
                result_color = "WHITE"
                is_confirmed = True
                    

            else:
                #print("SUSSY BAKA")
                left_sensor.lights.off()
                print(left_sensor.ambient())
                slot_votes[slot_no] += left_sensor.ambient()
                slot_readings += 1
                if left_sensor.ambient() < slot_low:
                    slot_low = left_sensor.ambient()
                    

                if left_sensor.ambient() > slot_high:
                    slot_high = left_sensor.ambient()
                
                left_sensor.lights.on()
            # hub.speaker.beep(440, 10)
        
        #print(db.distance())
        db.drive(200, 0)

"""CHANGE THE STRAIGHT VALUES TO THE ONE IN SLOT == 1 IF DOESNT WORK DURING THE COMP"""

def go_to_slot(slot: int, region: int):
    if slot == 0:
        ferris_wheel_turn("NONE")
        db.turn(-180)
        db.straight(-165)
        #main_motor.run_angle(600, 285)
        gyro_turn(90, "RIGHT")
        db.straight((region*80))
        

        position["region"] = region
        position["slot"] = slot


    elif slot == 1:
        ferris_wheel_turn("SHORT")
        db.turn(90)
        linetrack_to_corner("double", 0, min_distance=120 - (region*35))
        db.straight(37)
        db.turn(102)
        db.straight(-100)

        position["region"] = region
        position["slot"] = slot


    elif slot == 2:
        prevent_right = True
        ferris_wheel_turn("SHORT")
        db.turn(90)
        linetrack_to_corner("double", 0, min_distance=120 - (region*35))
        db.straight(37)
        db.turn(-102)
        db.straight(-115)

        position["region"] = region
        position["slot"] = slot

    elif slot == 3:
        ferris_wheel_turn("NONE")
        db.straight(-165)
        gyro_turn(-90, "LEFT")
        db.straight((region*80))

        position["region"] = region
        position["slot"] = slot

    else:
        print("INVALID SLOT NUMBER")
        hub.speaker.beep(440, 500)

        #db.straight(65)
    
def return_from_slot(slot: int, region: int):
    if slot == 0:
        if region == 0:
            db.straight(35)
        else:
            db.straight(24)
        db.turn(-90)
        linetrack_to_corner("left", 0, min_distance=10, speed_multiplier=0.25)
        turn_to_line("left_back")
        
        position["region"] = -1
        position["slot"] = -1


    elif slot == 1:
        

        db.straight(105)
        db.turn(78)
        if region == 1:
            linetrack_to_corner("double", 0, min_distance=40, speed_multiplier=0.25)
        else:
            linetrack_to_corner("right", 0, min_distance=40, speed_multiplier=0.25)

        db.straight(95)
        db.turn(90)
        

        position["region"] = -1
        position["slot"] = -1


    elif slot == 2:
        prevent_right = False
        db.straight(90)
        db.turn(-78)
        if region == 1:
            linetrack_to_corner("double", 0, min_distance=40, speed_multiplier=0.25)
        else:
            linetrack_to_corner("right", 0, min_distance=40, speed_multiplier=0.25)
        db.straight(110)
        db.turn(90)
        

        position["region"] = -1
        position["slot"] = -1

    elif slot == 3:
        if region == 0:
            db.straight(35)
        else:
            db.straight(24)
        db.turn(90)
        if(region == 1):
            linetrack_to_corner("right", 0, min_distance=10, speed_multiplier=0.25)
            db.straight(95)
        else: 
            db.straight(256)

        
        
        position["region"] = -1
        position["slot"] = -1
        
    else:
        print("INVALID SLOT NUMBER")
    

        
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
            
def collect(hand: str, colour: str):
    if position["slot"] == 2:
        prevent_right = True
    elif position["slot"] == 1:
        prevent_right = False
    ferris_wheel_turn_and_down(hand)
    if position["slot"] == 0 or position["slot"] == 3:
        db.straight(-95)
    else:
        db.straight(-80)
    ferris_wheel_up_and_turn("SHORT")
    if(hands[hand][1] == "NONE"):
        hands[hand][1] = hands[hand][0]
    
    hands[hand][0] = colour

    if position["slot"] == 0 or position["slot"] == 3:
        db.straight(95)
    else:
        db.straight(80)

    prevent_right = False
    print(hands)

def deposit(hand: str, pos: int):
    if position["slot"] == 2:
        prevent_right = True
    elif position["slot"] == 1:
        prevent_right = False
    db.straight(-30)
    ferris_wheel_turn_and_down(hand)

    


    if hand == "LONG":
        if pos == 0:
            db.straight(20)
            if position["slot"] == 1 or position["slot"] == 2:
                ferris_wheel_up_and_turn("SHORT")
            else:
                ferris_wheel_up_and_turn("NONE")
            db.straight(-20)
            hands[hand][0] = hands[hand][1]
            hands[hand][1] = "NONE"

        elif pos == 1:
            db.straight(-35)
            db.straight(44)
            if position["slot"] == 1 or position["slot"] == 2:
                ferris_wheel_up_and_turn("SHORT")
            else:
                ferris_wheel_up_and_turn("NONE")

            db.straight(-44)
            db.straight(35)

            hands[hand][0] = "NONE"
            hands[hand][1] = "NONE"
        
        else:
            print("INVALID HAND POSITION")
            hub.speaker.beep(440, 500)

    elif hand == "SHORT":
        if pos == 0:
            db.straight(28)
            if position["slot"] == 1 or position["slot"] == 2:
                ferris_wheel_up_and_turn("SHORT")
            else:
                ferris_wheel_up_and_turn("NONE")
            db.straight(-28)
            hands[hand][0] = hands[hand][1]
            hands[hand][1] = "NONE"

        elif pos == 1:
            db.straight(-42)
            db.straight(62)
            if position["slot"] == 1 or position["slot"] == 2:
                ferris_wheel_up_and_turn("SHORT")
            else:
                ferris_wheel_up_and_turn("NONE")
            db.straight(-62)
            db.straight(42)

            hands[hand][0] = "NONE"
            hands[hand][1] = "NONE"
        
        else:
            print("INVALID HAND POSITION")
            hub.speaker.beep(440, 500)

    db.straight(30) 
    prevent_right = False
    print(hands)


def calibrate_direction(bearing: int):
    initial_heading = hub.imu.heading()%360
    target_heading = bearing
    print(initial_heading, target_heading)

    
    while 0.3 < abs(target_heading - hub.imu.heading()):
        if(target_heading - hub.imu.heading() > 0):
            left_motor.run(50)
            right_motor.run(-50)
            print("P", target_heading - hub.imu.heading())
        elif(target_heading - hub.imu.heading() < 0):
            left_motor.run(-50)
            right_motor.run(50)
            print("N", target_heading - hub.imu.heading())
        else:
            break

            
    

    db.straight(0, Stop.HOLD)

def test_voting():
    print("INITIAL DISTANCE:", db.distance())
    while left_sensor.color() != Color.RED:
        db.drive(30, 0)
    print("FINAL DISTANCE:", db.distance())

def push_in(white: bool, black: bool, final_hand = "NONE"):

    
    ferris_wheel_turn_and_down("NONE")
    ferris_motor.reset_angle(0)
    if main_motor.angle() < -140:
        main_motor.run_angle(600, 285)

    if white:
        while ferris_motor.angle() < 450:
            left_motor.run(-300)
            right_motor.run(-300)
            ferris_motor.run(350)
        hands["LONG"][1] = hands["LONG"][0]
        hands["LONG"][0] = "NONE"

    
    if black:
        while ferris_motor.angle() > -450:
            left_motor.run(-300)
            right_motor.run(-300)
            ferris_motor.run(-350)
        hands["SHORT"][1] = hands["SHORT"][0]
        hands["SHORT"][0] = "NONE"


    db.straight(0)
    ferris_wheel_up_and_turn(final_hand)

def find_object(color: str, hand="NONE"):
    pass
    
if __name__ == "__main__":
    db.settings(straight_speed=450, straight_acceleration=500, turn_acceleration=300)
    distances = gen_slot_distances(15, 32, 15, 94)
    ferris_motor.reset_angle(0)
    db.curve(98, 48)
    db.curve(98, -48)
    main_motor.reset_angle(0)

    db.reset()
    mono_voting(slot_distances=distances)
    
    db.turn(-10)
    db.straight(-200)

    while main_motor.angle() > -290:
        main_motor.run(-500)

    ferris_motor.run_angle(1000, 210)
    ferris_motor.reset_angle(0)
    
    linetrack_to_color(color=Color.RED, speed_multiplier=0.4)

    db.turn(180)
    db.straight(200)

    ferris_wheel_turn_and_down("LONG")
    db.turn(14)
    db.straight(-330)
    prevent_right = True
    ferris_wheel_turn("SHORT")
    db.straight(330)
    db.turn(-23)
    db.straight(-330)
    to_angle(-285)
    db.curve(2 / 3 * 45, -80)
    db.curve(2 / 3 * 45, 90)

    # Line track to red disposal area
    #db.settings(turn_acceleration=500)
    linetrack_to_corner("left", -90, min_distance=25)
    db.settings(straight_speed=450, straight_acceleration=500, turn_acceleration=500)
    linetrack_to_corner("right", 90, min_distance=500)
    db.settings(straight_speed=250, straight_acceleration=200, turn_acceleration=400)
    linetrack_to_corner("right", 0, min_distance=90)
    
