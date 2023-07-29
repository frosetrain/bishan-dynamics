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
pick_angle = 10000
has_object = False


if __name__ == "__main__":
    third_motor.reset_angle()
    third_motor.run_target(200, 0, wait=False)
    # left_motor.run_angle(360, 270)
    # right_motor.run_angle(360, 270)
    # detect()

    while left_sensor.color() != Color.RED:
        ll = (left_sensor.reflection() - BLACK) / (WHITE - BLACK)
        rl = (right_sensor.reflection() - BLACK) / (WHITE - BLACK)
        db.drive(300, (ll - rl) * 60)

    db.stop()
    db.straight(-84)
    db.turn(-38)
    db.straight(60)
    third_motor.run_angle(200, 45)
    has_object = True
    db.turn(162)
    db.straight(210)
    db.turn(-25)

    pick_angle = left_motor.angle()

    for i in range(1000):
        ll = (left_sensor.reflection() - BLACK) / (WHITE - BLACK)
        rl = (right_sensor.reflection() - BLACK) / (WHITE - BLACK)
        db.drive(150, (ll - rl) * 100)

    # print(left_motor.angle())  # 2027

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
        # print(pick_angle)
        if (
            ll < 0.1
            and rl > 0.7
            and left_motor.angle() > pick_angle + 700
            and has_object == True
        ):  # funny left turn
            print("DEPOSITING WITH PICKANGLE", pick_angle)
            db.curve(200, -60, Stop.HOLD)
            third_motor.run_angle(200, -45)
            db.curve(-200, -60, Stop.HOLD)
            db.turn(185)
            has_object = False
            print("???")
            for i in range(1500):
                ll = (left_sensor.reflection() - BLACK) / (WHITE - BLACK)
                rl = (right_sensor.reflection() - BLACK) / (WHITE - BLACK)
                db.drive(150, (ll - rl) * 100)

        if ll < 0.1 and rl < 0.1:  # funny left turn
            db.straight(69)
            db.turn(90)
            # third_motor.run_angle(150, wait=False)
            db.straight(-150)
            db.straight(150)
            # third_motor.run_target(0, wait=False)

            print("PICKING UP")
            while left_sensor.color() != Color.RED:
                db.drive(300, 0)

            db.straight(-84)
            db.turn(-38)
            db.straight(69)
            third_motor.run_angle(200, 45)
            has_object = True
            db.turn(162)
            db.straight(230)
            db.turn(-25)

            pick_angle = left_motor.angle()
            print("DONE AT", pick_angle)
            for i in range(1000):
                ll = (left_sensor.reflection() - BLACK) / (WHITE - BLACK)
                rl = (right_sensor.reflection() - BLACK) / (WHITE - BLACK)
                db.drive(150, (ll - rl) * 100)
