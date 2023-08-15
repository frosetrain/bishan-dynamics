from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from pybricks.geometry import Matrix

hub = PrimeHub()

left_motor = Motor(Port.B, positive_direction=Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.F, positive_direction=Direction.CLOCKWISE)
left_sensor = ColorSensor(Port.A)
right_sensor = ColorSensor(Port.E)
db = DriveBase(left_motor, right_motor, 56, 160)
db.settings(straight_acceleration=1500, turn_rate=120, turn_acceleration=1000)

hue = 0
morse = "11101010100010100010101000101010100010111000111010000000111010100011101011101110001110100010111000111011100010100011101011101000101010"
name = "B           I     S       H         A       N    /      D         Y               N       A       M      I     C                S     "
a1 = Matrix(
    [
        [100, 0, 100, 0, 100],
        [0, 100, 0, 100, 0],
        [100, 0, 100, 0, 100],
        [0, 100, 0, 100, 0],
        [100, 0, 100, 0, 100],
    ]
)
a2 = Matrix(
    [
        [0, 100, 0, 100, 0],
        [100, 0, 100, 0, 100],
        [0, 100, 0, 100, 0],
        [100, 0, 100, 0, 100],
        [0, 100, 0, 100, 0],
    ]
)

if __name__ == "__main__":
    for c, cc in zip(morse, name):
        hue += 10
        hub.light.on(Color(h=hue, s=100, v=100))
        if cc == "/":
            hub.display.off()
        elif cc != " ":
            hub.display.char(cc)

        if int(c) == 1:
            db.turn(20 * int(c), then=Stop.NONE, wait=False)
        else:
            db.stop()
        wait(120)

    hub.display.animate([a1, a2], 100)
    db.settings(straight_acceleration=1500, turn_rate=300, turn_acceleration=1000)
    hue += 30
    hub.light.on(Color(h=hue, s=100, v=100))
    db.turn(180)
    hue += 30
    hub.light.on(Color(h=hue, s=100, v=100))
    db.turn(-180)
    hue += 30
    hub.light.on(Color(h=hue, s=100, v=100))
    db.turn(45)
    hue += 30
    hub.light.on(Color(h=hue, s=100, v=100))
    db.turn(45)
    hue += 30
    hub.light.on(Color(h=hue, s=100, v=100))
    db.turn(-180)
    hue += 30
    hub.light.on(Color(h=hue, s=100, v=100))
    db.turn(180)
    hue += 30
    hub.light.on(Color(h=hue, s=100, v=100))
    db.turn(-45)
    hue += 30
    hub.light.on(Color(h=hue, s=100, v=100))
    db.turn(-45)
    hue += 30
    hub.light.on(Color(h=hue, s=100, v=100))
    db.turn(90)
    hue += 30
    hub.light.on(Color(h=hue, s=100, v=100))
    db.turn(-180)
    hue += 30
    hub.light.on(Color(h=hue, s=100, v=100))
    db.turn(90)
    hue += 30
    hub.light.on(Color(h=hue, s=100, v=100))

    while True:
        db.drive(250, 1000)
