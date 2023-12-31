"""Resets the mechanisms to the proper starting position."""

from pybricks.hubs import PrimeHub
from pybricks.parameters import Color, Direction, Port
from pybricks.pupdevices import ColorSensor, Motor
from pybricks.robotics import GyroDriveBase

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

# mech_motor.
# main_motor.run_angle(100, -240)

if __name__ == "__main__":
    mech_motor.run_angle(300, 210)
    # print(mech_motor.angle())

    main_motor.run_angle(300, 300)
