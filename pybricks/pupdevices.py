from logging import FileHandler

import colorlog

from .parameters import Color, Direction, Port

handler = colorlog.StreamHandler()
handler.setFormatter(colorlog.ColoredFormatter("%(log_color)s%(message)s"))
filehandle = FileHandler("pybricks.log")

logger = colorlog.getLogger(__name__)
logger.setLevel("DEBUG")
logger.addHandler(handler)
logger.addHandler(filehandle)


class Motor:
    """LEGO® Powered Up motor with rotation sensors."""

    def __init__(
        self,
        port: Port,
        positive_direction: Direction = Direction.CLOCKWISE,
        reset_angle: bool = True,
        gears=None,
    ):
        self.motor_angle = 0

    def run_target(self, speed, target_angle, wait=True):
        logger.debug(f"run_target({speed}, {target_angle})")
        self.motor_angle = target_angle

    def run_angle(self, speed, angle, wait=True):
        logger.debug(f"run_angle({speed}, {angle})")
        self.motor_angle += angle / 10

    def reset_angle(self, speed, angle=None):
        logger.debug(f"reset_angle({speed}, {angle})")

    def run(self, speed):
        logger.debug(f"run({speed})")
        self.motor_angle += speed / 10

    def angle(self) -> int:
        return self.motor_angle


class ColorSensor:
    """LEGO® SPIKE Color Sensor."""

    def __init__(self, port: Port):
        ...

    def detectable_colors(self, colors):
        ...

    def reflection(self) -> int:
        return 0

    def color(self) -> Color:
        # TODO
        return Color.NONE
