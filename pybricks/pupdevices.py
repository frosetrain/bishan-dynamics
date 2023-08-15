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
        ...

    def run_target(self, speed, target_angle):
        logger.debug(f"run_target({speed}, {target_angle})")

    def run_angle(self, speed, angle):
        logger.debug(f"run_angle({speed}, {angle})")

    def reset_angle(self, speed, angle=None):
        logger.debug(f"reset_angle({speed}, {angle})")

    def angle(self) -> int:
        return 2


class ColorSensor:
    """LEGO® SPIKE Color Sensor."""

    def __init__(self, port: Port):
        ...

    def detectable_colors(self, colors):
        ...

    def reflection(self) -> int:
        # TODO
        return None

    def color(self) -> Color:
        # TODO
        return Color.NONE
