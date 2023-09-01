from logging import FileHandler

import colorlog

from .parameters import Stop
from .pupdevices import Motor

handler = colorlog.StreamHandler()
handler.setFormatter(colorlog.ColoredFormatter("%(log_color)s%(message)s"))
filehandle = FileHandler("pybricks.log")

logger = colorlog.getLogger(__name__)
logger.setLevel("DEBUG")
logger.addHandler(handler)
logger.addHandler(filehandle)


class GyroDriveBase:
    def __init__(
        self,
        left_motor: Motor,
        right_motor: Motor,
        wheel_diameter,
        axle_track,
    ):
        self.dist = 0

    def drive(self, speed, turn_rate) -> None:
        logger.debug(f"db.drive({speed}, {turn_rate})")
        self.dist += speed / 10

    def stop(self) -> None:
        logger.debug("db.stop()")

    def reset(self) -> None:
        logger.debug("db.reset()")
        self.dist = 0

    def distance(self) -> int:
        return self.dist

    def settings(self, **kwargs) -> None:
        logger.debug(f"db.settings({kwargs})")

    def straight(self, distance, then: Stop = Stop.HOLD, wait: bool = True) -> None:
        logger.debug(f"db.straight({distance})")
        self.dist += distance

    def turn(self, angle, then: Stop = Stop.HOLD, wait: bool = True) -> None:
        logger.debug(f"db.turn({angle})")

    def curve(self, radius, angle, then: Stop = Stop.HOLD, wait: bool = True) -> None:
        logger.debug(f"db.curve({radius}, {angle})")
