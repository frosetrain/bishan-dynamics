from enum import Enum, auto


class Color(Enum):
    """Light or surface color."""

    NONE = auto()
    BLACK = auto()
    GRAY = auto()
    WHITE = auto()
    RED = auto()
    ORANGE = auto()
    BROWN = auto()
    YELLOW = auto()
    GREEN = auto()
    CYAN = auto()
    BLUE = auto()
    VIOLET = auto()
    MAGENTA = auto()

    def __init__(self, h, s, v):
        self.h = h
        self.s = s
        self.v = v


Colors = {
    "NONE": Color.NONE,
    "BLACK": Color.BLACK,
    "GRAY": Color.GRAY,
    "WHITE": Color.WHITE,
    "RED": Color.RED,
    "ORANGE": Color.ORANGE,
    "BROWN": Color.BROWN,
    "YELLOW": Color.YELLOW,
    "GREEN": Color.GREEN,
    "CYAN": Color.CYAN,
    "BLUE": Color.BLUE,
    "VIOLET": Color.VIOLET,
    "MAGENTA": Color.MAGENTA,
}


class Direction(Enum):
    """Rotational direction for positive speed or angle values."""

    CLOCKWISE = 0
    COUNTERCLOCKWISE = 1


class Port(Enum):
    """Port on the programmable brick or hub."""

    # Generic motor/sensor ports
    A = ord("A")
    B = ord("B")
    C = ord("C")
    D = ord("D")
    E = ord("E")
    F = ord("F")

    # NXT/EV3 sensor ports
    S1 = ord("1")
    S2 = ord("2")
    S3 = ord("3")
    S4 = ord("4")


class Stop(Enum):
    """Action after the motor stops or reaches its target."""

    COAST = 0
    COAST_SMART = 4
    BRAKE = 1
    HOLD = 2
    NONE = 3
