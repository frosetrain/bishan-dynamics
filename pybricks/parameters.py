from enum import Enum


class Color:
    """Light or surface color."""

    NONE = ...
    BLACK = ...
    GRAY = ...
    WHITE = ...
    RED = ...
    ORANGE = ...
    BROWN = ...
    YELLOW = ...
    GREEN = ...
    CYAN = ...
    BLUE = ...
    VIOLET = ...
    MAGENTA = ...

    def __init__(self, h, s, v):
        self.h = h
        self.s = s
        self.v = v


Color.NONE = Color(0, 0, 0)
Color.BLACK = Color(0, 0, 10)
Color.GRAY = Color(0, 0, 50)
Color.WHITE = Color(0, 0, 100)
Color.RED = Color(0, 100, 100)
Color.ORANGE = Color(30, 100, 100)
Color.BROWN = Color(30, 100, 50)
Color.YELLOW = Color(60, 100, 100)
Color.GREEN = Color(120, 100, 100)
Color.CYAN = Color(180, 100, 100)
Color.BLUE = Color(240, 100, 100)
Color.VIOLET = Color(270, 100, 100)
Color.MAGENTA = Color(300, 100, 100)

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
