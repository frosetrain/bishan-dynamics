class Imu:
    """Hub's IMU."""

    def __init__(self):
        ...

    def heading(self) -> int:
        return 0


class PrimeHub:
    """LEGO® SPIKE Prime Hub."""

    def __init__(self):
        self.imu = Imu()
