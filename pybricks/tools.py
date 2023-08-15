from time import sleep, time


class StopWatch:
    def __init__(self) -> None:
        self.begin = time()

    def reset(self) -> None:
        self.begin = time()

    def time(self) -> int:
        return (time() - self.begin) * 1000


def wait(s):
    sleep(s / 1000)
