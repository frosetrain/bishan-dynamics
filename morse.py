from pysinewave import SineWave
from time import sleep

sinewave = SineWave(pitch=9)
morse = "-... .. ... .... .- -. / -.. -.-- -. .- -- .. -.-. ..."
s = ""

for c in morse:
    if c == "-":
        # print("1110", end="")
        s += "1110"
    elif c == ".":
        # print("10", end="")
        s += "10"
    elif c == " ":
        # print("00", end="")
        s += "00"
    elif c == "/":
        # print("00", end="")
        s += "00"

for c in s:
    if c == "1":
        sinewave.play()
    else:
        sinewave.stop()
    sleep(0.1)
