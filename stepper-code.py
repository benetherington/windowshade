#autocopy

from adafruit_motorkit import MotorKit
from adafruit_motor.stepper import (FORWARD, BACKWARD,
                                    MICROSTEP, INTERLEAVE,
                                    SINGLE, DOUBLE)


mk = MotorKit()

while True:
    mk.stepper2.onestep(style=INTERLEAVE)

"""
Single step FORWARD fails without a sleep, even a 0.001 sleep.
Everything else works fine.

Microstep hums in intermediate positions, silent on %16==0, quiet on
%16==8, progressively louder between.

MICROSTEP -- 3200 steps per 360
INTERLEAVE -- 400 steps per 360
SINGLE -- 200 steps per 360
DOUBLE -- 200 steps per 360
"""

