# aautocopy

# built-in imports
import board
from math import copysign
from time import monotonic_ns, sleep
import traceback
from rotaryio import IncrementalEncoder

# Adafruit library imports
from adafruit_motor.motor import FAST_DECAY, SLOW_DECAY
from adafruit_motorkit import MotorKit
from neopixel import NeoPixel

# Local imports
from fake_motor import FM

# turn off neopixel
pixels = NeoPixel(board.NEOPIXEL, 1)
pixels[0] = (0, 0, 0)


"""
Motion config
"""
MIN_THROTTLE = 0.4
MAX_THROTTLE = 0.8
RAMP_DISTANCE = 1000
PWM_FREQUENCY = 500
DECAY_MODE = SLOW_DECAY
DEBUG_MOTOR = False

"""
Watchdog config
"""
# acceptable position error
WATCHDOG_ERROR = 15
# acceptable error for low throttle (added to e
LOW_THROTTLE_ADJUSTMENT = 20
# acceptable number of frames with no movement
STALLED_FRAMES_ALLOWANCE = 3


"""
DEBUG UTILITIES
"""
# setup
debug_log = []
debug_analysis = []

# from code import *; ws = WS(); ws.go_to(3000); ws.go_to(0); ws.analyze_debug_log(); ws.csv_print_analysis();
def analyze_debug_log():
    # iterate over log, starting at the second element
    for prev_idx in range(len(debug_log) - 1):
        prev_mono, prev_throttle, prev_pos, prev_req_speed = debug_log[prev_idx]
        mono, throttle, pos, req_speed = debug_log[prev_idx + 1]

        delta_mono = mono - prev_mono
        delta_pos = pos - prev_pos
        speed = delta_pos / delta_mono * 100_000_000

        debug_analysis.append((mono, pos, speed, prev_throttle))


def csv_print_analysis():
    output = "time,pos,speed,throttle\n"
    for analyzed in debug_analysis:
        output += ",".join(map(str, analyzed)) + "\n"
    print(output)


def pretty_print_analysis():
    # rearrange logs into values
    mono_list, pos_list, speed_list, throttle_list = zip(*debug_analysis)

    # convert values to strings
    mono_list = list(map(str, mono_list))
    pos_list = list(map(str, pos_list))
    speed_list = list(map(str, speed_list))
    throttle_list = list(map(str, throttle_list))

    # find each longest string
    longest_mono = max(map(len, mono_list))
    longest_pos = max(map(len, pos_list))
    longest_speed = max(map(len, speed_list))
    longest_throttle = max(map(len, throttle_list))

    # don't run shorter than the header
    mono_length = max(longest_mono, len("time"))
    pos_length = max(longest_pos, len("pos"))
    speed_length = max(longest_speed, len("speed"))
    throttle_length = max(longest_throttle, len("throttle"))

    # print headers
    print(
        " | ".join(
            (
                "time".center(mono_length),
                "pos".center(pos_length),
                "speed".center(speed_length),
                "throttle".center(throttle_length),
            )
        )
    )

    # print each row
    for idx in range(len(mono_list)):
        mono = mono_list[idx]
        pos = pos_list[idx]
        speed = speed_list[idx]
        throttle = throttle_list[idx]
        print(
            " | ".join(
                (
                    mono.center(mono_length),
                    pos.center(pos_length),
                    speed.center(speed_length),
                    throttle.center(throttle_length),
                )
            )
        )


"""
POSITION
"""
# setup
ENCODER = IncrementalEncoder(board.D6, board.D5)
POS = lambda: ENCODER.position


def reset_position():
    ENCODER.position = 0


"""
MOVEMENT UTILITIES
"""


def bound(_min, value, _max):
    return max(_min, min(_max, value))


def get_expected_speed(throttle):
    # Calculates speed in encoder detents per second. NOTE: does not work
    # outside of expected throttle range (0.5-1)!
    return 129 * throttle - 2.13


def get_required_throttle(speed):
    # Calculates throttle. NOTE: does not work outside of expected speed
    # range (12-60 detents/sec)!
    _dir = copysign(1, speed)
    abs_throttle = abs((speed + 2.13) / 129)
    abs_bounded = bound(MIN_THROTTLE, abs_throttle, MAX_THROTTLE)
    return abs_bounded * _dir


def trapezoid_iter(target):
    # set initial values
    start = POS()
    _dir = copysign(1, target - start)

    # set ramping parameters
    if abs(target - start) > RAMP_DISTANCE * 2:
        # long move, use full ramp
        end_ramp_up = start + RAMP_DISTANCE * _dir
        start_ramp_down = target - RAMP_DISTANCE * _dir
    else:
        # short move, peak halfway (below max speed)
        end_ramp_up = start_ramp_down = (target + start) / 2

    # calculate current throttle
    print("# RAMP UP")
    while POS() * _dir <= end_ramp_up * _dir:
        # calculate target speed based on progression through ramp distance
        ramp_speed = RAMP_SLOPE * (POS() - start)
        target_speed = ramp_speed + MIN_SPEED * _dir
        # convert to an expected throttle value
        yield target_speed

    print("# CRUISE")
    while POS() * _dir <= start_ramp_down * _dir:
        yield MAX_SPEED * _dir

    print("# RAMP DOWN")
    while POS() * _dir < target * _dir:
        # calculate target speed based on progression through ramp distance
        ramp_speed = RAMP_SLOPE * (target - POS())
        target_speed = ramp_speed + MIN_SPEED * _dir
        # convert to an expected throttle value
        yield target_speed

    # We've arrived!
    yield 0


"""
MOVEMENT
"""
# SETUP
MAX_SPEED = get_expected_speed(MAX_THROTTLE)
MIN_SPEED = get_expected_speed(MIN_THROTTLE)
RAMP_SLOPE = (MAX_SPEED - MIN_SPEED) / RAMP_DISTANCE
if DEBUG_MOTOR:
    MOTOR = FM()
else:
    mk = MotorKit(pwm_frequency=PWM_FREQUENCY)
    MOTOR = mk.motor1
    MOTOR.decay_mode = DECAY_MODE

# COMMANDS
def stop():
    MOTOR.throttle = 0


def move_relative(delta_pos):
    go_to(POS() + delta_pos)


def go_to(target):
    if target == POS():
        print("go_to got current position!")
        return
    print(f"Moving from {POS()} to {target}.")

    # create curve iterator
    speed_curve = trapezoid_iter(target)

    # "hand control" to iterator
    speed = throttle = None
    try:
        with MOTOR as m:
            while True:
                # check watchdog
                watchdog_tick()

                # set throttle
                speed = next(speed_curve)
                throttle = get_required_throttle(speed)
                m.throttle = throttle

                # log
                debug_log.append((monotonic_ns(), throttle, POS(), speed))

                # wait a bit
                sleep(0.1)
    except StopIteration:
        # we're all good!
        print(f"We arrived at {POS()}!")
    except WatchdogException as e:
        print(f"\n\nWe made it to {POS()}. ¯\_(ツ)_/¯")
        traceback.print_exception(None, e, e.__traceback__)
        analyze_debug_log()
        pretty_print_analysis()
    except Exception as e:
        print(f"We made it to {POS()}.")
        traceback.print_exception(None, e, e.__traceback__)

    # capture final log point
    debug_log.append((monotonic_ns(), throttle, POS(), speed))


"""
WATCHDOG
"""
# SETUP
watchdog_state = {
    "time": None,
    "pos": None,
    "throttle": None,
}
watchdog_stalled_frames = 0
watchdog_max_error = 0


def reset_watchdog():
    watchdog_state["pos"] = POS()
    watchdog_state["throttle"] = MOTOR.throttle
    watchdog_state["time"] = monotonic_ns()
    global watchdog_stalled_frames
    watchdog_stalled_frames = 0


class WatchdogException(Exception):
    pass


# WATCHDOG
def watchdog_tick():
    # don't judge too early
    if not watchdog_state["time"] or watchdog_state["throttle"]:
        return reset_watchdog()

    # what happened this time
    delta_time = monotonic_ns() - watchdog_state["time"]
    if delta_time == 0:
        return
    delta_pos = POS() - watchdog_state["pos"]
    speed = delta_pos / delta_time * 100_000_000

    # what we expected to happen
    non_zero_throttle = watchdog_state["throttle"] or MOTOR.throttle
    if not non_zero_throttle:
        # we shouldn't be moving.
        expected_speed = 0
        difference = abs(delta_pos)
    else:
        expected_speed = get_expected_speed(non_zero_throttle)
        difference = abs(speed - expected_speed)
        # check for stalled frames
        if speed == 0:
            watchdog_stalled_frames += 1
        else:
            watchdog_stalled_frames = 0
        # low expectations for low throttles
        if abs(non_zero_throttle) < 0.4:
            difference -= LOW_THROTTLE_ADJUSTMENT

    # how we feel about it
    if difference >= WATCHDOG_ERROR:
        error_message = (
            "actual movement of "
            + str(delta_pos)
            + " exceeds expected movement of "
            + str(expected_speed)
        )
        if expected_speed:
            error_message += f" by {difference}, an error of {round(difference/expected_speed, 3):+}%"
        raise WatchdogException(error_message)
    if watchdog_stalled_frames > STALLED_FRAMES_ALLOWANCE:
        raise WatchdogException("stalled frames allowance exceeded")

    # prepare for next time
    watchdog_max_error = max(difference, watchdog_max_error)
    watchdog_state["pos"] = POS()
    watchdog_state["throttle"] = MOTOR.throttle
    watchdog_state["time"] = monotonic_ns()


# 3.98333 in 5 sec at throttle 0.4 but stalling out!
# 8.3875 in 5 sec at throttle 0.5
# 12.1208 in 5 sec at throttle 0.6

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

"""
https://engineering.stackexchange.com/questions/36386/how-can-i-implement-s-curve-motion-profile-on-mobile-robotic-platform-wheeled

Sigmoid derivative
# prepare
distance = ( target - start )
lamb = (MAX_ERROR/distance)
beta = 4 * max_speed / distance
charlie = 1/beta * math.log( (1-lamb)/lamb )

# calculate current throttle
e_pow = math.pow(math.e, -beta*(pos-charlie))
throttle = (target-start) * (   beta*e_pow / math.pow((1+e_pow), 2)   )

"""
