# autocopy

# built-in imports
import board
from math import copysign
from microcontroller import nvm
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
MOTION CONFIG
"""
CONTROL_FRAME_LENGTH_MS = 0.05
MIN_THROTTLE = 0.4
MAX_THROTTLE = 0.8
RAMP_DISTANCE = 1000

"""
MOTOR CONFIG
"""
PWM_FREQUENCY = 500
DECAY_MODE = SLOW_DECAY
DEBUG_MOTOR = False

"""
WATCHDOG CONFIG
"""
WATCHDOG_ERROR = 15
WATCHDOG_THROTTLE_WINDOW = 3
WATCHDOG_STALL_SPEED = 10
WATCHDOG_STALLED_FRAME_ALLOWANCE = 2

"""
NON-VOLATLE MEMORY
"""


def retrieve_encoder_position():
    value = int.from_bytes(nvm[0:2], "big")
    return value - 32767


def store_encoder_position(value):
    value += 32767
    nvm[0:2] = value.to_bytes(2, "big")


def retrieve_motor_starts_left():
    value = nvm[2]
    nvm[2] = max(0, value - 1)
    return value


"""
DEBUG UTILITIES
"""
# setup
debug_log = []
debug_analysis = []


def analyze_debug_log():
    # iterate over log, starting at the second element
    for prev_idx in range(len(debug_log) - 1):
        prev_mono, prev_throttle, prev_pos, prev_req_speed = debug_log[prev_idx]
        mono, throttle, pos, req_speed = debug_log[prev_idx + 1]

        delta_time = mono - prev_mono
        delta_pos = pos - prev_pos
        speed = delta_pos / delta_time * 100_000_000

        debug_analysis.append((mono, pos, speed, prev_throttle, prev_req_speed))


def csv_print_analysis():
    output = "time,pos,speed,throttle,req_speed\n"
    for analyzed in debug_analysis:
        output += ",".join(map(str, analyzed)) + "\n"
    print(output)


def pretty_print_analysis():
    # rearrange logs into values
    mono_list, pos_list, speed_list, throttle_list, requested_list = zip(
        *debug_analysis
    )

    # convert values to strings
    mono_list = list(map(str, mono_list))
    pos_list = list(map(str, pos_list))
    speed_list = list(map(str, speed_list))
    throttle_list = list(map(str, throttle_list))
    requested_list = list(map(str, requested_list))

    # find each longest string
    longest_mono = max(map(len, mono_list))
    longest_pos = max(map(len, pos_list))
    longest_speed = max(map(len, speed_list))
    longest_throttle = max(map(len, throttle_list))
    longest_requested = max(map(len, requested_list))

    # don't run shorter than the header
    mono_length = max(longest_mono, len("time"))
    pos_length = max(longest_pos, len("pos"))
    speed_length = max(longest_speed, len("speed"))
    throttle_length = max(longest_throttle, len("throttle"))
    requested_length = max(longest_requested, len("requested speed"))

    # print headers
    print(
        " | ".join(
            (
                "time".center(mono_length),
                "pos".center(pos_length),
                "speed".center(speed_length),
                "throttle".center(throttle_length),
                "requested speed".center(requested_length),
            )
        )
    )

    # print each row
    for idx in range(len(mono_list)):
        mono = mono_list[idx]
        pos = pos_list[idx]
        speed = speed_list[idx]
        throttle = throttle_list[idx]
        requested = requested_list[idx]
        print(
            " | ".join(
                (
                    mono.center(mono_length),
                    pos.center(pos_length),
                    speed.center(speed_length),
                    throttle.center(throttle_length),
                    requested.center(requested_length),
                )
            )
        )


"""
POSITION
"""
# setup
ENCODER = IncrementalEncoder(board.D6, board.D5)
ENCODER.position = retrieve_encoder_position()
pos = lambda: ENCODER.position


def reset_position():
    ENCODER.position = 0


"""
MOVEMENT UTILITIES
"""


def bound(_min, value, _max):
    return max(_min, min(_max, value))


def get_expected_speed(throttle):
    """
    Calculates speed in detents per second for a given throttle value. Does not
    anticipate lag due to acceleration.
    """
    return 129 * throttle - 2.13


def get_required_throttle(speed):
    """
    Calculates throttle to maintain a given speed (detents/sec). Does not anticipate lag
    due to acceleration.
    """
    if speed == 0:
        return 0
    _dir = copysign(1, speed)
    abs_throttle = abs((speed + 2.13) / 129)
    abs_bounded = bound(MIN_THROTTLE, abs_throttle, MAX_THROTTLE)
    return abs_bounded * _dir


def trapezoid_iter(target):
    """
    Creates a trapezoid-shaped speed vs position curve for smooth movements.
    Yields target speeds from three movement phases.
    """
    # set initial values
    start = pos()
    _dir = copysign(1, target - start)
    target = target - 10 * _dir  # stop a little short

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
    while pos() * _dir <= end_ramp_up * _dir:
        # calculate target speed based on progression through ramp distance
        ramp_speed = RAMP_SLOPE * (pos() - start)
        target_speed = ramp_speed + MIN_SPEED * _dir
        # convert to an expected throttle value
        yield target_speed

    print("# CRUISE")
    while pos() * _dir <= start_ramp_down * _dir:
        yield MAX_SPEED * _dir

    print("# RAMP DOWN")
    while pos() * _dir < target * _dir:
        # calculate target speed based on progression through ramp distance
        ramp_speed = RAMP_SLOPE * (target - pos())
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
if DEBUG_MOTOR or not retrieve_motor_starts_left():
    MOTOR = FM()
else:
    mk = MotorKit(pwm_frequency=PWM_FREQUENCY)
    MOTOR = mk.motor1
    MOTOR.decay_mode = DECAY_MODE

# COMMANDS
def stop():
    MOTOR.throttle = 0


def move_relative(delta_pos):
    go_to(pos() + delta_pos)


def go_to(target):
    """
    Changes windowshade position. Uses a trapezoid curve to generate desired
    speed vs position, sets the throttle, creates debug logs, and watches for
    exceptions from the watchdog.
    """
    if target == pos():
        print("go_to got current position!")
        return
    print(f"Moving from {pos()} to {target}.")

    # create curve iterator
    speed_curve = trapezoid_iter(target)

    # capture initial log point
    debug_log.append((monotonic_ns(), 0, pos(), 0))

    # "hand control" to iterator
    speed = throttle = None
    try:
        with MOTOR as m:
            reset_watchdog()
            while True:
                # check watchdog
                watchdog()

                # set throttle
                speed = next(speed_curve)
                throttle = get_required_throttle(speed)
                m.throttle = throttle

                # log
                debug_log.append((monotonic_ns(), throttle, pos(), speed))

                # wait a bit
                sleep(CONTROL_FRAME_LENGTH_MS)
    except StopIteration:
        # we're all good!
        print(f"We arrived at {pos()}!")
    except WatchdogException as e:
        print(f"\n\nWe made it to {pos()}. ¯\_(ツ)_/¯")
        traceback.print_exception(None, e, e.__traceback__)
        analyze_debug_log()
        pretty_print_analysis()
    except Exception as e:
        print(f"We made it to {pos()}.")
        traceback.print_exception(None, e, e.__traceback__)

    # capture final log point
    debug_log.append((monotonic_ns(), 0, pos(), 0))

    # persist position
    store_encoder_position(pos())


"""
WATCHDOG
"""
# SETUP
watchdog_settle_curve = (
    lambda x: 50.3 + -3.86e-07 * x + 1.11e-15 * pow(x, 2) + -1.08e-24 * pow(x, 3)
)  # Poly regression from partial load data
WATCHDOG_SETTLE_NS = 4.9e8  # The point at which the curve is invalid
watchdog_start = None
watchdog_last_time = None
watchdog_last_pos = None
watchdog_throttle_stack = []
watchdog_stalled_frames = 0
watchdog_max_error = 0


def reset_watchdog():
    """Get a new watchdog 'session.'"""
    global watchdog_start, watchdog_last_time, watchdog_last_pos
    watchdog_start = monotonic_ns()
    watchdog_last_time = monotonic_ns()
    watchdog_last_pos = pos()

    global watchdog_throttle_stack, watchdog_stalled_frames
    watchdog_throttle_stack = [0]
    watchdog_stalled_frames = 0


def throttle_average():
    """Calculate the sliding window average."""
    return sum(watchdog_throttle_stack) / len(watchdog_throttle_stack)


def add_watchdog_state():
    """Store information about this frame."""
    global watchdog_last_time, watchdog_last_pos
    watchdog_last_time = monotonic_ns()
    watchdog_last_pos = pos()

    # Advance sliding window
    while len(watchdog_throttle_stack) > WATCHDOG_THROTTLE_WINDOW - 1:
        watchdog_throttle_stack.pop(0)
    watchdog_throttle_stack.append(MOTOR.throttle or 0)


class WatchdogException(Exception):
    """Raised when movement expectations are not met."""

    pass


# WATCHDOG
def watchdog():
    """
    Called every control frame. Calculates expected speed based on a sliding
    window throttle average. Adjusts expectation for initial acceleration.
    Allows a specified error range, and raises an exception for out of family
    values.
    """
    # What happened this time
    delta_time = monotonic_ns() - watchdog_last_time
    if delta_time == 0:
        return
    elapsed_time_ns = monotonic_ns() - watchdog_start
    delta_pos = pos() - watchdog_last_pos
    speed = delta_pos / delta_time * 100_000_000
    add_watchdog_state()

    # What we expected to happen
    average_throttle = throttle_average()
    if average_throttle == 0:
        # We shouldn't be moving.
        expected_speed = 0
        difference = abs(delta_pos)
    else:
        # Calculate difference
        expected_speed = get_expected_speed(average_throttle)
        difference = abs(speed - expected_speed)
        # Low expectations when we're starting up
        if elapsed_time_ns <= WATCHDOG_SETTLE_NS:
            difference -= watchdog_settle_curve(elapsed_time_ns)
        # Check for stalls
        global watchdog_stalled_frames
        if abs(speed) <= WATCHDOG_STALL_SPEED:
            watchdog_stalled_frames += 1
        else:
            watchdog_stalled_frames = 0

    # Prepare for next time
    global watchdog_max_error
    watchdog_max_error = max(difference, watchdog_max_error)

    # Wow we feel about it
    if difference >= WATCHDOG_ERROR:
        error_message = (
            "actual speed of "
            + str(speed)
            + " does not match expected speed of "
            + str(expected_speed)
            + f" by {difference}, an error of "
            + f"{round(difference/expected_speed, 3):+}%"
        )
        raise WatchdogException(error_message)
    if watchdog_stalled_frames > WATCHDOG_STALLED_FRAME_ALLOWANCE:
        raise WatchdogException("stalled frames allowance exceeded")


# from microcontroller import nvm; nvm[1] += 1
if __name__ == "__main__":
    go_to(0)
    if not len(debug_analysis):
        analyze_debug_log()
    csv_print_analysis()
    print(f"watchdog_max_error: {watchdog_max_error}")
