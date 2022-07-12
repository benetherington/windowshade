"""
             | above zero | spanning zero | below zero |
positive dir |  5 to 100  |   -50 to 50   | -100 to -5 |
negative dir |  100 to 5  |   50 to -50   | -5 to -100 |

"""

from math import copysign, pow


CONTROL_FRAME_LENGTH_MS = 0.05
MIN_THROTTLE = 0.4
MAX_THROTTLE = 0.8
RAMP_DISTANCE = 1000


def bound(_min, value, _max):
    return max(_min, min(_max, value))


def get_expected_speed(throttle):
    # Calculates speed in encoder detents per second. NOTE: does not work
    # outside of expected throttle range (0.5-1)!
    return 129 * throttle - 2.13


def get_required_throttle(speed):
    # Calculates throttle. NOTE: does not work outside of expected speed
    # range (12-60 detents/sec)!
    if speed == 0:
        return 0
    _dir = copysign(1, speed)
    abs_throttle = abs((speed + 2.13) / 129)
    abs_bounded = bound(MIN_THROTTLE, abs_throttle, MAX_THROTTLE)
    return abs_bounded * _dir


MAX_SPEED = get_expected_speed(MAX_THROTTLE)
MIN_SPEED = get_expected_speed(MIN_THROTTLE)
RAMP_SLOPE = (MAX_SPEED - MIN_SPEED) / RAMP_DISTANCE

pos = 0

POS = lambda: pos


"""
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


DELTA_T = 0.05


def test_trap(target):
    global pos
    trap = trapezoid_iter(target)
    iter_count = 0
    while True:
        if iter_count % 20 == 0:
            print("pos  | throttle | speed")
        try:
            speed = next(trap)
            throttle = get_required_throttle(speed)
            print(f"{round(pos):4} | {round(throttle,3):8} | {round(speed)}")
            pos += speed * DELTA_T
        except StopIteration:
            print()
            break
        iter_count += 1


print("\n\n")


# print("pos dir, above zero (5 => 2000)")
# pos = 5
# test_trap(2000)

# print("pos dir, span zero (-50 => 2000)")
# pos = -50
# test_trap(2000)

# print("pos dir, below zero (-2000 => -5)")
# pos = -2000
# test_trap(-5)

# print("neg dir, above zero (2000 => 5)")
# pos = 2000
# test_trap(5)

print("neg dir, span zero (2000 => -50)")
pos = 2000
test_trap(-50)

# print("neg dir, below zero (-5 => -2000)")
# pos = -5
# test_trap(-2000)
"""
