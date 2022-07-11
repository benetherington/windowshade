"""
             | above zero | spanning zero | below zero |
positive dir |  5 to 100  |   -50 to 50   | -100 to -5 |
negative dir |  100 to 5  |   50 to -50   | -5 to -100 |

"""

from math import copysign, pow

def bound(_min, value, _max):
    return max(_min, min(_max, value))
class FakeSelf(object):
    pos = 0
    MIN_THROTTLE = 0.5
    MAX_THROTTLE = 1
    RAMP_DISTANCE = 500
    # RAMP_SLOPE = (MAX_THROTTLE - MIN_THROTTLE) / RAMP_DISTANCE
    def __init__(self):
        self.MAX_SPEED = self.expected_speed(self.MAX_THROTTLE)
        self.MIN_SPEED = self.expected_speed(self.MIN_THROTTLE)
        self.RAMP_SLOPE = (self.MAX_SPEED - self.MIN_SPEED) / self.RAMP_DISTANCE

    def expected_speed(self, throttle):
        return 129*throttle - 2.13

    def required_throttle(self, speed):
        _dir = copysign(1, speed)
        abs_throttle = abs((speed + 2.13) / 129)
        abs_bounded = bound(self.MIN_THROTTLE, abs_throttle, self.MAX_THROTTLE)
        return abs_bounded*_dir

self = FakeSelf()

def trapezoid_iter(self, target):
    # set initial values
    start = self.pos
    _dir = copysign(1, target - start)
    
    # set ramping parameters
    if abs(target-start) > self.RAMP_DISTANCE * 2:
        # long move, use full ramp
        end_ramp_up = start + self.RAMP_DISTANCE * _dir
        start_ramp_down = target - self.RAMP_DISTANCE * _dir
    else:
        # short move, peak halfway (below max speed)
        end_ramp_up = start_ramp_down = (target + start) / 2
    
    # calculate current throttle
    print("# RAMP UP")
    while self.pos*_dir <= end_ramp_up*_dir:
        # calculate target speed based on progression through ramp distance
        ramp_speed = self.RAMP_SLOPE * (self.pos - start)
        target_speed = ramp_speed + self.MIN_SPEED * _dir
        # convert to an expected throttle value
        yield target_speed
    
    print("# CRUISE")
    while self.pos*_dir <= start_ramp_down*_dir:
        yield self.MAX_THROTTLE * _dir
    
    print("# RAMP DOWN")
    while self.pos*_dir < target*_dir:
        # calculate target speed based on progression through ramp distance
        ramp_speed = self.RAMP_SLOPE * (target - self.pos)
        target_speed = ramp_speed + self.MIN_SPEED * _dir
        # convert to an expected throttle value
        yield target_speed
    
    # We've arrived!
    yield 0

def test_trap(delta_time, target):
    trap = trapezoid_iter(self, target)
    print("pos  | throttle | speed")
    while True:
        try:
            speed = next(trap)
            throttle = self.required_throttle(speed)
            print(f"{round(self.pos):4} | {round(throttle,3):8} | {round(speed)}")
            self.pos += speed * delta_time
        except StopIteration:
            print()
            break


delta_time = 0.1
print("\n\n")

print("pos dir, above zero (5 => 2000)")
self.pos = 5
test_trap(delta_time, 1010)

# print("pos dir, span zero (-50 => 2000)")
# self.pos = -50
# test_trap(STEP, 2000)

# print("pos dir, below zero (-2000 => -5)")
# self.pos = -2000
# test_trap(STEP, -5)

# print("neg dir, above zero (2000 => 5)")
# self.pos = 2000
# test_trap(-STEP, 5)

# print("neg dir, span zero (2000 => -50)")
# self.pos = 2000
# test_trap(-STEP, -50)

# print("neg dir, below zero (-5 => -2000)")
# self.pos = -5
# test_trap(-STEP, -2000)


class FakeMotor:
    def __init__(self):
        self._throttle = 0
    @property
    def throttle(self):
        return self._throttle
    @throttle.setter
    def throttle(self, val):
        print(val)
        self._throttle = val
    def __enter__(self):
        return self

    def __exit__(
        self,
        exception_type,
        exception_value,
        traceback,
    ):
        self.throttle = None