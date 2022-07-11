#autocopy

class FM:
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