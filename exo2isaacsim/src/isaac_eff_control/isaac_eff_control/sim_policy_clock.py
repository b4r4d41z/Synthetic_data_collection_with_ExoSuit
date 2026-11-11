"""Rational simulation deadlines; no rounding drift or catch-up bursts."""

class SimPolicyClock:
    def __init__(self, hz=15):
        if type(hz) is not int or hz <= 0:
            raise ValueError('Expected positive integer Hz')
        self.hz = hz
        self.last_ns = self.origin = None
        self.index = 1

    def advance(self, now_ns):
        if type(now_ns) is not int or now_ns < 0:
            raise ValueError('Expected nonnegative integer simulation ns')
        if self.last_ns is None or now_ns < self.last_ns:
            self.last_ns = self.origin = now_ns
            self.index = 1
            return False
        if now_ns == self.last_ns:
            return False
        self.last_ns = now_ns
        # ROS time conversion can floor a rational physics instant by one ns.
        reached = (now_ns - self.origin + 1) * self.hz // 1_000_000_000
        if reached < self.index:
            return False
        self.index = reached + 1
        return True
