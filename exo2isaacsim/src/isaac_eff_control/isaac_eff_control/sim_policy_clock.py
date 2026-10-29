"""Integer simulation-clock deadlines; no wall timer and no catch-up bursts."""


class SimPolicyClock:
    period_ns = 100_000_000

    def __init__(self):
        self.last_ns = None
        self.next_ns = None

    def advance(self, now_ns):
        if now_ns < 0:
            raise ValueError('Simulation clock must be nonnegative')
        if self.last_ns is None or now_ns < self.last_ns:
            self.last_ns = now_ns
            self.next_ns = now_ns + self.period_ns
            return False
        self.last_ns = now_ns
        if now_ns < self.next_ns:
            return False
        # Retain the integer grid without cumulative float drift. If transport
        # missed multiple deadlines, forward one current decision, never a burst.
        self.next_ns += ((now_ns - self.next_ns) // self.period_ns + 1) * self.period_ns
        return True
