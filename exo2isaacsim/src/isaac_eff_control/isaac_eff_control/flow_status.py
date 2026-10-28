"""Small event-only status display; never gates commands or uses ROS time."""
from time import monotonic


class FlowStatus:
    def __init__(self, logger, label, clock=monotonic):
        self.logger, self.label, self.clock = logger, label, clock
        self.last_received = None
        self.state = None
        self.show('WAITING')

    def show(self, state):
        if state != self.state:
            self.state = state
            log = self.logger.error if state == 'ERROR' else self.logger.info
            log(f'[{self.label}] {state}')

    def receive(self, valid=True):
        self.last_received = self.clock()
        self.show('ACTIVE' if valid else 'ERROR')

    def check(self):
        if self.last_received is not None and self.clock() - self.last_received > 1.0:
            self.show('STOPPED')
