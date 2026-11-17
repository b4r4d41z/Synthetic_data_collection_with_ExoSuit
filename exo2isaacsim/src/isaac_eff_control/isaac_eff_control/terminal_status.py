"""TTY-only coloring/rendering. ROS messages and file logs stay uncolored."""
import os
import re
import sys

COLORS = {'WAITING':90, 'STOPPED':90, 'STARTING':34, 'ACTIVE':32,
          'RECORDING':32, 'DATA STREAMS OK':32, 'SAVED':32, 'WARNING':33, 'ERROR':31}
ANSI = re.compile(r'\x1b\[[0-?]*[ -/]*[@-~]')
STATUS = re.compile(r'\[(ARM|HAND|REC|GATEWAY)\] (DATA STREAMS OK|WAITING|STOPPED|STARTING|ACTIVE|RECORDING|SAVED|ERROR|WARNING)(.*)')


def token(state, stream=None):
    stream = sys.stdout if stream is None else stream
    if 'NO_COLOR' not in os.environ and stream.isatty() and state in COLORS:
        return f'\x1b[{COLORS[state]}m{state}\x1b[0m'
    return state


class TerminalStatus:
    def __init__(self, stream=None):
        self.stream = sys.stdout if stream is None else stream
        self.previous = None

    def render(self, line):
        line = ANSI.sub('', line).rstrip('\r\n')
        match = STATUS.search(line)
        if match:
            label, state, suffix = match.groups()
            key = (label,state,suffix)
            if key == self.previous and state not in ('ERROR','WARNING'):
                return None
            self.previous = key
            return f'[{label}] {token(state,self.stream)}{suffix}'
        # Keep all warnings, errors and unclassified output (including tracebacks).
        # Only routine INFO goes to the existing backend files instead of the pane.
        if line.startswith('[INFO]'):
            return None
        for severity, state in (('[WARN]','WARNING'),('[ERROR]','ERROR'),('[FATAL]','ERROR')):
            if line.startswith(severity):
                return line.replace(severity, '['+token(state,self.stream)+']', 1)
        return line
