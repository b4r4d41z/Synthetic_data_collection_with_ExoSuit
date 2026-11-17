"""Own one ROS node, forward terminal signals, and wait for its cleanup."""
import os
import signal
import subprocess
import sys
from pathlib import Path

# The renderer needs no ROS imports and also works before a package rebuild.
PROJECT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT/'exo2isaacsim/src/isaac_eff_control'))
from isaac_eff_control.terminal_status import ANSI, TerminalStatus


def main():
    name = sys.argv[1]
    role = {"arm_control_node": "ARM", "eff_control_node": "HAND",
            "sim_recorder_node": "REC", "bfm_action_gateway_node": "GATEWAY"}[name]
    terminal = TerminalStatus()

    def report_error(message):
        print(terminal.render(message), flush=True)
        subprocess.run(["tmux", "display-message", "-d", "10000", "-t",
                        "isaac_control:main", message], check=False)

    child = None
    stopping = False

    def stop(signum, _frame):
        nonlocal stopping
        stopping = True
        if child is not None:
            try:
                os.killpg(child.pid, signal.SIGINT)
            except ProcessLookupError:
                pass

    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, stop)
    try:
        logs = PROJECT/'records'
        logs.mkdir(exist_ok=True)
        with (logs/f'worker_{role}.log').open('a', buffering=1) as backend:
            environment = dict(os.environ, RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity}] {message}',
                               RCUTILS_COLORIZED_OUTPUT='0', PYTHONUNBUFFERED='1')
            child = subprocess.Popen(["ros2", "run", "isaac_eff_control", name],
                                     start_new_session=True, stdout=subprocess.PIPE,
                                     stderr=subprocess.STDOUT, text=True, errors='replace', env=environment)
            if stopping:
                stop(signal.SIGINT, None)
            for line in child.stdout:
                backend.write(ANSI.sub('', line))
                visible = ANSI.sub('',line).rstrip() if role=='GATEWAY' else terminal.render(line)
                if visible is not None:
                    print(visible, flush=True)
            child.stdout.close()
            code = child.wait()
    except OSError as error:
        if child is not None:
            stop(signal.SIGINT, None)
            child.wait()
        report_error(f"[{role}] ERROR {error}")
        return 1
    if not stopping or code not in (0, -signal.SIGINT, 128 + signal.SIGINT):
        message = f"[{role}] ERROR node exited ({code}); check backend"
        report_error(message)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
