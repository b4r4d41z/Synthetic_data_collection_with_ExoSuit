"""Own one ROS node, forward terminal signals, and wait for its cleanup."""
import os
import signal
import subprocess
import sys


def main():
    name = sys.argv[1]
    role = {"arm_control_node": "ARM", "eff_control_node": "HAND",
            "sim_recorder_node": "REC", "bfm_action_gateway_node": "GATEWAY"}[name]

    def report_error(message):
        print(message, flush=True)
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
        child = subprocess.Popen(["ros2", "run", "isaac_eff_control", name],
                                 start_new_session=True)
        if stopping:
            stop(signal.SIGINT, None)
        code = child.wait()
    except OSError as error:
        report_error(f"[{role}] ERROR {error}")
        return 1
    if not stopping or code not in (0, -signal.SIGINT, 128 + signal.SIGINT):
        message = f"[{role}] ERROR node exited ({code}); check backend"
        report_error(message)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
