"""Interactive diagnostic rosbag2 MCAP capture; no command publishers."""
import argparse
from contextlib import contextmanager
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import termios
import time
import tty
from uuid import uuid4

from common import Lifecycle, recording, required_topics, subscriptions, clock_ns


def utc():
    return datetime.now(timezone.utc).isoformat()


def save(path, data):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(data, indent=2, allow_nan=False)+'\n')
    os.replace(temporary, path)


@contextmanager
def keyboard(stream):
    if not stream.isatty():
        raise ValueError('Interactive recorder requires a terminal (TTY)')
    settings = termios.tcgetattr(stream.fileno())
    try:
        tty.setcbreak(stream.fileno())
        yield
    finally:
        termios.tcsetattr(stream.fileno(), termios.TCSADRAIN, settings)


class Capture:
    def __init__(self, output, snapshot, policy_metadata):
        self.output, self.snapshot, self.policy_metadata = Path(output), snapshot, policy_metadata
        self.process = None
        self.path = None
        self.data = None

    def start(self):
        if self.process is not None:
            return
        self.output.mkdir(parents=True, exist_ok=True)
        self.path = self.output/('rollout_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid4().hex[:8])
        self.path.mkdir()  # Exclusive: never reuse or overwrite a recording.
        from isaac_eff_control.recording_provenance import recording_code_provenance
        import isaac_eff_control.recording_provenance as provenance
        self.started = time.monotonic()
        self.data = dict(recording_id=self.path.name, status='recording', start_wall_utc=utc(),
                         end_wall_utc=None, recording_selection='all including hidden, except /bfm/recorder_control; discovery enabled',
                         start=self.snapshot(), policy=self.policy_metadata,
                         recording_code=recording_code_provenance(provenance.__file__),
                         tool_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest()
                                      for p in Path(__file__).parent.glob('*.py')},
                         clocks={'bag_log':'rosbag2 system receive time; no --use-sim-time',
                                 'headers':'unchanged source-native domains', 'simulation':'/clock'})
        save(self.path/'capture.json', self.data)
        try:
            with (self.path/'rosbag2.log').open('ab') as log:
                self.process = subprocess.Popen(
                    ['ros2','bag','record','--storage','mcap','--output',str(self.path/'bag'),
                     '--all','--include-hidden-topics',
                     # This private channel counts subscribers requiring STOP
                     # acknowledgements. Rosbag must never become one of them.
                     '--exclude-regex','^/bfm/recorder_control$'], start_new_session=True,
                    stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT)
        except Exception as error:
            self.data.update(status='failed', stop_reason='start_error', end_wall_utc=utc(), error=str(error))
            save(self.path/'capture.json', self.data)
            raise
        print(f'[ROLLOUT] Recording: {self.path}', flush=True)

    def stop(self, reason):
        if self.process is None:
            return
        process = self.process
        escalated = False
        # Ignore a repeated terminal signal while the bounded flush/cleanup runs.
        handlers = {s:signal.signal(s, signal.SIG_IGN) for s in (signal.SIGINT,signal.SIGTERM)}
        try:
            for sig, timeout in ((signal.SIGINT,20), (signal.SIGTERM,5), (signal.SIGKILL,5)):
                if process.poll() is not None:
                    break
                if sig != signal.SIGINT:escalated = True
                try:
                    os.killpg(process.pid, sig)
                except ProcessLookupError:
                    pass
                try:
                    process.wait(timeout=timeout)
                except subprocess.TimeoutExpired:
                    continue
            if process.poll() is None:
                raise RuntimeError('rosbag2 did not exit; process handle retained, MCAP may be incomplete')
        finally:
            for sig, handler in handlers.items():signal.signal(sig, handler)
        self.process = None
        try:
            end_snapshot = self.snapshot()
        except Exception as error:
            end_snapshot = dict(unavailable_reason=str(error))
        self.data.update(end_wall_utc=utc(), duration_wall_sec=time.monotonic()-self.started,
                         end=end_snapshot, stop_reason=reason, returncode=process.returncode,
                         shutdown_escalated=escalated, status='failed')
        try:
            _, _, topics = recording(self.path)
            self.data['recorded_topics'] = {n:dict(type=t, count=c) for n,(t,c) in topics.items()}
            try:
                required_topics(topics)
                self.data['replayable'] = True
            except ValueError as error:
                self.data.update(replayable=False, replay_error=str(error))
            if escalated or process.returncode not in (0, -signal.SIGINT):
                raise ValueError('rosbag2 required escalation or exited with an error')
            self.data['status'] = 'complete'
        except (ValueError, OSError) as error:
            self.data['error'] = str(error)+'; MCAP may be incomplete, inspect rosbag2.log'
        save(self.path/'capture.json', self.data)
        print(f"[ROLLOUT] {self.data['status']}: {reason}; {self.path}", flush=True)
        if self.data['status'] != 'complete':
            raise RuntimeError(self.data['error'])


class Rollout:
    def __init__(self, capture, lifecycle):
        self.capture, self.life = capture, lifecycle
        self.clock = None
        self.boundary = None
        self.clock_at_play = None
        self.play_advancing = False
        self.pending_start = False

    def episode(self, text):
        previous = self.life.phase
        if not self.life.receive(text):return
        if self.life.phase not in ('playing','paused') or (self.boundary is not None and self.life.boundary != self.boundary):
            self.capture.stop('simulation_stop')
            self.play_advancing = False
        if self.life.phase != 'playing':self.pending_start = False
        if self.life.phase == 'playing' and previous != 'playing':
            self.clock_at_play = self.clock
            self.play_advancing = False
        self.boundary = self.life.boundary

    def time(self, now):
        if self.clock is not None and now < self.clock:
            self.capture.stop('clock_reset')
            self.play_advancing = False
            self.clock_at_play = now
            self.pending_start = False
        elif self.life.phase == 'playing' and self.clock_at_play is not None and now > self.clock_at_play:
            self.play_advancing = True
            if self.pending_start and self.clock is not None and now > self.clock:
                self.pending_start = False
                self.clock = now
                self.capture.start()
        if self.life.phase == 'playing' and self.clock_at_play is None:
            self.clock_at_play = now
        self.clock = now

    def key(self, key):
        if key == 'q':
            self.pending_start = False
            self.capture.stop('keyboard_quit')
            return False
        if key == ' ':
            if self.capture.process is not None:
                self.capture.stop('keyboard')
            elif self.pending_start:
                self.pending_start = False
            elif self.life.phase == 'playing' and self.play_advancing:
                self.pending_start = True
                print('[ROLLOUT] Start armed; waiting for the next advancing physics clock', flush=True)
            else:
                print('[ROLLOUT] Wait for live PLAY and an advancing /clock before starting', flush=True)
        return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path(__file__).resolve().parents[2]/'records/evaluation')
    parser.add_argument('--checkpoint', help='Optional user-supplied checkpoint identity, not verified')
    parser.add_argument('--task-z', help='Optional user-supplied task-z identity, not verified')
    args = parser.parse_args()
    if not sys.stdin.isatty():raise ValueError('Run this recorder in an interactive terminal')
    import rclpy
    rclpy.init()
    node = rclpy.create_node('bfm_rollout_recorder_'+uuid4().hex[:8])
    life = Lifecycle()
    def snapshot():
        return dict(simulation_ns=control.clock, phase=life.phase, boundary_id=life.boundary,
                    episode=life.episode, discovered_topics=dict(node.get_topic_names_and_types()))
    capture = Capture(args.output, snapshot, dict(checkpoint=args.checkpoint, task_z=args.task_z))
    control = Rollout(capture, life)
    handles = subscriptions(node, lambda m: control.time(clock_ns(m)), lambda m: control.episode(m.data))
    def interrupt(*_):raise KeyboardInterrupt
    old = {s:signal.signal(s, interrupt) for s in (signal.SIGINT,signal.SIGTERM)}
    print('[ROLLOUT] Space: start/stop; q: quit. PAUSE keeps the bag open; STOP finalizes.', flush=True)
    try:
        with keyboard(sys.stdin):
            running = True
            while rclpy.ok() and running:
                rclpy.spin_once(node, timeout_sec=.05)
                if capture.process is not None and capture.process.poll() is not None:
                    capture.stop('recorder_exit')
                    raise RuntimeError('rosbag2 exited unexpectedly; inspect rosbag2.log')
                if select.select([sys.stdin], [], [], 0)[0]:
                    running = control.key(sys.stdin.read(1) or 'q')
    finally:
        try:
            capture.stop('signal' if sys.exc_info()[0] is KeyboardInterrupt else 'exit')
        finally:
            for sig, handler in old.items():signal.signal(sig, handler)
            node.destroy_node()
            if rclpy.ok():rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('[ROLLOUT] Interrupted; recording finalized')
    except Exception as error:
        raise SystemExit(f'[ROLLOUT] ERROR: {error}')
