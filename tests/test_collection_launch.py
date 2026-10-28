"""Production launcher execution with fake dependencies; no Isaac/ROS claim."""
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

import test_recording_pipeline as harness

ROOT = harness.ROOT


class LaunchTests(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.root = Path(tmp.name)
        for name in ('launch_bfm_isaac.sh', 'start_isaac_control.sh', 'tools/collection'):
            source, target = ROOT/name, self.root/name
            if source.is_dir(): shutil.copytree(source, target)
            else: shutil.copy(source, target)
        self.setup = self.root/'exo2isaacsim/install/setup.bash'
        self.setup.parent.mkdir(parents=True)
        self.setup.write_text(':\n')
        ros = self.root/'jazzy.bash'; ros.write_text(':\n')
        env = self.root/'tools/collection/environment.bash'
        env.write_text(env.read_text().replace('/opt/ros/jazzy/setup.bash', str(ros)))
        self.log = self.root/'tmux.jsonl'
        self.executable('tmux', f'''#!{sys.executable}
import json, sys
with open({str(self.log)!r}, 'a') as stream:
    stream.write(json.dumps(sys.argv[1:])+'\\n')
if sys.argv[1] == 'has-session': sys.exit(1)
print('%1')
''')
        self.env = {**os.environ, 'PATH': str(self.root)+os.pathsep+os.environ['PATH']}
        for key in tuple(self.env):
            if key.startswith('BFM_') or key == 'ISAAC_SIM_PATH': del self.env[key]

    def executable(self, name, contents):
        path = self.root/name; path.write_text(contents); path.chmod(0o755)
        return path

    def run_script(self, name):
        return subprocess.run(['bash', str(self.root/name)], env=self.env,
                              text=True, capture_output=True)

    def test_no_prefix_three_panes_order_gateway_backend_once(self):
        result = self.run_script('start_isaac_control.sh')
        self.assertEqual(result.returncode, 0, result.stderr)
        calls = [json.loads(v) for v in self.log.read_text().splitlines()]
        titles = [c[-1] for c in calls if c[0] == 'select-pane' and '-T' in c]
        self.assertEqual(titles[:3], ['ARM / JOINTS', 'HAND / GRIPPERS', 'RECORDING'])
        splits = [c for c in calls if c[0] == 'split-window']
        self.assertEqual([c[1] for c in splits], ['-h', '-h', '-v'])
        self.assertIn(['select-layout', '-t', 'isaac_control:main', 'even-horizontal'], calls)
        self.assertTrue(any(c[0] == 'new-window' and 'backend' in c for c in calls))
        commands = [c[-2] for c in calls if c[0] == 'send-keys']
        for node in ('arm_control_node', 'eff_control_node', 'sim_recorder_node', 'bfm_action_gateway_node'):
            self.assertEqual(sum(node in c for c in commands), 1)
        self.assertIn(['select-window', '-t', 'isaac_control:main'], calls)
        text = (ROOT/'start_isaac_control.sh').read_text()
        self.assertLess(text.index('PANE_GATEWAY='), text.index('start_worker "$PANE_GATEWAY"'))
        self.assertNotIn('tiled', text)

    def test_fixed_environment_ignores_caller_and_tmux_modes(self):
        self.env.update(BFM_COLLECTION_MODE='0', ARM_INPUT_TOPIC='/bad',
                        HAND_INPUT_TOPIC='/bad', BFM_OBJECT_PRIM_PATH='/bad')
        command = f'''set -euo pipefail
PROJECT_DIR={str(self.root)!r}
source "$PROJECT_DIR/tools/collection/environment.bash"
python3 -c 'import os,json; print(json.dumps(dict(os.environ)))'
'''
        result = subprocess.run(['bash', '-c', command], env=self.env, capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        env = json.loads(result.stdout)
        self.assertEqual(env['BFM_COLLECTION_MODE'], '1')
        self.assertEqual(env['ARM_INPUT_TOPIC'], '/robot/command/arm_joint_target')
        self.assertEqual(env['HAND_INPUT_TOPIC'], '/robot/command/hand_target')
        self.assertNotIn('BFM_OBJECT_PRIM_PATH', env)
        self.assertEqual(env['ROS_DOMAIN_ID'], '0')

    def test_isaac_no_prefix_and_missing_prerequisites(self):
        isaac = self.executable('isaac-sim.sh', '#!/bin/bash\nprintf "%s\\n" "$@"\n')
        launch = self.root/'launch_bfm_isaac.sh'
        # Substitute only the deployment location, keeping execution flow intact.
        launch.write_text(launch.read_text().replace('/isaac-sim/isaac-sim.sh', str(isaac)))
        result = self.run_script('launch_bfm_isaac.sh')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('--enable\nbfm_collection', result.stdout)
        self.setup.unlink()
        result = self.run_script('start_isaac_control.sh')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('workspace is not built', result.stderr)
        self.assertFalse(self.log.exists())
        isaac.unlink()
        self.assertIn('Isaac Sim not found', self.run_script('launch_bfm_isaac.sh').stderr)

    def test_existing_session_is_interrupted_and_preserved_if_not_finished(self):
        self.executable('sleep', '#!/bin/bash\nexit 0\n')
        self.executable('tmux', f'''#!{sys.executable}
import json, sys
with open({str(self.log)!r}, 'a') as stream: stream.write(json.dumps(sys.argv[1:])+'\\n')
if sys.argv[1] == 'list-panes':
    print('%1 0' if '#{{pane_id}} #{{pane_dead}}' in sys.argv else '0')
''')
        result = self.run_script('start_isaac_control.sh')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('session preserved', result.stderr)
        calls = [json.loads(v) for v in self.log.read_text().splitlines()]
        self.assertIn(['send-keys', '-t', '%1', 'C-c'], calls)
        self.assertFalse(any(c[0] in ('kill-session', 'new-session') for c in calls))

    def test_completed_session_is_replaced_after_interrupt(self):
        self.executable('tmux', f"""#!{sys.executable}
import json, sys
with open({str(self.log)!r}, 'a') as stream: stream.write(json.dumps(sys.argv[1:])+'\\n')
if sys.argv[1] == 'list-panes':
    print('%1 1' if '#{{pane_id}} #{{pane_dead}}' in sys.argv else '1')
else: print('%1')
""")
        result = self.run_script('start_isaac_control.sh')
        self.assertEqual(result.returncode, 0, result.stderr)
        calls = [json.loads(v) for v in self.log.read_text().splitlines()]
        names = [c[0] for c in calls]
        self.assertLess(names.index('list-panes'), names.index('kill-session'))
        self.assertLess(names.index('kill-session'), names.index('new-session'))

    def test_worker_failure_announces_error(self):
        self.executable('ros2', '#!/bin/bash\nexit 7\n')
        result = subprocess.run([sys.executable, str(ROOT/'tools/collection/worker.py'),
                                  'bfm_action_gateway_node'], env=self.env,
                                 text=True, capture_output=True)
        self.assertEqual(result.returncode, 1)
        self.assertIn('[GATEWAY] ERROR', result.stdout)
        self.assertIn('display-message', self.log.read_text())

    def test_shell_syntax(self):
        for name in ('launch_bfm_isaac.sh', 'start_isaac_control.sh',
                     'tools/collection/environment.bash', 'tools/collection/worker.bash'):
            subprocess.run(['bash', '-n', str(ROOT/name)], check=True)

    def test_worker_forwards_interrupt_and_waits_for_cleanup(self):
        ready, done = self.root/'ready', self.root/'done'
        self.executable('ros2', f'''#!{sys.executable}
import signal,time
from pathlib import Path
stop=False
def interrupt(*args):
    global stop
    stop=True
signal.signal(signal.SIGINT, interrupt)
Path({str(ready)!r}).touch()
while not stop: time.sleep(.01)
time.sleep(.05)
Path({str(done)!r}).touch()
''')
        process = subprocess.Popen([sys.executable, str(ROOT/'tools/collection/worker.py'),
                                    'sim_recorder_node'], env=self.env,
                                   stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        try:
            deadline = time.monotonic()+5
            while not ready.exists() and time.monotonic()<deadline: time.sleep(.01)
            self.assertTrue(ready.exists())
            process.send_signal(signal.SIGINT)
            output, error = process.communicate(timeout=5)
            self.assertEqual(process.returncode, 0, (output, error))
            self.assertTrue(done.exists())
        finally:
            if process.poll() is None: process.kill(); process.wait()


class StatusTests(unittest.TestCase):
    setUp = harness.RecordingPipelineTests.setUp

    def test_arm_and_hand_transitions_without_changing_outputs(self):
        for filename, cls, label in [('arm_control_node.py', 'ArmControlNode', 'ARM'),
                                     ('eff_control_node.py', 'IsaacEffControlNode', 'HAND')]:
            module = harness.load_module('status_'+label,
                'exo2isaacsim/src/isaac_eff_control/isaac_eff_control/'+filename)
            node = getattr(module, cls)()
            now = [0.]
            node.flow.clock = lambda: now[0]
            msg = harness.Message()
            if label == 'ARM':
                msg.name = sorted(module.ARM_JOINTS); msg.position = [0.]*14
                receive = node.arm_callback
            else:
                msg.left_hand_position = msg.right_hand_position = [0.]*6
                receive = node.hand_callback
            receive(msg); receive(msg)
            now[0] = 1.1; node.flow.check(); node.flow.check(); receive(msg)
            events = [c.args[0] for c in node.logger.info.call_args_list]
            self.assertEqual(events, [f'[{label}] {s}' for s in ('WAITING','ACTIVE','STOPPED','ACTIVE')])
            if label == 'ARM': msg.position[0] = float('nan')
            else: msg.left_hand_position = []
            receive(msg)
            node.logger.error.assert_called_with(f'[{label}] ERROR')

    def test_hand_status_uses_raw_not_held_input(self):
        with patch.dict(os.environ, BFM_COLLECTION_MODE='1'):
            module = harness.load_module('status_held_hand',
                'exo2isaacsim/src/isaac_eff_control/isaac_eff_control/eff_control_node.py')
        node = module.IsaacEffControlNode()
        msg = harness.Message(); msg.left_hand_position = msg.right_hand_position = [0.]*6
        node.hand_callback(msg)
        self.assertEqual(node.flow.state, 'WAITING')
        node.subscriptions['/robot/command/hand_target'].callback(msg)
        self.assertEqual(node.flow.state, 'ACTIVE')
        node.timer_callback()
        self.assertEqual(len(node.pub_left.publish.call_args.args[0].position), 16)

    def test_recording_status_is_finalized_and_error_never_stopped(self):
        node = self.recorder.SimRecorder()
        node.recording_callback(harness.Bool(True))
        self.assertEqual(node._rec_status, 'RECORDING')
        node.recording_callback(harness.Bool(False))
        self.assertEqual(node._rec_status, 'STOPPED')
        node.recording_callback(harness.Bool(True))
        self.process.wait.side_effect = [subprocess.TimeoutExpired('bag', 15), 0]
        node.recording_callback(harness.Bool(False))
        self.assertEqual(node._rec_status, 'ERROR')
        node.stop_recording()
        self.assertEqual(node._rec_status, 'ERROR')
