"""Only frame identities and the simulation-clock trigger, before PLAY."""
import json
from pathlib import Path

CAMERA_FRAMES = {name: f'{name}_camera' for name in ('head', 'left_wrist', 'right_wrist')}
GRAPH_ROOT = '/World/s40_sharpa_two_cameras'


def configure_collection_graph(stage):
    from pxr import Sdf, Usd
    cameras = json.loads(Path(__file__).with_name('camera_metadata.json').read_text())['cameras']
    helpers = {}
    for prim in Usd.PrimRange(stage.GetPrimAtPath(GRAPH_ROOT + '/Control_ActionGraph')):
        if prim.GetAttribute('node:type').Get() == 'isaacsim.ros2.bridge.ROS2CameraHelper':
            topic = prim.GetAttribute('inputs:topicName').Get()
            if topic in helpers:
                raise ValueError(f'Duplicate camera helper for {topic}')
            helpers[topic] = prim
    if set(helpers) != {c['topic'] for c in cameras.values()}:
        raise ValueError('Expected the three configured camera publishers')
    graph = GRAPH_ROOT + '/Data_collection_ActionGraph'
    clock = stage.GetPrimAtPath(graph + '/ros2_publish_clock')
    if clock.GetAttribute('node:type').Get() != 'isaacsim.ros2.bridge.ROS2PublishClock':
        raise ValueError('Expected the existing /clock publisher')
    # Session opinions avoid rewriting binary scenes or robot/reference assets.
    # Only /clock changes trigger; joint-state and camera triggers stay intact.
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        for name, camera in cameras.items():
            helpers[camera['topic']].CreateAttribute('inputs:frameId', Sdf.ValueTypeNames.String).Set(CAMERA_FRAMES[name])
        step = stage.DefinePrim(graph + '/bfm_clock_physics_step', 'OmniGraphNode')
        step.CreateAttribute('node:type', Sdf.ValueTypeNames.Token, custom=False).Set('isaacsim.core.nodes.OnPhysicsStep')
        step.CreateAttribute('node:typeVersion', Sdf.ValueTypeNames.Int, custom=False).Set(1)
        output = step.CreateAttribute('outputs:step', Sdf.ValueTypeNames.UInt)
        clock.GetAttribute('inputs:execIn').SetConnections([output.GetPath()])
