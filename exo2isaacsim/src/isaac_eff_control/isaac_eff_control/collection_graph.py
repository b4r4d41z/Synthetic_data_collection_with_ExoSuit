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
        configure_physics_clock(stage)


def configure_physics_clock(stage):
    """Use OmniGraph 1.142 runtime pipeline APIs, never evaluationMode tokens."""
    import omni.graph.core as og
    from pxr import Usd
    path = GRAPH_ROOT + '/BFM_Clock_ActionGraph'
    old_path = GRAPH_ROOT + '/Data_collection_ActionGraph'
    old_clock = stage.GetPrimAtPath(old_path + '/ros2_publish_clock')
    if not old_clock:
        raise RuntimeError('Missing existing /clock publisher; refusing clock switch')
    misplaced = stage.GetPrimAtPath(old_path+'/bfm_clock_physics_step')
    misplaced_active = misplaced.IsActive() if misplaced else None
    stage_kind = og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND
    mode = og.GraphEvaluationMode.GRAPH_EVALUATION_MODE_AUTOMATIC
    graph = og.get_graph_by_path(path)
    old_active = old_clock.IsActive()
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        try:
            if graph is None or not graph.is_valid():
                graph = og.GraphController.create_graph(
                    dict(graph_path=path, evaluator_name='execution',
                         pipeline_stage=stage_kind, evaluation_mode=mode),
                    update_usd=True, undoable=False)
            if graph is None or not graph.is_valid():
                raise RuntimeError('Clock graph creation returned an invalid runtime graph')
            graph.set_disabled(True)  # Build while STOPPED; no duplicate publisher.
            if graph.get_pipeline_stage() != stage_kind:
                graph.change_pipeline_stage(stage_kind)
            graph.evaluation_mode = mode
            if graph.get_pipeline_stage() != stage_kind or graph.get_evaluator_name() != 'execution':
                raise RuntimeError('Clock runtime graph is not On-Demand/execution')
            types = {
                'physics_step': 'isaacsim.core.nodes.OnPhysicsStep',
                'ros2_context': 'isaacsim.ros2.bridge.ROS2Context',
                'isaac_read_simulation_time': 'isaacsim.core.nodes.IsaacReadSimulationTime',
                'ros2_publish_clock': 'isaacsim.ros2.bridge.ROS2PublishClock',
            }
            for name, node_type in types.items():
                node = og.GraphController.create_node(path+'/'+name, node_type,
                    allow_exists=True, update_usd=True, undoable=False)
                if not node.is_valid():
                    raise RuntimeError(f'Invalid clock node: {name}')
                # Retain configured context/QoS/time-reader inputs, not live outputs.
                source = stage.GetPrimAtPath(old_path+'/'+name)
                if source:
                    for attr in source.GetAttributes():
                        if attr.GetName().startswith('inputs:') and not attr.GetConnections() and attr.Get() is not None:
                            og.Controller.set(og.Controller.attribute(path+'/'+name+'.'+attr.GetName()), attr.Get())
            og.Controller.set(og.Controller.attribute(path+'/ros2_publish_clock.inputs:topicName'), '/clock')
            links = [
                ('physics_step.outputs:step', 'ros2_publish_clock.inputs:execIn'),
                ('isaac_read_simulation_time.outputs:simulationTime', 'ros2_publish_clock.inputs:timeStamp'),
                ('ros2_context.outputs:context', 'ros2_publish_clock.inputs:context'),
            ]
            for src, dst in links:
                source = og.Controller.attribute(path+'/'+src)
                target = og.Controller.attribute(path+'/'+dst)
                if not source.is_valid() or not target.is_valid():
                    raise RuntimeError(f'Missing clock port: {src} -> {dst}')
                if not source.is_connected(target):
                    og.GraphController.connect(source, target, update_usd=True, undoable=False)
                if [str(a.get_path()) for a in target.get_upstream_connections()] != [str(source.get_path())]:
                    raise RuntimeError(f'Invalid runtime clock connection: {dst}')
            if graph.get_pipeline_stage() != stage_kind:
                raise RuntimeError('Clock pipeline changed during construction')
            # Only now retire the old publisher. Never change the main graph.
            old_clock.SetActive(False)
            if misplaced:
                misplaced.SetActive(False)
            graph.set_disabled(False)
            if graph.is_disabled():
                raise RuntimeError('Dedicated clock graph remains disabled')
            publishers = [p for p in stage.Traverse()
                          if p.GetAttribute('node:type').Get() == types['ros2_publish_clock']
                          and p.GetAttribute('inputs:topicName').Get() in (None, '', '/clock')]
            if len(publishers) != 1 or str(publishers[0].GetPath()) != path+'/ros2_publish_clock':
                raise RuntimeError('Expected exactly one enabled /clock publisher')
        except Exception as error:
            if graph is not None and graph.is_valid():
                graph.set_disabled(True)
            old_clock.SetActive(old_active)
            if misplaced:
                misplaced.SetActive(misplaced_active)
            raise RuntimeError(f'BFM physics clock setup failed: {error}') from error
    print('[BFM CLOCK] runtime pipeline verified ONDEMAND; /clock physics publisher ready', flush=True)
    return graph
