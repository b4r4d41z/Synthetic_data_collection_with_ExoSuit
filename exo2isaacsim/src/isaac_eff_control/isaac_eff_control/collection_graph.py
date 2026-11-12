"""Transient camera cadence/identities and the physics clock, before PLAY."""
import json
from pathlib import Path

CAMERA_FRAMES = {name: f'{name}_camera' for name in ('head', 'left_wrist', 'right_wrist')}
GRAPH_ROOT = '/World/s40_sharpa_two_cameras'


CAMERA_RATE_HZ = 15.0


def _camera_bindings(stage):
    """Validate the saved creation recipe; render products are allocated at PLAY."""
    from pxr import Sdf, UsdGeom, UsdRender
    cameras = json.loads(Path(__file__).with_name('camera_metadata.json').read_text())['cameras']
    expected = {f'/sensors/camera/{name}/image_raw' for name in CAMERA_FRAMES}
    helpers = {}
    for prim in stage.Traverse():
        if prim.GetAttribute('node:type').Get() == 'isaacsim.ros2.bridge.ROS2CameraHelper':
            topic = prim.GetAttribute('inputs:topicName').Get()
            if topic in helpers:
                raise ValueError(f'[BFM CAMERA] Duplicate camera helper for {topic}')
            helpers[topic] = prim
    if set(cameras) != set(CAMERA_FRAMES) or set(helpers) != expected:
        raise ValueError(f'[BFM CAMERA] Expected three camera helpers {sorted(expected)}; got {list(helpers)}')
    bindings = {}
    for name, info in cameras.items():
        topic = f'/sensors/camera/{name}/image_raw'
        if info['topic'] != topic or info['resolution'] != [640, 480]:
            raise ValueError(f'[BFM CAMERA] {name}: unexpected topic/resolution metadata')
        helper = helpers[topic]
        camera_path = Sdf.Path(info['usd_camera_prim']).ReplacePrefix(
            Sdf.Path('/biped_s40_with_sharpa_wave_hands'), Sdf.Path(GRAPH_ROOT))
        camera = stage.GetPrimAtPath(camera_path)
        if not camera or not camera.IsActive() or not camera.IsA(UsdGeom.Camera):
            raise ValueError(f'[BFM CAMERA] {name}: missing Camera {camera_path}')
        source = helper.GetAttribute('inputs:renderProductPath').GetConnections()
        if len(source) != 1 or source[0].name != 'outputs:renderProductPath':
            raise ValueError(f'[BFM CAMERA] {name}: missing/invalid render-product connection')
        creator = stage.GetPrimAtPath(source[0].GetPrimPath())
        if (not creator or not creator.IsActive() or
                creator.GetAttribute('node:type').Get() != 'isaacsim.core.nodes.IsaacCreateRenderProduct'):
            raise ValueError(f'[BFM CAMERA] {name}: invalid render-product creator {source[0]}')
        targets = creator.GetRelationship('inputs:cameraPrim').GetForwardedTargets()
        if targets != [camera_path]:
            raise ValueError(f'[BFM CAMERA] {name}: creator camera {targets} != {camera_path}')
        if helper.GetAttribute('inputs:execIn').GetConnections() != [creator.GetPath().AppendProperty('outputs:execOut')]:
            raise ValueError(f'[BFM CAMERA] {name}: helper execIn must follow its render-product creator')
        trigger = creator.GetAttribute('inputs:execIn').GetConnections()
        step = stage.GetPrimAtPath(trigger[0].GetPrimPath()) if len(trigger) == 1 else None
        if (not step or not step.IsActive() or trigger[0].name != 'outputs:step' or
                step.GetAttribute('node:type').Get() != 'isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame'):
            raise ValueError(f'[BFM CAMERA] {name}: invalid render-product initialization trigger')
        ticks = step.GetAttribute('inputs:execIn').GetConnections()
        playback = stage.GetPrimAtPath(ticks[0].GetPrimPath()) if len(ticks) == 1 else None
        if (not playback or not playback.IsActive() or ticks[0].name != 'outputs:tick' or
                playback.GetAttribute('node:type').Get() != 'omni.graph.action.OnPlaybackTick'):
            raise ValueError(f'[BFM CAMERA] {name}: missing playback initialization path')
        for prim, fields in ((helper, ('topicName', 'type', 'enabled', 'frameSkipCount', 'frameId')),
                             (creator, ('width', 'height', 'enabled'))):
            if any(prim.GetAttribute('inputs:'+field).GetConnections() for field in fields):
                raise ValueError(f'[BFM CAMERA] {name}: dynamic publisher settings are unsupported')
            if prim.GetAttribute('inputs:enabled').Get() is False:
                raise ValueError(f'[BFM CAMERA] {name}: disabled publisher/render product')
        # Unauthored values use Isaac's rgb / zero-skip defaults.
        if helper.GetAttribute('inputs:type').Get() not in (None, 'rgb'):
            raise ValueError(f'[BFM CAMERA] {name}: expected RGB helper')
        if helper.GetAttribute('inputs:frameSkipCount').Get() not in (None, 0):
            raise ValueError(f'[BFM CAMERA] {name}: remove deprecated nonzero frameSkipCount; use sensor tickRate')
        resolution = [creator.GetAttribute('inputs:'+key).Get() for key in ('width', 'height')]
        if resolution != info['resolution']:
            raise ValueError(f'[BFM CAMERA] {name}: render-product resolution {resolution} != 640x480')
        # An empty target is normal: IsaacCreateRenderProduct creates it at PLAY.
        # If an existing product is supplied, validate its real camera relationship.
        products = creator.GetRelationship('inputs:renderProductPrim').GetForwardedTargets()
        if len(products) > 1:
            raise ValueError(f'[BFM CAMERA] {name}: multiple render-product targets')
        if products:
            product = UsdRender.Product(stage.GetPrimAtPath(products[0]))
            if (not product or product.GetCameraRel().GetForwardedTargets() != [camera_path]
                    or list(product.GetResolutionAttr().Get()) != info['resolution']):
                raise ValueError(f'[BFM CAMERA] {name}: invalid render product {products[0]}')
        bindings[name] = (camera, creator, helper)
    return bindings


def _set_camera_rate(camera):
    """Isaac 6.x generic USD API (no camera wrapper or geometry writes)."""
    from pxr import Sdf, Usd
    if not Usd.SchemaRegistry().FindAppliedAPIPrimDefinition('OmniSensorAPI'):
        raise RuntimeError('[BFM CAMERA] OmniSensorAPI unavailable; load the Isaac Sim 6.x sensor schema')
    if not camera.ApplyAPI('OmniSensorAPI'):
        raise RuntimeError(f'[BFM CAMERA] Cannot apply OmniSensorAPI to {camera.GetPath()}')
    camera.CreateAttribute('omni:sensor:tickRate', Sdf.ValueTypeNames.Float, custom=False).Set(CAMERA_RATE_HZ)


def validate_camera_publishers(stage):
    """Read-only check of the graph recipe and effective session configuration."""
    bindings = _camera_bindings(stage)
    for name, (camera, _, helper) in bindings.items():
        schemas = camera.GetMetadata('apiSchemas')
        rate = camera.GetAttribute('omni:sensor:tickRate')
        if (not schemas or 'OmniSensorAPI' not in schemas.GetAppliedItems()
                or not rate or rate.Get() != CAMERA_RATE_HZ or rate.ValueMightBeTimeVarying()
                or rate.GetConnections()):
            raise ValueError(f'[BFM CAMERA] {name}: expected OmniSensorAPI tickRate={CAMERA_RATE_HZ} Hz')
        if helper.GetAttribute('inputs:frameId').Get() != CAMERA_FRAMES[name]:
            raise ValueError(f'[BFM CAMERA] {name}: unexpected frame identity')
    return bindings


def configure_camera_publishers(stage):
    """Request equal sensor/render cadence, not frame skipping or duplicated images.

    Isaac 6.x multi-tick rendering uses physics simulation time. A requested
    15 Hz cannot guarantee 15 wall Hz or deadlines under low RTF/GPU overload.
    """
    from pxr import Sdf, Usd
    bindings = _camera_bindings(stage)  # Validate every mapping before authoring.
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        for name, (camera, _, helper) in bindings.items():
            _set_camera_rate(camera)
            helper.CreateAttribute('inputs:frameId', Sdf.ValueTypeNames.String).Set(CAMERA_FRAMES[name])
    validate_camera_publishers(stage)
    print(f'[BFM CAMERA] three cameras validated (head, left_wrist, right_wrist); '
          f'target={CAMERA_RATE_HZ:g} Hz simulation time; runtime/session configuration active', flush=True)


def configure_collection_graph(stage):
    graph = GRAPH_ROOT + '/Data_collection_ActionGraph'
    clock = stage.GetPrimAtPath(graph + '/ros2_publish_clock')
    if clock.GetAttribute('node:type').Get() != 'isaacsim.ros2.bridge.ROS2PublishClock':
        raise ValueError('Expected the existing /clock publisher')
    configure_physics_timestep(stage)
    configure_camera_publishers(stage)
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


def configure_physics_timestep(stage):
    """Existing PhysxSchema timestep attribute, session-only before PLAY."""
    from .bfm_contract import PHYSICS_HZ, validated_physics_hz
    requested_hz = validated_physics_hz(PHYSICS_HZ)
    from pxr import Usd, UsdPhysics, PhysxSchema
    scenes = [p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)]
    if len(scenes) != 1:
        raise RuntimeError('BFM v4 requires one existing PhysicsScene')
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        api = PhysxSchema.PhysxSceneAPI.Apply(scenes[0])
        try:
            if not api or not api.CreateTimeStepsPerSecondAttr().Set(requested_hz):
                raise RuntimeError('PhysxSceneAPI rejected the requested value')
        except Exception as error:
            raise RuntimeError(f'BFM physics frequency {requested_hz} Hz could not be applied: {error}') from error
    effective_hz = api.GetTimeStepsPerSecondAttr().Get()
    if effective_hz != requested_hz or api.GetTimeStepsPerSecondAttr().ValueMightBeTimeVarying():
        raise RuntimeError(f'BFM physics frequency override failed: requested={requested_hz}, effective={effective_hz}')
    print(f'[BFM PHYSICS] {scenes[0].GetPath()}: requested/effective={effective_hz} Hz; '
          'runtime/session configuration active (not wall FPS)', flush=True)
