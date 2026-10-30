"""OmniGraph API contract/failure tests with a fake runtime; not live Isaac."""
import sys
from types import ModuleType, SimpleNamespace
from unittest import TestCase
from unittest.mock import Mock, patch
from pxr import Usd, Sdf

from test_recording_pipeline import ROOT
sys.path.insert(0,str(ROOT/'exo2isaacsim/src/isaac_eff_control'))
from isaac_eff_control.collection_graph import GRAPH_ROOT, configure_physics_clock


class ClockGraphTests(TestCase):
    def setUp(self):
        self.stage = Usd.Stage.CreateInMemory()
        self.old = self.stage.DefinePrim(GRAPH_ROOT+'/Data_collection_ActionGraph/ros2_publish_clock', 'OmniGraphNode')
        self.old.CreateAttribute('node:type',Sdf.ValueTypeNames.Token).Set('isaacsim.ros2.bridge.ROS2PublishClock')
        self.path = GRAPH_ROOT+'/BFM_Clock_ActionGraph'
        self.kind, self.mode = object(), object()
        self.graph = Mock()
        self.graph.is_valid.return_value = True
        self.graph.is_disabled.return_value = False
        self.graph.get_pipeline_stage.return_value = self.kind
        self.graph.get_evaluator_name.return_value = 'execution'
        self.attrs, self.links = {}, {}
        self.og = ModuleType('omni.graph.core')
        self.og.GraphPipelineStage = SimpleNamespace(GRAPH_PIPELINE_STAGE_ONDEMAND=self.kind)
        self.og.GraphEvaluationMode = SimpleNamespace(GRAPH_EVALUATION_MODE_AUTOMATIC=self.mode)
        self.og.get_graph_by_path = Mock(return_value=None)
        self.og.GraphController = SimpleNamespace(
            create_graph=Mock(return_value=self.graph),
            create_node=Mock(side_effect=self.create_node),
            connect=Mock(side_effect=self.connect))
        self.og.Controller = SimpleNamespace(attribute=self.attribute, set=Mock())
        omni, graph = ModuleType('omni'), ModuleType('omni.graph')
        omni.graph=graph; graph.core=self.og
        mock = patch.dict(sys.modules, {'omni':omni,'omni.graph':graph,'omni.graph.core':self.og})
        mock.start(); self.addCleanup(mock.stop)

    def create_node(self, path, kind, **kwargs):
        prim = self.stage.DefinePrim(path, 'OmniGraphNode')
        prim.CreateAttribute('node:type', Sdf.ValueTypeNames.Token).Set(kind)
        return SimpleNamespace(is_valid=lambda:True)

    def attribute(self, path):
        if path not in self.attrs:
            self.attrs[path] = SimpleNamespace(is_valid=lambda:True, get_path=lambda:path,
                is_connected=lambda other:path in self.links.get(other.get_path(),[]),
                get_upstream_connections=lambda:[self.attribute(p) for p in self.links.get(path,[])])
        return self.attrs[path]

    def connect(self, source, target, **kwargs):
        self.assertTrue(self.old.IsActive())  # Switch only after connections verified.
        self.links[target.get_path()] = [source.get_path()]

    def test_create_runtime_pipeline_and_connections_before_old_clock_disable(self):
        root_before=self.stage.GetRootLayer().ExportToString()
        graph=configure_physics_clock(self.stage)
        self.assertIs(graph,self.graph)
        call=self.og.GraphController.create_graph.call_args
        self.assertEqual(call.args[0],dict(graph_path=self.path,evaluator_name='execution',
                                         pipeline_stage=self.kind,evaluation_mode=self.mode))
        self.assertEqual(call.kwargs,dict(update_usd=True,undoable=False))
        self.assertGreaterEqual(graph.get_pipeline_stage.call_count,2)
        self.assertIs(graph.evaluation_mode,self.mode)
        self.assertEqual(self.og.GraphController.create_node.call_count,4)
        self.assertEqual(len(self.links),3)
        self.assertFalse(self.old.IsActive())
        self.assertEqual(self.stage.GetRootLayer().ExportToString(),root_before)
        self.assertEqual(graph.set_disabled.call_args_list[-1].args,(False,))

    def test_existing_runtime_graph_changes_pipeline_via_api(self):
        self.og.get_graph_by_path.return_value=self.graph
        self.graph.get_pipeline_stage.side_effect=[object(),self.kind,self.kind]
        configure_physics_clock(self.stage)
        self.graph.change_pipeline_stage.assert_called_once_with(self.kind)
        self.og.GraphController.create_graph.assert_not_called()

    def test_wrong_runtime_pipeline_fails_with_old_clock_preserved(self):
        self.graph.get_pipeline_stage.return_value=object()
        with self.assertRaisesRegex(RuntimeError,'not On-Demand'):
            configure_physics_clock(self.stage)
        self.assertTrue(self.old.IsActive())
        self.og.GraphController.create_node.assert_not_called()
        self.assertEqual(self.graph.set_disabled.call_args_list[-1].args,(True,))

    def test_failed_creation_nodes_and_connections_preserve_old_clock(self):
        for operation in ('create_graph','create_node','connect'):
            with self.subTest(operation=operation):
                method=getattr(self.og.GraphController,operation)
                previous=method.side_effect
                method.side_effect=RuntimeError('injected failure')
                try:
                    with self.assertRaisesRegex(RuntimeError,'setup failed'):
                        configure_physics_clock(self.stage)
                    self.assertTrue(self.old.IsActive())
                finally:
                    method.side_effect=previous

    def test_unverified_connection_is_rejected(self):
        self.og.GraphController.connect.side_effect=None
        with self.assertRaisesRegex(RuntimeError,'Invalid runtime clock connection'):
            configure_physics_clock(self.stage)
        self.assertTrue(self.old.IsActive())

    def test_disabled_new_graph_and_duplicate_publisher_fail_closed(self):
        self.graph.is_disabled.return_value=True
        with self.assertRaisesRegex(RuntimeError,'remains disabled'):
            configure_physics_clock(self.stage)
        self.assertTrue(self.old.IsActive())
        self.graph.is_disabled.return_value=False
        duplicate=self.stage.DefinePrim('/ExtraClock','OmniGraphNode')
        duplicate.CreateAttribute('node:type',Sdf.ValueTypeNames.Token).Set('isaacsim.ros2.bridge.ROS2PublishClock')
        with self.assertRaisesRegex(RuntimeError,'exactly one'):
            configure_physics_clock(self.stage)
        self.assertTrue(self.old.IsActive())
