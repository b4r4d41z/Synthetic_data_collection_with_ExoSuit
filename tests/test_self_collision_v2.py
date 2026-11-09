"""Composed USD regression checks; not physical contact/solver validation."""
import math
from pathlib import Path
import unittest

from pxr import Usd, UsdGeom, UsdPhysics

ROOT = Path(__file__).resolve().parents[1]
ROBOT = '/World/s40_sharpa_two_cameras'
ARTICULATION = ROBOT + '/fix_base_joint'
V1 = ROOT/'models/stages/sorting_v1/stage_sorting_v1.usd'
V2 = ROOT/'models/stages/sorting_experimental/stage_sorting_v2.usda'


def robot_prims(stage):
    return list(Usd.PrimRange(stage.GetPrimAtPath(ROBOT), Usd.TraverseInstanceProxies()))


def authored_snapshot(stage):
    changed = {'newton:selfCollisionEnabled', 'physxArticulation:enabledSelfCollisions'}
    return {str(p.GetPath()): (
        p.GetTypeName(), p.IsActive(), p.IsInstanceable(),
        {a.GetName(): (a.Get(), tuple(a.GetConnections())) for a in p.GetAttributes()
         if a.HasAuthoredValueOpinion() and not (str(p.GetPath()) == ARTICULATION and a.GetName() in changed)},
        {r.GetName(): tuple(r.GetTargets()) for r in p.GetRelationships()})
        for p in stage.Traverse(Usd.TraverseInstanceProxies())}


class SelfCollisionV2Tests(unittest.TestCase):
    def setUp(self):
        self.v1 = Usd.Stage.Open(str(V1))
        self.v2 = Usd.Stage.Open(str(V2))
        # V1 has no saved selection; evaluate the intended PhysX variant without
        # changing its root layer or dependencies.
        with Usd.EditContext(self.v1, self.v1.GetSessionLayer()):
            self.v1.GetPrimAtPath(ROBOT).GetVariantSet('Physics').SetVariantSelection('physx')

    def test_v2_selects_physx_and_enables_the_existing_fixed_base_root(self):
        robot = self.v2.GetPrimAtPath(ROBOT)
        self.assertEqual(robot.GetVariantSet('Physics').GetVariantSelection(), 'physx')
        roots = [p for p in robot_prims(self.v2) if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
        self.assertEqual([str(p.GetPath()) for p in roots], [ARTICULATION])
        root = roots[0]
        self.assertIn('PhysxArticulationAPI', root.GetMetadata('apiSchemas').GetAppliedItems())
        self.assertTrue(root.GetAttribute('physxArticulation:enabledSelfCollisions').Get())
        self.assertTrue(root.GetAttribute('newton:selfCollisionEnabled').Get())
        self.assertFalse(self.v1.GetPrimAtPath(ARTICULATION).GetAttribute('newton:selfCollisionEnabled').Get())
        self.assertEqual(root.GetAttribute('physxArticulation:enabledSelfCollisions').GetPropertyStack()[0].layer,
                         self.v2.GetRootLayer())
        self.assertEqual(self.v2.GetDefaultPrim().GetPath(), '/World')
        self.assertEqual(UsdGeom.GetStageMetersPerUnit(self.v2), 1.)

    def test_existing_geometry_joints_drives_cameras_graphs_and_topics_are_identical(self):
        self.assertEqual(authored_snapshot(self.v1), authored_snapshot(self.v2))
        for name in ('timeCodesPerSecond', 'upAxis', 'metersPerUnit'):
            self.assertEqual(self.v1.GetMetadata(name), self.v2.GetMetadata(name))
        self.assertEqual(len([p for p in robot_prims(self.v2) if p.HasAPI(UsdPhysics.RigidBodyAPI)]), 81)
        self.assertEqual(len([p for p in robot_prims(self.v2) if p.IsA(UsdPhysics.Joint)]), 81)
        self.assertEqual(len([p for p in self.v2.Traverse() if p.IsA(UsdPhysics.Scene)]), 1)

    def test_enabled_colliders_have_valid_geometry_and_body_owners(self):
        colliders = [p for p in robot_prims(self.v2) if p.HasAPI(UsdPhysics.CollisionAPI)]
        self.assertEqual(len(colliders), 127)
        for prim in colliders:
            with self.subTest(path=str(prim.GetPath())):
                self.assertTrue(UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get())
                owner = prim
                while owner and not owner.HasAPI(UsdPhysics.RigidBodyAPI):
                    owner = owner.GetParent()
                self.assertTrue(owner)
                self.assertTrue(UsdPhysics.RigidBodyAPI(owner).GetRigidBodyEnabledAttr().Get())
                self.assertFalse(UsdPhysics.RigidBodyAPI(owner).GetKinematicEnabledAttr().Get())
                if prim.IsA(UsdGeom.Mesh):
                    mesh = UsdGeom.Mesh(prim)
                    points = mesh.GetPointsAttr().Get()
                    indices = mesh.GetFaceVertexIndicesAttr().Get()
                    self.assertTrue(points and indices)
                    self.assertEqual(sum(mesh.GetFaceVertexCountsAttr().Get()), len(indices))
                    self.assertTrue(0 <= min(indices) <= max(indices) < len(points))
                    self.assertTrue(all(math.isfinite(v) for p in points for v in p))
                    self.assertEqual(UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get(), 'convexHull')

    def test_filters_preserve_same_hand_mechanics_without_cross_limb_exclusions(self):
        pairs = [(p.GetPath(), t) for p in robot_prims(self.v2)
                 for t in p.GetRelationship('physics:filteredPairs').GetTargets()]
        self.assertEqual(len(pairs), 52)
        for a, b in pairs:
            self.assertTrue(('left_' in str(a) and 'left_' in str(b)) or
                            ('right_' in str(a) and 'right_' in str(b)))
            self.assertTrue(self.v2.GetPrimAtPath(b))
        self.assertFalse([p for p in self.v2.Traverse() if p.IsA(UsdPhysics.CollisionGroup)])
        # Check the real joint graph, not the USD nesting, for connectivity.
        adjacency = {}
        for p in robot_prims(self.v2):
            if not p.IsA(UsdPhysics.Joint): continue
            a, b = (p.GetRelationship('physics:body'+str(i)).GetTargets() for i in (0, 1))
            if a and b:
                adjacency.setdefault(a[0], set()).add(b[0])
                adjacency.setdefault(b[0], set()).add(a[0])
        seen, pending = set(), [UsdPhysics.Joint(self.v2.GetPrimAtPath(ARTICULATION)).GetBody1Rel().GetTargets()[0]]
        while pending:
            body = pending.pop()
            if body not in seen:
                seen.add(body); pending.extend(adjacency.get(body, set()) - seen)
        self.assertEqual(len(seen), 81)
