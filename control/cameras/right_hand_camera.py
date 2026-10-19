r"""Prepare a persistent right-hand USD Camera once after MJCF import.

Import s40_sharpa_two_cameras.xml, stop the timeline, and save the imported
stage to a writable USD file. In Isaac Sim 6.1's Script Editor, with the
repository root on sys.path, run::

    from control.cameras.right_hand_camera import prepare_right_hand_camera
    camera_path = prepare_right_hand_camera()  # edits and saves the root layer

For an unsaved stage, pass save=False and then use File > Save As. To select
one robot in a multi-robot stage, pass robot_prim_path="/World/my_robot".
Pass xml_path explicitly if the imported robot came from a different XML.

The returned path ends in right_hand_camera_head/right_hand_camera. It is
saved as a real UsdGeom.Camera and remains available after reopening USD;
this preparation does not need to run on Play. It creates no render product,
sensor runtime, ROS publisher, or Action Graph nodes.

Later, select this path in Isaac Create Render Product and set width=640,
height=480. Configure rendering/sensor scheduling for 15 Hz separately:
resolution and update rate are not UsdGeom.Camera properties. For example,
render at 15 Hz, or at 60 Hz publish every fourth rendered frame using ROS2
Camera Helper frameSkipCount=3. Frame skipping controls publication rate;
it does not reduce the renderer's capture rate.

The same preparation can run with Isaac Sim's python.sh without starting Kit::

    /path/to/isaac-sim/python.sh control/cameras/right_hand_camera.py \
        --stage /absolute/path/to/imported_robot.usd
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import xml.etree.ElementTree as ET


CAMERA_NAME = "right_hand_camera"
WRIST_NAME = "zarm_r7_link"
HEAD_NAME = "right_hand_camera_head"
RENDER_RESOLUTION = (640, 480)  # Render product (width, height); no product is created here.
TARGET_RATE_HZ = 15.0  # Configure the later renderer/sensor or graph scheduling separately.
DEFAULT_XML = Path(__file__).resolve().parents[2] / "models/robots/s40_sharpa/assembly/s40_sharpa_two_cameras.xml"


@dataclass(frozen=True)
class CameraMount:
    body_names: tuple[str, ...]
    position: tuple[float, ...]
    quaternion: tuple[float, ...]  # MJCF/USD wxyz; camera looks along local -Z.
    fovy: float


def read_camera_mount(xml_path: str | Path = DEFAULT_XML) -> CameraMount:
    """Read the nested camera's local calibration without changing the XML."""
    root = ET.parse(xml_path).getroot()
    cameras = root.findall(f".//camera[@name='{CAMERA_NAME}']")
    if len(cameras) != 1:
        raise ValueError(f"Expected exactly one {CAMERA_NAME} in {xml_path}")
    camera = cameras[0]
    parents = {child: parent for parent in root.iter() for child in parent}
    parent = parents[camera]
    if parent.tag != "body" or parent.get("name") != HEAD_NAME:
        raise ValueError(f"{CAMERA_NAME} must be directly inside {HEAD_NAME}")
    chain = []
    while parent.tag == "body":
        chain.append(parent.attrib["name"])
        if parent.get("name") == WRIST_NAME:
            break
        parent = parents[parent]
    if chain[-1] != WRIST_NAME:
        raise ValueError(f"{HEAD_NAME} must be a descendant of {WRIST_NAME}")
    if any(key in camera.attrib for key in ("euler", "axisangle", "xyaxes", "zaxis")):
        raise ValueError("Camera calibration must use an explicit MJCF wxyz quat")
    if camera.get("mode", "fixed") != "fixed":
        raise ValueError("The hand camera must use fixed mode to follow its parent")
    mount = CameraMount(
        tuple(reversed(chain)),
        tuple(float(value) for value in camera.get("pos", "0 0 0").split()),
        tuple(float(value) for value in camera.attrib["quat"].split()),
        float(camera.attrib["fovy"]),
    )
    if len(mount.position) != 3 or len(mount.quaternion) != 4:
        raise ValueError("Camera pos must have three components and quat must have four (wxyz)")
    if not all(math.isfinite(value) for value in (*mount.position, *mount.quaternion, mount.fovy)):
        raise ValueError("Camera calibration contains a non-finite value")
    if not math.isclose(sum(value * value for value in mount.quaternion), 1.0, abs_tol=1e-6):
        raise ValueError("Camera quaternion must have unit length")
    if not 0.0 < mount.fovy < 180.0:
        raise ValueError("Camera fovy must be between 0 and 180 degrees")
    return mount


def _find_head(stage, mount: CameraMount, robot_prim_path: str | None):
    from pxr import Usd, UsdGeom

    root = stage.GetPrimAtPath(robot_prim_path) if robot_prim_path else stage.GetPseudoRoot()
    if not root.IsValid():
        raise ValueError(f"Robot prim does not exist: {robot_prim_path}")
    matches = []
    for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
        if prim.GetName() != mount.body_names[-1]:
            continue
        ancestor, index = prim, len(mount.body_names) - 1
        while ancestor.IsValid() and index >= 0:
            if ancestor.GetName() == mount.body_names[index]:
                index -= 1
            ancestor = ancestor.GetParent()
        if index < 0:
            matches.append(prim)
    if len(matches) != 1:
        raise ValueError(
            f"Expected one imported {'/'.join(mount.body_names)}; found "
            f"{[str(prim.GetPath()) for prim in matches]}. Check the import or set robot_prim_path."
        )
    head = matches[0]
    if not UsdGeom.Xformable(head) or head.IsInstance() or head.IsInstanceProxy():
        raise ValueError(f"Camera head must be an editable, non-instanced transform: {head.GetPath()}")
    return head


def _check_camera_slot(stage, path) -> None:
    """Allow an existing Camera or a harmless leaf placeholder; reject occupied paths."""
    from pxr import UsdGeom, UsdPhysics

    prim = stage.GetPrimAtPath(path)
    session_layers = set(stage.GetLayerStack()) - set(stage.GetLayerStack(includeSessionLayers=False))
    if prim.IsValid() and any(spec.layer in session_layers for spec in prim.GetPrimStack()):
        raise ValueError(f"Reopen the saved stage or remove session opinions at {path} before preparation")
    if not prim.IsValid():
        return
    if not prim.IsActive() or prim.IsInstance() or prim.IsInstanceProxy() or prim.HasAPI(UsdPhysics.RigidBodyAPI):
        raise ValueError(f"Camera path is inactive or occupied by an instance or rigid body: {path}")
    if prim.IsA(UsdGeom.Camera):
        return
    harmless_properties = all(
        prop.GetName().startswith("xformOp:") or prop.GetName() in {"xformOpOrder", "visibility", "purpose"}
        for prop in prim.GetAuthoredProperties()
    )
    if (
        prim.GetTypeName() not in {"", "Xform", "Scope"}
        or prim.GetChildren() or prim.GetAppliedSchemas() or not harmless_properties
        or prim.HasAuthoredReferences() or prim.HasAuthoredPayloads()
    ):
        raise ValueError(f"Cannot safely promote occupied camera path to UsdGeom.Camera: {path}")
    # A verified empty placeholder is retyped at the same path, preserving its identity.


def _configure_optics(camera, mount: CameraMount, meters_per_unit: float) -> None:
    from pxr import Gf, UsdGeom

    # USD focal length/apertures use tenths of a stage unit. Their ratio sets FOV.
    focal_length = 0.024 / (0.1 * meters_per_unit)
    vertical_aperture = 2.0 * focal_length * math.tan(math.radians(mount.fovy) / 2.0)
    camera.CreateProjectionAttr().Set(UsdGeom.Tokens.perspective)
    camera.CreateFocalLengthAttr().Set(focal_length)
    camera.CreateVerticalApertureAttr().Set(vertical_aperture)
    camera.CreateHorizontalApertureAttr().Set(vertical_aperture * RENDER_RESOLUTION[0] / RENDER_RESOLUTION[1])
    camera.CreateHorizontalApertureOffsetAttr().Set(0.0)
    camera.CreateVerticalApertureOffsetAttr().Set(0.0)
    camera.CreateClippingRangeAttr().Set(Gf.Vec2f(0.005 / meters_per_unit, 100.0 / meters_per_unit))
    camera.CreateFStopAttr().Set(0.0)


def prepare_right_hand_camera(
    stage=None, *, xml_path: str | Path = DEFAULT_XML,
    robot_prim_path: str | None = None, save: bool = True,
) -> str:
    """Author XML calibration in the root layer and return the Action Graph camera path.

    Call once with playback stopped. save=True saves the root layer, including
    other pending root-layer edits. save=False permits an anonymous stage that
    you will subsequently Save As. Referenced robot/payload layers are not edited.
    """
    from pxr import Gf, Usd, UsdGeom

    mount = read_camera_mount(xml_path)
    if stage is None:
        import omni.usd

        stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("Open the imported robot stage before preparing the camera")
    layer = stage.GetRootLayer()
    if save and layer.anonymous:
        raise ValueError("Save the stage to a USD file first, or pass save=False and then Save As")
    if not layer.permissionToEdit or (save and not layer.permissionToSave):
        raise ValueError("The root layer must be writable; save a writable scene USD first")
    head = _find_head(stage, mount, robot_prim_path)
    path = head.GetPath().AppendChild(CAMERA_NAME)
    _check_camera_slot(stage, path)
    meters_per_unit = UsdGeom.GetStageMetersPerUnit(stage)
    if not math.isfinite(meters_per_unit) or meters_per_unit <= 0:
        raise ValueError("Stage metersPerUnit must be positive and finite")
    w, x, y, z = mount.quaternion
    matrix = Gf.Matrix4d(1).SetRotate(Gf.Quatd(w, Gf.Vec3d(x, y, z)))
    matrix.SetTranslateOnly(Gf.Vec3d(*(value / meters_per_unit for value in mount.position)))
    with Usd.EditContext(stage, layer):
        camera = UsdGeom.Camera.Define(stage, path)
        xform = UsdGeom.Xformable(camera)
        # One matrix op is reused on subsequent runs; no parent transforms change.
        op = xform.MakeMatrixXform()
        op.GetAttr().Clear()
        op.Set(matrix)
        xform.SetResetXformStack(False)
        _configure_optics(camera, mount, meters_per_unit)
    if not camera.GetPrim().IsA(UsdGeom.Camera) or xform.GetResetXformStack():
        raise RuntimeError(f"Camera authoring failed at {path}")
    if save and not layer.Save():
        raise OSError(f"Could not save camera preparation to {layer.identifier}")
    print(f"Camera Prim: {path}")
    print(f"XML calibration: {xml_path}; vertical FOV: {mount.fovy:g} degrees")
    width, height = RENDER_RESOLUTION
    print(f"Render product: width={width}, height={height}; configure {TARGET_RATE_HZ:g} Hz scheduling separately")
    print(f"Root layer: {layer.identifier} ({'saved' if save else 'save this layer before closing'})")
    return str(path)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--stage", required=True, help="Writable imported USD stage to prepare and save")
    parser.add_argument("--xml", type=Path, default=DEFAULT_XML, help="Authoritative source MJCF calibration")
    parser.add_argument("--robot-prim-path", help="Select one robot when several are present")
    args = parser.parse_args()
    from pxr import Usd

    stage = Usd.Stage.Open(args.stage)
    if stage is None:
        raise RuntimeError(f"Could not open USD stage: {args.stage}")
    prepare_right_hand_camera(stage, xml_path=args.xml, robot_prim_path=args.robot_prim_path)


if __name__ == "__main__":
    main()
