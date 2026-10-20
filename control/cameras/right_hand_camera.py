r"""Prepare persistent hand/head USD Cameras once after MJCF import.

Import s40_sharpa_two_cameras.xml, stop the timeline, and save the imported
stage to a writable USD file. In Isaac Sim 6.1's Script Editor, with the
repository root on sys.path, run::

    from control.cameras.right_hand_camera import prepare_all_cameras
    camera_paths = prepare_all_cameras()  # prints paths and saves the root layer

For an unsaved stage, pass save=False and then use File > Save As. To select
one robot in a multi-robot stage, pass robot_prim_path="/World/my_robot".
Pass xml_path explicitly if the imported robot came from a different XML.

prepare_right_hand_camera() remains compatible with the working right-only
workflow. prepare_left_hand_camera() and prepare_head_camera() prepare just
one camera. prepare_all_cameras() returns a {camera_name: prim_path} dict.
It preserves an existing persistent right camera, including manual USD
calibration. The right-only function keeps its original XML-reapply behavior.

All cameras inherit their XML parent frame and persist after reopening USD.
Only camera opinions in the writable root layer are authored. No render
products, runtime sensors, ROS publishers, rates, or Action Graph nodes are
created. Render resolution is configured manually later; the existing 4:3
aperture convention is retained for the hand cameras and the head preview.

Head calibration is unavailable: its XML intentionally omits fovy and uses
the MJCF default of 45 degrees, not a measured intrinsic. Set head_camera's
fovy in the XML when calibrated. Its optical center is assumed to coincide
with the existing mesh origin; mesh +X/+Z map to optical forward/up.

The same preparation can run with Isaac Sim's python.sh without starting Kit::

    /path/to/isaac-sim/python.sh control/cameras/right_hand_camera.py \
        --stage /absolute/path/to/imported_robot.usd --camera all

Without --camera, the CLI retains its original right-hand-only behavior.
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
MJCF_DEFAULT_FOVY = 45.0  # MJCF's built-in default, explicitly uncalibrated for the head.
CAMERA_ATTACHMENTS = {
    CAMERA_NAME: (WRIST_NAME, HEAD_NAME),
    "left_hand_camera": ("zarm_l7_link", "left_hand_camera_head"),
    "head_camera": ("zhead_2_link", "zhead_2_link"),
}
DEFAULT_XML = Path(__file__).resolve().parents[2] / "models/robots/s40_sharpa/assembly/s40_sharpa_two_cameras.xml"


@dataclass(frozen=True)
class CameraMount:
    body_names: tuple[str, ...]
    position: tuple[float, ...]
    quaternion: tuple[float, ...]  # MJCF/USD wxyz; camera looks along local -Z.
    fovy: float


def read_camera_mount(
    xml_path: str | Path = DEFAULT_XML, *, camera_name: str = CAMERA_NAME,
) -> CameraMount:
    """Read the nested camera's local calibration without changing the XML."""
    if camera_name not in CAMERA_ATTACHMENTS:
        raise ValueError(f"Unknown camera: {camera_name}")
    anchor_name, parent_name = CAMERA_ATTACHMENTS[camera_name]
    root = ET.parse(xml_path).getroot()
    cameras = root.findall(f".//camera[@name='{camera_name}']")
    if len(cameras) != 1:
        raise ValueError(f"Expected exactly one {camera_name} in {xml_path}")
    camera = cameras[0]
    parents = {child: parent for parent in root.iter() for child in parent}
    parent = parents[camera]
    if parent.tag != "body" or parent.get("name") != parent_name:
        raise ValueError(f"{camera_name} must be directly inside {parent_name}")
    chain = []
    while parent.tag == "body":
        chain.append(parent.attrib["name"])
        if parent.get("name") == anchor_name:
            break
        parent = parents[parent]
    if chain[-1] != anchor_name:
        raise ValueError(f"{parent_name} must be inside {anchor_name}")
    if any(key in camera.attrib for key in ("euler", "axisangle", "xyaxes", "zaxis")):
        raise ValueError("Camera calibration must use an explicit MJCF wxyz quat")
    if camera.get("mode", "fixed") != "fixed":
        raise ValueError("The camera must use fixed mode to follow its parent")
    mount = CameraMount(
        tuple(reversed(chain)),
        tuple(float(value) for value in camera.get("pos", "0 0 0").split()),
        tuple(float(value) for value in camera.attrib["quat"].split()),
        float(camera.get("fovy", str(MJCF_DEFAULT_FOVY))),
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


def _find_camera_parent(stage, mount: CameraMount, robot_prim_path: str | None):
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
        raise ValueError(f"Camera parent must be an editable, non-instanced transform: {head.GetPath()}")
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


def _prepare_cameras(
    camera_names: tuple[str, ...], stage=None, *, xml_path: str | Path = DEFAULT_XML,
    robot_prim_path: str | None = None, save: bool = True,
    preserve_existing: tuple[str, ...] = (),
) -> dict[str, str]:
    """Validate all requested cameras, author their XML calibration, and save once.

    Call once with playback stopped. save=True saves the root layer, including
    other pending root-layer edits. save=False permits an anonymous stage that
    you will subsequently Save As. Referenced robot/payload layers are not edited.
    """
    from pxr import Gf, Usd, UsdGeom

    mounts = {name: read_camera_mount(xml_path, camera_name=name) for name in camera_names}
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
    paths = {}
    preserved = set()
    for name, mount in mounts.items():
        parent = _find_camera_parent(stage, mount, robot_prim_path)
        path = parent.GetPath().AppendChild(name)
        _check_camera_slot(stage, path)
        paths[name] = path
        prim = stage.GetPrimAtPath(path)
        if name in preserve_existing and prim.IsValid() and prim.IsA(UsdGeom.Camera):
            if UsdGeom.Xformable(prim).GetResetXformStack():
                raise ValueError(f"Existing camera does not inherit its parent transform: {path}")
            preserved.add(name)
    meters_per_unit = UsdGeom.GetStageMetersPerUnit(stage)
    if not math.isfinite(meters_per_unit) or meters_per_unit <= 0:
        raise ValueError("Stage metersPerUnit must be positive and finite")
    with Usd.EditContext(stage, layer):
        for name, mount in mounts.items():
            if name in preserved:
                continue
            path = paths[name]
            w, x, y, z = mount.quaternion
            matrix = Gf.Matrix4d(1).SetRotate(Gf.Quatd(w, Gf.Vec3d(x, y, z)))
            matrix.SetTranslateOnly(Gf.Vec3d(*(value / meters_per_unit for value in mount.position)))
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
    for name, path in paths.items():
        print(f"Camera Prim: {path}")
        if name in preserved:
            print("Existing persistent camera retained, including its USD calibration")
        else:
            print(f"XML calibration: {xml_path}; vertical FOV: {mounts[name].fovy:g} degrees")
            if name == "head_camera":
                print("Head fovy is an uncalibrated MJCF default when omitted from XML")
    print(f"Root layer: {layer.identifier} ({'saved' if save else 'save this layer before closing'})")
    return {name: str(path) for name, path in paths.items()}


def prepare_right_hand_camera(
    stage=None, *, xml_path: str | Path = DEFAULT_XML,
    robot_prim_path: str | None = None, save: bool = True,
) -> str:
    """Prepare and save the right camera; same signature, calibration and path as before."""
    return _prepare_cameras((CAMERA_NAME,), stage, xml_path=xml_path,
                            robot_prim_path=robot_prim_path, save=save)[CAMERA_NAME]


def prepare_left_hand_camera(
    stage=None, *, xml_path: str | Path = DEFAULT_XML,
    robot_prim_path: str | None = None, save: bool = True,
) -> str:
    """Prepare and save the mirrored left camera using the XML calibration."""
    return _prepare_cameras(("left_hand_camera",), stage, xml_path=xml_path,
                            robot_prim_path=robot_prim_path, save=save)["left_hand_camera"]


def prepare_head_camera(
    stage=None, *, xml_path: str | Path = DEFAULT_XML,
    robot_prim_path: str | None = None, save: bool = True,
) -> str:
    """Prepare the head camera; its omitted XML FOV is an uncalibrated MJCF default."""
    return _prepare_cameras(("head_camera",), stage, xml_path=xml_path,
                            robot_prim_path=robot_prim_path, save=save)["head_camera"]


def prepare_all_cameras(
    stage=None, *, xml_path: str | Path = DEFAULT_XML,
    robot_prim_path: str | None = None, save: bool = True,
) -> dict[str, str]:
    """Prepare all three with playback stopped; save once and return/print their paths.

    Keep an existing persistent right camera exactly as saved; create it from
    XML only if missing. The individual right function retains XML reapplication.
    save=True saves pending root-layer edits too. For an anonymous stage, pass
    save=False and then Save As before closing. No referenced robot layers change.
    """
    return _prepare_cameras(tuple(CAMERA_ATTACHMENTS), stage, xml_path=xml_path,
                            robot_prim_path=robot_prim_path, save=save,
                            preserve_existing=(CAMERA_NAME,))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--stage", required=True, help="Writable imported USD stage to prepare and save")
    parser.add_argument("--xml", type=Path, default=DEFAULT_XML, help="Authoritative source MJCF calibration")
    parser.add_argument("--robot-prim-path", help="Select one robot when several are present")
    parser.add_argument("--camera", choices=(*CAMERA_ATTACHMENTS, "all"), default=CAMERA_NAME,
                        help="Camera to prepare; default preserves the right-only workflow")
    args = parser.parse_args()
    from pxr import Usd

    stage = Usd.Stage.Open(args.stage)
    if stage is None:
        raise RuntimeError(f"Could not open USD stage: {args.stage}")
    names = tuple(CAMERA_ATTACHMENTS) if args.camera == "all" else (args.camera,)
    _prepare_cameras(names, stage, xml_path=args.xml, robot_prim_path=args.robot_prim_path,
                     preserve_existing=(CAMERA_NAME,) if args.camera == "all" else ())


if __name__ == "__main__":
    main()
