r"""Right-wrist RGB camera for an already imported robot in Isaac Sim 6.1.

Uses ``isaacsim.sensors.experimental.rtx.RtxCamera`` and ``CameraSensor``.
The older ``isaacsim.sensors.camera`` API is deprecated in 6.1. API reference:
https://github.com/isaac-sim/IsaacSim/blob/v6.1.0/source/standalone_examples/api/isaacsim.sensors.experimental.rtx/create_camera_basic.py

Import ``s40_sharpa_cameras.xml`` with the MJCF importer first, keeping visual
geometry separate from collisions (do not enable collision-from-visuals).
An existing USD Camera is reused; otherwise a camera is authored under the
imported wrist. No robot joints, physics, scene lighting, or files are changed.

In Isaac Sim's Script Editor, after adding the repository root to sys.path::

    from control.cameras.right_hand_camera import RightHandCamera
    camera = RightHandCamera()  # or robot_prim_path="/World/my_robot"
    # Press Play and let rendering advance before reading; keep camera alive.
    rgb = camera.get_rgb()  # None during warm-up, then uint8 (480, 640, 3)
    camera.print_debug()
    # camera.close() when finished, before closing the stage

For a bounded asynchronous check in the running GUI::

    import asyncio
    from control.cameras.right_hand_camera import debug_current_stage
    camera_task = asyncio.ensure_future(debug_current_stage())

Standalone, from the repository root, using Isaac Sim 6.1's python.sh::

    /path/to/isaac-sim/python.sh control/cameras/right_hand_camera.py \
        --stage /absolute/path/to/imported_scene.usd --headless

The standalone check opens an already imported scene; it does not import,
export, or save USD files. Use a scene with lighting and visible geometry.
Rendering/physics must advance at least 15 times per simulation second for
the requested 15 Hz sensor tick rate. Reading get_rgb() faster returns the
latest buffer again; it is not a new-frame or synchronization API.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET


CAMERA_NAME = "right_hand_camera"
WRIST_NAME = "zarm_r7_link"
RESOLUTION = (480, 640)  # CameraSensor 6.1 takes (height, width), not (width, height).
CAPTURE_HZ = 15.0
DEFAULT_XML = Path(__file__).resolve().parents[2] / "models/robots/s40_sharpa/assembly/s40_sharpa_cameras.xml"


@dataclass(frozen=True)
class CameraMount:
    position: tuple[float, ...]
    quaternion: tuple[float, ...]  # MJCF and USD both use w, x, y, z; view direction is -Z.
    fovy: float


def read_camera_mount(xml_path: str | Path = DEFAULT_XML) -> CameraMount:
    """Read the XML as the calibration source without rewriting it."""
    root = ET.parse(xml_path).getroot()
    cameras = root.findall(f".//camera[@name='{CAMERA_NAME}']")
    wrists = root.findall(f".//body[@name='{WRIST_NAME}']")
    if len(cameras) != 1 or len(wrists) != 1 or cameras[0] not in list(wrists[0]):
        raise ValueError(f"Expected exactly one {CAMERA_NAME} directly inside {WRIST_NAME} in {xml_path}")
    camera = cameras[0]
    mount = CameraMount(
        tuple(float(value) for value in camera.attrib["pos"].split()),
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


def _find_wrist(stage, robot_prim_path: str | None):
    from pxr import Usd, UsdGeom

    root = stage.GetPrimAtPath(robot_prim_path) if robot_prim_path else stage.GetPseudoRoot()
    if not root.IsValid():
        raise ValueError(f"Robot prim does not exist: {robot_prim_path}")
    wrists = [prim for prim in Usd.PrimRange(root) if prim.GetName() == WRIST_NAME]
    if len(wrists) != 1:
        paths = [str(prim.GetPath()) for prim in wrists]
        raise ValueError(
            f"Expected one imported {WRIST_NAME}; found {paths}. "
            "Import the camera XML first, or set robot_prim_path to select one robot."
        )
    wrist = wrists[0]
    if not UsdGeom.Xformable(wrist) or wrist.IsInstance() or wrist.IsInstanceProxy():
        raise ValueError(f"Wrist must be an editable, non-instanced transform: {wrist.GetPath()}")
    return wrist


def _inherits_wrist_transform(prim, wrist) -> bool:
    from pxr import UsdGeom, UsdPhysics

    while prim.IsValid() and prim != wrist:
        xform = UsdGeom.Xformable(prim)
        if (xform and xform.GetResetXformStack()) or prim.HasAPI(UsdPhysics.RigidBodyAPI):
            return False
        prim = prim.GetParent()
    return prim == wrist


def _find_or_create_camera(stage, wrist, mount: CameraMount):
    from pxr import Gf, Usd, UsdGeom

    candidates = [
        prim for prim in Usd.PrimRange(wrist)
        if prim.GetName() == CAMERA_NAME and prim.IsA(UsdGeom.Camera)
        and _inherits_wrist_transform(prim, wrist)
    ]
    if len(candidates) > 1:
        raise ValueError(f"Multiple usable {CAMERA_NAME} cameras below {wrist.GetPath()}")
    if candidates:
        return UsdGeom.Camera(candidates[0]), False

    # Preserve any importer placeholder with this name instead of changing its type.
    path = wrist.GetPath().AppendChild(CAMERA_NAME)
    if stage.GetPrimAtPath(path).IsValid():
        path = wrist.GetPath().AppendChild(CAMERA_NAME + "_rgb")
        existing = stage.GetPrimAtPath(path)
        if existing.IsValid():
            if existing.IsA(UsdGeom.Camera) and _inherits_wrist_transform(existing, wrist):
                return UsdGeom.Camera(existing), False
            raise ValueError(f"Fallback camera path is occupied by an unusable prim: {path}")

    camera = UsdGeom.Camera.Define(stage, path)
    meters_per_unit = UsdGeom.GetStageMetersPerUnit(stage)
    xform = UsdGeom.Xformable(camera.GetPrim())
    xform.AddTranslateOp().Set(Gf.Vec3d(*(value / meters_per_unit for value in mount.position)))
    w, x, y, z = mount.quaternion
    xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(w, Gf.Vec3d(x, y, z)))
    return camera, True


def _configure_optics(camera, mount: CameraMount, meters_per_unit: float) -> None:
    from pxr import Gf, UsdGeom

    # USD focal length/apertures are in tenths of a stage unit. Only their ratio
    # determines pinhole FOV; use an equivalent 24 mm lens, not a claimed calibration.
    focal_length = 0.024 / (0.1 * meters_per_unit)
    vertical_aperture = 2.0 * focal_length * math.tan(math.radians(mount.fovy) / 2.0)
    camera.CreateProjectionAttr().Set(UsdGeom.Tokens.perspective)
    camera.CreateFocalLengthAttr().Set(focal_length)
    camera.CreateVerticalApertureAttr().Set(vertical_aperture)
    camera.CreateHorizontalApertureAttr().Set(vertical_aperture * RESOLUTION[1] / RESOLUTION[0])
    camera.CreateHorizontalApertureOffsetAttr().Set(0.0)
    camera.CreateVerticalApertureOffsetAttr().Set(0.0)
    camera.CreateClippingRangeAttr().Set(Gf.Vec2f(0.005 / meters_per_unit, 100.0 / meters_per_unit))
    camera.CreateFStopAttr().Set(0.0)  # Pinhole RGB; no depth of field.


class RightHandCamera:
    """Discover a wrist camera and expose its latest RGB frame as a NumPy array.

    Construct after SimulationApp/Kit and the imported stage exist. The caller
    owns timeline/physics/render stepping. Recreate after replacing the stage.
    Keep only one live adapter per camera; call close() before constructing another.
    """

    def __init__(self, robot_prim_path: str | None = None, xml_path: str | Path = DEFAULT_XML):
        self._sensor = None
        self._authoring = None
        self.mount = read_camera_mount(xml_path)
        # Imports are deferred so normal Python can inspect calibration and --help.
        import omni.kit.app
        import omni.usd
        from pxr import Usd, UsdGeom

        manager = omni.kit.app.get_app().get_extension_manager()
        extension = "isaacsim.sensors.experimental.rtx"
        if not manager.is_extension_enabled(extension):
            manager.set_extension_enabled_immediate(extension, True)
        try:
            from isaacsim.sensors.experimental.rtx import CameraSensor, RtxCamera
        except ImportError as error:
            raise RuntimeError("Isaac Sim 6.1 with isaacsim.sensors.experimental.rtx is required") from error

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("Open a stage containing the imported robot before initializing the camera")
        self._stage = stage
        wrist = _find_wrist(stage, robot_prim_path)
        self.wrist_prim_path = str(wrist.GetPath())
        # Runtime sensor opinions belong in the session layer, not the robot asset.
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            camera, self.created_fallback = _find_or_create_camera(stage, wrist, self.mount)
            self.prim_path = str(camera.GetPath())
            self._authoring = RtxCamera(
                self.prim_path, tick_rate=CAPTURE_HZ, reset_xform_op_properties=False
            )
            self._sensor = CameraSensor(self._authoring, resolution=RESOLUTION, annotators=["rgb"])
            if tuple(self._sensor.resolution) != RESOLUTION:
                self.close()
                raise RuntimeError("An existing render product overrides 640x480; remove that camera's old render product first")
            if not _inherits_wrist_transform(camera.GetPrim(), wrist):
                self.close()
                raise RuntimeError("The initialized camera does not inherit the wrist transform")
            # CameraSensor enforces square pixels at construction; author our exact
            # 4:3 apertures afterwards so the final vertical FOV is the XML value.
            _configure_optics(camera, self.mount, UsdGeom.GetStageMetersPerUnit(stage))

    def get_rgb(self):
        """Return a copied uint8 (480, 640, 3) RGB array, or None during warm-up.

        Reads the latest sensor buffer. Does not step simulation, start recording,
        or guarantee a different capture on every call.
        """
        import numpy as np
        import omni.usd

        if self._sensor is None:
            raise RuntimeError("Camera is closed")
        if omni.usd.get_context().get_stage() != self._stage:
            raise RuntimeError("The stage changed; close and recreate RightHandCamera")
        data, _ = self._sensor.get_data("rgb")
        if data is None:
            return None
        frame = np.asarray(data.numpy())  # 6.1 CameraSensor returns a Warp array.
        if frame.size == 0:
            return None
        if frame.shape != (*RESOLUTION, 3) or frame.dtype != np.uint8:
            raise RuntimeError(f"Unexpected RGB buffer: shape={frame.shape}, dtype={frame.dtype}")
        return frame.copy()  # The renderer may reuse its buffer on the next tick.

    def print_debug(self) -> None:
        frame = self.get_rgb()
        print(f"Camera prim: {self.prim_path} (runtime fallback created: {self.created_fallback})")
        print(f"Wrist prim: {self.wrist_prim_path}")
        print(f"Resolution: 640 x 480; sensor tick rate: {CAPTURE_HZ:g} Hz; fovy: {self.mount.fovy:g} degrees")
        print(f"RGB frame shape: {None if frame is None else frame.shape}")

    def close(self) -> None:
        """Release RGB annotation/render resources; leave the camera prim reusable."""
        if self._sensor is not None:
            self._sensor.detach_annotators(["rgb"])
            # 6.1 SensorRuntime.__del__ releases the render product/subscription.
            self._sensor = None
        self._authoring = None


async def debug_current_stage(robot_prim_path: str | None = None, timeout: float = 30.0) -> None:
    """Bounded GUI check; start Play if needed, then restore its previous state."""
    import omni.kit.app
    import omni.timeline

    if timeout <= 0:
        raise ValueError("timeout must be positive")
    camera = RightHandCamera(robot_prim_path=robot_prim_path)
    timeline = omni.timeline.get_timeline_interface()
    was_playing = timeline.is_playing()
    try:
        timeline.play()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            await omni.kit.app.get_app().next_update_async()
            if camera.get_rgb() is not None:
                camera.print_debug()
                return
        raise TimeoutError("No RGB frame: check Play, RTX renderer/GPU, and the render-product errors")
    finally:
        camera.close()
        if not was_playing:
            timeline.pause()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--stage", required=True, help="Existing USD scene imported from the camera XML")
    parser.add_argument("--robot-prim-path", help="Restrict discovery when the stage contains several robots")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--timeout", type=float, default=60.0, help="Maximum warm-up seconds")
    args = parser.parse_args()
    if args.timeout <= 0:
        parser.error("--timeout must be positive")

    from isaacsim import SimulationApp

    app = SimulationApp({"headless": args.headless})
    camera = None
    timeline = None
    try:
        import omni.timeline
        import omni.usd

        context = omni.usd.get_context()
        if not context.open_stage(args.stage):
            raise RuntimeError(f"Could not open USD stage: {args.stage}")
        deadline = time.monotonic() + args.timeout
        while context.get_stage_loading_status()[2] > 0:
            if not app.is_running() or time.monotonic() >= deadline:
                raise TimeoutError(f"Stage loading did not complete: {args.stage}")
            app.update()
        camera = RightHandCamera(robot_prim_path=args.robot_prim_path)
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        deadline = time.monotonic() + args.timeout
        while app.is_running() and time.monotonic() < deadline:
            app.update()
            if camera.get_rgb() is not None:
                camera.print_debug()
                return
        raise TimeoutError("No RGB frame: check RTX GPU support, Play, and renderer errors")
    finally:
        if camera is not None:
            camera.close()
        if timeline is not None:
            timeline.stop()
        app.close()


if __name__ == "__main__":
    main()
