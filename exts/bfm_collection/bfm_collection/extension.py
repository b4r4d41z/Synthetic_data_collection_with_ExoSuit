"""Kit lifecycle adapter. Simulation must be stopped when enabling the extension.

Timeline POP subscription handles PLAY/PAUSE/STOP events from the Isaac Sim UI.
Runtime object composition is performed when PLAY is received.
"""

import importlib
import site
import sys
from pathlib import Path

import carb
import omni.ext
import omni.kit.app
import omni.timeline
import omni.usd

from .scene_randomizer import (
    CollectionLifecycle,
    SceneRandomizer,
    load_configuration,
)


_active = None


def get_status():
    """Optional diagnostics only; enabling/using the extension needs no script."""
    if _active is None or _active.lifecycle is None:
        return {"attached": False}

    lifecycle = _active.lifecycle
    sampler = getattr(
        getattr(lifecycle.handle, "node", None),
        "sampler",
        None,
    )

    return {
        "attached": True,
        "state": lifecycle.state,
        "episode": lifecycle.randomizer.last_episode,
        "resolved_body_counts": getattr(
            sampler,
            "resolved_body_counts",
            {},
        ),
    }


class CollectionExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        global _active

        self.lifecycle = None
        self.stage = None
        self.failed_stage = None
        self.closing = False

        # Repository root:
        # .../Synthetic_data_collection_with_ExoSuit
        self.repository = Path(__file__).resolve().parents[3]

        # Make ROS/colcon Python packages visible to Kit embedded Python.
        self._bootstrap_ros_python_paths()

        print(
            "[BFM] Extension loaded successfully.",
            flush=True,
        )

        self.timeline = omni.timeline.get_timeline_interface()
        self.usd = omni.usd.get_context()

        self.stage_subscription = (
            self.usd.get_stage_event_stream().create_subscription_to_pop(
                self._stage_event,
                name="bfm_collection_stage",
            )
        )

        # IMPORTANT:
        # UI PLAY/PAUSE/STOP events are received through POP subscription
        # in the current Isaac Sim 6.1 runtime.
        self.timeline_subscription = (
            self.timeline.get_timeline_event_stream().create_subscription_to_pop(
                self._timeline_event,
                order=-10000,
                name="bfm_collection_timeline",
            )
        )

        self.update_subscription = (
            omni.kit.app.get_app()
            .get_update_event_stream()
            .create_subscription_to_pop(
                self._update,
                name="bfm_collection_stage_ready",
            )
        )

        self.shutdown_subscription = (
            omni.kit.app.get_app()
            .get_shutdown_event_stream()
            .create_subscription_to_pop(
                lambda event: self._detach(),
                name="bfm_collection_shutdown",
            )
        )

        _active = self

        carb.log_info(
            "BFM collection loaded; "
            "open stage_sorting_v1.usd with simulation stopped"
        )

        print(
            "[BFM] Open stage_sorting_v1.usd and keep simulation stopped.",
            flush=True,
        )

        self._attach()

    def _bootstrap_ros_python_paths(self):
        """Add colcon-built ROS Python packages to Kit's active sys.path."""
        python_dir = (
            f"python{sys.version_info.major}.{sys.version_info.minor}"
        )

        workspace = self.repository / "exo2isaacsim"

        roots = [
            workspace / "build/isaac_eff_control",
            workspace
            / "install/dynamic_biped/lib"
            / python_dir
            / "site-packages",
        ]

        for root in roots:
            if not root.is_dir():
                raise RuntimeError(
                    f"Missing ROS Python package directory: {root}"
                )

            site.addsitedir(str(root))

            carb.log_info(
                f"BFM collection added Python path: {root}"
            )

        importlib.invalidate_caches()

        # Validate immediately so failure is explicit during extension startup.
        try:
            import isaac_eff_control
            import dynamic_biped
            import rclpy
        except Exception as error:
            raise RuntimeError(
                f"ROS Python bootstrap failed: {error}"
            ) from error

        carb.log_info(
            "BFM collection ROS Python bootstrap OK: "
            f"isaac_eff_control={isaac_eff_control.__file__}, "
            f"dynamic_biped={dynamic_biped.__file__}, "
            f"rclpy={rclpy.__file__}"
        )

        print(
            "[BFM] ROS Python bootstrap OK.",
            flush=True,
        )

    def _attach(self):
        stage = self.usd.get_stage()

        if (
            self.closing
            or stage is None
            or stage == self.stage
            or stage == self.failed_stage
        ):
            return

        # Other scenes/robot-only USDs are completely outside this extension.
        expected = (
            self.repository
            / "models/stages/stage_sorting_v1.usd"
        )

        root_real_path = stage.GetRootLayer().realPath

        if not root_real_path:
            return

        if Path(root_real_path).resolve() != expected.resolve():
            return

        try:
            if not self.timeline.is_stopped():
                raise RuntimeError(
                    "Stop simulation before enabling BFM collection "
                    "or opening the sorting stage"
                )

            from isaac_eff_control.bfm_contract import (
                scene_config_default,
                state_defaults,
            )

            path = scene_config_default()

            data = load_configuration(
                path,
                self.repository,
            )

            self.parameters = state_defaults(path)

            if (
                self.parameters["object_prim_path"]
                != data["parameters"]["object_prim_path"]
            ):
                raise ValueError(
                    "Remove BFM_OBJECT_PRIM_PATH override; "
                    "collection uses /World/Task/Object"
                )

            self.config_path = path

            randomizer = SceneRandomizer(
                stage,
                data,
                self.repository,
            )

            self.lifecycle = CollectionLifecycle(
                randomizer,
                self._start_publisher,
            )

            self.stage = stage

            message = (
                "BFM collection ready: "
                "next Stop -> Play selects one fruit"
            )

            carb.log_info(message)

            print(
                "\n"
                "============================================\n"
                "[BFM READY] Scene is ready.\n"
                "[BFM READY] You can press PLAY now.\n"
                "============================================\n",
                flush=True,
            )

        except Exception as error:
            self.failed_stage = stage

            carb.log_error(
                f"BFM collection setup disabled: {error}. "
                "Fix setup and reload the extension."
            )

            print(
                f"[BFM ERROR] Collection setup failed: {error}",
                flush=True,
            )

    def _start_publisher(self):
        from isaac_eff_control.bfm_task_state import (
            start_task_state_publisher,
        )

        print(
            "[BFM] Preparing /bfm/state; waiting for PhysX steps...",
            flush=True,
        )

        return start_task_state_publisher(
            bfm_collection_mode=True,
            scene_config_path=self.config_path,
            **self.parameters,
        )

    def _detach(self):
        try:
            if self.lifecycle is not None:
                self.lifecycle.close()

        except Exception as error:
            carb.log_error(
                f"BFM collection cleanup failed: {error}"
            )

            print(
                f"[BFM ERROR] Cleanup failed: {error}",
                flush=True,
            )

        finally:
            self.lifecycle = None
            self.stage = None

    def _stage_event(self, event):
        if event.type in (
            int(omni.usd.StageEventType.CLOSING),
            int(omni.usd.StageEventType.CLOSED),
        ):
            # Do not reattach old stage during asynchronous close.
            self.closing = True
            self._detach()
            self.failed_stage = None

        elif event.type == int(
            omni.usd.StageEventType.OPENED
        ):
            self.closing = False
            self._attach()

    def _update(self, event):
        if (
            self.stage is not None
            and self.usd.get_stage() != self.stage
        ):
            self._detach()

        self._attach()

    def _timeline_event(self, event):
        types = omni.timeline.TimelineEventType

        play_type = int(types.PLAY)
        pause_type = int(types.PAUSE)
        stop_type = int(types.STOP)

        # Ignore high-frequency timeline events such as type=4.
        if event.type not in (
            play_type,
            pause_type,
            stop_type,
        ):
            return

        print(
            f"[BFM DEBUG] timeline event: type={event.type}",
            flush=True,
        )

        if self.lifecycle is None:
            return

        if self.usd.get_stage() != self.stage:
            self._detach()
            return

        try:
            if event.type == play_type:
                if self.lifecycle.state == "stopped":
                    print(
                        "[BFM] PLAY received. Creating new episode...",
                        flush=True,
                    )

                episode = self.lifecycle.play()

                if episode is not None:
                    carb.log_info(
                        "BFM collection episode object: "
                        f"asset={episode['asset']} "
                        f"position={episode['position']} "
                        f"yaw={episode['yaw']}"
                    )

                    print(
                        "[BFM] Episode created:\n"
                        f"      asset={episode['asset']}\n"
                        f"      position={episode['position']}\n"
                        f"      yaw={episode['yaw']}\n"
                        f"      container_position={episode['container_position']}\n"
                        f"      container_offset={episode['container_offset']}",
                        flush=True,
                    )

            elif event.type == pause_type:
                print(
                    "[BFM] PAUSE received.",
                    flush=True,
                )

                self.lifecycle.pause()

            elif event.type == stop_type:
                print(
                    "[BFM] STOP received.",
                    flush=True,
                )

                self.lifecycle.stop()

        except Exception as error:
            carb.log_error(
                f"BFM collection Play cancelled: {error}"
            )

            print(
                f"[BFM ERROR] PLAY cancelled: {error}",
                flush=True,
            )

            try:
                self.lifecycle.stop()

            finally:
                self.timeline.stop()

    def on_shutdown(self):
        global _active

        self.stage_subscription = None
        self.timeline_subscription = None
        self.update_subscription = None
        self.shutdown_subscription = None

        self._detach()

        if _active is self:
            _active = None
