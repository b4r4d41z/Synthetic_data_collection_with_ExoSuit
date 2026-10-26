"""Kit lifecycle adapter. Simulation must be stopped when enabling the extension.

Timeline PUSH subscription (early order) authors the object synchronously on
PLAY, before queued timeline consumers/physics updates. It performs no async
work. Validate this ordering in the target Isaac 6.1 runtime before collection.
"""

from pathlib import Path

import carb
import omni.ext
import omni.kit.app
import omni.timeline
import omni.usd

from .scene_randomizer import CollectionLifecycle, SceneRandomizer, load_configuration


_active = None


def get_status():
    """Optional diagnostics only; enabling/using the extension needs no script."""
    if _active is None or _active.lifecycle is None:
        return {"attached": False}
    lifecycle = _active.lifecycle
    sampler = getattr(getattr(lifecycle.handle, "node", None), "sampler", None)
    return {"attached": True, "state": lifecycle.state,
            "episode": lifecycle.randomizer.last_episode,
            "resolved_body_counts": getattr(sampler, "resolved_body_counts", {})}


class CollectionExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        global _active
        self.lifecycle = None
        self.stage = None
        self.failed_stage = None
        self.closing = False
        self.repository = Path(__file__).resolve().parents[3]
        self.timeline = omni.timeline.get_timeline_interface()
        self.usd = omni.usd.get_context()
        self.stage_subscription = self.usd.get_stage_event_stream().create_subscription_to_pop(
            self._stage_event, name="bfm_collection_stage")
        self.timeline_subscription = self.timeline.get_timeline_event_stream().create_subscription_to_push(
            self._timeline_event, order=-10000, name="bfm_collection_before_physics")
        self.update_subscription = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
            self._update, name="bfm_collection_stage_ready")
        self.shutdown_subscription = omni.kit.app.get_app().get_shutdown_event_stream().create_subscription_to_pop(
            lambda event: self._detach(), name="bfm_collection_shutdown")
        _active = self
        carb.log_info("BFM collection loaded; open stage_sorting_v1.usd with simulation stopped")
        self._attach()

    def _attach(self):
        stage = self.usd.get_stage()
        if self.closing or stage is None or stage == self.stage or stage == self.failed_stage:
            return
        # Other scenes/robot-only USDs are completely outside this extension.
        expected = self.repository / "models/stages/stage_sorting_v1.usd"
        if Path(stage.GetRootLayer().realPath).resolve() != expected.resolve():
            return
        try:
            if not self.timeline.is_stopped():
                raise RuntimeError("Stop simulation before enabling BFM collection or opening the sorting stage")
            from isaac_eff_control.bfm_contract import scene_config_default, state_defaults
            path = scene_config_default()
            data = load_configuration(path, self.repository)
            self.parameters = state_defaults(path)
            if self.parameters["object_prim_path"] != data["parameters"]["object_prim_path"]:
                raise ValueError("Remove BFM_OBJECT_PRIM_PATH override; collection uses /World/Task/Object")
            self.config_path = path
            randomizer = SceneRandomizer(stage, data, self.repository)
            self.lifecycle = CollectionLifecycle(randomizer, self._start_publisher)
            self.stage = stage
            carb.log_info("BFM collection ready: next Stop -> Play selects one fruit")
        except Exception as error:
            self.failed_stage = stage
            carb.log_error(f"BFM collection setup disabled: {error}. Fix setup and reload the extension.")

    def _start_publisher(self):
        from isaac_eff_control.bfm_task_state import start_task_state_publisher
        return start_task_state_publisher(bfm_collection_mode=True,
            scene_config_path=self.config_path, **self.parameters)

    def _detach(self):
        try:
            if self.lifecycle is not None:
                self.lifecycle.close()
        except Exception as error:
            carb.log_error(f"BFM collection cleanup failed: {error}")
        finally:
            self.lifecycle = None
            self.stage = None

    def _stage_event(self, event):
        if event.type in (int(omni.usd.StageEventType.CLOSING), int(omni.usd.StageEventType.CLOSED)):
            self.closing = True  # Do not reattach the old stage during asynchronous close.
            self._detach()
            self.failed_stage = None
        elif event.type == int(omni.usd.StageEventType.OPENED):
            self.closing = False
            self._attach()

    def _update(self, event):
        if self.stage is not None and self.usd.get_stage() != self.stage:
            self._detach()
        self._attach()

    def _timeline_event(self, event):
        # Synchronous push callback: never wait a frame before changing references.
        if self.lifecycle is None:
            return
        if self.usd.get_stage() != self.stage:
            self._detach()
            return
        types = omni.timeline.TimelineEventType
        try:
            if event.type == int(types.PLAY):
                episode = self.lifecycle.play()
                if episode is not None:
                    carb.log_info(f"BFM collection episode object: asset={episode['asset']} "
                                  f"position={episode['position']} yaw={episode['yaw']}")
            elif event.type == int(types.PAUSE):
                self.lifecycle.pause()
            elif event.type == int(types.STOP):
                self.lifecycle.stop()
        except Exception as error:
            # A failed sample/composition/publisher must never leave old state alive.
            carb.log_error(f"BFM collection Play cancelled: {error}")
            try:
                self.lifecycle.stop()
            finally:
                self.timeline.stop()

    def on_shutdown(self):
        global _active
        self.stage_subscription = self.timeline_subscription = None
        self.update_subscription = self.shutdown_subscription = None
        self._detach()
        if _active is self:
            _active = None
