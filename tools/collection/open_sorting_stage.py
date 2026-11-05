"""Isaac --exec startup hook: open the repository stage without PLAY."""
import asyncio
from pathlib import Path

import omni.kit.app
import omni.timeline
import omni.usd


async def open_sorting_stage():
    # Let Isaac's startup stage creation finish before replacing it.
    await omni.kit.app.get_app().next_update_async()
    omni.timeline.get_timeline_interface().stop()
    stage = Path(__file__).resolve().parents[2] / 'models/stages/stage_sorting_v3.usda'
    opened, error = await omni.usd.get_context().open_stage_async(str(stage))
    if not opened:
        raise RuntimeError(f'Cannot open sorting stage {stage}: {error}')


asyncio.ensure_future(open_sorting_stage())
