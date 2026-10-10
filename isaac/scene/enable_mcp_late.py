"""Kit --exec script: enable the Isaac Sim MCP extension once the app has finished starting up, then
optionally open a stage (HG1_OPEN_STAGE, set by launch_isaac.sh --open).

Enabling it on the command line (--enable) crashed Isaac Sim 6.0 during startup on this machine
(NULL UsdStage in omni.usd), and a stage path given on the command line is not opened, so both happen here.
"""

import asyncio
import os

import omni.kit.app
import omni.usd


async def _enable_later(frames=120):
    app = omni.kit.app.get_app()
    for _ in range(frames):
        await app.next_update_async()
    app.get_extension_manager().set_extension_enabled_immediate("isaac.sim.mcp_extension", True)
    print("[Hospital_G1] isaac.sim.mcp_extension enabled")
    stage = os.environ.get("HG1_OPEN_STAGE")
    if stage:
        await omni.usd.get_context().open_stage_async(stage)
        print(f"[Hospital_G1] opened {stage}")


asyncio.ensure_future(_enable_later())
