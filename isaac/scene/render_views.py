"""Render the scene's cameras to renders/<camera>.png from inside Isaac Sim.

Over MCP:  import runpy; runpy.run_path("<repo>/scene/render_views.py", run_name="__main__")
Starts an async task (so the MCP call returns immediately) that opens usd/hospital_g1.usd if needed,
waits for loading, renders each camera, and writes progress to renders/status.json.
Set RENDER_CAMERAS (comma separated) to render a subset, RENDER_REOPEN=1 to reload the stage, and
RENDER_RESOLUTION=WxH to resize the viewport (resizing reallocates every RTX buffer; avoid on 8 GB GPUs).
"""

import asyncio
import json
import os
import time

import omni.kit.app
import omni.usd
from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ROOT = os.path.join(REPO, "usd", "hospital_g1.usd")
OUT = os.path.join(REPO, "renders")
SETTLE_FRAMES = 90  # RTX accumulation + texture streaming per view


def _status(**kw):
    os.makedirs(OUT, exist_ok=True)
    kw = {"time": time.strftime("%H:%M:%S"), **kw}
    with open(os.path.join(OUT, "status.json"), "w") as f:
        json.dump(kw, f, indent=1)
    with open(os.path.join(OUT, "status.log"), "a") as f:
        f.write(json.dumps(kw) + "\n")


async def _frames(n):
    app = omni.kit.app.get_app()
    for _ in range(n):
        await app.next_update_async()


async def run(cameras=None, reopen=False):
    ctx = omni.usd.get_context()
    stage = ctx.get_stage()
    if reopen or stage is None or os.path.abspath(stage.GetRootLayer().realPath or "") != ROOT:
        _status(state="opening")
        await ctx.open_stage_async(ROOT)
    _status(state="loading")
    while True:
        _, loaded, total = ctx.get_stage_loading_status()
        if total == 0 or loaded >= total:
            break
        await _frames(10)
    await _frames(120)
    stage = ctx.get_stage()
    if cameras is None:
        cameras = [p.GetName() for p in stage.GetPrimAtPath("/World/Cameras").GetChildren()]
    vp = get_active_viewport()
    if os.environ.get("RENDER_RESOLUTION"):  # resizing reallocates every RTX buffer + DLSS; opt-in only
        vp.resolution = tuple(int(v) for v in os.environ["RENDER_RESOLUTION"].split("x"))
    done = []
    for cam in cameras:
        _status(state="rendering", camera=cam, done=done)
        vp.camera_path = f"/World/Cameras/{cam}"
        await _frames(SETTLE_FRAMES)
        path = os.path.join(OUT, f"{cam}.png")
        cap = capture_viewport_to_file(vp, file_path=path)
        await cap.wait_for_result()
        await _frames(5)
        done.append(cam)
    _status(state="finished", done=done)


if __name__ == "__main__":
    subset = os.environ.get("RENDER_CAMERAS")
    asyncio.ensure_future(run(subset.split(",") if subset else None, reopen=bool(os.environ.get("RENDER_REOPEN"))))
