"""Render the walkthrough frames with RTX path tracing from inside Isaac Sim.

Over MCP:  import runpy; runpy.run_path("<repo>/scene/render_walkthrough.py", run_name="__main__")
Starts an async task and returns immediately; progress goes to renders/walkthrough/status.json.
Frames land in renders/walkthrough/<shot>/0000.png... and existing frames are skipped (resumable).

Environment:
  WALK_SPP=128        path-tracing samples per frame (denoised)
  WALK_PREVIEW=1      render only the first, middle and last frame of each shot
  WALK_SHOTS=a,b      render a subset of shots
  WALK_GROUP=wing     which video stage to render (see walkthrough.VIDEO_STAGES); only its shots render
Run each group in a FRESH Isaac session (./launch_isaac.sh), so the other group's scene never loads.
"""

import asyncio
import json
import os
import sys
import time

import carb
import omni.kit.app
import omni.usd
from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
from pxr import Gf, Usd, UsdGeom

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import walkthrough as W  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GROUP = os.environ.get("WALK_GROUP", "wing")
ROOT = os.path.join(REPO, "usd", f"video_{GROUP}.usd")
OUT = os.path.join(REPO, "renders", "walkthrough")
CAM = "/World/Cameras/Cam_Walkthrough"
SPP_PER_UPDATE = 16


def _status(**kw):
    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, "status.json"), "w") as f:
        json.dump({"time": time.strftime("%H:%M:%S"), **kw}, f, indent=1)


async def _frames(n):
    app = omni.kit.app.get_app()
    for _ in range(n):
        await app.next_update_async()


def _path_tracing(spp):
    s = carb.settings.get_settings()
    s.set("/rtx/rendermode", "PathTracing")
    s.set("/rtx/pathtracing/spp", SPP_PER_UPDATE)
    s.set("/rtx/pathtracing/totalSpp", spp)
    s.set("/rtx/pathtracing/optixDenoiser/enabled", True)
    s.set("/rtx/post/histogram/enabled", False)


async def run():
    spp = int(os.environ.get("WALK_SPP", "128"))
    preview = bool(os.environ.get("WALK_PREVIEW"))
    wanted = os.environ.get("WALK_SHOTS")
    ctx = omni.usd.get_context()
    stage = ctx.get_stage()
    if stage is None or os.path.abspath(stage.GetRootLayer().realPath or "") != ROOT:
        _status(state="opening")
        await ctx.open_stage_async(ROOT)
        stage = ctx.get_stage()
    while True:
        _, loaded, total = ctx.get_stage_loading_status()
        if total == 0 or loaded >= total:
            break
        await _frames(10)

    # The walkthrough camera lives in the session layer, so the saved scene is untouched.
    stage.SetEditTarget(Usd.EditTarget(stage.GetSessionLayer()))
    cam = UsdGeom.Camera.Define(stage, CAM)
    cam.GetHorizontalApertureAttr().Set(20.955)
    cam.GetVerticalApertureAttr().Set(20.955 * W.RESOLUTION[1] / W.RESOLUTION[0])
    cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.05, 1000.0))
    xform = cam.MakeMatrixXform()

    _path_tracing(spp)
    vp = get_active_viewport()
    vp.resolution = W.RESOLUTION
    vp.camera_path = CAM
    await _frames(60)
    await vp.wait_for_rendered_frames(10)

    n = W.frames_per_shot()
    shots = [s for s in W.SHOTS if W.stage_group(s[0]) == GROUP and (not wanted or s[0] in wanted.split(","))]
    todo = [(s, i) for s in shots for i in ((0, n // 2, n - 1) if preview else range(n))]
    started, done = time.time(), 0
    for shot, i in todo:
        name = shot[0]
        path = os.path.join(OUT, name, f"{i:04d}.png")
        if os.path.exists(path):
            continue
        os.makedirs(os.path.dirname(path), exist_ok=True)
        eye, target = W.pose(shot, i)
        view = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(0, 0, 1))
        xform.Set(view.GetInverse())
        cam.GetFocalLengthAttr().Set(float(shot[2]))
        # Wait on frames the GPU actually delivered: path-traced frames lag app updates, so counting updates
        # captured images from earlier camera poses. The tracer restarts accumulation when the camera moves.
        await vp.wait_for_rendered_frames(spp // SPP_PER_UPDATE + 4)
        cap = capture_viewport_to_file(vp, file_path=path)
        await cap.wait_for_result()
        done += 1
        per = (time.time() - started) / done
        _status(state="rendering", shot=name, frame=i, done=done, total=len(todo),
                sec_per_frame=round(per, 2), eta_min=round(per * (len(todo) - done) / 60, 1))
    _status(state="finished", total=len(todo), minutes=round((time.time() - started) / 60, 1))


if __name__ == "__main__":
    asyncio.ensure_future(run())
