#!/usr/bin/env python3
"""Checks for the generated objects (run by build.sh; the hospital-sim .venv python with mujoco:
    .venv/bin/python hospital-sim-env/objects/check_objects.py   # from the hospital-sim root).

1. every per-object MJCF and theatre_objects.xml compiles;
2. settle test: 3 s of physics with every instrument standing in the rack -> each instrument must stay
   upright (tilt < 12 deg), stay in its slot (xy drift < 15 mm) and come to rest;
3. preview renders -> docs/ (skip with --no-render).
Writes docs/check_report.json. Exit 1 on failure.
"""
import argparse
import json
import os
import sys
from pathlib import Path

import numpy as np

os.environ.setdefault("MUJOCO_GL", "egl")
import mujoco  # noqa: E402

HERE = Path(__file__).resolve().parent
DOCS = HERE / "docs"


def tilt_deg(q):
    w, x, y, z = q
    zz = 1 - 2 * (x * x + y * y)  # z-axis z-component
    return float(np.degrees(np.arccos(np.clip(zz, -1, 1))))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-render", action="store_true")
    a = ap.parse_args()
    cat = json.loads((HERE / "catalog.json").read_text())
    report = {"compile": {}, "settle": {}, "ok": True}
    for f in sorted((HERE / "mjcf").glob("*.xml")):
        try:
            m = mujoco.MjModel.from_xml_path(str(f))
            report["compile"][f.name] = dict(ok=True, nbody=m.nbody, ngeom=m.ngeom)
        except Exception as e:  # noqa: BLE001
            report["compile"][f.name] = dict(ok=False, error=str(e))
            report["ok"] = False
    m = mujoco.MjModel.from_xml_path(str(HERE / "mjcf" / "theatre_objects.xml"))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    names = cat["labels"]
    start = {n: d.body(n).xpos.copy() for n in names}
    for _ in range(1500):
        mujoco.mj_step(m, d)
    for n in names:
        b = d.body(n)
        jnt = m.body(n).jntadr[0]
        vel = np.linalg.norm(d.qvel[m.jnt_dofadr[jnt]:m.jnt_dofadr[jnt] + 3])
        drift = float(np.linalg.norm(b.xpos[:2] - start[n][:2]))
        t = tilt_deg(b.xquat)
        ok = bool(t < 12 and drift < 0.015 and vel < 0.01)
        report["settle"][n] = dict(tilt_deg=round(t, 2), xy_drift_mm=round(drift * 1000, 2),
                                   z_drop_mm=round(float(start[n][2] - b.xpos[2]) * 1000, 2), speed=round(float(vel), 5), ok=ok)
        report["ok"] &= ok
    if not a.no_render:
        DOCS.mkdir(exist_ok=True)
        render(m, d, cat)
    (DOCS / "check_report.json").parent.mkdir(exist_ok=True)
    (DOCS / "check_report.json").write_text(json.dumps(report, indent=1))
    for k, v in report["settle"].items():
        print(f"  settle {k:16s} tilt {v['tilt_deg']:5.2f} deg  drift {v['xy_drift_mm']:5.2f} mm  "
              f"drop {v['z_drop_mm']:5.2f} mm  {'OK' if v['ok'] else 'FAIL'}")
    bad = [k for k, v in report["compile"].items() if not v["ok"]]
    print(f"compile: {len(report['compile']) - len(bad)}/{len(report['compile'])} OK {bad}")
    print("CHECK", "PASS" if report["ok"] else "FAIL")
    return 0 if report["ok"] else 1


def render(m, d, cat):
    from PIL import Image

    m.vis.global_.offwidth, m.vis.global_.offheight = 1280, 960
    r = mujoco.Renderer(m, 720, 960)
    opt = mujoco.MjvOption()
    rack = np.asarray(cat["world_poses"]["tool_rack"]["pos"])
    views = {
        "theatre_setup.png": dict(lookat=rack + [0, 0.4, 0.2], distance=2.6, azimuth=135, elevation=-22),
        "rack_closeup.png": dict(lookat=rack + [0, 0, 0.10], distance=0.75, azimuth=90, elevation=-25),
        "rack_robot_view.png": dict(lookat=rack + [0, 0, 0.08], distance=0.62, azimuth=90, elevation=-50),
    }
    for fn, v in views.items():
        cam = mujoco.MjvCamera()
        cam.lookat[:] = v["lookat"]
        cam.distance, cam.azimuth, cam.elevation = v["distance"], v["azimuth"], v["elevation"]
        r.update_scene(d, cam, opt)
        Image.fromarray(r.render()).save(DOCS / fn)
    # instruments laid out side by side (canonical frames) in a separate lineup model
    parts = ['<mujoco><compiler angle="radian"/><asset>']
    for n in cat["labels"]:
        parts.append(f'<model name="{n}" file="{HERE / "mjcf" / (n + ".xml")}"/>')
    parts.append('</asset><visual><global offwidth="1600" offheight="700"/></visual><worldbody>'
                 '<light pos="0 -0.3 1" dir="0 0.3 -1"/><geom type="plane" size="1 1 .1" rgba=".95 .95 .95 1"/>')
    for i, n in enumerate(cat["labels"]):
        parts.append(f'<frame pos="{(i - 3) * 0.075} 0 0.02"><attach model="{n}" body="{n}" prefix=""/></frame>')
    parts.append('</worldbody></mujoco>')
    lm = mujoco.MjModel.from_xml_string("".join(parts))
    lm.opt.gravity[:] = 0
    ld = mujoco.MjData(lm)
    mujoco.mj_forward(lm, ld)
    rr = mujoco.Renderer(lm, 700, 1600)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [0, 0, 0.11]
    cam.distance, cam.azimuth, cam.elevation = 0.75, 90, -5
    rr.update_scene(ld, cam, opt)
    Image.fromarray(rr.render()).save(DOCS / "instrument_lineup.png")


if __name__ == "__main__":
    sys.exit(main())
