#!/usr/bin/env python3
"""Reach check: G1 pelvis fixed at the placements.yaml stance; damped-least-squares IK puts each hand's grasp point
(tools/g1_ik.py) on every instrument's grasp site with the fingers levelled forward (wrap grasp of an upright
instrument). Arm only (waist at 0) and arm + waist. Writes docs/reach_report.json.

    .venv/bin/python hospital-sim-env/objects/tools/reach_check.py   # from the hospital-sim root
"""
import json
import sys
from pathlib import Path

import mujoco
import numpy as np
import yaml

REPO = Path(__file__).resolve().parents[3]      # hospital-sim root
OBJ = Path(__file__).resolve().parents[1]       # hospital-sim-env/objects
sys.path.insert(0, str(Path(__file__).resolve().parent))
from g1_ik import ArmIK  # noqa: E402

cat = json.loads((OBJ / "catalog.json").read_text())
pl = yaml.safe_load((OBJ / "placements.yaml").read_text())
m = mujoco.MjModel.from_xml_path(str(REPO / "g1-brainco-description/g1_29dof_brainco/mjcf/g1_29dof_brainco.xml"))
stance = pl["robot_stances"]["theatre_instrument_table"]

out = {"stance": stance}
for side in ("right", "left"):
    for waist in (False, True):
        ik = ArmIK(m, side, stance, waist=waist)
        res = {}
        for n in cat["labels"]:
            g = np.asarray(cat["world_poses"][n]["pos"]) + np.array([0, 0, cat["instruments"][n]["grasp"]["site"][2]])
            q, ep, er, margin = ik.solve(g)
            res[n] = dict(pos_err_mm=round(ep * 1000, 1), rot_err_deg=round(er, 1), min_joint_margin_rad=round(margin, 3),
                          q=dict(zip(ik.joints, np.round(q, 3).tolist())))
        out[side + ("_with_waist" if waist else "")] = res
(OBJ / "docs/reach_report.json").write_text(json.dumps(out, indent=1))
print(f"{'slot':16s} {'right arm':>16s} {'left arm':>16s} {'right+waist':>16s} {'left+waist':>16s}   (pos mm / rot deg)")
for n in cat["labels"]:
    cells = [out[k][n] for k in ("right", "left", "right_with_waist", "left_with_waist")]
    print(f"{n:16s} " + " ".join(f"{c['pos_err_mm']:7.1f} / {c['rot_err_deg']:4.1f}" .rjust(16) for c in cells))
