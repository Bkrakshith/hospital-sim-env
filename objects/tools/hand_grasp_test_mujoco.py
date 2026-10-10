#!/usr/bin/env python3
"""In-hand grasp stability test (MuJoCo); the Isaac twin is g1-isaac-sim/tools/hand_grasp_test.py.

A fixed-base G1 (pelvis welded at the origin) holds its right hand in free space, fingers levelled forward. Each
instrument is placed upright in the open hand (its grasp site at a point in the wrist frame, pinned while the
fingers close, then 0.3 s gravity-free so the contacts settle), the hand closes (0.6 s), the instrument is released and the instrument must stay in the hand:
  hold 1.5 s  ->  shake: wrist pitch +-0.35 rad and shoulder pitch +-0.25 rad at 1 Hz for 3 s  ->  hold 1 s.
Metrics (instrument pose in the wrist frame): settle slip from release (seating), shake slip once seated
(from the start of the shake), max finger joint speed. Held (retained) = < 80 mm from the release pose
at all times; firm = < 10 mm slip during the shake.
The chosen in-hand pose + arm pose are written to docs/hand_grasp_spec.json for the Isaac twin.

    .venv/bin/python hospital-sim-env/objects/tools/hand_grasp_test_mujoco.py            # all instruments, writes the report
    .venv/bin/python hospital-sim-env/objects/tools/hand_grasp_test_mujoco.py --search   # grid-search the in-hand position first
(from the hospital-sim root, which holds the sibling repo g1-brainco-description)
"""
import argparse
import json
import sys
from pathlib import Path

import mujoco
import numpy as np

REPO = Path(__file__).resolve().parents[3]      # hospital-sim root
OBJ = Path(__file__).resolve().parents[1]       # hospital-sim-env/objects
sys.path.insert(0, str(Path(__file__).resolve().parent))
from g1_ik import ArmIK  # noqa: E402

SIDE = "right"
# Pinch grasp: with thumb_aux 0.8, thumb 0.6 and index/middle 0.55 the thumb pad meets the index/middle pads at
# (142, 45, 15) mm in the wrist frame. The instrument sits there and the targets over-close past contact, so the
# position servos squeeze it (force ~ kp x residual angle, capped by the actuator force range).
PINCH_POINT = [0.142, 0.045, 0.015]
HAND_OPEN = [0.0, 0.8, 0.0, 0.0, 0.0, 0.0]
HAND_CLOSED = [0.9, 0.8, 0.8, 0.8, 1.0, 1.0]
HAND_TARGET_W = [0.33, -0.15, 0.92]    # grasp point in the world (robot at the origin, facing +x)
T_CLOSE, T_SOAK, T_HOLD, T_SHAKE, T_END = 0.6, 0.3, 1.5, 3.0, 1.0


def quat_from_R(R):
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, R.ravel())
    return q


def pinch_height(inst):
    """Pinch at the centre-of-mass height (minimal gravity torque about the pinch), clamped to the grasp region."""
    c = json.loads((OBJ / "catalog.json").read_text())["instruments"][inst]
    lo, hi = c["grasp"]["z_range"]
    return float(np.clip(c["com"][2], lo, hi))


def build(inst, q_arm, joints, rel_pos, rel_R):
    """World: floor + robot (fixed) + the instrument placed at wrist-frame pose (rel_pos, rel_R)."""
    world = mujoco.MjSpec()
    world.option.timestep = 0.002
    world.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    world.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    world.option.impratio = 10
    world.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_PLANE, size=[3, 3, 0.1])
    robot = mujoco.MjSpec.from_file(str(REPO / "g1-brainco-description/g1_29dof_brainco/mjcf/g1_29dof_brainco.xml"))
    pel = robot.body("pelvis")
    for j in list(pel.joints):
        robot.delete(j)
    for k in list(robot.keys):
        robot.delete(k)
    pel.pos = [0, 0, 0]
    world.worldbody.add_frame(pos=[0, 0, 0.784]).attach_body(pel, "", "")
    obj = mujoco.MjSpec.from_file(str(OBJ / f"mjcf/{inst}.xml"))
    obj.body(inst).gravcomp = 1.0  # nonzero at compile time so it can be toggled at runtime
    world.worldbody.add_frame().attach_body(obj.body(inst), "", "")
    m = world.compile()
    d = mujoco.MjData(m)
    mk = mujoco.MjModel.from_xml_path(str(REPO / "g1-brainco-description/g1_29dof_brainco/mjcf/g1_29dof_brainco.xml"))
    dk = mujoco.MjData(mk)
    mujoco.mj_resetDataKeyframe(mk, dk, 0)
    for j in range(mk.njnt):
        if mk.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE:
            d.qpos[m.jnt_qposadr[m.joint(mk.joint(j).name).id]] = dk.qpos[mk.jnt_qposadr[j]]
    for n, v in zip(joints, q_arm):
        d.qpos[m.jnt_qposadr[m.joint(n + "_joint").id]] = v
    mujoco.mj_kinematics(m, d)
    w = m.body(f"{SIDE}_wrist_yaw_link").id
    Rw = d.xmat[w].reshape(3, 3)
    grasp_z = pinch_height(inst)
    Ro = Rw @ rel_R
    po = d.xpos[w] + Rw @ rel_pos - Ro @ np.array([0, 0, grasp_z])  # body origin so that the grasp site lands there
    fj = m.body(inst).jntadr[0]
    a = m.jnt_qposadr[fj]
    d.qpos[a:a + 3] = po
    d.qpos[a + 3:a + 7] = quat_from_R(Ro)
    return m, d


def run(inst, q_arm, joints, rel_pos, rel_R, video=None):
    m, d = build(inst, q_arm, joints, rel_pos, rel_R)
    mujoco.mj_forward(m, d)
    pen = min([d.contact[i].dist for i in range(d.ncon)
               if inst in (m.body(m.geom_bodyid[d.contact[i].geom1]).name, m.body(m.geom_bodyid[d.contact[i].geom2]).name)] or [0.0])
    act_j = [m.actuator(i).trnid[0] for i in range(29)]
    qadr = np.array([m.jnt_qposadr[j] for j in act_j])
    dadr = np.array([m.jnt_dofadr[j] for j in act_j])
    q_ref = d.qpos[qadr].copy()
    names = [m.joint(j).name[:-6] for j in act_j]
    i_wp = names.index(f"{SIDE}_wrist_pitch")
    i_sp = names.index(f"{SIDE}_shoulder_pitch")
    hand_act = [m.actuator(f"{SIDE}_hand_{k}").id for k in ("thumb", "thumb_aux", "index", "middle", "ring", "pinky")]
    hand_hi = m.actuator_ctrlrange[hand_act, 1]
    finger_dofs = [m.jnt_dofadr[m.actuator(a).trnid[0]] for a in hand_act]
    w = m.body(f"{SIDE}_wrist_yaw_link").id
    b = m.body(inst).id
    fj = m.body(inst).jntadr[0]
    qa, va = m.jnt_qposadr[fj], m.jnt_dofadr[fj]
    q_hold = d.qpos[qa:qa + 7].copy()
    dt = m.opt.timestep
    total = T_CLOSE + T_HOLD + T_SHAKE + T_END
    rel0, relR0, max_slip, max_rot, max_fv = None, None, 0.0, 0.0, 0.0
    rel1, relR1, shake_slip, shake_rot = None, None, 0.0, 0.0
    frames = []
    renderer = None
    if video:
        import os
        os.environ.setdefault("MUJOCO_GL", "egl")
        renderer = mujoco.Renderer(m, 360, 480)
    for k in range(int(total / dt)):
        t = k * dt
        tgt = q_ref.copy()
        if T_CLOSE + T_HOLD <= t < T_CLOSE + T_HOLD + T_SHAKE:
            ph = 2 * np.pi * (t - T_CLOSE - T_HOLD)
            tgt[i_wp] += 0.35 * np.sin(ph)
            tgt[i_sp] += 0.25 * np.sin(ph)
        tau = 150.0 * (tgt - d.qpos[qadr]) - 5.0 * d.qvel[dadr]
        d.ctrl[:29] = np.clip(tau, m.actuator_ctrlrange[:29, 0], m.actuator_ctrlrange[:29, 1])
        s = min(1.0, t / (0.8 * T_CLOSE))
        d.ctrl[hand_act] = ((1 - s) * np.asarray(HAND_OPEN) + s * np.asarray(HAND_CLOSED)) * hand_hi
        # pinned while the fingers close, then 0.3 s gravity-free (contacts settle without a pop), then gravity on
        m.body_gravcomp[b] = 1.0 if t < T_CLOSE + T_SOAK else 0.0
        mujoco.mj_step(m, d)
        if t < T_CLOSE:
            d.qpos[qa:qa + 7] = q_hold
            d.qvel[va:va + 6] = 0
        Rw = d.xmat[w].reshape(3, 3)
        rel = Rw.T @ (d.xpos[b] - d.xpos[w])
        relR = Rw.T @ d.xmat[b].reshape(3, 3)
        if t >= T_CLOSE:
            if rel0 is None:
                rel0, relR0 = rel.copy(), relR.copy()
            max_slip = max(max_slip, float(np.linalg.norm(rel - rel0)))
            c = (np.trace(relR0.T @ relR) - 1) / 2
            max_rot = max(max_rot, float(np.degrees(np.arccos(np.clip(c, -1, 1)))))
        if t >= T_CLOSE + T_HOLD:  # seated grasp: slip during the shake + final hold
            if rel1 is None:
                rel1, relR1 = rel.copy(), relR.copy()
            shake_slip = max(shake_slip, float(np.linalg.norm(rel - rel1)))
            c = (np.trace(relR1.T @ relR) - 1) / 2
            shake_rot = max(shake_rot, float(np.degrees(np.arccos(np.clip(c, -1, 1)))))
        max_fv = max(max_fv, float(np.abs(d.qvel[finger_dofs]).max()))
        if renderer is not None and k % 25 == 0:
            cam = mujoco.MjvCamera()
            cam.lookat[:] = d.xpos[w] + Rw @ np.array([0.12, 0.03, 0])
            cam.distance, cam.azimuth, cam.elevation = 0.45, 150, -20
            renderer.update_scene(d, cam)
            frames.append(renderer.render())
    if renderer is not None:
        import imageio
        imageio.mimsave(video, frames, fps=20)
    held = max_slip < 0.08  # retained: still in the hand (< 80 mm from where it was released) after the shake
    firm = shake_slip < 0.01
    return dict(instrument=inst, held=bool(held), firm=bool(firm), settle_slip_mm=round(max_slip * 1000, 1), settle_rot_deg=round(max_rot, 1),
                shake_slip_mm=round(shake_slip * 1000, 1), shake_rot_deg=round(shake_rot, 1),
                max_finger_speed=round(max_fv, 2), initial_penetration_mm=round(-pen * 1000, 1))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--search", action="store_true")
    ap.add_argument("--video", default=None, help="mp4 of the scissors test")
    a = ap.parse_args()
    cat = json.loads((OBJ / "catalog.json").read_text())
    mk = mujoco.MjModel.from_xml_path(str(REPO / "g1-brainco-description/g1_29dof_brainco/mjcf/g1_29dof_brainco.xml"))
    ik = ArmIK(mk, SIDE, (0.0, 0.0, 0.0))
    q_arm, ep, er, _ = ik.solve(np.array(HAND_TARGET_W))
    # instrument canonical z (long axis) along the wrist z (finger stack, index on top); its wide x along the fingers
    rel_R = np.eye(3)
    specf = OBJ / "docs/hand_grasp_spec.json"
    rel_pos = np.array(PINCH_POINT)
    if a.search:
        best = None
        variants = {"A": [1.0, 0.8, 0.8, 0.8, 1.0, 1.0], "B": [1.0, 0.8, 0.8, 0.8, 0.2, 0.2], "C": [1.0, 0.8, 1.0, 1.0, 1.0, 1.0]}
        for dx, dy in ((0, 0), (-0.004, -0.004), (0.004, 0.004)):
            for rot in (0, 45, 90, 135):
                for hv, hc in variants.items():
                    HAND_CLOSED[:] = hc
                    c, s_ = np.cos(np.radians(rot)), np.sin(np.radians(rot))
                    R = np.array([[c, -s_, 0], [s_, c, 0], [0, 0, 1.0]])
                    pos = np.array(PINCH_POINT) + [dx, dy, 0]
                    rs = [run(n, q_arm, ik.joints, pos, R) for n in ("scissors", "scalpel_handle", "trocar", "veress_needle")]
                    score = sum(r["settle_slip_mm"] + 5 * r["shake_slip_mm"] + (100 if not r["held"] else 0) for r in rs)
                    print(f"d {dx},{dy} rot {rot} hand {hv}: " + ", ".join(
                        f"{r['instrument']} {r['settle_slip_mm']}/{r['shake_slip_mm']}mm" for r in rs), flush=True)
                    if best is None or score < best[0]:
                        best = (score, pos.tolist(), rot, hv, list(hc))
        _, pos, rot, hv, hc = best
        HAND_CLOSED[:] = hc
        rel_pos = np.array(pos)
        c, s_ = np.cos(np.radians(rot)), np.sin(np.radians(rot))
        rel_R = np.array([[c, -s_, 0], [s_, c, 0], [0, 0, 1.0]])
        print("best", best)
    elif specf.exists():
        sp = json.loads(specf.read_text())
        rel_pos, rel_R = np.asarray(sp["rel_pos"]), np.asarray(sp["rel_R"])
        HAND_CLOSED[:] = sp["hand_closed"]
    res = [run(n, q_arm, ik.joints, rel_pos, rel_R, video=a.video if n == "scissors" else None) for n in cat["labels"]]
    spec = dict(side=SIDE, joints=ik.joints, q_arm=q_arm.tolist(), ik_err_mm=round(ep * 1000, 2), rel_pos=rel_pos.tolist(),
                rel_R=rel_R.tolist(), hand_open=HAND_OPEN, hand_closed=HAND_CLOSED,
                phases=dict(close=T_CLOSE, hold=T_HOLD, shake=T_SHAKE, end=T_END, wrist_pitch_amp=0.35,
                            shoulder_pitch_amp=0.25, freq_hz=1.0), mujoco_results=res)
    specf.write_text(json.dumps(spec, indent=1))
    for r in res:
        print(json.dumps(r))
    return 0 if all(r["held"] for r in res) else 1


if __name__ == "__main__":
    sys.exit(main())
