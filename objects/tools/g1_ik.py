"""Small damped-least-squares IK for the G1 arm (MuJoCo kinematics), used by the reach check and grasp tests.

Grasp frame of a hand: origin = midpoint of the thumb and middle-finger pads of the open hand (home pose), shifted
`fwd` metres along the finger direction; orientation = the home wrist orientation levelled so the fingers point
horizontally forward (palm faces the body mid-plane, finger stack vertical -> a wrap grasp of an upright object).
"""
import mujoco
import numpy as np

ARM = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw")
WAIST = ("waist_yaw", "waist_roll", "waist_pitch")


class ArmIK:
    def __init__(self, m, side, stance_xyyaw, waist=False, fwd=0.0, pelvis_z=0.784):
        self.m, self.side = m, side
        self.d = mujoco.MjData(m)
        d = self.d
        x, y, yaw = stance_xyyaw
        self.yaw = yaw
        mujoco.mj_resetDataKeyframe(m, d, 0)
        self.free = m.body("pelvis").jntnum[0] > 0
        if self.free:
            d.qpos[:3] = [x, y, pelvis_z]
            d.qpos[3:7] = [np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]
        self.joints = (list(WAIST) if waist else []) + [f"{side}_{j}" for j in ARM]
        jid = [m.joint(j + "_joint").id for j in self.joints]
        self.qa = np.array([m.jnt_qposadr[j] for j in jid])
        self.da = np.array([m.jnt_dofadr[j] for j in jid])
        self.lo, self.hi = m.jnt_range[jid, 0], m.jnt_range[jid, 1]
        self.wrist = m.body(f"{side}_wrist_yaw_link").id
        mujoco.mj_kinematics(m, d)
        R0 = d.xmat[self.wrist].reshape(3, 3).copy()
        pc = (d.site(f"{side}_thumb_touch_link").xpos + d.site(f"{side}_middle_touch_link").xpos) / 2
        fd = d.site(f"{side}_middle_tip_link").xpos - d.xpos[self.wrist]
        fwd_w = np.array([np.cos(yaw), np.sin(yaw), 0])
        lat = np.array([-np.sin(yaw), np.cos(yaw), 0])
        ang = np.arctan2(-fd[2], fd @ fwd_w)
        K = np.array([[0, -lat[2], lat[1]], [lat[2], 0, -lat[0]], [-lat[1], lat[0], 0]])
        self.R_level = (np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * K @ K) @ R0
        f_local = R0.T @ (fd / np.linalg.norm(fd))
        self.off = R0.T @ (pc - d.xpos[self.wrist]) + fwd * f_local  # grasp point in the wrist frame
        self.q0 = d.qpos.copy()

    def grasp_point(self, q=None):
        d = self.d
        if q is not None:
            d.qpos[self.qa] = q
        mujoco.mj_kinematics(self.m, d)
        R = d.xmat[self.wrist].reshape(3, 3)
        return d.xpos[self.wrist] + R @ self.off

    def solve(self, target, R_target=None, q_init=None, iters=300, w_rot=0.3):
        m, d = self.m, self.d
        R_target = self.R_level if R_target is None else R_target
        d.qpos[:] = self.q0
        if q_init is not None:
            d.qpos[self.qa] = q_init
        for _ in range(iters):
            mujoco.mj_kinematics(m, d)
            mujoco.mj_comPos(m, d)
            R = d.xmat[self.wrist].reshape(3, 3)
            p = d.xpos[self.wrist] + R @ self.off
            ep = target - p
            er = 0.5 * sum(np.cross(R[:, k], R_target[:, k]) for k in range(3))
            Jp, Jr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
            mujoco.mj_jac(m, d, Jp, Jr, p, self.wrist)
            J = np.vstack([Jp[:, self.da], w_rot * Jr[:, self.da]])
            e = np.r_[ep, w_rot * er]
            dq = J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(6), e)
            d.qpos[self.qa] = np.clip(d.qpos[self.qa] + 0.5 * dq, self.lo, self.hi)
        margin = float(np.min(np.minimum(d.qpos[self.qa] - self.lo, self.hi - d.qpos[self.qa])))
        return d.qpos[self.qa].copy(), float(np.linalg.norm(ep)), float(np.degrees(np.linalg.norm(er))), margin
