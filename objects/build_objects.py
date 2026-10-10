#!/usr/bin/env python3
"""Regenerate every generated file under hospital-sim-env/objects/: meshes, MJCF, USD, catalog.

    hospital-sim-env/objects/build.sh            (= uv run ... python hospital-sim-env/objects/build_objects.py)

Inputs: the i4h instrument USDs (through the sibling hospital-sim-env/isaac/assets/i4h switch, lite or full), the procedural
definitions below and placements.yaml. Outputs (paths relative to this folder; never edit by hand):

    meshes/<name>/visual_<k>.obj      visual meshes in the object's canonical frame, one per colour
    meshes/<name>/collision_<k>.obj   convex parts (CoACD) for the real meshes
    mjcf/<name>.xml                   one MJCF model per object (attach with <model>/<attach>)
    mjcf/theatre_objects.xml          every object placed per placements.yaml (+ floor, loadable alone)
    usd/<name>.usd                    physics-ready USD per object (rigid body, mass, colliders)
    catalog.json                      dimensions, mass, inertia, grasp regions, rack slots, world poses

Canonical instrument frame: +z along the long axis towards the handle / ring end, origin on that axis at
the far (tip) end, so an instrument standing in the rack has identity orientation and z = 0 at its tip.
+x is the wider transverse direction (plane of the ring handles / jaws), +y the thin direction.
"""
from __future__ import annotations

import json
import math
import shutil
import sys
from pathlib import Path

import numpy as np
import trimesh
import yaml

HERE = Path(__file__).resolve().parent
REPO = HERE.parent                      # hospital-sim-env (this repo)
I4H = REPO / "isaac" / "assets" / "i4h"
MESH_DIR, MJCF_DIR, USD_DIR = HERE / "meshes", HERE / "mjcf", HERE / "usd"

COLLISION_MODE = "slices"     # real instruments: "slices" (stacked boxes, default) or "hulls" (CoACD convex parts)
INERTIA_FLOOR = 1e-6          # kg m^2, applied identically in MJCF and USD (solver conditioning)
INSTR_FRICTION = (0.8, 0.02, 0.0005)   # sliding, torsional, rolling (MuJoCo); USD uses static/dynamic 0.8/0.7
FURN_FRICTION = 0.6
STEEL = (0.74, 0.74, 0.77, 1.0)

# ------------------------------------------------------------------------------------------------ items
SURG_A = "Props/shared_OR_without_Mark/Collected_surgery_room_movie_with_heart_adjusted/Assets/Surgery Room/SurgicalInstruments_A/"
REAL = {
    "scalpel_handle": dict(
        title="Scalpel (handle with No. 10-type blade, 153 mm), blade down", src=SURG_A + "sm_scalpelshandle_a01_01.usd",
        include=["/sm_scalpelshandle_a01_01/sm_scalpelshandle_a01_01"], mass=0.025,
        source="i4h OR suite, SurgicalInstruments_A/sm_scalpelshandle_a01_01 (replaces the earlier procedural stand-in)",
        licence="Apache-2.0 (Isaac for Healthcare assets)"),
    "scissors": dict(
        title="Mayo scissors (curved, 190 mm)", src="Props/SurgicalInstruments/SurgicalScissors.usd",
        include=["/Root/SurgicalScissor/surgical_scissor/Object002"], mass=0.085,
        source="i4h-workflows scissor pick-and-place asset (Props/SurgicalInstruments/SurgicalScissors.usd)",
        licence="Apache-2.0 (Isaac for Healthcare assets)"),
    "angled_tweezers": dict(
        title="Angled tweezers (160 mm)", src="Props/Lightwheel/Assets/SurgicalTweezers/AngledTweezers001.usd",
        include=["/root/AngledTweezers001/Visuals"], mass=0.030, flip=True,  # angled tips down
        source="i4h Lightwheel AngledTweezers001", licence="CC BY-NC 4.0 (Lightwheel, via i4h)"),
    "veress_needle": dict(
        title="Veress (pneumoperitoneum) needle", src="Props/Lightwheel/Assets/PneumoperitoneumNeedle001/"
        "PneumoperitoneumNeedle001.usd", include=["/root/PneumoperitoneumNeedle001/Visuals"], mass=0.020,
        source="i4h Lightwheel PneumoperitoneumNeedle001 (needle body only, cap removed)",
        licence="CC BY-NC 4.0 (Lightwheel, via i4h)"),
    "trocar": dict(
        title="Laparoscopic trocar", src="Props/Lightwheel/Assets/Trocar002/Trocar002.usd",
        include=["/root/Trocar002/Visuals"], mass=0.060,
        source="i4h Lightwheel Trocar002", licence="CC BY-NC 4.0 (Lightwheel, via i4h)"),
}

STANDIN = {
    "adson_forceps": dict(title="Adson tissue forceps 120 mm (STAND-IN)", mass=0.016,
                          source="procedural stand-in (this repo)", licence="same as repo"),
    "needle_driver": dict(title="Mayo-Hegar needle driver 180 mm / 7 in (STAND-IN)", mass=0.062,
                          source="procedural stand-in (this repo)", licence="same as repo"),
}
INSTRUMENTS = ["scalpel_handle", "adson_forceps", "angled_tweezers", "scissors", "needle_driver",
               "veress_needle", "trocar"]

# Rack: slot row along rack +y, the robot stands on the rack's -x side.
RACK = dict(pitch=0.09, n=7, depth=0.12, base_t=0.008, guide_gap=0.040, guide_t=0.010, clearance=0.0025, tip_t=0.008,
            min_hole=0.006, end_margin=0.03)
TABLE = dict(top=0.78, lx=0.46, ly=0.76, tray_t=0.006, lip_h=0.02, lip_t=0.006, post_x=0.18, post_r=0.022)


# ------------------------------------------------------------------------------------------------ helpers
def quat_from_yaw(yaw):
    return (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))


def quat_from_matrix(R):
    q = trimesh.transformations.quaternion_from_matrix(np.block([[R, np.zeros((3, 1))], [np.zeros((1, 3)), 1]]))
    return q if q[0] >= 0 else -q  # wxyz


def quat_from_axis_angle(axis, ang):
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    return np.r_[math.cos(ang / 2), math.sin(ang / 2) * axis]


def quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def quat_to_matrix(q):
    return trimesh.transformations.quaternion_matrix(q)[:3, :3]


def pose_mat(pos, quat):
    T = np.eye(4)
    T[:3, :3] = quat_to_matrix(quat)
    T[:3, 3] = pos
    return T


def fmt(v, n=6):
    return " ".join(f"{x:.{n}g}" if abs(x) >= 1e-12 else "0" for x in np.ravel(v))


# Primitive: dict(type=box|capsule|sphere|cylinder, size=..., pos, quat, rgba, collide=True, visual=True)
# box size = half extents; capsule/cylinder size = (radius, half-length) along local z; sphere (radius,)
def prim_mesh(p):
    t = p["type"]
    if t == "box":
        m = trimesh.creation.box(extents=2 * np.asarray(p["size"]))
    elif t == "capsule":
        m = trimesh.creation.capsule(height=2 * p["size"][1], radius=p["size"][0], count=[16, 8])
    elif t == "cylinder":
        m = trimesh.creation.cylinder(radius=p["size"][0], height=2 * p["size"][1], sections=24)
    elif t == "sphere":
        m = trimesh.creation.icosphere(subdivisions=2, radius=p["size"][0])
    else:
        raise ValueError(t)
    m.apply_transform(pose_mat(p["pos"], p.get("quat", (1, 0, 0, 0))))
    return m


def P(t, size, pos, quat=(1, 0, 0, 0), rgba=STEEL, collide=True, visual=True):
    return dict(type=t, size=list(size), pos=list(pos), quat=list(quat), rgba=list(rgba), collide=collide,
                visual=visual)


def seg_capsule(a, b, r, rgba=STEEL, **kw):
    """Capsule primitive between points a and b."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    d = b - a
    L = np.linalg.norm(d)
    z = np.array([0, 0, 1.0])
    ax = np.cross(z, d / L)
    s = np.linalg.norm(ax)
    q = (1, 0, 0, 0) if s < 1e-9 else quat_from_axis_angle(ax, math.atan2(s, np.dot(z, d / L)))
    return P("capsule", (r, L / 2), (a + b) / 2, q, rgba, **kw)


def seg_box(a, b, hx, hy, rgba=STEEL, **kw):
    """Box whose local z runs from a to b (both in the x-z plane or general), half sizes hx, hy."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    d = b - a
    L = np.linalg.norm(d)
    z = np.array([0, 0, 1.0])
    ax = np.cross(z, d / L)
    s = np.linalg.norm(ax)
    q = (1, 0, 0, 0) if s < 1e-9 else quat_from_axis_angle(ax, math.atan2(s, np.dot(z, d / L)))
    return P("box", (hx, hy, L / 2), (a + b) / 2, q, rgba, **kw)


def ring_prims(center, R, r, plane="xz", n=10, rgba=STEEL):
    """Finger ring (torus) as n capsules; ring lies in `plane`."""
    c = np.asarray(center, float)
    pts = []
    for k in range(n + 1):
        t = 2 * math.pi * k / n
        if plane == "xz":
            pts.append(c + np.array([R * math.cos(t), 0, R * math.sin(t)]))
    return [seg_capsule(pts[k], pts[k + 1], r, rgba) for k in range(n)]


def torus_mesh(center, R, r, plane="xz"):
    m = trimesh.creation.torus(major_radius=R, minor_radius=r, major_sections=32, minor_sections=12)
    if plane == "xz":  # torus is created in the xy plane: rotate about x by 90 deg
        m.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [1, 0, 0]))
    m.apply_translation(center)
    return m


# ------------------------------------------------------------------------------------------------ stand-ins
def build_scalpel_handle():
    """No. 3 scalpel handle, 125 mm, blade fitting at the bottom (tip end), grip at the top."""
    prims = [
        P("box", (0.0020, 0.0007, 0.0110), (0, 0, 0.0110)),             # blade fitting tongue (22 mm)
        P("box", (0.0035, 0.0012, 0.0080), (0, 0, 0.0300)),             # neck
        P("box", (0.0050, 0.0014, 0.0425), (0, 0, 0.0805)),             # flat grip, 10 x 2.8 x 85 mm
    ]
    vis = []
    for k in range(9):  # knurled grip bands (visual only)
        vis.append(P("box", (0.0052, 0.0016, 0.0012), (0, 0, 0.066 + 0.005 * k), collide=False,
                     rgba=(0.55, 0.55, 0.58, 1)))
    return prims + vis


def build_adson_forceps():
    """Adson tissue forceps 120 mm: two tines spread along y, joined at the top; tips (z=0) 8 mm apart."""
    prims = []
    for s in (-1, 1):
        # tip section: narrow (3 mm wide) 40 mm; grip section: 8 mm wide
        a, m, b = np.array([0, s * 0.0040, 0.0]), np.array([0, s * 0.0034, 0.040]), np.array([0, s * 0.0014, 0.114])
        prims.append(seg_box(a, m, 0.0015, 0.0009))
        prims.append(seg_box(m, b, 0.0040, 0.0010))
    prims.append(P("box", (0.0040, 0.0025, 0.0040), (0, 0, 0.116)))     # spring joint
    for s in (-1, 1):  # serrated grip pads (visual)
        for k in range(6):
            prims.append(P("box", (0.0041, 0.0003, 0.0010), (0, s * 0.0034, 0.070 + 0.006 * k),
                           collide=False, rgba=(0.55, 0.55, 0.58, 1)))
    return prims


def build_needle_driver():
    """Mayo-Hegar needle driver 180 mm (7 in): jaws at the bottom, box lock, long shanks, finger rings at the top.
    The shanks run parallel (3 mm apart each side) up to 100 mm, then flare to the rings, so there is a ~60 mm
    straight grip below the rings (a hand-wide grasp region; the 140 mm version's rings caught the fingers)."""
    prims = [
        P("box", (0.0022, 0.0020, 0.0125), (0, 0, 0.0125)),            # closed jaws (25 mm)
        P("box", (0.0040, 0.0030, 0.0075), (0, 0, 0.0325)),            # box lock
        seg_capsule((-0.0025, 0, 0.040), (-0.0035, 0, 0.110), 0.0022),  # straight shanks
        seg_capsule((0.0025, 0, 0.040), (0.0035, 0, 0.110), 0.0022),
        seg_capsule((-0.0035, 0, 0.110), (-0.013, 0, 0.148), 0.0022),   # flare to the rings
        seg_capsule((0.0035, 0, 0.110), (0.013, 0, 0.148), 0.0022),
        P("box", (0.0020, 0.0015, 0.0060), (-0.0065, 0, 0.140), collide=False, rgba=(0.6, 0.6, 0.62, 1)),  # ratchet
        P("box", (0.0020, 0.0015, 0.0060), (0.0065, 0, 0.140), collide=False, rgba=(0.6, 0.6, 0.62, 1)),
    ]
    for s in (-1, 1):
        prims += [dict(q, visual=False) for q in ring_prims((s * 0.016, 0, 0.1665), 0.0105, 0.0022)]
    return prims, [torus_mesh((s * 0.016, 0, 0.1665), 0.0105, 0.0022) for s in (-1, 1)]


# ------------------------------------------------------------------------------------------------ real meshes
def extract_usd_mesh(spec):
    """Triangles of the include prims in stage metres, grouped by colour, plus the asset->metre transform."""
    from pxr import Usd, UsdGeom, UsdShade

    path = I4H / spec["src"]
    stage = Usd.Stage.Open(str(path))
    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    groups = {}  # rgba tuple -> list of (V, F)

    def mat_color(mat):
        if not mat:
            return STEEL
        for p in Usd.PrimRange(mat.GetPrim()):
            if not p.IsA(UsdShade.Shader):
                continue
            sh = UsdShade.Shader(p)
            col, tex = None, None
            for i in sh.GetInputs():
                n = i.GetBaseName()
                v = i.Get()
                if n in ("diffuseColor", "diffuse_color_constant") and v is not None:
                    col = tuple(float(x) for x in v)
                if n == "diffuse_texture" and v is not None and v.resolvedPath:
                    tex = v.resolvedPath
            if tex:
                from PIL import Image
                im = np.asarray(Image.open(tex).convert("RGB").resize((64, 64)), float) / 255.0
                c = im.reshape(-1, 3).mean(0)
                if col:
                    c = c * np.asarray(col)
                return (*[round(float(x), 3) for x in c], 1.0)
            if col:
                return (*[round(x, 3) for x in col], 1.0)
        return STEEL

    for inc in spec["include"]:
        root = stage.GetPrimAtPath(inc)
        assert root, f"{spec['src']}: missing {inc}"
        for p in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
            if not p.IsA(UsdGeom.Mesh):
                continue
            mesh = UsdGeom.Mesh(p)
            pts = np.asarray(mesh.GetPointsAttr().Get(), float)
            cnt = np.asarray(mesh.GetFaceVertexCountsAttr().Get())
            idx = np.asarray(mesh.GetFaceVertexIndicesAttr().Get())
            M = np.asarray(UsdGeom.Xformable(p).ComputeLocalToWorldTransform(Usd.TimeCode.Default()), float).T
            V = (pts @ M[:3, :3].T + M[:3, 3]) * mpu
            # fan triangulation, remember face index of each triangle
            tris, fidx = [], []
            off = np.r_[0, np.cumsum(cnt)[:-1]]
            for f, (o, c) in enumerate(zip(off, cnt)):
                for k in range(1, c - 1):
                    tris.append((idx[o], idx[o + k], idx[o + k + 1]))
                    fidx.append(f)
            tris, fidx = np.asarray(tris), np.asarray(fidx)
            face_col = np.full(len(cnt), -1)
            cols = []
            subsets = UsdGeom.Subset.GetAllGeomSubsets(mesh)
            for s in subsets:
                m, _ = UsdShade.MaterialBindingAPI(s.GetPrim()).ComputeBoundMaterial()
                cols.append(mat_color(m))
                face_col[np.asarray(s.GetIndicesAttr().Get(), int)] = len(cols) - 1
            m, _ = UsdShade.MaterialBindingAPI(p).ComputeBoundMaterial()
            cols.append(mat_color(m))
            face_col[face_col < 0] = len(cols) - 1
            for ci, col in enumerate(cols):
                sel = face_col[fidx] == ci
                if sel.any():
                    groups.setdefault(col, []).append((V, tris[sel]))
    out = {}
    for col, parts in groups.items():
        out[col] = trimesh.util.concatenate([trimesh.Trimesh(V, F, process=False) for V, F in parts])
    return out, mpu


def canonical_transform(mesh, flip=None):
    """4x4 transform (stage metres -> canonical instrument frame) and whether the long axis was flipped."""
    V = mesh.vertices
    ext = V.max(0) - V.min(0)
    order = np.argsort(-ext)
    long, wide, thin = order[0], order[1], order[2]
    R = np.zeros((3, 3))
    R[0, wide] = R[1, thin] = R[2, long] = 1
    if np.linalg.det(R) < 0:
        R[1] *= -1
    W = V @ R.T
    z = W[:, 2]
    lo, hi = z.min(), z.max()
    L = hi - lo

    def area(sel):
        s = W[sel]
        return np.ptp(s[:, 0]) * np.ptp(s[:, 1]) if len(s) else 0

    bottom, top = area(z < lo + 0.15 * L), area(z > hi - 0.15 * L)
    do_flip = (bottom > top) if flip is None else flip
    if do_flip:
        R = np.diag([-1, 1, -1]) @ R
        W = V @ R.T
    c = np.r_[(W[:, 0].max() + W[:, 0].min()) / 2, (W[:, 1].max() + W[:, 1].min()) / 2, W[:, 2].min()]
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = -c
    return T, bool(do_flip)


def slice_boxes(mesh, dz=0.010, pad=0.0005):
    """Collision as a stack of boxes: each is the canonical-axis-aligned bounds of the mesh section in a `dz` slice.
    Boxes are the most robust contact primitive in both PhysX and MuJoCo (thin curved CoACD hulls let PhysX
    push the Mayo scissors' blades through the rack guide); finger rings become solid boxes."""
    zmax = mesh.vertices[:, 2].max()
    prims = []
    z = 0.0
    while z < zmax - 1e-4:
        z1 = min(zmax, z + dz)
        pts = section_points(mesh, z, z1, n=5)
        if len(pts):
            lo, hi = pts[:, :2].min(0) - pad, pts[:, :2].max(0) + pad
            prims.append(P("box", ((hi[0] - lo[0]) / 2, (hi[1] - lo[1]) / 2, (z1 - z) / 2),
                           ((hi[0] + lo[0]) / 2, (hi[1] + lo[1]) / 2, (z + z1) / 2), visual=False))
        z = z1
    return prims


def convex_parts(mesh, max_hulls=12, threshold=0.04):
    import coacd

    coacd.set_log_level("error")
    parts = coacd.run_coacd(coacd.Mesh(mesh.vertices, mesh.faces), threshold=threshold,
                            max_convex_hull=max_hulls, preprocess_mode="auto", resolution=2000, seed=1)
    hulls = [limit_hull(trimesh.Trimesh(v, f).convex_hull) for v, f in parts]
    return hulls


def limit_hull(h, max_verts=60):
    """PhysX cooks convex hulls with <= 64 vertices by default (255 max) and simplifies larger ones itself; reduce
    here instead, so MuJoCo and PhysX collide with the identical hull."""
    faces = min(2 * max_verts, len(h.faces) - 1)
    for _ in range(12):
        if len(h.vertices) <= max_verts or faces < 20:
            break
        h = h.simplify_quadric_decimation(face_count=faces).convex_hull
        faces = int(faces * 0.8)
    return h


# ------------------------------------------------------------------------------------------------ mass props
def mass_props(meshes, mass):
    """Uniform-density mass properties of a set of (convex) meshes scaled to `mass`."""
    vols = np.array([abs(m.volume) for m in meshes])
    rho = mass / vols.sum()
    com = sum(m.center_mass * v for m, v in zip(meshes, vols)) / vols.sum()
    I = np.zeros((3, 3))
    for m, v in zip(meshes, vols):
        mm = m.copy()
        mm.density = rho
        Ic = mm.moment_inertia  # about its own COM
        d = mm.center_mass - com
        I += Ic + rho * v * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    w, Rv = np.linalg.eigh(I)
    w = np.maximum(w, INERTIA_FLOOR)
    if np.linalg.det(Rv) < 0:
        Rv[:, 2] *= -1
    return dict(mass=mass, com=com.tolist(), diag=w.tolist(), iquat=quat_from_matrix(Rv).tolist(),
                density=float(rho))


# ------------------------------------------------------------------------------------------------ build
def build_instruments():
    objs = {}
    for name, spec in REAL.items():
        groups, mpu = extract_usd_mesh(spec)
        allm = trimesh.util.concatenate(list(groups.values()))
        T, flipped = canonical_transform(allm, spec.get("flip"))
        vis = []
        for col, m in groups.items():
            m = m.copy()
            m.apply_transform(T)
            if len(m.faces) > 12000:
                m = m.simplify_quadric_decimation(face_count=12000)
            vis.append((col, m))
        whole = allm.copy()
        whole.apply_transform(T)
        if COLLISION_MODE == "hulls":
            hulls, sprims = convex_parts(whole), []
        else:
            hulls, sprims = [], slice_boxes(whole)
        objs[name] = dict(spec, kind="instrument", real=True, visual=vis, hulls=hulls, prims=sprims,
                          T_asset=T, mpu=mpu, flipped=flipped, whole=whole)
    stand = {"adson_forceps": build_adson_forceps()}
    nd_prims, nd_extra = build_needle_driver()
    stand["needle_driver"] = nd_prims
    for name, prims in stand.items():
        extra = nd_extra if name == "needle_driver" else []
        vis_by_col = {}
        for p in prims:
            if p["visual"]:
                vis_by_col.setdefault(tuple(p["rgba"]), []).append(prim_mesh(p))
        for m in extra:
            vis_by_col.setdefault(tuple(STEEL), []).append(m)
        vis = [(c, trimesh.util.concatenate(ms)) for c, ms in vis_by_col.items()]
        whole = trimesh.util.concatenate([m for _, m in vis])
        objs[name] = dict(STANDIN[name], kind="instrument", real=False, visual=vis, hulls=[],
                          prims=[p for p in prims if p["collide"]], whole=whole)
    for name, o in objs.items():
        coll = o["hulls"] or [prim_mesh(p) for p in o["prims"]]
        o["inertial"] = mass_props(coll, o["mass"])
        o["coll_mesh"] = trimesh.util.concatenate(coll)
        V = o["whole"].vertices
        o["bbox"] = [V.min(0).tolist(), V.max(0).tolist()]
        o["length"] = float(V[:, 2].max())
    return objs


def section_points(mesh, z0, z1, n=9):
    """Points of the mesh surface between heights z0 and z1 (plane sections + vertices in the band)."""
    pts = [mesh.vertices[(mesh.vertices[:, 2] > z0) & (mesh.vertices[:, 2] < z1)]]
    for z in np.linspace(z0, z1, n):
        sec = mesh.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1])
        if sec is not None:
            pts.append(np.asarray(sec.vertices))
    return np.vstack(pts)


def rack_geometry(instr):
    """Rack primitives (rack frame: origin at the bottom centre, row along +y, robot on the -x side) and slots."""
    r = RACK
    half_len = r["pitch"] * (r["n"] - 1) / 2 + r["end_margin"] + 0.02
    dx = r["depth"] / 2
    z_base = r["base_t"]
    zg0 = z_base + r["guide_gap"]
    zg1 = zg0 + r["guide_t"]
    wood = (0.82, 0.84, 0.86, 1)   # powder-coated / sterilisable polymer, light grey
    accent = (0.15, 0.45, 0.65, 1)
    prims = [P("box", (dx, half_len, z_base / 2), (0, 0, z_base / 2), rgba=wood)]
    for s in (-1, 1):  # end posts
        prims.append(P("box", (dx, 0.01, (zg1 - z_base) / 2), (0, s * (half_len - 0.01), (z_base + zg1) / 2), rgba=wood))
    slots = {}
    holes, tip_holes = [], []
    for i, name in enumerate(INSTRUMENTS):
        y = (i - (r["n"] - 1) / 2) * r["pitch"]
        # Holes are sized from the COLLISION geometry (what both engines see) and from the SWEPT section: everything
        # of the instrument below the guide top (tip up to guide_gap + guide_t, plus 2 mm) passes through the guide
        # when it is lifted straight out, so the hole is the bounds of that whole stretch (the curved scissor tips
        # and angled tweezer tips sit off the shank axis) plus clearance. Upright hold: gravity leans each
        # instrument towards its centre of mass, which lies on the tight side of the hole (settle tests below).
        coll = instr[name]["coll_mesh"]
        band = section_points(coll, 0.0, r["guide_gap"] + r["guide_t"] + 0.002)
        cx, cy = (band[:, 0].max() + band[:, 0].min()) / 2, (band[:, 1].max() + band[:, 1].min()) / 2
        hx = max(np.ptp(band[:, 0]) / 2 + r["clearance"], r["min_hole"] / 2)
        hy = max(np.ptp(band[:, 1]) / 2 + r["clearance"], r["min_hole"] / 2)
        assert hx < dx - 0.012 and hy < r["pitch"] / 2 - 0.01, (name, hx, hy)
        holes.append((y, hx, hy))
        tip = section_points(coll, 0.0, r["tip_t"] + 0.002)
        tcx, tcy = (tip[:, 0].max() + tip[:, 0].min()) / 2, (tip[:, 1].max() + tip[:, 1].min()) / 2
        thx = max(np.ptp(tip[:, 0]) / 2 + 0.0025, 0.003)
        thy = max(np.ptp(tip[:, 1]) / 2 + 0.0025, 0.003)
        tip_holes.append((y - cy + tcy, thx, thy, -cx + tcx))
        slots[name] = dict(index=i, y=y, hole_half=[hx, hy],
                           instrument_pos=[-cx, y - cy, z_base + 0.0005])
    prims += perforated_plate(holes, half_len - 0.02, dx, zg0, zg1, wood)
    prims += perforated_plate(tip_holes, half_len - 0.02, dx, z_base, z_base + r["tip_t"], wood)
    for y, hx, hy in holes:
        # coloured slot marker on the front edge (visual only; helps perception + humans)
        prims.append(P("box", (0.002, 0.012, 0.004), (-dx - 0.002, y, (zg0 + zg1) / 2), rgba=accent, collide=False))
    return prims, slots, dict(half_len=half_len, depth=r["depth"], guide_top=zg1, base_top=z_base)


def perforated_plate(holes, half_len, dx, z0, z1, rgba):
    """Plate |x|<dx, |y|<half_len, z0..z1 with rectangular holes (y, hx, hy[, x_offset]) as boxes."""
    out, hz, zc = [], (z1 - z0) / 2, (z0 + z1) / 2
    hs = sorted(holes)
    edges = [-half_len] + sum([[h[0] - h[2], h[0] + h[2]] for h in hs], []) + [half_len]
    for a, b in zip(edges[0::2], edges[1::2]):
        if b - a > 1e-4:
            out.append(P("box", (dx, (b - a) / 2, hz), (0, (a + b) / 2, zc), rgba=rgba))
    for h in hs:
        y, hx, hy = h[:3]
        xo = h[3] if len(h) > 3 else 0.0
        out.append(P("box", ((dx - (xo + hx)) / 2, hy, hz), ((dx + xo + hx) / 2, y, zc), rgba=rgba))
        out.append(P("box", ((dx + (xo - hx)) / 2, hy, hz), ((-dx + xo - hx) / 2, y, zc), rgba=rgba))
    return out


def table_geometry():
    t = TABLE
    top, hx, hy = t["top"], t["lx"] / 2, t["ly"] / 2
    steel = (0.78, 0.79, 0.81, 1)
    dark = (0.25, 0.25, 0.27, 1)
    z_tray = top - t["tray_t"] / 2
    prims = [P("box", (hx, hy, t["tray_t"] / 2), (0, 0, z_tray), rgba=steel)]
    zl = top + t["lip_h"] / 2 - 0.0
    lt = t["lip_t"] / 2
    prims += [P("box", (hx, lt, t["lip_h"] / 2), (0, s * (hy - lt), zl), rgba=steel) for s in (-1, 1)]
    prims += [P("box", (lt, hy, t["lip_h"] / 2), (s * (hx - lt), 0, zl), rgba=steel) for s in (-1, 1)]
    # cantilever: support arm under the tray, vertical post on the far side, H base on the floor
    prims.append(P("box", (0.16, 0.03, 0.012), (0.04, 0, top - t["tray_t"] - 0.012), rgba=steel))
    prims.append(P("cylinder", (t["post_r"], (top - 0.07) / 2), (t["post_x"], 0, 0.045 + (top - 0.07) / 2), rgba=steel))
    prims.append(P("box", (0.025, 0.28, 0.015), (t["post_x"], 0, 0.045), rgba=dark))          # cross bar
    for s in (-1, 1):
        prims.append(P("box", (0.24, 0.02, 0.015), (0.03, s * 0.27, 0.045), rgba=dark))       # rails
        for x in (-0.19, 0.25):
            prims.append(P("sphere", (0.03,), (x, s * 0.27, 0.03), rgba=dark))                  # casters
    return prims


def surgeon_geometry():
    """Static mannequin, local frame: floor between the feet, facing +x, right hand held out for handover."""
    scrubs = (0.30, 0.55, 0.55, 1)
    skin = (0.87, 0.72, 0.60, 1)
    glove = (0.90, 0.85, 0.70, 1)   # latex glove
    cap = (0.25, 0.45, 0.70, 1)
    shoe = (0.9, 0.9, 0.9, 1)
    prims = []
    for s in (-1, 1):
        prims.append(P("box", (0.13, 0.05, 0.03), (0.04, s * 0.10, 0.03), rgba=shoe))
        prims.append(seg_capsule((0, s * 0.10, 0.10), (0, s * 0.10, 0.88), 0.065, scrubs))
    prims.append(P("box", (0.11, 0.18, 0.09), (0, 0, 0.95), rgba=scrubs))
    prims.append(P("box", (0.12, 0.20, 0.26), (0, 0, 1.28), rgba=scrubs))
    prims.append(P("cylinder", (0.05, 0.04), (0, 0, 1.58), rgba=skin))
    prims.append(P("sphere", (0.105,), (0.0, 0, 1.70), rgba=skin))
    prims.append(P("sphere", (0.108,), (-0.01, 0, 1.73), rgba=cap))
    prims.append(P("box", (0.02, 0.07, 0.04), (0.095, 0, 1.67), rgba=(0.55, 0.75, 0.85, 1)))    # mask
    # left arm: working over the operating table
    prims.append(seg_capsule((0, 0.25, 1.47), (0.12, 0.27, 1.21), 0.045, scrubs))
    prims.append(seg_capsule((0.12, 0.27, 1.21), (0.38, 0.15, 1.10), 0.038, glove))
    # right arm: held out to the side, palm up, for the handover
    prims.append(seg_capsule((0, -0.25, 1.47), (0.06, -0.33, 1.20), 0.045, scrubs))
    prims.append(seg_capsule((0.06, -0.33, 1.20), (0.18, -0.35, 1.03), 0.038, glove))
    prims.append(P("box", (0.05, 0.042, 0.012), (0.245, -0.35, 1.005), rgba=glove))          # palm, up
    for k in range(4):   # fingers, slightly curled up at the end
        prims.append(seg_capsule((0.29, -0.378 + 0.019 * k, 1.008), (0.33, -0.378 + 0.019 * k, 1.02), 0.0085, glove))
    prims.append(seg_capsule((0.24, -0.39, 1.01), (0.27, -0.41, 1.03), 0.009, glove))          # thumb
    hand_site = dict(pos=[0.25, -0.35, 1.02], quat=[1, 0, 0, 0])
    return prims, hand_site


# ------------------------------------------------------------------------------------------------ writers
def write_obj(mesh, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    mesh.export(path, file_type="obj", include_normals=False, include_texture=False)


def mj_prim(p, cls, extra="", with_rgba=True):
    t = p["type"]
    q = "" if np.allclose(p["quat"], (1, 0, 0, 0)) else f' quat="{fmt(p["quat"])}"'
    rgba = f' rgba="{fmt(p["rgba"], 3)}"' if with_rgba else ""
    return f'<geom class="{cls}" type="{t}" size="{fmt(p["size"])}" pos="{fmt(p["pos"])}"{q}{rgba}{extra}/>'


def write_mjcf(name, o, static=False, site=None):
    """One MJCF model per object; body `name` at the origin. Paths relative to mjcf/."""
    lines = [f"<!-- GENERATED by hospital-sim-env/objects/build_objects.py - do not edit. {o['title']}. Source: {o['source']} -->",
             f'<mujoco model="{name}">', '  <compiler angle="radian" autolimits="true"/>', "  <default>",
             f'    <default class="{name}_visual"><geom contype="0" conaffinity="0" group="2" density="0"/></default>']
    if o["kind"] == "instrument":
        lines.append(f'    <default class="{name}_collision"><geom group="3" density="0" contype="1" conaffinity="1" '
                     f'condim="4" friction="{fmt(INSTR_FRICTION)}" solref="0.004 1" solimp="0.95 0.99 0.001" '
                     f'margin="0.0005" rgba="0.8 0.3 0.2 0.5"/></default>')
    else:
        lines.append(f'    <default class="{name}_collision"><geom group="0" density="0" contype="1" conaffinity="1" '
                     f'condim="3" friction="{FURN_FRICTION} 0.005 0.0001" rgba="0.5 0.5 0.5 1"/></default>')
    lines += ["  </default>", "  <asset>"]
    for k, (col, m) in enumerate(o.get("visual_files", [])):
        lines.append(f'    <mesh name="{name}_vis{k}" file="../meshes/{name}/visual_{k}.obj"/>')
    for k in range(len(o["hulls"])):
        lines.append(f'    <mesh name="{name}_col{k}" file="../meshes/{name}/collision_{k}.obj"/>')
    lines += ["  </asset>", "  <worldbody>", f'    <body name="{name}">']
    if not static:
        lines.append(f'      <freejoint name="{name}_free"/>')
        i = o["inertial"]
        lines.append(f'      <inertial pos="{fmt(i["com"])}" quat="{fmt(i["iquat"])}" mass="{i["mass"]:.4g}" '
                     f'diaginertia="{fmt(i["diag"])}"/>')
    for k, (col, m) in enumerate(o.get("visual_files", [])):
        lines.append(f'      <geom class="{name}_visual" type="mesh" mesh="{name}_vis{k}" rgba="{fmt(col, 3)}"/>')
    for p in o.get("vis_prims", []):
        lines.append("      " + mj_prim(p, f"{name}_visual"))
    for k in range(len(o["hulls"])):
        lines.append(f'      <geom class="{name}_collision" type="mesh" mesh="{name}_col{k}"/>')
    for p in o["prims"]:
        lines.append("      " + mj_prim(p, f"{name}_collision", with_rgba=o["kind"] != "instrument"))
    for sname, s in (o.get("sites") or {}).items():
        lines.append(f'      <site name="{sname}" pos="{fmt(s["pos"])}" quat="{fmt(s.get("quat", [1, 0, 0, 0]))}" '
                     f'size="{fmt(s.get("size", [0.008]))}" type="{s.get("type", "sphere")}" group="4" rgba="1 0.5 0 0.6"/>')
    lines += ["    </body>", "  </worldbody>", "</mujoco>", ""]
    (MJCF_DIR / f"{name}.xml").write_text("\n".join(lines))


def write_usd(name, o, static=False):
    """Physics-ready USD per object: /<name> Xform (rigid body + mass unless static), visual, collision."""
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

    path = USD_DIR / f"{name}.usd"
    if path.exists():
        path.unlink()
    st = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(st, 1.0)
    root = UsdGeom.Xform.Define(st, f"/{name}")
    st.SetDefaultPrim(root.GetPrim())
    root.GetPrim().SetCustomDataByKey("hg1:title", o["title"])
    root.GetPrim().SetCustomDataByKey("hg1:source", o["source"])
    root.GetPrim().SetCustomDataByKey("hg1:licence", o["licence"])
    looks = UsdGeom.Scope.Define(st, f"/{name}/Looks")
    mats = {}

    def material(rgba, metallic=None, rough=None):
        key = tuple(round(x, 3) for x in rgba)
        if key in mats:
            return mats[key]
        mp = f"/{name}/Looks/M{len(mats)}"
        mat = UsdShade.Material.Define(st, mp)
        sh = UsdShade.Shader.Define(st, mp + "/Shader")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*key[:3]))
        is_steel = abs(key[0] - key[1]) < 0.03 and abs(key[1] - key[2]) < 0.05 and key[0] > 0.5 and o["kind"] == "instrument"
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic if metallic is not None else (0.9 if is_steel else 0.0))
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough if rough is not None else (0.3 if is_steel else 0.6))
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        mats[key] = mat
        return mat

    pm = UsdShade.Material.Define(st, f"/{name}/Looks/PhysicsMaterial")
    pma = UsdPhysics.MaterialAPI.Apply(pm.GetPrim())
    if o["kind"] == "instrument":
        pma.CreateStaticFrictionAttr(0.8), pma.CreateDynamicFrictionAttr(0.7), pma.CreateRestitutionAttr(0.0)
    else:
        pma.CreateStaticFrictionAttr(FURN_FRICTION), pma.CreateDynamicFrictionAttr(FURN_FRICTION * 0.9)
        pma.CreateRestitutionAttr(0.0)

    vis = UsdGeom.Xform.Define(st, f"/{name}/visual")
    if o.get("real"):
        # reference the i4h asset (keeps its textures and the lite/full switch), minus its own physics
        rel = Path("../../isaac/assets/i4h") / o["src"]   # from usd/ to the sibling isaac/assets switch
        ref = UsdGeom.Xform.Define(st, f"/{name}/visual/asset")
        ref.GetPrim().GetReferences().AddReference(str(rel))
        T = o["T_asset"] @ np.diag([o["mpu"]] * 3 + [1])  # asset units -> canonical metres
        ref.AddTransformOp().Set(Gf.Matrix4d(T.T.tolist()))
        strip_asset_physics(st, ref.GetPrim(), o)
    else:
        for k, (col, m) in enumerate(o["visual_files"]):
            mesh = UsdGeom.Mesh.Define(st, f"/{name}/visual/mesh{k}")
            mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(np.asarray(m.vertices, np.float32)))
            mesh.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(m.faces)))
            mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(m.faces.ravel().tolist()))
            mesh.CreateSubdivisionSchemeAttr("none")
            UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material(col))
        for k, p in enumerate(o.get("vis_prims", [])):
            g = usd_prim(st, f"/{name}/visual/p{k}", p)
            UsdShade.MaterialBindingAPI.Apply(g.GetPrim()).Bind(material(p["rgba"]))

    col = UsdGeom.Scope.Define(st, f"/{name}/collision")
    for k, h in enumerate(o["hulls"]):
        mesh = UsdGeom.Mesh.Define(st, f"/{name}/collision/hull{k}")
        mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(np.asarray(h.vertices, np.float32)))
        mesh.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(h.faces)))
        mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(h.faces.ravel().tolist()))
        mesh.CreatePurposeAttr(UsdGeom.Tokens.guide)
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("convexHull")
        # the parts are already convex: let PhysX keep all their vertices (default limit 64 blunts the thin tips)
        mesh.GetPrim().AddAppliedSchema("PhysxConvexHullCollisionAPI")
        mesh.GetPrim().CreateAttribute("physxConvexHullCollision:hullVertexLimit", Sdf.ValueTypeNames.Int).Set(255)
        collision_tuning(mesh.GetPrim(), o)
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(pm, UsdShade.Tokens.weakerThanDescendants, "physics")
    for k, p in enumerate(o["prims"]):
        g = usd_prim(st, f"/{name}/collision/p{k}", p)
        UsdPhysics.CollisionAPI.Apply(g.GetPrim())
        collision_tuning(g.GetPrim(), o)
        UsdShade.MaterialBindingAPI.Apply(g.GetPrim()).Bind(pm, UsdShade.Tokens.weakerThanDescendants, "physics")
        if o["kind"] == "instrument" or not p["visual"]:
            g.CreatePurposeAttr(UsdGeom.Tokens.guide)
        else:
            UsdShade.MaterialBindingAPI(g.GetPrim()).Bind(material(p["rgba"]))

    for sname, s in (o.get("sites") or {}).items():
        x = UsdGeom.Xform.Define(st, f"/{name}/{sname}")
        x.AddTranslateOp().Set(Gf.Vec3d(*s["pos"]))
        q = s.get("quat", [1, 0, 0, 0])
        x.AddOrientOp().Set(Gf.Quatf(q[0], *q[1:]))

    if not static:
        prim = root.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        ma = UsdPhysics.MassAPI.Apply(prim)
        i = o["inertial"]
        ma.CreateMassAttr(i["mass"])
        ma.CreateCenterOfMassAttr(Gf.Vec3f(*i["com"]))
        ma.CreateDiagonalInertiaAttr(Gf.Vec3f(*i["diag"]))
        q = i["iquat"]
        ma.CreatePrincipalAxesAttr(Gf.Quatf(q[0], *q[1:]))
        # PhysX tuning for small, light, thin bodies (see README.md)
        prim.AddAppliedSchema("PhysxRigidBodyAPI")
        prim.CreateAttribute("physxRigidBody:enableCCD", Sdf.ValueTypeNames.Bool).Set(True)
        prim.CreateAttribute("physxRigidBody:solverPositionIterationCount", Sdf.ValueTypeNames.UInt).Set(16)
        prim.CreateAttribute("physxRigidBody:solverVelocityIterationCount", Sdf.ValueTypeNames.UInt).Set(4)
        prim.CreateAttribute("physxRigidBody:maxDepenetrationVelocity", Sdf.ValueTypeNames.Float).Set(1.0)
        prim.CreateAttribute("physxRigidBody:sleepThreshold", Sdf.ValueTypeNames.Float).Set(0.0005)
        prim.CreateAttribute("physxRigidBody:stabilizationThreshold", Sdf.ValueTypeNames.Float).Set(0.0001)
    st.GetRootLayer().Save()


def collision_tuning(prim, o):
    from pxr import Sdf

    prim.AddAppliedSchema("PhysxCollisionAPI")
    if o["kind"] == "instrument":
        prim.CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float).Set(0.002)
        prim.CreateAttribute("physxCollision:restOffset", Sdf.ValueTypeNames.Float).Set(0.0)
    else:
        prim.CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float).Set(0.002)
        prim.CreateAttribute("physxCollision:restOffset", Sdf.ValueTypeNames.Float).Set(0.0)


def usd_prim(st, path, p):
    from pxr import Gf, UsdGeom

    t = p["type"]
    if t == "box":
        g = UsdGeom.Cube.Define(st, path)
        g.CreateSizeAttr(1.0)
        scale = 2 * np.asarray(p["size"])
    elif t == "sphere":
        g = UsdGeom.Sphere.Define(st, path)
        g.CreateRadiusAttr(p["size"][0])
        scale = None
    elif t == "capsule":
        g = UsdGeom.Capsule.Define(st, path)
        g.CreateRadiusAttr(p["size"][0]), g.CreateHeightAttr(2 * p["size"][1]), g.CreateAxisAttr("Z")
        scale = None
    elif t == "cylinder":
        g = UsdGeom.Cylinder.Define(st, path)
        g.CreateRadiusAttr(p["size"][0]), g.CreateHeightAttr(2 * p["size"][1]), g.CreateAxisAttr("Z")
        scale = None
    g.AddTranslateOp().Set(Gf.Vec3d(*p["pos"]))
    q = p["quat"]
    g.AddOrientOp().Set(Gf.Quatf(float(q[0]), *[float(x) for x in q[1:]]))
    if scale is not None:
        g.AddScaleOp().Set(Gf.Vec3f(*scale.tolist()))
    g.CreateDisplayColorAttr([tuple(p["rgba"][:3])])
    return g


def strip_asset_physics(st, ref_prim, o):
    """Delete the i4h asset's own rigid bodies / colliders and hide collider / site / lid prims."""
    from pxr import Sdf, Usd

    src = Usd.Stage.Open(str(I4H / o["src"]))
    dp = src.GetDefaultPrim()
    inc = [Sdf.Path(p) for p in o["include"]]
    base = ref_prim.GetPath()
    layer = st.GetRootLayer()
    phys = {"PhysicsRigidBodyAPI", "PhysicsMassAPI", "PhysicsCollisionAPI", "PhysicsMeshCollisionAPI",
            "PhysicsFilteredPairsAPI", "PhysxRigidBodyAPI", "PhysxCollisionAPI", "PhysicsArticulationRootAPI"}

    def mapped(p):
        return base.AppendPath(p.MakeRelativePath(dp.GetPath())) if p != dp.GetPath() else base

    for p in Usd.PrimRange(dp):
        path = p.GetPath()
        inside = any(path.HasPrefix(i) for i in inc)
        ancestor = any(i.HasPrefix(path) for i in inc)
        keep_looks = any(q.GetTypeName() == "Material" and "Physics" not in str(q.GetPath())
                         for q in Usd.PrimRange(p)) and p.GetName() != "Physics"
        applied = set(p.GetAppliedSchemas()) & phys
        if not (inside or ancestor or keep_looks) or (inside and p.GetName() in ("Collisions", "Sites")):
            spec = Sdf.CreatePrimInLayer(layer, mapped(path))
            spec.active = False
            continue
        if applied:
            spec = Sdf.CreatePrimInLayer(layer, mapped(path))
            spec.SetInfo("apiSchemas", Sdf.TokenListOp.Create(deletedItems=sorted(applied)))


# ------------------------------------------------------------------------------------------------ main
def main():
    for d in (MESH_DIR, MJCF_DIR, USD_DIR):
        if d.exists():
            shutil.rmtree(d)
        d.mkdir(parents=True)
    placements = yaml.safe_load((HERE / "placements.yaml").read_text())

    objs = build_instruments()
    rack_prims, slots, rack_info = rack_geometry(objs)
    surgeon_prims, hand_site = surgeon_geometry()
    furniture = {
        "instrument_table": dict(title="Instrument table (Mayo-stand-like, tray top 0.78 m)", kind="furniture",
                                 prims=table_geometry(), source="procedural (this repo)", licence="same as repo"),
        "tool_rack": dict(title="Upright instrument rack, 7 slots, 90 mm pitch", kind="furniture", prims=rack_prims,
                          source="procedural (this repo)", licence="same as repo"),
        "surgeon": dict(title="Surgeon mannequin (static stand-in)", kind="person", prims=surgeon_prims,
                        source="procedural (this repo)", licence="same as repo",
                        sites={"surgeon_hand": dict(hand_site, size=[0.02])}),
    }
    for name, f in furniture.items():
        f.update(hulls=[], real=False)
        f["vis_prims"] = [p for p in f["prims"] if p["visual"] and not p["collide"]]
        f["prims"] = [p for p in f["prims"] if p["collide"]]
        f["visual_files"] = []
        V = np.vstack([prim_mesh(p).vertices for p in f["prims"] + f["vis_prims"]])
        f["bbox"] = [V.min(0).tolist(), V.max(0).tolist()]

    # grasp sites + regions for instruments (canonical frame)
    guide_top_rel = RACK["guide_gap"] + RACK["guide_t"]  # above the tip when standing in the rack
    for name, o in objs.items():
        L = o["length"]
        z0 = guide_top_rel + 0.020
        z1 = max(z0 + 0.02, L - (0.045 if name in ("scissors", "needle_driver") else 0.008))
        o["grasp"] = dict(z_range=[round(z0, 4), round(z1, 4)], site=[0.0, 0.0, round((z0 + z1) / 2, 4)])
        o["sites"] = {f"{name}_grasp": dict(pos=o["grasp"]["site"], size=[0.006])}

    # meshes + per-object MJCF/USD
    for name, o in {**objs, **furniture}.items():
        o.setdefault("visual_files", [])
        if o["kind"] == "instrument":
            o["visual_files"] = []
            for k, (col, m) in enumerate(o["visual"]):
                write_obj(m, MESH_DIR / name / f"visual_{k}.obj")
                o["visual_files"].append((col, m))
            for k, h in enumerate(o["hulls"]):
                write_obj(h, MESH_DIR / name / f"collision_{k}.obj")
            o["vis_prims"] = []
        write_mjcf(name, o, static=o["kind"] != "instrument")
        write_usd(name, o, static=o["kind"] != "instrument")

    # world poses
    pl = placements["objects"]
    T = {}
    for name in ("instrument_table", "surgeon"):
        T[name] = pose_mat(pl[name]["pos"], quat_from_yaw(pl[name]["yaw"]))
    T["tool_rack"] = T[pl["tool_rack"]["parent"]] @ pose_mat(pl["tool_rack"]["pos"], quat_from_yaw(pl["tool_rack"]["yaw"]))
    for name in INSTRUMENTS:
        s = slots[name]
        assert placements["rack_slots"][name] == s["index"], "rack_slots in placements.yaml must match INSTRUMENTS order"
        T[name] = T["tool_rack"] @ pose_mat(s["instrument_pos"], (1, 0, 0, 0))
    world = {n: dict(pos=M[:3, 3].tolist(), quat=quat_from_matrix(M[:3, :3]).tolist()) for n, M in T.items()}
    hs = T["surgeon"] @ pose_mat(hand_site["pos"], (1, 0, 0, 0))
    world["surgeon_hand"] = dict(pos=hs[:3, 3].tolist(), quat=quat_from_matrix(hs[:3, :3]).tolist())

    write_theatre_mjcf(world, placements)
    write_catalog(objs, furniture, slots, rack_info, world, placements)
    print(f"built {len(objs)} instruments + {len(furniture)} furniture/person models -> {HERE}")
    for n, o in objs.items():
        print(f"  {n:16s} L={o['length'] * 1000:6.1f} mm  m={o['mass'] * 1000:5.1f} g  "
              f"hulls={len(o['hulls']):2d} prims={len(o['prims']):2d}  flipped={o.get('flipped')}  "
              f"hole={[round(h * 2000, 1) for h in slots[n]['hole_half']]} mm")


def write_theatre_mjcf(world, placements):
    lines = ["<!-- GENERATED by hospital-sim-env/objects/build_objects.py from placements.yaml - do not edit.",
             "     Every theatre object at its world pose. Loadable on its own (floor + light) for checks;",
             "     to add the objects to another scene copy the <model>/<attach> lines (paths relative to this dir). -->",
             '<mujoco model="theatre_objects">', '  <compiler angle="radian"/>',
             '  <option timestep="0.002" integrator="implicitfast" cone="elliptic" impratio="10"/>', "  <asset>"]
    names = ["instrument_table", "tool_rack", "surgeon"] + INSTRUMENTS
    for n in names:
        lines.append(f'    <model name="{n}" file="{n}.xml"/>')
    lines += ["  </asset>", "  <worldbody>",
              '    <light pos="25.6 -20 3" dir="0 0 -1" diffuse="0.8 0.8 0.8"/>',
              '    <geom name="floor" type="plane" size="40 40 0.1" pos="0 0 0" rgba="0.6 0.6 0.62 1"/>']
    for n in names:
        w = world[n]
        lines.append(f'    <frame pos="{fmt(w["pos"])}" quat="{fmt(w["quat"])}"><attach model="{n}" body="{n}" prefix=""/></frame>')
    lines += ["  </worldbody>", "</mujoco>", ""]
    (MJCF_DIR / "theatre_objects.xml").write_text("\n".join(lines))


def write_catalog(objs, furniture, slots, rack_info, world, placements):
    cat = dict(generated_by="hospital-sim-env/objects/build_objects.py", frame_convention=__doc__.split("Canonical instrument frame:")[1].strip(),
               labels=INSTRUMENTS, instruments={}, furniture={}, rack=dict(RACK, **rack_info, slots=slots),
               table=TABLE, world_poses=world, robot_stances=placements.get("robot_stances", {}),
               remove_scene_props=placements.get("remove_scene_props", []))
    for n, o in objs.items():
        lo, hi = np.asarray(o["bbox"][0]), np.asarray(o["bbox"][1])
        cat["instruments"][n] = dict(
            title=o["title"], stand_in=not o["real"], source=o["source"], licence=o["licence"],
            size_xyz_m=(hi - lo).round(4).tolist(), bbox_min=lo.round(4).tolist(), bbox_max=hi.round(4).tolist(),
            mass_kg=o["mass"], com=np.round(o["inertial"]["com"], 5).tolist(),
            inertia_diag=[float(f"{x:.4g}") for x in o["inertial"]["diag"]], inertia_quat=np.round(o["inertial"]["iquat"], 5).tolist(),
            collision=("%d CoACD convex hulls" % len(o["hulls"])) if o["hulls"] else ("%d primitives" % len(o["prims"])),
            grasp=o["grasp"], mjcf=f"hospital-sim-env/objects/mjcf/{n}.xml", usd=f"hospital-sim-env/objects/usd/{n}.usd",
            asset=(f"hospital-sim-env/isaac/assets/i4h/{o['src']}" if o["real"] else None), canonical_flip=o.get("flipped"))
    for n, f in furniture.items():
        lo, hi = np.asarray(f["bbox"][0]), np.asarray(f["bbox"][1])
        cat["furniture"][n] = dict(title=f["title"], size_xyz_m=(hi - lo).round(4).tolist(), bbox_min=lo.round(4).tolist(),
                                   bbox_max=hi.round(4).tolist(), static=True, mjcf=f"hospital-sim-env/objects/mjcf/{n}.xml",
                                   usd=f"hospital-sim-env/objects/usd/{n}.usd", sites=f.get("sites"))
    (HERE / "catalog.json").write_text(json.dumps(cat, indent=1))


if __name__ == "__main__":
    sys.exit(main())
