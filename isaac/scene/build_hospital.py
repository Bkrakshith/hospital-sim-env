"""Build the Hospital_G1 scene as layered USD. No physics is authored.

    usd/hospital_g1.usd      root: sublayers below, defaultPrim /World
    usd/building.usd         Isaac hospital (SE door leaves removed so the wing is reachable)
    usd/wing_shell.usd       kit floors/ceilings/walls/doors of link, pre-op, PACU, corridor
    usd/or_suite.usd         i4h OR suite rotated into the wing + door cuts + prop moves (overrides only)
    usd/dressing_<room>.usd  furniture per room
    usd/cameras.usd          viewing cameras

Run inside Isaac Sim:  import runpy; runpy.run_path("<repo>/scene/build_hospital.py", run_name="__main__")
or standalone:         python scene/build_hospital.py   (needs usd-core)
"""

import json
import math
import os
import re
import sys

from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import assets as A  # noqa: E402
import layout as L  # noqa: E402

REPO = A.REPO
USD_DIR = os.path.join(REPO, "usd")
KIT_SCALE = 0.01

# Kit children switched off per use (matched against prim names inside the module).
WALL_ONLY = [r"^sm_floor", r"^sm_ceiling", r"^Ceiling_Lamp$"]
DOOR_LEAVES = [r"^sm_door_a0\d", r"^sm_skirting"]
DOOR_BARE = DOOR_LEAVES + WALL_ONLY

# Per-module lights (one rect light per 2 m cell / wall strip) and the suite's shelf LED strips.
MODULE_LIGHTS = [r"^Ceiling_Lamp", r"^Led_Lamp$"]
# "rooms" (default): strip the ~400 module lights and light each room with a few ceiling panels (fits 8 GB);
# "kit": keep every module light (12 GB+ GPUs); "none": strip without replacing (diagnostics).
LIGHTS = os.environ.get("HG1_LIGHTS", "rooms")

# Hospital door leaves in the SE doorway the wing attaches to.
HOSPITAL_DOOR_LEAVES = ["Geo_M_DobleDoor_867", "Geo_M_DobleDoor6"]


def rel(asset):
    return os.path.relpath(os.path.join(REPO, asset), USD_DIR)


def new_stage(name):
    path = os.path.join(USD_DIR, name)
    layer = Sdf.Layer.FindOrOpen(path)
    if layer:
        layer.Clear()
    else:
        layer = Sdf.Layer.CreateNew(path)
    stage = Usd.Stage.Open(layer, Usd.Stage.LoadNone)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetDefaultPrim(stage.DefinePrim("/World", "Xform"))
    return stage


def xf(stage, path, t=(0, 0, 0), rz=0.0, s=None):
    x = UsdGeom.Xform.Define(stage, path)
    x.AddTranslateOp().Set(Gf.Vec3d(*t))
    if rz:
        x.AddRotateZOp().Set(float(rz))
    if s is not None and s != 1.0:
        x.AddScaleOp().Set(Gf.Vec3f(s, s, s))
    return x.GetPrim()


def add_ref(prim, asset, prim_path=None):
    if prim_path:
        prim.GetReferences().AddReference(rel(asset), Sdf.Path(prim_path))
    else:
        prim.GetReferences().AddReference(rel(asset))


_children = {}


def kit_children(asset):
    if asset not in _children:
        st = Usd.Stage.Open(os.path.join(REPO, asset))
        root = st.GetDefaultPrim()
        _children[asset] = [
            (p.GetPath().MakeRelativePath(root.GetPath()), p.GetName()) for p in Usd.PrimRange(root) if p != root
        ]
    return _children[asset]


def switch_off(stage, prim, asset, patterns):
    off = []
    for relp, name in kit_children(asset):
        if any(str(relp).startswith(str(o) + "/") for o in off):
            continue
        if any(re.match(pt, name) for pt in patterns):
            stage.OverridePrim(prim.GetPath().AppendPath(relp)).SetActive(False)
            off.append(relp)


def box(stage, path, lo, hi, color=(0.93, 0.93, 0.91)):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.GetSizeAttr().Set(1.0)
    c = [(a + b) / 2 for a, b in zip(lo, hi)]
    s = [b - a for a, b in zip(lo, hi)]
    cube.AddTranslateOp().Set(Gf.Vec3d(*c))
    cube.AddScaleOp().Set(Gf.Vec3f(*s))
    cube.GetDisplayColorAttr().Set([Gf.Vec3f(*color)])


# ---------------------------------------------------------------- building


def build_building():
    st = new_stage("building.usd")
    hosp = st.DefinePrim("/World/Hospital", "Xform")
    add_ref(hosp, A.HOSPITAL_BUILDING)
    for n in HOSPITAL_DOOR_LEAVES:
        st.OverridePrim(f"/World/Hospital/{n}").SetActive(False)
    st.GetRootLayer().Save()


# ---------------------------------------------------------------- wing shell


def _on_span(seg, span):
    (a0, b0, a1, b1), (c0, d0, c1, d1) = seg, span
    if a0 == a1 == c0 == c1:
        return min(d0, d1) <= min(b0, b1) and max(b0, b1) <= max(d0, d1)
    if b0 == b1 == d0 == d1:
        return min(c0, c1) <= min(a0, a1) and max(a0, a1) <= max(c0, c1)
    return False


def _boundary(rect):
    x0, y0, x1, y1 = rect
    c = L.CELL
    for x in range(int(x0), int(x1), int(c)):
        yield "S", (x, y0, x + c, y0)
        yield "N", (x, y1, x + c, y1)
    for y in range(int(y0), int(y1), int(c)):
        yield "W", (x0, y, x0, y + c)
        yield "E", (x1, y, x1, y + c)


# Straight-module placement per wall side, from the wall segment (x0, y0, x1, y1): the module's wall
# lies on its cell's y=0 edge with the cell at -y, so each side needs a rotation and a segment end as origin.
_STRAIGHT = {"N": (0, lambda s: (s[0], s[1])), "S": (180, lambda s: (s[2], s[1])),
             "E": (-90, lambda s: (s[0], s[3])), "W": (90, lambda s: (s[0], s[1]))}


def build_wing_shell():
    st = new_stage("wing_shell.usd")
    xf(st, "/World/Wing", L.WING_ORIGIN)
    root = "/World/Wing/Shell"
    st.DefinePrim(root, "Xform")
    no_walls = list(L.OPENINGS) + list(L.WING_DOORS.values()) + list(L.SUITE_DOORS.values())
    # Corridor cells in front of the suite doors are covered by the suite door modules' floor/ceiling.
    covered = {(x, -10) for (a, _, b, _) in L.SUITE_DOORS.values() for x in range(a, b, 2)}
    counts = {}
    for room, rect in L.KIT_ROOMS.items():
        rp = f"{root}/{room}"
        st.DefinePrim(rp, "Xform")
        x0, y0, x1, y1 = rect
        n = 0
        for x in range(x0, x1, 2):
            for y in range(y0, y1, 2):
                if room == "Corridor" and (x, y) in covered:
                    continue
                p = xf(st, f"{rp}/Cell_{n}", (x, y + 2, 0), 0, KIT_SCALE)
                add_ref(p, A.KIT_CENTER)
                if LIGHTS != "kit":
                    switch_off(st, p, A.KIT_CENTER, MODULE_LIGHTS)
                else:
                    p.SetInstanceable(True)
                n += 1
        w = 0
        for side, seg in _boundary(rect):
            if any(_on_span(seg, sp) for sp in no_walls):
                continue
            rz, origin = _STRAIGHT[side]
            p = xf(st, f"{rp}/Wall_{w}", (*origin(seg), 0), rz, KIT_SCALE)
            add_ref(p, A.KIT_STRAIGHT)
            switch_off(st, p, A.KIT_STRAIGHT, WALL_ONLY + (MODULE_LIGHTS if LIGHTS != "kit" else []))
            w += 1
        counts[room] = {"cells": n, "walls": w}
    for room, (a, y, b, _) in L.WING_DOORS.items():
        p = xf(st, f"{root}/Door_{room}", (a, y, 0), 0, KIT_SCALE)
        add_ref(p, A.KIT_DOOR)
        switch_off(st, p, A.KIT_DOOR, DOOR_BARE + (MODULE_LIGHTS if LIGHTS != "kit" else []))
    # Close the gaps where the 2 m link meets the hospital's 2.56 m doorway and 3.0 m wall top.
    box(st, f"{root}/Junction/Header", (-0.28, -0.05, 2.41), (2.28, 0.0, 3.62))
    box(st, f"{root}/Junction/JambWest", (-0.28, -0.05, 0.0), (0.0, 0.0, 2.41))
    box(st, f"{root}/Junction/JambEast", (2.0, -0.05, 0.0), (2.28, 0.0, 2.41))
    st.GetRootLayer().Save()
    return counts


# ---------------------------------------------------------------- OR suite


def _suite_xy(wx, wy):
    """Wing-local metres -> suite cm frame."""
    tx, ty, _ = L.SUITE_TRANSLATE
    return -(wy - ty) * 100.0, (wx - tx) * 100.0


def build_or_suite():
    src = Usd.Stage.Open(os.path.join(REPO, A.OR_SUITE))
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])

    st = new_stage("or_suite.usd")
    st.DefinePrim("/World/Wing", "Xform")
    suite = xf(st, "/World/Wing/ORSuite", L.SUITE_TRANSLATE, L.SUITE_ROTATE_Z, KIT_SCALE)
    add_ref(suite, A.OR_SUITE, "/World")
    for n in L.SUITE_REMOVE:
        st.OverridePrim(f"/World/Wing/ORSuite/Walls/{n}").SetActive(False)
    for n in L.SUITE_HIDE:
        st.OverridePrim(f"/World/Wing/ORSuite/Props/{n}").SetActive(False)
    if LIGHTS == "rooms":
        # The suite's remaining lights (surgical/feature lights, dome) are dimmed to the same level.
        # Lights inside the instanced props and the stripped module/shelf lights are left alone.
        skip = ("/World/Props", "/World/Walls", "/World/Led_Lights_Shelves")
        for q in src.Traverse():
            if q.HasAPI(UsdLux.LightAPI) and not str(q.GetPath()).startswith(skip):
                target = Sdf.Path("/World/Wing/ORSuite").AppendPath(q.GetPath().MakeRelativePath(Sdf.Path("/World")))
                base = UsdLux.LightAPI(q).GetIntensityAttr().Get()
                UsdLux.LightAPI(st.OverridePrim(target)).CreateIntensityAttr(base * LIGHT_LEVEL)
    for n in L.SUITE_OPEN_DOORS:
        it = iter(Usd.PrimRange(src.GetPrimAtPath(f"/World/Walls/{n}")))
        for q in it:
            if re.match(DOOR_LEAVES[0], q.GetName()):
                rel_path = q.GetPath().MakeRelativePath(Sdf.Path("/World"))
                st.OverridePrim(Sdf.Path("/World/Wing/ORSuite").AppendPath(rel_path)).SetActive(False)
                it.PruneChildren()
    # The i4h props are film-density (a mask box is 1.1 M triangles); instancing lets RTX share one copy.
    for p in src.GetPrimAtPath("/World/Props").GetChildren():
        if p.GetName() not in L.SUITE_HIDE:
            st.OverridePrim(f"/World/Wing/ORSuite/Props/{p.GetName()}").SetInstanceable(True)

    xf(st, "/World/Wing/ORSuiteMods", L.SUITE_TRANSLATE, L.SUITE_ROTATE_Z, KIT_SCALE)
    for i, (kind, (tx, ty), rz) in enumerate(L.SUITE_ADD):
        asset = A.KIT_DOOR if kind == "door" else A.KIT_STRAIGHT
        p = xf(st, f"/World/Wing/ORSuiteMods/{kind}_{i}", (tx, ty, 0), rz)
        add_ref(p, asset)
        switch_off(st, p, asset, (DOOR_LEAVES if kind == "door" else WALL_ONLY)
                   + (MODULE_LIGHTS if LIGHTS != "kit" else []))
    if LIGHTS != "kit":
        st.OverridePrim("/World/Wing/ORSuite/Led_Lights_Shelves").SetActive(False)
        for w in src.GetPrimAtPath("/World/Walls").GetChildren():
            if w.GetName() in L.SUITE_REMOVE:
                continue
            it = iter(Usd.PrimRange(w))
            for q in it:
                if any(re.match(pt, q.GetName()) for pt in MODULE_LIGHTS):
                    rel_path = q.GetPath().MakeRelativePath(Sdf.Path("/World"))
                    st.OverridePrim(Sdf.Path("/World/Wing/ORSuite").AppendPath(rel_path)).SetActive(False)
                    it.PruneChildren()

    moved = {}
    for name, how in L.SUITE_MOVES:
        prim = src.GetPrimAtPath(f"/World/Props/{name}")
        m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        if "delta" in how:
            dx, dy = how["delta"]
            w = Gf.Matrix4d().SetTranslate(Gf.Vec3d(dx, dy, 0))
        else:
            c = cache.ComputeWorldBound(prim).ComputeAlignedRange().GetMidpoint()
            sx, sy = _suite_xy(*how["to"])
            w = (Gf.Matrix4d().SetTranslate(Gf.Vec3d(-c[0], -c[1], 0))
                 * Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), how.get("yaw", 0)))
                 * Gf.Matrix4d().SetTranslate(Gf.Vec3d(sx, sy, 0)))
        moved[name] = w
        _set_matrix(st, f"/World/Wing/ORSuite/Props/{name}", m * w)
    for rider, parent in L.SUITE_MOVE_WITH.items():
        prim = src.GetPrimAtPath(f"/World/Props/{rider}")
        m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        _set_matrix(st, f"/World/Wing/ORSuite/Props/{rider}", m * moved[parent])
    st.GetRootLayer().Save()


def _set_matrix(stage, path, m):
    prim = stage.OverridePrim(path)
    op_attr = prim.CreateAttribute("xformOp:transform", Sdf.ValueTypeNames.Matrix4d)
    op_attr.Set(m)
    prim.CreateAttribute("xformOpOrder", Sdf.ValueTypeNames.TokenArray).Set(["xformOp:transform"])


# ---------------------------------------------------------------- dressing

_info = {}
CATALOG = A.build_catalog()


def asset_info(key):
    if key not in _info:
        s = Usd.Stage.Open(os.path.join(REPO, CATALOG[key]))
        mpu = UsdGeom.GetStageMetersPerUnit(s)
        root = s.GetDefaultPrim() or s.GetPseudoRoot()
        r = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"]).ComputeWorldBound(root)
        r = r.ComputeAlignedRange()
        _info[key] = {"mpu": mpu, "min": [v * mpu for v in r.GetMin()], "max": [v * mpu for v in r.GetMax()]}
    return _info[key]


_YAW = {"+x": 0, "+y": 90, "-x": 180, "-y": -90}
_OUT = {"E": 0, "N": 90, "W": 180, "S": -90}


class Room:
    def __init__(self, stage, name, rect, placed):
        self.stage, self.name = stage, name
        self.x0, self.y0, self.x1, self.y1 = rect
        self.W, self.D = self.x1 - self.x0, self.y1 - self.y0
        self.root = f"/World/Wing/Dressing/{name}"
        stage.DefinePrim(self.root, "Xform")
        self.placed = placed

    def put(self, name, key, x, y, yaw=0.0, z=0.0, anchor="bottom"):
        info = asset_info(key)
        mn, mx = info["min"], info["max"]
        cx, cy = (mn[0] + mx[0]) / 2, (mn[1] + mx[1]) / 2
        h = mx[2] - mn[2]
        base = z - h if anchor == "top" else z
        wx, wy = self.x0 + x, self.y0 + y
        slot = xf(self.stage, f"{self.root}/{name}", (wx, wy, base), yaw)
        asset = xf(self.stage, f"{slot.GetPath()}/Asset", (-cx, -cy, -mn[2]), 0, info["mpu"])
        add_ref(asset, CATALOG[key])
        asset.SetInstanceable(True)  # identical props share one GPU copy of their (very dense) meshes
        hx, hy = (mx[0] - mn[0]) / 2, (mx[1] - mn[1]) / 2
        c, s = abs(math.cos(math.radians(yaw))), abs(math.sin(math.radians(yaw)))
        ex, ey = c * hx + s * hy, s * hx + c * hy
        self.placed.append({"room": self.name, "name": name, "key": key,
                            "box": [wx - ex, wy - ey, base, wx + ex, wy + ey, base + h]})

    def wall(self, name, key, wall, u, z=0.0):
        back = L.BACK.get(key, "-y")
        info = asset_info(key)
        axis = 0 if back.endswith("x") else 1
        depth = info["max"][axis] - info["min"][axis]
        off = L.WALL_T + depth / 2 + 0.01
        x, y = {"N": (u, self.D - off), "S": (u, off), "E": (self.W - off, u), "W": (off, u)}[wall]
        self.put(name, key, x, y, _OUT[wall] - _YAW[back], z)


def build_dressing():
    placed = []
    rooms = dict(L.KIT_ROOMS)
    rooms.update(L.SUITE_ROOMS)
    for name, fn in L.DRESSING.items():
        st = new_stage(f"dressing_{name.lower()}.usd")
        st.DefinePrim("/World/Wing", "Xform")
        st.DefinePrim("/World/Wing/Dressing", "Xform")
        fn(Room(st, name, rooms[name], placed))
        st.GetRootLayer().Save()
    return placed


# ---------------------------------------------------------------- lighting

# The kit lights 4 m2 of floor with a 1 m2 panel at intensity 130000; room panels start from that per-area
# level. LIGHT_LEVEL scales room panels and the OR suite's own lights together. It was calibrated from
# renders (scene/image_stats.py): 0.12 gives a median pixel of ~0.5 with nothing clipped under the default
# ACES exposure. The kit's own level left the white rooms 20%+ clipped. 5000 K reads neutral at that exposure.
KIT_LUX_PER_M2 = 130000 / 4.0 * 0.25
LIGHT_LEVEL = 0.12
LIGHT_KELVIN = 5000.0
PANEL = 1.2  # m, square ceiling panel
PANEL_Z = 3.45


def _panels(st, root, rooms, z):
    """Grid of ~3.2 m-spaced ceiling panels per room rectangle, at the calibrated per-area level."""
    n = 0
    for room, (x0, y0, x1, y1) in rooms.items():
        nx, ny = max(1, round((x1 - x0) / 3.2)), max(1, round((y1 - y0) / 3.2))
        per_light = (x1 - x0) * (y1 - y0) / (nx * ny)
        for i in range(nx):
            for j in range(ny):
                x = x0 + (i + 0.5) * (x1 - x0) / nx
                y = y0 + (j + 0.5) * (y1 - y0) / ny
                light = UsdLux.RectLight.Define(st, f"{root}/{room}_{i}_{j}")
                light.AddTranslateOp().Set(Gf.Vec3d(x, y, z))  # rect lights emit along -Z: straight down
                light.GetWidthAttr().Set(PANEL)
                light.GetHeightAttr().Set(PANEL)
                light.GetIntensityAttr().Set(KIT_LUX_PER_M2 * per_light / (PANEL * PANEL) * LIGHT_LEVEL)
                light.CreateEnableColorTemperatureAttr(True)
                light.CreateColorTemperatureAttr(LIGHT_KELVIN)
                n += 1
    return n


def build_lighting():
    st = new_stage("lighting.usd")
    st.DefinePrim("/World/Wing", "Xform")
    st.DefinePrim("/World/Wing/Lighting", "Xform")
    st.DefinePrim("/World/HospitalLighting", "Xform")
    rooms = dict(L.KIT_ROOMS)
    rooms.update({k: v for k, v in L.SUITE_ROOMS.items() if k in L.LIT_SUITE_ROOMS})
    n = _panels(st, "/World/Wing/Lighting", rooms, PANEL_Z)
    n += _hospital_fixtures(st, "/World/HospitalLighting")
    st.GetRootLayer().Save()
    return n


def _hospital_fixtures(st, root):
    """Turn on every hospital ceiling fitting: a light just under each fixture mesh, same per-area level."""
    src = Usd.Stage.Open(os.path.join(REPO, A.HOSPITAL_BUILDING))
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
    n = 0
    for prim in src.GetDefaultPrim().GetChildren():
        spec = next((f for f in L.HOSPITAL_FIXTURES if prim.GetName().startswith(f[0])), None)
        if spec is None:
            continue
        _, kind, cover = spec
        r = cache.ComputeWorldBound(prim).ComputeAlignedRange()  # the hospital sits at the world origin
        (x, y, _), z = r.GetMidpoint(), r.GetMin()[2] - 0.02
        size = r.GetSize()
        path = f"{root}/{prim.GetName()}"
        if kind == "panel":
            light = UsdLux.RectLight.Define(st, path)
            light.GetWidthAttr().Set(float(size[0]) * 0.9)
            light.GetHeightAttr().Set(float(size[1]) * 0.9)
            area = size[0] * size[1] * 0.81
        else:
            light = UsdLux.DiskLight.Define(st, path)
            radius = float(min(size[0], size[1])) / 2
            light.GetRadiusAttr().Set(radius)
            area = math.pi * radius * radius
        light.AddTranslateOp().Set(Gf.Vec3d(x, y, z))  # both emit along -Z: straight down
        light.GetIntensityAttr().Set(KIT_LUX_PER_M2 * cover / area * LIGHT_LEVEL)
        light.CreateEnableColorTemperatureAttr(True)
        light.CreateColorTemperatureAttr(LIGHT_KELVIN)
        n += 1
    return n


# ---------------------------------------------------------------- cameras


def build_cameras():
    st = new_stage("cameras.usd")
    st.DefinePrim("/World/Cameras", "Xform")
    ox, oy, oz = L.WING_ORIGIN

    def cam(name, eye, target, focal, ortho_width=None):
        c = UsdGeom.Camera.Define(st, f"/World/Cameras/{name}")
        up = Gf.Vec3d(0, 1, 0) if ortho_width else Gf.Vec3d(0, 0, 1)
        view = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), up)
        c.MakeMatrixXform().Set(view.GetInverse())
        c.GetClippingRangeAttr().Set(Gf.Vec2f(0.05, 1000.0))
        if ortho_width:
            c.GetProjectionAttr().Set(UsdGeom.Tokens.orthographic)
            c.GetHorizontalApertureAttr().Set(ortho_width * 10.0)
            c.GetVerticalApertureAttr().Set(ortho_width * 10.0 * 9 / 16)
        else:
            c.GetFocalLengthAttr().Set(float(focal))
            c.GetHorizontalApertureAttr().Set(20.955)
            c.GetVerticalApertureAttr().Set(20.955 * 9 / 16)

    for name, (eye, target, focal) in L.CAMERAS.items():
        cam(name, [a + b for a, b in zip(eye, (ox, oy, oz))], [a + b for a, b in zip(target, (ox, oy, oz))], focal)
    # Plan view: orthographic, just under the kit ceiling (3.5 m) so ceilings are behind the camera.
    cx, cy = ox + 1.0, oy - 10.0
    cam("Cam_Plan", (cx, cy, 3.42), (cx, cy, 0.0), 0, ortho_width=38.0)
    st.GetRootLayer().Save()


# ---------------------------------------------------------------- root + verification


def build_root(dressing_layers):
    st = new_stage("hospital_g1.usd")
    subs = ["./cameras.usd"] + (["./lighting.usd"] if LIGHTS == "rooms" else []) + [f"./{n}" for n in dressing_layers] + ["./or_suite.usd", "./wing_shell.usd",
                                                                        "./building.usd"]
    st.GetRootLayer().subLayerPaths = subs
    st.GetRootLayer().documentation = "Hospital_G1: Isaac hospital + surgical wing. No physics authored."
    st.GetRootLayer().Save()
    return os.path.join(USD_DIR, "hospital_g1.usd")


def build_video_stages():
    """usd/video_<group>.usd: the full scene minus the prims a group of walkthrough shots never sees."""
    sys.path.insert(0, HERE)
    import walkthrough as W
    for group, hidden in W.VIDEO_STAGES.items():
        st = new_stage(f"video_{group}.usd")
        st.GetRootLayer().subLayerPaths = ["./hospital_g1.usd"]
        for path in hidden:
            st.OverridePrim(path).SetActive(False)
        st.GetRootLayer().Save()


def _overlap(a, b, tol=0.02):
    return all(min(a[i + 3], b[i + 3]) - max(a[i], b[i]) > tol for i in range(3))


def _mesh_tris(prim):
    f = UsdGeom.Mesh(prim).GetFaceVertexCountsAttr().Get()
    return sum(max(0, c - 2) for c in f) if f is not None else 0


def _gpu_triangles(stage):
    """Triangles the renderer has to hold: instanced geometry counts once per prototype."""
    total = sum(_mesh_tris(p) for p in stage.Traverse() if p.IsA(UsdGeom.Mesh))
    for proto in stage.GetPrototypes():
        total += sum(_mesh_tris(p) for p in Usd.PrimRange(proto) if p.IsA(UsdGeom.Mesh))
    return total


def verify(root_path, placed, shell_counts):
    st = Usd.Stage.Open(root_path)
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
    ox, oy, _ = L.WING_ORIGIN
    moved = {n for n, _ in L.SUITE_MOVES} | set(L.SUITE_MOVE_WITH)
    suite_props = []
    for p in st.GetPrimAtPath("/World/Wing/ORSuite/Props").GetChildren():
        r = cache.ComputeWorldBound(p).ComputeAlignedRange()
        if r.IsEmpty():
            continue
        mn, mx = r.GetMin(), r.GetMax()
        group = L.SUITE_MOVE_WITH.get(p.GetName(), p.GetName())
        suite_props.append({"room": "suite", "name": p.GetName(), "moved": p.GetName() in moved, "group": group,
                            "box": [mn[0] - ox, mn[1] - oy, mn[2], mx[0] - ox, mx[1] - oy, mx[2]]})

    issues = []
    # 1. Doorway clearance (1 m each side of every opening, up to 2 m high).
    for dname, (x0, y0, x1, y1) in L.DOORWAYS.items():
        d = L.DOOR_CLEARANCE
        zone = ([min(x0, x1), y0 - d, 0.0, max(x0, x1), y0 + d, 2.0] if y0 == y1
                else [x0 - d, min(y0, y1), 0.0, x0 + d, max(y0, y1), 2.0])
        for it in placed + suite_props:
            if _overlap(zone, it["box"]):
                issues.append(f"doorway {dname} blocked by {it['room']}/{it['name']}")
    # 2. New or moved items overlapping anything else.
    movers = placed + [s for s in suite_props if s["moved"]]
    others = placed + suite_props
    seen = set()
    for a in movers:
        for b in others:
            if a is b or (id(b), id(a)) in seen or (a.get("group") and a.get("group") == b.get("group")):
                continue
            seen.add((id(a), id(b)))
            if _overlap(a["box"], b["box"]):
                issues.append(f"overlap {a['room']}/{a['name']} <-> {b['room']}/{b['name']}")
    # 3. Dressing stays inside its room.
    rooms = dict(L.KIT_ROOMS)
    rooms.update(L.SUITE_ROOMS)
    for it in placed:
        x0, y0, x1, y1 = rooms[it["room"]]
        b = it["box"]
        if b[0] < x0 + 0.09 or b[1] < y0 + 0.09 or b[3] > x1 - 0.09 or b[4] > y1 - 0.09:
            issues.append(f"outside room: {it['room']}/{it['name']} {[round(v, 2) for v in b]}")
    # 4. No physics authored in our layers.
    physics = []
    for layer_path in st.GetRootLayer().subLayerPaths:
        lyr = Sdf.Layer.FindOrOpen(os.path.join(USD_DIR, layer_path))

        def visit(path):
            spec = lyr.GetObjectAtPath(path)
            if spec and hasattr(spec, "GetInfo") and spec.HasInfo("apiSchemas"):
                if any("Physics" in str(t) or "Physx" in str(t) for t in spec.GetInfo("apiSchemas").GetAddedOrExplicitItems()):
                    physics.append(str(path))
        lyr.Traverse(Sdf.Path.absoluteRootPath, visit)

    report = {
        "root": os.path.relpath(root_path, REPO),   # relative to hospital-sim-env/isaac/
        "gpu_triangles_M": round(_gpu_triangles(st) / 1e6, 2),
        "shell": shell_counts,
        "dressing_items": len(placed),
        "dressing_per_room": {r: sum(1 for p in placed if p["room"] == r) for r in L.DRESSING},
        "suite_props": len(suite_props),
        "physics_specs_authored": physics,
        "issues": issues,
    }
    with open(os.path.join(USD_DIR, "build_report.json"), "w") as f:
        json.dump(report, f, indent=1)
    return report


def main():
    os.makedirs(USD_DIR, exist_ok=True)
    build_building()
    counts = build_wing_shell()
    build_or_suite()
    placed = build_dressing()
    build_cameras()
    if LIGHTS == "rooms":
        build_lighting()
    root = build_root([f"dressing_{n.lower()}.usd" for n in L.DRESSING])
    build_video_stages()
    report = verify(root, placed, counts)
    print(json.dumps({k: v for k, v in report.items() if k != "issues"}, indent=1))
    print(f"{len(report['issues'])} issues")
    for i in report["issues"]:
        print("  ", i)
    return report


if __name__ == "__main__":
    main()
