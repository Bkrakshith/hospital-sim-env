"""Build the Hospital_G1 scene as layered USD. Only the fixtures layer authors physics (its articulations).

    usd/hospital_g1.usd      root: sublayers below, defaultPrim /World
    usd/building.usd         Isaac hospital with the prims of the re-plan deactivated (layout.BUILDING_HIDE through
                             hospital_fixtures.building_hide, for the fixture tier in HG1_FIXTURES, default tier2)
    usd/building_mods.usd    main-building partitions, screens, rebuilt walls and link junction boxes
                             (hospital_fixtures.static_partitions, world frame) as visual boxes in the building's own
                             wall materials; vision panels in its glass; panels of the new ceiling lights
    usd/wing_shell.usd       kit floors/ceilings/walls/doors of link, pre-op, PACU, corridor
    usd/or_suite.usd         i4h OR suite rotated into the wing + door cuts + prop moves (overrides only)
    usd/dressing_<room>.usd  furniture per wing room
    usd/dressing_bldg_<group>.usd  furniture of the main-building rooms (ward, ae, central), world frame
    usd/cameras.usd          viewing cameras
    usd/lighting.usd         room ceiling panels + hospital fittings (hidden fittings skipped) + BUILDING_LIGHTS
    usd/fixtures.usd         fire doorsets, bed lifts and buttons as PhysX articulations (scene/fixtures_usd.py,
                             geometry from hospital-sim-env/fixtures/hospital_fixtures.py)

Run inside Isaac Sim:  import runpy; runpy.run_path("<repo>/scene/build_hospital.py", run_name="__main__")
or standalone:         uv run --no-project --with usd-core --with numpy python scene/build_hospital.py
"""

import json
import math
import os
import re
import sys

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdShade, Vt

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import assets as A  # noqa: E402
import fixtures_usd as FX  # noqa: E402
import layout as L  # noqa: E402
import build_exterior as X  # noqa: E402  (drive + ambulance, usd/exterior.usd)

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

# Sky: the scene keeps ONE dome light. RTX uses only one, and the two assets bring one each:
#   - /World/Hospital/DomeLight (intensity 10000) points at NVIDIA/Assets/Skies/Cloudy/abandoned_parking_4k.hdr, which
#     the local asset packs do not have. When RTX picked it, the upload failed and the sky became a full-intensity
#     white dome, so outdoor views clipped at random.
#   - the OR suite's /World/Light/DomeLight (CloudyDay_04_04.hdr, dimmed to 360).
# The build deactivates the suite's dome (build_or_suite) and points the hospital's dome at the suite's overcast HDR,
# which both asset packs have (through the assets/i4h link). HG1_DOME_INTENSITY overrides the level.
DOME_TEXTURE = f"{A.OR_LIB}/HDRI/CloudyDay_04_04.hdr"
DOME_INTENSITY = float(os.environ.get("HG1_DOME_INTENSITY", "1000"))
SUITE_DOME = "Light/DomeLight"  # under the suite's /World

# hospital_fixtures (geometry + tiers). The fixture tier comes from HG1_FIXTURES (wing | tier1 | tier2 | full, default
# tier2). It decides which static door leaves the building keeps (building_hide) and whether the main-entrance glass
# goes (tier2 and up). Set it once for the build and for every consumer (fixtures.usd, MJCF, Isaac, ROS).
HF = FX.load_fixtures_module()


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


def build_building(spec):
    """building.usd: the Isaac hospital minus the prims the re-plan removes. Returns the report summary and the set of
    deactivated top-level names. Instanced prims cost no triangles when hidden; hiding removes their colliders."""
    st = new_stage("building.usd")
    hosp = st.DefinePrim("/World/Hospital", "Xform")
    add_ref(hosp, A.HOSPITAL_BUILDING)
    hidden = list(HOSPITAL_DOOR_LEAVES)
    hidden += [n for n in HF.building_hide(L, spec) if n not in hidden]
    for n in hidden:
        st.OverridePrim(f"/World/Hospital/{n}").SetActive(False)
    # Tier 2 and up: open the main entrance for the walk-in route. The two sliding glass units (the keys of
    # BUILDING_UNINSTANCE_HIDE) are deactivated at their instance roots, frames included, as g1-isaac-sim/make_stage.py
    # does for physics: their colliders live inside the instances, so only deactivation removes them, and the visual
    # scene then matches the physics (an open entrance, no free-standing mullions).
    entrance = []
    if HF.TIERS[spec["tier"]] >= 2:
        for unit in L.BUILDING_UNINSTANCE_HIDE:
            st.OverridePrim(f"/World/Hospital/{unit}").SetActive(False)
            entrance.append(f"/World/Hospital/{unit}")
    hidden += [u.split("/")[-1] for u in entrance]
    dome = st.OverridePrim("/World/Hospital/DomeLight")
    tex = Sdf.AssetPath(rel(DOME_TEXTURE))
    for name in ("inputs:texture:file", "texture:file"):  # the asset authors the old, unprefixed name
        dome.CreateAttribute(name, Sdf.ValueTypeNames.Asset).Set(tex)
    for name in ("inputs:intensity", "intensity"):
        dome.CreateAttribute(name, Sdf.ValueTypeNames.Float).Set(DOME_INTENSITY)
    st.GetRootLayer().Save()
    return ({"tier": spec["tier"], "hidden_prims": len(hidden), "main_entrance_opened": entrance,
             "dome": {"texture": DOME_TEXTURE, "intensity": DOME_INTENSITY,
                      "texture_found": os.path.exists(os.path.join(REPO, DOME_TEXTURE))}}, set(hidden), {})


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
    for room, (a, y, b, y1) in L.WING_DOORS.items():
        # The 4 m module runs along its +x from the origin, its wall centred on the grid line (+-0.1 m).
        # Horizontal (y const): no rotation, origin at the west end. Vertical (x const): +90 deg (as the
        # "W" straight), origin at the south end, so the module spans y0..y1 on x = a.
        if a == b:
            p = xf(st, f"{root}/Door_{room}", (a, min(y, y1), 0), 90, KIT_SCALE)
        else:
            p = xf(st, f"{root}/Door_{room}", (min(a, b), y, 0), 0, KIT_SCALE)
        add_ref(p, A.KIT_DOOR)
        switch_off(st, p, A.KIT_DOOR, DOOR_BARE + (MODULE_LIGHTS if LIGHTS != "kit" else []))
    # The joint between the 4 m link and the hospital wall (layout.JUNCTION_BOXES: the rebuilt wall pier and header of
    # the widened opening, the link header, the strip east of the opening, the FD60 side panel) is built with the other
    # static partitions in building_mods.usd, as Junction_<name> (hospital_fixtures.static_partitions).
    st.GetRootLayer().Save()
    return counts


# ---------------------------------------------------------------- main building: geometry probe


class BuildingMeshes:
    """World-space triangles of the active meshes of hospital.usd, with the bound material of each (pxr + numpy).

    build_building_mods() casts rays on them to pick the wall paint for each face of a new partition. verify() slices
    them at fixed heights to find dressing that cuts a building wall. The hospital sits at the world origin.
    """

    def __init__(self, hidden, glass):
        src = Usd.Stage.Open(os.path.join(REPO, A.HOSPITAL_BUILDING))
        root = src.GetDefaultPrim()
        skip = {root.GetPath().AppendPath(f"{u}/{r}") for u, rels in glass.items() for r in rels}
        xc = UsdGeom.XformCache()
        tris, owner, mat = [], [], []
        self.names, self.mats, self.mat_src = [], [], {}
        for top in root.GetChildren():
            if top.GetName() in hidden:
                continue
            self.names.append(top.GetName())
            it = iter(Usd.PrimRange(top, Usd.TraverseInstanceProxies()))
            for p in it:
                if p.GetPath() in skip:
                    it.PruneChildren()
                    continue
                if not p.IsA(UsdGeom.Mesh):
                    continue
                m = UsdGeom.Mesh(p)
                pts, idx, cnt = (m.GetPointsAttr().Get(), m.GetFaceVertexIndicesAttr().Get(),
                                 m.GetFaceVertexCountsAttr().Get())
                if not pts or not idx or not cnt:
                    continue
                M = np.array(xc.GetLocalToWorldTransform(p))
                P = (np.c_[np.array(pts, float), np.ones(len(pts))] @ M)[:, :3]
                idx, cnt = np.array(idx), np.array(cnt)
                starts = np.r_[0, np.cumsum(cnt)[:-1]]
                fan = [np.c_[idx[starts[cnt > k + 1]], idx[starts[cnt > k + 1] + k], idx[starts[cnt > k + 1] + k + 1]]
                       for k in range(1, int(cnt.max()) - 1)]
                if not fan:
                    continue
                T = P[np.concatenate(fan)]
                material, _ = UsdShade.MaterialBindingAPI(p).ComputeBoundMaterial()
                mname = material.GetPrim().GetName() if material else ""
                if mname not in self.mats:
                    self.mats.append(mname)
                    if material:
                        spec = material.GetPrim().GetPrimStack()[-1]  # the asset file that defines the material
                        self.mat_src[mname] = (spec.layer.identifier, spec.path)
                tris.append(T)
                owner.append(np.full(len(T), len(self.names) - 1))
                mat.append(np.full(len(T), self.mats.index(mname)))
        self.T = np.concatenate(tris)
        self.owner = np.concatenate(owner)
        self.mat = np.concatenate(mat)
        self.zmin, self.zmax = self.T[:, :, 2].min(1), self.T[:, :, 2].max(1)

    def ray_material(self, o, d, prefix="M_Wall_", max_t=40.0):
        """Material name of the nearest triangle hit by the horizontal ray o + t d (only materials whose name starts
        with prefix, a string or a tuple of strings)."""
        wall = np.array([m.startswith(prefix) for m in self.mats])
        sel = wall[self.mat] & (self.zmin <= o[2]) & (self.zmax >= o[2])
        T, mats = self.T[sel], self.mat[sel]
        o, d = np.asarray(o, float), np.asarray(d, float)
        e1, e2 = T[:, 1] - T[:, 0], T[:, 2] - T[:, 0]
        p = np.cross(d, e2)
        det = (e1 * p).sum(1)
        ok = np.abs(det) > 1e-12
        inv = np.where(ok, 1.0 / np.where(ok, det, 1.0), 0.0)
        s = o - T[:, 0]
        u = (s * p).sum(1) * inv
        q = np.cross(s, e1)
        v = (q * d).sum(1) * inv
        t = (q * e2).sum(1) * inv
        hit = ok & (u >= 0) & (v >= 0) & (u + v <= 1) & (t > 1e-4) & (t < max_t)
        if not hit.any():
            return None
        return self.mats[mats[hit][np.argmin(t[hit])]]

    def segments(self, z, prefixes=("Geo_", "Glass"), exclude=("Geo_M_Stairs", "Geo_M_DobleDoor")):
        """Plan segments (x0, y0, x1, y1) where the active wall meshes cross the plane at height z, and their owners."""
        keep = np.array([n.startswith(prefixes) and not n.startswith(exclude) for n in self.names])
        sel = keep[self.owner] & (self.zmin < z) & (self.zmax > z)
        T, own = self.T[sel], self.owner[sel]
        out, names = [], []
        for t, o in zip(T, own):
            pts = []
            for a, b in ((0, 1), (1, 2), (2, 0)):
                za, zb = t[a, 2], t[b, 2]
                if (za - z) * (zb - z) < 0:
                    pts.append(t[a, :2] + (t[b, :2] - t[a, :2]) * (z - za) / (zb - za))
            if len(pts) == 2:
                out.append((*pts[0], *pts[1]))
                names.append(self.names[o])
        return np.array(out).reshape(-1, 4), names


def _seg_hits_box(segs, names, box2, shrink=0.03):
    """Names of the segments that cross the plan box (x0, y0, x1, y1) shrunk by `shrink` (Liang-Barsky clip)."""
    x0, y0, x1, y1 = box2[0] + shrink, box2[1] + shrink, box2[2] - shrink, box2[3] - shrink
    if len(segs) == 0:
        return set()
    sx0, sy0, sx1, sy1 = segs.T
    near = (np.minimum(sx0, sx1) <= x1) & (np.maximum(sx0, sx1) >= x0) & (np.minimum(sy0, sy1) <= y1) & \
           (np.maximum(sy0, sy1) >= y0)
    hits = set()
    for i in np.nonzero(near)[0]:
        ax, ay, bx, by = segs[i]
        dx, dy = bx - ax, by - ay
        t0, t1, ok = 0.0, 1.0, True
        for p, q in ((-dx, ax - x0), (dx, x1 - ax), (-dy, ay - y0), (dy, y1 - ay)):
            if p == 0:
                if q < 0:
                    ok = False
                    break
            elif p < 0:
                t0 = max(t0, q / p)
            else:
                t1 = min(t1, q / p)
        if ok and t0 <= t1:
            hits.add(names[i])
    return hits


# ---------------------------------------------------------------- main building: partitions, screens, junctions

# Wall bands of the building's rooms, measured on hospital.usd: skirting trim 0..0.16 m, dado panel 0.16..0.96 m,
# painted plaster above. Each new box is drawn in the same bands. Each of its two long faces takes the material that
# rays from the face hit first on the facing building wall, at the band's height (so a face in a ward room takes the
# ward paint and a face on a corridor takes the corridor paint). Ends and tops take the paint of both sides when
# they agree, else the neutral plaster.
WALL_BANDS = ((0.0, 0.16), (0.16, 0.96), (0.96, 99.0))
# Material families a band may take (the skirting is a plastic trim, the dado panel and the paint are plaster).
BAND_MATERIALS = (("M_Wall_Plasti",), ("M_Wall_Plaster", "M_Wall_Cream"), ("M_Wall_Plaster", "M_Wall_Cream"))
FACE_SAMPLES = (0.2, 0.5, 0.8)  # ray positions along a face; the most frequent material wins
# UV density per band, measured on the building meshes (UV units per metre): trims 0.55, dado panels 0.45, walls 0.20.
BAND_UV_PER_M = (0.55, 0.45, 0.20)
DEFAULT_WALL_MATERIAL = "M_Wall_Plaster"
GLASS_MATERIAL = "M_Glass"
MODS_ROOT = "/World/BuildingMods"
LIGHT_PANEL_T = 0.015         # m, visible panel of a new ceiling light, under the roof slab


def _box_mesh(stage, path, lo, hi, face_mats, uv_per_m=0.2):
    """Axis-aligned box as a Mesh with per-face st (metres x uv_per_m) and one material subset per material.
    face_mats: material path per face, in the order -x, +x, -y, +y, -z, +z."""
    x, y, z = (lo[0], hi[0]), (lo[1], hi[1]), (lo[2], hi[2])
    corners = {  # outward, counter-clockwise from outside
        "-x": [(0, 0, 0), (0, 0, 1), (0, 1, 1), (0, 1, 0)], "+x": [(1, 0, 0), (1, 1, 0), (1, 1, 1), (1, 0, 1)],
        "-y": [(0, 0, 0), (1, 0, 0), (1, 0, 1), (0, 0, 1)], "+y": [(0, 1, 0), (0, 1, 1), (1, 1, 1), (1, 1, 0)],
        "-z": [(0, 0, 0), (0, 1, 0), (1, 1, 0), (1, 0, 0)], "+z": [(0, 0, 1), (1, 0, 1), (1, 1, 1), (0, 1, 1)]}
    normal = {"-x": (-1, 0, 0), "+x": (1, 0, 0), "-y": (0, -1, 0), "+y": (0, 1, 0), "-z": (0, 0, -1), "+z": (0, 0, 1)}
    uv_axes = {"x": (1, 2), "y": (0, 2), "z": (0, 1)}
    pts, nrm, st = [], [], []
    for f, cs in corners.items():
        a, b = uv_axes[f[1]]
        for c in cs:
            p = (x[c[0]], y[c[1]], z[c[2]])
            pts.append(Gf.Vec3f(*p))
            nrm.append(Gf.Vec3f(*normal[f]))
            st.append(Gf.Vec2f(p[a] * uv_per_m, p[b] * uv_per_m))
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(Vt.Vec3fArray(pts))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray([4] * 6))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(list(range(24))))
    mesh.CreateNormalsAttr(Vt.Vec3fArray(nrm))
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(*lo), Gf.Vec3f(*hi)]))
    UsdGeom.PrimvarsAPI(mesh).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray,
                                            UsdGeom.Tokens.vertex).Set(Vt.Vec2fArray(st))
    groups = {}
    for i, m in enumerate(face_mats):
        groups.setdefault(m, []).append(i)
    binding = UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim())
    if len(groups) == 1:
        binding.Bind(UsdShade.Material(stage.GetPrimAtPath(face_mats[0])))
        return
    for k, (m, faces) in enumerate(groups.items()):
        sub = binding.CreateMaterialBindSubset(f"mat{k}", Vt.IntArray(faces), UsdGeom.Tokens.face)
        UsdShade.MaterialBindingAPI.Apply(sub.GetPrim()).Bind(UsdShade.Material(stage.GetPrimAtPath(m)))


def build_building_mods(probe):
    """building_mods.usd: every static_partitions() box (world frame) as visual geometry in the building's materials,
    and a thin panel for each new ceiling light. Physics for the same boxes comes from the same function in
    g1-isaac-sim/make_stage.py (HG1_Colliders) and g1-mujoco-sim/build_world.py; this layer authors none."""
    st = new_stage("building_mods.usd")
    st.DefinePrim(MODS_ROOT, "Xform")
    st.DefinePrim(f"{MODS_ROOT}/Looks", "Scope")
    looks = {}

    def look(name):
        if name not in looks:
            path = f"{MODS_ROOT}/Looks/{name}"
            if name in probe.mat_src:
                ident, spec_path = probe.mat_src[name]
                prim = st.DefinePrim(path, "Material")
                prim.GetReferences().AddReference(os.path.relpath(ident, USD_DIR), spec_path)
            else:  # fallback (the glazing may be hidden at tier 2): a plain preview material
                mat = UsdShade.Material.Define(st, path)
                sh = UsdShade.Shader.Define(st, f"{path}/Shader")
                sh.CreateIdAttr("UsdPreviewSurface")
                glass = name == GLASS_MATERIAL
                sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
                    Gf.Vec3f(0.80, 0.88, 0.90) if glass else Gf.Vec3f(0.95, 0.95, 0.93))
                sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.05 if glass else 0.5)
                sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(0.25 if glass else 1.0)
                mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
            looks[name] = path
        return looks[name]

    sampled = {}
    count = {}
    for name, lo, hi, kind in HF.static_partitions(L):
        count[kind] = count.get(kind, 0) + 1
        path = f"{MODS_ROOT}/Partitions/{name}"
        if kind == "glass":
            _box_mesh(st, path, lo, hi, [look(GLASS_MATERIAL)] * 6)
            continue
        thin = 0 if hi[0] - lo[0] < hi[1] - lo[1] else 1  # the box's thickness axis
        c = [(a + b) / 2 for a, b in zip(lo, hi)]
        bands = [(max(lo[2], z0), min(hi[2], z1)) for z0, z1 in WALL_BANDS if min(hi[2], z1) - max(lo[2], z0) > 1e-3]
        along = 1 - thin
        for k, (z0, z1) in enumerate(bands):
            band = next(i for i, (a, b) in enumerate(WALL_BANDS) if a <= z0 < b)
            zs = (z0 + min(z1, z0 + 1.0)) / 2  # sample low in the upper band: under the top trims of the walls
            side = {}
            for sgn in (-1, 1):
                d = [0.0, 0.0, 0.0]
                d[thin] = float(sgn)
                votes = []
                for f in FACE_SAMPLES:
                    o = list(c)
                    o[thin] = (lo if sgn < 0 else hi)[thin] + 0.02 * sgn
                    o[along] = lo[along] + f * (hi[along] - lo[along])
                    o[2] = zs
                    m = probe.ray_material(o, d, prefix=BAND_MATERIALS[band])
                    if m:
                        votes.append(m)
                m = max(set(votes), key=votes.count) if votes else DEFAULT_WALL_MATERIAL
                side[sgn] = m
                sampled.setdefault(f"{name}{'-+'[sgn > 0]}", []).append(m)
            # Ends and top: the paint of both sides when they agree, else the neutral corridor plaster (a jamb seen
            # from a corridor must not take the colour of the room behind it).
            edge = side[1] if side[1] == side[-1] else DEFAULT_WALL_MATERIAL
            faces = [edge] * 6
            faces[2 * thin], faces[2 * thin + 1] = side[-1], side[1]
            lo_b, hi_b = list(lo), list(hi)
            lo_b[2], hi_b[2] = z0, z1
            _box_mesh(st, f"{path}_b{k}" if len(bands) > 1 else path, lo_b, hi_b, [look(m) for m in faces],
                      BAND_UV_PER_M[band])
    # Visible panels of the new ceiling lights (the light itself is in lighting.usd).
    for lname, (x, y, size) in L.BUILDING_LIGHTS.items():
        z1 = L.BUILDING_CEILING_Z
        box(st, f"{MODS_ROOT}/LightPanels/{lname}", (x - size / 2, y - size / 2, z1 - LIGHT_PANEL_T),
            (x + size / 2, y + size / 2, z1), color=(0.97, 0.97, 0.97))
    st.GetRootLayer().Save()
    return {"boxes": count, "materials": sorted(looks), "face_materials": {k: sorted(set(v)) for k, v in sampled.items()}}


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
    st.OverridePrim(f"/World/Wing/ORSuite/{SUITE_DOME}").SetActive(False)  # one sky only: the hospital's dome
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
    """Dressing frame of one room. rect is (x0, y0, x1, y1) in the frame of `root`'s parent: wing-local for the wing
    rooms (/World/Wing/Dressing), world for the main-building rooms (/World/Building/Dressing). wall_t is the wall
    offset of r.wall(): the kit's 0.10 m in the wing, 0.0 in the building (its room rects are measured wall faces)."""

    def __init__(self, stage, name, rect, placed, root="/World/Wing/Dressing", wall_t=L.WALL_T, frame="wing"):
        self.stage, self.name = stage, name
        self.x0, self.y0, self.x1, self.y1 = rect
        self.W, self.D = self.x1 - self.x0, self.y1 - self.y0
        self.root = f"{root}/{name}"
        stage.DefinePrim(self.root, "Xform")
        self.placed, self.wall_t, self.frame = placed, wall_t, frame

    def put(self, name, key, x, y, yaw=0.0, z=0.0, anchor="bottom", scale=1.0):
        info = asset_info(key)
        mn, mx = [v * scale for v in info["min"]], [v * scale for v in info["max"]]
        cx, cy = (mn[0] + mx[0]) / 2, (mn[1] + mx[1]) / 2
        h = mx[2] - mn[2]
        base = z - h if anchor == "top" else z
        wx, wy = self.x0 + x, self.y0 + y
        slot = xf(self.stage, f"{self.root}/{name}", (wx, wy, base), yaw)
        asset = xf(self.stage, f"{slot.GetPath()}/Asset", (-cx, -cy, -mn[2]), 0, info["mpu"] * scale)
        add_ref(asset, CATALOG[key])
        asset.SetInstanceable(True)  # identical props share one GPU copy of their (very dense) meshes
        hx, hy = (mx[0] - mn[0]) / 2, (mx[1] - mn[1]) / 2
        c, s = abs(math.cos(math.radians(yaw))), abs(math.sin(math.radians(yaw)))
        ex, ey = c * hx + s * hy, s * hx + c * hy
        self.placed.append({"room": self.name, "name": name, "key": key, "frame": self.frame,
                            "box": [wx - ex, wy - ey, base, wx + ex, wy + ey, base + h]})

    def wall(self, name, key, wall, u, z=0.0, scale=1.0):
        back = L.BACK.get(key, "-y")
        info = asset_info(key)
        axis = 0 if back.endswith("x") else 1
        depth = (info["max"][axis] - info["min"][axis]) * scale
        off = self.wall_t + depth / 2 + 0.01
        x, y = {"N": (u, self.D - off), "S": (u, off), "E": (self.W - off, u), "W": (off, u)}[wall]
        self.put(name, key, x, y, _OUT[wall] - _YAW[back], z, scale=scale)


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


BUILDING_DRESSING_ROOT = "/World/Building/Dressing"


def building_dressing_group(room):
    """Layer group of a main-building room: ward (C3), ae (A&E) or central (WCs, store, lift lobby)."""
    return "ward" if room.startswith("Ward_") else "ae" if room.startswith("AE_") else "central"


def build_building_dressing():
    """dressing_bldg_<group>.usd: furniture of layout.BUILDING_DRESSING, world frame (no wing transform)."""
    placed, groups = [], {}
    for name in L.BUILDING_DRESSING:
        groups.setdefault(building_dressing_group(name), []).append(name)
    layers = []
    for group, names in groups.items():
        st = new_stage(f"dressing_bldg_{group}.usd")
        st.DefinePrim("/World/Building", "Xform")
        st.DefinePrim(BUILDING_DRESSING_ROOT, "Xform")
        for name in names:
            L.BUILDING_DRESSING[name](Room(st, name, L.BUILDING_ROOMS[name]["rect"], placed, root=BUILDING_DRESSING_ROOT,
                                           wall_t=L.BUILDING_WALL_T, frame="world"))
        st.GetRootLayer().Save()
        layers.append(f"dressing_bldg_{group}.usd")
    return placed, layers


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


def build_lighting(hidden=()):
    """lighting.usd. Returns the light counts: wing room panels, hospital fittings, new building panels."""
    st = new_stage("lighting.usd")
    st.DefinePrim("/World/Wing", "Xform")
    st.DefinePrim("/World/Wing/Lighting", "Xform")
    st.DefinePrim("/World/HospitalLighting", "Xform")
    rooms = dict(L.KIT_ROOMS)
    rooms.update({k: v for k, v in L.SUITE_ROOMS.items() if k in L.LIT_SUITE_ROOMS})
    counts = {"wing_panels": _panels(st, "/World/Wing/Lighting", rooms, PANEL_Z),
              "hospital_fittings": _hospital_fixtures(st, "/World/HospitalLighting", hidden),
              "building_panels": _building_lights(st, "/World/HospitalLighting/New")}
    st.GetRootLayer().Save()
    counts["total"] = sum(counts.values())
    return counts


def _building_lights(st, root):
    """layout.BUILDING_LIGHTS: a rect light per new sub-room (pods, en-suites, office lobby, public WCs). They replace
    the fittings that the new partitions cut (hidden in BUILDING_HIDE["light_fittings"]). Same per-area level as the
    room panels: the light carries the floor area of the room it sits in."""
    n = 0
    for name, (x, y, size) in L.BUILDING_LIGHTS.items():
        room = next((r["rect"] for r in L.BUILDING_ROOMS.values()
                     if r["rect"][0] < x < r["rect"][2] and r["rect"][1] < y < r["rect"][3]), None)
        area = (room[2] - room[0]) * (room[3] - room[1]) if room else 4.0
        light = UsdLux.RectLight.Define(st, f"{root}/{name}")
        light.AddTranslateOp().Set(Gf.Vec3d(x, y, L.BUILDING_LIGHT_Z))  # emits along -Z: straight down
        light.GetWidthAttr().Set(size)
        light.GetHeightAttr().Set(size)
        light.GetIntensityAttr().Set(KIT_LUX_PER_M2 * area / (size * size) * LIGHT_LEVEL)
        light.CreateEnableColorTemperatureAttr(True)
        light.CreateColorTemperatureAttr(LIGHT_KELVIN)
        n += 1
    return n


def _hospital_fixtures(st, root, hidden=()):
    """Turn on every hospital ceiling fitting: a light just under each fixture mesh, same per-area level. Fittings
    that the build deactivates (cut by new partitions) get no light."""
    src = Usd.Stage.Open(os.path.join(REPO, A.HOSPITAL_BUILDING))
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
    n = 0
    for prim in src.GetDefaultPrim().GetChildren():
        if prim.GetName() in hidden:
            continue
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


# ---------------------------------------------------------------- fixtures


def build_fixtures():
    """usd/fixtures.usd: moving fixtures with physics (see fixtures_usd.py). Returns the report summary."""
    st = new_stage("fixtures.usd")
    car = L.LIFT_SPEC["car_w"] * L.LIFT_SPEC["car_d"]
    summary = FX.author(st, L, car_light_intensity=KIT_LUX_PER_M2 * car / (1.6 * 1.0) * LIGHT_LEVEL)
    st.GetRootLayer().Save()
    return summary


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
    cx, cy = ox + 5.0, oy - 10.0  # wing x -14..24: the wing, the lift lobby and the lift cars
    cam("Cam_Plan", (cx, cy, 3.42), (cx, cy, 0.0), 0, ortho_width=38.0)
    st.GetRootLayer().Save()


# ---------------------------------------------------------------- root + verification


def build_root(dressing_layers):
    st = new_stage("hospital_g1.usd")
    subs = (["./cameras.usd"] + (["./lighting.usd"] if LIGHTS == "rooms" else []) + ["./fixtures.usd", "./exterior.usd"]
            + [f"./{n}" for n in dressing_layers]
            + ["./building_mods.usd", "./or_suite.usd", "./wing_shell.usd", "./building.usd"])
    st.GetRootLayer().subLayerPaths = subs
    st.GetRootLayer().documentation = ("Hospital_G1: Isaac hospital + surgical wing. Physics only on /World/Fixtures "
                                       "(fire doors, lifts, buttons).")
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


# Dressing stacked on purpose: a computer on a desk, a mirror over a basin, a defibrillator on a trolley.
STACKED = {("SM_Computer_02b", "SM_Desk_04a"), ("Defibrillator_A", "SM_SupplyCart_02a"),
           ("SM_Mirror", "SM_Washbasin"), ("Defibrillator_A", "EmergencyTrolley_A")}
SLICE_Z = (0.3, 1.0, 1.6)  # heights at which dressing must not cut a building wall mesh


def _fixture_zones(spec):
    """Keep-clear boxes for the fixtures, WORLD frame [x0, y0, z0, x1, y1, z1]: the swept area of every fire-door
    leaf over its whole range, and the bed-turning space in front of each lift (HBN 00-04 Fig 28: 4.31 m wide,
    3.37 m deep). The spec coordinates are wing-local; spec["wing_origin"] moves them to the world frame."""
    ox, oy, _ = spec["wing_origin"]
    zones = []
    for d in spec["doors"]:
        for lf in d["leaves"]:
            (hx, hy), w = lf["hinge"], lf["width"] + 0.005
            lo, hi = lf["range"]
            pts = [(hx, hy)]
            for k in range(41):
                a = lf["closed_yaw"] + lo + (hi - lo) * k / 40
                pts.append((hx + w * math.cos(a), hy + w * math.sin(a)))
            xs, ys = [p[0] + ox for p in pts], [p[1] + oy for p in pts]
            zones.append((f"fire door {d['name']} leaf swing", [min(xs), min(ys), 0.0, max(xs), max(ys), 2.0]))
    for l in spec["lifts"]:
        fx, cy = l["front_x"] + ox, l["center_y"] + oy
        zones.append((f"lift landing {l['name']}", [fx - 3.37, cy - 4.31 / 2, 0.0, fx, cy + 4.31 / 2, 2.0]))
    return zones


def _doorway_zone(seg, d):
    """Clear zone d metres each side of a doorway segment (x0, y0, x1, y1), up to 2 m high."""
    x0, y0, x1, y1 = seg
    if abs(y0 - y1) < 1e-9:
        return [min(x0, x1), y0 - d, 0.0, max(x0, x1), y0 + d, 2.0]
    return [x0 - d, min(y0, y1), 0.0, x0 + d, max(y0, y1), 2.0]


def _building_prims(hidden):
    """World boxes [x0, y0, z0, x1, y1, z1] of the active top-level prims of hospital.usd, by name."""
    src = Usd.Stage.Open(os.path.join(REPO, A.HOSPITAL_BUILDING))
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
    out, names = {}, set()
    for p in src.GetDefaultPrim().GetChildren():
        names.add(p.GetName())
        if p.GetName() in hidden:
            continue
        r = cache.ComputeWorldBound(p).ComputeAlignedRange()
        if not r.IsEmpty():
            out[p.GetName()] = list(r.GetMin()) + list(r.GetMax())
    return out, names


def _verify_building(spec_full, items, probe, hidden):
    """Checks of the main-building re-plan (design_final.md s.13, ported from scripts/check.py s.1-5), world frame.
    items: every placed dressing item, wing and building, with world boxes. Returns (issues, notes, summary)."""
    issues, notes = [], []
    prims, all_names = _building_prims(hidden)
    missing = sorted(n for n in hidden if n not in all_names)
    if missing:
        issues.append(f"hidden prims not found in hospital.usd: {missing}")
    # Loose building furniture that stays (below 2 m), and every active prim that could stand in a lift or a box.
    furn = [(n, b) for n, b in prims.items() if not n.startswith(("Geo_", "Light", "Cube", "SM_Door", "GlassDoor"))
            and b[2] < 2.0]
    solid = [(n, b) for n, b in prims.items() if not n.startswith(("Light", "Geo_Roof", "Geo_M3_Ceiling"))
             and b[5] > 0.02 and "Floor" not in n]
    bitems = [it for it in items if it["src"] == "building"]
    # 1. Building dressing inside its room (the rects are the measured inner faces of the walls).
    for it in bitems:
        x0, y0, x1, y1 = L.BUILDING_ROOMS[it["room"]]["rect"]
        b = it["box"]
        if b[0] < x0 - 0.005 or b[1] < y0 - 0.005 or b[3] > x1 + 0.005 or b[4] > y1 + 0.005:
            issues.append(f"outside room: {it['room']}/{it['name']} {[round(v, 2) for v in b]}")
    # 2. Dressing against the building's own furniture and walls (mesh slices at 0.3, 1.0 and 1.6 m).
    for it in items:
        if it["src"] == "suite":
            continue
        for n, b in furn:
            if _overlap(it["box"], b):
                issues.append(f"dressing {it['room']}/{it['name']} hits building {n}")
    slices = {z: probe.segments(z) for z in SLICE_Z}
    for it in bitems:
        b = it["box"]
        for z in SLICE_Z:
            if b[2] <= z <= b[5]:
                h = _seg_hits_box(*slices[z], (b[0], b[1], b[3], b[4]))
                if h:
                    issues.append(f"dressing {it['room']}/{it['name']} cuts building wall {sorted(h)[:3]} at z {z}")
                    break
    # 3. Building doorways: 1 m each side clear of dressing and of building furniture.
    for dname, seg in L.BUILDING_DOORWAYS.items():
        zone = _doorway_zone(seg, L.DOOR_CLEARANCE)
        for it in items:
            if _overlap(zone, it["box"]):
                issues.append(f"doorway {dname} blocked by {it['room']}/{it['name']}")
        for n, b in furn:
            if _overlap(zone, b):
                issues.append(f"doorway {dname} blocked by building {n}")
    # 4. Fire-door swings (every door of the full tier) and lift landings against building furniture.
    for zname, zone in _fixture_zones(spec_full):
        for n, b in furn:
            if _overlap(zone, b):
                issues.append(f"{zname} blocked by building {n}")
    # 5. Partitions, screens, rebuilt walls, junction boxes: clear of building furniture and of dressing.
    parts = HF.static_partitions(L)
    for name, lo, hi, kind in parts:
        b = list(lo) + list(hi)
        for n, fb in furn:
            if _overlap(b, fb):
                issues.append(f"partition {name} hits building furniture {n}")
        for it in items:
            if it["src"] != "suite" and _overlap(b, it["box"], tol=0.005):
                issues.append(f"partition {name} hits dressing {it['room']}/{it['name']}")
        if kind != "glass" and lo[2] < 1.0 < hi[2]:
            h = _seg_hits_box(*slices[1.0], (lo[0], lo[1], hi[0], hi[1]), shrink=0.06)
            if h:
                notes.append(f"partition {name} touches wall meshes {sorted(h)} (expected only at its ends)")
    # 6. Main lift bank: static boxes and cars clear of every active building prim.
    ox, oy, _ = spec_full["wing_origin"]
    main_lifts = [l for l in spec_full["lifts"] if l.get("bank") == "Main"]
    for n, lo, hi in spec_full["static_boxes"]:
        if not n.startswith(("fx_Main_", "fx_MainLift")):
            continue
        b = [lo[0] + ox, lo[1] + oy, lo[2], hi[0] + ox, hi[1] + oy, hi[2]]
        for pn, pb in solid:
            if _overlap(b, pb, tol=0.005):
                issues.append(f"lift static {n} hits building {pn}")
    for l in main_lifts:
        c = l["car"]
        car = [c["x0"] + ox - 0.04, c["y0"] + oy - 0.05, 0.0, c["x1"] + ox + 0.05, c["y1"] + oy + 0.05, c["h"] + 0.05]
        for pn, pb in solid:
            if _overlap(car, pb, tol=0.005):
                issues.append(f"lift car {l['name']} hits building {pn}")
    # 7. Every room of the re-plan has a light: an active building fitting or a new panel.
    fit = [b for n, b in prims.items() if any(n.startswith(f[0]) for f in L.HOSPITAL_FIXTURES)]
    for room, r in L.BUILDING_ROOMS.items():
        x0, y0, x1, y1 = r["rect"]
        lit = any(x0 < (b[0] + b[3]) / 2 < x1 and y0 < (b[1] + b[4]) / 2 < y1 for b in fit)
        lit |= any(x0 < x < x1 and y0 < y < y1 for x, y, _ in L.BUILDING_LIGHTS.values())
        if not lit:
            issues.append(f"room {room} has no light")
    summary = {"rooms": len(L.BUILDING_ROOMS), "dressing_items": len(bitems),
               "dressing_per_room": {r: sum(1 for p in bitems if p["room"] == r) for r in L.BUILDING_DRESSING},
               "doorways_checked": len(L.BUILDING_DOORWAYS), "static_partitions": len(parts),
               "hidden_prims": len(hidden), "main_lift_cars": len(main_lifts)}
    return issues, notes, summary


def verify(root_path, placed, shell_counts, fixtures=None, building_placed=(), probe=None, hidden=(), extra=None):
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
                            "src": "suite", "key": None, "box": list(mn) + list(mx)})
    # All checks run in the world frame: the wing dressing is authored wing-local under /World/Wing.
    wing = [dict(it, src="wing", box=[it["box"][0] + ox, it["box"][1] + oy, it["box"][2],
                                      it["box"][3] + ox, it["box"][4] + oy, it["box"][5]]) for it in placed]
    bldg = [dict(it, src="building") for it in building_placed]
    items = wing + bldg

    issues = []
    # 1. Doorway clearance (1 m each side of every wing opening, up to 2 m high).
    for dname, (x0, y0, x1, y1) in L.DOORWAYS.items():
        zone = _doorway_zone((x0 + ox, y0 + oy, x1 + ox, y1 + oy), L.DOOR_CLEARANCE)
        for it in items + suite_props:
            if _overlap(zone, it["box"]):
                issues.append(f"doorway {dname} blocked by {it['room']}/{it['name']}")
    # 1b. Fire-door swings and lift landings, every door of the full tier (so no tier can find a blocked swing). The
    # suite props count too: the Theatre and ScrubStore leaves swing into suite rooms (wall-mounted props above the
    # 2.0 m zone top, such as signs over the doors, do not).
    spec_full = HF.build_spec(L, "full")
    for zname, zone in _fixture_zones(spec_full):
        for it in items + suite_props:
            if _overlap(zone, it["box"]):
                issues.append(f"{zname} blocked by {it['room']}/{it['name']}")
    # 2. New or moved items overlapping anything else (stacked pairs excepted).
    movers = items + [s for s in suite_props if s["moved"]]
    others = items + suite_props
    seen = set()
    for a in movers:
        for b in others:
            if a is b or (id(b), id(a)) in seen or (a.get("group") and a.get("group") == b.get("group")):
                continue
            seen.add((id(a), id(b)))
            if (a["key"], b["key"]) in STACKED or (b["key"], a["key"]) in STACKED:
                continue
            if _overlap(a["box"], b["box"]):
                issues.append(f"overlap {a['room']}/{a['name']} <-> {b['room']}/{b['name']}")
    # 3. Wing dressing stays inside its room (0.09 m off the kit wall line).
    rooms = dict(L.KIT_ROOMS)
    rooms.update(L.SUITE_ROOMS)
    for it in placed:
        x0, y0, x1, y1 = rooms[it["room"]]
        b = it["box"]
        if b[0] < x0 + 0.09 or b[1] < y0 + 0.09 or b[3] > x1 - 0.09 or b[4] > y1 - 0.09:
            issues.append(f"outside room: {it['room']}/{it['name']} {[round(v, 2) for v in b]}")
    # 3b. Main building: rooms, walls, doorways, swings, partitions, lift bank, lights.
    building = None
    if probe is not None:
        b_issues, b_notes, building = _verify_building(spec_full, items, probe, set(hidden))
        issues += b_issues
        building["notes"] = b_notes
    # 4. Physics authored in our layers: only the fixtures layer may author it (listed under "fixtures").
    physics = []
    for layer_path in st.GetRootLayer().subLayerPaths:
        lyr = Sdf.Layer.FindOrOpen(os.path.join(USD_DIR, layer_path))

        def visit(path):
            spec = lyr.GetObjectAtPath(path)
            if spec and hasattr(spec, "GetInfo") and spec.HasInfo("apiSchemas"):
                if any("Physics" in str(t) or "Physx" in str(t) for t in spec.GetInfo("apiSchemas").GetAddedOrExplicitItems()):
                    if not str(path).startswith(FX.ROOT):
                        physics.append(str(path))
        lyr.Traverse(Sdf.Path.absoluteRootPath, visit)

    exterior = X.check(st)
    issues += exterior.pop("issues")
    report = {
        "root": os.path.relpath(root_path, REPO),   # relative to hospital-sim-env/isaac/
        "gpu_triangles_M": round(_gpu_triangles(st) / 1e6, 2),
        "shell": shell_counts,
        "dressing_items": len(placed),
        "dressing_per_room": {r: sum(1 for p in placed if p["room"] == r) for r in L.DRESSING},
        "suite_props": len(suite_props),
        "building": building,
        **(extra or {}),
        "physics_specs_authored": physics,
        "fixtures": fixtures,
        "exterior": exterior,
        "issues": issues,
    }
    with open(os.path.join(USD_DIR, "build_report.json"), "w") as f:
        json.dump(report, f, indent=1)
    return report


def main():
    os.makedirs(USD_DIR, exist_ok=True)
    spec = HF.build_spec(L)  # the active fixture tier (HG1_FIXTURES, default tier2)
    building, hidden, glass = build_building(spec)
    probe = BuildingMeshes(hidden, glass)
    counts = build_wing_shell()
    mods = build_building_mods(probe)
    build_or_suite()
    placed = build_dressing()
    bplaced, blayers = build_building_dressing()
    fixtures = build_fixtures()
    build_cameras()
    lights = build_lighting(hidden) if LIGHTS == "rooms" else None
    X.build(new_stage)
    root = build_root([f"dressing_{n.lower()}.usd" for n in L.DRESSING] + blayers)
    build_video_stages()
    report = verify(root, placed, counts, fixtures, building_placed=bplaced, probe=probe, hidden=hidden,
                    extra={"tier": spec["tier"], "building_layer": building, "building_mods": mods, "lights": lights})
    print(json.dumps({k: v for k, v in report.items() if k not in ("issues", "building_mods")}, indent=1))
    print(f"{len(report['issues'])} issues")
    for i in report["issues"]:
        print("  ", i)
    return report


if __name__ == "__main__":
    main()
