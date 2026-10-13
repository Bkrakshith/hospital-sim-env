"""Moving building fixtures as USD with PhysX articulations: usd/fixtures.usd (called by build_hospital.py).

The geometry and the joint names come from hospital-sim-env/fixtures/hospital_fixtures.py (`build_spec`), so
the Isaac and MuJoCo scenes stay the same. This is the only hospital-sim-env layer that authors physics.
The fixture tier comes from $HG1_FIXTURES (wing, tier1, tier2 = default, full), as for every other consumer of the
spec. /World/Fixtures records it in customData "hg1:tier"; g1-isaac-sim/fixtures_isaac.py reads it at run time.

    /World/Fixtures                 Xform at WING_ORIGIN (children are in wing-local metres)
      Doors/<door>                  articulation: Base (fixed to the world) + Leaf0, Leaf1 (revolute about +z)
      Doors1/<door>                 articulation: Base + Leaf0 (single-leaf doorsets)
      Lifts/<lift>                  articulation: Base + PanelA, PanelB (prismatic) + BtnOpen, BtnClose (prismatic)
      Calls/<call>                  articulation: Base + Btn (prismatic)
      Static/                       static colliders and visuals: lift fronts, cars, sills, door heads, transoms
      CollisionGroups/Moving|Static the moving parts do not collide with the static parts (they sit in walls)

One tensor-view group per spec group (spec["groups"]): every articulation in a group has the same links and joints
(Doors 2 revolute DOFs, Doors1 1, Lifts 4 prismatic, Calls 1), so one view per group (/World/Fixtures/<group>/*)
covers it. PhysX only batches articulations with identical link AND joint names, so the joint prims have generic
names (fx_leaf0, fx_door_a, fx_Open, fx_button, ...) and carry the spec joint name (fx_CrossCorridor_leaf0, ...) in
customData "hg1:joint". No drives: the simulator applies the efforts of FixtureController as joint forces.
Bodies are authored at FixtureController.initial_q() (held-open doors at +-90 deg), with a matching joint state.

Doors of the main building (spec "source_frame" world):
- In screen openings (W/E_Compartment, W/E_Stair, Main_LiftLobby): the screen (hospital_fixtures.static_partitions)
  has a head piece over the opening from z 2.12, so no transom and no head stop are built here.
- In the asset's own door frames (FD30S hazard rooms, D07/D34 exits): the leaf visuals fill the measured frame hole
  (BUILDING_FRAMES); the leaf colliders keep the spec size (the same as MuJoCo). The frame and threshold colliders of
  the asset (triangle meshes) are added to the Static collision group, so the leaves never touch them.
"""

import importlib.util
import math
import os

from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade

HERE = os.path.dirname(os.path.abspath(__file__))
FIXTURES_PY = os.path.normpath(os.path.join(HERE, "..", "..", "fixtures", "hospital_fixtures.py"))
ROOT = "/World/Fixtures"


def load_fixtures_module():
    spec = importlib.util.spec_from_file_location("hospital_fixtures", FIXTURES_PY)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


# name: (base colour, roughness, metallic, opacity, emissive colour)
MATERIALS = {
    "Laminate": ((0.70, 0.57, 0.42), 0.55, 0.0, 1.0, None),   # door leaf facing
    "Glass": ((0.80, 0.88, 0.90), 0.05, 0.0, 0.25, None),     # vision panel
    "Stainless": ((0.78, 0.78, 0.77), 0.30, 1.0, 1.0, None),  # plates, lift panels, car walls
    "DarkSteel": ((0.30, 0.30, 0.32), 0.40, 0.8, 1.0, None),  # sills, handles
    "LiftFront": ((0.55, 0.60, 0.66), 0.60, 0.0, 1.0, None),  # contrasts with the walls (AD M 3.34)
    "Paint": ((0.90, 0.90, 0.88), 0.80, 0.0, 1.0, None),      # transom, bulkhead
    "CarFloor": ((0.22, 0.24, 0.27), 0.70, 0.0, 1.0, None),
    "CarRoof": ((0.95, 0.95, 0.95), 0.50, 0.0, 1.0, None),
    "ButtonCap": ((0.85, 0.85, 0.85), 0.25, 0.7, 1.0, None),
    "ButtonRing": ((1.0, 0.65, 0.20), 0.40, 0.0, 1.0, (1.0, 0.55, 0.12)),
    "Indicator": ((0.03, 0.03, 0.03), 0.30, 0.0, 1.0, None),
    "IndicatorLit": ((0.10, 0.90, 0.25), 0.40, 0.0, 1.0, (0.10, 0.90, 0.25)),
}


# Asset door frames that hold main-building doorsets (world frame, measured with pxr on hospital.usd, lite mirror):
#   head    underside of the frame head (or of the wall hole for D07/D34, whose DobleDoor frames are hidden), m
#   sill    top of the threshold strip (Geo_M_DoorFloor*), m, or None
#   wall_t  half depth of the wall at the doorway (exit sign offset), m
#   filter  asset prims under /World/Hospital that go into the Static collision group: the leaves sit in these
#           frames and over these thresholds (the 25 mm threshold is above the 10 mm leaf undercut), and the
#           building's triangle colliders are otherwise not filtered against the moving parts.
# Hole widths (1.243 m single frames, 2.484 m D07, 2.477 m D34) match the spec hinges at jamb + 30 mm.
_FD30_FRAME_HEAD = 2.2101
BUILDING_FRAMES = {
    "AE_DirtyUtility": {"head": _FD30_FRAME_HEAD, "sill": None, "wall_t": 0.087,
                        "filter": ["Geo_M_DoorFrame21_32"]},
    "AE_CleanLinen": {"head": _FD30_FRAME_HEAD, "sill": None, "wall_t": 0.087, "filter": ["Geo_M_DoorFrame20"]},
    "Central_Store": {"head": _FD30_FRAME_HEAD, "sill": 0.0254, "wall_t": 0.087,
                      "filter": ["Geo_M_DoorFrame47", "Geo_M_DoorFloor6"]},
    "Pharmacy1": {"head": _FD30_FRAME_HEAD, "sill": None, "wall_t": 0.087, "filter": ["Geo_M_DoorFrame54"]},
    "Pharmacy2": {"head": _FD30_FRAME_HEAD, "sill": None, "wall_t": 0.087, "filter": ["Geo_M_DoorFrame53"]},
    "Ward_Cleaners": {"head": _FD30_FRAME_HEAD, "sill": 0.0254, "wall_t": 0.087,
                      "filter": ["Geo_M_DoorFrame40", "Geo_M_DoorFloor17"]},
    "Ward_CleanLinen": {"head": _FD30_FRAME_HEAD, "sill": 0.0254, "wall_t": 0.087,
                        "filter": ["Geo_M_DoorFrame42", "Geo_M_DoorFloor19"]},
    "Ward_DirtyUtility": {"head": _FD30_FRAME_HEAD, "sill": 0.0254, "wall_t": 0.087,
                          "filter": ["Geo_M_DoorFrame44", "Geo_M_DoorFloor21"]},
    "D07_Exit": {"head": 2.3246, "sill": None, "wall_t": 0.05, "filter": []},
    "D34_Exit": {"head": 2.3137, "sill": None, "wall_t": 0.05, "filter": []},
}
BUILDING_PRIM_ROOT = "/World/Hospital"   # build_hospital.build_building references hospital.usd here
LEAF_TOP_GAP = 0.004                     # m, leaf visual top below the frame head
LEAF_SILL_GAP = 0.006                    # m, leaf visual bottom above a threshold

# Blue mandatory fire-door signs (BS 5499-4 / ISO 7010 M001 style, white text on safety blue), one texture per kind.
# The textures are drawn with PIL by `python scene/fixtures_usd.py --signs` and kept in git, like fire_exit.png.
SIGN_DIR = "../textures/signs"           # relative to usd/
SIGNS = {  # kind: (file, lines of text)
    "keep_shut": ("fire_door_keep_shut.png", ("FIRE DOOR", "KEEP SHUT")),
    "keep_clear": ("fire_door_keep_clear.png", ("AUTOMATIC FIRE DOOR", "KEEP CLEAR")),
    "keep_locked": ("fire_door_keep_locked.png", ("FIRE DOOR", "KEEP LOCKED")),
}
SIGN_SIZE = (0.30, 0.10)                 # m, width x height, on the leaf top rail (above the vision panel)
SIGN_BLUE = (0, 82, 147)                 # safety blue (ISO 3864-4, approx. sRGB)


def sign_kind(d):
    """Sign on the leaves of door entry d: held open on magnets -> keep clear; locked stores -> keep locked."""
    if d["hold"] is not None:
        return "keep_clear"
    if "locked" in (d.get("use") or ""):
        return "keep_locked"
    return "keep_shut"


def draw_sign_textures(out_dir=None):
    """Write the SIGNS textures (needs Pillow): uv run --no-project --with pillow --with usd-core python
    scene/fixtures_usd.py --signs. Generic signs drawn here, not copies of a product."""
    from PIL import Image, ImageDraw, ImageFont
    out_dir = out_dir or os.path.normpath(os.path.join(HERE, "..", "textures", "signs"))
    os.makedirs(out_dir, exist_ok=True)
    W, H = int(SIGN_SIZE[0] * 3000), int(SIGN_SIZE[1] * 3000)
    font = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
    for fname, lines in SIGNS.values():
        img = Image.new("RGB", (W, H), SIGN_BLUE)
        dr = ImageDraw.Draw(img)
        dr.rectangle((8, 8, W - 9, H - 9), outline=(250, 250, 248), width=6)
        px = int(H * 0.36)
        while True:  # largest size at which every line fits
            f = ImageFont.truetype(font, px) if os.path.exists(font) else ImageFont.load_default(px)
            boxes = [dr.textbbox((0, 0), t, font=f) for t in lines]
            if max(b[2] - b[0] for b in boxes) <= W * 0.88 or px < 12:
                break
            px -= 2
        lh = max(b[3] - b[1] for b in boxes)
        gap = lh * 0.35
        y = (H - (len(lines) * lh + (len(lines) - 1) * gap)) / 2
        for t, b in zip(lines, boxes):
            dr.text(((W - (b[2] - b[0])) / 2 - b[0], y - b[1]), t, fill=(250, 250, 248), font=f)
            y += lh + gap
        img.save(os.path.join(out_dir, fname))
        print(os.path.join(out_dir, fname), img.size)


def _materials(st):
    UsdGeom.Scope.Define(st, f"{ROOT}/Looks")
    out = {}
    for name, (col, rough, metal, opac, emis) in MATERIALS.items():
        mat = UsdShade.Material.Define(st, f"{ROOT}/Looks/{name}")
        sh = UsdShade.Shader.Define(st, f"{ROOT}/Looks/{name}/Shader")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*col))
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metal)
        if opac < 1.0:
            sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opac)
            sh.CreateInput("ior", Sdf.ValueTypeNames.Float).Set(1.5)
        if emis:
            sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*emis))
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        out[name] = mat
    for kind, (fname, _lines) in SIGNS.items():
        out["Sign_" + kind] = _texture_material(st, f"{ROOT}/Looks/Sign_{kind}", f"{SIGN_DIR}/{fname}", emissive=False)
    return out


def _texture_material(st, path, file, emissive):
    """UsdPreviewSurface with a texture on diffuse (and emission for lit signs), read through primvar st."""
    mat = UsdShade.Material.Define(st, path)
    sh = UsdShade.Shader.Define(st, f"{path}/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.5)
    tex = UsdShade.Shader.Define(st, f"{path}/Texture")
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(file)
    tex.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
    rd = UsdShade.Shader.Define(st, f"{path}/UV")
    rd.CreateIdAttr("UsdPrimvarReader_float2")
    rd.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(rd.ConnectableAPI(), "result")
    rgb = tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(rgb)
    if emissive:
        sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(rgb)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def _quad(st, path, P, w, h, normal, mat):
    """Textured quad: P(u, v) gives the point at (u, v) on the sign; uv (0, 0) is the bottom left seen from the front."""
    pts = [P(-w / 2, -h / 2), P(w / 2, -h / 2), P(w / 2, h / 2), P(-w / 2, h / 2)]
    m = UsdGeom.Mesh.Define(st, path)
    m.CreatePointsAttr([Gf.Vec3f(*p) for p in pts])
    m.CreateFaceVertexCountsAttr([4])
    m.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    m.CreateNormalsAttr([Gf.Vec3f(*normal)] * 4)
    m.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    m.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    m.CreateExtentAttr([Gf.Vec3f(*[min(p[i] for p in pts) for i in range(3)]),
                        Gf.Vec3f(*[max(p[i] for p in pts) for i in range(3)])])
    UsdGeom.PrimvarsAPI(m).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex).Set(
        [Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)])
    _bind(m.GetPrim(), mat)
    return m.GetPrim()


def _bind(prim, mat):
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat)


def _xform(st, path, t=(0, 0, 0), yaw_deg=0.0):
    x = UsdGeom.Xform.Define(st, path)
    x.AddTranslateOp().Set(Gf.Vec3d(*t))
    if yaw_deg:
        x.AddRotateZOp().Set(float(yaw_deg))
    return x.GetPrim()


def _cube(st, path, center, size, mat=None, collide=False, guide=False):
    c = UsdGeom.Cube.Define(st, path)
    c.CreateSizeAttr(1.0)
    c.AddTranslateOp().Set(Gf.Vec3d(*center))
    c.AddScaleOp().Set(Gf.Vec3f(*size))
    if guide:  # collision-only shape, like the colliders of g1-isaac-sim/make_stage.py
        c.CreatePurposeAttr(UsdGeom.Tokens.guide)
        c.CreateVisibilityAttr(UsdGeom.Tokens.invisible)
    elif mat is not None:
        c.CreateDisplayColorAttr([Gf.Vec3f(*MATERIALS[mat.GetPrim().GetName()][0])])
        _bind(c.GetPrim(), mat)
    if collide:
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
    return c.GetPrim()


def _box(st, path, lo, hi, mat=None, collide=False, guide=False):
    return _cube(st, path, [(a + b) / 2 for a, b in zip(lo, hi)], [b - a for a, b in zip(lo, hi)], mat, collide, guide)


def _cyl(st, path, center, axis, radius, height, mat):
    c = UsdGeom.Cylinder.Define(st, path)
    c.CreateAxisAttr(axis)
    c.CreateRadiusAttr(radius)
    c.CreateHeightAttr(height)
    c.AddTranslateOp().Set(Gf.Vec3d(*center))
    c.CreateDisplayColorAttr([Gf.Vec3f(*MATERIALS[mat.GetPrim().GetName()][0])])
    _bind(c.GetPrim(), mat)
    return c.GetPrim()


def _quat_z(rad):
    return Gf.Quatf(math.cos(rad / 2), 0.0, 0.0, math.sin(rad / 2))


def _body(st, path, t, yaw_rad, mass, com=(0, 0, 0), inertia=(1e-3, 1e-3, 1e-3)):
    p = _xform(st, path, t, math.degrees(yaw_rad))
    UsdPhysics.RigidBodyAPI.Apply(p)
    m = UsdPhysics.MassAPI.Apply(p)
    m.CreateMassAttr(float(mass))
    m.CreateCenterOfMassAttr(Gf.Vec3f(*com))
    m.CreateDiagonalInertiaAttr(Gf.Vec3f(*[max(float(v), 1e-6) for v in inertia]))
    m.CreatePrincipalAxesAttr(Gf.Quatf(1, 0, 0, 0))
    return p


def _box_inertia(mass, sx, sy, sz):
    return (mass * (sy * sy + sz * sz) / 12, mass * (sx * sx + sz * sz) / 12, mass * (sx * sx + sy * sy) / 12)


def _articulation(st, path):
    p = UsdGeom.Xform.Define(st, path).GetPrim()
    UsdPhysics.ArticulationRootAPI.Apply(p)
    # PhysX settings as raw attributes (usd-core has no PhysxSchema): no self-collision (the leaves of a pair
    # overlap 10 mm at the meeting stiles), never sleep (the controller's efforts must always act).
    p.AddAppliedSchema("PhysxArticulationAPI")
    p.CreateAttribute("physxArticulation:enabledSelfCollisions", Sdf.ValueTypeNames.Bool).Set(False)
    p.CreateAttribute("physxArticulation:sleepThreshold", Sdf.ValueTypeNames.Float).Set(0.0)
    p.CreateAttribute("physxArticulation:stabilizationThreshold", Sdf.ValueTypeNames.Float).Set(0.0)
    p.CreateAttribute("physxArticulation:solverPositionIterationCount", Sdf.ValueTypeNames.Int).Set(8)
    p.CreateAttribute("physxArticulation:solverVelocityIterationCount", Sdf.ValueTypeNames.Int).Set(1)
    return p


def _fix_to_world(st, art_path, base_path, base_world):
    j = UsdPhysics.FixedJoint.Define(st, f"{art_path}/RootJoint")
    j.CreateBody1Rel().SetTargets([Sdf.Path(base_path)])
    j.CreateLocalPos0Attr(Gf.Vec3f(*base_world))  # body0 empty = the world frame
    j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
    j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))


def _joint_state(prim, kind, value):
    """Initial joint position (PhysxSchema JointStateAPI, angular in degrees, linear in metres)."""
    prim.AddAppliedSchema(f"PhysicsJointStateAPI:{kind}")
    prim.CreateAttribute(f"state:{kind}:physics:position", Sdf.ValueTypeNames.Float).Set(float(value))
    prim.CreateAttribute(f"state:{kind}:physics:velocity", Sdf.ValueTypeNames.Float).Set(0.0)


_AX = {(1, 0, 0): ("X", None), (-1, 0, 0): ("X", 180), (0, 1, 0): ("Y", None), (0, -1, 0): ("Y", 180),
       (0, 0, 1): ("Z", None), (0, 0, -1): ("Z", 180)}


def _prismatic(st, path, base_path, body_path, pos0, axis, travel, joint_name):
    """Slide along a signed world axis: the joint frames turn 180 deg about z (x/y) or x (z) for a minus sign."""
    j = UsdPhysics.PrismaticJoint.Define(st, path)
    name, flip = _AX[tuple(int(round(v)) for v in axis)]
    rot = Gf.Quatf(1, 0, 0, 0)
    if flip:
        rot = Gf.Quatf(0, 1, 0, 0) if name == "Z" else Gf.Quatf(0, 0, 0, 1)
    j.CreateAxisAttr(name)
    j.CreateBody0Rel().SetTargets([Sdf.Path(base_path)])
    j.CreateBody1Rel().SetTargets([Sdf.Path(body_path)])
    j.CreateLocalPos0Attr(Gf.Vec3f(*pos0))
    j.CreateLocalRot0Attr(rot)
    j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    j.CreateLocalRot1Attr(rot)
    j.CreateLowerLimitAttr(0.0)
    j.CreateUpperLimitAttr(float(travel))
    j.GetPrim().SetCustomDataByKey("hg1:joint", joint_name)
    _joint_state(j.GetPrim(), "linear", 0.0)
    return j


# ---------------------------------------------------------------- doors


def _leaf_visual_z(d, lf):
    """Leaf visual bottom and top (m): the spec leaf (z 0.01 .. 0.01 + height), or the measured frame hole of a door
    in an asset frame (BUILDING_FRAMES), so the leaf fills the frame. The collider always keeps the spec size."""
    z0, z1 = 0.01, 0.01 + lf["height"]
    fr = BUILDING_FRAMES.get(d["name"]) if d.get("source_frame") == "world" else None
    if fr:
        z1 = max(z1, fr["head"] - LEAF_TOP_GAP)
        if fr["sill"] is not None:
            z0 = max(z0, fr["sill"] + LEAF_SILL_GAP)
    return z0, z1


def _door(st, d, q0, M, wing_origin):
    ap = f"{ROOT}/{d.get('group', 'Doors')}/{d['name']}"
    _articulation(st, ap)
    hs = [lf["hinge"] for lf in d["leaves"]]
    bx, by = sum(h[0] for h in hs) / len(hs), sum(h[1] for h in hs) / len(hs)
    _body(st, f"{ap}/Base", (bx, by, 0.0), 0.0, 1.0)
    _fix_to_world(st, ap, f"{ap}/Base", (bx + wing_origin[0], by + wing_origin[1], wing_origin[2]))
    sign = M["Sign_" + sign_kind(d)]
    for i, lf in enumerate(d["leaves"]):
        w, t, h = lf["width"], lf["thickness"], lf["height"]
        x0, x1, z0, z1 = 0.005, 0.005 + w, 0.01, 0.01 + h  # leaf frame: x from the hinge along the leaf, z up
        zv0, zv1 = _leaf_visual_z(d, lf)
        a0 = q0[lf["joint"]]
        lp = f"{ap}/Leaf{i}"
        _body(st, lp, (lf["hinge"][0], lf["hinge"][1], 0.0), lf["closed_yaw"] + a0, lf["mass"],
              com=((x0 + x1) / 2, 0.0, (z0 + z1) / 2), inertia=_box_inertia(lf["mass"], w, t, h))
        _box(st, f"{lp}/Collider", (x0, -t / 2, z0), (x1, t / 2, z1), collide=True, guide=True)
        # Leaf with a glazed vision panel (HTM 05-02 App C: down to 500 mm), kick plates, push/pull furniture.
        vi, vz0, vz1 = lf["vision_panel"]
        lam = M["Laminate"]
        _box(st, f"{lp}/BottomRail", (x0, -t / 2, zv0), (x1, t / 2, vz0), lam)
        _box(st, f"{lp}/TopRail", (x0, -t / 2, vz1), (x1, t / 2, zv1), lam)
        _box(st, f"{lp}/HingeStile", (x0, -t / 2, vz0), (x0 + vi, t / 2, vz1), lam)
        _box(st, f"{lp}/LockStile", (x1 - vi, -t / 2, vz0), (x1, t / 2, vz1), lam)
        _box(st, f"{lp}/VisionPanel", (x0 + vi, -0.005, vz0), (x1 - vi, 0.005, vz1), M["Glass"])
        lo, hi = lf["range"]
        # A leaf turning +angle moves towards its local +y: that face is the pull side.
        pull = None if (lo < 0 < hi) else (1.0 if hi > 0 else -1.0)
        exit_door = d["rating"] == "EXIT"
        for s, tag in ((1.0, "P"), (-1.0, "N")):
            f = s * t / 2
            _box(st, f"{lp}/KickPlate{tag}", (x0 + 0.02, min(f, f + s * 0.0015), zv0),
                 (x1 - 0.01, max(f, f + s * 0.0015), zv0 + 0.30), M["Stainless"])
            if exit_door:
                # Final exit (BS EN 1125): a horizontal push bar on the inside face at 1.0 m, no hardware and no
                # fire-door sign outside. The green FIRE EXIT sign is over the doorway (_door_static).
                if s != pull:
                    _push_bar(st, f"{lp}/PushBar{tag}", x0, x1, f, s, M)
                continue
            # Blue fire-door sign on the top rail, both faces (text reads left to right from each side).
            sw, sh = SIGN_SIZE
            sc = ((x0 + x1) / 2, (vz1 + min(zv1, z1)) / 2)
            fo = f + s * 0.0012

            def P(u, v, sc=sc, fo=fo, s=s):
                return (sc[0] - s * u, fo, sc[1] + v)
            _quad(st, f"{lp}/Sign{tag}", P, sw, sh, (0.0, s, 0.0), sign)
            if pull is None or s != pull:  # push plate on every side that is pushed
                _box(st, f"{lp}/PushPlate{tag}", (x1 - 0.125, min(f, f + s * 0.002), 1.0),
                     (x1 - 0.025, max(f, f + s * 0.002), 1.3), M["Stainless"])
            else:  # pull handle on the side the leaf swings towards
                hx = x1 - 0.07
                _cyl(st, f"{lp}/PullHandle{tag}", (hx, f + s * 0.06, 1.05), "Z", 0.0125, 0.40, M["DarkSteel"])
                for k, z in enumerate((0.9, 1.2)):
                    _box(st, f"{lp}/PullStandoff{tag}{k}", (hx - 0.01, min(f, f + s * 0.06), z - 0.01),
                         (hx + 0.01, max(f, f + s * 0.06), z + 0.01), M["DarkSteel"])
        j = UsdPhysics.RevoluteJoint.Define(st, f"{ap}/fx_leaf{i}")
        j.CreateAxisAttr("Z")
        j.CreateBody0Rel().SetTargets([Sdf.Path(f"{ap}/Base")])
        j.CreateBody1Rel().SetTargets([Sdf.Path(lp)])
        j.CreateLocalPos0Attr(Gf.Vec3f(lf["hinge"][0] - bx, lf["hinge"][1] - by, 0.0))
        j.CreateLocalRot0Attr(_quat_z(lf["closed_yaw"]))  # joint angle 0 = closed
        j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLowerLimitAttr(math.degrees(lo))
        j.CreateUpperLimitAttr(math.degrees(hi))
        j.GetPrim().SetCustomDataByKey("hg1:joint", lf["joint"])
        _joint_state(j.GetPrim(), "angular", math.degrees(a0))


def _push_bar(st, path, x0, x1, f, s, M):
    """Push bar (panic bar) across a leaf face at z = 1.0 m: a 32 mm round bar on two standoffs, 70 mm proud."""
    a, b, z, off = x0 + 0.10, x1 - 0.07, 1.0, 0.07
    _cyl(st, f"{path}/Bar", ((a + b) / 2, f + s * off, z), "X", 0.016, b - a, M["Stainless"])
    for k, x in enumerate((a + 0.03, b - 0.03)):
        _box(st, f"{path}/Standoff{k}", (x - 0.02, min(f, f + s * off), z - 0.03),
             (x + 0.02, max(f, f + s * off), z + 0.03), M["DarkSteel"])


EXIT_SIGN_PNG = "../textures/signs/fire_exit.png"  # relative to usd/ (scene/sign_textures.py draws it)
EXIT_SIGN = (0.6, 0.2, 2.45, 0.115)  # width, height, centre z, distance of its face from the wall line (m)


def _exit_sign(st, path, center_xy, along, out, M, d=None, zc=None):
    """Green FIRE EXIT sign on the wall over a final exit: a thin back plate and a textured quad facing `out`.

    along: unit vector along the wall (the sign's +u), out: unit vector from the wall into the room it faces.
    d: distance of the back plate from the wall line (default EXIT_SIGN), zc: centre height (default EXIT_SIGN)."""
    w, h, zc0, d0 = EXIT_SIGN
    d = d0 if d is None else d
    zc = zc0 if zc is None else zc
    mat = UsdShade.Material.Define(st, f"{ROOT}/Looks/ExitSign")
    sh = UsdShade.Shader.Define(st, f"{ROOT}/Looks/ExitSign/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.5)
    tex = UsdShade.Shader.Define(st, f"{ROOT}/Looks/ExitSign/Texture")
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(EXIT_SIGN_PNG)
    tex.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
    rd = UsdShade.Shader.Define(st, f"{ROOT}/Looks/ExitSign/UV")
    rd.CreateIdAttr("UsdPrimvarReader_float2")
    rd.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(rd.ConnectableAPI(), "result")
    rgb = tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(rgb)
    # A maintained, internally lit sign (BS 5266-1): a little emission so it reads in any light.
    sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(rgb)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    cx, cy = center_xy
    ux, uy = along
    nx, ny = out

    def P(u, v, n):
        return Gf.Vec3f(cx + ux * u + nx * n, cy + uy * u + ny * n, zc + v)
    face = d + 0.012
    m = UsdGeom.Mesh.Define(st, f"{path}/Face")
    m.CreatePointsAttr([P(-w / 2, -h / 2, face), P(w / 2, -h / 2, face), P(w / 2, h / 2, face), P(-w / 2, h / 2, face)])
    m.CreateFaceVertexCountsAttr([4])
    m.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    m.CreateNormalsAttr([Gf.Vec3f(nx, ny, 0.0)] * 4)
    m.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    m.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    m.CreateExtentAttr([Gf.Vec3f(*[min(p[i] for p in m.GetPointsAttr().Get()) for i in range(3)]),
                        Gf.Vec3f(*[max(p[i] for p in m.GetPointsAttr().Get()) for i in range(3)])])
    UsdGeom.PrimvarsAPI(m).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex).Set(
        [Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)])
    _bind(m.GetPrim(), mat)
    # Back plate (housing) between the wall and the face.
    lo = [min(a, b) for a, b in zip(P(-w / 2 - 0.01, -h / 2 - 0.01, d), P(w / 2 + 0.01, h / 2 + 0.01, face - 0.001))]
    hi = [max(a, b) for a, b in zip(P(-w / 2 - 0.01, -h / 2 - 0.01, d), P(w / 2 + 0.01, h / 2 + 0.01, face - 0.001))]
    _box(st, f"{path}/Housing", lo, hi, M["Paint"])


def _covered_by_partition(d, partitions, z0, z1):
    """True when a static partition box (wing-local) holds the whole door line from z0 to z1: the screen's head
    piece over the opening already closes the space a transom would fill."""
    (ax, ay), (bx, by) = d["span"]
    for _n, lo, hi, _k in partitions:
        if lo[2] <= z0 + 0.02 and hi[2] >= z1 - 0.02 and all(
                lo[0] - 1e-3 <= x <= hi[0] + 1e-3 and lo[1] - 1e-3 <= y <= hi[1] + 1e-3 for x, y in ((ax, ay), (bx, by))):
            return True
    return False


def _door_static(st, d, M, partitions):
    """Fixed parts that belong to a doorset; returns the number of colliders built.

    - Transom (d["transom"]): a panel from the leaf head to d["ceiling"] on a steel head, a collider. Skipped when a
      static partition already fills that space (the 0.10 m screens of the main building have a head piece from
      z 2.12 over each opening).
    - Head stop (visual): wing kit frames have a 1.83 m x 2.20 m hole; the spec leaves span 2.0 m x 2.11 m, so their
      hinge edges sit in the frame jambs and a head stop closes the 90 mm above them. Not needed in the main
      building: screen openings have their head at 2.12 m and the leaves in asset frames fill the frame hole.
    - FIRE EXIT sign (final exits), over the hole on the inside face.
    Fixed side panels of a doorset (d["side_panels"]) are static partitions: they are built with the screens, not here.
    """
    sp = f"{ROOT}/Static/Doors/{d['name']}"
    UsdGeom.Scope.Define(st, sp)
    (ax, ay), (bx, by) = d["span"]
    top = d["leaves"][0]["height"] + 0.01
    if abs(ay - by) < 1e-9:  # door line along x
        lo, hi = (min(ax, bx), ay - 0.03), (max(ax, bx), ay + 0.03)
    else:
        lo, hi = (ax - 0.03, min(ay, by)), (ax + 0.03, max(ay, by))
    building = d.get("source_frame") == "world"
    ceiling = d.get("ceiling", 3.6)
    n = 0
    built = {"transom": False, "transom_skipped": False}
    if d.get("transom") and _covered_by_partition(d, partitions, top, ceiling):
        built["transom_skipped"] = True
    elif d.get("transom"):
        # Free-standing doorset: a full-height transom (smoke-tight to the ceiling) on a steel head.
        tlo = (lo[0], lo[1] - 0.02) if abs(ay - by) < 1e-9 else (lo[0] - 0.02, lo[1])
        thi = (hi[0], hi[1] + 0.02) if abs(ay - by) < 1e-9 else (hi[0] + 0.02, hi[1])
        _box(st, f"{sp}/Transom", (tlo[0], tlo[1], top + 0.005), (thi[0], thi[1], ceiling), M["Paint"], collide=True)
        hlo = (lo[0], lo[1] - 0.025) if abs(ay - by) < 1e-9 else (lo[0] - 0.025, lo[1])
        hhi = (hi[0], hi[1] + 0.025) if abs(ay - by) < 1e-9 else (hi[0] + 0.025, hi[1])
        _box(st, f"{sp}/Head", (hlo[0], hlo[1], top + 0.005), (hhi[0], hhi[1], top + 0.06), M["DarkSteel"])
        built["transom"] = True
        n += 1
    elif not building:
        _box(st, f"{sp}/HeadStop", (lo[0], lo[1], top + 0.005), (hi[0], hi[1], 2.21), M["Paint"])
    if d["rating"] == "EXIT":
        # FIRE EXIT sign over the doorway on the inside face: the side away from the swing (escape is outwards).
        sw = {"+x": (1, 0), "-x": (-1, 0), "+y": (0, 1), "-y": (0, -1)}[d["swing"]]
        out = (-sw[0], -sw[1])
        along = (1, 0) if abs(ay - by) < 1e-9 else (0, 1)
        if along[0] * out[1] - along[1] * out[0] > 0:  # +u must be the viewer's right, seen from the `out` side
            along = (-along[0], -along[1])
        fr = BUILDING_FRAMES.get(d["name"]) if building else None
        kw = {}
        if fr:  # over the measured hole, on the building's thinner wall
            kw = {"d": fr["wall_t"] + 0.003, "zc": max(EXIT_SIGN[2], fr["head"] + 0.03 + EXIT_SIGN[1] / 2)}
        _exit_sign(st, f"{sp}/ExitSign", ((ax + bx) / 2, (ay + by) / 2), along, out, M, **kw)
    return n, built


# ---------------------------------------------------------------- lifts


BTN_PLATE = 0.004   # faceplate thickness in front of the wall
BTN_CAP = 0.012     # cap depth (proud of the faceplate when out)


def _button(st, ap, base_path, base_xyz, b, name, M, size, mass, travel, joint_prim):
    """One push button: a prismatic body whose origin is on the wall face at `pos` (q = 0, out)."""
    ax = b["axis"]
    pos = b["pos"]
    bp = f"{ap}/{name}"
    off = [-a * (BTN_PLATE + BTN_CAP / 2) for a in ax]  # cap centre, in front of the faceplate
    _body(st, bp, tuple(pos), 0.0, mass, com=tuple(off),
          inertia=_box_inertia(mass, *[(BTN_CAP if abs(a) > 0.5 else size) for a in ax]))
    dims = [BTN_CAP if abs(a) > 0.5 else size * 0.9 for a in ax]
    _cube(st, f"{bp}/Collider", off, dims, collide=True, guide=True)
    axis_name = "XYZ"[[abs(a) > 0.5 for a in ax].index(True)]
    _cyl(st, f"{bp}/Cap", off, axis_name, size * 0.44, BTN_CAP, M["ButtonCap"])
    _prismatic(st, f"{ap}/{joint_prim}", base_path, bp,
               [p - q for p, q in zip(pos, base_xyz)], ax, travel, b["joint"])
    # lit ring and faceplate (static, visual only)
    sp = f"{ROOT}/Static/Buttons/{b['joint']}"
    ring_c = [p - a * (BTN_PLATE + 0.0005) for p, a in zip(pos, ax)]
    _cyl(st, f"{sp}_Ring", ring_c, axis_name, size * 0.56, 0.001, M["ButtonRing"])


def _faceplate(st, path, pos, axis, half_w, z0, z1, M):
    """Stainless plate on a wall face (`pos` on the face, `axis` into the wall)."""
    ax = axis
    if abs(ax[0]) > 0.5:
        lo = (pos[0] - ax[0] * BTN_PLATE if ax[0] > 0 else pos[0], pos[1] - half_w, z0)
        hi = (pos[0] if ax[0] > 0 else pos[0] + BTN_PLATE, pos[1] + half_w, z1)
    else:
        lo = (pos[0] - half_w, pos[1] - ax[1] * BTN_PLATE if ax[1] > 0 else pos[1], z0)
        hi = (pos[0] + half_w, pos[1] if ax[1] > 0 else pos[1] + BTN_PLATE, z1)
    _box(st, path, lo, hi, M["Stainless"])


def _lift(st, l, spec, M, wing_origin):
    ap = f"{ROOT}/Lifts/{l['name']}"
    _articulation(st, ap)
    base = (l["front_x"], l["center_y"], 0.0)
    _body(st, f"{ap}/Base", base, 0.0, 1.0)
    _fix_to_world(st, ap, f"{ap}/Base", tuple(b + o for b, o in zip(base, wing_origin)))
    for k, p in zip(("PanelA", "PanelB"), l["panels"]):
        bp = f"{ap}/{k}"
        _body(st, bp, tuple(p["closed_center"]), 0.0, p["mass"], inertia=_box_inertia(p["mass"], *p["size"]))
        _cube(st, f"{bp}/Panel", (0, 0, 0), p["size"], M["Stainless"], collide=True)
        # a dark vertical edge strip on the leading edge (reads as the meeting line when closed)
        sx, sy, sz = p["size"]
        lead = -p["axis"][1] * sy / 2  # leading edge is opposite to the opening direction
        _box(st, f"{bp}/Edge", (-sx / 2 - 0.001, lead - 0.004 if lead > 0 else lead, -sz / 2),
             (sx / 2 + 0.001, lead if lead > 0 else lead + 0.004, sz / 2), M["DarkSteel"])
        _prismatic(st, f"{ap}/fx_door_{k[-1].lower()}", f"{ap}/Base", bp,
                   [c - b for c, b in zip(p["closed_center"], base)], p["axis"], p["travel"], p["joint"])
    B = spec["button"]
    for b in l["buttons"]:
        kind = b["kind"].capitalize()
        _button(st, ap, f"{ap}/Base", base, b, "Btn" + kind, M, B["size"], B["mass"], B["travel"], f"fx_{kind}")
    # car operating panel (static plate behind the two buttons)
    b0 = l["buttons"][0]
    zs = [b["pos"][2] for b in l["buttons"]]
    _faceplate(st, f"{ROOT}/Static/Buttons/{l['name']}_COP", b0["pos"], b0["axis"], 0.09,
               min(zs) - 0.12, max(zs) + 0.12, M)


def _call(st, c, spec, M, wing_origin):
    ap = f"{ROOT}/Calls/{c['joint'][3:]}"
    _articulation(st, ap)
    base = tuple(c["pos"][:2]) + (0.0,)
    _body(st, f"{ap}/Base", base, 0.0, 1.0)
    _fix_to_world(st, ap, f"{ap}/Base", tuple(b + o for b, o in zip(base, wing_origin)))
    B = spec["button"]
    _button(st, ap, f"{ap}/Base", base, c, "Btn", M, B["size"], B["mass"], B["travel"], "fx_button")
    _faceplate(st, f"{ROOT}/Static/Buttons/{c['joint']}_Plate", c["pos"], c["axis"], 0.06,
               c["pos"][2] - 0.10, c["pos"][2] + 0.10, M)


STATIC_MAT = {"front": "LiftFront", "header": "LiftFront", "car_floor": "CarFloor", "car_roof": "CarRoof",
              "sill": "DarkSteel"}


def _lift_static(st, spec, M, car_light_intensity):
    """Static boxes of every bank (spec["static_boxes"]: fronts, headers, car shells, sills, all colliders), a
    bulkhead over each bank front whose ceiling is above the 3.0 m front top (only the wing: 3.6 m), and per lift the
    visual architrave, position indicator and car light. Returns the number of colliders built."""
    sp = f"{ROOT}/Static/Lifts"
    UsdGeom.Scope.Define(st, sp)
    n = 0
    for name, lo, hi in spec["static_boxes"]:
        key = next((k for k in STATIC_MAT if k in name), None)
        _box(st, f"{sp}/{name}", lo, hi, M[STATIC_MAT[key]] if key else M["Stainless"], collide=True)
        n += 1
    lifts = {l["name"]: l for l in spec["lifts"]}
    for bank in spec.get("lift_banks", []):
        fx, y_lo, y_hi = bank["front"]
        if bank["ceiling_z"] > 3.0:
            # Bulkhead from the top of the lift front (3.0 m) to the ceiling (wing: 3.6 m incl. the ceiling slab).
            _box(st, f"{sp}/fx_{bank['prefix']}bulkhead", (fx, y_lo, 3.0), (fx + 0.04, y_hi, bank["ceiling_z"]),
                 M["Paint"], collide=True)
            n += 1
    for l in lifts.values():
        fx, cy, hw, h = l["front_x"], l["center_y"], l["door_w"] / 2, l["door_h"]
        nm = l["name"]
        # stainless architrave on the lobby face, visual only
        _box(st, f"{sp}/{nm}_arch_s", (fx - 0.015, cy - hw - 0.08, 0.0), (fx, cy - hw, h + 0.08), M["Stainless"])
        _box(st, f"{sp}/{nm}_arch_n", (fx - 0.015, cy + hw, 0.0), (fx, cy + hw + 0.08, h + 0.08), M["Stainless"])
        _box(st, f"{sp}/{nm}_arch_head", (fx - 0.015, cy - hw - 0.08, h), (fx, cy + hw + 0.08, h + 0.08),
             M["Stainless"])
        # position indicator above the door (dark display with a lit arrow), visual only
        _box(st, f"{sp}/{nm}_indicator", (fx - 0.012, cy - 0.18, h + 0.18), (fx - 0.002, cy + 0.18, h + 0.32),
             M["Indicator"])
        _box(st, f"{sp}/{nm}_indicator_lit", (fx - 0.014, cy - 0.03, h + 0.21), (fx - 0.012, cy + 0.03, h + 0.29),
             M["IndicatorLit"])
        c = l["car"]
        light = UsdLux.RectLight.Define(st, f"{sp}/{nm}_car_light")
        light.AddTranslateOp().Set(Gf.Vec3d((c["x0"] + c["x1"]) / 2, (c["y0"] + c["y1"]) / 2, c["h"] - 0.02))
        light.GetWidthAttr().Set(1.6)
        light.GetHeightAttr().Set(1.0)
        light.GetIntensityAttr().Set(car_light_intensity)
        light.CreateEnableColorTemperatureAttr(True)
        light.CreateColorTemperatureAttr(5000.0)
    return n


def building_filter_prims(spec):
    """Asset prims (paths under /World/Hospital) that join the Static collision group: the frames and thresholds
    of the doors in the spec that sit in asset door frames (BUILDING_FRAMES[...]["filter"])."""
    out = []
    for d in spec["doors"]:
        fr = BUILDING_FRAMES.get(d["name"]) if d.get("source_frame") == "world" else None
        for n in (fr or {}).get("filter", []):
            p = f"{BUILDING_PRIM_ROOT}/{n}"
            if p not in out:
                out.append(p)
    return out


def _collision_groups(st, spec):
    """Moving parts never collide with the static parts: the spec places hinges and panels on wall lines (leaves sit
    in the jambs and screen openings, panels run against the front). The robot collides with everything.
    Static = /World/Fixtures/Static + the asset frames and thresholds of the building doors (building_filter_prims);
    g1-isaac-sim/make_stage.py adds its colliders (/World/HG1_Colliders: walls, floors, screens, junction boxes).
    Other building triangle colliders are not filtered: the building leaves keep 30 mm from the jambs and swing at
    most 92 deg (g1-isaac-sim/tools/fixtures_test.py checks the contact force on a full sweep)."""
    gp = f"{ROOT}/CollisionGroups"
    UsdGeom.Scope.Define(st, gp)
    moving = UsdPhysics.CollisionGroup.Define(st, f"{gp}/Moving")
    static = UsdPhysics.CollisionGroup.Define(st, f"{gp}/Static")
    inc = moving.GetCollidersCollectionAPI().CreateIncludesRel()
    for sub in GROUPS:
        inc.AddTarget(Sdf.Path(f"{ROOT}/{sub}"))
    sinc = static.GetCollidersCollectionAPI().CreateIncludesRel()
    sinc.AddTarget(Sdf.Path(f"{ROOT}/Static"))
    extra = building_filter_prims(spec)
    for p in extra:
        sinc.AddTarget(Sdf.Path(p))
    moving.CreateFilteredGroupsRel().AddTarget(static.GetPath())
    return extra


GROUPS = ("Doors", "Doors1", "Lifts", "Calls")  # = hospital_fixtures.GROUPS (tensor-view groups)


def author(st, layout, car_light_intensity=3000.0, tier=None):
    """Author /World/Fixtures into stage `st` from the layout module. Returns a summary dict for the report.

    tier: fixture tier for build_spec (None: $HG1_FIXTURES, default tier2). It is recorded in customData "hg1:tier"."""
    F = load_fixtures_module()
    spec = F.build_spec(layout, tier)
    q0 = F.FixtureController(spec).initial_q()
    wo = tuple(layout.WING_ORIGIN)
    x = UsdGeom.Xform.Define(st, ROOT)
    x.AddTranslateOp().Set(Gf.Vec3d(*wo))
    root = x.GetPrim()
    root.SetCustomDataByKey("hg1:tier", spec["tier"])
    root.SetCustomDataByKey("hg1:schema", spec.get("schema", 1))
    groups = spec.get("groups") or {"Doors": [d["name"] for d in spec["doors"]],
                                     "Lifts": [l["name"] for l in spec["lifts"]],
                                     "Calls": [c["joint"][3:] for c in spec["calls"]]}
    for sub in GROUPS:
        UsdGeom.Xform.Define(st, f"{ROOT}/{sub}")
    for sub in ("Static", "Static/Doors", "Static/Buttons"):
        UsdGeom.Scope.Define(st, f"{ROOT}/{sub}")
    M = _materials(st)
    ox, oy, oz = wo
    parts = []
    if hasattr(F, "static_partitions"):  # world -> wing-local, for the transom test
        parts = [(n, [lo[0] - ox, lo[1] - oy, lo[2] - oz], [hi[0] - ox, hi[1] - oy, hi[2] - oz], k)
                 for n, lo, hi, k in F.static_partitions(layout)]
    n_static, transoms, skipped = 0, [], []
    for d in spec["doors"]:
        _door(st, d, q0, M, wo)
        k, built = _door_static(st, d, M, parts)
        n_static += k
        if built["transom"]:
            transoms.append(d["name"])
        if built["transom_skipped"]:
            skipped.append(d["name"])
    for l in spec["lifts"]:
        _lift(st, l, spec, M, wo)
    for c in spec["calls"]:
        _call(st, c, spec, M, wo)
    n_static += _lift_static(st, spec, M, car_light_intensity)
    filtered = _collision_groups(st, spec)
    return {
        "tier": spec["tier"],
        "articulations": {g: list(groups.get(g, [])) for g in GROUPS},
        "joints": F.joint_names(spec),
        "static_colliders": n_static,
        "transoms": transoms,
        "transoms_in_screen_heads": skipped,
        "building_prims_filtered": filtered,
        "initial_q": {k: round(v, 4) for k, v in q0.items() if v},
    }


if __name__ == "__main__":
    import sys

    if "--signs" in sys.argv:
        draw_sign_textures()
    else:
        print(__doc__)
