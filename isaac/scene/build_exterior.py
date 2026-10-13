"""Exterior layer: the drive and the patient-transport bays at the main entrance, the A&E lane, apron, ambulance
bay AE1 and canopy at the courtyard (the ambulance entrance), a parked UK emergency ambulance, a wayfinding sign
and the paved path outside the wing's final exit. Visual only, no physics (the colliders are
layout.EXTERIOR_COLLIDERS, built by g1-isaac-sim/make_stage.py and g1-mujoco-sim/build_world.py).

    usd/exterior.usd   /World/Exterior/Ground (lawn, asphalt, footway, kerbs, A&E asphalt and kerbs, courtyard
                       paving, ExitPath, ExitLawn), /World/Exterior/Ambulance, /World/Exterior/Canopy (slab, fascia
                       with the A&E sign, posts, two soffit luminaires and rect lights), /World/Exterior/Signs

Called by scene/build_hospital.py (build() and check()). All positions come from layout.EXTERIOR (world frame).
The geometry is plain meshes with UVs. The textures are in textures/exterior/ (scene/exterior_textures.py).
The ambulance is built in its own frame (+x forward, +y left, z up, origin on the ground under the footprint centre)
and placed by layout.EXTERIOR["ambulance"]. It has about 1,500 triangles.
"""

import math
import os

from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdShade, Vt

import exterior_textures as T
import layout as L

TEX = "../textures/exterior"  # relative to usd/
E = L.EXTERIOR


# ---------------------------------------------------------------- mesh helpers


def _sub(a, b):
    return [a[i] - b[i] for i in range(3)]


def _cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def _unit(v):
    n = math.sqrt(sum(c * c for c in v)) or 1.0
    return [c / n for c in v]


def _newell(pts):
    n = [0.0, 0.0, 0.0]
    for i, p in enumerate(pts):
        q = pts[(i + 1) % len(pts)]
        n[0] += (p[1] - q[1]) * (p[2] + q[2])
        n[1] += (p[2] - q[2]) * (p[0] + q[0])
        n[2] += (p[0] - q[0]) * (p[1] + q[1])
    return _unit(n)


def earclip(poly):
    """Triangles (index triples) of a simple 2D polygon, any winding."""
    area = sum(poly[i][0] * poly[(i + 1) % len(poly)][1] - poly[(i + 1) % len(poly)][0] * poly[i][1]
               for i in range(len(poly)))
    idx = list(range(len(poly))) if area > 0 else list(range(len(poly)))[::-1]

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    tris = []
    while len(idx) > 3:
        for k in range(len(idx)):
            i0, i1, i2 = idx[k - 1], idx[k], idx[(k + 1) % len(idx)]
            a, b, c = poly[i0], poly[i1], poly[i2]
            if cross(a, b, c) <= 1e-12:
                continue
            if any(cross(a, b, poly[j]) >= 0 and cross(b, c, poly[j]) >= 0 and cross(c, a, poly[j]) >= 0
                   for j in idx if j not in (i0, i1, i2)):
                continue
            tris.append((i0, i1, i2))
            idx.pop(k)
            break
        else:
            raise ValueError("earclip: degenerate polygon")
    tris.append(tuple(idx))
    return tris


class Buf:
    """Mesh under construction: unshared vertices, face-varying UVs and normals."""

    def __init__(self):
        self.pts, self.counts, self.uv, self.nrm = [], [], [], []

    def poly(self, pts, uvs, normal=None, normals=None):
        n = _newell(pts)
        if normal is not None and sum(a * b for a, b in zip(n, normal)) < 0:
            pts, uvs = pts[::-1], uvs[::-1]
            normals = normals[::-1] if normals else None
            n = [-c for c in n]
        self.pts += [tuple(p) for p in pts]
        self.uv += [tuple(u) for u in uvs]
        self.nrm += [tuple(v) for v in normals] if normals else [tuple(n)] * len(pts)
        self.counts.append(len(pts))

    def concave(self, pts2d, to3d, uvfn, normal):
        for tri in earclip(pts2d):
            p = [to3d(pts2d[t]) for t in tri]
            self.poly(p, [uvfn(q) for q in p], normal)

    def box(self, lo, hi, uvfn, skip=()):
        x0, y0, z0 = lo
        x1, y1, z1 = hi
        faces = {"-x": [(x0, y0, z0), (x0, y1, z0), (x0, y1, z1), (x0, y0, z1)],
                 "+x": [(x1, y0, z0), (x1, y1, z0), (x1, y1, z1), (x1, y0, z1)],
                 "-y": [(x0, y0, z0), (x1, y0, z0), (x1, y0, z1), (x0, y0, z1)],
                 "+y": [(x0, y1, z0), (x1, y1, z0), (x1, y1, z1), (x0, y1, z1)],
                 "-z": [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0)],
                 "+z": [(x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]}
        axes = {"x": 0, "y": 1, "z": 2}
        for k, f in faces.items():
            if k in skip:
                continue
            nrm = [0.0, 0.0, 0.0]
            nrm[axes[k[1]]] = 1.0 if k[0] == "+" else -1.0
            self.poly(f, [uvfn(p, k) for p in f], nrm)

    def triangles(self):
        return sum(c - 2 for c in self.counts)

    def write(self, stage, path, material):
        m = UsdGeom.Mesh.Define(stage, path)
        m.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in self.pts]))
        m.CreateFaceVertexCountsAttr(self.counts)
        m.CreateFaceVertexIndicesAttr(list(range(len(self.pts))))
        m.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        lo = [min(p[i] for p in self.pts) for i in range(3)]
        hi = [max(p[i] for p in self.pts) for i in range(3)]
        m.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(*lo), Gf.Vec3f(*hi)]))
        m.CreateNormalsAttr(Vt.Vec3fArray([Gf.Vec3f(*n) for n in self.nrm]))
        m.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
        pv = UsdGeom.PrimvarsAPI(m).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray,
                                                  UsdGeom.Tokens.faceVarying)
        pv.Set(Vt.Vec2fArray([Gf.Vec2f(*u) for u in self.uv]))
        UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(material)
        return m


def material(stage, path, color=(0.5, 0.5, 0.5), tex=None, rough=0.6, metallic=0.0, emissive=None):
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, f"{path}/Surface")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
    if emissive:
        sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*emissive))
    if tex:
        st = UsdShade.Shader.Define(stage, f"{path}/St")
        st.CreateIdAttr("UsdPrimvarReader_float2")
        st.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
        tx = UsdShade.Shader.Define(stage, f"{path}/Tex")
        tx.CreateIdAttr("UsdUVTexture")
        tx.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(f"{TEX}/{tex}")
        tx.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
        tx.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
        tx.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
        tx.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(st.ConnectableAPI(), "result")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tx.ConnectableAPI(), "rgb")
    else:
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


# ---------------------------------------------------------------- ground


def build_ground(st, root, mats):
    h = E["kerb_h"]
    fx0, fy0, fx1, fy1 = E["footway"]
    kw = E["kerb_w"]
    n = 0

    def tile(kind):
        t = T.TILE_M[kind]
        return lambda p, *_: (p[0] / t, p[1] / t)

    lawn = Buf()
    lawn.concave(E["lawn"], lambda q: (q[0], q[1], -0.01), tile("lawn"), (0, 0, 1))
    lawn.write(st, f"{root}/Lawn", mats["lawn"])
    # Asphalt: apron + access road as one L-shaped polygon, world-planar UVs on the marking texture.
    ax0, ay0, ax1, ay1 = E["apron"]
    rx0, ry0, rx1, ry1 = E["road"]
    outline = [(rx0, ry0), (rx1, ry0), (rx1, ay0), (ax1, ay0), (ax1, ay1), (ax0, ay1)]
    asph = Buf()
    asph.concave(outline, lambda q: (q[0], q[1], 0.0),
                 lambda p: ((p[0] - T.ASPHALT_X0) / T.ASPHALT_SIZE, (p[1] - T.ASPHALT_Y0) / T.ASPHALT_SIZE),
                 (0, 0, 1))
    asph.write(st, f"{root}/Asphalt", mats["asphalt"])
    # Footway: raised part (paving box + kerb box), 1:12 ramp, flush part with flush kerb and tactile strip.
    rx_a, rx_b = E["footway_raised_x"]
    ra, rb = E["footway_ramp_x"]
    pave, pavef, kerb, flush, tact = Buf(), Buf(), Buf(), Buf(), Buf()
    pave.box((rx_a, fy0 + kw, -0.02), (rx_b, fy1, h), lambda p, f: (p[0] + p[2], p[1] + p[2]) if f[1] != "z" else
             tile("paving")(p), skip=("-z", "+y", "+x"))
    kerb.box((rx_a, fy0, -0.02), (rx_b, fy0 + kw, h), lambda p, f: (0, 0), skip=("-z", "+x"))
    for buf, (y0, y1) in ((pave, (fy0 + kw, fy1)), (kerb, (fy0, fy0 + kw))):
        top = [(ra, y0, h), (rb, y0, 0.0), (rb, y1, 0.0), (ra, y1, h)]
        buf.poly(top, [tile("paving")(p) for p in top], (0, 0, 1))
        side = [(ra, y0, h), (ra, y0, -0.02), (rb, y0, -0.02), (rb, y0, 0.0)]
        if y0 == fy0:
            buf.poly(side, [(0, 0)] * 4, (0, -1, 0))
    tact_y = fy0 + kw + 0.8
    for buf, (y0, y1), z, kind in ((flush, (fy0, fy0 + kw), 0.002, None), (tact, (fy0 + kw, tact_y), 0.002, "tactile"),
                                   (pavef, (tact_y, fy1), 0.0, "paving")):
        q = [(rb, y0, z), (fx1, y0, z), (fx1, y1, z), (rb, y1, z)]
        buf.poly(q, [tile(kind)(p) if kind else (0, 0) for p in q], (0, 0, 1))
    for k, (x0, y0, x1, y1) in E["kerbs"].items():
        if not k.startswith("AE"):
            kerb.box((x0, y0, -0.02), (x1, y1, h), lambda p, f: (0, 0), skip=("-z",))
    for name, buf, mat in (("Paving", pave, "paving"), ("PavingFlush", pavef, "paving"), ("Kerbs", kerb, "kerbs"),
                           ("FlushKerb", flush, "flushkerb"), ("Tactile", tact, "tactile")):
        buf.write(st, f"{root}/{name}", mats[mat])
    for buf in (lawn, asph, pave, pavef, kerb, flush, tact):
        n += buf.triangles()
    return n + build_ae_ground(st, root, mats, tile) + build_exit_path(st, root, mats, tile)


def build_ae_ground(st, root, mats, tile):
    """A&E lane and apron (one L-shaped asphalt polygon on asphalt_ae.jpg), their upstand kerbs, the paved
    courtyard and door lobby, and the level route from the apron into the courtyard: a flush (dropped) kerb on
    the apron edge and a strip of blister paving inside the courtyard mouth."""
    if "ae_lane" not in E:
        return 0
    h, n = E["kerb_h"], 0
    lx0, ly0, lx1, ly1 = E["ae_lane"]
    ax0, ay0, ax1, ay1 = E["ae_apron"]
    outline = [(lx0, ly0), (lx1, ly0), (lx1, ly1), (ax1, ay0), (ax1, ay1), (ax0, ay1)]
    asph = Buf()
    asph.concave(outline, lambda q: (q[0], q[1], 0.0),
                 lambda p: ((p[0] - T.AE_X0) / T.AE_W, (p[1] - T.AE_Y0) / T.AE_H), (0, 0, 1))
    kerb = Buf()
    for k, (x0, y0, x1, y1) in E["kerbs"].items():
        if k.startswith("AE"):
            kerb.box((x0, y0, -0.02), (x1, y1, h), lambda p, f: (0, 0), skip=("-z",))
    court = Buf()
    for x0, y0, x1, y1 in E.get("courtyard_floor", []):
        q = [(x0, y0, 0.0), (x1, y0, 0.0), (x1, y1, 0.0), (x0, y1, 0.0)]
        court.poly(q, [tile("paving")(p) for p in q], (0, 0, 1))
    flush, tact = Buf(), Buf()
    if E.get("courtyard_floor"):
        mx0, my0, mx1, _ = E["courtyard_floor"][0]   # the courtyard mouth: open to the apron along y = my0
        for buf, (y0, y1), kind in ((flush, (ay1 - 0.15, ay1), None), (tact, (my0, my0 + 0.8), "tactile")):
            q = [(mx0, y0, 0.002), (mx1, y0, 0.002), (mx1, y1, 0.002), (mx0, y1, 0.002)]
            buf.poly(q, [tile(kind)(p) if kind else (0, 0) for p in q], (0, 0, 1))
    for name, buf, mat in (("AEAsphalt", asph, "asphalt_ae"), ("AEKerbs", kerb, "kerbs"),
                           ("CourtyardPaving", court, "paving"), ("AEFlushKerb", flush, "flushkerb"),
                           ("AETactile", tact, "tactile")):
        if buf.counts:
            buf.write(st, f"{root}/{name}", mats[mat])
            n += buf.triangles()
    return n


def build_exit_path(st, root, mats, tile):
    """Paved path outside the final exit and lawn around it (layout.EXIT_PATH, wing-local -> world)."""
    P = getattr(L, "EXIT_PATH", None)
    if not P:
        return 0
    ox, oy, _ = L.WING_ORIGIN
    x0, y0, x1, y1 = P["paving"]
    pave = Buf()
    q = [(x0 + ox, y0 + oy, 0.0), (x1 + ox, y0 + oy, 0.0), (x1 + ox, y1 + oy, 0.0), (x0 + ox, y1 + oy, 0.0)]
    pave.poly(q, [tile("paving")(p) for p in q], (0, 0, 1))
    pave.write(st, f"{root}/ExitPath", mats["paving"])
    n = pave.triangles()
    for i, (x0, y0, x1, y1) in enumerate(P["lawn"]):
        lawn = Buf()
        q = [(x0 + ox, y0 + oy, -0.01), (x1 + ox, y0 + oy, -0.01), (x1 + ox, y1 + oy, -0.01), (x0 + ox, y1 + oy, -0.01)]
        lawn.poly(q, [tile("lawn")(p) for p in q], (0, 0, 1))
        lawn.write(st, f"{root}/ExitLawn{i}", mats["lawn"])
        n += lawn.triangles()
    return n


# ---------------------------------------------------------------- ambulance

# Side profiles (x, z) in the vehicle frame, counter-clockwise. Wheel arches are notches over the axles.
R_TYRE, R_ARCH = 0.36, 0.47
AXLE_F, AXLE_R = 2.55, -1.75
BOX_HW, CAB_HW = 1.15, 1.0
CAB = {"bumper": ((3.47, 0.45), (3.47, 0.95)), "grille": ((3.47, 0.95), (3.32, 1.20)),
       "bonnet": ((3.32, 1.20), (2.72, 1.38)), "screen": ((2.72, 1.38), (1.97, 2.40)), "roof": ((1.97, 2.40), (1.30, 2.50))}


def _arch(cx, base=0.45, segs=10):
    a0 = math.degrees(math.asin((base - R_TYRE) / R_ARCH))
    return [(cx + R_ARCH * math.cos(math.radians(a)), R_TYRE + R_ARCH * math.sin(math.radians(a)))
            for a in [180 - a0 - (180 - 2 * a0) * i / segs for i in range(segs + 1)]]


def _profiles():
    box = [(-3.40, 0.45)] + _arch(AXLE_R) + [(1.30, 0.45), (1.30, 2.90), (-3.40, 2.90)]
    cab = [(1.30, 0.45)] + _arch(AXLE_F) + [CAB["bumper"][0], CAB["grille"][0], CAB["bonnet"][0], CAB["screen"][0],
                                            CAB["roof"][0], CAB["roof"][1]]
    return box, cab


def _atlas(region, a, b):
    px0, py0, w, h = T.REGIONS[region]
    a, b = min(max(a, 0.5), w - 0.5), min(max(b, 0.5), h - 0.5)
    return ((px0 + a) / T.ATLAS_W, 1.0 - (py0 + b) / T.ATLAS_H)


def _swatch(name):
    px0, py0, w, h = T.REGIONS[f"sw_{name}"]
    return _atlas(f"sw_{name}", w / 2, h / 2)


def _extrude(buf, prof, hw, cap_uv, edge_uv):
    for sgn in (1, -1):
        buf.concave(prof, lambda q, s=sgn: (q[0], s * hw, q[1]), lambda p, s=sgn: cap_uv(s, p), (0, sgn, 0))
    for e in range(len(prof)):
        (x0, z0), (x1, z1) = prof[e], prof[(e + 1) % len(prof)]
        quad = [(x0, -hw, z0), (x1, -hw, z1), (x1, hw, z1), (x0, hw, z0)]
        out = _unit([z1 - z0, 0.0, -(x1 - x0)])
        buf.poly(quad, [edge_uv((x0, z0), (x1, z1), p) for p in quad], out)


def _wheel(tyre, hub, x, yc, width, side, segs=24):
    """Tyre (cylinder along y) and a hub disc on its outer face (side = +1 left, -1 right)."""
    y0, y1 = yc - width / 2, yc + width / 2
    ring = [(x + R_TYRE * math.cos(2 * math.pi * i / segs), R_TYRE + R_TYRE * math.sin(2 * math.pi * i / segs))
            for i in range(segs)]
    for i in range(segs):
        (a, b), (c, d) = ring[i], ring[(i + 1) % segs]
        na = _unit([a - x, 0, b - R_TYRE])
        nc = _unit([c - x, 0, d - R_TYRE])
        tyre.poly([(a, y0, b), (c, y0, d), (c, y1, d), (a, y1, b)], [(0, 0)] * 4, na, [na, nc, nc, na])
    for y, s in ((y0, -1), (y1, 1)):
        tyre.poly([(a, y, b) for a, b in ring], [(0, 0)] * segs, (0, s, 0))
    yo = (y1 if side > 0 else y0) + side * 0.004
    disc = [(x + 0.21 * math.cos(2 * math.pi * i / 16), yo, R_TYRE + 0.21 * math.sin(2 * math.pi * i / 16))
            for i in range(16)]
    hub.poly(disc, [(0, 0)] * 16, (0, side, 0))


def build_ambulance(st, root, mats):
    box, cab = _profiles()
    liv, glass, trim, tyre, hub, blue = Buf(), Buf(), Buf(), Buf(), Buf(), Buf()
    P = T.PPM

    # Box body: sides, rear and front from the atlas; roof yellow; underside and arch liners dark.
    def box_cap(s, p):
        x, _, z = p
        return _atlas("side_left", (1.30 - x) * P, (2.90 - z) * P) if s > 0 else \
            _atlas("side_right", (x + 3.40) * P, (2.90 - z) * P)

    def box_edge(a, b, p):
        x, y, z = p
        if a[0] == b[0] == 1.30:
            return _atlas("box_front", (y + 1.15) * P, (2.90 - z) * P)
        if a[0] == b[0] == -3.40:
            return _atlas("rear", (1.15 - y) * P, (2.90 - z) * P)
        return _swatch("yellow" if a[1] == b[1] == 2.90 else "dark")
    _extrude(liv, box, BOX_HW, box_cap, box_edge)

    # Cab: sides share one atlas region; front faces map onto the bumper, grille and bonnet regions.
    def cab_cap(s, p):
        return _atlas("cab_side", (3.45 - p[0]) * P, (2.50 - p[2]) * P)

    def cab_edge(a, b, p):
        x, y, z = p
        for key, region in (("bumper", "bumper"), ("grille", "grille"), ("bonnet", "bonnet")):
            (ax, az), (bx, bz) = CAB[key]
            if (a, b) == ((ax, az), (bx, bz)):
                h = T.REGIONS[region][3]
                f = (z - az) / (bz - az)
                return _atlas(region, (y + 1.0) * P, (1.0 - f) * h)
        if (a, b) == CAB["screen"]:
            return _swatch("black")
        if (a, b) == CAB["roof"] or a[0] == b[0] == 1.30:
            return _swatch("yellow")
        return _swatch("dark")
    _extrude(liv, cab, CAB_HW, cab_cap, cab_edge)

    # Glass: windscreen (just proud of the cab) and cab door windows.
    (ax, az), (bx, bz) = CAB["screen"]
    ln = math.hypot(bx - ax, bz - az)
    d, nrm = ((bx - ax) / ln, (bz - az) / ln), ((bz - az) / ln, -(bx - ax) / ln)
    nrm = (-nrm[0], -nrm[1]) if nrm[0] < 0 else nrm
    e, o = 0.05, 0.006
    p0 = (ax + d[0] * e + nrm[0] * o, az + d[1] * e + nrm[1] * o)
    p1 = (bx - d[0] * e + nrm[0] * o, bz - d[1] * e + nrm[1] * o)
    glass.poly([(p0[0], -0.92, p0[1]), (p0[0], 0.92, p0[1]), (p1[0], 0.92, p1[1]), (p1[0], -0.92, p1[1])],
               [(0, 0)] * 4, (nrm[0], 0, nrm[1]))
    win = [(1.45, 1.45), (2.55, 1.45), (2.00, 2.20), (1.45, 2.20)]
    for s in (1, -1):
        glass.poly([(x, s * (CAB_HW + 0.006), z) for x, z in win], [(0, 0)] * 4, (0, s, 0))

    # Trim: mirrors, rear step, chassis.
    def none(p, f):
        return (0, 0)
    for s in (1, -1):
        trim.box((2.50, min(s * 0.95, s * 1.10), 1.70), (2.56, max(s * 0.95, s * 1.10), 1.76), none)
        trim.box((2.46, min(s * 1.08, s * 1.30), 1.50), (2.58, max(s * 1.08, s * 1.30), 1.88), none)
    trim.box((-3.52, -1.05, 0.36), (-3.40, 1.05, 0.50), none)
    trim.box((-3.30, -0.60, 0.30), (3.30, 0.60, 0.46), none, skip=("+z",))

    # Wheels: single front tyres, twin rear tyres (one wide tyre).
    for s in (1, -1):
        _wheel(tyre, hub, AXLE_F, s * 0.86, 0.24, s)
        _wheel(tyre, hub, AXLE_R, s * 0.77, 0.42, s)

    # Blue warning lights: cab roof light bar and the four top corners of the box body.
    blue.box((1.42, -0.72, 2.42), (1.74, 0.72, 2.58), none, skip=("-z",))
    for x0, x1 in ((1.18, 1.32), (-3.42, -3.28)):
        for s in (1, -1):
            blue.box((x0, min(s * 0.98, s * 1.16), 2.74), (x1, max(s * 0.98, s * 1.16), 2.90), none, skip=("-z",))

    amb = E["ambulance"]
    xf = UsdGeom.Xform.Define(st, root)
    xf.AddTranslateOp().Set(Gf.Vec3d(amb["pos"][0], amb["pos"][1], 0.0))
    xf.AddRotateZOp().Set(float(amb["yaw"]))
    n = 0
    for name, buf in (("Body", liv), ("Glass", glass), ("Trim", trim), ("Tyres", tyre), ("Hubs", hub),
                      ("BlueLights", blue)):
        buf.write(st, f"{root}/{name}", mats[name.lower()])
        n += buf.triangles()
    return n


# ---------------------------------------------------------------- A&E canopy and wayfinding sign

# Soffit lights: the hospital's per-area level (build_hospital.KIT_LUX_PER_M2 x LIGHT_LEVEL, 975 per m2 of floor
# at a 2.95 m ceiling), scaled by (4.0 / 2.95)^2 for the higher mounting.
CANOPY_LUX_PER_M2 = 130000 / 4.0 * 0.25 * 0.12 * (4.0 / 2.95) ** 2
LIGHT_KELVIN = 5000.0
FASCIA_H, FASCIA_T = 0.55, 0.10


def _sign_uv(region):
    """UV function of a sign panel in the vertical plane: maps (u, v) in 0..1 (left->right, bottom->top) into the
    atlas region of textures/exterior/signs.png."""
    px0, py0, w, h = T.SIGN_REGIONS[region]

    def f(u, v):
        return ((px0 + 1 + u * (w - 2)) / T.SIGN_W, 1.0 - (py0 + 1 + (1 - v) * (h - 2)) / T.SIGN_H)
    return f


def _panel_south(buf, region, x0, x1, y, z0, z1):
    """Sign face in the plane y, read from the south (normal -y)."""
    f = _sign_uv(region)
    q = [(x0, y, z0), (x1, y, z0), (x1, y, z1), (x0, y, z1)]
    buf.poly(q, [f(0, 0), f(1, 0), f(1, 1), f(0, 1)], (0, -1, 0))


def build_canopy(st, root, mats):
    """Canopy over the courtyard mouth and the back of the ambulance bay: slab, fascia upstand with the A&E sign,
    two posts (hi-vis yellow foot), two soffit luminaires with rect lights (HBN 15-01 17.3: lit)."""
    can = E.get("canopy")
    if not can:
        return 0, 0
    x0, y0, x1, y1 = can["rect"]
    z0, z1 = can["z"]
    none = (lambda p, f: (0, 0))
    UsdGeom.Xform.Define(st, root)
    slab, fascia, post, foot, lum, sign = Buf(), Buf(), Buf(), Buf(), Buf(), Buf()
    slab.box((x0, y0, z0), (x1, y1, z1), none)
    t, fz = FASCIA_T, z0 + FASCIA_H
    fascia.box((x0 - t, y0 - t, z0), (x1 + t, y0, fz), none)               # south (front)
    fascia.box((x0 - t, y0, z0), (x0, y1, fz), none)                       # west
    fascia.box((x1, y0, z0), (x1 + t, y1, fz), none)                       # east
    sw = min(4.0, x1 - x0 - 0.6)
    cx = (x0 + x1) / 2
    _panel_south(sign, "canopy", cx - sw / 2, cx + sw / 2, y0 - t - 0.01, z0 + 0.03, z0 + 0.03 + sw / 8)
    s, band = can["post_size"], 1.2
    for px, py in can["posts"]:
        foot.box((px - s / 2, py - s / 2, 0.0), (px + s / 2, py + s / 2, band), none, skip=("-z",))
        post.box((px - s / 2, py - s / 2, band), (px + s / 2, py + s / 2, z0), none, skip=("-z", "+z"))
    lw, ld = can.get("light_size", (1.2, 0.4))
    n_lights = 0
    cover = (x1 - x0) * (y1 - y0) / max(1, len(can.get("lights", [])))
    for i, (lx, ly) in enumerate(can.get("lights", [])):
        lum.box((lx - lw / 2, ly - ld / 2, z0 - 0.05), (lx + lw / 2, ly + ld / 2, z0), none, skip=("+z",))
        light = UsdLux.RectLight.Define(st, f"{root}/Light{i}")
        light.AddTranslateOp().Set(Gf.Vec3d(lx, ly, z0 - 0.06))   # rect lights emit along -Z: straight down
        light.GetWidthAttr().Set(float(lw))
        light.GetHeightAttr().Set(float(ld))
        light.GetIntensityAttr().Set(CANOPY_LUX_PER_M2 * cover / (lw * ld))
        light.CreateEnableColorTemperatureAttr(True)
        light.CreateColorTemperatureAttr(LIGHT_KELVIN)
        n_lights += 1
    n = 0
    for name, buf, mat in (("Slab", slab, "canopy"), ("Fascia", fascia, "fascia"), ("Posts", post, "post"),
                           ("PostFeet", foot, "hivis"), ("Luminaires", lum, "luminaire"), ("Sign", sign, "signs")):
        if buf.counts:
            buf.write(st, f"{root}/{name}", mats[mat])
            n += buf.triangles()
    return n, n_lights


def build_signs(st, root, mats):
    """Free-standing wayfinding signs (layout.EXTERIOR["signs"]): two posts and stacked panels, read from the
    south. The back and the edges are plain grey."""
    n = 0
    for name, sg in E.get("signs", {}).items():
        (sx, sy), w, ph, pz, p, dx = sg["pos"], sg["width"], sg["panel_h"], sg["z0"], sg["post"], sg["post_dx"]
        top = pz + len(sg["panels"]) * ph
        none = (lambda q, f: (0, 0))
        posts, face, back = Buf(), Buf(), Buf()
        for d in (-dx, dx):   # same boxes as the colliders (layout._exterior_colliders)
            posts.box((sx + d - p / 2, sy - p / 2, 0.0), (sx + d + p / 2, sy + p / 2, top), none, skip=("-z",))
        th, yf = 0.03, sy - p / 2   # panel thickness; the panel is fixed to the south face of the posts
        back.box((sx - w / 2, yf - th, pz), (sx + w / 2, yf, top), none, skip=("-y",))
        for k, region in enumerate(sg["panels"]):   # first panel on top
            za = top - (k + 1) * ph
            _panel_south(face, region, sx - w / 2, sx + w / 2, yf - th, za, za + ph)
        for part, buf, mat in (("Posts", posts, "post"), ("Back", back, "signback"), ("Face", face, "signs")):
            buf.write(st, f"{root}/{name}/{part}", mats[mat])
            n += buf.triangles()
    return n


# ---------------------------------------------------------------- layer + checks


def build(new_stage):
    """Write usd/exterior.usd. new_stage is build_hospital.new_stage (clears or creates the layer)."""
    missing = [f for f in ("asphalt.jpg", "asphalt_ae.jpg", "paving.png", "tactile.png", "lawn.png",
                           "ambulance_livery.png", "signs.png")
               if not os.path.exists(os.path.join(T.TEX_DIR, f))]
    if missing:
        raise SystemExit(f"missing textures {missing}: run scene/exterior_textures.py (see README)")
    st = new_stage("exterior.usd")
    st.DefinePrim("/World/Exterior", "Xform")
    m = "/World/Exterior/Looks"
    mats = {
        "lawn": material(st, f"{m}/Lawn", tex="lawn.png", rough=0.95),
        "asphalt": material(st, f"{m}/Asphalt", tex="asphalt.jpg", rough=0.85),
        "asphalt_ae": material(st, f"{m}/AsphaltAE", tex="asphalt_ae.jpg", rough=0.85),
        "paving": material(st, f"{m}/Paving", tex="paving.png", rough=0.8),
        "kerbs": material(st, f"{m}/Kerb", color=(0.55, 0.55, 0.53), rough=0.8),
        "flushkerb": material(st, f"{m}/FlushKerb", color=(0.33, 0.33, 0.32), rough=0.8),
        "tactile": material(st, f"{m}/Tactile", tex="tactile.png", rough=0.8),
        "body": material(st, f"{m}/Livery", tex="ambulance_livery.png", rough=0.35),
        "glass": material(st, f"{m}/Glass", color=(0.02, 0.025, 0.03), rough=0.05),
        "trim": material(st, f"{m}/Trim", color=(0.02, 0.02, 0.02), rough=0.5),
        "tyres": material(st, f"{m}/Tyre", color=(0.025, 0.025, 0.025), rough=0.9),
        "hubs": material(st, f"{m}/Hub", color=(0.62, 0.62, 0.64), rough=0.3, metallic=1.0),
        "bluelights": material(st, f"{m}/BlueLight", color=(0.0, 0.03, 0.35), rough=0.2, emissive=(0.0, 0.06, 0.5)),
        "canopy": material(st, f"{m}/Canopy", color=(0.78, 0.78, 0.76), rough=0.7),
        "fascia": material(st, f"{m}/Fascia", color=(0.30, 0.32, 0.34), rough=0.5, metallic=0.3),
        "post": material(st, f"{m}/Post", color=(0.32, 0.33, 0.35), rough=0.4, metallic=0.6),
        "hivis": material(st, f"{m}/HiVis", color=(0.85, 0.72, 0.02), rough=0.5),
        "luminaire": material(st, f"{m}/Luminaire", color=(0.95, 0.95, 0.92), rough=0.3, emissive=(1.0, 1.0, 0.95)),
        "signs": material(st, f"{m}/Sign", tex="signs.png", rough=0.4),
        "signback": material(st, f"{m}/SignBack", color=(0.42, 0.43, 0.44), rough=0.5, metallic=0.4),
    }
    n = build_ground(st, "/World/Exterior/Ground", mats)
    n += build_ambulance(st, "/World/Exterior/Ambulance", mats)
    nc, _ = build_canopy(st, "/World/Exterior/Canopy", mats)
    n += nc + build_signs(st, "/World/Exterior/Signs", mats)
    st.GetRootLayer().documentation = ("Hospital_G1 exterior: drive and patient-transport bays at the main entrance, "
                                       "A&E lane, apron, ambulance bay and canopy, UK ambulance. Visual only.")
    st.GetRootLayer().Save()
    return n


def _overlap(a, b, tol=0.02):
    return all(min(a[i + 3], b[i + 3]) - max(a[i], b[i]) > tol for i in range(3))


def check(stage):
    """Exterior part of build_report.json: {"issues": [...], ...}. stage is the composed root."""
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])

    def bb(prim):
        r = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        return list(r.GetMin()) + list(r.GetMax())
    issues = []
    ext = stage.GetPrimAtPath("/World/Exterior")
    if not ext:
        return {"issues": ["exterior: /World/Exterior missing"]}
    parts = {str(p.GetPath()): bb(p) for p in Usd.PrimRange(ext) if p.IsA(UsdGeom.Mesh)}
    # 1. The ambulance stands on the ground inside its bay.
    a = bb(stage.GetPrimAtPath("/World/Exterior/Ambulance"))
    x0, y0, x1, y1 = E["bays"][E["ambulance"]["bay"]]
    if not (x0 <= a[0] and a[3] <= x1 and y0 <= a[1] and a[4] <= y1):
        issues.append(f"exterior: ambulance {[round(v, 2) for v in a]} outside bay {E['ambulance']['bay']}")
    if abs(a[2]) > 0.005:
        issues.append(f"exterior: ambulance lowest point at z {a[2]:.3f}, not on the ground")
    # 2. Nothing reaches into a wing room or the lift cars (the exit path lies outside, between them).
    ox, oy, _ = L.WING_ORIGIN
    rooms = dict(L.KIT_ROOMS)
    rooms.update(L.SUITE_ROOMS)
    cars = [(l["front_x"], l["center_y"] - L.LIFT_SPEC["car_w"] / 2 - 0.1,
             l["front_x"] + 0.2 + L.LIFT_SPEC["car_d"], l["center_y"] + L.LIFT_SPEC["car_w"] / 2 + 0.1)
            for l in L.LIFTS.values()]
    for rx0, ry0, rx1, ry1 in list(rooms.values()) + cars:
        w = (rx0 + ox, ry0 + oy, rx1 + ox, ry1 + oy)
        for p, b in parts.items():
            if b[0] < w[2] - 0.02 and b[3] > w[0] + 0.02 and b[1] < w[3] - 0.02 and b[4] > w[1] + 0.02:
                issues.append(f"exterior: {p} overlaps the wing room {[round(v, 2) for v in w]}")
    # 3. No exterior part intersects a hospital part (ground-level boxes, 2 cm tolerance).
    skip = ("GroundPlane", "Navmesh", "DomeLight")
    for h in stage.GetPrimAtPath("/World/Hospital").GetChildren():
        if h.GetName().startswith(skip):
            continue
        hb = bb(h)
        if hb[0] > hb[3]:
            continue
        for p, b in parts.items():
            if _overlap(b, hb):
                issues.append(f"exterior: {p} intersects /World/Hospital/{h.GetName()}")
    # 4. Every bay lies on asphalt, and the ambulance bay is next to the courtyard mouth.
    pads = [E[k] for k in ("apron", "road", "ae_apron", "ae_lane") if k in E]
    for name, (x0, y0, x1, y1) in E["bays"].items():
        if not any(p[0] <= x0 and x1 <= p[2] and p[1] <= y0 and y1 <= p[3] for p in pads):
            issues.append(f"exterior: bay {name} is not on the asphalt")
    # 5. Canopy: clears the ambulance (light bar and aerials, 0.5 m margin); the posts stand clear of the ambulance
    # bay, the lane and the unloading zone behind the bay.
    can = E.get("canopy")
    if can:
        if can["z"][0] < E["ambulance"]["size"][2] + 0.5:
            issues.append(f"exterior: canopy soffit {can['z'][0]} m is less than ambulance height + 0.5 m")
        bx0, by0, bx1, by1 = E["bays"][E["ambulance"]["bay"]]
        keep = [(bx0, by0, bx1, E["ae_apron"][3] if "ae_apron" in E else by1)] + ([E["ae_lane"]] if "ae_lane" in E else [])
        h = can["post_size"] / 2
        for i, (px, py) in enumerate(can["posts"]):
            for k in keep:
                if px + h > k[0] and px - h < k[2] and py + h > k[1] and py - h < k[3]:
                    issues.append(f"exterior: canopy post {i} at ({px}, {py}) stands in {k}")
    # 6. No collider box reaches more than 0.1 m below the ground.
    for name, (c, size, _) in L.EXTERIOR_COLLIDERS.items():
        if c[2] - size[2] / 2 < -0.11:
            issues.append(f"exterior: collider {name} reaches below the ground")
    tris = sum(c - 2 for p in Usd.PrimRange(ext) if p.IsA(UsdGeom.Mesh)
               for c in UsdGeom.Mesh(p).GetFaceVertexCountsAttr().Get())
    lights = [str(p.GetPath()) for p in Usd.PrimRange(ext) if p.HasAPI(UsdLux.LightAPI)]
    return {"issues": issues, "triangles": tris, "ambulance_box": [round(v, 3) for v in a], "lights": len(lights),
            "colliders": sorted(L.EXTERIOR_COLLIDERS)}
