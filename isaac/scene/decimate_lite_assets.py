"""Simplify the film-density i4h meshes inside the lite asset mirror so the scene fits an 8 GB GPU.

Each asset file gets a triangle budget (default 25k) shared across its meshes; meshes are simplified
with texture-preserving quadric edge collapse (UVs kept), and face-varying normals are rebuilt with a
crease angle. Files are unlinked from the originals before editing, so the original assets never change.
Meshes with GeomSubsets or other per-vertex/face-varying primvars are left alone and reported.

    uv run --no-project --with usd-core --with pymeshlab python scene/decimate_lite_assets.py [--budget 60000]
Run scene/make_lite_assets.py first.
"""

import argparse
import os
import shutil
import tempfile

import numpy as np
import pymeshlab
from pxr import Gf, Sdf, Usd, UsdGeom, Vt

LITE = os.path.expanduser("~/isaacsim_assets/lite/Assets/Isaac/Healthcare/0.7.0/724f82e/Props")
ROOTS = [f"{LITE}/shared_OR_without_Mark/Collected_surgery_room_movie_with_heart_adjusted/Assets", f"{LITE}/Lightwheel/Assets"]
MIN_KEEP = 2000  # meshes below this are never touched
CREASE_DEG = 40.0


def mesh_tris(m):
    fvc = m.GetFaceVertexCountsAttr().Get()
    return int(sum(c - 2 for c in fvc)) if fvc else 0


def triangulate(counts, indices):
    counts = np.asarray(counts)
    starts = np.concatenate([[0], np.cumsum(counts)[:-1]])
    tri_face, tri_corner = [], []
    for n in np.unique(counts):
        sel = np.nonzero(counts == n)[0]
        s = starts[sel]
        for k in range(1, n - 1):
            tri_face.append(sel)
            tri_corner.append(np.stack([s, s + k, s + k + 1], axis=1))
    order = np.argsort(np.concatenate(tri_face), kind="stable")
    corners = np.concatenate(tri_corner)[order]  # (T, 3) positions into the face-vertex list
    return np.asarray(indices)[corners], corners


def crease_normals(v, f):
    fn = np.cross(v[f[:, 1]] - v[f[:, 0]], v[f[:, 2]] - v[f[:, 0]])
    vn = np.zeros_like(v)
    for k in range(3):
        np.add.at(vn, f[:, k], fn)
    fn_u = fn / np.maximum(np.linalg.norm(fn, axis=1, keepdims=True), 1e-12)
    vn_u = vn / np.maximum(np.linalg.norm(vn, axis=1, keepdims=True), 1e-12)
    wedge = vn_u[f]  # (F, 3, 3)
    flat = np.einsum("fkc,fc->fk", wedge, fn_u) < np.cos(np.radians(CREASE_DEG))
    wedge[flat] = np.repeat(fn_u[:, None, :], 3, axis=1)[flat]
    return wedge.reshape(-1, 3)


def simplify(mesh, target):
    pts = np.asarray(mesh.GetPointsAttr().Get(), dtype=np.float64)
    counts, idx = mesh.GetFaceVertexCountsAttr().Get(), mesh.GetFaceVertexIndicesAttr().Get()
    faces, corners = triangulate(counts, idx)
    st = UsdGeom.PrimvarsAPI(mesh).GetPrimvar("st")
    wedge_uv = None
    if st and st.HasAuthoredValue():
        uv = np.asarray(st.ComputeFlattened(), dtype=np.float64)  # one value per face-vertex
        if st.GetInterpolation() == UsdGeom.Tokens.faceVarying:
            wedge_uv = uv[corners].reshape(-1, 2)
        elif st.GetInterpolation() in (UsdGeom.Tokens.vertex, UsdGeom.Tokens.varying):
            wedge_uv = uv[faces].reshape(-1, 2)
    ms = pymeshlab.MeshSet()
    if wedge_uv is not None:
        # pymeshlab cannot take per-wedge texture ids, so give each (vertex, uv) pair its own vertex,
        # let it transfer vertex UVs to wedges (which assigns the texture id), then re-weld the seams.
        key = np.concatenate([faces.reshape(-1, 1).astype(np.float64), np.round(wedge_uv, 6)], axis=1)
        uniq, inv = np.unique(key, axis=0, return_inverse=True)
        ms.add_mesh(pymeshlab.Mesh(vertex_matrix=pts[uniq[:, 0].astype(int)],
                                   face_matrix=inv.reshape(-1, 3).astype(np.int32), v_tex_coords_matrix=uniq[:, 1:]))
        ms.compute_texcoord_transfer_vertex_to_wedge()
        ms.meshing_merge_close_vertices(threshold=pymeshlab.PercentageValue(0.0001))
        ms.meshing_decimation_quadric_edge_collapse_with_texture(
            targetfacenum=int(target), qualitythr=0.3, preserveboundary=True, boundaryweight=1.0,
            optimalplacement=True, preservenormal=True, planarquadric=True)
    else:
        ms.add_mesh(pymeshlab.Mesh(vertex_matrix=pts, face_matrix=faces))
        ms.meshing_decimation_quadric_edge_collapse(
            targetfacenum=int(target), qualitythr=0.3, preserveboundary=True, preservenormal=True,
            optimalplacement=True, planarquadric=True, autoclean=True)
    out = ms.current_mesh()
    v, f = out.vertex_matrix(), out.face_matrix()
    for name in DROPPABLE:
        pv = UsdGeom.PrimvarsAPI(mesh).GetPrimvar(name)
        if pv and pv.HasAuthoredValue() and pv.GetInterpolation() != UsdGeom.Tokens.constant:
            pv.GetAttr().Block()
            if pv.IsIndexed():
                pv.GetIndicesAttr().Block()
    mesh.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(v.astype(np.float32)))
    mesh.GetFaceVertexCountsAttr().Set(Vt.IntArray.FromNumpy(np.full(len(f), 3, dtype=np.int32)))
    mesh.GetFaceVertexIndicesAttr().Set(Vt.IntArray.FromNumpy(f.reshape(-1).astype(np.int32)))
    mesh.GetNormalsAttr().Set(Vt.Vec3fArray.FromNumpy(crease_normals(v, f).astype(np.float32)))
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
    if wedge_uv is not None:
        new_st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray,
                                                         UsdGeom.Tokens.faceVarying)
        new_st.Set(Vt.Vec2fArray.FromNumpy(out.wedge_tex_coord_matrix().astype(np.float32)))
        if new_st.IsIndexed():
            new_st.GetIndicesAttr().Block()
    lo, hi = v.min(axis=0), v.max(axis=0)
    mesh.GetExtentAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*lo), Gf.Vec3f(*hi)]))
    return len(f)


# Viewport preview colours; the MDL materials ignore them, so they are dropped rather than remapped.
DROPPABLE = {"displayColor", "displayOpacity"}


def unsupported(mesh):
    if UsdGeom.Subset.GetAllGeomSubsets(mesh):
        return "geomsubsets"
    for pv in UsdGeom.PrimvarsAPI(mesh).GetPrimvars():
        if pv.GetPrimvarName() in DROPPABLE:
            continue
        if pv.GetPrimvarName() != "st" and pv.HasAuthoredValue() and pv.GetInterpolation() in (
                UsdGeom.Tokens.faceVarying, UsdGeom.Tokens.vertex, UsdGeom.Tokens.varying, UsdGeom.Tokens.uniform):
            return f"primvar {pv.GetPrimvarName()}"
    return None


def unlink(path):
    """Replace a hard link with its own copy so edits never reach the original asset."""
    fd, tmp = tempfile.mkstemp(dir=os.path.dirname(path))
    os.close(fd)
    shutil.copy2(path, tmp)
    os.replace(tmp, path)


def process(path, budget):
    layer = Sdf.Layer.FindOrOpen(path)
    stage = Usd.Stage.Open(layer, Usd.Stage.LoadNone)
    meshes = [UsdGeom.Mesh(p) for p in stage.Traverse() if p.IsA(UsdGeom.Mesh) and layer.GetPrimAtPath(p.GetPath())]
    total = sum(mesh_tris(m) for m in meshes)
    if total <= budget:
        return None
    heavy = [m for m in meshes if mesh_tris(m) > MIN_KEEP]
    light = total - sum(mesh_tris(m) for m in heavy)
    ratio = max(budget - light, MIN_KEEP * len(heavy)) / max(1, total - light)
    skipped = []
    if os.stat(path).st_nlink > 1:
        unlink(path)
        layer = Sdf.Layer.FindOrOpen(path)
        layer.Reload(force=True)
        stage = Usd.Stage.Open(layer, Usd.Stage.LoadNone)
        heavy = [UsdGeom.Mesh(stage.GetPrimAtPath(m.GetPath())) for m in heavy]
    after = light
    for m in heavy:
        why = unsupported(m)
        if why:
            skipped.append(f"{m.GetPath().name}: {why}")
            after += mesh_tris(m)
            continue
        after += simplify(m, max(MIN_KEEP, mesh_tris(m) * ratio))
    layer.Save()
    return total, after, skipped


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--budget", type=int, default=25000)
    args = ap.parse_args()
    before = after = 0
    for root in ROOTS:
        for dirpath, _, files in os.walk(root):
            for name in sorted(files):
                if not name.endswith((".usd", ".usdc", ".usda")) or "/.thumbs" in dirpath:
                    continue
                path = os.path.join(dirpath, name)
                r = process(path, args.budget)
                if r:
                    t0, t1, skipped = r
                    before, after = before + t0, after + t1
                    print(f"{t0 / 1e3:8.0f}k -> {t1 / 1e3:6.0f}k  {os.path.relpath(path, LITE)}"
                          + (f"  (skipped {skipped})" if skipped else ""))
    print(f"total {before / 1e6:.2f} M -> {after / 1e6:.2f} M triangles")


if __name__ == "__main__":
    main()
