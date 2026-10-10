"""Build a reduced-texture mirror of the assets this scene uses, so it fits an 8 GB GPU.

The mirror keeps the originals' directory layout, so every relative path inside the USD files still
resolves. Non-texture files are hard links (no extra disk space); textures are rewritten at lower
resolution under the same name and format:  >=1600 px -> 1024 px,  513..1024 px -> 512 px,  smaller kept.
Directories the scene does not use are symlinked to the originals. The originals are never modified.

    uv run --no-project --with pillow python scene/make_lite_assets.py [--force]
"""

import os
import sys
from multiprocessing import Pool

from PIL import Image

SRC = os.path.expanduser("~/isaacsim_assets/Assets/Isaac")
DST = os.path.expanduser("~/isaacsim_assets/lite/Assets/Isaac")
# (root relative to SRC, subtrees under it that get a real mirror)
MIRRORS = [
    ("6.0", ["Isaac/Environments/Hospital"]),
    ("Healthcare/0.7.0/724f82e", ["Props/shared_OR_without_Mark", "Props/Lightwheel"]),
]
TEXTURE_EXT = {".png", ".jpg", ".jpeg", ".tga", ".tif", ".tiff", ".bmp"}
FORCE = "--force" in sys.argv


def target_size(w, h):
    m = max(w, h)
    cap = 1024 if m >= 1600 else 512 if m > 512 else None
    if cap is None:
        return None
    s = cap / m
    return max(1, round(w * s)), max(1, round(h * s))


def link(src, dst):
    try:
        os.link(src, dst)
    except OSError:
        import shutil
        shutil.copy2(src, dst)


def process(job):
    src, dst = job
    if os.path.exists(dst) and not FORCE:
        return "skip"
    if os.path.exists(dst):
        os.remove(dst)
    if os.path.splitext(src)[1].lower() not in TEXTURE_EXT or "/.thumbs/" in src:
        link(src, dst)
        return "link"
    try:
        with Image.open(src) as im:
            size = target_size(*im.size)
            if size is None:
                link(src, dst)
                return "small"
            fmt = im.format
            out = im.resize(size, Image.LANCZOS)
            kw = {"quality": 92} if fmt == "JPEG" else {"compress_level": 1} if fmt == "PNG" else {}
            out.save(dst, format=fmt, **kw)
            return "resized"
    except Exception as e:  # unusual modes (e.g. 16-bit) keep the original texture
        if os.path.exists(dst):
            os.remove(dst)
        link(src, dst)
        return f"kept:{type(e).__name__}"


def mirror_parents(root, subtrees):
    """Symlink every sibling of the path down to each mirrored subtree; return file jobs."""
    src_root, dst_root = os.path.join(SRC, root), os.path.join(DST, root)
    os.makedirs(dst_root, exist_ok=True)
    keep = {tuple(s.split("/")) for s in subtrees}

    def walk(parts):
        here_src = os.path.join(src_root, *parts)
        here_dst = os.path.join(dst_root, *parts)
        os.makedirs(here_dst, exist_ok=True)
        for name in os.listdir(here_src):
            p = parts + (name,)
            if p in keep:
                continue
            if any(k[: len(p)] == p for k in keep):
                walk(p)
            elif not os.path.lexists(os.path.join(here_dst, name)):
                os.symlink(os.path.join(here_src, name), os.path.join(here_dst, name))

    walk(())
    jobs = []
    for sub in subtrees:
        s0, d0 = os.path.join(src_root, sub), os.path.join(dst_root, sub)
        for dirpath, _, files in os.walk(s0):
            d = os.path.join(d0, os.path.relpath(dirpath, s0))
            os.makedirs(d, exist_ok=True)
            jobs += [(os.path.join(dirpath, f), os.path.join(d, f)) for f in files]
    return jobs


def main():
    jobs = []
    for root, subtrees in MIRRORS:
        jobs += mirror_parents(root, subtrees)
    stats = {}
    with Pool(8) as pool:
        for r in pool.imap_unordered(process, jobs, chunksize=16):
            stats[r] = stats.get(r, 0) + 1
    print(f"{len(jobs)} files -> {DST}")
    for k, v in sorted(stats.items()):
        print(f"  {k}: {v}")


if __name__ == "__main__":
    main()
