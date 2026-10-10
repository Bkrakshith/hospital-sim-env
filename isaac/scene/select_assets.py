"""Point the scene's asset links at full-resolution or reduced-texture assets.

    python scene/select_assets.py lite   # textures capped at 512/1024 px, fits an 8 GB GPU (default)
    python scene/select_assets.py full   # original assets, needs roughly 12 GB+ of VRAM

The USD layers reference assets through assets/isaac and assets/i4h, so no rebuild is needed.
Create the lite mirror first with scene/make_lite_assets.py.
"""

import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BASE = {"full": "~/isaacsim_assets/Assets/Isaac", "lite": "~/isaacsim_assets/lite/Assets/Isaac"}
LINKS = {"isaac": "6.0", "i4h": "Healthcare/0.7.0/724f82e"}


def select(mode):
    base = os.path.expanduser(BASE[mode])
    os.makedirs(os.path.join(REPO, "assets"), exist_ok=True)
    for name, sub in LINKS.items():
        target = os.path.join(base, sub)
        if not os.path.isdir(target):
            sys.exit(f"missing {target} (run scene/make_lite_assets.py for lite)")
        link = os.path.join(REPO, "assets", name)
        if os.path.lexists(link):
            os.remove(link)
        os.symlink(target, link)
        print(f"assets/{name} -> {target}")


if __name__ == "__main__":
    select(sys.argv[1] if len(sys.argv) > 1 else "lite")
