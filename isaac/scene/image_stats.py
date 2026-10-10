"""Exposure stats for renders: mean luminance and share of near-white (clipped) pixels.

    uv run --no-project --with pillow --with numpy python scene/image_stats.py renders/*.png
"""

import sys

import numpy as np
from PIL import Image

for path in sys.argv[1:]:
    rgb = np.asarray(Image.open(path).convert("RGB"), dtype=np.float32) / 255.0
    lum = rgb @ np.array([0.2126, 0.7152, 0.0722], dtype=np.float32)
    print(f"{path.split('/')[-1]:28s} mean {lum.mean():.2f}  clipped(>0.97) {100 * (lum > 0.97).mean():5.1f}%  "
          f"p5/p50/p95 {np.percentile(lum, 5):.2f}/{np.percentile(lum, 50):.2f}/{np.percentile(lum, 95):.2f}")
