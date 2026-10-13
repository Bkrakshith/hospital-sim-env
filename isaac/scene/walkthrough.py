"""Walkthrough video: shot list and camera paths (pure data + maths, no Isaac imports).

Each shot eases its camera eye and look-at target from a start pose to an end pose (or orbits a point).
Coordinates are wing-local metres (see layout.py); WING_ORIGIN converts them to world space.
"""

import math

from layout import WING_ORIGIN

FPS = 30
RESOLUTION = (1920, 1080)
SHOT_SECONDS = 5.0
CROSSFADE_SECONDS = 0.5

# name, caption, focal length (mm), and either {"eye": (a, b), "target": (a, b)} or {"orbit": {...}}.
SHOTS = [
    ("hospital", "Hospital corridor", 18,
     {"eye": ((1.0, 9.5, 1.6), (1.0, 4.2, 1.55)), "target": ((1.0, -4.0, 1.25), (1.0, -6.0, 1.2))}),
    ("link", "Link to the surgical wing", 18,  # x 1.4: centre of the 2.73 m doorway into the 4 m link
     {"eye": ((1.4, -0.3, 1.6), (1.4, -3.8, 1.6)), "target": ((1.4, -12.0, 1.6), (1.4, -12.0, 1.7))}),
    ("corridor", "Surgical corridor", 18,
     {"eye": ((-10.8, -7.0, 1.7), (5.5, -7.0, 1.7)), "target": ((-3.0, -9.3, 1.2), (14.0, -9.3, 1.2))}),
    ("preop", "Pre-operative bay", 16,
     {"eye": ((-3.4, -5.3, 2.2), (-5.6, -5.2, 2.1)), "target": ((-8.8, -1.2, 0.7), (-1.6, -1.2, 0.7))}),
    ("pacu", "Post-anaesthesia recovery", 16,
     {"eye": ((12.9, -5.3, 2.2), (9.4, -5.3, 2.0)), "target": ((4.2, -0.8, 0.7), (6.4, -0.8, 0.7))}),
    ("store", "Sterile store", 16,
     {"eye": ((-8.0, -10.8, 2.2), (-8.0, -12.4, 1.9)), "target": ((-9.0, -17.6, 0.8), (-6.2, -17.6, 0.8))}),
    ("scrub", "Scrub room", 16,
     {"eye": ((-2.0, -10.7, 1.8), (-2.0, -13.6, 1.7)), "target": ((-2.0, -17.8, 1.2), (2.5, -14.2, 1.3))}),
    ("theatre", "Operating theatre", 16,
     {"orbit": {"center": (5.2, -14.6, 0.9), "radius": 3.8, "height": 2.4, "deg": (150.0, 235.0)}}),
    # Main building (wing-local = world - WING_ORIGIN): the ward corridor from the nurse base, the A&E majors corridor.
    ("ward", "Six-bed ward", 16,
     {"eye": ((-0.33, 10.95, 1.6), (0.67, 18.45, 1.6)), "target": ((1.07, 32.45, 1.2), (1.07, 32.45, 1.2))}),
    ("ae", "Emergency department: majors", 16,
     {"eye": ((-58.23, 13.45, 1.7), (-58.23, 24.45, 1.7)), "target": ((-58.23, 37.45, 1.1), (-58.23, 37.45, 1.1))}),
]

# Each shot is rendered from a stage that drops what that shot can never see, so path tracing at 1080p
# fits an 8 GB GPU. Shot name -> stage group; the stages are written by build_hospital.py.
VIDEO_STAGES = {
    "hospital": ["/World/Wing/ORSuite", "/World/Wing/ORSuiteMods", "/World/Wing/Dressing/Store",
                 "/World/Wing/Dressing/Scrub", "/World/Wing/Dressing/Theatre"],
    "wing": ["/World/Hospital", "/World/HospitalLighting", "/World/Building", "/World/BuildingMods"],
}
SHOT_STAGE = {"hospital": "hospital", "ward": "hospital", "ae": "hospital"}  # every other shot uses "wing"


def stage_group(shot_name):
    return SHOT_STAGE.get(shot_name, "wing")


CREDITS_TITLE = "Hospital Digital Twin"
CREDITS_SUBTITLE = "A surgical wing built in NVIDIA Isaac Sim"
CREDITS = [
    ("3D assets", [
        "NVIDIA Isaac for Healthcare — i4h sim-ready assets (Apache 2.0)",
        "Lightwheel SimReady medical assets © Lightwheel AI (CC BY-NC 4.0)",
        "NVIDIA Isaac Sim hospital environment",
    ]),
    ("Rendered with", ["NVIDIA Isaac Sim 6.0 · RTX path tracing"]),
]
WATERMARK = "Assets: NVIDIA Isaac for Healthcare · Lightwheel SimReady · NVIDIA Isaac Sim"


def frames_per_shot():
    return int(round(SHOT_SECONDS * FPS))


def _ease(t):
    return t * t * (3.0 - 2.0 * t)  # smoothstep: gentle start and stop


def _lerp(a, b, t):
    return tuple(x + (y - x) * t for x, y in zip(a, b))


def pose(shot, i):
    """World-space (eye, target) for frame i of a shot."""
    _, _, _, path = shot
    t = _ease(i / max(1, frames_per_shot() - 1))
    if "orbit" in path:
        o = path["orbit"]
        a = math.radians(o["deg"][0] + (o["deg"][1] - o["deg"][0]) * t)
        cx, cy, cz = o["center"]
        eye = (cx + o["radius"] * math.cos(a), cy + o["radius"] * math.sin(a), o["height"])
        target = o["center"]
    else:
        eye = _lerp(*path["eye"], t)
        target = _lerp(*path["target"], t)
    ox, oy, oz = WING_ORIGIN
    return tuple(v + d for v, d in zip(eye, (ox, oy, oz))), tuple(v + d for v, d in zip(target, (ox, oy, oz)))
