# Isaac Sim hospital scene

The `isaac/` half of `hospital-sim-env`; every path in this README is relative to this folder. The scene knows
nothing about any robot and needs no sibling repo, only Isaac Sim 6.0 and the asset mirrors described below.
`g1-isaac-sim` layers the robot and physics on top of it, `g1-mujoco-sim` rebuilds the geometry from
`scene/layout.py`, and `hospital-sim-ros2` navigates in the same world frame. `docs/interfaces.md` in the
master repo `hospital-sim` lists what the other components use from here. Until the split into component repos
this folder was `sim/isaac/hospital/` in the master repo.

A hospital scene for Isaac Sim 6.0: NVIDIA's Isaac `hospital.usd` building with a surgical wing
attached at its south-east door. The wing has a link corridor, a surgical corridor, pre-op, recovery (PACU),
a sterile store, a scrub room and a fully fitted operating theatre (the Isaac for Healthcare OR suite).
It is built for a Unitree G1: one floor level, 2 m and 4 m corridors, 2 m clear doorways, and furniture
kept at least 1 m from every opening.

**This scene authors no physics.** The robot stages in `g1-isaac-sim/usd/` add colliders on top of it
(`g1-isaac-sim/make_stage.py`). Some referenced assets bring their own physics: the Isaac hospital has static
colliders, and the Lightwheel props have rigid bodies and colliders.

## Open it

```bash
./launch_isaac.sh --open      # Isaac Sim + MCP extension, then opens usd/hospital_g1.usd
```

Run the `uv run … python scene/…` commands below from this folder.

The scene includes viewing cameras under `/World/Cameras`. Pick one from the viewport's camera menu:

| Camera | View |
|---|---|
| `Cam_HospitalToWing` | from the hospital corridor through the SE door into the wing |
| `Cam_Link`, `Cam_CorridorWest`, `Cam_CorridorEast` | link corridor, surgical corridor both ways |
| `Cam_PreOp`, `Cam_PACU`, `Cam_Store`, `Cam_Scrub` | each room from its doorway |
| `Cam_Theatre`, `Cam_TheatreWide` | operating theatre |
| `Cam_Plan` | orthographic top-down plan (sits just under the ceiling) |

Reference renders of every camera are in `renders/`.

## Layout (wing-local metres, origin at the hospital's SE door)

```
 y=0   ────────────── hospital south wall ── [SE double door] ──────────────
       |      PRE-OP (x -10..0)      | LINK |          PACU (x 2..14)          |
 y=-6  ───────────[door]─────────────       ───────────[door]─────────────────
       |                  SURGICAL CORRIDOR (x -12..14)                        |
 y=-10 ──[door]────────────┬─[door]─┬──────────[door]────────────────────────
       |  STERILE STORE    | SCRUB  |          OPERATING THEATRE (x 0..10)
 y=-18 ────────────────────┴────────┤                                       y=-20
```

The wing's origin is at world (17.93, -4.45, 0). The i4h OR suite is rotated -90° so that its three rooms
(side room, anteroom, theatre) open onto the corridor. Its side room becomes the sterile store and its
anteroom the scrub room.

## Repository

| Path | What |
|---|---|
| `usd/hospital_g1.usd` | root stage (sublayers below); open this one |
| `usd/building.usd` | Isaac hospital, with the SE door leaves removed |
| `usd/wing_shell.usd` | floors, ceilings, walls and doors built from the i4h Hospital_A 2 m kit |
| `usd/or_suite.usd` | i4h OR suite placed into the wing; door cuts and prop moves are overrides only |
| `usd/dressing_<room>.usd` | furniture per room |
| `usd/lighting.usd`, `usd/cameras.usd` | room ceiling lights, viewing cameras |
| `usd/build_report.json` | last build's checks: doorway clearance, overlaps, room bounds, physics, GPU triangles |
| `scene/layout.py` | **edit this** to change rooms, doors, furniture, cameras |
| `scene/build_hospital.py` | turns `layout.py` into the USD layers and verifies them |
| `scene/assets.py` | asset catalog (short key to USD path) |
| `scene/make_lite_assets.py`, `scene/decimate_lite_assets.py`, `scene/select_assets.py` | the 8 GB asset mirror (below) |
| `scene/render_views.py` | renders every camera to `renders/` from inside Isaac Sim (e.g. over MCP) |
| `launch_isaac.sh`, `scene/enable_mcp_late.py` | Isaac launcher; enables the MCP extension after startup |
| `scene/walkthrough.py`, `scene/render_walkthrough.py`, `scene/make_video.py` | walkthrough video: shots, path-traced frames, edit + encode |
| `scene/image_stats.py` | exposure check for renders (median brightness, % clipped) |
| `renders/video/` | the finished video |

## Rebuild after editing `layout.py`

```bash
uv run --no-project --with usd-core python scene/build_hospital.py
```

The build prints the report and exits with its issue list. It needs `0 issues` before you use the scene.
Reopen the stage in a **fresh** Isaac session: reopening in the same session keeps the old stage's GPU
memory, which is enough to push an 8 GB card over.

## Fitting an 8 GB GPU (RTX 3070)

At full fidelity the scene needs roughly 12 GB+ of VRAM. The i4h props are film-density, for example a
1.1 M-triangle mask box and 1.6 M-triangle drug cabinets. RTX runs out of memory building their ray-tracing
structures. The scene therefore loads assets through `assets/isaac` and `assets/i4h`, which point to one of:

- **lite** (default): `~/isaacsim_assets/lite/…`, a mirror of the used asset trees. Every i4h/Lightwheel
  asset is simplified to at most 25k triangles (UVs kept) and textures are capped at 512–1024 px.
  Untouched files are hard links, and the original assets are never modified.
- **full**: the original assets, for a 12 GB+ GPU.

```bash
uv run --no-project --with pillow python scene/make_lite_assets.py                       # textures + hard links
uv run --no-project --with usd-core --with pymeshlab python scene/decimate_lite_assets.py # meshes
python scene/select_assets.py lite     # or: full   (no rebuild needed)
```

Lighting has the same kind of switch. By default the ~400 per-module kit lights are replaced by about 40
room ceiling panels, and every one of the hospital's 153 ceiling fittings gets a real light (the building
ships with only a dome light). `HG1_LIGHTS=kit` keeps every kit light, which only fits larger GPUs.

Light levels are calibrated, not guessed: `LIGHT_LEVEL` in `build_hospital.py` puts the median pixel at
about 0.5 with nothing clipped under the default ACES exposure, and colour temperature is 5000 K.
`scene/image_stats.py renders/*.png` measures this; before calibration the rooms were 20%+ clipped white.

Measured on the RTX 3070 (lite, room lights, real-time RTX): all 11 cameras render with a peak of about
6.3–7.0 GB, including about 1.9 GB of desktop and Isaac overhead.

## Walkthrough video

A ~44 s, 1920×1080, 30 fps path-traced walkthrough: title card, eight shots (hospital corridor, link,
surgical corridor, pre-op, PACU, sterile store, scrub room, theatre orbit) with crossfades and captions,
and a credits card for NVIDIA Isaac for Healthcare, Lightwheel and NVIDIA Isaac Sim.

```bash
# Render each group in its own FRESH Isaac session; frames are resumable if a session dies.
./launch_isaac.sh     # then over MCP:  WALK_GROUP=hospital, runpy scene/render_walkthrough.py
./launch_isaac.sh     # restart, then:  WALK_GROUP=wing
uv run --no-project --with pillow --with numpy --with imageio-ffmpeg python scene/make_video.py
```

- Shots, captions and credits are in `scene/walkthrough.py`. `usd/video_hospital.usd` and `usd/video_wing.usd`
  are the full scene minus what their shots never see. Path tracing at 1080p does not fit 8 GB with
  everything loaded.
- Samples per frame: `WALK_SPP` (the video uses 96, denoised; about 21 s per frame on the RTX 3070, so the
  full render takes several hours). `WALK_PREVIEW=1` renders 3 frames per shot for checking camera paths.
- ffmpeg comes from the `imageio-ffmpeg` package (a bundled static binary), so no system install is needed.
- `make_video.py --draft` builds the edit from whatever frames exist.
- Licences: Lightwheel assets are CC BY-NC 4.0 (non-commercial, credit required), i4h assets are Apache 2.0,
  and the Isaac Sim hospital building is NVIDIA "Limited Use Content". Check these terms before public or
  commercial use.

## Notes for adding G1 and physics later

- Furniture and i4h props are USD **instances**, which is what lets identical props share GPU memory.
  Author collision on the instance root, or in the referenced asset, rather than on instance-proxy children.
- The Lightwheel carts and trays carry their own rigid bodies. Decide whether to keep or disable them when
  you set up physics.
- Every doorway in `layout.DOORWAYS` is kept clear by the build check, and the scrub-to-store double door's
  leaves are removed so G1 can pass through.
