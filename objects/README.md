# objects: scrub-nurse props: instruments, tool rack, instrument table, surgeon

The `objects/` half of `hospital-sim-env`: everything the scrub-nurse mission handles, as MJCF for MuJoCo and
physics-ready USD for Isaac. One script generates all of it from the i4h instrument meshes, procedural
definitions and `placements.yaml`. The real instruments are read through the `../isaac/assets/i4h` switch, so
the i4h assets have to be installed first (see `../isaac/README.md`). The G1-specific scripts in `tools/` need
`g1-brainco-description` next door. `g1-mujoco-sim`, `g1-isaac-sim` and `hospital-sim-ros2` consume the
outputs, and the master repo `hospital-sim` arranges all of these and provides `env.sh` for the shared `.venv`
with mujoco. Paths below are relative to this folder; the commands run from the master checkout.

```bash
hospital-sim-env/objects/build.sh        # regenerate meshes/, mjcf/, usd/, catalog.json (~1 min), then check_objects.py
.venv/bin/python hospital-sim-env/objects/check_objects.py              # compile + settle-in-rack test + preview renders (docs/)
.venv/bin/python hospital-sim-env/objects/tools/reach_check.py          # G1 IK reach to every slot   -> docs/reach_report.json
.venv/bin/python hospital-sim-env/objects/tools/hand_grasp_test_mujoco.py   # Revo2 in-hand grasp test -> docs/hand_grasp_spec.json
./g1-isaac-sim/run.sh --stage                          # after a rebuild: Isaac theatre layer (g1-isaac-sim/usd/)
```

Generated files are never edited by hand. **`placements.yaml` is the single source of object poses** for both
sims. It is read by `build_objects.py` (→ `mjcf/theatre_objects.xml`, `catalog.json:world_poses`) and by
`g1-isaac-sim/make_theatre_layer.py` (→ `g1-isaac-sim/usd/theatre_instruments.usd`). The mission's
`g1_mission/config/locations.yaml` is derived from it.

## Items

Canonical instrument frame: **+z along the long axis towards the handle / finger rings, origin on the axis at the
tip end.** An instrument standing in the rack therefore has the rack's orientation, with z = 0 at its tip. +x is
the wide transverse direction (plane of the rings or jaws), +y the thin one. In the rack, +x points away from the
robot. `/sim/objects`, the perception 6-D labels and the detections' `pose` all use this frame.

| Name (= label) | What | Size x·y·z (mm) | Mass | Grasp region z (mm) | Source / licence |
|---|---|---|---|---|---|
| `scalpel_handle` | scalpel (handle + blade), blade down | 14·3·153 | 25 g | 70–145 | i4h OR suite `SurgicalInstruments_A/sm_scalpelshandle_a01_01` (replaced the earlier procedural stand-in), Apache-2.0 |
| `adson_forceps` | Adson tissue forceps, 120 mm, **STAND-IN** | 8·10·120 | 16 g | 70–112 | procedural (this repo) |
| `angled_tweezers` | angled tweezers, tips down | 20·15·160 | 30 g | 70–152 | i4h Lightwheel `AngledTweezers001`, CC BY-NC 4.0 |
| `scissors` | Mayo scissors, rings up | 58·20·190 | 85 g | 70–145 (shanks; rings above) | i4h scissor workflow `Props/SurgicalInstruments/SurgicalScissors.usd`, Apache-2.0 |
| `needle_driver` | Mayo-Hegar needle driver 180 mm (7 in), rings up, **STAND-IN** | 57·9·179 | 62 g | 70–134 (straight shanks; rings from 154) | procedural (this repo) |
| `veress_needle` | Veress (pneumoperitoneum) needle, hub up | 19·10·214 | 20 g | 70–206 (hub 150–206) | i4h Lightwheel `PneumoperitoneumNeedle001` (cap removed), CC BY-NC 4.0 |
| `trocar` | laparoscopic trocar, head up | 65·53·145 | 60 g | 70–137 (cannula + head) | i4h Lightwheel `Trocar002`, CC BY-NC 4.0 |
| `tool_rack` | 7-slot upright rack, 90 mm pitch (static) | 124·640·58 | – | – | procedural |
| `instrument_table` | Mayo-stand-like cantilever tray, top 0.78 m (static) | 510·760·800 | – | – | procedural |
| `surgeon` | static mannequin 1.84 m, right hand held out palm-up, site `surgeon_hand` (static) | 538·731·1838 | – | hand site 2 cm above the palm | procedural |

- **Mission pick results** (`g1_mission/scripts/offline_trials.py`, this build): scissors, angled tweezers and
  needle driver 5/5 each, both without and with rack offsets of ±2 cm / ±0.08 rad. Scalpel, Adson, Veress and
  trocar 3/3 each.
- **Stand-ins.** Only two remain: the Adson forceps and the needle driver, which are box/capsule/torus models with
  catalogue dimensions and masses. The i4h OR suite's `sm_forceps` (a 180 mm dressing forceps) and `sm_clamps` (a
  hemostat) exist, but they are different instruments. The perception data uses them as neither labels nor
  distractors. Replace a stand-in by adding a mesh entry to `REAL` in `build_objects.py`.
- **Collision.** Real instruments use a stack of boxes, one per 10 mm slice along the axis: each box is the
  axis-aligned bounds of the collision section in that slice, and finger rings become solid. Stand-ins use their
  primitives. CoACD convex parts are still available (`COLLISION_MODE = "hulls"`), but PhysX let the thin, curved
  Mayo-scissor blade hulls through the rack guide (25–35° lean in Isaac vs 5° in MuJoCo). Boxes behave the same
  in both engines.
- **Mass and friction.** Masses are typical catalogue values. Inertia is computed from the collision geometry at
  uniform density, with a floor of 1e-6 kg m² applied identically in MJCF and USD. Friction: instruments 0.8
  (MuJoCo `condim 4`; USD static 0.8 / dynamic 0.7), furniture 0.6. `catalog.json` has every number: COM,
  principal inertia, bboxes, rack slot geometry, world poses.
- **Licences.** The Lightwheel assets are **CC BY-NC 4.0** (non-commercial, attribution). The i4h scissors and
  OR-suite scalpel are Apache-2.0.

## Tool rack and instrument table design

- **Hand size drives the pitch.** The Revo2 hand is about 226 × 171 × 56 mm (wrist to fingertip, finger-stack
  width, palm thickness), with 16–18 mm wide fingers. A wrap or pinch around an upright instrument puts the palm
  on one side and the fingers on the other, so it needs about ±45 mm along the row. The **90 mm pitch** leaves at
  least 58 mm from a grasped instrument's axis to the edge of its neighbour, even next to the 65 mm trocar head,
  which is at the end slot. Rings and jaws point at the robot, never into the neighbouring slots.
- **Upright, tips down, two-level guide, straight-lift extraction.**
  - A tip plate with pockets (0–8 mm above the tip) and a guide plate (40–50 mm) hold each instrument near
    vertical.
  - Each guide hole is sized per instrument from the **swept** collision section: everything from the tip up to
    2 mm above the guide top passes through the guide when the instrument is lifted straight out. This matters
    because the curved scissor tips and angled tweezer tips sit up to 11 mm off the shank axis. Clearance is
    2.5 mm per side, with a 6 mm minimum hole. Pockets use the tip section plus 2.5 mm, minimum 6 mm.
  - Holes sized only at guide height jammed the scissors (8.8 mm) and tweezers (4.2 mm) during the lift.
  - Upright holding comes from gravity: each instrument's centre of mass lies on the tight side of its hole, so it
    leans against that wall.
  - Checks:
    - `ros2_ws/src/hospital-sim-ros2/g1_mission/scripts/check_extraction.py`: all 7 lift clear (0.0 mm) through 10 cm.
    - Settle test (3 s): MuJoCo tilt ≤ 3.5°; Isaac (`g1-isaac-sim/tools/rack_settle_test.py`) ≤ 5.0°.
  - Instruments protrude 70–165 mm above the guide. The hand grasps from 20 mm above the guide up to just below the
    rings or hub.
- **Slot order.** Along rack +y, which is world +x (the robot's left when it faces the table): 0 scalpel_handle,
  1 adson_forceps, 2 angled_tweezers, 3 scissors, 4 needle_driver, 5 veress_needle, 6 trocar. Coloured slot markers
  are on the robot-side edge.
- **Table height 0.78 m** (tray top). In the G1 home pose the shoulder is at 1.076 m and the elbow at 0.889 m, so the
  grasp sites (0.86–0.93 m) are at about elbow height. The D435 (1.258 m, pitched 47.6° down) sees the whole rack
  from the stance. A Mayo stand is cantilevered: the post and H-base are on the far side, so the robot's feet never
  touch them. The tray is enlarged to 760 × 460 mm (large Mayo tray size) for the 7 × 90 mm rack.
- **Warning for the arm skills.** At the home pose the hands (z 0.67–0.84 m, 0.22–0.37 m ahead of the pelvis) are
  inside the 0.78 m tray when the robot stands at `theatre_instrument_table`. Raise the arms before the last step
  in, or the hands hit the tray and rack. With finger collisions now on in Isaac, this chatters and knocks
  instruments.
- **Reach** (`tools/reach_check.py`; pelvis fixed at the stance; damped-least-squares IK puts the hand's grasp point
  on each grasp site with the fingers levelled forward):
  - Arm only: the right hand reaches slots 0–4 (≤ 4.3 mm) and the left hand slots 1–6 (≤ 17 mm at slot 5).
  - With the waist: both hands reach all 7 slots, with the outer slots near the waist-roll limit.
  - Pick the hand by slot (right 0–3, left 4–6).

## Grasp physics (both sims)

`tools/hand_grasp_test_mujoco.py` and its Isaac twin `g1-isaac-sim/tools/hand_grasp_test.py` use the same setup. A
fixed-base G1 holds its right hand in free space. The instrument is placed at the pinch point (thumb_aux 0.8; the
thumb pad meets the index/middle pads at (142, 45, 15) mm in the wrist frame), pinched at its COM height. The hand
closes, the instrument is released, and the test holds for 1.5 s, then shakes (wrist ±0.35 rad and shoulder
±0.25 rad at 1 Hz, for 3 s). "Held" means within 80 mm of the release pose; "firm" means < 10 mm slip during the
shake.

| | MuJoCo | Isaac, hand physics fixed | Isaac, finger colliders raw |
|---|---|---|---|
| retained | 6/7 (Veress drops) | 5/7 final run (scalpel and Adson drop; 7/7 in an earlier run) | 6/7 (trocar drops) |
| firm through the shake | 1/7 (12–19 mm creep) | 5/7 (≤ 7.7 mm) | 5/7 |

A single generic pinch is not a grasp design. A closed Revo2 fist cannot squeeze anything thinner than about 25 mm,
so thin instruments need the thumb–index pinch, and they swing 20–60° about it while seating. Per-instrument grasps
belong to the mission's pick skill. The point of these tests is that finger contacts are **stable** in both engines
(no NaN, no explosion). See `g1-isaac-sim/README.md` for the fall test that motivated the Isaac fix.

## Placement (world frame; `placements.yaml`)

| Object | World pose | Notes |
|---|---|---|
| `instrument_table` | (25.60, -20.60), yaw -π/2 | theatre, south-east of the operating table; local +x (far side) = world -y |
| `tool_rack` | on the tray, 0.08 m toward the robot | row along world x (25.33 … 25.87), y ≈ -20.52 |
| `surgeon` | (23.85, -19.05), yaw π (facing west, at the operating table's east side) | `surgeon_hand` site at world (23.60, -18.70, 1.02) |
| robot stance `theatre_instrument_table` | pelvis (25.60, -20.19), yaw -π/2 | pelvis 0.33 m from the rack row |
| robot stance `theatre_surgeon_side` | pelvis (23.80, -18.25), yaw -π/2 | hand site 0.45 m ahead, 0.20 m to the right |

The surgeon stands where the two OR-suite surgical stools were, so `remove_scene_props` lists
`sm_surgicalchair_a01_01/_02`. The Isaac theatre layer deactivates them together with their collider boxes. **The
MuJoCo theatre must skip those two boxes of `g1-mujoco-sim/worlds/hospital_boxes.json`.**

## Using the MJCF

Each `mjcf/<name>.xml` is a self-contained model: body `<name>` sits at the origin, with a free joint for
instruments and none for static props. Mesh paths are relative to `mjcf/`, so use `<model>` / `<attach>`;
a plain `<include>` breaks the relative mesh paths:

```xml
<asset>
  <model name="scissors" file="../hospital-sim-env/objects/mjcf/scissors.xml"/>   <!-- path relative to your scene file (here: a sibling repo) -->
</asset>
<worldbody>
  <frame pos="25.605 -20.52 0.788" quat="0.7071 0 0 -0.7071">      <!-- from catalog.json world_poses -->
    <attach model="scissors" body="scissors" prefix=""/>
  </frame>
</worldbody>
```

`mjcf/theatre_objects.xml` does exactly this for all 10 objects (plus a floor). Python: `mujoco.MjSpec` +
`frame.attach_body(child_spec.body(name), "", "")`. The robot MJCF's pelvis body carries its own height, so set its
`pos` to 0 when you attach it under a stance frame.

Contact classes:
- Instrument collision geoms are group 3, `contype=conaffinity=1`, `condim=4`, margin 0.5 mm. They collide with the
  robot (collision geoms `contype 2 / conaffinity 1`) and with the world.
- Furniture collision is group 0, so the lidar sees it. Visual geoms are group 2.
- Sites: `<name>_grasp` (centre of the grasp region) and `surgeon_hand`. Use `cone="elliptic" impratio="10"`.

## Using the USD

`usd/<name>.usd` has defaultPrim `/<name>`, with `visual/`, `collision/` (guide purpose) and site Xforms.
- **Instruments.** `PhysicsRigidBodyAPI` + `MassAPI` on the root. PhysX tuning: CCD on, 16/4 solver iterations, max
  depenetration 1 m/s, contact offset 2 mm, rest offset 0.
- **Real instruments** reference the i4h asset through `../../isaac/assets/i4h/...`. This keeps the textures and the
  repo's lite/full switch. The asset's own rigid bodies and colliders are stripped with `delete apiSchemas`, and
  its collider, site and cap prims are deactivated.
- **Furniture and the surgeon** are static colliders.

## Files

| Path | What |
|---|---|
| `placements.yaml` | world poses, rack slots, robot stances, props to remove (**edit this**) |
| `build_objects.py`, `build.sh` | the generator (uv: usd-core, trimesh, coacd) |
| `check_objects.py` | compile, settle test, renders → `docs/` |
| `tools/g1_ik.py`, `tools/reach_check.py` | arm IK helper, reach analysis → `docs/reach_report.json` |
| `tools/hand_grasp_test_mujoco.py` | in-hand grasp test, writes `docs/hand_grasp_spec.json` (read by the Isaac twin) |
| `catalog.json` | generated: dimensions, masses, inertia, grasp regions, rack slot geometry, world poses |
| `meshes/<name>/` | generated OBJ visuals (one per colour) |
| `mjcf/`, `usd/` | generated models |
| `docs/` | `theatre_setup.png`, `rack_closeup.png`, `rack_robot_view.png`, `instrument_lineup.png`, reports |
