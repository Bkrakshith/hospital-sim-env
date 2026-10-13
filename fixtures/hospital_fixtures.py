"""Moving building fixtures of the hospital: fire doors, bed lifts and their buttons.

One module, two users. The scene builders (Isaac USD, MuJoCo MJCF) call `build_spec(layout)` to get the
geometry and the joint names. The simulators (g1-isaac-sim, g1_mujoco_sim) run one `FixtureController`
each and call `step()` every physics step: it reads the joint positions and velocities and returns the
joint efforts (N*m for hinges, N for slides). The door closers, hold-open magnets, lift door drives and
button springs are therefore the same code in both simulators.

Pure Python (math only), so it imports in Isaac's Python, in ROS 2 nodes and in plain tests.

Conventions
- Wing-local frame of hospital-sim-env/isaac/scene/layout.py (metres, +x east, +y north, +z up).
  `to_world()` adds WING_ORIGIN. `to_wing()` subtracts it.
- Every joint, body and geom name starts with "fx_" so map tools can ignore moving fixtures.
- Hinge angle 0 = closed. Positive = counter-clockwise about +z, seen from above.
- Lift panel slide 0 = closed, `travel` = fully open. Button slide 0 = out, positive = pressed in.

Sources for the numbers are in layout.py (FIRE_DOORS, BUILDING_FIRE_DOORS, LIFT_SPEC, LIFT_BANKS) and below.

Spec contract (schema 2; the scene builders WP3 Isaac and WP4 MuJoCo read it)
- Frame. Every coordinate in the spec is wing-local, also for the main building. build_spec() converts the
  world-frame entries of layout.py (BUILDING_FIRE_DOORS, BUILDING_LIFTS, BUILDING_LIFT_CALL, LIFT_BANKS["Main"])
  by subtracting WING_ORIGIN. "source_frame" ("wing" or "world") tells where an item came from. Builders add
  "wing_origin" once, as before.
- Tiers. build_spec(layout, tier) keeps the items whose "tier" is <= the tier: "wing" (only the surgical wing, the
  schema-1 output), "tier1", "tier2" (the default) or "full". With tier=None it reads the environment variable
  HG1_FIXTURES. spec["tier"] records the choice and spec["excluded"] lists the items left out. Every consumer must
  use the same tier: the scene build (building_hide(), building_openings()), the fixture USD/MJCF and the run time.
- Groups. Each door has "group": "Doors" (2 leaves) or "Doors1" (1 leaf). spec["groups"] lists the names per group
  in spec order: {"Doors": [...], "Doors1": [...], "Lifts": [...], "Calls": [...]}, and spec["group_dof"] gives the
  joints per articulation: Doors 2, Doors1 1, Lifts 4 (2 panels + Open + Close), Calls 1. All members of a group
  have the same joint count and the same joint order, so one Isaac tensor view per group works without a fallback.
  Isaac authors /World/Fixtures/<group>/<name>. A door with 3 or more leaves raises ValueError.
- Uniform door fields, for 1 and 2 leaves alike: "n_leaves"; "span" = [[x, y], [x, y]], the two ends of the closed
  doorset along the door line (hinge to hinge for a pair, hinge to the leaf's free edge for a single leaf); "transom"
  (bool: a panel from the leaf head to "ceiling", m); "swing_limit_deg" (the joint "range" is clamped to it: 100 in
  free-standing frames, 92 in building jambs, which are not in the moving parts' collision filter);
  "side_panels" ([{"name", "lo", "hi"}], fixed panels of the doorset, already built as static partitions: do not
  build them again); "use", "replaces", "opening" (pass-through from layout.py).
- Leaf fields (unchanged from schema 1): "joint" fx_<door>_leaf<i>, "hinge", "closed_dir", "closed_yaw", "width",
  "thickness", "height", "range" [lo, hi] rad, "hold" (rad, or None = not held), "bolted", "mass" kg,
  "inertia_z" kg m2, "lever" m (hinge to handle), "vision_panel".
- Lifts: "name", "bank", "front_x", "center_y", "door_w", "door_h", "car", "panels" (fx_<lift>_door_a/b),
  "buttons" (fx_<lift>_Open/Close). Calls: "joint" fx_<call>, "kind" "call", "lifts", "pos", "axis" (press
  direction, into the wall), "bank". spec["lift_banks"]: per bank "name", "lifts", "calls", "front" [x, y_lo, y_hi],
  "lobby" [x0, y0, x1, y1], "ceiling_z", "prefix", "static_boxes" (names). spec["static_boxes"]: (name, lo, hi)
  for all banks; the wing bank names are unchanged (fx_front_i, fx_header_i, fx_<lift>_car_*), the main bank's
  front and header boxes are fx_Main_front_i and fx_Main_header_i.
- Names are unique across doors, lifts, calls, joints and static boxes; build_spec() raises ValueError otherwise
  (for example a building door named like a wing door).
- Static partitions (screens, rebuilt walls, junction boxes) are not fixtures. static_partitions(layout) returns
  them as world-frame boxes for the visuals, the Isaac colliders and the MuJoCo world.

Fire alarm. One building-wide alarm (`set_alarm`) releases every magnet in the wing and the main building and
puts every lift on fire recall. Alarm zoning (per compartment) is future work.
"""

import json
import math
import os

PREFIX = "fx_"

# Door closer (BS EN 1154) tuned so that the opening force at the handle stays inside Approved Document M
# vol 2 3.10(a): <= 30 N from 0-30 deg and <= 22.5 N from 30-60 deg (checked in the tests).
CLOSER_T0 = 11.0         # N*m, closing moment at the closed position
CLOSER_K = 6.0           # N*m/rad, extra moment per radian of opening
CLOSER_C = 60.0          # N*m*s/rad, hydraulic damping, closing direction only (about 5 s from 90 deg)
CLOSER_C0 = 1.0          # N*m*s/rad, hinge friction, both directions
CLOSER_EPS = 0.02        # rad, width of the smooth sign at the closed position
MAGNET_T = 80.0          # N*m, magnet holding moment (about 100 N at 0.8 m); a harder pull releases it
MAGNET_K = 2000.0        # N*m/rad, magnet stiffness inside the capture band
MAGNET_CAPTURE = math.radians(3.0)
BOLT_K, BOLT_C = 5000.0, 200.0  # flush-bolted leaf
HOLD_ANGLE = math.radians(90.0)
SWING_LIMIT = math.radians(100.0)
LEAF_H = 2.1             # m, doorset leaf height
LEAF_T = {"FD30S": 0.044, "FD60S": 0.054, "EXIT": 0.054}  # m, leaf thickness
LEAF_KG_PER_M2 = {"FD30S": 20.0, "FD60S": 26.0, "EXIT": 26.0}
HANDLE_INSET = 0.07      # m, handle distance from the leading edge
VISION_PANEL = (0.15, 0.5, 1.8)  # (inset from the edges, bottom z, top z), HTM 05-02 App C: down to 500 mm

# Lift buttons. BS EN 81-70 (secondary source): operating force 2.5-5 N. Travel 4 mm, contact at 2.5 mm.
BUTTON_F0, BUTTON_K, BUTTON_C = 2.5, 625.0, 4.0
BUTTON_TRAVEL, BUTTON_TRIGGER, BUTTON_RELEASE = 0.004, 0.0025, 0.0012
BUTTON_SIZE, BUTTON_KG = 0.05, 0.02
# Lift door drive (closed-loop VVVF, HTM 08-02 6.114): trajectory tracking with a force limit.
DOOR_KP, DOOR_KD, DOOR_OPEN_F = 4000.0, 400.0, 400.0
STALL_ERR, STALL_V, STALL_S = 0.03, 0.02, 0.3  # reopen when the panels stop this far from the target
FORCE_MARGIN = 0.95      # closing force cap = 95 % of the stall limit, so the force stays below it
# Nudging (BS EN 81-20, secondary source; HTM 08-02 6.115 adjustable nudging): after repeated light-curtain
# reopens or a long obstruction, the doors close slowly with the curtain ignored (KE <= 4 J, buzzer).
NUDGE_REOPENS, NUDGE_BLOCKED_S, NUDGE_SLOWDOWN = 3, 18.0, 2.5

_DIRS = {"+x": (1.0, 0.0), "-x": (-1.0, 0.0), "+y": (0.0, 1.0), "-y": (0.0, -1.0)}


def _sign(v):
    return 1.0 if v > 0 else -1.0


def _rot(d, a):
    c, s = math.cos(a), math.sin(a)
    return (c * d[0] - s * d[1], s * d[0] + c * d[1])


# ---------------------------------------------------------------- spec

SCHEMA = 2
TIERS = {"wing": 0, "tier1": 1, "tier2": 2, "full": 3}
DEFAULT_TIER = "tier2"   # design: ship tiers 1-2 by default; tier 3 behind HG1_FIXTURES=full (CPU budget)
TIER_ENV = "HG1_FIXTURES"
GROUPS = ("Doors", "Doors1", "Lifts", "Calls")
GROUP_DOF = {"Doors": 2, "Doors1": 1, "Lifts": 4, "Calls": 1}
WING_CEILING = 3.6       # m, kit ceiling of the wing (default "ceiling" of wing doors)


def resolve_tier(tier=None):
    """Tier name from the argument, else from $HG1_FIXTURES, else DEFAULT_TIER."""
    t = tier or os.environ.get(TIER_ENV) or DEFAULT_TIER
    if t not in TIERS:
        raise ValueError(f"unknown fixture tier {t!r}: use one of {sorted(TIERS, key=TIERS.get)}")
    return t


def _included(item_tier, source_frame, tier):
    if tier == "wing":
        return source_frame == "wing"
    return item_tier <= TIERS[tier]


def _to_wing_xy(layout, x, y):
    ox, oy, _ = layout.WING_ORIGIN
    return (round(x - ox, 9), round(y - oy, 9))


def _merge(a, b, what):
    """Merge two name -> entry dicts. A name in both raises ValueError (it would overwrite silently)."""
    clash = sorted(set(a) & set(b))
    if clash:
        raise ValueError(f"{what}: names used twice {clash}")
    out = dict(a)
    out.update(b)
    return out


def _door_spec(layout, dname, d, src):
    rating = d["rating"]
    t = LEAF_T[rating]
    lim = math.radians(d.get("swing_limit_deg", 100))
    leaves = []
    for i, (hinge, closed, width) in enumerate(d["leaves"]):
        if src == "world":
            hinge = _to_wing_xy(layout, hinge[0], hinge[1])
        cd = _DIRS[closed]
        ccw = (-cd[1], cd[0])  # where the leaf points after +90 deg

        def side_sign(side):
            v = _DIRS[side]
            return _sign(ccw[0] * v[0] + ccw[1] * v[1])

        if d["swing"] == "both":
            lo, hi = -lim, lim
        else:
            s = side_sign(d["swing"])
            lo, hi = (0.0, lim) if s > 0 else (-lim, 0.0)
        bolted = i in d.get("bolted", [])
        hold = None if (d["hold"] is None or bolted) else side_sign(d["hold"]) * HOLD_ANGLE
        mass = LEAF_KG_PER_M2[rating] * width * LEAF_H
        leaves.append({
            "joint": f"{PREFIX}{dname}_leaf{i}",
            "hinge": [hinge[0], hinge[1]],
            "closed_dir": list(cd),
            "closed_yaw": math.atan2(cd[1], cd[0]),
            "width": width, "thickness": t, "height": LEAF_H,
            "range": [lo, hi], "hold": hold, "bolted": bolted,
            "mass": round(mass, 2),
            "inertia_z": round(mass * width * width / 3.0, 3),
            "lever": width - HANDLE_INSET,
            "vision_panel": [VISION_PANEL[0], VISION_PANEL[1], VISION_PANEL[2]],
        })
    n = len(leaves)
    if n not in (1, 2):
        raise ValueError(f"door {dname}: {n} leaves (the tensor-view groups take 1 or 2)")
    # span: the extreme ends of the closed leaves along the door line
    u = leaves[0]["closed_dir"]
    ends = []
    for lf in leaves:
        hx, hy = lf["hinge"]
        ends += [(hx, hy), (hx + lf["closed_dir"][0] * lf["width"], hy + lf["closed_dir"][1] * lf["width"])]
    ends.sort(key=lambda p: p[0] * u[0] + p[1] * u[1])
    span = [[round(ends[0][0], 9), round(ends[0][1], 9)], [round(ends[-1][0], 9), round(ends[-1][1], 9)]]
    junction = getattr(layout, "JUNCTION_BOXES", {})
    side_panels = [{"name": p, "lo": list(junction[p][0]), "hi": list(junction[p][1])}
                   for p in d.get("side_panels", [])]
    ceiling = d.get("ceiling", WING_CEILING if src == "wing" else getattr(layout, "BUILDING_CEILING_Z", 2.99))
    return {"name": dname, "rating": rating, "swing": d["swing"], "hold": d["hold"], "leaves": leaves,
            "group": "Doors1" if n == 1 else "Doors", "n_leaves": n, "source_frame": src,
            "tier": d.get("tier", 1), "transom": bool(d.get("transom", False)), "ceiling": ceiling,
            "swing_limit_deg": d.get("swing_limit_deg", 100), "span": span, "side_panels": side_panels,
            "use": d.get("use"), "replaces": list(d.get("replaces", [])), "opening": d.get("opening")}


def _lift_spec(S, lname, fx, cy):
    half = S["door_w"] / 2.0
    pw = half + 0.05  # panel width: 50 mm overlap into the jamb pocket
    panels = []
    for k, sgn in (("a", -1.0), ("b", 1.0)):  # a opens towards -y, b towards +y
        panels.append({
            "joint": f"{PREFIX}{lname}_door_{k}",
            # closed panel centre: leading edges meet at cy, panels sit just on the car side of the front
            "closed_center": [fx + 0.06, cy + sgn * (pw / 2.0), S["door_h"] / 2.0],
            "size": [0.04, pw, S["door_h"]],
            "axis": [0.0, sgn, 0.0],
            "travel": half,
            "mass": S["panel_mass"],
        })
    car = {"x0": fx + 0.12, "x1": fx + 0.12 + S["car_d"], "y0": cy - S["car_w"] / 2.0,
           "y1": cy + S["car_w"] / 2.0, "h": 2.5}
    # Car operating panel on the +y side wall, 0.45 m in from the front return (AD M 3.34: >= 400 mm).
    px = car["x0"] + 0.45
    buttons = [
        {"joint": f"{PREFIX}{lname}_Open", "kind": "open", "lift": lname,
         "pos": [px, car["y1"], 1.05], "axis": [0.0, 1.0, 0.0]},
        {"joint": f"{PREFIX}{lname}_Close", "kind": "close", "lift": lname,
         "pos": [px, car["y1"], 1.15], "axis": [0.0, 1.0, 0.0]},
    ]
    return {"name": lname, "front_x": fx, "center_y": cy, "door_w": S["door_w"],
            "door_h": S["door_h"], "car": car, "panels": panels, "buttons": buttons}


def lift_banks(layout):
    """Lift banks of the layout, wing-local: name -> {lifts, calls, front, lobby, ceiling_z, prefix, source_frame}.

    A layout without LIFT_BANKS (schema 1) has one bank "Wing" from LIFTS, LIFT_CALL and KIT_ROOMS["LiftLobby"].
    """
    banks = getattr(layout, "LIFT_BANKS", None)
    if banks is None:
        lobby = layout.KIT_ROOMS["LiftLobby"]
        fx = next(iter(layout.LIFTS.values()))["front_x"] if layout.LIFTS else lobby[2]
        banks = {"Wing": {"frame": "wing", "lifts": list(layout.LIFTS), "calls": list(layout.LIFT_CALL),
                          "front": (fx, lobby[1], lobby[3]), "ceiling_z": WING_CEILING, "prefix": "",
                          "lobby": tuple(lobby)}}
    out = {}
    for bname, b in banks.items():
        fx, y_lo, y_hi = b["front"]
        x0, y0, x1, y1 = b["lobby"]
        if b["frame"] == "world":
            fx, y_lo = _to_wing_xy(layout, fx, y_lo)
            _, y_hi = _to_wing_xy(layout, 0.0, y_hi)
            x0, y0 = _to_wing_xy(layout, x0, y0)
            x1, y1 = _to_wing_xy(layout, x1, y1)
        out[bname] = {"name": bname, "source_frame": b["frame"], "lifts": list(b["lifts"]), "calls": list(b["calls"]),
                      "front": [fx, y_lo, y_hi], "lobby": [x0, y0, x1, y1], "ceiling_z": b["ceiling_z"],
                      "prefix": b["prefix"]}
    return out


def build_spec(layout, tier=None):
    """Geometry and joint names of every fixture, from the layout module. JSON-serialisable.

    tier: "wing", "tier1", "tier2" or "full"; None reads $HG1_FIXTURES (default "tier2"). See the module docstring.
    """
    tier = resolve_tier(tier)
    excluded = {"doors": [], "lifts": [], "calls": []}

    # -- doors: the wing's FIRE_DOORS (wing-local), then BUILDING_FIRE_DOORS (world frame)
    src_doors = [(n, d, "wing") for n, d in layout.FIRE_DOORS.items()]
    bdoors = getattr(layout, "BUILDING_FIRE_DOORS", {})
    _merge(layout.FIRE_DOORS, bdoors, "FIRE_DOORS / BUILDING_FIRE_DOORS")
    src_doors += [(n, d, d.get("frame", "world")) for n, d in bdoors.items()]
    doors = []
    for dname, d, src in src_doors:
        if not _included(d.get("tier", 1), src, tier):
            excluded["doors"].append(dname)
            continue
        doors.append(_door_spec(layout, dname, d, src))

    # -- lifts and landing calls, per bank
    S = layout.LIFT_SPEC
    blifts = getattr(layout, "BUILDING_LIFTS", {})
    bcalls = getattr(layout, "BUILDING_LIFT_CALL", {})
    _merge(layout.LIFTS, blifts, "LIFTS / BUILDING_LIFTS")
    _merge(layout.LIFT_CALL, bcalls, "LIFT_CALL / BUILDING_LIFT_CALL")
    banks = lift_banks(layout)
    bank_of = {}
    for bname, b in banks.items():
        for n in b["lifts"] + b["calls"]:
            if n in bank_of:
                raise ValueError(f"LIFT_BANKS: {n} is in banks {bank_of[n]} and {bname}")
            bank_of[n] = bname
    lifts = []
    for lname, l, src in ([(n, l, "wing") for n, l in layout.LIFTS.items()]
                          + [(n, l, l.get("frame", "world")) for n, l in blifts.items()]):
        if not _included(l.get("tier", 1), src, tier):
            excluded["lifts"].append(lname)
            continue
        fx, cy = l["front_x"], l["center_y"]
        if src == "world":
            fx, cy = _to_wing_xy(layout, fx, cy)
        ls = _lift_spec(S, lname, fx, cy)
        ls.update({"bank": bank_of.get(lname, "Wing"), "source_frame": src, "tier": l.get("tier", 1)})
        lifts.append(ls)
    present = {l["name"] for l in lifts}
    calls = []
    for cname, c, src in ([(n, c, "wing") for n, c in layout.LIFT_CALL.items()]
                          + [(n, c, c.get("frame", "world")) for n, c in bcalls.items()]):
        served = [n for n in c["lifts"] if n in present]
        if not served or not _included(c.get("tier", 1), src, tier):
            excluded["calls"].append(cname)
            continue
        x, y, z = c["pos"]
        if src == "world":
            x, y = _to_wing_xy(layout, x, y)
        calls.append({"joint": f"{PREFIX}{cname}", "kind": "call", "lifts": served,
                      "pos": [x, y, z], "axis": list(c.get("axis", (1.0, 0.0, 0.0))),  # pressed into the wall
                      "bank": bank_of.get(cname, "Wing"), "source_frame": src, "tier": c.get("tier", 1)})

    static, bank_list = [], []
    for bname, b in banks.items():
        bl = [l for l in lifts if l["bank"] == bname]
        if not bl:
            continue
        boxes = _bank_static_boxes(S, b, bl)
        static += boxes
        b = dict(b)
        b["lifts"] = [l["name"] for l in bl]
        b["calls"] = [c["joint"][len(PREFIX):] for c in calls if c["bank"] == bname]
        b["static_boxes"] = [n for n, _, _ in boxes]
        bank_list.append(b)

    groups = {g: [] for g in GROUPS}
    for d in doors:
        groups[d["group"]].append(d["name"])
    groups["Lifts"] = [l["name"] for l in lifts]
    groups["Calls"] = [c["joint"][len(PREFIX):] for c in calls]
    spec = {
        "frame": "wing", "wing_origin": list(layout.WING_ORIGIN),
        "doors": doors, "lifts": lifts, "calls": calls,
        "lift_spec": dict(S),
        "button": {"size": BUTTON_SIZE, "mass": BUTTON_KG, "travel": BUTTON_TRAVEL},
        "static_boxes": static,
        "schema": SCHEMA, "tier": tier, "groups": groups, "group_dof": dict(GROUP_DOF),
        "lift_banks": bank_list, "excluded": excluded,
    }
    check_unique(spec)
    return spec


def check_unique(spec):
    """Raise ValueError when two doors, lifts, calls, joints or static boxes share a name."""
    def dups(names):
        seen, out = set(), set()
        for n in names:
            (out if n in seen else seen).add(n)
        return sorted(out)

    items = [d["name"] for d in spec["doors"]] + [l["name"] for l in spec["lifts"]] + \
        [c["joint"][len(PREFIX):] for c in spec["calls"]]
    for what, names in (("fixture", items), ("joint", joint_names(spec)),
                        ("static box", [b[0] for b in spec["static_boxes"]]),
                        ("body/geom", joint_names(spec) + [b[0] for b in spec["static_boxes"]])):
        d = dups(names)
        if d:
            raise ValueError(f"duplicate {what} names: {d}")


def lift_static_boxes(layout, lifts):
    """Fixed parts of the lift fronts and cars: (name, lo xyz, hi xyz) wing-local boxes, bank by bank.

    Per bank: the front wall along x = front_x (lobby face) from y_lo to y_hi, with one door opening per lift
    (fx_<prefix>front_i, fx_<prefix>header_i), then each car shell. The front boxes stop at z 3.0; the bulkhead
    above (when the bank's ceiling_z > 3.0) belongs to the scene builders.
    """
    out = []
    for b in lift_banks(layout).values():
        bl = [l for l in lifts if l["name"] in b["lifts"]]
        if bl:
            out += _bank_static_boxes(layout.LIFT_SPEC, b, bl)
    return out


def _bank_static_boxes(S, b, bl):
    """Static boxes of one bank (b from lift_banks()) with its lift specs bl."""
    boxes = []
    fx, y_lo, y_hi = b["front"]
    p = PREFIX + b["prefix"]
    ys = sorted(l["center_y"] for l in bl)
    cuts = [(y - S["door_w"] / 2.0, y + S["door_w"] / 2.0) for y in ys]
    y = y_lo
    for i, (a, c) in enumerate(cuts):
        boxes.append((f"{p}front_{i}", (fx, y, 0.0), (fx + 0.04, a, 3.0)))
        boxes.append((f"{p}header_{i}", (fx, a, S["door_h"]), (fx + 0.04, c, 3.0)))
        y = c
    boxes.append((f"{p}front_{len(cuts)}", (fx, y, 0.0), (fx + 0.04, y_hi, 3.0)))
    for l in bl:
        c, n, t = l["car"], l["name"], 0.05
        boxes += [
            (f"{PREFIX}{n}_car_floor", (c["x0"], c["y0"], -0.05), (c["x1"], c["y1"], 0.0)),
            (f"{PREFIX}{n}_car_roof", (c["x0"], c["y0"], c["h"]), (c["x1"], c["y1"], c["h"] + 0.05)),
            (f"{PREFIX}{n}_car_back", (c["x1"], c["y0"] - t, 0.0), (c["x1"] + t, c["y1"] + t, c["h"])),
            (f"{PREFIX}{n}_car_south", (c["x0"], c["y0"] - t, 0.0), (c["x1"], c["y0"], c["h"])),
            (f"{PREFIX}{n}_car_north", (c["x0"], c["y1"], 0.0), (c["x1"], c["y1"] + t, c["h"])),
            # car front returns either side of the door opening
            (f"{PREFIX}{n}_return_s", (c["x0"] - 0.04, c["y0"], 0.0),
             (c["x0"], l["center_y"] - l["door_w"] / 2.0 - 0.0, c["h"])),
            (f"{PREFIX}{n}_return_n", (c["x0"] - 0.04, l["center_y"] + l["door_w"] / 2.0, 0.0),
             (c["x0"], c["y1"], c["h"])),
            # sill gap filler between the lobby floor and the car floor (no trip step, HBN 00-04 8.9)
            (f"{PREFIX}{n}_sill", (fx, l["center_y"] - l["door_w"] / 2.0, -0.05),
             (c["x0"], l["center_y"] + l["door_w"] / 2.0, 0.0)),
        ]
    return [(n, list(lo), list(hi)) for n, lo, hi in boxes]


# ---------------------------------------------------------------- static building geometry (world frame)


def static_partitions(layout):
    """Static boxes of the main building and the link junction: [(name, lo xyz, hi xyz, kind)], WORLD frame.

    One source for the visuals (build_hospital), the Isaac colliders (make_stage, HG1_Colliders) and the MuJoCo
    world (build_world, class world). Every box is solid (a collider). kind:
      "compartment", "stair", "lobby", "fixed", "shaft", "partition", "wc"  BUILDING_SCREENS pieces "<screen>_<i>"
      "rebuilt"                                    BUILDING_WALL_REBUILDS pieces "<wall>_<i>" (and "<wall>_<i>h")
      "glass"                                      vision panels "<wall>_<i>g" (glass material, 12 mm, solid)
      "junction"                                   JUNCTION_BOXES "Junction_<name>" (collide=True only)
    Screens run from z 0 to BUILDING_CEILING_Z; over each opening a head piece runs from the opening's head_z.
    Rebuilt walls: door openings get a head from REBUILD_DOOR_HEAD; vision openings get a sill to VISION_Z[0],
    the glass, and a head from VISION_Z[1].
    """
    out = []
    zc = getattr(layout, "BUILDING_CEILING_Z", 2.99)
    for n, s in getattr(layout, "BUILDING_SCREENS", {}).items():
        x0, y0, x1, y1 = s["line"]
        t = s["t"]
        vertical = abs(x0 - x1) < 1e-9
        if not vertical and abs(y0 - y1) > 1e-9:
            raise ValueError(f"screen {n} is not axis-aligned")
        lo, hi = (min(y0, y1), max(y0, y1)) if vertical else (min(x0, x1), max(x0, x1))
        pieces, cur = [], lo
        for a, b, head in sorted(s["openings"]):
            if a > cur + 1e-6:
                pieces.append((cur, a, 0.0, zc))
            pieces.append((a, b, head, zc))  # head over the opening
            cur = b
        if hi > cur + 1e-6:
            pieces.append((cur, hi, 0.0, zc))
        pieces = [pc for pc in pieces if pc[3] - pc[2] > 1e-3]
        for i, (a, b, z0, z1) in enumerate(pieces):
            if vertical:
                box = ([x0 - t / 2, a, z0], [x0 + t / 2, b, z1])
            else:
                box = ([a, y0 - t / 2, z0], [b, y0 + t / 2, z1])
            out.append((f"{n}_{i}", box[0], box[1], s["kind"]))
    head = getattr(layout, "REBUILD_DOOR_HEAD", 2.10)
    vz = getattr(layout, "VISION_Z", (0.90, 2.00))
    for n, w in getattr(layout, "BUILDING_WALL_REBUILDS", {}).items():
        x0, y0, x1, y1 = w["box"]
        cur, i = x0, 0
        for kind, a, b in sorted(w["openings"], key=lambda o: o[1]):
            out.append((f"{n}_{i}", [cur, y0, 0.0], [a, y1, zc], "rebuilt"))
            i += 1
            if kind == "door":
                out.append((f"{n}_{i}", [a, y0, head], [b, y1, zc], "rebuilt"))
            else:
                ym = (y0 + y1) / 2
                out.append((f"{n}_{i}", [a, y0, 0.0], [b, y1, vz[0]], "rebuilt"))
                out.append((f"{n}_{i}g", [a, ym - 0.006, vz[0]], [b, ym + 0.006, vz[1]], "glass"))
                out.append((f"{n}_{i}h", [a, y0, vz[1]], [b, y1, zc], "rebuilt"))
            i += 1
            cur = b
        out.append((f"{n}_{i}", [cur, y0, 0.0], [x1, y1, zc], "rebuilt"))
    ox, oy, oz = layout.WING_ORIGIN
    for n, (lo, hi, collide) in getattr(layout, "JUNCTION_BOXES", {}).items():
        if collide:
            out.append((f"Junction_{n}", [round(lo[0] + ox, 9), round(lo[1] + oy, 9), lo[2] + oz],
                        [round(hi[0] + ox, 9), round(hi[1] + oy, 9), hi[2] + oz], "junction"))
    names = [b[0] for b in out]
    if len(names) != len(set(names)):
        raise ValueError("static_partitions: duplicate names")
    return out


def building_hide(layout, spec):
    """Building prims to deactivate (names under /World/Hospital), for the doors that spec really has.

    BUILDING_HIDE lists every prim. The static leaves that a door of an excluded tier would replace stay active, so
    the doorway keeps its old leaves (for example D07 and D34 stay closed double doors below tier 2).
    """
    keep = set()
    bdoors = getattr(layout, "BUILDING_FIRE_DOORS", {})
    for n in spec["excluded"]["doors"]:
        if n in bdoors:
            keep |= set(bdoors[n].get("replaces", []))
    out, seen = [], set()
    for names in getattr(layout, "BUILDING_HIDE", {}).values():
        for n in names:
            if n not in keep and n not in seen:
                seen.add(n)
                out.append(n)
    return out


def building_openings(layout, spec):
    """BUILDING_OPENINGS (world rects of MuJoCo wall cuts) without the cuts of doors that spec leaves out."""
    bdoors = getattr(layout, "BUILDING_FIRE_DOORS", {})
    skip = {bdoors[n]["opening"] for n in spec["excluded"]["doors"] if n in bdoors and bdoors[n].get("opening")}
    return {k: v for k, v in getattr(layout, "BUILDING_OPENINGS", {}).items() if k not in skip}


def to_wing(spec, x, y, z=0.0):
    ox, oy, oz = spec["wing_origin"]
    return (x - ox, y - oy, z - oz)


def to_world(spec, x, y, z=0.0):
    ox, oy, oz = spec["wing_origin"]
    return (x + ox, y + oy, z + oz)


def leaf_box(leaf, angle):
    """Leaf box at a hinge angle: (centre xyz wing-local, yaw rad, size xyz along the leaf)."""
    d = _rot(leaf["closed_dir"], angle)
    hx, hy = leaf["hinge"]
    r = 0.005 + leaf["width"] / 2.0
    return ((hx + d[0] * r, hy + d[1] * r, leaf["height"] / 2.0 + 0.01), leaf["closed_yaw"] + angle,
            (leaf["width"], leaf["thickness"], leaf["height"]))


def joint_names(spec):
    names = [lf["joint"] for d in spec["doors"] for lf in d["leaves"]]
    for l in spec["lifts"]:
        names += [p["joint"] for p in l["panels"]] + [b["joint"] for b in l["buttons"]]
    names += [c["joint"] for c in spec["calls"]]
    return names


def curtain_rays(spec, lift_name, q, n=8):
    """Light-curtain rays across the gap between the lift panels' leading edges, wing-local.

    Returns [(origin xyz, unit direction xyz, length)]. Empty when the gap is under 20 mm (the curtain
    is muted in the last 20 mm of closing, BS EN 81-20, secondary source).
    """
    lift = next(l for l in spec["lifts"] if l["name"] == lift_name)
    pa, pb = lift["panels"]
    ya = lift["center_y"] - q.get(pa["joint"], 0.0)
    yb = lift["center_y"] + q.get(pb["joint"], 0.0)
    gap = yb - ya
    if gap < 0.02:
        return []
    z0, z1 = spec["lift_spec"]["curtain_z"]
    x = lift["front_x"] + 0.06
    return [((x, ya + 0.005, z0 + (z1 - z0) * i / (n - 1)), (0.0, 1.0, 0.0), gap - 0.01) for i in range(n)]


# ---------------------------------------------------------------- controller


class _Lift:
    def __init__(self, spec, L):
        self.spec, self.L = spec, L
        self.name = spec["name"]
        self.travel = spec["panels"][0]["travel"]
        self.state = "closed"   # closed | opening | open | closing
        self.u = 0.0            # motion phase 0 closed .. 1 open; position = travel * (1 - cos(pi u)) / 2
        self.pos_ref = 0.0
        self.timer = 0.0        # dwell time left while open
        self.stall_t = 0.0
        self.reopen_count = 0
        self.curtain_streak = 0  # light-curtain reopens since the doors last closed fully
        self.blocked_t = 0.0     # time the curtain has been blocked since the doors last closed
        self.nudging = False

    def request_open(self, dwell=None):
        if self.state in ("closed", "closing"):
            if self.state == "closing":
                self.reopen_count += 1
            self.state = "opening"
        if self.state == "open":
            self.timer = max(self.timer, self.L["dwell_s"] if dwell is None else dwell)

    def request_close(self):
        if self.state == "open":
            self.timer = min(self.timer, self.L["dwell_short_s"])


class FixtureController:
    """Closers, magnets, lift door drives and buttons. Call `step()` once per physics step.

    Works on any spec of build_spec(): every door (1 or 2 leaves), every lift of every bank and every call button.
    One building-wide fire alarm: set_alarm(True) releases all magnets and puts all lifts on fire recall.
    """

    def __init__(self, spec):
        self.spec = spec
        self.alarm = False
        self.leaves = [lf for d in spec["doors"] for lf in d["leaves"]]
        self.lifts = {l["name"]: _Lift(l, spec["lift_spec"]) for l in spec["lifts"]}
        self.buttons = [b for l in spec["lifts"] for b in l["buttons"]] + list(spec["calls"])
        self._down = {b["joint"]: False for b in self.buttons}
        self.events = []   # (time, text), newest last, trimmed
        self.t = 0.0
        self._next_call = {}  # call joint -> round-robin index into its lifts (per bank)

    # -- inputs
    def set_alarm(self, on):
        on = bool(on)
        if on != self.alarm:
            self.alarm = on
            self._log("fire alarm " + ("ON: magnets released, lifts on fire recall" if on else "reset"))
            for lift in self.lifts.values():
                if on:
                    lift.request_open()
                elif lift.state == "open":
                    lift.timer = lift.L["dwell_s"]

    def press(self, joint_or_name):
        """Virtual button press (also used by services). Accepts the joint name or the short name."""
        name = joint_or_name if joint_or_name.startswith(PREFIX) else PREFIX + joint_or_name
        b = next((b for b in self.buttons if b["joint"] == name), None)
        if b is None:
            raise KeyError(joint_or_name)
        self._pressed(b)

    def initial_q(self):
        """Joint positions at reset: held doors open on their magnets, everything else closed."""
        q = {j: 0.0 for j in joint_names(self.spec)}
        for lf in self.leaves:
            if lf["hold"] is not None:
                q[lf["joint"]] = lf["hold"]
        return q

    def reset(self):
        self.alarm = False
        self.t = 0.0
        self._next_call = {}
        for n, l in list(self.lifts.items()):
            self.lifts[n] = _Lift(l.spec, l.L)
        self._down = {k: False for k in self._down}

    # -- the step
    def step(self, dt, q, qd, curtain_blocked=None):
        """q, qd: dicts joint -> position / velocity. curtain_blocked: dict lift -> bool.

        Returns dict joint -> effort (N*m or N), to add as a generalised force on that joint.
        """
        self.t += dt
        curtain_blocked = curtain_blocked or {}
        tau = {}
        for lf in self.leaves:
            tau[lf["joint"]] = self._leaf(lf, q.get(lf["joint"], 0.0), qd.get(lf["joint"], 0.0))
        for b in self.buttons:
            j = b["joint"]
            s, v = q.get(j, 0.0), qd.get(j, 0.0)
            tau[j] = -(BUTTON_F0 + BUTTON_K * max(s, 0.0)) - BUTTON_C * v
            if not self._down[j] and s > BUTTON_TRIGGER:
                self._down[j] = True
                self._pressed(b)
            elif self._down[j] and s < BUTTON_RELEASE:
                self._down[j] = False
        for lift in self.lifts.values():
            tau.update(self._lift(lift, dt, q, qd, curtain_blocked.get(lift.name, False)))
        return tau

    def _leaf(self, lf, a, w):
        if lf["bolted"]:
            return -BOLT_K * a - BOLT_C * w
        t = -(CLOSER_T0 * math.tanh(a / CLOSER_EPS) + CLOSER_K * a) - CLOSER_C0 * w
        if a * w < 0:  # moving towards closed: hydraulic check
            t -= CLOSER_C * w
        h = lf["hold"]
        if h is not None and not self.alarm and abs(a - h) < MAGNET_CAPTURE:
            t += max(-MAGNET_T, min(MAGNET_T, MAGNET_K * (h - a)))
        return t

    def _lift(self, lift, dt, q, qd, blocked):
        L = lift.L
        pa, pb = lift.spec["panels"]
        s = 0.5 * (q.get(pa["joint"], 0.0) + q.get(pb["joint"], 0.0))
        v = 0.5 * (qd.get(pa["joint"], 0.0) + qd.get(pb["joint"], 0.0))
        T = lift.travel
        # smooth (cosine) motion profile over open_s / close_s; reversals keep the reference position
        du = 0.0
        if lift.state == "opening":
            du = dt / L["open_s"]
            lift.u = min(1.0, lift.u + du)
        elif lift.state == "closing":
            du = -dt / (L["close_s"] * (NUDGE_SLOWDOWN if lift.nudging else 1.0))
            lift.u = max(0.0, lift.u + du)
        lift.pos_ref = T * (1.0 - math.cos(math.pi * lift.u)) / 2.0
        vref = T * math.pi / 2.0 * math.sin(math.pi * lift.u) * (du / dt if dt > 0 else 0.0)
        if lift.state == "opening":
            if lift.u >= 1.0 and s > T - 0.01:
                lift.state, lift.timer = "open", L["dwell_s"]
                self._log(f"{lift.name} doors open")
        elif lift.state == "open":
            if blocked:
                lift.blocked_t += dt
                if lift.blocked_t < NUDGE_BLOCKED_S:
                    lift.timer = max(lift.timer, L["dwell_short_s"] + 1.0)
            if not self.alarm:
                lift.timer -= dt
                if lift.timer <= 0:
                    lift.state = "closing"
                    if (lift.curtain_streak >= NUDGE_REOPENS or lift.blocked_t >= NUDGE_BLOCKED_S) \
                            and not lift.nudging:
                        lift.nudging = True
                        self._log(f"{lift.name} nudging: slow close, light curtain ignored")
        elif lift.state == "closing":
            stalled = s - lift.pos_ref > STALL_ERR and abs(v) < STALL_V
            lift.stall_t = lift.stall_t + dt if stalled else 0.0
            curtain = blocked and not lift.nudging
            if curtain or lift.stall_t > STALL_S:
                self._log(f"{lift.name} reopens ({'light curtain' if curtain else 'door stalled'})")
                if curtain:
                    lift.curtain_streak += 1
                lift.stall_t = 0.0
                lift.nudging = False
                lift.request_open()
            elif lift.u <= 0.0 and s < 0.005:
                lift.state = "closed"
                lift.curtain_streak, lift.blocked_t, lift.nudging = 0, 0.0, False
                self._log(f"{lift.name} doors closed")
        out = {}
        for p in (pa, pb):
            j = p["joint"]
            f = DOOR_KP * (lift.pos_ref - q.get(j, 0.0)) + DOOR_KD * (vref - qd.get(j, 0.0))
            if lift.state == "closing" or (lift.state == "closed" and f < 0):
                lim_lo = -FORCE_MARGIN * L["stall_n"]
            else:
                lim_lo = -DOOR_OPEN_F
            out[j] = max(lim_lo, min(DOOR_OPEN_F, f))
        return out

    def _pressed(self, b):
        self._log(f"button {b['joint'][len(PREFIX):]} pressed")
        if self.alarm:
            return  # fire recall: landing and car calls are ignored
        if b["kind"] == "open":
            self.lifts[b["lift"]].request_open()
        elif b["kind"] == "close":
            self.lifts[b["lift"]].request_close()
        elif b["kind"] == "call":
            group = [self.lifts[n] for n in b["lifts"]]
            for lift in group:  # a car already at the landing answers the call
                if lift.state in ("open", "opening", "closing"):
                    lift.request_open()
                    return
            k = self._next_call.get(b["joint"], 0)
            self._next_call[b["joint"]] = k + 1
            group[k % len(group)].request_open()

    def _log(self, text):
        self.events.append((round(self.t, 3), text))
        del self.events[:-50]

    # -- outputs
    def state(self, q):
        doors = {}
        for d in self.spec["doors"]:
            leaves = {lf["joint"]: round(math.degrees(q.get(lf["joint"], 0.0)), 1) for lf in d["leaves"]}
            held = [lf for lf in d["leaves"] if lf["hold"] is not None]
            doors[d["name"]] = {
                "rating": d["rating"], "leaves_deg": leaves,
                "closed": all(abs(v) < 2.0 for v in leaves.values()),
                "held_open": bool(held) and not self.alarm and all(
                    abs(q.get(lf["joint"], 0.0) - lf["hold"]) < MAGNET_CAPTURE for lf in held),
            }
        lifts = {}
        for n, lift in self.lifts.items():
            pa, pb = lift.spec["panels"]
            gap = q.get(pa["joint"], 0.0) + q.get(pb["joint"], 0.0)
            lifts[n] = {"state": lift.state, "door_gap_m": round(gap, 3), "reopens": lift.reopen_count, "nudging": lift.nudging,
                        "dwell_left_s": round(max(lift.timer, 0.0), 2) if lift.state == "open" else 0.0}
        return {"t": round(self.t, 3), "fire_alarm": self.alarm, "doors": doors, "lifts": lifts,
                "events": self.events[-5:]}

    def state_json(self, q):
        return json.dumps(self.state(q))


def load_layout(path):
    import importlib.util
    spec = importlib.util.spec_from_file_location("layout", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod
