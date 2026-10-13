"""Checks the fixture spec and controller against the UK guidance values in layout.py, with simple joint dynamics.

The spec is built at tier "full": the surgical wing plus every main-building door, both lift banks and both call
buttons (22 doors, 4 lifts, 2 calls, 52 joints). test_wing_regression.py checks that the wing items do not change.

    python3 -m pytest hospital-sim-env/fixtures -q
"""

import json
import math
import os

import pytest

import hospital_fixtures as F

HERE = os.path.dirname(os.path.abspath(__file__))
LAYOUT = F.load_layout(os.path.join(HERE, "..", "isaac", "scene", "layout.py"))
DT = 0.002


@pytest.fixture
def spec():
    return F.build_spec(LAYOUT, "full")


class Sim:
    """Each joint as an independent 1-DOF body with joint limits (enough to test the control laws)."""

    def __init__(self, spec):
        self.spec = spec
        self.ctl = F.FixtureController(spec)
        self.q = self.ctl.initial_q()
        self.qd = {j: 0.0 for j in self.q}
        self.m, self.lim = {}, {}
        for d in spec["doors"]:
            for lf in d["leaves"]:
                self.m[lf["joint"]] = lf["inertia_z"]
                self.lim[lf["joint"]] = tuple(lf["range"])
        for l in spec["lifts"]:
            for p in l["panels"]:
                self.m[p["joint"]] = p["mass"]
                self.lim[p["joint"]] = (0.0, p["travel"])
            for b in l["buttons"]:
                self.m[b["joint"]] = spec["button"]["mass"]
                self.lim[b["joint"]] = (0.0, spec["button"]["travel"])
        for c in spec["calls"]:
            self.m[c["joint"]] = spec["button"]["mass"]
            self.lim[c["joint"]] = (0.0, spec["button"]["travel"])
        self.ext = {}
        self.blocked = {}
        self.block_at = {}  # joint -> position it cannot pass (an obstacle)

    def run(self, seconds):
        for _ in range(int(round(seconds / DT))):
            tau = self.ctl.step(DT, self.q, self.qd, self.blocked)
            for j, t in tau.items():
                a = (t + self.ext.get(j, 0.0)) / self.m[j]
                self.qd[j] += a * DT
                self.q[j] += self.qd[j] * DT
                lo, hi = self.lim[j]
                if j in self.block_at:
                    lo = max(lo, self.block_at[j])
                if self.q[j] < lo or self.q[j] > hi:
                    self.q[j] = min(max(self.q[j], lo), hi)
                    self.qd[j] = 0.0

    def lift(self, n):
        return self.ctl.state(self.q)["lifts"][n]


def leaf(spec, door, i=0):
    return next(d for d in spec["doors"] if d["name"] == door)["leaves"][i]


def test_spec_is_json_and_names_are_prefixed(spec):
    json.dumps(spec)
    names = F.joint_names(spec)
    assert len(names) == len(set(names))
    assert all(n.startswith(F.PREFIX) for n in names)
    assert len(spec["doors"]) == 22 and len(spec["lifts"]) == 4 and len(spec["calls"]) == 2
    assert len(names) == 52
    boxes = [b[0] for b in spec["static_boxes"]]
    assert len(boxes) == len(set(boxes)) and not set(boxes) & set(names)
    assert all(b.startswith(F.PREFIX) for b in boxes)


def test_lift_dimensions_follow_htm_08_02(spec):
    for l in spec["lifts"]:
        c = l["car"]
        assert c["y1"] - c["y0"] == pytest.approx(1.8)   # HTM 08-02 4.19
        assert c["x1"] - c["x0"] == pytest.approx(2.7)
        assert l["door_w"] >= 1.37 and l["door_h"] >= 2.1  # HBN 00-04 6.30
        # leading edges meet in the middle when closed, open fully to the door width
        pa, pb = l["panels"]
        assert pa["travel"] + pb["travel"] == pytest.approx(l["door_w"])
    for call in spec["calls"]:
        assert 0.9 <= call["pos"][2] <= 1.1                 # AD M vol 2 3.34
    for l in spec["lifts"]:
        for b in l["buttons"]:
            assert 0.9 <= b["pos"][2] <= 1.2


def test_double_swing_and_single_swing_ranges(spec):
    lo, hi = leaf(spec, "CrossCorridor")["range"]
    assert lo < 0 < hi
    # lobby door opens towards the corridor (-x), the direction of escape
    lf = leaf(spec, "LiftLobby")
    box, _, _ = F.leaf_box(lf, lf["hold"])
    assert box[0] < 14.0
    # store door opens into the store (-y)
    lf = leaf(spec, "Store")
    a = lf["range"][1] if lf["range"][1] > 0 else lf["range"][0]
    assert F.leaf_box(lf, a)[0][1] < -10.0


def test_opening_force_within_approved_document_m(spec):
    for d in spec["doors"]:
        for lf in d["leaves"]:
            if lf["bolted"]:
                continue
            for deg in range(1, 61):
                a = math.radians(deg)
                tq = F.CLOSER_T0 * math.tanh(a / F.CLOSER_EPS) + F.CLOSER_K * a
                limit = 30.0 if deg <= 30 else 22.5  # AD M vol 2 3.10(a)
                assert tq / lf["lever"] <= limit, (lf["joint"], deg)


def test_self_closing_from_90_deg_takes_several_seconds(spec):
    sim = Sim(spec)
    j = leaf(spec, "Store")["joint"]
    sim.q[j] = -math.radians(90)
    t = 0.0
    while abs(sim.q[j]) > math.radians(2) and t < 30:
        sim.run(0.1)
        t += 0.1
    assert 3.0 < t < 10.0, t
    sim.run(3.0)
    assert abs(sim.q[j]) < math.radians(1)


def test_magnets_hold_until_alarm_then_doors_close(spec):
    sim = Sim(spec)
    sim.run(5.0)
    st = sim.ctl.state(sim.q)["doors"]
    assert st["CrossCorridor"]["held_open"] and st["Compartment"]["held_open"] and st["LiftLobby"]["held_open"]
    assert st["Store"]["closed"]
    sim.ctl.set_alarm(True)
    sim.run(15.0)
    st = sim.ctl.state(sim.q)["doors"]
    assert all(d["closed"] for d in st.values()), st


def test_hard_pull_releases_a_magnet(spec):
    sim = Sim(spec)
    lf = leaf(spec, "CrossCorridor")
    j = lf["joint"]
    sim.ext[j] = -math.copysign(120.0 * lf["lever"], lf["hold"])  # 120 N towards closed
    sim.run(0.5)
    sim.ext[j] = 0.0
    sim.run(15.0)
    assert abs(sim.q[j]) < math.radians(2)


def test_bolted_leaf_stays_shut(spec):
    sim = Sim(spec)
    j = leaf(spec, "Store", 1)["joint"]
    sim.ext[j] = 25.0
    sim.run(2.0)
    assert abs(sim.q[j]) < math.radians(1)


def press(sim, joint, seconds=0.3):
    sim.ext[joint] = 6.0  # a 6 N push, above the 5 N full-travel force
    sim.run(seconds)
    sim.ext[joint] = 0.0


def test_call_button_opens_a_lift_with_guidance_timing(spec):
    sim = Sim(spec)
    press(sim, F.PREFIX + "LobbyCall")
    t = 0.0
    while sim.lift("Lift1")["state"] != "open" and t < 10:
        sim.run(0.05)
        t += 0.05
    assert 2.5 < t + 0.3 < 4.5, t     # opening time near 3.0 s
    assert sim.lift("Lift2")["state"] == "closed"
    assert sim.lift("Lift1")["door_gap_m"] == pytest.approx(1.4, abs=0.02)
    sim.run(6.5)
    assert sim.lift("Lift1")["state"] == "open"  # 7 s dwell (HTM 08-02 5.32)
    sim.run(6.0)
    assert sim.lift("Lift1")["state"] == "closed"


def test_close_button_shortens_dwell(spec):
    sim = Sim(spec)
    sim.ctl.press("Lift2_Open")
    sim.run(4.0)
    assert sim.lift("Lift2")["state"] == "open"
    press(sim, F.PREFIX + "Lift2_Close")
    sim.run(1.0)
    assert sim.lift("Lift2")["state"] == "closing"


def test_light_curtain_and_stall_reopen_the_doors(spec):
    sim = Sim(spec)
    sim.ctl.press("Lift1_Open")
    sim.run(12.0)
    assert sim.lift("Lift1")["state"] == "closing"
    sim.blocked = {"Lift1": True}
    sim.run(0.1)
    assert sim.lift("Lift1")["state"] == "opening"
    sim.blocked = {}
    while sim.lift("Lift1")["state"] != "closing":
        sim.run(0.05)
    # an obstacle stops the panels 0.3 m before closed: the stall detector reopens
    for p in spec["lifts"][0]["panels"]:
        sim.block_at[p["joint"]] = 0.3
    t = 0.0
    while sim.lift("Lift1")["state"] != "opening" and t < 10:
        sim.run(0.05)
        t += 0.05
    assert sim.lift("Lift1")["state"] == "opening"
    assert sim.lift("Lift1")["reopens"] >= 2


def test_closing_force_and_kinetic_energy_limits(spec):
    sim = Sim(spec)
    sim.ctl.press("Lift1_Open")
    sim.run(11.0)
    peak_ke, peak_f = 0.0, 0.0
    for _ in range(int(4.0 / DT)):
        tau = sim.ctl.step(0.0, sim.q, sim.qd)
        ke = sum(0.5 * p["mass"] * sim.qd[p["joint"]] ** 2 for p in spec["lifts"][0]["panels"])
        peak_ke = max(peak_ke, ke)
        if sim.lift("Lift1")["state"] == "closing":
            peak_f = max(peak_f, max(-tau[p["joint"]] for p in spec["lifts"][0]["panels"]))
        sim.run(DT)
    assert peak_ke < 10.0                       # BS EN 81-20 / HTM 08-02 App C
    assert peak_f < 150.0


def test_fire_recall_opens_lifts_and_ignores_calls(spec):
    sim = Sim(spec)
    sim.ctl.set_alarm(True)
    sim.run(5.0)
    assert sim.lift("Lift1")["state"] == "open" and sim.lift("Lift2")["state"] == "open"
    sim.ctl.press("Lift1_Close")
    sim.run(20.0)
    assert sim.lift("Lift1")["state"] == "open"
    sim.ctl.set_alarm(False)
    sim.run(12.0)
    assert sim.lift("Lift1")["state"] == "closed"


def test_curtain_rays_span_the_gap(spec):
    ctl = F.FixtureController(spec)
    q = ctl.initial_q()
    assert F.curtain_rays(spec, "Lift1", q) == []
    for p in spec["lifts"][0]["panels"]:
        q[p["joint"]] = 0.3
    rays = F.curtain_rays(spec, "Lift1", q)
    assert len(rays) == 8 and rays[0][2] == pytest.approx(0.59)


def test_nudging_after_repeated_curtain_reopens(spec):
    sim = Sim(spec)
    sim.ctl.press("Lift1_Open")
    for _ in range(F.NUDGE_REOPENS):
        while sim.lift("Lift1")["state"] != "closing":
            sim.run(0.05)
        sim.blocked = {"Lift1": True}
        sim.run(0.1)
        assert sim.lift("Lift1")["state"] in ("opening", "open")
        sim.blocked = {}
    while sim.lift("Lift1")["state"] != "closing":
        sim.run(0.05)
    assert sim.lift("Lift1")["nudging"]
    sim.blocked = {"Lift1": True}          # the curtain is ignored while nudging
    peak_ke = 0.0
    while sim.lift("Lift1")["state"] == "closing":
        sim.run(0.02)
        ke = sum(0.5 * p["mass"] * sim.qd[p["joint"]] ** 2 for p in spec["lifts"][0]["panels"])
        peak_ke = max(peak_ke, ke)
    assert sim.lift("Lift1")["state"] == "closed"
    assert peak_ke < 4.0                    # BS EN 81-20 nudging limit (secondary source)


# ---------------------------------------------------------------- whole hospital (schema 2)

OX, OY, _ = LAYOUT.WING_ORIGIN
# Swing side of every doorset (design_final.md s.4.3; world and wing axes are parallel).
SWING = {
    "Compartment": "both", "CrossCorridor": "both", "LiftLobby": "-x", "Store": "-y", "ScrubStore": "-x",
    "Theatre": "both", "FinalExit": "-y",
    "W_Compartment": "both", "E_Compartment": "both", "W_Stair": "-y", "E_Stair": "-y", "Main_LiftLobby": "+y",
    "D07_Exit": "+y", "D34_Exit": "+x", "AE_DirtyUtility": "+x", "AE_CleanLinen": "+x", "Central_Store": "+y",
    "Pharmacy1": "-y", "Pharmacy2": "-y", "Ward_Cleaners": "-x", "Ward_CleanLinen": "-x", "Ward_DirtyUtility": "-x",
}
# Doors held open on fail-safe magnets (HTM 05-02 App C, HBN 00-04 8.26), with the side they are held on.
HELD = {"Compartment": "-y", "CrossCorridor": "+x", "LiftLobby": "-x", "Theatre": "-y",
        "W_Compartment": "+x", "E_Compartment": "+x", "Main_LiftLobby": "+y"}
# Fire-hazard room doors open into their room (HTM 05-02 5.40, Table 6): door -> BUILDING_ROOMS name.
# The ward's dirty-utility corridor door opens into the dirty utility (east part of old room W5, next to the corridor);
# the disposal hold (west part, window end) is behind the open doorway in partition DU_Disposal (design s.6.1).
INTO_ROOM = {"AE_DirtyUtility": "AE_DirtyUtility", "AE_CleanLinen": "AE_CleanUtility", "Central_Store": "Central_Store",
             "Ward_Cleaners": "Ward_Cleaners", "Ward_CleanLinen": "Ward_CleanUtility",
             "Ward_DirtyUtility": "Ward_DirtyUtility"}
DIRS = {"+x": (1, 0), "-x": (-1, 0), "+y": (0, 1), "-y": (0, -1)}


def door(spec, name):
    return next(d for d in spec["doors"] if d["name"] == name)


def open_limits(lf):
    return [a for a in lf["range"] if abs(a) > 1e-9]


def leaf_offset(lf, angle):
    """Leaf centre minus hinge, at a hinge angle."""
    c, _, _ = F.leaf_box(lf, angle)
    return (c[0] - lf["hinge"][0], c[1] - lf["hinge"][1])


def test_every_door_is_listed_with_the_design_swing(spec):
    assert {d["name"] for d in spec["doors"]} == set(SWING)
    for d in spec["doors"]:
        assert d["swing"] == SWING[d["name"]], d["name"]
        for lf in d["leaves"]:
            lims = open_limits(lf)
            if d["swing"] == "both":
                assert len(lims) == 2 and lims[0] < 0 < lims[1], lf["joint"]
                continue
            assert len(lims) == 1, lf["joint"]
            off = leaf_offset(lf, lims[0])
            v = DIRS[d["swing"]]
            assert off[0] * v[0] + off[1] * v[1] > 0.45 * lf["width"], (lf["joint"], off)  # on the swing side


def test_groups_are_homogeneous_for_isaac_tensor_views(spec):
    g = spec["groups"]
    assert len(g["Doors"]) == 12 and len(g["Doors1"]) == 10 and len(g["Lifts"]) == 4 and len(g["Calls"]) == 2
    for d in spec["doors"]:
        assert d["n_leaves"] == len(d["leaves"]) == spec["group_dof"][d["group"]]
        assert d["name"] in g[d["group"]]
        assert [lf["joint"] for lf in d["leaves"]] == [f"fx_{d['name']}_leaf{i}" for i in range(d["n_leaves"])]
    assert g["Doors1"] == [d["name"] for d in spec["doors"] if d["n_leaves"] == 1]
    assert g["Calls"] == ["LobbyCall", "MainCall"] and g["Lifts"] == ["Lift1", "Lift2", "MainLift1", "MainLift2"]
    for l in spec["lifts"]:
        assert len(l["panels"]) + len(l["buttons"]) == spec["group_dof"]["Lifts"]


def test_world_to_wing_round_trip(spec):
    for name, d in LAYOUT.BUILDING_FIRE_DOORS.items():
        for (hinge, _, w), lf in zip(d["leaves"], door(spec, name)["leaves"]):
            x, y, _ = F.to_world(spec, *lf["hinge"])
            assert (x, y) == pytest.approx(hinge, abs=1e-9) and lf["width"] == w
            assert F.to_wing(spec, x, y)[:2] == pytest.approx(lf["hinge"], abs=1e-9)
        assert door(spec, name)["source_frame"] == "world"
    for name, l in LAYOUT.BUILDING_LIFTS.items():
        s = next(x for x in spec["lifts"] if x["name"] == name)
        assert F.to_world(spec, s["front_x"], s["center_y"])[:2] == pytest.approx((l["front_x"], l["center_y"]))
    c = next(x for x in spec["calls"] if x["joint"] == "fx_MainCall")
    assert F.to_world(spec, *c["pos"]) == pytest.approx(LAYOUT.BUILDING_LIFT_CALL["MainCall"]["pos"])


def test_held_doors_and_kept_shut_doors(spec):
    for d in spec["doors"]:
        assert d["hold"] == HELD.get(d["name"]), d["name"]
        for lf in d["leaves"]:
            if d["hold"] is None or lf["bolted"]:
                assert lf["hold"] is None, lf["joint"]
                continue
            assert abs(lf["hold"]) == pytest.approx(math.pi / 2)
            assert lf["range"][0] <= lf["hold"] <= lf["range"][1]
            off = leaf_offset(lf, lf["hold"])
            v = DIRS[d["hold"]]
            assert off[0] * v[0] + off[1] * v[1] > 0.45 * lf["width"], lf["joint"]
    held = [lf for d in spec["doors"] for lf in d["leaves"] if lf["hold"] is not None]
    assert len(held) == 14


def test_stair_doors_are_never_held_and_open_out_of_the_stair(spec):
    for name, screen_y in (("W_Stair", 14.45), ("E_Stair", 10.40)):
        d = door(spec, name)
        assert d["hold"] is None and d["rating"] == "FD30S" and d["group"] == "Doors1"   # HTM 05-02 App C
        lf = d["leaves"][0]
        assert lf["hold"] is None
        c, _, _ = F.leaf_box(lf, open_limits(lf)[0])
        assert c[1] + OY < screen_y - 0.4  # the stair is north of its screen: the leaf opens south, out of it


def test_final_exits_open_outwards(spec):
    # D07 is on the north facade (outside is +y), D34 on the east facade (+x), the wing exit on y = -10 (-y).
    for name, axis, sign, line in (("D07_Exit", 1, 1, 32.881), ("D34_Exit", 0, 1, 20.81), ("FinalExit", 1, -1, None)):
        d = door(spec, name)
        assert d["hold"] is None and d["rating"] == "EXIT"
        for lf in d["leaves"]:
            c, _, _ = F.leaf_box(lf, open_limits(lf)[0])
            world = (c[0] + OX, c[1] + OY)
            ref = line if line is not None else -10.0 + OY
            assert sign * (world[axis] - ref) > 0.4, lf["joint"]


def test_hazard_room_doors_open_into_the_room(spec):
    for name, room in INTO_ROOM.items():
        lf = door(spec, name)["leaves"][0]
        assert lf["hold"] is None and door(spec, name)["rating"] == "FD30S"
        c, _, _ = F.leaf_box(lf, open_limits(lf)[0])
        x0, y0, x1, y1 = LAYOUT.BUILDING_ROOMS[room]["rect"]
        assert x0 < c[0] + OX < x1 and y0 < c[1] + OY < y1, (name, room)
        assert "shut" in door(spec, name)["use"] or "self-closing" in door(spec, name)["use"]


def test_swing_limits_in_building_jambs(spec):
    for d in spec["doors"]:
        lim = math.radians(d["swing_limit_deg"])
        for lf in d["leaves"]:
            assert max(abs(a) for a in lf["range"]) == pytest.approx(lim)
        if d["source_frame"] == "world" and d["name"] not in ("W_Compartment", "E_Compartment", "W_Stair",
                                                               "E_Stair", "Main_LiftLobby"):
            assert d["swing_limit_deg"] == 92  # leaves in building jambs (not collision-filtered)


def test_opening_force_covers_every_leaf_width(spec):
    widths = sorted({lf["width"] for d in spec["doors"] for lf in d["leaves"] if not lf["bolted"]})
    assert widths == [0.955, 1.0, 1.19, 1.2, 1.2085, 1.212]  # 0.8 m Store leaf is flush-bolted
    for w in widths:
        lever = w - F.HANDLE_INSET
        for deg in range(1, 91):
            a = math.radians(deg)
            tq = F.CLOSER_T0 * math.tanh(a / F.CLOSER_EPS) + F.CLOSER_K * a
            if deg <= 60:
                assert tq / lever <= (30.0 if deg <= 30 else 22.5), (w, deg)  # AD M vol 2 3.10(a)
        assert 9.0 < F.CLOSER_T0 / lever < 13.0  # closer force at the handle, closed position


def test_door_spans_and_transoms(spec):
    for d in spec["doors"]:
        (x0, y0), (x1, y1) = d["span"]
        total = sum(lf["width"] for lf in d["leaves"])
        if d["n_leaves"] == 1:
            assert math.dist((x0, y0), (x1, y1)) == pytest.approx(total)
            assert d["leaves"][0]["hinge"] in ([x0, y0], [x1, y1])
        else:
            hinges = sorted(tuple(lf["hinge"]) for lf in d["leaves"])
            assert sorted([(x0, y0), (x1, y1)]) == pytest.approx(hinges)
            assert math.dist(*hinges) >= total - 1e-6  # the leaves do not overlap when closed
        assert d["transom"] in (True, False)
        assert d["ceiling"] > F.LEAF_H
    assert door(spec, "W_Stair")["ceiling"] == LAYOUT.BUILDING_CEILING_Z
    assert [d["name"] for d in spec["doors"] if d["transom"]] == [
        "Compartment", "W_Compartment", "E_Compartment", "W_Stair", "E_Stair", "Main_LiftLobby"]


def test_lift_banks(spec):
    banks = {b["name"]: b for b in spec["lift_banks"]}
    assert set(banks) == {"Wing", "Main"}
    assert banks["Main"]["lifts"] == ["MainLift1", "MainLift2"] and banks["Main"]["calls"] == ["MainCall"]
    assert banks["Wing"]["lifts"] == ["Lift1", "Lift2"] and banks["Wing"]["calls"] == ["LobbyCall"]
    names = [b[0] for b in spec["static_boxes"]]
    for b in banks.values():
        fx, y_lo, y_hi = b["front"]
        lx0, ly0, lx1, ly1 = b["lobby"]
        assert lx1 == pytest.approx(fx) and ly0 <= y_lo + 0.05 and y_hi <= ly1 + 1e-9
        assert set(b["static_boxes"]) <= set(names) and len(b["static_boxes"]) == 3 + 8 * 2 + 2
        lifts = [l for l in spec["lifts"] if l["bank"] == b["name"]]
        for l in lifts:
            assert l["front_x"] == pytest.approx(fx)
            assert y_lo < l["center_y"] - l["door_w"] / 2 and l["center_y"] + l["door_w"] / 2 < y_hi
            # HBN 00-04 Fig 28: 3370 mm bed-turning depth in front of each car, inside the lobby
            assert fx - lx0 >= 3.37 - 1e-6
        c0, c1 = sorted((l["car"] for l in lifts), key=lambda c: c["y0"])
        assert c0["y1"] + 0.1 <= c1["y0"]  # the car shells do not touch
        for c in spec["calls"]:
            if c["bank"] == b["name"]:
                x, y, z = c["pos"]
                assert x == pytest.approx(fx) and c["axis"] == [1.0, 0.0, 0.0]
                assert min(y - y_lo, y_hi - y) >= 0.5  # AD M vol 2 3.34: >= 500 mm from a return wall
    assert "fx_Main_front_0" in names and "fx_Main_header_1" in names and "fx_front_0" in names
    main = [b for b in spec["static_boxes"] if b[0].startswith("fx_Main_front")]
    assert min(b[1][1] for b in main) == pytest.approx(banks["Main"]["front"][1])
    assert max(b[2][1] for b in main) == pytest.approx(banks["Main"]["front"][2])


@pytest.mark.parametrize("tier,doors,doors1,lifts,joints", [
    ("wing", 7, 0, 2, 23), ("tier1", 9, 2, 4, 38), ("tier2", 12, 2, 4, 44), ("full", 12, 10, 4, 52)])
def test_tiers(tier, doors, doors1, lifts, joints):
    s = F.build_spec(LAYOUT, tier)
    assert s["tier"] == tier and s["schema"] == 2
    assert len(s["groups"]["Doors"]) == doors and len(s["groups"]["Doors1"]) == doors1
    assert len(s["lifts"]) == lifts and len(F.joint_names(s)) == joints
    ctl = F.FixtureController(s)
    q = ctl.initial_q()
    assert set(ctl.step(DT, q, {k: 0.0 for k in q})) == set(F.joint_names(s))
    hidden = F.building_hide(LAYOUT, s)
    cuts = F.building_openings(LAYOUT, s)
    if tier in ("tier2", "full"):
        assert set(cuts) == {"D07", "D34", "D13", "D14"}
    else:
        assert set(cuts) == {"D13", "D14"}
        assert "Geo_M_DobleDoor3" not in hidden  # D07 keeps its old closed leaves without the exit fixture
    assert ("SM_Door_02b8" in hidden) == (tier == "full")
    assert len(hidden) == len(set(hidden))
    assert "Geo_W_WallDoor2_539" in hidden


def test_tier_from_environment(monkeypatch):
    monkeypatch.delenv(F.TIER_ENV, raising=False)
    assert F.build_spec(LAYOUT)["tier"] == F.DEFAULT_TIER == "tier2"
    monkeypatch.setenv(F.TIER_ENV, "tier1")
    assert F.build_spec(LAYOUT)["tier"] == "tier1"
    monkeypatch.setenv(F.TIER_ENV, "everything")
    with pytest.raises(ValueError):
        F.build_spec(LAYOUT)


def _layout_copy(**changes):
    import types
    m = types.SimpleNamespace(**{k: v for k, v in vars(LAYOUT).items() if not k.startswith("__")})
    for k, v in changes.items():
        setattr(m, k, v)
    return m


def test_name_clashes_raise():
    # A building door named like a wing door would overwrite it (the old "Store" clash, now "Central_Store").
    bad = dict(LAYOUT.BUILDING_FIRE_DOORS)
    bad["Store"] = bad["Central_Store"]
    with pytest.raises(ValueError, match="Store"):
        F.build_spec(_layout_copy(BUILDING_FIRE_DOORS=bad), "full")
    with pytest.raises(ValueError):
        F.build_spec(_layout_copy(BUILDING_LIFTS={"Lift1": LAYOUT.BUILDING_LIFTS["MainLift1"]}), "full")
    with pytest.raises(ValueError):  # a door named like a lift
        F.build_spec(_layout_copy(BUILDING_FIRE_DOORS={"MainLift1": LAYOUT.BUILDING_FIRE_DOORS["W_Stair"]}), "full")
    banks = {k: dict(v) for k, v in LAYOUT.LIFT_BANKS.items()}
    banks["Main"]["prefix"] = ""
    with pytest.raises(ValueError, match="static box"):
        F.build_spec(_layout_copy(LIFT_BANKS=banks), "full")


def test_static_partitions():
    parts = F.static_partitions(LAYOUT)
    names = [p[0] for p in parts]
    assert len(names) == len(set(names)) == 87
    kinds = {}
    for n, lo, hi, k in parts:
        kinds[k] = kinds.get(k, 0) + 1
        assert all(hi[i] > lo[i] for i in range(3)), n
        assert hi[2] <= max(LAYOUT.BUILDING_CEILING_Z, 3.62) + 1e-9
    assert kinds["junction"] == 5 and kinds["glass"] == 4
    j = {n: (lo, hi) for n, lo, hi, k in parts if k == "junction"}
    lo, hi = j["Junction_CompartmentSidePanel"]
    assert lo == pytest.approx([2.53 + OX, -0.33 + OY, 0.0]) and hi == pytest.approx([3.90 + OX, -0.27 + OY, 3.6])
    # every screen opening with a fire door lines up with that door's span (world frame)
    spec = F.build_spec(LAYOUT, "full")
    for sname, door_name in (("C1C2_WLink", "W_Compartment"), ("C2C3_East", "E_Compartment"),
                             ("StairW_South", "W_Stair"), ("StairE_South", "E_Stair"),
                             ("LiftLobby_North", "Main_LiftLobby")):
        s = LAYOUT.BUILDING_SCREENS[sname]
        a, b, head = s["openings"][0]
        (x0, y0), (x1, y1) = door(spec, door_name)["span"]
        vertical = s["line"][0] == s["line"][2]
        lo_s, hi_s = sorted((y0 + OY, y1 + OY)) if vertical else sorted((x0 + OX, x1 + OX))
        assert a - 1e-6 <= lo_s and hi_s <= b + 1e-6, (sname, a, b, lo_s, hi_s)
        assert head >= F.LEAF_H + 0.01


def test_alarm_releases_every_magnet_in_the_hospital(spec):
    sim = Sim(spec)
    sim.run(3.0)
    st = sim.ctl.state(sim.q)["doors"]
    for name in HELD:
        assert st[name]["held_open"], name
    for name in set(SWING) - set(HELD):
        assert st[name]["closed"], name
    sim.ctl.set_alarm(True)
    sim.run(15.0)
    st = sim.ctl.state(sim.q)
    assert all(d["closed"] for d in st["doors"].values()), st["doors"]
    assert all(l["state"] == "open" for l in st["lifts"].values())  # fire recall, both banks


@pytest.mark.parametrize("name", ["Central_Store", "W_Stair", "Ward_DirtyUtility"])
def test_single_leaf_doors_self_close(spec, name):
    sim = Sim(spec)
    lf = leaf(spec, name)
    j = lf["joint"]
    sim.q[j] = open_limits(lf)[0] * 90.0 / LAYOUT.BUILDING_FIRE_DOORS[name]["swing_limit_deg"]  # 90 deg open
    t = 0.0
    while abs(sim.q[j]) > math.radians(2) and t < 30:
        sim.run(0.1)
        t += 0.1
    assert 3.0 < t < 7.0, t


@pytest.mark.parametrize("call,first,second", [("LobbyCall", "Lift1", "Lift2"), ("MainCall", "MainLift1", "MainLift2")])
def test_each_call_button_serves_its_own_bank(spec, call, first, second):
    sim = Sim(spec)
    press(sim, F.PREFIX + call)
    t = 0.0
    while sim.lift(first)["state"] != "open" and t < 10:
        sim.run(0.05)
        t += 0.05
    assert 2.5 < t + 0.3 < 4.5, t     # opening time near 3.0 s (LIFT_SPEC open_s)
    others = {l["name"] for l in spec["lifts"]} - {first}
    assert all(sim.lift(n)["state"] == "closed" for n in others)
    assert sim.lift(first)["door_gap_m"] == pytest.approx(1.4, abs=0.02)
    sim.run(6.5)
    assert sim.lift(first)["state"] == "open"  # 7 s dwell (HTM 08-02 5.32)
    sim.run(6.0)
    assert sim.lift(first)["state"] == "closed"
    press(sim, F.PREFIX + call)               # round robin inside the bank
    sim.run(1.0)
    assert sim.lift(second)["state"] == "opening"


def test_round_robin_is_per_bank(spec):
    ctl = F.FixtureController(spec)
    ctl.press("MainCall")
    ctl.press("LobbyCall")
    assert ctl.lifts["MainLift1"].state == "opening" and ctl.lifts["Lift1"].state == "opening"


def test_curtain_rays_main_bank(spec):
    ctl = F.FixtureController(spec)
    q = ctl.initial_q()
    l = next(x for x in spec["lifts"] if x["name"] == "MainLift2")
    for p in l["panels"]:
        q[p["joint"]] = 0.3
    rays = F.curtain_rays(spec, "MainLift2", q)
    assert len(rays) == 8 and rays[0][0][0] == pytest.approx(l["front_x"] + 0.06)
