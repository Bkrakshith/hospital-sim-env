"""Regression: the surgical wing's fixtures do not change when the main building is added (spec schema 2).

wing_spec_golden.json is the build_spec() output of the wing before the main-building work (schema 1, 2 m link):
7 doors, 2 lifts, 1 call button, 23 joints, 21 static boxes. The only intended change is the link's compartment
doorset: 2 x 1.20 m leaves (was 2 x 0.87 m), east hinge at x 2.53 (was 1.87), for the 4 m link.

    python3 -m pytest hospital-sim-env/fixtures -q
"""

import json
import math
import os

import pytest

import hospital_fixtures as F

HERE = os.path.dirname(os.path.abspath(__file__))
LAYOUT = F.load_layout(os.path.join(HERE, "..", "isaac", "scene", "layout.py"))
with open(os.path.join(HERE, "wing_spec_golden.json")) as _f:
    GOLDEN = json.load(_f)

# Leaf keys of the compartment doorset that change with the leaf width or the east hinge position.
RESIZED = {"width", "mass", "inertia_z", "lever", "hinge"}


def _json(x):
    return json.loads(json.dumps(x))


@pytest.fixture(params=["wing", "tier1", "tier2", "full"])
def spec(request):
    return _json(F.build_spec(LAYOUT, request.param))


def _wing_doors(spec):
    return [d for d in spec["doors"] if d["source_frame"] == "wing"]


def test_wing_items_are_listed_first_and_in_the_same_order(spec):
    assert [d["name"] for d in spec["doors"][:7]] == [d["name"] for d in GOLDEN["doors"]]
    assert [d["name"] for d in _wing_doors(spec)] == [d["name"] for d in GOLDEN["doors"]]
    assert [l["name"] for l in spec["lifts"][:2]] == ["Lift1", "Lift2"]
    assert spec["calls"][0]["joint"] == "fx_LobbyCall"


def test_wing_doors_unchanged_except_the_compartment_resize(spec):
    for g, d in zip(GOLDEN["doors"], _wing_doors(spec)):
        for k in g:  # every schema-1 key keeps its value
            if k == "leaves":
                continue
            assert d[k] == g[k], (d["name"], k)
        assert len(d["leaves"]) == len(g["leaves"])
        for gl, lf in zip(g["leaves"], d["leaves"]):
            keys = set(gl) - (RESIZED if d["name"] == "Compartment" else set())
            for k in keys:
                assert lf[k] == gl[k], (lf["joint"], k)
            assert set(lf) == set(gl), lf["joint"]  # no leaf key added or lost


def test_compartment_doorset_is_resized_as_designed(spec):
    d = next(x for x in spec["doors"] if x["name"] == "Compartment")
    a, b = d["leaves"]
    assert a["hinge"] == [0.13, -0.3] and b["hinge"] == [2.53, -0.3]
    assert a["width"] == b["width"] == 1.20
    # effective clear width at 90 deg: hinge-to-hinge minus two leaf thicknesses (HBN 00-04 8.21: >= 1.74 m)
    ecw = math.dist(a["hinge"], b["hinge"]) - 2 * a["thickness"]
    assert ecw == pytest.approx(2.292, abs=1e-3) and ecw >= 1.74
    assert a["mass"] == pytest.approx(65.52)
    assert F.CLOSER_T0 / a["lever"] == pytest.approx(9.73, abs=0.01)  # AD M vol 2 3.10(a): <= 30 N
    assert d["transom"] is True and d["ceiling"] == 3.6
    assert d["span"] == [[0.13, -0.3], [2.53, -0.3]]
    assert [p["name"] for p in d["side_panels"]] == ["CompartmentSidePanel"]
    panel = d["side_panels"][0]
    assert panel["lo"][0] == pytest.approx(b["hinge"][0]) and panel["hi"][0] == pytest.approx(3.90)


def test_wing_lifts_and_call_unchanged(spec):
    for g, l in zip(GOLDEN["lifts"], spec["lifts"][:2]):
        for k in g:
            assert l[k] == g[k], (l["name"], k)
        assert l["bank"] == "Wing"
    for k, v in GOLDEN["calls"][0].items():
        assert spec["calls"][0][k] == v, k
    assert spec["lift_spec"] == GOLDEN["lift_spec"] and spec["button"] == GOLDEN["button"]
    assert spec["frame"] == GOLDEN["frame"] and spec["wing_origin"] == GOLDEN["wing_origin"]


def test_wing_joint_names_unchanged(spec):
    names = F.joint_names(spec)
    assert names[:sum(len(d["leaves"]) for d in GOLDEN["doors"])] == \
        [lf["joint"] for d in GOLDEN["doors"] for lf in d["leaves"]]
    golden = F.joint_names(GOLDEN)
    assert set(golden) <= set(names)
    wing = [n for n in names if n in golden]
    # the same names in the same relative order (doors, lift panels and buttons, calls)
    assert wing == golden


def test_wing_static_boxes_unchanged(spec):
    bank = next(b for b in spec["lift_banks"] if b["name"] == "Wing")
    wing = [b for b in spec["static_boxes"] if b[0] in bank["static_boxes"]]
    assert wing == GOLDEN["static_boxes"]
    assert spec["static_boxes"][:len(GOLDEN["static_boxes"])] == GOLDEN["static_boxes"]


def test_wing_tier_is_the_schema_1_output_plus_new_keys():
    s = _json(F.build_spec(LAYOUT, "wing"))
    assert len(s["doors"]) == 7 and len(s["lifts"]) == 2 and len(s["calls"]) == 1
    assert len(F.joint_names(s)) == 23 and len(s["static_boxes"]) == 21
    assert s["excluded"]["doors"] == list(LAYOUT.BUILDING_FIRE_DOORS)
    assert s["excluded"]["lifts"] == ["MainLift1", "MainLift2"] and s["excluded"]["calls"] == ["MainCall"]
