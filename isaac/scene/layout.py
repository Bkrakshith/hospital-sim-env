"""Layout of the Isaac hospital: the surgical wing (wing-local frame) and the main building (world frame).

Two frames are used in this file.
- Wing-local frame: metres, axes aligned with the hospital's world frame, origin at WING_ORIGIN (the west edge
  of the 4 m link corridor, flush with the hospital's south exterior wall). +x is east, +y is north (towards the
  hospital), so the whole wing sits at negative y. All wing names (KIT_ROOMS, DOORWAYS, FIRE_DOORS, LIFTS,
  CAMERAS, ...) use this frame.
- World frame: the frame of hospital_g1.usd (/World/Hospital sits at the origin). EXTERIOR and every BUILDING_*
  name use this frame. world = wing + WING_ORIGIN (a translation only, no rotation).

    y=0   ──────────── hospital south wall ──── [opening 2.73 m] ──────────────────────────
          |      PRE-OP (-10..0)       |    LINK (0..4)   |        PACU (4..14)        |
          |                            |  [FD60 doorset]  |                            |
    y=-6  ───────────[door]─────────────                  ─────[door]────────────────┐ LIFT LOBBY  |=LIFT 2
          |               SURGICAL CORRIDOR (-12..14)    [FD]  x=4             [FD]  (14..20)  [call]
    y=-10 ──[FD]─────────────┬─[door]─┬──────────[FD]──────────[EXIT]────────┘            |=LIFT 1
          |  STERILE STORE [FD]SCRUB  |          OPERATING THEATRE (0..10)
          |    (-12..-4)     | (-4..0)|
    y=-18 ───────────────────┴────────┤                                       y=-20

The link is 4 m on the grid (3.8 m clear between its kit walls). The hospital wall opening at its north end is
2.73 m clear (wing x 0.10..2.83): the old office wall stands east of wing x 2.83 (JUNCTION_BOXES).
[FD] = fire door (FIRE_DOORS). The link carries the FD60S compartment doorset 0.3 m south of the hospital wall:
two 1.20 m leaves and a fixed FD60 side panel (wing x 2.53..3.90).
[EXIT] = final exit (FIRE_DOORS["FinalExit"]), x 11..13 on the corridor south wall. Outside it, between the theatre
(x <= 10) and the lift lobby (x >= 14), a paved exit path (EXIT_PATH) runs south.

The main building (section "main hospital building" below) holds a 6-bed in-patient ward in the east corridor
(compartment C3), a small A&E in the west corridor and west hall (C1, C2), three fire compartments, a main bed-lift
pair and the A&E ambulance entrance. It is HBN/HTM-informed, not a certified design.

UK guidance used (NHS England): HTM 05-02 (2015) fire safety, HTM 08-02 (2016) lifts, HBN 00-04 (2014)
circulation, HBN 04-01 (2013) in-patient care, HBN 15-01 (2013) urgent and emergency care, Approved Document M
vol 2 (2024). Paragraph numbers are given next to values.
"""

CELL = 2.0
WING_ORIGIN = (17.93, -4.45, 0.0)
WALL_T = 0.10  # kit wall thickness, inside each room's cell

# Rooms built from the Hospital_A kit, as (x0, y0, x1, y1) in wing-local metres on the 2 m grid.
# The link is 4 m wide (user decision): it takes 2 m from PACU. HBN 00-04 3.10: 2960 mm lets two beds pass.
KIT_ROOMS = {
    "Link": (0, -6, 4, 0),          # was (0, -6, 2, 0)
    "PreOp": (-10, -6, 0, 0),
    "PACU": (4, -6, 14, 0),         # was (2, -6, 14, 0): 3 recovery bays instead of 4
    "Corridor": (-12, -10, 14, -6),
    # Bed-lift lobby. HBN 00-04 6.19: lifts do not open onto corridors. Fig 28: bed turning space
    # 4310 x 3370 mm in front of a bed lift; HTM 05-02 5.36: 4100 mm clear beyond the lobby door swing.
    "LiftLobby": (14, -12, 20, -4),
}

# Wall segments left open: link <-> corridor and link <-> hospital. No kit wall on the link's two ends. The 1.17 m
# north-end strip east of the hospital opening (wing x 2.83..4.0) is closed by JUNCTION_BOXES["CloseEast"]
# (kit walls come in 2 m pieces).
OPENINGS = [(0, -6, 4, -6), (0, 0, 4, 0),
            (20, -12, 20, -4)]  # lobby east side: the lift fronts (LIFTS) close it

# Boxes that join the kit link to the hospital wall, wing-local: name -> (lo xyz, hi xyz, collide).
# The hospital's SE doorway was 2.484 m between its jambs (world x 17.718..20.202, measured with pxr on the wall
# module Geo_W_WallDoor2_539, x 17.16..20.76). That module is hidden (BUILDING_HIDE["link"]). PierWest and
# HeaderBuilding rebuild its solid parts in the same plane (wing y 0.0..0.1). collide=True boxes are also static
# colliders: g1-isaac-sim/make_stage.py (HG1_Colliders) and g1-mujoco-sim/build_world.py (class world).
# hospital_fixtures.static_partitions() returns them in the world frame as "Junction_<name>".
JUNCTION_BOXES = {
    "PierWest": ((-0.77, 0.00, 0.00), (0.00, 0.10, 3.00), True),        # world x 17.16..17.93
    "HeaderBuilding": ((0.00, 0.00, 2.41), (2.83, 0.10, 3.00), True),   # over the new 2.73 m opening
    "HeaderLink": ((0.00, -0.05, 2.41), (4.00, 0.00, 3.62), True),      # link end above the opening (kit ceiling)
    "CloseEast": ((2.83, -0.05, 0.00), (4.00, 0.10, 3.62), True),       # link end in front of the office wall
    # Fixed FD60 side panel of the compartment screen, east of the doorset (the doorset spans x 0.13..2.53).
    "CompartmentSidePanel": ((2.53, -0.33, 0.00), (3.90, -0.27, 3.60), True),
}

# 4 m wall segments that get a kit double-door module (leaves removed, 2 m clear opening).
WING_DOORS = {"PreOp": (-6, -6, -2, -6), "PACU": (6, -6, 10, -6),
              # Vertical modules (x = const): fire-door frames across the corridor and into the lift lobby.
              "CrossCorridor": (4, -10, 4, -6), "LiftLobby": (14, -10, 14, -6),
              # Final exit to outside from the east sub-compartment (HTM 05-02 3.27: two exits).
              "FinalExit": (10, -10, 14, -10)}

# Interior wall lines that are not room edges, wing-local (x0, y0, x1, y1). Walls are built along them
# minus DOORWAYS. x = 4 is the sub-compartment line of the corridor (HTM 05-02 3.25-3.27).
PARTITIONS = [(4, -10, 4, -6)]

# The i4h OR suite (authored in cm) is rotated -90 deg and scaled into the wing so that its west wall
# (suite x = -200 cm) runs along the corridor's south edge (y = -10):
#   wing (x, y) = (sy / 100, -sx / 100 - 12)
SUITE_TRANSLATE = (0.0, -12.0, 0.0)
SUITE_ROTATE_Z = -90.0

# Corridor doors cut into the suite's west wall, in wing-local metres (on y = -10).
SUITE_DOORS = {"Store": (-10, -10, -6, -10), "Scrub": (-4, -10, 0, -10), "Theatre": (6, -10, 10, -10)}

# Suite wall modules replaced to make those doors (suite prim names under /World/Walls).
SUITE_REMOVE = [
    "sm_hospital_a01_2m_straight_08", "sm_hospital_a01_2m_straight_12",  # store
    "sm_hospital_a01_2m_cornerin_03", "sm_hospital_a01_2m_cornerin_02",  # scrub
    "sm_hospital_a01_2m_straight_06", "sm_hospital_a01_2m_cornerin_06",  # theatre
]
# Replacement modules in the suite's cm frame: (kind, translate_xy_cm, rotate_z_deg).
# Doors keep their floor/ceiling (they cover the removed cells and the corridor cells in front).
# The straights restore the wall segments the removed corner modules also carried.
SUITE_ADD = [
    ("door", (-200, -1000), 90),
    ("door", (-200, -400), 90),
    ("door", (-200, 600), 90),
    ("wall", (-200, 0), 0),     # scrub side of the scrub/theatre wall, x -200..0
    ("wall", (0, -400), 180),   # scrub side of the scrub/store wall, x -200..0
    ("wall", (-200, 1000), 0),  # theatre north wall, x -200..0
]

# Existing suite props that must move. "delta" shifts in the suite cm frame; "to" puts the prop's
# footprint centre at a wing-local (x, y) and adds yaw (deg) about it.
SUITE_MOVES = [
    # Theatre: mobile light sat in front of the new corridor door, crash cart in the scrub-room doorway.
    ("sm_surgicallightceilingmobile_a01_01", {"to": (2.1, -13.9)}),
    ("sm_emergencytrolley_a01_01", {"to": (1.2, -17.2)}),
    # Scrub sinks sat on the wall that now holds the corridor door: move them to the opposite wall.
    ("sm_surgicalsink_a01_01", {"to": (-1.0, -17.55), "yaw": 180}),
    ("sm_surgicalsink_a01_02", {"to": (-3.0, -17.55), "yaw": 180}),
    # Anteroom clutter in the walking path -> sterile store.
    ("sm_instrumenttrolley_a01_01", {"to": (-4.8, -15.9)}),
    ("sm_mobilecartsandtables_f01_01", {"to": (-5.9, -10.5), "yaw": 90}),
]
# i4h furnished the side room as a family lounge; it becomes the sterile store, so the lounge pieces go.
# The scrub room's resuscitation bed blocks the scrub <-> store path.
SUITE_HIDE = [
    "sm_softfurniture_b03_01", "sm_softfurniture_b03_02", "sm_tv_a01_01", "sm_tv_a01_02", "sm_tv_a01_03",
    "sm_tv_a01_04", "sm_plant_a02_01", "sm_plant_a02_02", "sm_wastecollector_a03_01", "sm_wastecollector_a03_02",
    "sm_wheelchair_c01_01", "sm_diagnosticequipmentcart_a01_01", "sm_resuscitationbed_a01_01",
    "sm_thermalfiredetector_a01_01",  # was on the wall that is now the scrub-room doorway
]
# i4h suite door modules whose (closed) leaves are removed so the doorway is walkable: scrub <-> store.
SUITE_OPEN_DOORS = ["sm_hospital_a02_4x2m_doordouble_01"]

# Props that ride along with a moved prop (they sit on it).
SUITE_MOVE_WITH = {
    "sm_medicalfacemasks_a01_01": "sm_instrumenttrolley_a01_01",
    "sm_sterilisingcontainersystem_b05_02": "sm_instrumenttrolley_a01_01",
}

# Rooms used only as dressing frames (the suite provides their shell).
SUITE_ROOMS = {
    "Store": (-12, -18, -4, -10),
    "Scrub": (-4, -18, 0, -10),
    "Theatre": (0, -20, 10, -10),
}

# Suite rooms whose kit module lights are replaced by room panels (the theatre keeps its surgical lights).
LIT_SUITE_ROOMS = ["Store", "Scrub", "Theatre"]

# The Isaac hospital's ceiling fittings are meshes only (it ships with just a dome light). Each fitting gets a
# real light: (prim name prefix, light kind, floor area the fitting is taken to light in m2).
HOSPITAL_FIXTURES = [
    ("Light_test", "panel", 12.0),  # 0.88 m square ceiling panels
    ("Geo_M_Light2_low", "downlight", 4.0),  # 0.2 m round downlights
]

# Every doorway, wing-local: (x0, y0, x1, y1) clear-opening segment. Furniture must stay 1 m clear.
# WING_DOORS["PACU"] (6, -6, 10, -6) and DOORWAYS["PACU"] (7, -6, 9, -6) did not change with the link widening:
# they are still inside the new PACU (4..14).
DOORWAYS = {
    "Hospital-Link": (0.1, 0, 2.83, 0),     # was (0, 0, 2, 0): 2.73 m clear between the link wall and the office wall
    "Link-Corridor": (0, -6, 4, -6),        # was (0, -6, 2, -6)
    "PreOp": (-5, -6, -3, -6),
    "PACU": (7, -6, 9, -6),
    "Store": (-9, -10, -7, -10),
    "Scrub": (-3, -10, -1, -10),
    "Theatre": (7, -10, 9, -10),
    "Scrub-Theatre": (0, -15, 0, -13),
    "Scrub-Store": (-4, -15, -4, -13),
    "CrossCorridor": (4, -9, 4, -7),
    "LiftLobby": (14, -9, 14, -7),
    "Lift1": (20, -10.7, 20, -9.3),
    "Lift2": (20, -6.7, 20, -5.3),
    "FinalExit": (11, -10, 13, -10),
}

# ---------------------------------------------------------------- fire doors (HTM 05-02)
# Fire strategy of the wing (single-storey, below the 750 m2 / 30-patient limits of HTM 05-02 3.25):
#   - Compartment wall between the hospital and the wing: FD60S (Table C1, multi-storey building).
#   - Sub-compartment line across the corridor at x = 4: FD30S (3.26, 3.69 operating departments).
#   - Lift lobby off the corridor: FD30S (Table C1, lift shaft not off a hospital street).
#   - Sterile store, a fire hazard room: FD30S, kept shut (5.40, Table 6), on both of its doorways.
#   - Theatre corridor door: FD30S, so the theatre does not join the two corridor sub-compartments
#     (fire could otherwise pass corridor -> theatre -> scrub room -> west corridor round the x = 4 line).
#     Held open on magnets so normal traffic is not blocked; it closes on the alarm.
#   - Final exit from the east sub-compartment to outside: a second exit (3.27), opens outwards.
#     Final exit doors need no fire rating; "EXIT" leaves use FD60S-like 54 mm leaves with a push bar.
# All are self-closing (App C, BS EN 1154). Doors across routes with escape both ways are double-swing,
# others open in the direction of escape (App C). Doors on circulation routes are held open by fail-safe
# magnets that release on the fire alarm (App C, BS 7273-4; HBN 00-04 8.26). Vision panels go down to
# 500 mm above the floor (App C). Leaves: (hinge xy wing-local, closed direction from the hinge, width m).
# swing: side the leaves may open into ("both" = double-swing). hold: side the magnets hold them on,
# None = kept shut. bolted: leaf indices held by flush bolts (leaf-and-a-half doorsets).
# Optional keys (all doors; hospital_fixtures.build_spec reads them): "transom" (True: a panel over the leaves up to
# "ceiling", default 3.6 m in the wing), "side_panels" (JUNCTION_BOXES names of fixed panels that belong to the
# doorset), "swing_limit_deg" (default 100), "tier" (default 1), "use" (free text).
FIRE_DOORS = {
    # Link compartment doorset, re-sized for the 4 m link: 2 x 1.20 m FD60S leaves (was 2 x 0.87 m). The west hinge
    # is 30 mm off the link's west wall face (wing x 0.10); the east hinge is on the fixed FD60 side panel
    # (JUNCTION_BOXES["CompartmentSidePanel"], x 2.53..3.90). The leaves stand 0.3 m inside the link as before.
    # Hinge-to-hinge 2.40 m: effective clear width with both leaves at 90 deg = 2.40 - 2 x 0.054 = 2.29 m
    # (HBN 00-04 8.21: >= 1740 mm on bed routes). Leaf 65.5 kg; closer force at the handle 9.7 N (AD M vol 2
    # 3.10(a): <= 30 N). 1.20 m is inside common FD60 test evidence for 2.1 m leaves [unverified: check the doorset
    # certificate]. "transom": True replaces the name test d["name"] == "Compartment" in fixtures_usd._door_static.
    "Compartment": {"rating": "FD60S", "leaves": [((0.13, -0.3), "+x", 1.20), ((2.53, -0.3), "-x", 1.20)],
                    "swing": "both", "hold": "-y", "bolted": [], "transom": True, "ceiling": 3.6,
                    "side_panels": ["CompartmentSidePanel"]},
    "CrossCorridor": {"rating": "FD30S", "leaves": [((4.0, -9.0), "+y", 1.0), ((4.0, -7.0), "-y", 1.0)],
                      "swing": "both", "hold": "+x", "bolted": []},
    "LiftLobby": {"rating": "FD30S", "leaves": [((14.0, -9.0), "+y", 1.0), ((14.0, -7.0), "-y", 1.0)],
                  "swing": "-x", "hold": "-x", "bolted": []},
    "Store": {"rating": "FD30S", "leaves": [((-9.0, -10.0), "+x", 1.2), ((-7.0, -10.0), "-x", 0.8)],
              "swing": "-y", "hold": None, "bolted": [1]},
    "ScrubStore": {"rating": "FD30S", "leaves": [((-4.0, -15.0), "+y", 1.0), ((-4.0, -13.0), "-y", 1.0)],
                   "swing": "-x", "hold": None, "bolted": []},
    "Theatre": {"rating": "FD30S", "leaves": [((7.0, -10.0), "+x", 1.0), ((9.0, -10.0), "-x", 1.0)],
                "swing": "both", "hold": "-y", "bolted": []},
    "FinalExit": {"rating": "EXIT", "leaves": [((11.0, -10.0), "+x", 1.0), ((13.0, -10.0), "-x", 1.0)],
                  "swing": "-y", "hold": None, "bolted": []},
}

# Outside the final exit, wing-local (visual only, built into usd/exterior.usd by scene/build_exterior.py).
# The final exit opens onto a paved path, flush with the corridor floor (z = 0, no step at the threshold),
# between the theatre's east wall (x = 10) and the lift lobby (x >= 14). It runs south, away from the building.
# The physics ground under it is the hospital's infinite ground plane (Isaac) or world_floor (MuJoCo).
# Lawn rectangles (z = -0.01) fill the open ground south of the suite and of the lift lobby.
EXIT_PATH = {
    "paving": (10.0, -24.0, 14.0, -10.0),
    "lawn": [(-12.0, -24.0, 0.0, -18.0), (0.0, -24.0, 10.0, -20.0), (14.0, -24.0, 24.0, -12.0)],
}

# ---------------------------------------------------------------- lifts (HTM 08-02)
# A duplex pair of bed lifts (HTM 08-02 5.23). The cars do not travel: one landing, doors only.
# Each lift: front plane x (lobby on -x, car on +x) and door centre y, wing-local.
LIFTS = {"Lift1": {"front_x": 20.0, "center_y": -10.0}, "Lift2": {"front_x": 20.0, "center_y": -6.0}}
LIFT_SPEC = {
    "car_w": 1.8, "car_d": 2.7,          # HTM 08-02 4.19: 2500 kg bed lift, car 1800 x 2700 mm
    "door_w": 1.4, "door_h": 2.1,        # 4.19: 1400 mm clear; HBN 00-04 6.30: min 1370 x 2100 mm
    "open_s": 3.0, "close_s": 3.6,       # Table 3 gives 2.7 / 3.3 s for 1100 mm; extrapolated to 1400 mm
    "dwell_s": 7.0, "dwell_short_s": 0.5,  # 5.32: 7 s for bed lifts, 0.5 s after the close button
    "stall_n": 150.0, "ke_j": 10.0,      # App C / BS EN 81-20: stall force < 150 N, kinetic energy < 10 J
    "curtain_z": (0.025, 1.6),           # BS EN 81-20 light curtain height band (secondary source)
    "panel_mass": 45.0,
}
# Landing call button shared by the duplex pair, wing-local (x, y, z), facing -x into the lobby.
# AD M vol 2 3.34: 900-1100 mm high, >= 500 mm from a return wall. Car buttons: 900-1200 mm (1100 mm).
LIFT_CALL = {"LobbyCall": {"pos": (20.0, -8.0, 1.0), "lifts": ["Lift1", "Lift2"]}}
DOOR_CLEARANCE = 1.0

# Asset "back" directions: the side that goes against a wall, in the asset's own frame.
BACK = {
    "SM_Washbasin": "-x", "SM_SoapDispenser_01a": "-x", "SM_Toweldispenser_01a": "-x",
    "SM_Intercom_01a": "-x", "SM_Xraylightbox_01a": "-x", "SM_SupplyCabinet_01c": "-x",
    "SM_MedicalCabinet_01a": "-y", "SM_ShelfSet_01a": "+y", "SM_Shelf_06f": "-y", "SM_MedShelf_01d": "-y",
    "SM_Desk_04a": "+y", "SM_Computer_02b": "+y", "SM_WaterDispenser_01a": "-y", "SM_FirstAidKit_01a": "+y",
    "Refrigerator_A": "+y", "Defibrillator_A": "+y", "VitalPad_A": "-y", "SignSideWall": "+x",
    "SignDoNotEnter": "+y", "Glovebox_A": "+y", "MedicalFacemasks_A": "+y", "EmergencyTrolley_A": "-y",
    "SM_HospitalBed_01b": "+y", "SM_HospitalBed_02d": "+y", "SM_Gurney_01a": "-y", "SM_Gurney_01b": "-y",
    "SM_SupplyCart_01e": "-y", "SM_SupplyCart_02a": "-y", "SM_WheelChair_01a": "-y", "Wheelchair_C": "-y",
    "SM_GasCart_01b": "-y", "SM_GasCart_01c": "-y", "Cart001": "-x", "UVCRobot_A": "-y",
    "InstrumentTrolley001": "-y", "MedicalUtilityCart_A": "-y", "S_WetFloorSign": "-y", "SM_TrashCan": "-y",
    "DiagnosticEquipmentCart_A": "-x", "WasteCollector_A": "-y", "Plant": "-y", "SM_Chart_Rack_01c": "-y",
    "SM_BedSideTable_01b": "-y", "SM_Scale_01a": "-x", "MobileCartsAndTables_A": "-y", "Wheelchair_B": "-y",
    # Main building dressing (assets.py adds NEW_ASSET_KEYS for the first six).
    "SM_Toilet": "-x", "SM_ToiletPaper": "-x", "SM_Mirror": "-x", "SM_MopSet_01a": "-y", "SM_MopSet_01b": "-y",
    "SM_DisposalStand_02": "-y", "SM_Chair_02a": "+y", "SM_SideTable_02a": "-y",
}


def dress_link(r):
    # W = 4 m. The held-open west leaf lies along the west wall from room-local v 4.5 to 5.7: keep it clear.
    r.wall("FirstAid", "SM_FirstAidKit_01a", "W", 2.2, z=1.35)
    r.wall("Sanitizer", "SM_SoapDispenser_01a", "W", 4.0, z=1.1)
    r.wall("Intercom", "SM_Intercom_01a", "E", 4.4, z=1.25)


def dress_preop(r):
    # Three bed bays along the north wall, divided by privacy screens.
    for i, bx in enumerate((1.3, 4.3, 7.3)):
        r.wall(f"Bed{i}", "SM_HospitalBed_02d", "N", bx)
        r.wall(f"BedsideTable{i}", "SM_BedSideTable_01b", "N", bx + 0.95)
        r.wall(f"Oxygen{i}", "SM_GasCart_01c", "N", bx - 0.8)
    r.put("Screen0", "HospitalScreen_A", 2.85, 4.45, yaw=90)
    r.put("Screen1", "HospitalScreen_A", 5.85, 4.45, yaw=90)
    r.wall("DiagCart", "DiagnosticEquipmentCart_A", "E", 4.6)
    # Nurse desk on the south wall, west of the door.
    r.wall("Desk", "SM_Desk_04a", "S", 2.0)
    r.wall("Computer", "SM_Computer_02b", "S", 2.0, z=0.83)
    r.put("DeskChair", "SM_Chair_01a", 2.0, 1.05, yaw=180)
    r.wall("Scale", "SM_Scale_01a", "W", 2.3)
    # Hand-wash station on the south wall, east of the door.
    r.wall("Washbasin", "SM_Washbasin", "S", 8.6, z=0.78)
    r.wall("Soap", "SM_SoapDispenser_01a", "S", 8.15, z=1.3)
    r.wall("Towels", "SM_Toweldispenser_01a", "S", 9.3, z=1.25)
    r.wall("Bin", "SM_TrashCan", "E", 1.0)
    r.wall("MedCabinet", "SM_MedicalCabinet_01a", "E", 2.4)


def dress_pacu(r):
    # 10 m x 6 m (was 12 m): three recovery bays along the north wall, 3.4 m apart (HBN 26: 2-3 bays per theatre).
    for i, bx in enumerate((1.6, 5.0, 8.4)):
        r.wall(f"Bed{i}", "SM_HospitalBed_01b", "N", bx)
        r.wall(f"Oxygen{i}", "SM_GasCart_01c", "N", bx - 0.95)
        r.wall(f"BedsideTable{i}", "SM_BedSideTable_01b", "N", bx + 0.9)
        r.wall(f"VitalPad{i}", "VitalPad_A", "N", bx, z=1.55)
    for i, sx in enumerate((3.3, 6.7)):
        r.put(f"Screen{i}", "HospitalScreen_A", sx, 4.45, yaw=90)
    # Door module on the south wall at room-local u 2..6 (clear 3..5). Nurse station east of it.
    for i, dx in enumerate((7.0, 8.6)):
        r.wall(f"Desk{i}", "SM_Desk_04a", "S", dx)
        r.wall(f"Computer{i}", "SM_Computer_02b", "S", dx, z=0.83)
        r.put(f"Chair{i}", "SM_Chair_01a", dx, 1.05, yaw=180)
    # Cabinet and crash cart west of the door, water east of the nurse station.
    r.wall("MedCabinet", "SM_MedicalCabinet_01a", "S", 0.53)
    r.wall("CrashCart", "EmergencyTrolley_A", "S", 1.72)  # was 1.95: its mesh reached the PACU doorway zone (x 7)
    r.wall("Water", "SM_WaterDispenser_01a", "S", 9.7)
    r.wall("Defibrillator", "Defibrillator_A", "E", 3.4, z=1.3)
    r.wall("Washbasin", "SM_Washbasin", "E", 1.7, z=0.78)
    r.wall("Soap", "SM_SoapDispenser_01a", "E", 1.25, z=1.3)
    r.wall("Towels", "SM_Toweldispenser_01a", "E", 2.4, z=1.25)
    r.wall("Bin", "SM_TrashCan", "E", 3.0)


def dress_corridor(r):
    # Parked along the north wall (room-local y = 4), clear of the pre-op (7..9), link (12..14) and PACU
    # (19..21) openings. The fire doors at u = 16 (x = 4) and u = 26 (x = 14) need their swing and the
    # pockets behind their held-open leaves kept clear (HTM 05-02 App C), so nothing stands at u 15..17 or 25..26.
    r.wall("Gurney0", "SM_Gurney_01a", "N", 2.5)
    r.wall("Gurney1", "SM_Gurney_01b", "N", 5.0)
    r.wall("SupplyCart", "SM_SupplyCart_01e", "N", 10.6)  # was u 14.85: inside the 4 m link opening (u 12..16)
    r.wall("Wheelchair0", "SM_WheelChair_01a", "N", 17.6)
    r.wall("Wheelchair1", "Wheelchair_C", "N", 23.0)
    r.wall("OxygenCart", "SM_GasCart_01b", "N", 24.3)
    # South wall, clear of the store (3..5), scrub (9..11) and theatre (19..21) openings. The theatre fire door
    # swings into the corridor at u 18.8..21.2 and the final exit (u 23..25) needs 1 m clear on its inside: the
    # utility cart and the wet-floor sign stand between the CrossCorridor pocket (u 17) and the theatre door.
    r.wall("LWCart", "Cart001", "S", 1.0)
    r.wall("UVCRobot", "UVCRobot_A", "S", 7.0)
    r.wall("LWTrolley", "InstrumentTrolley001", "S", 12.9)
    r.wall("SupplyCart2", "SM_SupplyCart_02a", "S", 14.3)
    r.wall("UtilityCart", "MedicalUtilityCart_A", "S", 17.6)
    r.wall("WetFloor", "S_WetFloorSign", "S", 18.15)
    # The east wall is now the lift-lobby fire door: the plant moved to the lobby (dress_liftlobby).
    # Wall fixtures: signs and hand sanitizer at every door, first aid, intercom.
    for name, wall, u in (("PreOp", "N", 6.6), ("PACU", "N", 18.6), ("Store", "S", 2.6),
                          ("Scrub", "S", 8.6), ("Theatre", "S", 18.6)):
        r.wall(f"Sign{name}", "SignSideWall", wall, u, z=2.2)
        r.wall(f"Sanitizer{name}", "SM_SoapDispenser_01a", wall, u, z=1.1)
    r.wall("NoEntry", "SignDoNotEnter", "S", 20.0, z=2.45)
    r.wall("FirstAid", "SM_FirstAidKit_01a", "N", 10.3, z=1.35)
    r.wall("Intercom", "SM_Intercom_01a", "N", 11.3, z=1.25)
    r.put("SignBoard", "SignBoard", 13.0, 2.0, z=3.35, anchor="top")


def dress_liftlobby(r):
    # Light on purpose: HBN 00-04 Fig 28 needs a 4.31 x 3.37 m bed-turning space in front of each lift
    # (room-local x 2.63..6), and HTM 05-02 5.36 keeps the lobby clear of stored items. A sign and a
    # sanitizer by the corridor door, the corridor's plant in the north-west corner.
    r.wall("SignLifts", "SignSideWall", "W", 1.4, z=2.2)
    r.wall("Sanitizer", "SM_SoapDispenser_01a", "W", 1.4, z=1.1)
    r.wall("Plant", "Plant", "N", 0.8)


def dress_store(r):
    # Shelving along the south wall, supply cabinets on the west wall.
    for i, sx in enumerate((1.1, 3.0, 4.9)):
        r.wall(f"ShelfSet{i}", "SM_ShelfSet_01a", "S", sx)
    r.wall("Shelf", "SM_Shelf_06f", "S", 6.6)
    r.wall("SupplyCabinet0", "SM_SupplyCabinet_01c", "W", 5.6)
    r.wall("SupplyCabinet1", "SM_SupplyCabinet_01c", "W", 3.9)
    r.wall("MedShelf", "SM_MedShelf_01d", "W", 2.4)
    r.wall("Fridge", "Refrigerator_A", "E", 7.1)
    # Central prep island: two carts with sterile trays and container systems.
    r.put("CartA", "Cart001", 2.8, 3.9, yaw=90)
    r.put("CartB", "Cart003", 3.8, 3.9, yaw=90)
    r.put("TrayA", "SurgicalTray001", 2.8, 3.9, z=0.87)
    r.put("TrayB", "SurgicalTray006", 2.8, 3.9, z=0.96)
    r.put("SterilA", "SterilisingContainerSystem_A", 3.8, 3.9, z=0.87)
    r.put("SterilB", "SterilisingContainerSystem_B", 3.8, 3.9, z=1.0)


def dress_scrub(r):
    r.wall("Towels", "SM_Toweldispenser_01a", "S", 2.0, z=1.3)
    for i, z in enumerate((1.15, 1.3, 1.45)):
        r.wall(f"Gloves{i}", "Glovebox_A", "E", 7.4, z=z)
    r.wall("Masks", "MedicalFacemasks_A", "E", 7.1, z=1.6)


def dress_theatre(r):
    # Lightwheel laparoscopy set on a trolley by the east wall.
    r.wall("LWTrolley", "InstrumentTrolley001", "E", 5.0)
    r.put("Tray", "SurgicalTrayNoLid006", 9.7, 5.0, z=0.78, yaw=90)
    r.put("Trocar", "Trocar002", 9.62, 4.62, z=0.87)
    r.put("Needle", "PneumoperitoneumNeedle001", 9.78, 4.62, z=0.87)
    r.put("Tweezers", "SurgicalTweezers", 9.7, 5.2, z=0.87)


DRESSING = {
    "Link": dress_link,
    "PreOp": dress_preop,
    "PACU": dress_pacu,
    "Corridor": dress_corridor,
    "LiftLobby": dress_liftlobby,
    "Store": dress_store,
    "Scrub": dress_scrub,
    "Theatre": dress_theatre,
}

# Viewing cameras, wing-local: name -> (eye xyz, target xyz, focal length mm).
CAMERAS = {
    "Cam_HospitalToWing": ((1.45, 6.0, 1.5), (1.45, -8.0, 1.0), 14),  # centred on the 2.73 m opening
    "Cam_Link": ((1.45, -0.7, 1.6), (2.0, -9.0, 0.8), 12),
    "Cam_CorridorWest": ((12.2, -8.0, 1.7), (-12.0, -8.0, 0.9), 14),
    "Cam_CorridorEast": ((-11.4, -8.0, 1.7), (14.0, -8.0, 0.9), 14),
    "Cam_PreOp": ((-4.0, -5.4, 2.4), (-5.0, -1.0, 0.6), 10),
    "Cam_PACU": ((8.0, -5.4, 2.4), (8.0, -1.0, 0.6), 10),  # unchanged: still inside the new PACU
    "Cam_Store": ((-8.0, -10.6, 2.4), (-8.0, -17.0, 0.6), 10),
    "Cam_Scrub": ((-2.0, -10.6, 2.2), (-2.0, -17.5, 0.8), 10),
    "Cam_Theatre": ((8.0, -10.6, 2.6), (4.0, -15.5, 0.8), 10),
    "Cam_TheatreWide": ((9.3, -19.3, 3.0), (3.0, -12.0, 0.6), 10),
    "Cam_CrossCorridor": ((0.6, -6.7, 2.1), (5.0, -8.6, 0.9), 12),
    "Cam_LiftLobby": ((14.6, -11.4, 2.3), (20.0, -7.0, 1.0), 10),
    "Cam_StoreDoor": ((-6.2, -7.0, 1.9), (-8.6, -10.6, 0.8), 12),
    # Fire doors added from the UK guidance review (docs/uk-guidance-compliance.md).
    "Cam_TheatreDoor": ((8.0, -6.6, 1.8), (8.0, -12.5, 0.9), 12),        # from the corridor, held open
    "Cam_FinalExit": ((12.0, -6.5, 1.6), (12.0, -10.0, 1.55), 12),       # from inside the corridor
    "Cam_FinalExitOutside": ((12.0, -17.5, 1.7), (12.0, -10.0, 1.3), 14),  # from the paved exit path
    "Cam_ScrubStoreDoor": ((-0.6, -13.4, 1.7), (-4.0, -14.1, 1.1), 12),  # from the scrub room
}

# ---------------------------------------------------------------- exterior: drive, ambulance entrance, ambulance
# WORLD frame (not wing-local): metres, the frame of hospital_g1.usd. scene/build_exterior.py builds it into
# usd/exterior.usd (visual only). g1-isaac-sim/make_stage.py and g1-mujoco-sim/build_world.py build
# EXTERIOR_COLLIDERS. The main entrance of the Isaac hospital is the pair of glass sliding doors GlassDoor_55 and
# GlassDoor2_5 (x -5.64..3.16) in the south facade (outer face y = -2.85); they open into the reception.
# The drive comes in from the south (site edge y = -25); UK traffic keeps left. Rectangles are (x0, y0, x1, y1).
#   footway  along the facade. Raised by kerb_h west of x = -7.5, a 1:12 ramp, then flush with the road (dropped
#            kerb) from x = -6.0 to its east end: the path from the doors to the patient-transport bays is level.
#   apron    asphalt forecourt: two patient-transport bays (white), a keep-clear hatch, a drop-off lay-by (white).
#   road     two-lane access road, south to the apron. It stays west of the wing (wing west wall x = 5.93).
#   ae_lane, ae_apron  the A&E road: a 6 m two-way lane west from the apron's west edge, and a 12 x 9.2 m apron in
#            front of the courtyard for reversing into the ambulance bay AE1.
# HBN 15-01 17.2-17.4: separate ambulance and walk-in entrances; walk-ins do not pass the ambulance entrance; a lit
# canopy that clears vehicle lights and aerials; bays close to the ambulance entrance. The ambulance entrance is the
# south-facing courtyard (open notch x -38.24..-34.31, y 1.75..7.25) with the doors D13 (to the majors corridor)
# and D14 (to the handover bay and resus). It is 35 m west of the main entrance (x -1.24) and 4.6 m behind its
# facade line. Walk-in patients use the main entrance.
# The asphalt, the flush paving and the hospital floor are at z = 0. Lawn is at z = -0.01.
EXTERIOR = {
    "entrance": {"doors_x": (-5.64, 3.16), "facade_y": -2.85, "center": (-1.24, -2.85)},
    "kerb_h": 0.125, "kerb_w": 0.15,
    "footway": (-11.74, -5.85, 5.30, -2.85),
    "footway_raised_x": (-11.74, -7.5), "footway_ramp_x": (-7.5, -6.0),  # flush from x = -6.0 to the east end
    "apron": (-14.0, -17.0, 5.30, -5.85),
    "road": (-14.0, -25.0, -7.5, -17.0),
    # PTS1/PTS2: the two bays in front of the main entrance are patient-transport (and accessible drop-off) bays,
    # white markings "PATIENT TRANSPORT". AE1: the A&E ambulance bay in the A&E apron, yellow "AMBULANCE".
    "bays": {"PTS1": (-3.2, -14.35, 0.6, -5.85), "PTS2": (0.6, -14.35, 4.4, -5.85),
             "AE1": (-38.08, -7.40, -34.48, 0.00)},
    "bay_kind": {"PTS1": "pts", "PTS2": "pts", "AE1": "ambulance"},
    "keep_clear": (-6.0, -10.0, -3.4, -5.85),
    "dropoff": (-11.5, -8.35, -7.5, -5.85),
    # A&E road, level with the building floor (z 0). The ambulance (6.97 m) turns in three points on the apron and
    # the lane. The apron ends at y 1.65, the kerbs at y 1.60, clear of the facade walls.
    "ae_lane": (-42.0, -13.5, -14.0, -7.5),
    "ae_apron": (-42.0, -7.5, -30.0, 1.65),
    # Paved floor over the courtyard and the door lobby (visual). The physics floor is already there: the Isaac
    # HospitalFloor slab and the MuJoCo floor plane.
    # The courtyard paving starts at the apron edge (y 1.65): the joint is level and no lawn shows in it.
    "courtyard_floor": [(-38.24, 1.65, -34.31, 7.25), (-37.74, 7.25, -32.74, 10.45)],
    # Canopy over the courtyard and the back of AE1: visual roof, two posts (colliders), soffit 4.0 m. It clears the
    # 2.9 m ambulance with its light bar and aerials (HBN 15-01 17.3 gives no height [unverified]). Two lights go
    # under it (HBN 15-01 17.3: lit).
    "canopy": {"rect": (-39.20, -2.50, -33.30, 7.25), "z": (4.00, 4.15), "posts": [(-39.05, -2.35), (-33.45, -2.35)],
               "post_size": 0.20,
               # Two luminaires on the soffit (1.2 x 0.4 m, rect lights straight down): one over the rear doors
               # of the ambulance, one over the courtyard. The fascia carries an "A&E EMERGENCY" panel.
               "lights": [(-36.25, -0.60), (-36.25, 4.60)], "light_size": (1.2, 0.4)},
    # Wayfinding totem on the lawn at the lane junction, read by drivers coming north on the access road:
    # "A&E Emergency" (arrow left, to the A&E lane) over "Main entrance" (arrow ahead). Generic panels, no logo.
    # pos = centre of the footprint, two posts (colliders) at +-post_dx along x, panels from z0 to z0 + 2 x panel_h.
    "signs": {"Totem": {"pos": (-15.60, -15.00), "width": 1.6, "panel_h": 0.5, "z0": 1.0, "post": 0.08,
                        "post_dx": 0.65, "panels": ["totem_ae", "totem_main"]}},
    # Upstand kerbs on the outer edges of the asphalt (top at kerb_h). The West kerb is split where the A&E lane
    # joins the apron.
    "kerbs": {"West": (-14.15, -25.0, -14.0, -13.65), "WestUpper": (-14.15, -7.35, -14.0, -5.70),
              "NorthWest": (-14.15, -5.85, -11.74, -5.70),
              "East": (5.30, -17.15, 5.45, -5.85), "South": (-7.5, -17.15, 5.45, -17.0),
              "RoadEast": (-7.5, -25.0, -7.35, -17.0),
              "AELaneNorth": (-30.0, -7.50, -14.15, -7.35), "AELaneSouth": (-42.15, -13.65, -14.15, -13.50),
              "AEApronWest": (-42.15, -13.65, -42.00, 1.60), "AEApronEast": (-30.00, -7.35, -29.85, 1.60)},
    # The lawn reaches x -50, so DriveFloor (from the lawn extents) also covers the A&E lane and apron.
    "lawn": [(-50.0, -27.0), (5.6, -27.0), (5.6, -2.85), (-11.8, -2.85), (-11.8, 1.7), (-50.0, 1.7)],
    # UK emergency ambulance (box-body van conversion), reversed into AE1: rear doors face the courtyard (rear at
    # y -0.30), 2.0 m from the courtyard mouth and about 7.5 m from D14. pos = footprint centre on the ground,
    # yaw = heading of the bonnet (deg, -90 = south).
    "ambulance": {"bay": "AE1", "pos": (-36.28, -3.82), "yaw": -90.0, "size": (6.97, 2.30, 2.90)},
    "cameras": {  # world frame: eye, target, focal length (mm)
        "Cam_Ambulance": ((-29.5, -12.5, 2.2), (-36.3, -3.8, 1.3), 16),
        "Cam_Entrance": ((-11.5, -18.0, 3.4), (-1.0, -4.5, 1.0), 14),
        "Cam_AEEntrance": ((-24.0, -18.0, 8.0), (-34.0, -3.0, 0.5), 14),  # raised: lane, apron, AE1, canopy, courtyard
    },
}


def _exterior_colliders(E):
    """Static boxes for the robot: name -> (centre xyz, size xyz, quat wxyz). Shared by Isaac and MuJoCo."""
    import math
    h, out = E["kerb_h"], {}

    def rect(name, r, z0, z1):
        out[name] = (((r[0] + r[2]) / 2, (r[1] + r[3]) / 2, (z0 + z1) / 2),
                     (r[2] - r[0], r[3] - r[1], z1 - z0), (1.0, 0.0, 0.0, 0.0))
    # Ground slab south of the facade (Isaac only: the MuJoCo world already has an infinite floor plane).
    # North of the facade line, the HospitalFloor slab of make_stage.py covers the lawn.
    xs, ys = [p[0] for p in E["lawn"]], [p[1] for p in E["lawn"]]
    rect("DriveFloor", (min(xs), min(ys), max(xs), E["entrance"]["facade_y"]), -0.1, 0.0)
    fx0, fy0, fx1, fy1 = E["footway"]
    rect("FootwayRaised", (E["footway_raised_x"][0], fy0, E["footway_raised_x"][1], fy1), -0.02, h)
    # Ramp: a 0.1 m thick box pitched so its top face runs from z = h (west) to z = 0 (east).
    a, b = E["footway_ramp_x"]
    th, t = math.atan2(h, b - a), 0.1
    out["FootwayRamp"] = (((a + b) / 2 - t / 2 * math.sin(th), (fy0 + fy1) / 2, h / 2 - t / 2 * math.cos(th)),
                          (math.hypot(b - a, h), fy1 - fy0, t), (math.cos(th / 2), 0.0, math.sin(th / 2), 0.0))
    for k, r in E["kerbs"].items():
        rect(f"Kerb{k}", r, -0.02, h)
    # Canopy posts, full height to the canopy soffit.
    can = E.get("canopy")
    if can:
        s = can["post_size"]
        for i, (px, py) in enumerate(can["posts"]):
            out[f"CanopyPost{i}"] = ((px, py, can["z"][0] / 2), (s, s, can["z"][0]), (1.0, 0.0, 0.0, 0.0))
    # Sign posts, full height of the sign (the panels are between them, above the robot).
    for name, sg in E.get("signs", {}).items():
        (sx, sy), top, p = sg["pos"], sg["z0"] + len(sg["panels"]) * sg["panel_h"], sg["post"]
        for i, dx in enumerate((-sg["post_dx"], sg["post_dx"])):
            out[f"Sign{name}Post{i}"] = ((sx + dx, sy, top / 2), (p, p, top), (1.0, 0.0, 0.0, 0.0))
    # Ambulance: rear box body and cab (vehicle frame: +x forward), turned by the yaw.
    amb = E["ambulance"]
    yaw = math.radians(amb["yaw"])
    c, s = math.cos(yaw), math.sin(yaw)
    q = (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))
    for name, (x0, x1, half_w, z1) in {"AmbulanceBody": (-3.52, 1.30, 1.15, 2.90),
                                       "AmbulanceCab": (1.30, 3.47, 1.0, 2.45)}.items():
        cx = (x0 + x1) / 2
        out[name] = ((amb["pos"][0] + c * cx, amb["pos"][1] + s * cx, z1 / 2), (x1 - x0, 2 * half_w, z1), q)
    return out


EXTERIOR_COLLIDERS = _exterior_colliders(EXTERIOR)


# =====================================================================================================================
# Main hospital building (WORLD frame of hospital_g1.usd, metres; /World/Hospital sits at the origin)
# =====================================================================================================================
# Final integrated design of 2026-10-13: (1) a 6-bed adult in-patient ward (HBN 04-01) in the east corridor and a
# small A&E (HBN 15-01) in the west corridor and west hall; (2) three fire compartments (HTM 05-02 3.11: more than
# 1000 m2 needs at least 3); (3) the link widened to 4 m (above). All coordinates were measured with pxr on the real
# asset (hospital.usd, lite mirror). Readers:
#   hospital_fixtures.build_spec()       BUILDING_FIRE_DOORS, BUILDING_LIFTS, BUILDING_LIFT_CALL, LIFT_BANKS
#   hospital_fixtures.static_partitions() BUILDING_SCREENS, BUILDING_WALL_REBUILDS, JUNCTION_BOXES
#   scene/build_hospital.py              BUILDING_HIDE, BUILDING_UNINSTANCE_HIDE, BUILDING_ROOMS, BUILDING_DRESSING,
#                                        BUILDING_LIGHTS, BUILDING_DOORWAYS (verify)
#   g1-mujoco-sim/extract_scene_boxes.py BUILDING_OPENINGS
#   g1-isaac-sim/tools/check_routes.py   BUILDING_ROUTES
# BUILDING_CAMERAS are merged into CAMERAS (wing-local) at the end of this file.
BUILDING_FRAME = "world"
BUILDING_WALL_T = 0.0      # Room.wall() offset for building rooms: their rects are the measured inner wall faces
BUILDING_CEILING_Z = 2.99  # underside of the building roof slab (Geo_Roof_Costum*, z 2.99..3.337)

# Asset prims to deactivate: names of children of the building's defaultPrim (/World/Hospital/<name>).
# Measured with pxr on hospital.usd (scripts/inspect1.py, doorframes.json, rooms0.py). Hiding instanced leaves saves
# no triangles (shared prototypes); it removes their static colliders.
BUILDING_HIDE = {
    # Link widening: the SE wall module with the old 2.48 m hole, and a skirting strip now inside the opening.
    # (Geo_M_DobleDoor_867 / Geo_M_DobleDoor6 are already in HOSPITAL_DOOR_LEAVES.)
    "link": ["Geo_W_WallDoor2_539", "Geo_W_TrimStrateBase1_15"],
    # Bed-lift shafts in the two WCs (LIFT_BANKS["Main"]): west walls (the lift front replaces them), door leaves and
    # frames on the east walls (closed by BUILDING_SCREENS "ShaftFill*"), fittings, floors and skirtings inside the cars.
    "lift_shafts": [
        "Geo_M3_BackWall6", "Geo_M3_BackWall2", "Geo_M_DoorFrame34", "Geo_M_DoorFrame35", "SM_Door_2", "SM_Door_3",
        "Geo_M3_Floor_396", "Geo_M3_Floor2",
        "Geo_M3_TrimStrateDoorInside_284", "Geo_M3_TrimStrateDoorInside2", "Geo_M3_TrimStrateInside2",
        "Geo_M3_TrimStrateInside3", "Geo_M3_TrimStrateInsideSide_288", "Geo_M3_TrimStrateInsideSide2",
        "Geo_M3_TrimStrateInsideSide3", "Geo_M3_TrimStrateInsideSide4", "Geo_M_CornerTrim20",
        "Geo_W_TrimStrateBase2_319", "Geo_M3_TrimStrateSide4", "Geo_M3_TrimStrateSide2_309",  # lobby-side skirtings of the hidden WC west walls
        "SM_Toilet_227", "SM_Washbasin_230", "SM_Mirror_236", "SM_ToiletPaper_224", "SM_TrashCan_1",
        "SM_Toweldispenser_01a_241", "SM_Toilet2", "SM_Washbasin2", "SM_Mirror2", "SM_ToiletPaper2", "SM_TrashCan_2",
        "SM_Toweldispenser_01a2",
        # loose items standing in the new lift lobby and on its screen lines
        "SM_SideTable_02a2", "SM_SupplyCart_02a2_91", "SM_WheelChair_01a2", "BP_DrinksMachine_2",
    ],
    # The 6 side walls rebuilt as boxes in the same place (BUILDING_WALL_REBUILDS lists them per wall).
    "rebuilt_walls": [],  # filled below from BUILDING_WALL_REBUILDS
    # Closed double doors replaced by EXIT doorsets with physics (D07, D34), and the courtyard doors opened (D13, D14:
    # ambulance entrance, automatic doors modelled held open on power loss, HTM 05-02 3.59; no leaves).
    "exits": ["Geo_M_DobleDoor3", "Geo_M_DobleDoor7", "Geo_M_DobleDoor2", "Geo_M_DobleDoor8"],
    "courtyard": ["Geo_M_DobleDoor4", "Geo_M_DobleDoor10", "Geo_M_DobleDoor5", "Geo_M_DobleDoor9"],
    # Static leaves replaced by single FD30S doorsets with physics (BUILDING_FIRE_DOORS, Doors1 group).
    "fd30_leaves": ["SM_Door_01c5", "SM_Door_01c4", "SM_Door_01b2", "SM_Door_01b5_8", "SM_Door_01b6",
                    "SM_Door_02b12", "SM_Door_02b10", "SM_Door_02b8"],
    # Static leaves (shut or ajar) of rooms with a new use: the doorway becomes an open doorway. Bedroom, cubicle and
    # clinic doors are not fire doors (HTM 05-02 5.40); leaves can come back later as non-fire Doors1 fixtures.
    "open_doorways": [
        "SM_Door_01c10", "SM_Door_01c9", "SM_Door_01c8", "SM_Door_01c7",            # majors M5, M4, M3, M2
        "SM_Door_01c_8", "SM_Door_01c2", "SM_Door_01c3",                            # relatives, majors M7, M6
        "SM_Door_01b3", "SM_Door_01b9", "SM_Door_01b7", "SM_Door_01b8",             # X-ray, triage, minors 1, 2
        "SM_Door_02b11", "SM_Door_02b9",                                            # bed B6, pantry
        "SM_Door_02b7", "SM_Door_02b6", "SM_Door_02b5", "SM_Door_02b4", "SM_Door_02b3",  # B1, WS1, B2, B3, WS2
        "SM_Door_01_48", "SM_Door_02b_51", "SM_Door_02b2",                          # B4, B5, old office
    ],
    # Furniture that conflicts with a new use (beds in utility rooms, office kits in bedrooms/cubicles, clutter in
    # stair enclosures, corridors and new rooms). Gurneys SM_Gurney_01a_95 / SM_Gurney_01a2 stood under the west
    # stair: edit (c) parks two gurneys in the A&E handover bay instead (dressing), not in resus.
    "furniture": [
        # west stair enclosure and resus
        "SM_Gurney_01a_95", "SM_Gurney_01a2", "SM_GasCart_01b_99", "SM_GasCart_01c_102", "SM_Chart_Rack_01a_92",
        "warningsign_01", "SM_DisposalStand_4", "SM_SideTable_02a6", "SM_BooksSet_28", "SM_BottleF3",
        "SM_Shelf_06f2", "SM_Shelf_06f3", "SM_Shelf_06f4", "SM_Shelf_06f5", "SM_SupplyCart_02a3", "SM_Cart_01a_89",
        # A&E rooms: office kits in majors/isolation, bed + kit in relatives, bed in clean utility
        "SM_ShelfSet_01a9", "SM_ShelfSet_01a10", "SM_SupplyCart_01e6", "SM_Chair_01a7", "SM_Computer_02b7",       # M1
        "SM_ShelfSet_01a7", "SM_ShelfSet_01a8", "SM_SupplyCart_01e5", "SM_Chair_01a6", "SM_Computer_02b6",        # M3
        "SM_ShelfSet_01a5", "SM_ShelfSet_01a6", "SM_SupplyCart_01e4", "SM_Chair_01a5", "SM_Computer_02b5",        # M4
        "SM_ShelfSet_01a3", "SM_ShelfSet_01a4", "SM_SupplyCart_01e3", "SM_Chair_01a4", "SM_Computer_02b4",        # isolation
        "SM_HospitalBed_02d8", "SM_ShelfSet_01a13", "SM_ShelfSet_01a14", "SM_SupplyCart_01e8", "SM_Computer_02b9",  # relatives
        "SM_HospitalBed_02d7", "SM_Chair_01a8",                                                                   # A&E clean util
        # central rooms and A&E walk-in
        "SM_SupplyCart_01e9", "SM_SideTable_02a_73", "SM_Scale_01a_43", "SM_Chair_01a2_68",
        # ward: bedrooms, pods, utility rooms
        "SM_ShelfSet_01a21", "SM_ShelfSet_01a22", "SM_SupplyCart_01e13", "SM_Chair_01a13", "SM_Computer_02b13",    # B2
        "SM_ShelfSet_01a23", "SM_ShelfSet_01a24", "SM_SupplyCart_01e14", "SM_Chair_01a14", "SM_Computer_02b14",   # pod 2
        "SM_HospitalBed_02d15",
        "SM_SupplyCart_01e15", "SM_Chair_01a15",                                                                  # B5
        "SM_ShelfSet_01a19", "SM_SupplyCart_01e12", "SM_Chair_01a12", "SM_Computer_02b12",                        # pod 3
        "SM_HospitalBed_02d12", "SM_Chair_01a11",                                                                 # clean util
        "SM_HospitalBed_02d11", "SM_Chair_01a10", "SM_Computer_02b10", "SM_SupplyCart_01e10",                     # pantry
        "SM_MopSet_01a2", "SM_MopSet_01b2", "S_WetWarningSign", "SM_Shelf_06f7", "SM_SupplyCart_02a7",            # ward corridor
        # old office (becomes en-suite B5, assisted shower, day room): every loose item
        "SM_Desk_04a_112", "SM_Desk_04d_115", "SM_Desk_04d2", "SM_Desk_04d3", "SM_ReceptionDesk_01b2_120",
        "SM_Desk_04a2", "SM_Desk_04d4", "SM_Desk_04d5", "SM_Desk_04d6", "SM_Chair_04a4", "SM_BedSideTable_01b_132",
        "SM_BedSideTable_01b2", "SM_BedSideTable_01b3", "SM_BedSideTable_01b4", "SM_BedSideTable_01b5",
        "SM_Container_01a2_143", "SM_Computer_02a_146", "SM_MedicalBox_01g_155", "SM_MedicalCabinet_01a3_158",
        "SM_MedicalCabinet_01a4", "SM_SupplyCabinet_01c_167", "SM_SupplyCart_02a9_170", "SM_Toolkit_02i2_173",
        "SM_Toolkit_02f_211", "SM_UtilitiesTray_01e2_208", "SM_BooksSet_2", "SM_Frame11_5", "SM_Frame17_8",
        "SM_Frame15_11", "SM_Frame12_17", "SM_Chart_Rack_01c4_20",
        "SM_PillBottleSet_01a16", "SM_PillBottleSet_01a17", "SM_PillBottleSet_01a18", "SM_PillBottleSet_01a19",
        "SM_PillBottleSet_01a20", "SM_PillBottleSet_01a21", "SM_PillBottleSet_01a22", "SM_PillBottleSet_01a23",
        "SM_PillBottleSet_01a24", "SM_PillBottleSet_01a25", "SM_PillBottleSet_01a26", "SM_PillBottleSet_01a27",
        "SM_PillBottleSet_01a28", "SM_PillBottleSet_01a29", "SM_PillBottleSet_01e14", "SM_PillBottleSet_01e15",
        "SM_PillBottleSet_01e16", "SM_PillBottleSet_01e17", "SM_PillBottleSet_01e18", "SM_PillBottleSet_01e19",
        "SM_PillBottleSet_01e20", "SM_PillBottleSet_01e21", "SM_PillBottleSet_01e22", "SM_PillBottleSet_01e23",
        "SM_PillBottleSet_01e24", "SM_PillBottleSet_01e25",
    ],
    # Ceiling fittings that new partitions would cut through (pods, WC block): replaced by BUILDING_LIGHTS.
    # build_lighting() must skip hidden fittings (it reads hospital.usd directly today).
    "light_fittings": ["Light_test70", "Light_test71", "Light_test76", "Light_test77", "Geo_M_Light2_low165",
                       "Geo_M_Light2_low168"],
    # Down-stair guard rails that stand on the new fixed-screen lines.
    "stair_rails": ["Geo_M_StairsPlaneHalfRail", "Geo_M_StairsPlaneHalfRail2", "Geo_M_StairsPlaneHalfRail3",
                    "Geo_M_StairsPlaneHalfRail4", "Geo_M_StairsPlaneHalfRail5"],
}

# Optional, tier 2 (walk-in route through the main entrance in Isaac): the glass leaves of the main entrance are
# instanced; un-instance the two units (instanceable=False override) and deactivate their glazing mesh. Verify in
# Isaac that Section1 is the glass (it is the 13 mm pane on the windows: Geo_M_Window20/Section1).
BUILDING_UNINSTANCE_HIDE = {
    "GlassDoor_55": ["GlassDoor/GlassDoor/Section1"],
    "GlassDoor2_5": ["GlassDoor/GlassDoor/Section1"],
}

# ---------------------------------------------------------------- rebuilt side walls (6, Option A)
# Each wall between two rooms is hidden (all its prims) and rebuilt as one box in the same place, minus openings:
# en-suite doorways ("door": 0.95 m sliding leaf, not modelled; head 2.10 m) and vision panels ("vision": glass
# box from z 0.90 to 2.00 between a nurse workstation and its bedroom, HBN 04-01 4.44 / 4.51).
# box = (x0, y0, x1, y1) world, full height 0..BUILDING_CEILING_Z; openings run along x (these walls are y-walls).
BUILDING_WALL_REBUILDS = {
    "R1_B1_P1": {"box": (20.76, 24.900, 26.86, 25.190),
                 "hide": ["Geo_M1_SideWall12", "Geo_M1_SideWall13", "Geo_M1_SideWallPanel10", "Geo_M1_SideWallPanel11",
                          "Geo_M1_TrimInsideSide61", "Geo_M1_TrimInsideSide62"],
                 "openings": [("door", 22.30, 23.25), ("vision", 21.00, 22.00)]},
    "R2_P1_B2": {"box": (20.76, 21.200, 26.86, 21.500),
                 "hide": ["Geo_M1_SideWall10", "Geo_M1_SideWall11", "Geo_M1_SideWallPanel8", "Geo_M1_SideWallPanel9",
                          "Geo_M1_TrimInsideSide59", "Geo_M1_TrimInsideSide60"],
                 "openings": [("door", 22.30, 23.25), ("vision", 21.00, 22.00)]},
    "R3_B3_P2": {"box": (20.76, 13.799, 26.86, 14.100),
                 "hide": ["Geo_M1_SideWall7", "Geo_M1_SideWall50", "Geo_M1_SideWallPanel2", "Geo_M1_SideWallPanel5",
                          "Geo_M1_TrimInsideSide55", "Geo_M1_TrimInsideSide56"],
                 "openings": [("door", 22.30, 23.25), ("vision", 21.00, 22.00)]},
    "R4_P2_B4": {"box": (20.76, 10.077, 26.86, 10.400),
                 "hide": ["Geo_M1_SideWall6", "Geo_M1_SideWall48", "Geo_M1_TrimInsideSide21", "Geo_M1_SideWallPanel_845",
                          "Geo_M1_SideWallPanel4", "Geo_M1_TrimInsideSide53", "Geo_M1_TrimInsideSide54",
                          "Geo_M1_SideWallPanel25"],
                 "openings": [("door", 22.30, 23.25), ("vision", 21.00, 22.00)]},
    "R5_B5_Office": {"box": (20.76, 2.718, 26.86, 3.000),
                     "hide": ["Geo_M1_SideWall_720", "Geo_O_SideWall2", "Geo_M1_TrimInsideSide24",
                              "Geo_M1_TrimInsideSide47", "Geo_M1_SideWallPanel23", "Geo_M1_TrimInsideSide48"],
                     "openings": [("door", 22.30, 23.25)]},
    "R6_P3_B6": {"box": (11.16, 24.900, 17.26, 25.200),
                 "hide": ["Geo_M1_SideWall16", "Geo_M1_SideWall17", "Geo_M1_SideWallPanel14", "Geo_M1_SideWallPanel15",
                          "Geo_M1_TrimInsideSide70", "Geo_M1_TrimInsideSide73"],
                 "openings": [("door", 11.55, 12.50)]},
}
REBUILD_DOOR_HEAD = 2.10
VISION_Z = (0.90, 2.00)
for _w in BUILDING_WALL_REBUILDS.values():
    BUILDING_HIDE["rebuilt_walls"] += _w["hide"]

# ---------------------------------------------------------------- new partitions and screens
# name -> {"line": (x0, y0, x1, y1) centre line (axis-aligned), "t": thickness, "kind", "openings": [(a, b, head_z)]
# along the line, "glazed": [(a, b, z0, z1)] glass in the box}. Height 0..BUILDING_CEILING_Z.
# kind: "compartment" (FD60 screen), "stair" (FD30 stair enclosure), "lobby" (FD30 lift lobby), "fixed" (FD30 fixed
# screen closing a down-stair; no door), "shaft" (lift shaft fill), "partition" (non-rated), "wc" (non-rated).
# Door leaves in these openings are BUILDING_FIRE_DOORS (same coordinates); openings without one are open doorways.
BUILDING_SCREENS = {
    # Compartment screens at the two ward entrances (C1|C2 in the W-LINK passage, C2|C3 at the east-hall opening).
    "C1C2_WLink": {"line": (-35.50, 10.551, -35.50, 14.350), "t": 0.10, "kind": "compartment",
                   "openings": [(11.47, 13.44, 2.12)]},
    "C2C3_East": {"line": (11.21, 4.150, 11.21, 10.250), "t": 0.10, "kind": "compartment",
                  "openings": [(6.27, 8.24, 2.12)]},
    # Up-stair enclosures (C2): FD30S self-closing door, never held open, opening out of the stair.
    "StairW_South": {"line": (-32.44, 14.45, -29.50, 14.45), "t": 0.10, "kind": "stair",
                     "openings": [(-31.93, -30.86, 2.12)]},
    "StairW_East": {"line": (-29.55, 14.40, -29.55, 23.751), "t": 0.10, "kind": "stair",  # = resus west wall (edit b)
                    "openings": []},
    "StairE_West": {"line": (7.40, 10.35, 7.40, 19.350), "t": 0.10, "kind": "stair", "openings": []},
    "StairE_South": {"line": (7.35, 10.40, 11.16, 10.40), "t": 0.10, "kind": "stair",
                     "openings": [(8.37, 9.44, 2.12)]},
    # Down-stairs to the sealed pits: fixed screens (the robot can no longer walk over the pit holes).
    "DownW_Fixed": {"line": (-42.00, 1.751, -42.00, 7.000), "t": 0.10, "kind": "fixed", "openings": []},
    "DownNE_FixedS": {"line": (10.76, 32.15, 18.17, 32.15), "t": 0.10, "kind": "fixed", "openings": []},
    "DownNE_FixedE": {"line": (18.12, 32.10, 18.12, 36.851), "t": 0.10, "kind": "fixed", "openings": []},
    # Protected lobby of the main bed-lift pair (C2): FD30S doorset in the north screen, held open on magnets.
    "LiftLobby_West": {"line": (6.75, -2.75, 6.75, 4.20), "t": 0.10, "kind": "lobby", "openings": []},
    "LiftLobby_North": {"line": (6.70, 4.15, 11.16, 4.15), "t": 0.10, "kind": "lobby",
                        "openings": [(7.00, 8.97, 2.12)]},
    # Lift shafts: the old WC door openings on the east walls (x 14.05..14.16) are filled.
    "ShaftFill_North": {"line": (14.105, 2.556, 14.105, 3.980), "t": 0.11, "kind": "shaft", "openings": []},
    "ShaftFill_South": {"line": (14.105, -0.947, 14.105, 0.476), "t": 0.11, "kind": "shaft", "openings": []},
    # A&E resus (2 bays) south wall with a 1.9 m bed doorway (no leaves: non-fire double doors, not modelled).
    "Resus_South": {"line": (-29.50, 14.45, -23.539, 14.45), "t": 0.10, "kind": "partition",
                    "openings": [(-27.40, -25.50, 2.99)]},  # full height: a ceiling fitting (Light_test34) sits over it
    # Ward pods: nurse workstation | two en-suites (P1 = old room E2, P2 = old room E5), en-suite B6 | cleaners (P3).
    "P1_WS": {"line": (22.20, 21.50, 22.20, 24.90), "t": 0.10, "kind": "partition", "openings": []},
    "P1_Mid": {"line": (22.25, 23.20, 26.891, 23.20), "t": 0.10, "kind": "partition", "openings": []},
    "P2_WS": {"line": (22.20, 10.40, 22.20, 13.799), "t": 0.10, "kind": "partition", "openings": []},
    "P2_Mid": {"line": (22.25, 12.10, 26.891, 12.10), "t": 0.10, "kind": "partition", "openings": []},
    "P3_Mid": {"line": (14.05, 25.20, 14.05, 28.60), "t": 0.10, "kind": "partition", "openings": []},
    # Dirty utility | disposal hold (old room W5): opening without a leaf. The corridor door opens into the dirty utility
    # (east, 12.0 m2); the lockable disposal hold (west, window end, 7.5 m2) is reached through it (design s.6.1).
    # x 13.55 clears both fittings: Light_test14 (x <= 13.04) and Light_test13 (x >= 14.94).
    "DU_Disposal": {"line": (13.55, 10.40, 13.55, 13.80), "t": 0.10, "kind": "partition",
                    "openings": [(12.55, 13.55, 2.10)]},
    # Old office: en-suite B5 | lobby; lobby | assisted shower (1.1 m door); lobby | day room (1.2 m door).
    "Office_EnsuiteS": {"line": (20.862, 0.57, 23.25, 0.57), "t": 0.10, "kind": "partition", "openings": []},
    "Office_Shower": {"line": (23.30, -1.85, 23.30, 2.718), "t": 0.10, "kind": "partition",
                      "openings": [(-1.55, -0.45, 2.10)]},
    "Office_DayRoom": {"line": (20.862, -1.85, 25.469, -1.85), "t": 0.10, "kind": "partition",
                       "openings": [(21.10, 22.30, 2.10)]},
    # Two public WCs (accessible + standard) at the east end of the south corridor, next to the A&E waiting.
    "WC_Front": {"line": (-16.50, 3.50, -11.70, 3.50), "t": 0.10, "kind": "wc",
                 "openings": [(-15.45, -14.50, 2.10), (-13.60, -12.65, 2.10)]},
    "WC_West": {"line": (-16.45, 1.751, -16.45, 3.45), "t": 0.10, "kind": "wc", "openings": []},
    "WC_Mid": {"line": (-14.00, 1.751, -14.00, 3.45), "t": 0.10, "kind": "wc", "openings": []},
    "WC_East": {"line": (-11.75, 1.751, -11.75, 3.45), "t": 0.10, "kind": "wc", "openings": []},
}

# Wall cuts for the MuJoCo extractor (g1-mujoco-sim/extract_scene_boxes.py): world rects (x0, y0, x1, y1) through
# the full wall thickness, removed from every extracted building wall box (any thickness). Needed because the hidden
# DobleDoor prims no longer mark these holes, and the courtyard walls are 0.6 m thick (never cut today).
BUILDING_OPENINGS = {
    "D07": (-41.577, 32.80, -39.093, 32.96),
    "D34": (20.73, 29.548, 20.89, 32.025),
    "D13": (-38.36, 7.711, -37.72, 10.160),
    "D14": (-32.76, 7.692, -32.12, 10.110),
}

# ---------------------------------------------------------------- fire doors in the building
# Same schema as FIRE_DOORS plus: "frame": "world" (build_spec subtracts WING_ORIGIN), "replaces" (asset prims
# hidden, listed in BUILDING_HIDE too), "transom" (full-height panel over the leaves up to "ceiling"),
# "swing_limit_deg" (building jambs are not in the moving parts' collision filter: limit the swing so the leaf
# clears the jamb, see design_final.md s.7), "tier" (CPU budget: 1 = always; 2 = HG1_FIXTURES=tier2, the default;
# 3 = HG1_FIXTURES=full), "opening" (BUILDING_OPENINGS key of the wall cut the doorset fills), "use" (free text).
# One-leaf doors go to the tensor-view group /World/Fixtures/Doors1 (identical structure inside the group).
# Hinges: 30 mm off the jamb faces (measured: 1.243 m between the jambs of every single door frame).
# HTM 05-02: 5.40 and Table 6 (fire-hazard rooms: FD30S), App C (self-closing, BS EN 1154; held open only on
# fail-safe magnets, BS 7273-4; double-swing where escape runs both ways), 3.59 (automatic doors on escape routes
# open on power loss). AD M vol 2 3.10(a): opening force at the leading edge <= 30 N (0-30 deg), <= 22.5 N (30-60 deg).
_FD30_W = 1.19
BUILDING_FIRE_DOORS = {
    # Compartment doors at the two ward entrances: FD60S, double-swing, held open on fail-safe magnets. Hinge-to-hinge
    # 1.91 m: effective clear width 1.80 m at 90 deg (HBN 00-04 8.21: >= 1740 mm on bed routes).
    "W_Compartment": {"rating": "FD60S", "leaves": [((-35.50, 11.50), "+y", 0.955), ((-35.50, 13.41), "-y", 0.955)],
                      "swing": "both", "hold": "+x", "bolted": [], "transom": True, "ceiling": BUILDING_CEILING_Z,
                      "frame": "world", "replaces": [], "tier": 1, "swing_limit_deg": 100},
    "E_Compartment": {"rating": "FD60S", "leaves": [((11.21, 6.30), "+y", 0.955), ((11.21, 8.21), "-y", 0.955)],
                      "swing": "both", "hold": "+x", "bolted": [], "transom": True, "ceiling": BUILDING_CEILING_Z,
                      "frame": "world", "replaces": [], "tier": 1, "swing_limit_deg": 100},
    # Up-stair doors: FD30S, self-closing, never held open (HTM 05-02 App C: hold None), opening out of the stair.
    "W_Stair": {"rating": "FD30S", "leaves": [((-31.90, 14.45), "+x", 1.00)], "swing": "-y", "hold": None,
                "bolted": [], "transom": True, "ceiling": BUILDING_CEILING_Z, "frame": "world", "replaces": [],
                "tier": 1, "swing_limit_deg": 100},
    "E_Stair": {"rating": "FD30S", "leaves": [((8.40, 10.40), "+x", 1.00)], "swing": "-y", "hold": None,
                "bolted": [], "transom": True, "ceiling": BUILDING_CEILING_Z, "frame": "world", "replaces": [],
                "tier": 1, "swing_limit_deg": 100},
    # Main lift lobby door: FD30S, held open on magnets, opens out of the lobby, so its swing never enters the lobby
    # (HTM 05-02 5.36: >= 4100 mm clear beyond the door swing; 6.85 m along the door axis).
    "Main_LiftLobby": {"rating": "FD30S", "leaves": [((7.03, 4.15), "+x", 0.955), ((8.94, 4.15), "-x", 0.955)],
                       "swing": "+y", "hold": "+y", "bolted": [], "transom": True, "ceiling": BUILDING_CEILING_Z,
                       "frame": "world", "replaces": [], "tier": 2, "swing_limit_deg": 100},
    # Final exits replacing closed double doors (opening out, push bar, no fire rating): D07 west ward north end,
    # D34 NE stair hall. Holes measured on the wall prims: D07 x -41.577..-39.093, D34 y 29.548..32.025.
    "D07_Exit": {"rating": "EXIT", "leaves": [((-41.547, 32.881), "+x", 1.212), ((-39.123, 32.881), "-x", 1.212)],
                 "swing": "+y", "hold": None, "bolted": [], "transom": False, "frame": "world",
                 "replaces": ["Geo_M_DobleDoor3", "Geo_M_DobleDoor7"], "tier": 2, "swing_limit_deg": 92,
                 "opening": "D07"},
    "D34_Exit": {"rating": "EXIT", "leaves": [((20.81, 29.578), "+y", 1.2085), ((20.81, 31.995), "-y", 1.2085)],
                 "swing": "+x", "hold": None, "bolted": [], "transom": False, "frame": "world",
                 "replaces": ["Geo_M_DobleDoor2", "Geo_M_DobleDoor8"], "tier": 2, "swing_limit_deg": 92,
                 "opening": "D34"},
    # Single FD30S doors in existing frames on fire-hazard rooms (HTM 05-02 5.40, Table 6). hold None everywhere.
    # Locked/kept shut: dirty utilities, cleaners, store. Self-closing: linen (clean utilities), pharmacies.
    "AE_DirtyUtility": {"rating": "FD30S", "leaves": [((-38.476, 30.426), "+y", _FD30_W)], "swing": "+x",
                        "hold": None, "bolted": [], "frame": "world", "replaces": ["SM_Door_01c5"], "tier": 3,
                        "swing_limit_deg": 92, "use": "locked shut"},
    "AE_CleanLinen": {"rating": "FD30S", "leaves": [((-38.476, 26.697), "+y", _FD30_W)], "swing": "+x",
                      "hold": None, "bolted": [], "frame": "world", "replaces": ["SM_Door_01c4"], "tier": 3,
                      "swing_limit_deg": 92, "use": "self-closing"},
    # Named Central_Store, not Store: FIRE_DOORS["Store"] (the wing's sterile store) has that name already.
    "Central_Store": {"rating": "FD30S", "leaves": [((-21.490, 12.899), "+x", _FD30_W)], "swing": "+y",
                      "hold": None, "bolted": [], "frame": "world", "replaces": ["SM_Door_01b2"], "tier": 3,
                      "swing_limit_deg": 92, "use": "locked shut"},
    "Pharmacy1": {"rating": "FD30S", "leaves": [((-9.694, 8.770), "+x", _FD30_W)], "swing": "-y", "hold": None,
                  "bolted": [], "frame": "world", "replaces": ["SM_Door_01b5_8"], "tier": 3, "swing_limit_deg": 92,
                  "use": "self-closing"},
    "Pharmacy2": {"rating": "FD30S", "leaves": [((-2.699, 8.770), "+x", _FD30_W)], "swing": "-y", "hold": None,
                  "bolted": [], "frame": "world", "replaces": ["SM_Door_01b6"], "tier": 3, "swing_limit_deg": 92,
                  "use": "self-closing"},
    "Ward_Cleaners": {"rating": "FD30S", "leaves": [((17.192, 26.291), "+y", _FD30_W)], "swing": "-x", "hold": None,
                      "bolted": [], "frame": "world", "replaces": ["SM_Door_02b12"], "tier": 3,
                      "swing_limit_deg": 92, "use": "locked shut"},
    "Ward_CleanLinen": {"rating": "FD30S", "leaves": [((17.192, 18.908), "+y", _FD30_W)], "swing": "-x",
                        "hold": None, "bolted": [], "frame": "world", "replaces": ["SM_Door_02b10"], "tier": 3,
                        "swing_limit_deg": 92, "use": "self-closing"},
    "Ward_DirtyUtility": {"rating": "FD30S", "leaves": [((17.192, 11.506), "+y", _FD30_W)], "swing": "-x",
                          "hold": None, "bolted": [], "frame": "world", "replaces": ["SM_Door_02b8"], "tier": 3,
                          "swing_limit_deg": 92, "use": "locked shut (disposal hold behind)"},
}

# ---------------------------------------------------------------- lifts: per-bank data
# The wing bank is unchanged (LIFTS, LIFT_CALL, static names fx_front_i ...). The main bank is new: a duplex pair of
# bed lifts in the two old WCs, facing -x into a protected lobby, same orientation as the wing's (no "facing" code).
# Front x 11.10 (4 cm clear of the WC side walls' west ends at 11.16). Cars x 11.22..13.92 (front + 0.12 + 2.7),
# y -2.00..-0.20 and 1.50..3.30; backs end at x 13.97 < the WC east walls' inner face (14.05); the WC middle wall
# (y 0.55..0.75) stands between the cars.
# HTM 08-02 4.19 and 5.23 (duplex bed lifts, car 1800 x 2700 mm, LIFT_SPEC unchanged), 4.12 (fire recall: the
# controller opens and holds the cars on the alarm). HBN 00-04 6.19-6.20 and Fig 28: protected lobby, bed-turning
# space 4310 x 3370 mm per car. AD M vol 2 3.34: landing call button 900-1100 mm high, >= 500 mm from a return wall.
BUILDING_LIFTS = {"MainLift1": {"front_x": 11.10, "center_y": -1.10, "frame": "world", "tier": 1},
                  "MainLift2": {"front_x": 11.10, "center_y": 2.40, "frame": "world", "tier": 1}}
BUILDING_LIFT_CALL = {"MainCall": {"pos": (11.10, 0.65, 1.0), "lifts": ["MainLift1", "MainLift2"],
                                   "axis": (1.0, 0.0, 0.0), "frame": "world", "tier": 1}}
LIFT_BANKS = {
    # front: (front_x, y_lo, y_hi) of the lift front wall (lobby face), in the bank's frame; lobby: (x0, y0, x1, y1)
    # of the lift lobby, in the bank's frame; ceiling_z: underside of the ceiling over the front (the front boxes
    # stop at 3.0 m; a bulkhead fills 3.0..ceiling_z when ceiling_z > 3.0); prefix: name prefix of the bank's front
    # and header static boxes (fx_<prefix>front_i, fx_<prefix>header_i). The wing prefix is "" (names unchanged).
    # Both banks face -x (lobby on -x, cars on +x): the lift code has no "facing" key.
    "Wing": {"frame": "wing", "lifts": ["Lift1", "Lift2"], "calls": ["LobbyCall"], "front": (20.0, -12.0, -4.0),
             "ceiling_z": 3.6, "prefix": "", "lobby": (14.0, -12.0, 20.0, -4.0)},
    "Main": {"frame": "world", "lifts": ["MainLift1", "MainLift2"], "calls": ["MainCall"],
             "front": (11.10, -2.73, 4.10), "ceiling_z": 3.0, "prefix": "Main_", "lobby": (6.80, -2.75, 11.10, 4.10)},
}

# ---------------------------------------------------------------- rooms (new uses) and dressing
# Interior rects measured with pxr ray casts at z = 1 m on the wall meshes (scripts/rooms1.py). Room-local u, v run
# from (x0, y0). Dressing goes to /World/Building/Dressing/<room> (no wing transform), Room(..., wall_t=0.0).
# Room.put/Room.wall need a new keyword scale= (building WC fittings are authored at x1.31).
BUILDING_ROOMS = {
    # ---- C3: 6-bed adult in-patient ward (HBN 04-01) in the east corridor
    "Ward_B1": {"rect": (20.869, 25.202, 26.751, 28.577), "use": "single bedroom B1 (old E1)", "comp": "C3"},
    "Ward_WS1": {"rect": (20.862, 21.516, 22.150, 24.891), "use": "nurse workstation 1 + resus trolley bay", "comp": "C3"},
    "Ward_ES_B1": {"rect": (22.250, 23.250, 26.751, 24.891), "use": "en-suite B1", "comp": "C3"},
    "Ward_ES_B2": {"rect": (22.250, 21.516, 26.751, 23.150), "use": "en-suite B2", "comp": "C3"},
    "Ward_B2": {"rect": (20.861, 17.811, 26.751, 21.186), "use": "single bedroom B2 (old E3)", "comp": "C3"},
    "Ward_B3": {"rect": (20.869, 14.116, 26.751, 17.491), "use": "single bedroom B3 (old E4)", "comp": "C3"},
    "Ward_WS2": {"rect": (20.861, 10.413, 22.150, 13.787), "use": "nurse workstation 2 + hoist/scales bay", "comp": "C3"},
    "Ward_ES_B3": {"rect": (22.250, 12.150, 26.751, 13.787), "use": "en-suite B3", "comp": "C3"},
    "Ward_ES_B4": {"rect": (22.250, 10.413, 26.751, 12.050), "use": "en-suite B4", "comp": "C3"},
    "Ward_B4": {"rect": (20.861, 6.728, 26.751, 10.074), "use": "single bedroom B4 (old E6)", "comp": "C3"},
    "Ward_B5": {"rect": (20.861, 3.013, 26.751, 6.373), "use": "single bedroom B5 (old E7)", "comp": "C3"},
    "Ward_ES_B5": {"rect": (20.862, 0.620, 23.250, 2.718), "use": "en-suite B5 (in the old office)", "comp": "C3"},
    "Ward_OfficeLobby": {"rect": (20.862, -1.800, 23.250, 0.520), "use": "lobby: assisted shower + day room", "comp": "C3"},
    "Ward_AssistedShower": {"rect": (23.350, -1.800, 25.358, 2.718), "use": "assisted shower/WC", "comp": "C3"},
    "Ward_DayRoom": {"rect": (20.862, -4.218, 25.358, -1.900), "use": "day room", "comp": "C3"},
    "Ward_ES_B6": {"rect": (11.269, 25.209, 14.000, 28.584), "use": "en-suite B6 (old W1, window end)", "comp": "C3"},
    "Ward_Cleaners": {"rect": (14.100, 25.209, 17.159, 28.584), "use": "cleaners' room (FD30S locked)", "comp": "C3"},
    "Ward_B6": {"rect": (11.269, 21.513, 17.158, 24.887), "use": "single bedroom B6 (old W2)", "comp": "C3"},
    "Ward_CleanUtility": {"rect": (11.269, 17.813, 17.159, 21.187), "use": "clean utility + medicines + linen", "comp": "C3"},
    "Ward_Pantry": {"rect": (11.269, 14.109, 17.159, 17.484), "use": "ward pantry", "comp": "C3"},
    "Ward_DirtyUtility": {"rect": (13.600, 10.413, 17.155, 13.787), "use": "dirty utility (corridor door FD30S)",
                          "comp": "C3"},
    "Ward_Disposal": {"rect": (11.269, 10.413, 13.500, 13.787), "use": "disposal hold (through the dirty utility)",
                      "comp": "C3"},
    "Ward_Entrance": {"rect": (11.270, 4.150, 20.760, 10.250), "use": "ward entrance, reception + nurse base", "comp": "C3"},
    # ---- C1: A&E majors (west corridor)
    "AE_Majors1": {"rect": (-48.023, 29.311, -42.141, 32.685), "use": "majors cubicle 1 (old W-row 1)", "comp": "C1"},
    "AE_Majors2": {"rect": (-48.023, 25.608, -42.141, 28.983), "use": "majors cubicle 2", "comp": "C1"},
    "AE_Majors3": {"rect": (-48.023, 21.908, -42.141, 25.283), "use": "majors cubicle 3", "comp": "C1"},
    "AE_StaffBase": {"rect": (-48.140, 18.050, -42.130, 21.750), "use": "A&E staff base (open bay)", "comp": "C1"},
    "AE_Majors4": {"rect": (-48.023, 14.519, -42.141, 17.893), "use": "majors cubicle 4", "comp": "C1"},
    "AE_Majors5": {"rect": (-48.023, 10.808, -42.141, 14.183), "use": "majors cubicle 5", "comp": "C1"},
    "AE_Isolation": {"rect": (-48.023, 7.109, -42.141, 10.484), "use": "isolation room (opposite the ambulance door)", "comp": "C1"},
    "AE_DirtyUtility": {"rect": (-38.431, 29.304, -32.549, 32.687), "use": "A&E dirty utility (FD30S locked)", "comp": "C1"},
    "AE_CleanUtility": {"rect": (-38.431, 25.614, -32.549, 28.989), "use": "A&E clean utility + linen (FD30S)", "comp": "C1"},
    "AE_Majors6": {"rect": (-38.431, 21.914, -32.549, 25.289), "use": "majors cubicle 6", "comp": "C1"},
    "AE_Majors7": {"rect": (-38.431, 18.208, -32.549, 21.583), "use": "majors cubicle 7", "comp": "C1"},
    "AE_Relatives": {"rect": (-38.414, 14.508, -32.549, 17.883), "use": "relatives' room", "comp": "C1"},
    # ---- C2: A&E resus / handover / X-ray in the west hall, walk-in side off the main lobby
    "AE_Resus": {"rect": (-29.500, 14.500, -23.539, 23.751), "use": "resus, 2 bays", "comp": "C2"},
    "AE_Handover": {"rect": (-32.000, 5.400, -26.500, 7.550), "use": "ambulance handover bay (open)", "comp": "C2"},
    "AE_XRay": {"rect": (-24.706, 5.508, -14.466, 8.693), "use": "X-ray room (old lightbox room)", "comp": "C2"},
    "AE_Waiting": {"rect": (-11.635, -2.749, -5.640, 1.750), "use": "A&E walk-in reception + 15-seat waiting", "comp": "C2"},
    "AE_Triage": {"rect": (-5.913, 12.508, -2.667, 15.692), "use": "triage", "comp": "C2"},
    "AE_Minors1": {"rect": (-2.413, 12.508, 0.833, 15.692), "use": "minors 1", "comp": "C2"},
    "AE_Minors2": {"rect": (1.087, 12.508, 4.333, 15.692), "use": "minors 2", "comp": "C2"},
    "WC_Accessible": {"rect": (-16.400, 1.751, -14.050, 3.450), "use": "public accessible WC (AD M)", "comp": "C2"},
    "WC_Public": {"rect": (-13.950, 1.751, -11.800, 3.450), "use": "public WC", "comp": "C2"},
    "Staff_WC": {"rect": (-19.936, 12.960, -16.643, 15.738), "use": "staff WC", "comp": "C2"},
    "Central_Store": {"rect": (-23.436, 12.960, -20.142, 15.738), "use": "store (FD30S locked)", "comp": "C2"},
    "Main_LiftLobby": {"rect": (6.800, -2.750, 11.100, 4.100), "use": "protected bed-lift lobby", "comp": "C2"},
}

# Ceiling lights for the new sub-rooms without a fitting (rect lights just under the ceiling, the kit's per-area level
# of build_hospital._panels): name -> (centre x, y, panel size m). A 0.6 m panel mesh can be added for looks.
BUILDING_LIGHTS = {
    "WS1": (21.51, 23.20, 0.6), "ES_B1": (24.50, 24.07, 0.6), "ES_B2": (24.50, 22.33, 0.6),
    "WS2": (21.51, 12.10, 0.6), "ES_B3": (24.50, 12.97, 0.6), "ES_B4": (24.50, 11.23, 0.6),
    "OfficeLobby": (22.05, -0.64, 0.6), "WC_Accessible": (-15.22, 2.60, 0.6), "WC_Public": (-12.87, 2.60, 0.6),
}
BUILDING_LIGHT_Z = 2.95

# New asset keys for assets.build_catalog() (all in the Isaac hospital Props folder of the lite mirror, verified):
NEW_ASSET_KEYS = {"SM_Toilet": "Props/SM_Toilet.usd", "SM_ToiletPaper": "Props/SM_ToiletPaper.usd",
                  "SM_Mirror": "Props/SM_Mirror.usd", "SM_MopSet_01a": "Props/SM_MopSet_01a.usd",
                  "SM_MopSet_01b": "Props/SM_MopSet_01b.usd", "SM_DisposalStand_02": "Props/SM_DisposalStand_02.usd"}
BACK_UPDATE = {  # merge into BACK
    "SM_Toilet": "-x", "SM_ToiletPaper": "-x", "SM_Mirror": "-x", "SM_MopSet_01a": "-y", "SM_MopSet_01b": "-y",
    "SM_DisposalStand_02": "-y", "SM_Chair_02a": "+y", "SM_SideTable_02a": "-y",
}
WC_SCALE = 1.31  # the building's own WC fittings are authored at x1.31


def _bed_room(r, pod_wall, far_wall):
    """East-row bedroom (6.04 x 3.40 m): bed space 3.6 m at the window end (u 2.44..6.04), entrance zone at the
    corridor end with the en-suite door (pod wall, u 1.45..2.40) and the clinical hand-wash basin (far wall)."""
    r.wall("Bed", "SM_HospitalBed_01b", far_wall, 4.24)
    r.wall("BedsideTable", "SM_BedSideTable_01b", far_wall, 5.40)
    r.wall("Chair", "SM_Chair_01a", far_wall, 2.95)
    r.wall("Washbasin", "SM_Washbasin", far_wall, 0.75, z=0.78)
    r.wall("Soap", "SM_SoapDispenser_01a", far_wall, 1.40, z=1.3)
    r.wall("Towels", "SM_Toweldispenser_01a", far_wall, 1.85, z=1.25)
    r.wall("Oxygen", "SM_GasCart_01c", pod_wall, 5.6)


def dress_ward_b1(r): _bed_room(r, "S", "N")
def dress_ward_b2(r): _bed_room(r, "N", "S")
def dress_ward_b3(r): _bed_room(r, "S", "N")
def dress_ward_b4(r): _bed_room(r, "N", "S")
def dress_ward_b5(r): _bed_room(r, "S", "N")


def dress_ward_b6(r):
    # West row (5.97 x 3.40): corridor door on the E wall, en-suite door on the N wall at the window end (u 0.35..1.30),
    # bed space in the middle (u 1.6..5.2).
    r.wall("Bed", "SM_HospitalBed_01b", "S", 3.0)
    r.wall("BedsideTable", "SM_BedSideTable_01b", "S", 1.95)
    r.wall("Chair", "SM_Chair_01a", "S", 4.2)
    r.wall("Washbasin", "SM_Washbasin", "N", 4.6, z=0.78)
    r.wall("Soap", "SM_SoapDispenser_01a", "N", 4.0, z=1.3)
    r.wall("Towels", "SM_Toweldispenser_01a", "N", 5.25, z=1.25)
    r.wall("Oxygen", "SM_GasCart_01c", "N", 2.2)


def _ensuite(r, door_wall, toilet_wall, basin_wall, t_u, b_u):
    r.wall("Toilet", "SM_Toilet", toilet_wall, t_u, z=0.30, scale=WC_SCALE)
    r.wall("ToiletPaper", "SM_ToiletPaper", toilet_wall, t_u + 0.55, z=0.85, scale=WC_SCALE)
    r.wall("Washbasin", "SM_Washbasin", basin_wall, b_u, z=0.78)
    r.wall("Mirror", "SM_Mirror", basin_wall, b_u, z=1.40)


def dress_es_b1(r): _ensuite(r, "N", "E", "S", 0.80, 2.60)
def dress_es_b2(r): _ensuite(r, "S", "E", "N", 0.80, 2.60)
def dress_es_b3(r): _ensuite(r, "N", "E", "S", 0.80, 2.60)
def dress_es_b4(r): _ensuite(r, "S", "E", "N", 0.80, 2.60)
def dress_es_b5(r): _ensuite(r, "N", "W", "S", 1.05, 1.10)
def dress_es_b6(r): _ensuite(r, "S", "N", "E", 1.80, 1.90)


def dress_ws1(r):
    # 1.30 m deep touchdown base: wall terminal on the partition between the two vision panels (it stood in front
    # of the B1 panel), resus trolley parking bay (2 m2).
    r.wall("Terminal", "SM_Computer_02b", "E", 1.69, z=1.00)
    r.wall("ResusTrolley", "SM_SupplyCart_02a", "S", 0.65)
    r.put("Defibrillator", "Defibrillator_A", 0.65, 0.25, z=0.80)


def dress_ws2(r):
    r.wall("Terminal", "SM_Computer_02b", "E", 1.69, z=1.00)  # on the partition, clear of both vision panels
    r.wall("Scale", "SM_Scale_01a", "S", 0.40)
    r.wall("HoistStandIn", "Wheelchair_C", "W", 2.90)  # no hoist asset: stand-in for the hoist/scales bay


def dress_assisted_shower(r):
    r.wall("Toilet", "SM_Toilet", "E", 3.6, z=0.30, scale=WC_SCALE)
    r.wall("ToiletPaper", "SM_ToiletPaper", "E", 4.15, z=0.85, scale=WC_SCALE)
    r.wall("Washbasin", "SM_Washbasin", "N", 0.95, z=0.78)
    r.wall("Mirror", "SM_Mirror", "N", 0.95, z=1.40)
    r.wall("ShowerChair", "Wheelchair_C", "E", 1.2)


def dress_day_room(r):
    r.wall("Bench", "SM_Chair_02a", "S", 1.50)
    r.wall("SideTable", "SM_SideTable_02a", "S", 3.40)
    r.wall("Chair0", "SM_Chair_01a", "N", 2.75)
    r.wall("Chair1", "SM_Chair_01a", "N", 3.75)


def dress_cleaners(r):
    r.wall("MopA", "SM_MopSet_01a", "S", 0.60)
    r.wall("MopB", "SM_MopSet_01b", "S", 1.45)
    r.wall("WetFloor", "S_WetFloorSign", "S", 2.20)
    r.wall("Sink", "SM_Washbasin", "W", 1.70, z=0.78)


def dress_clean_utility(r):
    r.wall("MedCabinet", "SM_MedicalCabinet_01a", "S", 1.0)
    r.wall("MedFridge", "Refrigerator_A", "S", 2.2)
    r.wall("LinenTrolley", "SM_SupplyCart_02a", "S", 3.4)
    r.wall("Washbasin", "SM_Washbasin", "W", 1.0, z=0.78)


def dress_pantry(r):
    r.wall("Fridge", "Refrigerator_A", "S", 0.8)
    r.wall("Water", "SM_WaterDispenser_01a", "S", 1.75)
    r.wall("Worktop", "SM_Desk_04a", "S", 3.2)
    r.wall("Bin", "SM_TrashCan", "S", 4.4)


def dress_dirty_utility(r):
    # 3.56 x 3.37 m next to the corridor. Keep clear: the corridor door swing (x >= 16.0) and the opening to the
    # disposal hold (W wall, v 2.14..3.14).
    r.wall("Sluice", "SM_Washbasin", "S", 0.65, z=0.78)
    r.wall("Shelves", "SM_ShelfSet_01a", "N", 1.86)
    r.wall("Bin0", "SM_TrashCan", "S", 1.60)
    r.wall("Bin1", "SM_TrashCan", "S", 2.00)


def dress_disposal(r):
    r.wall("Bin0", "SM_DisposalStand_02", "S", 0.45)
    r.wall("Bin1", "SM_DisposalStand_02", "S", 1.15)


def dress_ward_entrance(r):
    # Reception + nurse base, 1.49 m low counter facing north up the ward corridor: from the chair (17.35, 5.9) the
    # far end of the corridor (19.0, 28.5) is visible past the corridor's west corner (17.27, 10.25).
    r.put("Counter", "SM_Desk_04a", 6.08, 2.40, yaw=0)
    r.put("Computer", "SM_Computer_02b", 6.08, 2.45, z=0.83)
    r.put("Chair", "SM_Chair_01a", 6.08, 1.75, yaw=0)


def _majors(r, window_end_u, yaw, door_wall):
    r.put("Trolley", "SM_HospitalBed_02d", window_end_u, 2.67, yaw=yaw)
    r.wall("Monitor", "VitalPad_A", "N", window_end_u, z=1.55)
    r.wall("CarerChair0", "SM_Chair_01a", "S", 2.8)
    r.wall("CarerChair1", "SM_Chair_01a", "S", 3.65)
    basin_u = 5.10 if door_wall == "E" else 0.90
    r.wall("Washbasin", "SM_Washbasin", "S", basin_u, z=0.78)
    r.wall("Soap", "SM_SoapDispenser_01a", "S", basin_u + (-0.65 if door_wall == "E" else 0.65), z=1.3)


def dress_majors_w(r): _majors(r, 1.56, -90, "E")    # west row: door on the E wall, window end at u 0
def dress_majors_w_keepbed(r):                       # majors 3/4 keep the building's bed (same pose)
    r.wall("Monitor", "VitalPad_A", "N", 1.56, z=1.55)
    r.wall("CarerChair0", "SM_Chair_01a", "S", 2.8)
    r.wall("CarerChair1", "SM_Chair_01a", "S", 3.65)
    r.wall("Washbasin", "SM_Washbasin", "S", 5.10, z=0.78)
    r.wall("Soap", "SM_SoapDispenser_01a", "S", 4.45, z=1.3)
def dress_majors_e(r): _majors(r, 4.36, 90, "W")     # east row: door on the W wall


def dress_isolation(r):
    r.put("Bed", "SM_HospitalBed_02d", 1.56, 2.67, yaw=-90)
    r.wall("Monitor", "VitalPad_A", "N", 1.56, z=1.55)
    r.wall("Washbasin", "SM_Washbasin", "S", 3.4, z=0.78)
    r.wall("Gloves", "Glovebox_A", "S", 4.3, z=1.3)
    r.wall("Bin", "SM_TrashCan", "S", 4.9)


def dress_relatives(r):
    r.wall("Sofa", "SM_Chair_02a", "N", 3.0)
    r.wall("SideTable", "SM_SideTable_02a", "N", 4.9)
    r.wall("Chair0", "SM_Chair_01a", "S", 2.6)
    r.wall("Chair1", "SM_Chair_01a", "S", 3.5)
    r.wall("Water", "SM_WaterDispenser_01a", "S", 5.3)


def dress_ae_dirty(r):
    r.wall("Sluice", "SM_Washbasin", "N", 1.2, z=0.78)
    r.wall("Shelves", "SM_ShelfSet_01a", "S", 3.4)
    r.wall("Bin0", "SM_DisposalStand_02", "E", 0.7)
    r.wall("Bin1", "SM_DisposalStand_02", "E", 1.5)


def dress_ae_clean(r):
    r.wall("MedCabinet", "SM_MedicalCabinet_01a", "S", 2.6)
    r.wall("LinenTrolley", "SM_SupplyCart_02a", "S", 4.0)


def dress_ae_staffbase(r):
    for i, (wall, u) in enumerate((("N", 4.3), ("S", 3.1))):
        r.wall(f"Desk{i}", "SM_Desk_04a", wall, u)
        r.wall(f"Computer{i}", "SM_Computer_02b", wall, u, z=0.83)
    r.put("Chair0", "SM_Chair_01a", 4.3, 2.75, yaw=0)
    r.put("Chair1", "SM_Chair_01a", 3.1, 1.00, yaw=180)


def dress_resus(r):
    # 5.96 x 9.25 m, two bays split by a sliding screen at v 4.65; trolleys free-standing for 360 deg access.
    # Each bay: trolley, wall monitor, a bed-head services panel on the W wall (wall panel as a stand-in for the
    # O2 / air / vacuum terminal units, a suction-jar look-alike below it), a cylinder cart, a procedure trolley
    # on the E wall and its own crash trolley (bay 1 at the bay split, bay 2 at the N wall). No pendant asset.
    for i, v in enumerate((2.30, 7.00)):
        r.put(f"Trolley{i}", "SM_Gurney_01b", 2.96, v, yaw=90)
        r.wall(f"Monitor{i}", "VitalPad_A", "W", v, z=1.55)  # wall-mounted at the bed head (was free in the air)
        r.wall(f"Oxygen{i}", "SM_GasCart_01c", "W", v + 1.3)
        r.wall(f"GasPanel{i}", "SM_Intercom_01a", "W", v - 0.62, z=1.30)
        r.wall(f"Suction{i}", "SM_Container_01a", "W", v - 0.62, z=1.05)
        r.wall(f"ProcTrolley{i}", "SM_SupplyCart_02a", "E", (2.00, 7.60)[i])
    r.put("BayScreen", "HospitalScreen_A", 1.60, 4.65, yaw=0)
    r.wall("CrashCart", "EmergencyTrolley_A", "E", 4.65)
    r.wall("CrashCart1", "EmergencyTrolley_A", "N", 1.20)
    r.wall("Defibrillator", "Defibrillator_A", "E", 6.2, z=1.3)
    r.wall("Cabinet", "SM_SupplyCabinet_01c", "N", 3.2)
    r.wall("Washbasin", "SM_Washbasin", "N", 4.85, z=0.78)
    r.wall("Soap", "SM_SoapDispenser_01a", "N", 5.5, z=1.3)


def dress_handover(r):
    # Open bay south of the ambulance doorway: private information desk, the two gurneys (edit c), charging bay.
    r.put("Gurney0", "SM_Gurney_01a", 1.35, 0.55, yaw=0)
    r.put("Gurney1", "SM_Gurney_01a", 1.35, 1.50, yaw=0)
    r.put("Desk", "SM_Desk_04a", 4.20, 1.10, yaw=90)
    r.put("Computer", "SM_Computer_02b", 4.20, 1.10, z=0.83, yaw=90)
    r.put("Chair", "SM_Chair_01a", 4.85, 1.10, yaw=90)
    r.put("ChargingBay", "Wheelchair_C", 3.10, 0.50, yaw=0)


def dress_xray(r):
    r.put("Table", "SM_Gurney_01a", 6.0, 1.60, yaw=0)
    r.wall("Control", "SM_Desk_04a", "S", 9.1)
    r.wall("ControlPC", "SM_Computer_02b", "S", 9.1, z=0.83)
    r.put("ControlChair", "SM_Chair_01a", 9.1, 1.05, yaw=180)
    r.wall("Lightbox", "SM_Xraylightbox_01a", "E", 1.60, z=1.62)


def dress_ae_waiting(r):
    # 15 seats: the two existing 3-seat benches (SM_Chair_02a4/5) + three new ones; reception counter on the north.
    r.put("Bench0", "SM_Chair_02a", 1.34, 0.46, yaw=180)
    r.put("Bench1", "SM_Chair_02a", 1.34, 2.05, yaw=0)
    r.put("Bench2", "SM_Chair_02a", 4.09, 0.46, yaw=180)
    r.put("Reception", "SM_Desk_04a", 1.34, 3.95, yaw=180)
    r.put("ReceptionPC", "SM_Computer_02b", 1.34, 3.95, z=0.83, yaw=180)


def dress_triage(r):
    r.wall("Desk", "SM_Desk_04a", "N", 1.60)
    r.wall("Computer", "SM_Computer_02b", "N", 1.60, z=0.83)
    r.put("Chair", "SM_Chair_01a", 1.60, 2.15, yaw=0)
    r.put("PatientChair", "SM_Chair_01a", 2.60, 1.45, yaw=90)
    r.wall("Scale", "SM_Scale_01a", "W", 1.00)
    r.wall("Washbasin", "SM_Washbasin", "E", 2.40, z=0.78)
    r.wall("Monitor", "VitalPad_A", "E", 1.20, z=1.30)


def dress_minors(r):
    r.wall("Couch", "SM_Gurney_01a", "N", 1.62)
    r.wall("Chair", "SM_Chair_01a", "W", 1.50)
    r.wall("Washbasin", "SM_Washbasin", "E", 1.40, z=0.78)
    r.wall("Soap", "SM_SoapDispenser_01a", "E", 2.10, z=1.3)


def dress_wc_accessible(r):
    r.wall("Toilet", "SM_Toilet", "W", 0.60, z=0.30, scale=WC_SCALE)
    r.wall("ToiletPaper", "SM_ToiletPaper", "W", 1.15, z=0.85, scale=WC_SCALE)
    r.wall("Washbasin", "SM_Washbasin", "S", 1.75, z=0.78)
    r.wall("Mirror", "SM_Mirror", "S", 1.75, z=1.40)


def dress_wc_public(r):
    r.wall("Toilet", "SM_Toilet", "E", 0.60, z=0.30, scale=WC_SCALE)
    r.wall("ToiletPaper", "SM_ToiletPaper", "E", 1.15, z=0.85, scale=WC_SCALE)
    r.wall("Washbasin", "SM_Washbasin", "S", 0.80, z=0.78)


def dress_staff_wc(r):
    r.wall("Toilet", "SM_Toilet", "N", 0.80, z=0.30, scale=WC_SCALE)
    r.wall("ToiletPaper", "SM_ToiletPaper", "N", 1.35, z=0.85, scale=WC_SCALE)
    r.wall("Washbasin", "SM_Washbasin", "W", 1.80, z=0.78)
    r.wall("Mirror", "SM_Mirror", "W", 1.80, z=1.40)


def dress_central_store(r):
    r.wall("Shelves", "SM_ShelfSet_01a", "N", 1.00)
    r.wall("Shelf", "SM_Shelf_06f", "E", 2.20)


def dress_main_lift_lobby(r):
    # Light on purpose: two 4.31 x 3.37 m bed-turning spaces in front of the lifts (HBN 00-04 Fig 28).
    r.wall("SignLifts", "SignSideWall", "W", 3.2, z=2.2)
    r.wall("Sanitizer", "SM_SoapDispenser_01a", "W", 2.5, z=1.1)


BUILDING_DRESSING = {
    "Ward_B1": dress_ward_b1, "Ward_B2": dress_ward_b2, "Ward_B3": dress_ward_b3, "Ward_B4": dress_ward_b4,
    "Ward_B5": dress_ward_b5, "Ward_B6": dress_ward_b6,
    "Ward_ES_B1": dress_es_b1, "Ward_ES_B2": dress_es_b2, "Ward_ES_B3": dress_es_b3, "Ward_ES_B4": dress_es_b4,
    "Ward_ES_B5": dress_es_b5, "Ward_ES_B6": dress_es_b6,
    "Ward_WS1": dress_ws1, "Ward_WS2": dress_ws2, "Ward_AssistedShower": dress_assisted_shower,
    "Ward_DayRoom": dress_day_room, "Ward_Cleaners": dress_cleaners, "Ward_CleanUtility": dress_clean_utility,
    "Ward_Pantry": dress_pantry, "Ward_DirtyUtility": dress_dirty_utility, "Ward_Disposal": dress_disposal,
    "Ward_Entrance": dress_ward_entrance,
    "AE_Majors1": dress_majors_w, "AE_Majors2": dress_majors_w, "AE_Majors3": dress_majors_w_keepbed,
    "AE_Majors4": dress_majors_w_keepbed, "AE_Majors5": dress_majors_w, "AE_Majors6": dress_majors_e,
    "AE_Majors7": dress_majors_e, "AE_Isolation": dress_isolation, "AE_Relatives": dress_relatives,
    "AE_DirtyUtility": dress_ae_dirty, "AE_CleanUtility": dress_ae_clean, "AE_StaffBase": dress_ae_staffbase,
    "AE_Resus": dress_resus, "AE_Handover": dress_handover, "AE_XRay": dress_xray, "AE_Waiting": dress_ae_waiting,
    "AE_Triage": dress_triage, "AE_Minors1": dress_minors, "AE_Minors2": dress_minors,
    "WC_Accessible": dress_wc_accessible, "WC_Public": dress_wc_public, "Staff_WC": dress_staff_wc,
    "Central_Store": dress_central_store, "Main_LiftLobby": dress_main_lift_lobby,
}

# Building doorways that must stay clear (furniture 1 m each side, like DOORWAYS): world (x0, y0, x1, y1) segments.
BUILDING_DOORWAYS = {
    "W_Compartment": (-35.50, 11.50, -35.50, 13.41), "E_Compartment": (11.21, 6.30, 11.21, 8.21),
    "W_Stair": (-31.90, 14.45, -30.90, 14.45), "E_Stair": (8.40, 10.40, 9.40, 10.40),
    "Main_LiftLobby": (7.03, 4.15, 8.94, 4.15), "Resus": (-27.40, 14.45, -25.50, 14.45),
    "D07": (-41.547, 32.881, -39.123, 32.881), "D34": (20.81, 29.578, 20.81, 31.995),
    "D13": (-38.04, 7.711, -38.04, 10.160), "D14": (-32.44, 7.692, -32.44, 10.110),
    "Link": (17.93 + 0.1, -4.45, 17.93 + 2.83, -4.45),
    # room doors in existing frames (jamb to jamb)
    "AE_Majors1": (-42.097, 30.404, -42.097, 31.647), "AE_Majors2": (-42.097, 26.693, -42.097, 27.936),
    "AE_Majors3": (-42.097, 22.985, -42.097, 24.228), "AE_Majors4": (-42.097, 15.597, -42.097, 16.840),
    "AE_Majors5": (-42.097, 11.866, -42.097, 13.109), "AE_Isolation": (-42.097, 8.189, -42.097, 9.432),
    "AE_DirtyUtility": (-38.476, 30.396, -38.476, 31.639), "AE_CleanUtility": (-38.476, 26.667, -38.476, 27.910),
    "AE_Majors6": (-38.476, 22.958, -38.476, 24.201), "AE_Majors7": (-38.476, 19.260, -38.476, 20.503),
    "AE_Relatives": (-38.476, 15.567, -38.476, 16.810), "AE_XRay": (-23.726, 8.770, -22.483, 8.770),
    "AE_Triage": (-4.893, 12.433, -3.650, 12.433), "AE_Minors1": (-1.407, 12.433, -0.164, 12.433),
    "AE_Minors2": (2.060, 12.433, 3.303, 12.433), "Central_Store": (-21.520, 12.899, -20.278, 12.899),
    "Staff_WC": (-18.039, 12.899, -16.797, 12.899), "Pharmacy1": (-9.724, 8.770, -8.481, 8.770),
    "Pharmacy2": (-2.729, 8.770, -1.486, 8.770),
    "Ward_B1": (20.829, 26.261, 20.829, 27.503), "Ward_WS1": (20.829, 22.578, 20.829, 23.821),
    "Ward_B2": (20.829, 18.879, 20.829, 20.122), "Ward_B3": (20.829, 15.178, 20.829, 16.421),
    "Ward_WS2": (20.829, 11.479, 20.829, 12.722), "Ward_B4": (20.829, 7.776, 20.829, 9.019),
    "Ward_B5": (20.829, 4.063, 20.829, 5.305), "Ward_Office": (20.829, -1.400, 20.829, -0.157),
    "Ward_Cleaners": (17.192, 26.261, 17.192, 27.503), "Ward_B6": (17.192, 22.559, 17.192, 23.801),
    "Ward_CleanUtility": (17.192, 18.878, 17.192, 20.121), "Ward_Pantry": (17.192, 15.186, 17.192, 16.429),
    "Ward_DirtyUtility": (17.192, 11.476, 17.192, 12.719),
    # en-suite and internal openings
    "ES_B1": (22.30, 25.045, 23.25, 25.045), "ES_B2": (22.30, 21.350, 23.25, 21.350),
    "ES_B3": (22.30, 13.950, 23.25, 13.950), "ES_B4": (22.30, 10.240, 23.25, 10.240),
    "ES_B5": (22.30, 2.860, 23.25, 2.860), "ES_B6": (11.55, 25.050, 12.50, 25.050),
    "DU_Disposal": (13.55, 12.55, 13.55, 13.55), "Office_Shower": (23.30, -1.55, 23.30, -0.45),
    "Office_DayRoom": (21.10, -1.85, 22.30, -1.85),
    "WC_Accessible": (-15.45, 3.50, -14.50, 3.50), "WC_Public": (-13.60, 3.50, -12.65, 3.50),
}

# Final-exit and compartment summary (for docs/tests; areas and distances: design_final.md s.6).
COMPARTMENTS = {
    "C1": {"name": "west: A&E majors corridor", "adjoins": ["C2"], "final_exits": ["D07", "D13"]},
    "C2": {"name": "central: lobby, A&E resus/X-ray/walk-in, offices, pharmacy, stairs, main lifts",
           "adjoins": ["C1", "C3"], "final_exits": ["MainEntrance", "D14"]},
    "C3": {"name": "east: 6-bed in-patient ward", "adjoins": ["C2", "Wing"], "final_exits": ["D34"]},
}

# World-frame robot routes for g1-isaac-sim/tools/check_routes.py (ROUTES gains a "world" flag; waypoints in metres).
BUILDING_ROUTES = {
    "ambulance bay -> courtyard -> D14 -> handover -> resus": [(-36.28, 0.1), (-36.28, 5.0), (-36.0, 8.9),
                                                              (-33.0, 8.9), (-29.0, 9.5), (-26.45, 12.5),
                                                              (-26.45, 14.45), (-26.45, 15.0)],
    "resus -> X-ray": [(-26.45, 15.0), (-26.45, 12.5), (-23.1, 10.6), (-23.1, 8.77), (-23.1, 7.2)],
    "courtyard -> D13 -> isolation": [(-36.28, 5.0), (-36.28, 8.9), (-40.3, 8.9), (-42.1, 8.8), (-43.6, 8.3)],
    "majors corridor -> W compartment door -> resus": [(-40.3, 20.0), (-40.3, 12.45), (-35.5, 12.45),
                                                       (-29.0, 12.45), (-26.45, 12.8), (-26.45, 15.0)],
    # x 1.46: centre of the east glass unit's wide opening (x -1.24 is the mullion between the two units)
    "walk-in: main entrance -> A&E waiting": [(1.46, -4.5), (1.46, -1.6), (-5.0, -1.6), (-9.0, -1.4)],
    "A&E waiting -> triage": [(-9.0, -1.4), (-5.0, -1.6), (5.70, -1.6), (5.70, 10.6), (-4.27, 10.6),
                              (-4.27, 12.43), (-4.27, 13.6)],
    "resus -> main corridor -> E compartment door -> ward": [(-26.45, 12.5), (-23.0, 10.6), (3.0, 10.6),
                                                             (6.0, 7.25), (11.21, 7.25), (14.0, 7.25),
                                                             (19.0, 9.0), (19.0, 20.0)],
    "ward round (corridor)": [(19.0, 8.0), (19.0, 28.0)],
    "ward corridor -> B1 bed side": [(19.0, 26.88), (20.83, 26.88), (22.4, 26.88)],
    "ward corridor -> B6 bed side": [(19.0, 23.18), (17.19, 23.18), (15.6, 23.18)],
    "ward entrance -> main lift lobby -> MainLift1": [(14.0, 7.25), (11.21, 7.25), (8.0, 5.6), (8.0, 4.15),
                                                     (8.0, 2.0), (9.0, -1.1), (10.6, -1.1), (12.6, -1.1)],
    "main lift lobby -> MainLift2": [(8.0, 2.0), (9.0, 2.4), (10.6, 2.4), (12.6, 2.4)],
    "ward -> SE lobby -> link (4 m) -> surgical corridor": [(19.0, 9.0), (19.0, -2.0), (19.4, -4.45),
                                                            (19.4, -7.5), (19.4, -12.45)],
    # The stair routes end on the landing, 0.10 m short of the first riser with a 0.5 m footprint (W flight y >= 15.45,
    # E flight y >= 11.0).
    "west stair door": [(-31.4, 12.5), (-31.4, 14.45), (-31.4, 15.10)],
    "east stair door": [(8.9, 9.2), (8.9, 10.40), (8.9, 10.65)],
    "D07 final exit": [(-40.3, 30.0), (-40.3, 32.88), (-40.3, 35.0)],
    "D34 final exit": [(18.5, 30.8), (20.81, 30.8), (23.0, 30.8)],
}

# Building cameras, world frame: eye, target, focal length (mm). Merged into CAMERAS (wing-local) below.
BUILDING_CAMERAS = {
    "Cam_WardCorridor": ((17.6, 6.0, 1.6), (19.0, 28.0, 1.0), 14),     # from the nurse base, up the ward corridor
    "Cam_WardB1": ((21.1, 26.9, 2.2), (25.0, 26.9, 0.6), 10),
    "Cam_WardPod1": ((22.78, 27.9, 1.75), (22.78, 23.4, 0.9), 12),    # from B1 through its en-suite door (pod 1)
    "Cam_EastCompartment": ((8.5, 7.25, 1.8), (14.0, 7.25, 1.0), 12),
    "Cam_MainLiftLobby": ((7.1, 0.6, 2.3), (11.10, 0.6, 1.0), 10),
    "Cam_Resus": ((-23.9, 14.9, 2.5), (-27.5, 20.0, 0.7), 10),          # from the SE corner: both bays
    "Cam_AEHandover": ((-27.0, 9.5, 2.2), (-31.0, 6.4, 0.6), 12),
    "Cam_MajorsCorridor": ((-40.3, 8.0, 1.7), (-40.3, 32.0, 1.0), 14),
    "Cam_WestCompartment": ((-38.0, 12.45, 1.8), (-33.0, 12.45, 1.0), 12),
    "Cam_AEWaiting": ((-5.9, 1.5, 2.3), (-10.0, -1.5, 0.6), 10),
    "Cam_WardB1EnSuite": ((26.3, 28.2, 1.9), (22.0, 25.3, 0.9), 11),   # B1 from the window end: bed, en-suite door
    "Cam_WS1Glazing": ((21.5, 21.75, 1.6), (21.5, 27.0, 1.25), 11),     # workstation WS1: vision panel into B1
    "Cam_StairEast": ((8.9, 7.0, 1.7), (8.9, 14.0, 1.2), 12),           # east stair enclosure and its FD30S door
    "Cam_WardDirtyUtility": ((16.9, 10.9, 2.1), (12.3, 13.2, 0.9), 10),  # dirty utility, disposal hold beyond
}


# EXTERIOR and building cameras are given in the world frame; CAMERAS are wing-local.
_ox, _oy, _oz = WING_ORIGIN
for _name, (_eye, _target, _focal) in list(EXTERIOR["cameras"].items()) + list(BUILDING_CAMERAS.items()):
    CAMERAS[_name] = ((_eye[0] - _ox, _eye[1] - _oy, _eye[2] - _oz), (_target[0] - _ox, _target[1] - _oy, _target[2] - _oz),
                      _focal)
