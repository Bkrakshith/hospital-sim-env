"""Layout of the surgical wing attached to the Isaac hospital.

Wing-local frame: metres, axes aligned with the hospital's world frame, origin at WING_ORIGIN
(the west edge of the 2 m link corridor, flush with the hospital's south exterior wall).
+x is east, +y is north (towards the hospital), so the whole wing sits at negative y.

    y=0   ─────────────── hospital south wall ── [SE double door] ─────────────
          |      PRE-OP (-10..0)       | LINK |          PACU (2..14)          |
    y=-6  ───────────[door]─────────────      ───────────[door]───────────────
          |                     SURGICAL CORRIDOR (-12..14)                    |
    y=-10 ──[door]───────────┬─[door]─┬──────────[door]──────────────────────
          |  STERILE STORE   | SCRUB  |          OPERATING THEATRE (0..10)
          |    (-12..-4)     | (-4..0)|
    y=-18 ───────────────────┴────────┤                                       y=-20
"""

CELL = 2.0
WING_ORIGIN = (17.93, -4.45, 0.0)
WALL_T = 0.10  # kit wall thickness, inside each room's cell

# Rooms built from the Hospital_A kit, as (x0, y0, x1, y1) in wing-local metres on the 2 m grid.
KIT_ROOMS = {
    "Link": (0, -6, 2, 0),
    "PreOp": (-10, -6, 0, 0),
    "PACU": (2, -6, 14, 0),
    "Corridor": (-12, -10, 14, -6),
}

# Wall segments left open: link <-> corridor and link <-> hospital SE door.
OPENINGS = [(0, -6, 2, -6), (0, 0, 2, 0)]

# 4 m wall segments that get a kit double-door module (leaves removed, 2 m clear opening).
WING_DOORS = {"PreOp": (-6, -6, -2, -6), "PACU": (6, -6, 10, -6)}

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
DOORWAYS = {
    "Hospital-Link": (0, 0, 2, 0),
    "Link-Corridor": (0, -6, 2, -6),
    "PreOp": (-5, -6, -3, -6),
    "PACU": (7, -6, 9, -6),
    "Store": (-9, -10, -7, -10),
    "Scrub": (-3, -10, -1, -10),
    "Theatre": (7, -10, 9, -10),
    "Scrub-Theatre": (0, -15, 0, -13),
    "Scrub-Store": (-4, -15, -4, -13),
}
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
}


def dress_link(r):
    r.wall("FirstAid", "SM_FirstAidKit_01a", "W", 2.2, z=1.35)
    r.wall("Sanitizer", "SM_SoapDispenser_01a", "W", 4.6, z=1.1)
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
    for i, bx in enumerate((1.4, 4.4, 7.6, 10.6)):
        r.wall(f"Bed{i}", "SM_HospitalBed_01b", "N", bx)
        r.wall(f"Oxygen{i}", "SM_GasCart_01c", "N", bx - 0.95)
        r.wall(f"BedsideTable{i}", "SM_BedSideTable_01b", "N", bx + 0.9)
        r.wall(f"VitalPad{i}", "VitalPad_A", "N", bx, z=1.55)
    for i, sx in enumerate((2.9, 6.0, 9.1)):
        r.put(f"Screen{i}", "HospitalScreen_A", sx, 4.45, yaw=90)
    # Nurse station on the south wall, east of the door.
    for i, dx in enumerate((8.9, 10.5)):
        r.wall(f"Desk{i}", "SM_Desk_04a", "S", dx)
        r.wall(f"Computer{i}", "SM_Computer_02b", "S", dx, z=0.83)
        r.put(f"Chair{i}", "SM_Chair_01a", dx, 1.05, yaw=180)
    # Crash cart, cabinet, water on the south wall, west of the door.
    r.wall("MedCabinet", "SM_MedicalCabinet_01a", "S", 0.7)
    r.wall("CrashCart", "EmergencyTrolley_A", "S", 2.4)
    r.wall("Defibrillator", "Defibrillator_A", "S", 3.4, z=1.3)
    r.wall("Water", "SM_WaterDispenser_01a", "S", 4.3)
    r.wall("Washbasin", "SM_Washbasin", "E", 1.7, z=0.78)
    r.wall("Soap", "SM_SoapDispenser_01a", "E", 1.25, z=1.3)
    r.wall("Towels", "SM_Toweldispenser_01a", "E", 2.4, z=1.25)
    r.wall("Bin", "SM_TrashCan", "E", 0.5)


def dress_corridor(r):
    # Parked along the north wall (room-local y = 4), clear of the pre-op (7..9), link (12..14)
    # and PACU (19..21) openings.
    r.wall("Gurney0", "SM_Gurney_01a", "N", 2.5)
    r.wall("Gurney1", "SM_Gurney_01b", "N", 5.0)
    r.wall("SupplyCart", "SM_SupplyCart_01e", "N", 16.0)
    r.wall("Wheelchair0", "SM_WheelChair_01a", "N", 17.5)
    r.wall("Wheelchair1", "Wheelchair_C", "N", 23.0)
    r.wall("OxygenCart", "SM_GasCart_01b", "N", 24.4)
    # South wall, clear of the store (3..5), scrub (9..11) and theatre (19..21) openings.
    r.wall("LWCart", "Cart001", "S", 1.0)
    r.wall("UVCRobot", "UVCRobot_A", "S", 7.0)
    r.wall("LWTrolley", "InstrumentTrolley001", "S", 14.0)
    r.wall("SupplyCart2", "SM_SupplyCart_02a", "S", 15.4)
    r.wall("UtilityCart", "MedicalUtilityCart_A", "S", 23.5)
    r.wall("WetFloor", "S_WetFloorSign", "S", 25.4)
    r.wall("Plant", "Plant", "E", 2.0)
    # Wall fixtures: signs and hand sanitizer at every door, first aid, intercom.
    for name, wall, u in (("PreOp", "N", 6.6), ("PACU", "N", 18.6), ("Store", "S", 2.6),
                          ("Scrub", "S", 8.6), ("Theatre", "S", 18.6)):
        r.wall(f"Sign{name}", "SignSideWall", wall, u, z=2.2)
        r.wall(f"Sanitizer{name}", "SM_SoapDispenser_01a", wall, u, z=1.1)
    r.wall("NoEntry", "SignDoNotEnter", "S", 20.0, z=2.45)
    r.wall("FirstAid", "SM_FirstAidKit_01a", "N", 10.3, z=1.35)
    r.wall("Intercom", "SM_Intercom_01a", "N", 11.3, z=1.25)
    r.put("SignBoard", "SignBoard", 13.0, 2.0, z=3.35, anchor="top")


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
    "Store": dress_store,
    "Scrub": dress_scrub,
    "Theatre": dress_theatre,
}

# Viewing cameras, wing-local: name -> (eye xyz, target xyz, focal length mm).
CAMERAS = {
    "Cam_HospitalToWing": ((1.0, 6.0, 1.5), (1.0, -8.0, 1.0), 14),
    "Cam_Link": ((1.0, -0.6, 1.6), (1.0, -9.0, 0.8), 12),
    "Cam_CorridorWest": ((12.2, -8.0, 1.7), (-12.0, -8.0, 0.9), 14),
    "Cam_CorridorEast": ((-11.4, -8.0, 1.7), (14.0, -8.0, 0.9), 14),
    "Cam_PreOp": ((-4.0, -5.4, 2.4), (-5.0, -1.0, 0.6), 10),
    "Cam_PACU": ((8.0, -5.4, 2.4), (8.0, -1.0, 0.6), 10),
    "Cam_Store": ((-8.0, -10.6, 2.4), (-8.0, -17.0, 0.6), 10),
    "Cam_Scrub": ((-2.0, -10.6, 2.2), (-2.0, -17.5, 0.8), 10),
    "Cam_Theatre": ((8.0, -10.6, 2.6), (4.0, -15.5, 0.8), 10),
    "Cam_TheatreWide": ((9.3, -19.3, 3.0), (3.0, -12.0, 0.6), 10),
}
