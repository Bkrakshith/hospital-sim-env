"""Asset catalog for the hospital scene.

Every asset is addressed by a short key. Paths are relative to the repo root and go through the
`assets/isaac` and `assets/i4h` symlinks, so the generated USD layers stay portable and can be switched
between full-resolution and reduced-texture assets with `scene/select_assets.py` (no rebuild needed).
"""

import os

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

ISAAC = "assets/isaac/Isaac"
I4H = "assets/i4h/Props"
OR_LIB = f"{I4H}/shared_OR_without_Mark/Collected_surgery_room_movie_with_heart_adjusted"
KIT = f"{OR_LIB}/Modular Hospital/Hospital_A"
SURG = f"{OR_LIB}/Assets/Surgery Room"
PATIENT = f"{OR_LIB}/Assets/Patient Room"
HOSP_PROPS = f"{ISAAC}/Environments/Hospital/Props"
LW = f"{I4H}/Lightwheel/Assets"

HOSPITAL_BUILDING = f"{ISAAC}/Environments/Hospital/hospital.usd"
OR_SUITE = f"{I4H}/shared_OR_without_Mark/main.usd"

# Hospital_A modular kit: 2 m cells, authored in cm.
KIT_CENTER = f"{KIT}/sm_hospital_a01_2m_center_01.usd"
KIT_STRAIGHT = f"{KIT}/sm_hospital_a01_2m_straight_01.usd"
KIT_DOOR = f"{KIT}/sm_hospital_a02_4x2m_doordouble_01.usd"


def _find_usd(folder):
    """Top-level .usd in an asset folder (the wrapper, not the Model/ payload)."""
    full = os.path.join(REPO, folder)
    files = sorted(f for f in os.listdir(full) if f.endswith(".usd"))
    if not files:
        raise FileNotFoundError(f"no .usd in {folder}")
    return f"{folder}/{files[0]}"


_SURG_KEYS = [
    "AnesthesiaMachine_A", "DrugCabinets_A", "EmergencyTrolley_A", "Glovebox_A", "InstrumentTrolley_A",
    "InstrumentTrolley_B", "MedicalFacemasks_A", "MedicalUtilityCart_A", "MobileCartsAndTables_A",
    "Refrigerator_A", "ResuscitationBed_A", "SterilisingContainerSystem_A", "SterilisingContainerSystem_B",
    "SurgicalMonitor_A", "SurgicalSink_A", "VitalPad_A", "Disinfectant_A", "InstrrumentSterilizationTray_C",
    "SurgicalDrape_A", "UltrasoundUSGMachine_A",
]
_PATIENT_KEYS = [
    "Defibrillator_A", "DiagnosticEquipmentCart_A", "HospitalScreen_A", "Humidifier_A", "WasteCollector_A",
    "Wheelchair_B", "Wheelchair_C", "UVCRobot_A", "UVCLamp_B", "SoftFurniture_B", "TV_A", "Socket_A",
]
_HOSP_KEYS = [
    "SM_HospitalBed_01b", "SM_HospitalBed_02d", "SM_Gurney_01a", "SM_Gurney_01b", "SM_BedSideTable_01b",
    "SM_Chair_01a", "SM_Chair_02a", "SM_Computer_02a", "SM_Computer_02b", "SM_Desk_04a", "SM_ReceptionDesk_01a",
    "SM_MedicalCabinet_01a", "SM_SupplyCabinet_01c", "SM_ShelfSet_01a", "SM_Shelf_06f", "SM_MedShelf_01d",
    "SM_SupplyCart_01e", "SM_SupplyCart_02a", "SM_SupplyCart_03a", "SM_GasCart_01b", "SM_GasCart_01c",
    "SM_Washbasin", "SM_SoapDispenser_01a", "SM_Toweldispenser_01a", "SM_TrashCan",
    "SM_FirstAidKit_01a", "SM_Intercom_01a", "SM_Scale_01a", "SM_WaterDispenser_01a", "SM_WheelChair_01a",
    "SM_Chart_Rack_01c", "SM_Xraylightbox_01a", "S_WetFloorSign", "SM_MedicalBox_01g", "SM_Container_01a",
    "SM_PillBottleSet_01a", "SM_FirstAidKit_01a", "SM_SideTable_02a", "SM_Frame11",
    # main-building re-plan (layout.NEW_ASSET_KEYS): WC fittings, cleaners' room, clinical waste
    "SM_Toilet", "SM_ToiletPaper", "SM_Mirror", "SM_MopSet_01a", "SM_MopSet_01b", "SM_DisposalStand_02",
]
_LW_KEYS = [
    "Box001", "Cart001", "Cart003", "InstrumentTrolley001", "Plate001", "SurgicalTray001", "SurgicalTray006",
    "SurgicalTrayNoLid006", "SurgicalTweezers", "Trocar002", "DrainageTube002", "PneumoperitoneumNeedle001",
]


def build_catalog():
    cat = {}
    for k in _SURG_KEYS:
        cat[k] = _find_usd(f"{SURG}/{k}")
    for k in _PATIENT_KEYS:
        cat[k] = _find_usd(f"{PATIENT}/{k}")
    for k in _HOSP_KEYS:
        cat[k] = f"{HOSP_PROPS}/{k}.usd"
    for k in _LW_KEYS:
        cat[k] = _find_usd(f"{LW}/{k}")
    cat["SignBoard"] = f"{OR_LIB}/Assets/Signs/HospitalSigns/sm_hospitalsingboard_a01_01.usd"
    cat["SignSideWall"] = f"{OR_LIB}/Assets/Signs/HospitalSigns/sm_hospitalsingsidewall_a03_01.usd"
    cat["SignDoNotEnter"] = f"{OR_LIB}/Assets/Signs/HospitalSigns/sm_hospitalsingdonotenter_a01_01.usd"
    cat["Plant"] = f"{OR_LIB}/Assets/Vegetation/sm_plant_a02_01.usd"
    return cat
