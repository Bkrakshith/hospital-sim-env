# Hospital + surgical wing: design record (2026-09-29)

## Goal

A walkable hospital environment in Isaac Sim 6.0 for a Unitree G1: corridors, operating theatre and prep
rooms, all furnished in detail. Collisions and physics are out of scope for now.

## Decisions

- **Base building:** Isaac `Environments/Hospital/hospital.usd` (two floors, about 76 × 42 m; wards,
  reception, pharmacy). It has no operating theatre, so a surgical wing is attached to its ground floor at
  the south-east exterior double door, world (18.9, -4.45).
- **Wing construction:** the i4h `Hospital_A` modular kit (2 m cells, 3.5 m ceiling) is the same kit
  the i4h OR suite is built from, so new and reused rooms match. Rooms are rectangles on the 2 m grid in
  `scene/layout.py`. The builder places floor/ceiling cells, a wall module on every boundary edge that is
  not a door or an opening, and double-door modules with their leaves removed (2 m clear).
- **OR suite reuse:** i4h `shared_OR_without_Mark` (authored in cm, labelled as metres) is referenced with
  scale 0.01 and rotated -90° so its shared west wall faces the corridor. Three corridor doors are cut into
  that wall by overriding wall modules. i4h's side room (a family lounge) becomes the sterile store with the
  lounge furniture switched off. The anteroom becomes the scrub room; its sinks moved off the new door
  wall. Everything is done with overrides, and the i4h files are untouched.
- **G1 walkability rules:** one level (z = 0); corridors 2 m and 4 m; 2 m clear doorways; the build fails
  any furniture inside a 1 m zone on either side of a doorway (`layout.DOORWAYS`).
- **Layers:** one USD layer per concern (building, shell, suite, dressing per room, lighting, cameras) under
  `usd/hospital_g1.usd`. The generator is pure `pxr`, so it runs in Isaac Sim or with `usd-core`.

## GPU memory investigation (RTX 3070, 8 GB)

The full scene crashed Isaac Sim with RTX `ERROR_OUT_OF_DEVICE_MEMORY` on the first full view.

| Tried | Result |
|---|---|
| Texture streaming with a 50% budget | still OOM; the budget is not enforced for these PNG assets |
| Texture cap (`maxMipCount=12`) | still OOM |
| Lite mirror with textures downscaled 4–16× | still OOM; textures were not the main consumer |
| Instancing identical props (67 M → 34 M triangles) | still OOM |
| No viewport resize | still OOM (lighting caches) |
| Strip the ~400 per-module lights | 2 of 11 views, then `AccelStruct creation failed` |
| **Decimate i4h meshes (34 M → 10 M triangles) + ~40 room lights** | **all views render, peak about 7.0 GB** |

Root cause: triangle density. The i4h props are film assets, and RTX acceleration structures for about
34 M+ triangles do not fit next to everything else. The many small lights were a secondary cost.

A separate issue: enabling the MCP extension with `--enable` at launch began crashing Isaac Sim during
startup (`NULL TfWeakPtr<UsdStage>`) after a stale instance was killed. Enabling it after startup
(`scene/enable_mcp_late.py`) works. The root cause is not identified.

## Exterior: drive and ambulance (2026-10-13)

The main entrance is the pair of glass sliding doors in the south facade (world x -5.64 to 3.16, y = -2.85).
It opens into the reception and the pharmacy. The building had no visible ground outside.

Decisions:

- The drive is a new layer, `usd/exterior.usd`, built by `scene/build_exterior.py`. The data is
  `layout.EXTERIOR`, in the world frame, so MuJoCo can use it too.
- The drive comes from the south and stays west of the wing. UK traffic keeps left.
- Asphalt and the hospital floor are both at z = 0. The Isaac `GroundPlane` collides at z = 0, so a lower road
  would leave the robot above the visible road. The footway is raised by 0.125 m in the west only. In front of
  the doors it drops to road level (dropped kerb), so the path from the doors to the bays is level.
- The ambulance is procedural: about 760 triangles for the whole exterior and one 2048 x 1024 livery texture.
  No ambulance asset exists in the local packs. It is reversed into its bay, rear doors to the entrance.
- The livery is generic (yellow, green Battenburg, AMBULANCE). It has no real logo or service name.
- The layer has no physics. `EXTERIOR_COLLIDERS` gives ten boxes. `g1-isaac-sim` and `g1-mujoco-sim` add them.
- Two cameras show it: `Cam_Ambulance` and `Cam_Entrance`.

## Fire doors and bed lifts (2026-10-13)

The wing gets moving fixtures with physics. The data and the UK guidance are in `layout.py`. The behaviour is in
`hospital-sim-env/fixtures/hospital_fixtures.py`, and both simulators run it.

Decisions:

- A bed-lift lobby (x 14..20, y -12..-4) opens off the east end of the corridor through an FD30S door. Two bed
  lifts open onto it from the east (HBN 00-04 6.19: lifts do not open onto corridors).
- A fire door (FD30S, double swing, held open) crosses the corridor at x = 4. The store door is FD30S, kept shut,
  leaf and a half. The link has an FD60S compartment door.
- The CrossCorridor and LiftLobby frames are kit 4 m door modules on lines with constant x. The builder turns
  them +90 deg. The kit hole is 1.83 m x 2.20 m, so the 2.0 m spec leaves sit 85 mm into the jambs.
- All fixture physics is in one layer, `usd/fixtures.usd`. The other layers stay physics-free. Each doorset,
  lift and call button is one fixed-base articulation without drives. The simulator applies the controller
  efforts as joint forces.
- The corridor dressing moved clear of the door swings and the pockets behind the held-open leaves. The plant
  moved from the corridor's east wall (now the lobby door) to the lobby. The lobby has only a sign, a sanitizer
  and the plant, so the bed-turning space (4.31 m x 3.37 m per lift) stays clear.
- The build check now covers the swept area of each fire-door leaf and the bed-turning spaces.

The result: 0 issues, 10.26 million GPU triangles (was 10.1). The fixtures are implicit cubes and cylinders.

## UK guidance review: three more fire doors and a final exit (2026-10-13)

`docs/uk-guidance-compliance.md` checked the layout against HTM 05-02, HTM 08-02, HBN 00-04 and Approved
Document M. Its three priority-1 gaps are now closed in `layout.py`:

- G1, second exit: the east sub-compartment had one exit. A final exit (`FIRE_DOORS["FinalExit"]`, rating
  `EXIT`) is now in the corridor south wall at x 11..13. It is a kit 4 m door module with no rotation. Its leaves
  open outwards, have push bars inside, and a green FIRE EXIT sign hangs over them. Outside, a flush paved path
  (`EXIT_PATH`) runs south between the theatre (x <= 10) and the lift lobby (x >= 14). The utility cart and the
  wet-floor sign moved from the exit to u = 17.6 and 18.15 on the corridor south wall.
- G2, bypass of the x = 4 line: fire could pass corridor → theatre → scrub → west corridor. The theatre
  corridor door is now FD30S (`FIRE_DOORS["Theatre"]`), double swing, held open into the theatre. The theatre
  joins the west sub-compartment.
- G3, open store: the sterile store was open to the scrub room. `FIRE_DOORS["ScrubStore"]` (FD30S, kept shut,
  opens into the store) now closes the suite's scrub–store doorway.

Other changes in the same step:

- The compartment hinges moved 30 mm off the link walls (x 0.13 and 1.87, leaves 0.87 m). The held-open leaves
  had been 27 mm inside the walls. The clear width fell to 1.69 m (sim hinge).
- The lift controller has nudging and a 95 % closing-force margin (142.5 N).
- The build's swing check now covers the suite props as well, because two doors swing into suite rooms.
- Physics ground outside the exit: the hospital's infinite `GroundPlane` and a new `ExitPathFloor` slab in
  `g1-isaac-sim/make_stage.py`. MuJoCo has an infinite floor plane.

The result: 7 doorsets (14 leaves), 0 issues, 10.3 million GPU triangles. The link corridor is still 1.8 m wide
(gap G4), and the lifts have no emergency bed or theatre-service mode (gap G6).

## Main building re-plan: ward, A&E, compartments, 4 m link (2026-10-13)

The ground floor of the Isaac hospital gets new uses. The design study is HBN/HTM-informed. It is not certified.

Decisions:

- **Rooms.** The east corridor becomes a 6-bed adult in-patient ward (compartment C3). It has 6 single bedrooms
  with en-suites, 2 nurse workstations with vision panels, a ward entrance with a nurse base, clean and dirty
  utilities, a disposal hold, a cleaners' room, a pantry, an assisted shower and a day room. The west corridor
  and the west hall become a small A&E: 7 majors cubicles, isolation, relatives' room, staff base, 2-bay resus,
  handover bay, X-ray room, triage, 2 minors rooms and a walk-in waiting with 15 seats. The two old WCs hold a
  pair of bed lifts in a protected lobby. New public WCs go in the south corridor.
- **Data.** All main-building data is in `layout.py` in the world frame (`BUILDING_*`). `hospital_fixtures`
  gives the hide list for the active fixture tier (`building_hide`) and the static boxes (`static_partitions`).
  The build, the Isaac colliders and MuJoCo all read the same functions.
- **Geometry.** The build deactivates about 250 asset prims. It draws 87 boxes in `building_mods.usd`: screens,
  partitions, 6 rebuilt walls with en-suite doorways and vision panels, and the link junction boxes. Each long
  face takes the building's own material in three bands (skirting, dado panel, paint). Ray casts on the facing
  asset wall pick each material.
- **Dirty utility.** The corridor FD30S door opens into the dirty utility (12.0 m²). The disposal hold (7.5 m²)
  is beyond it, at the window end. The partition moved from x 14.88 to x 13.55, between the two ceiling fittings.
- **Lights.** The partitions cut 6 fittings. These are hidden, and 9 new rect lights with thin panels light the
  pods, the en-suites, the office lobby and the public WCs. Every room of the re-plan has a light (build check).
- **Link.** The link is 4 m wide and PACU is 10 m with 3 bays. The hospital doorway widens west only, to
  2.73 m: the old office wall stands east of it.
- **Tiers.** `HG1_FIXTURES` selects the fixture tier (default tier2). At tier 2 and up the main-entrance glass
  units are deactivated, so the walk-in route is open in the visuals and in the physics.
- **Sky.** The scene has two dome lights (hospital and OR suite), but RTX uses one. The hospital's dome named an
  HDR that the local packs do not have, so outdoor views were sometimes a clipped white sky. The suite's dome is
  now off. The hospital's dome uses the suite's overcast `CloudyDay_04_04.hdr` at intensity 1000. Check renders:
  `Cam_Entrance` median 0.44, `Cam_WardCorridor` median 0.36, nothing clipped.
- **Verification.** `verify()` ports the design checks to the composed stage, in the world frame: rooms, wall
  slices, building furniture, doorways, fire-door swings (full tier), lift landings, lift cars, partitions and
  lights.

The result: 0 issues, 10.28 million GPU triangles, 197 lights, 222 building dressing items. The check renders
are in `renders/` (`Cam_Ward*`, `Cam_WS1Glazing`, `Cam_Resus`, `Cam_MajorsCorridor`, `Cam_AEWaiting`,
`Cam_MainLiftLobby`, `Cam_StairEast`, `Cam_HospitalToWing`).

Known gaps: the new boxes have no trim profiles; bedroom and cubicle doorways have no leaves; at tier 2 the
8 fire-hazard rooms keep their closed asset leaves.

## Known limits

- Lite assets: close-up edges on i4h props are simplified and textures are softer. Switch to full on a
  bigger GPU with `scene/select_assets.py full`.
- A few i4h parts were left undecimated because they carry second UV sets or GeomSubsets (endoscope,
  microscope, breathing circuits, the Lightwheel trolley).
- The Isaac hospital's own doors are left as authored, except where the re-plan removes them (`BUILDING_HIDE`).
