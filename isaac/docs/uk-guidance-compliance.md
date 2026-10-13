# UK guidance check: simulated surgical wing and main building

Review date: 2026-10-13. Layout reviewed: `isaac/scene/layout.py` (with `FIRE_DOORS`, `LIFTS`, `LIFT_SPEC`,
`LIFT_CALL`, `EXIT_PATH`, `COMPARTMENTS` and the `BUILDING_*` names) and `fixtures/hospital_fixtures.py`.

Update 2026-10-13 (same day): the layout now has the changes of gaps G1, G2, G3, G5 and part of G6. The wing has
7 fire doors: a final exit, a theatre fire door and a scrub–store fire door were added. The lift controller now
has nudging and a closing-force margin. The tables below give the new values. Section 4 lists what remains.

Update 2026-10-13 (final design): the link is now 4 m on the grid (3.8 m clear). The hospital opening is 2.73 m
clear. The link compartment doorset has two 1.20 m leaves. The PACU is 10 m x 6 m with 3 recovery bays. Gap G4
is done. Section 5 is new: it checks the re-planned main building (ward, A&E, compartments, stairs, lifts).

## 1. Summary

The wing is a single-storey surgical department on the ground floor of the multi-storey Isaac hospital. It
connects to the hospital through a 4 m link corridor (3.8 m clear) at the south-east door. The hospital opening
of the link is 2.73 m clear. The wing contains a pre-op bay (3 beds), a PACU (3 beds), one operating theatre, a
scrub room, a sterile store, a 4 m surgical corridor and a bed-lift lobby with a duplex pair of bed lifts.

The main building (section 5) has a 6-bed ward, a small A&E and three fire compartments (C1, C2, C3). It has
two enclosed up-stairs, a second pair of bed lifts in a protected lobby, and 15 new fire doorsets.

Fire strategy in the layout:

- The compartment line is at the hospital/link opening (FD60S, double-swing).
- A sub-compartment line crosses the corridor at x = 4 (FD30S, double-swing). It continues along the
  theatre's north and east walls. The theatre corridor door on this line is FD30S, double-swing. West: link,
  pre-op, store, scrub, theatre, west corridor. East: PACU, east corridor, lift lobby.
- The east sub-compartment has a final exit to outside (x 11..13 on the corridor south wall). It opens
  outwards onto a level paved path.
- The lift lobby is a protected lobby (FD30S, opens towards the corridor).
- The sterile store is a fire hazard room. Both of its doorways have FD30S doors that are kept shut: a
  leaf-and-a-half door to the corridor and a double door to the scrub room.
- Doors on circulation routes (compartment, x = 4, lift lobby, theatre) are held open on fail-safe magnets
  that release on the fire alarm.

**Status of this document.** The design follows the guidance for robot testing; it is not a certified design.
No fire engineer or clinical planner reviewed it. Describe it as HBN/HTM-informed, never as compliant. The
geometry is simplified: walls are 0.10 m kit modules with no fire rating, and the pre-op, PACU and scrub
doorways have no leaves. Sections 1 to 4 cover the wing. Section 5 covers the main building and the hospital
side of the junction.

**Method.** Areas use the 2 m grid and the clear size (grid minus 0.10 m walls inside each cell). Travel
distances are straight lines inside a room to the door centre, then along the corridor centre line
(y = -8). Furniture is ignored, so real routes are about 1 m longer. Effective clear width (ecw) of a swing
door is given for two hinge models: the simulation (leaf thickness centred on the hinge axis) and a real
butt hinge (full leaf thickness in the opening). The kit door frame (all doors except the compartment door)
has a 1.83 m clear opening between its jambs, so the frame sets the clear width of those doors. Handles and push
bars are not in the clear-width calculation.

Status values (wing, sections 2 to 4): **Meets**, **Partly**, **Does not meet**, **Not modelled**. Status values
(main building, section 5): **Meets**, **Partly**, **Gap** (provided, but below the guidance), **Missing** (not
provided). [secondary] marks a value not read in a primary source. [unverified] marks a value from memory or
extrapolation, or a value not re-read in the source.

## 2. Computed quantities

| Space | Grid (m) | Grid area (m²) | Clear (m) | Clear area (m²) |
|---|---|---|---|---|
| Link | 4 × 6 | 24 | 3.8 × 5.8 | 22.0 |
| Pre-op | 10 × 6 | 60 | 9.8 × 5.8 | 56.8 |
| PACU | 10 × 6 | 60 | 9.8 × 5.8 | 56.8 |
| Corridor | 26 × 4 | 104 | 25.8 × 3.8 | 98.0 |
| Lift lobby | 6 × 8 | 48 | 5.9 × 7.8 (east side is the lift front) | 46.0 |
| Sterile store | 8 × 8 | 64 | 7.8 × 7.8 | 60.8 |
| Scrub | 4 × 8 | 32 | 3.8 × 7.8 | 29.6 |
| Theatre | 10 × 10 | 100 | 9.8 × 9.8 | 96.0 |
| **Wing (one compartment)** | | **492** | | |

Sub-compartments (grid): west 344 m² including the 100 m² theatre and the 24 m² link (3 pre-op beds, 1 theatre
table); east 148 m² including the 48 m² lobby (3 PACU beds). Patients in the wing: 7. The theatre door opens into the east corridor.
It is on the sub-compartment line, so it is an exit from the west sub-compartment into the east one.

| Start point (furthest) | In room | To sub-compartment line | To hospital | To final exit |
|---|---|---|---|---|
| Store, SW corner | 8.8 | 21.0 | 24.7 | 29.2 |
| Pre-op, NW corner | 8.3 | 16.6 | 19.3 | 24.8 |
| Scrub, SW corner | 8.1 | 14.4 | 19.1 | 22.7 |
| Corridor, west end | 0 | 16.0 | 19.5 | 24.2 |
| Theatre, SW corner | 12.7 | 12.7 (theatre door) | 26.7 | 16.7 |
| PACU, NE corner | 8.3 | 12.8 | 21.3 | 14.0 |
| Lift lobby, far corner | 7.1 | 17.1 | 26.2 | 9.9 |

Distances in metres. "To sub-compartment line" is the x = 4 door, except for the theatre (its corridor door).
The "To hospital" values were measured with the 2 m link (route at wing x = 1). The 4 m link moves the centre of
the hospital opening to wing x 1.47. The values were not measured again; the change is less than 0.5 m.
The final exit is at (12, -10). Routes from the west go through the x = 4 door. In the east sub-compartment,
escape has two directions as soon as a person is in the corridor: west to the x = 4 door, or the final exit.

## 3. Guidance matrix (wing)

### Fire (HTM 05-02)

| Item | Guidance | Simulated | Status | Note |
|---|---|---|---|---|
| Compartment area | ≤ 2000 m², multi-storey building (5.11) | 492 m² | Meets | |
| Sub-compartment needed | > 750 m² or > 30 patients (3.25) | 492 m², 7 patients | Meets | Not required by 3.25. Provided for the operating department (3.69–3.70). |
| Sub-compartment enclosure | 30 min walls to the roof/compartment floor (3.26) | Line across the corridor at x = 4, then the theatre's north and east walls; theatre door FD30S | Meets | The theatre → scrub bypass is closed (G2, done). The walls carry no rating in the model. |
| Exits from each sub-compartment | ≥ 2, to separate (sub-)compartments (3.27) | West: 2 (hospital, east). East: 2 (west, final exit) | Meets | G1, done. The second east exit is a final exit, not a sub-compartment. The fire engineer must accept this. |
| Exits from the compartment | 60 m to each of two adjoining compartments, or one compartment and a stair/final exit (3.34) | Hospital, max 26.7 m; final exit, max 29.2 m | Meets | G1, done. |
| Final exit | Level, no step; doors open outwards; clear space outside (3.59–3.60) | Flush threshold onto a 4 m wide paved path; leaves open outwards; push bars inside | Meets | The path runs 14 m south between the theatre and the lift lobby. The place of safety beyond it is not modelled. |
| Single direction of escape | 15 m in-patient; 18 m other (3.30) | PACU 8.3 m; lobby 7.1 m; store about 17 m to the link junction | Meets | The final exit removed the east dead end. The store margin is small. |
| Stub corridor | > 4.5 m single direction: 30 min construction (3.31) | East corridor 10 m single direction | Not modelled | Walls carry no rating. |
| Travel in sub-compartment | ≤ 30 m to an adjoining (sub-)compartment (3.35) | Max 21.0 m (store); theatre 12.7 m | Meets | |
| Compartment door | FD60S "elsewhere" (Table C1) | FD60S | Meets | FD30S is allowed if the hospital is sprinklered. |
| Sub-compartment doors | FD30S (Table C1) | x = 4 and theatre doors FD30S | Meets | |
| Lift lobby door | FD30S, circulation to lobby (Table C1) | FD30S | Meets | |
| Lift landing doors | FD30 (E30), BS EN 81-58 (Table C1, note 3) | No rating in the model | Not modelled | |
| Fire hazard room | Store rooms listed (Table 6); 30 min enclosure (5.40–5.42); FD30S | Corridor door and scrub–store door FD30S, both kept shut | Meets | G3, done. The walls carry no rating in the model. |
| Self-closing | All fire doors, BS EN 1154 (App C) | All 14 leaves of the 7 doors have closers | Meets | Closer moment 11 N·m at closed is low for a fire door [unverified, BS EN 1154 power size 3 ≈ 18 N·m]. A pushed leaf closes from 100° in about 6.7 s in both simulators. |
| Hold-open | Fail-safe, alarm-linked, not on stair doors (App C) | Compartment, x = 4, lobby and theatre doors; magnets release on alarm | Meets | No stairs in the wing. All 7 doors close in about 6.2 s after the alarm. |
| Swing | Double-swing where escape is in two directions; else in the escape direction (App C) | Compartment, x = 4, theatre: double. Lobby: opens out of the lobby. Final exit: opens outwards | Meets | Both store doors open into the store, like a normal room door. Few people use the store [unverified: the occupancy limit for inward-opening doors was not checked]. |
| Vision panels | Down to 500 mm above threshold (App C) | 500–1800 mm | Meets | AD M 3.10(h) zone 500–1500 mm is inside it. |
| Protected lobby to lifts | Required unless off a hospital street (5.34); not a through route (5.35); no other use (5.37) | Lobby at the corridor end, no dressing | Meets | |
| Lobby length | 4100 mm clear of door swing, beds (5.36; HBN 00-04 4.9) | 5.9 m; door swings out of the lobby | Meets | |

### Circulation and doors (HBN 00-04, AD M vol 2)

| Item | Guidance | Simulated | Status | Note |
|---|---|---|---|---|
| Surgical corridor | 2150 mm with passing spaces (3.9); 2960 mm where beds pass (3.10) | 3.8 m | Meets | Parked equipment narrows it (3.14). 3.5 m remains with 150 mm handrail zones (3.30). |
| Link corridor | 2150 mm (3.9); 2960 mm where beds pass (3.10) | 3.8 m clear (4 m grid) | Meets | G4, done. Two beds can pass in the link. |
| Hospital opening of the link | Bed doorway | 2.73 m clear (world x 18.03..20.76; was 2.48 m) | Meets | The old office wall east of x 20.76 limits the width. Beds pass the doorway one at a time. |
| Compartment door ecw | Double doors, beds, heavy traffic: 1740 mm (8.21) | 2.29–2.34 m between the held leaves at 90° (by the point of measurement) | Meets | G4, done. Two 1.20 m FD60S leaves on magnets, and a fixed FD60 side panel. The 1.2 m FD60 leaf size needs doorset certificate evidence [unverified]. |
| x = 4 and lobby doors ecw | 1740 mm (8.21) | 1.83 m (frame); leaves 1.96 m (sim), 1.91 m (real) | Meets | |
| Theatre door ecw | 1740 mm (8.21) | 1.83 m (frame); leaves 1.96 m (sim), 1.91 m (real) | Meets | |
| Final exit ecw | 1740 mm (8.21) | 1.83 m (frame); leaves 1.95 m (sim), 1.89 m (real) | Meets | |
| Store door ecw | General traffic: larger leaf 850 mm (8.19) | Active leaf 1.18 m; both 1.96 m | Meets | |
| Scrub–store door ecw | General traffic: 850 mm (8.19) | 1.83 m (frame); one leaf about 0.9 m | Meets | |
| Room doorways (pre-op, PACU, scrub) | Bed turning from a corridor: 1550 mm (8.19) | 1.83 m frame, no leaves | Not modelled | Width meets. No leaves, closers or seals. |
| Open doors in corridor | Recessed or contrasting edges (3.17, 8.27) | Lobby leaves stand 1 m into the corridor end; x = 4 leaves stand as fins; theatre leaves held inside the theatre | Partly | 1 m nibs prevent folding to the wall. |
| Door opening force | ≤ 30 N 0–30°, ≤ 22.5 N 30–60° (AD M 3.10(a)) | Compartment 1.20 m leaf: 9.7 N at the handle. Other wing leaves: max 21.6 N before the change [unverified: not measured again] | Meets | Static closer moment only. Bolted 0.8 m leaf: 23.7 N, not opened in use. |
| Thresholds | Avoid; else ≤ 10–13 mm (8.9–8.10; 4.16) | One floor level; flush lift sill; flush final exit | Meets | |
| Handrails / wall protection | Main routes (3.29–3.30); top 900–1000 mm (7.10) | None | Not modelled | |
| Ceiling height | Theatre ≥ 3000 mm (HBN 26 4.82) | 3.5 m kit | Meets | Hospital junction header 2.41 m. |

### Lifts (HTM 08-02, HBN 00-04, AD M vol 2)

| Item | Guidance | Simulated | Status | Note |
|---|---|---|---|---|
| Car size | 1800 × 2700 mm, 2500 kg (HTM 08-02 4.19; HBN 00-04 6.16) | 1.8 × 2.7 m | Meets | |
| Door | 1400 mm (4.19); ≥ 1370 × 2100 mm (HBN 00-04 6.30); centre-opening (6.110); no vision panels (6.103, 6.112) | 1.4 × 2.1 m, centre-opening | Meets | |
| Number | Duplex pairs (5.23); not a single theatre lift (5.8) | 2 | Meets | |
| Not onto corridor | HBN 00-04 6.19 | Opens into the lobby | Meets | |
| Landing space | 4310 × 3370 mm per bed lift (HBN 00-04 Fig 28, Fig 11) | 5.9 × 7.8 m lobby, lifts 4 m apart | Meets | Two turns at once fit if the 3370 mm side runs along the lift front. |
| Door times | Table 3: 2.7 s open / 3.3 s close at 1100 mm (5.29) | 3.0 / 3.6 s at 1400 mm | Partly | Extrapolated [unverified]. Table 3 is for passenger lifts. |
| Dwell | 7 s for bed lifts; 0.5 s after close/call (5.32) | 7.0 s / 0.5 s | Meets | |
| Door detection | Multi-beam, contact-less (6.117) | 8 beams, 25–1600 mm | Meets | Height band from BS EN 81-20 [secondary]. |
| Stall force | < 150 N (App C, section 3) | Closing force limit 142.5 N per panel (95 % of 150 N) | Meets | |
| Closing kinetic energy | < 10 J (App C) | 4.2 J peak, both landing panels | Meets | Car door panels not modelled. |
| Nudging | Onset < 20 s; < 4 J (App C); adjustable (6.115) | After 3 light-curtain reopens, or 18 s of obstruction: slow close (2.5 × close time), curtain ignored; 0.67 J peak | Meets | Onset is 19.5 s after a continuous obstruction (18 s, then the 1.5 s short dwell). |
| Emergency bed service | Required where a lift serves a theatre area (6.151–6.153) | None | Not modelled | See G6. |
| Theatre service | Dedicated lift (6.154) or theatre-service button, 60 s return (6.155–6.158) | None | Not modelled | See G6. |
| Landing call button | 900–1100 mm; ≥ 500 mm from a return wall (AD M 3.34(h)) | 1.00 m; 3.9 m from the lobby walls | Meets | Push force 2.5–4.1 N; BS EN 81-70 2.5–5 N [secondary]. |
| Car buttons | 900–1200 mm; ≥ 400 mm from a return wall (AD M 3.34(g)) | 1.05 / 1.15 m; 0.45 m | Meets | |
| Arrival signal, contrast floor | AD M 3.34(j); 1500 × 1500 mm contrast area (HBN 00-04 6.23) | Event log only | Not modelled | |

### Clinical rooms (secondary for this purpose)

| Item | Guidance | Simulated | Status | Note |
|---|---|---|---|---|
| Theatre size | 55 m², about square (HBN 26 4.69–4.70) | 96 m², 9.8 × 9.8 m | Meets | |
| Scrub room | Dedicated, ≥ 11 m² (HBN 26 4.54) | 29.6 m² | Meets | |
| Anaesthetic room, exit bay, dirty utility | 19 m² (4.34), 12 m² (4.101), 12 m² (4.123) | None | Not modelled | |
| Recovery places | 16 for 8 theatres (HBN 26 4.147) | 3 for 1 theatre | Meets | |
| Recovery bay | 13.5 m² (HBN 26 schedule, 4.147); island bed, 360° access (4.149); solid partitions (4.150) | 3.4 m pitch; beds against the north wall; screens | Does not meet | |
| Pre-op bed space | 3600 × 3700 mm clear (HBN 04-01 4.24) | 3.0 m pitch × 5.8 m | Does not meet | HBN 04-01 is for in-patient rooms; reference only. |

## 4. Gaps and recommended layout changes (wing)

G1, G2, G3, G4, G5 and G9 are done. G6 and G7 are done in part. G6 (rest), G7 (rest) and G8 remain open. The "Status" column
gives the state on 2026-10-13.

| # | Priority | Gap | Recommended change in `layout.py` | Status |
|---|---|---|---|---|
| G1 | 1 | East sub-compartment has one exit (3.27). The compartment has no second compartment, stair or final exit (3.34). | Add a level final exit to outside in the corridor south wall east of the theatre: `OPENINGS += [(10, -10, 14, -10)]` with a 4 m door module or a `FIRE_DOORS["FinalExit"]` entry (leaves opening outwards, no step, 1 m level outside, 3.59–3.60). Move `UtilityCart` (u = 23.5) and `WetFloor` (u = 25.4). The fire engineer must accept a final exit as the second sub-compartment exit. Alternative: a second link to the hospital from the east side. | Done. `WING_DOORS["FinalExit"]`, `DOORWAYS["FinalExit"]`, `FIRE_DOORS["FinalExit"]` (rating `EXIT`, push bars, FIRE EXIT sign), `EXIT_PATH` (paved path outside). `UtilityCart` and `WetFloor` moved to u = 17.6 and 18.15. |
| G2 | 1 | The sub-compartment line has a bypass through theatre → scrub (3.26). | Put the theatre in the west sub-compartment. Add `FIRE_DOORS["Theatre"]`: FD30S, double-swing, two 1.0 m leaves at (7, -10) and (9, -10), held open on alarm-release magnets. Treat the theatre east and north walls as the 30 min line. Alternative: FD30S on `DOORWAYS["Scrub-Theatre"]`. | Done. `FIRE_DOORS["Theatre"]`, held open into the theatre. |
| G3 | 1 | Sterile store enclosure is open to the scrub room (5.42). | Add `FIRE_DOORS["ScrubStore"]`: FD30S, self-closing, kept shut, leaves on (-4, -15)–(-4, -13), or close the doorway: drop `SUITE_OPEN_DOORS` entry and `DOORWAYS["Scrub-Store"]`. | Done. `FIRE_DOORS["ScrubStore"]`, kept shut, opens into the store. |
| G4 | 2 | Link corridor 1.8 m clear; compartment door ecw 1.69–1.75 m (HBN 00-04 3.9, 3.10, 8.21). | `KIT_ROOMS["Link"] = (0, -6, 4, 0)` (3.8 m clear) and widen `OPENINGS` and `DOORWAYS` for link–corridor. Shrink `PACU` to (4, -6, 14, 0) and re-space its beds. Fit a 1900 mm doorset, or a leaf-and-a-half 1300 + 600 mm doorset: `FIRE_DOORS["Compartment"]["leaves"]` with widths 1.3 and 0.6 and `"bolted": [1]`. The hospital doorway (2.48 m between the jambs) limits the junction width. | Done. `KIT_ROOMS["Link"] = (0, -6, 4, 0)`, 3.8 m clear. `PACU` = (4, -6, 14, 0) with 3 bays. The build rebuilds the hospital opening to 2.73 m clear with `JUNCTION_BOXES`. `FIRE_DOORS["Compartment"]` has two 1.20 m leaves: 2.29–2.34 m between the held leaves. |
| G5 | 2 | Corridor dressing clashes with the new frames: `SupplyCart` at x = 4 (north wall) sits in the x = 4 partition; `Plant` (east wall, u = 2) sits in the lift lobby doorway at y = -8. | Move `SupplyCart` to u ≥ 5.5 clear of the x = 4 door zone, and move or remove `Plant` in `dress_corridor`. The build doorway check reports these. | Done. `SupplyCart` at u = 14.85; `Plant` in the lift lobby. `verify()` reports 0 issues. |
| G6 | 2 | No nudging, emergency bed service or theatre-service mode (HTM 08-02 App C, 6.151–6.158). | Add `LIFT_SPEC` keys: `"nudge_s": 20.0`, `"nudge_ke_j": 4.0`, `"theatre_return_s": 60.0`. Add `LIFT_CALL` entries for an EBS key switch and a red 50 × 50 mm theatre-service button (6.156). Implement the modes in `FixtureController`. Set `stall_n` to 140 N for margin. | Part done. Nudging and a 95 % closing-force margin are in `FixtureController`. Nudging starts about 1.5 s late (set the obstruction time to 18 s to fix). Emergency bed service and theatre-service modes are not implemented. |
| G7 | 3 | Recovery and pre-op bed spaces are small; no 360° bed access (HBN 26 4.149; HBN 04-01 4.24). | Pull PACU beds off the north wall (`r.put` with a 0.6 m head gap) at 3.6 m pitch. Extend pre-op to `(-12, -6, 0, 0)` for three 3.6 m bays. | Part done. The PACU has 3 bays at 3.4 m pitch. The beds stay against the north wall. Pre-op is unchanged. |
| G8 | 3 | Handrails, wall protection, lift contrast floor, landing door ratings, fire door discs not modelled. | Add handrail and protection strips along corridor walls at 0.9–1.0 m. Add a `rating` field to `LIFT_SPEC` for landing doors (FD30, BS EN 81-58). | Open. The final exit has a FIRE EXIT sign; the other signs are not modelled. |
| G9 | 3 | Two citations in the code point to the wrong paragraph. | Internal door force is AD M vol 2 3.10(a), not 2.13. Closed-loop VVVF door operators are HTM 08-02 6.114, not 6.113. | Done. |

## 5. Main building

### 5.1 Scope and status

The ground floor of the Isaac hospital is re-planned for robot testing. The east corridor holds a 6-bed in-patient
ward. The west corridor and the west hall hold a small emergency department (A&E). Compartment screens with FD60S
doorsets divide the floor into three fire compartments. The design follows the guidance for robot testing; it is
not a certified design. No fire engineer or clinical planner reviewed it. Describe it as HBN/HTM-informed.

**Method.** All coordinates were measured on the real asset (`hospital.usd`) with `pxr`: mesh slices at z 0.3, 1.0,
1.6 and 2.35 m, ray casts for room faces, and door-frame jambs. Room areas are clear floor at the dado panel.
Compartment areas and travel distances come from a 0.1 m raster of the net floor at 1 m height. The raster
ignores furniture. The check script reported 0 issues for rooms, doorways, door swings, lift landings and routes.

### 5.2 Compartments (HTM 05-02)

The gross storey is about 1781 m². The net area excludes walls, stairs and fixed furniture.

| Compartment | Net area m² | Beds / patients | Adjoins | Final exits | Max travel to the adjoining compartment door | Max travel to a final exit | Max travel to the nearest of either |
|---|---|---|---|---|---|---|---|
| **C1** west, A&E majors | 330 | 7 majors + 1 isolation = 8 trolleys | C2 | D07, D13 | 27.9 m | 17.0 m | 16.0 m |
| **C2** central | 729 (with the stairs, 26.4 + 26.6 gross, and the lift lobby, 29.5) | resus 2, X-ray 1, triage and minors 3 (day); waiting 15 | C1, C3 | main entrance, D14 | 30.3 m | 51.5 m (from the N3 office) | 30.3 m |
| **C3** east, ward | 444 | 6 beds | C2, wing | D34 | 34.7 m | 36.8 m | 20.6 m |

The wing is a fourth compartment behind the FD60S doorset of the link (sections 1 to 4).

| Check | Guidance | Design | Status |
|---|---|---|---|
| Number of compartments | More than 1000 m² needs at least 3 compartments (3.11) | 3 compartments on about 1781 m² | Meets |
| Beds per compartment | At most 20 (3.9) | C3: 6 beds; C1: 8 trolleys | Meets |
| Sub-compartments | Needed over 750 m² or 30 patients (3.25) | All compartments below both limits | Meets (not needed) |
| Exits into separate compartments | Two exits into separate compartments (3.20) | C2 and C3: two adjoining compartments each. C1: one adjoining compartment (C2) plus final exits D07 and D13 (3.34(b)) | Meets (C2, C3); Partly (C1) |
| Travel distance | Limits of 3.34 | All distances are below the 60 m that the compartment design used | Meets [unverified: the 60 m figure was taken from the design summary and not re-read in the PDF] |

### 5.3 Stairs and exits

| Item | Guidance | Design | Status |
|---|---|---|---|
| Up-stair enclosure | Protected stair, FD30S self-closing doors, not held open (App C) | Both up-stairs enclosed (W 26.4 m², E 33.2 m² gross). FD30S self-closing doors `W_Stair`, `E_Stair`, never held open, open out of the stair | Meets |
| Stair discharge | To outside or to a street (3.56) | Both stairs discharge into C2 | Gap |
| Protected lobby to the stairs | Required (5.34) | None | Gap |
| Flight width | At most 1800 mm (Table 3) | 2.6 m | Gap |
| Landing at the foot | At least the stair width | 1.0 m (W), 0.55 m (E) in front of the first riser | Gap |
| Upper floors | | Notional: not modelled | Gap |
| Down-stairs to the sealed pits | | Closed with fixed screens (W x −42.00; NE y 32.15 and x 18.12). The robot cannot walk over the holes | Meets (robot safety) |
| Final exits | Level, doors open outwards (3.59–3.60) | C1: D07, D13. C2: main entrance, D14. C3: D34. D13 and D14 are automatic doors held open | Meets |

The main-entrance glass units are removed at fixture tier `tier2` and `full`. Below `tier2` they stay solid in
both simulators, and the main entrance is then not an open exit in the simulation.

### 5.4 Fire doors by type

The main building has 15 new doorsets. The link doorset of the wing is re-sized. Totals: FD60S 2 new and 1
re-sized, FD30S double 1, FD30S single 10, final exits 2.

| Type | Doors | Leaves | Hold / use | Fixture tier |
|---|---|---|---|---|
| FD60S double, compartment | `W_Compartment` (C1/C2), `E_Compartment` (C2/C3) | 2 × 0.955 m; ecw 1.80 m | held open on magnets, double-swing | `tier1` |
| FD60S double, compartment (re-sized) | `Compartment` (hospital/wing, in the link) | 2 × 1.20 m; 2.29–2.34 m between the held leaves | held open on magnets, double-swing; fixed FD60 side panel | all tiers |
| FD30S single, stair | `W_Stair`, `E_Stair` | 1 × 1.0 m | self-closing, never held open, opens out of the stair | `tier1` |
| FD30S double, lift lobby | `Main_LiftLobby` | 2 leaves | held open on magnets, opens out of the lobby | `tier2` |
| Final exit double, push bar | `D07_Exit`, `D34_Exit` | 2 × 1.21 m | kept shut, opens outwards | `tier2` |
| FD30S single, hazard room, kept shut | `AE_DirtyUtility`, `Central_Store`, `Ward_Cleaners`, `Ward_DirtyUtility` | 1 × 1.19 m | kept shut by the closer ("locked shut"; no lock is modelled), opens into the room | `full` |
| FD30S single, hazard room, self-closing | `AE_CleanLinen`, `Ward_CleanLinen`, `Pharmacy1`, `Pharmacy2` | 1 × 1.19 m | self-closing, opens into the room | `full` |

| Check | Guidance | Design | Status |
|---|---|---|---|
| Compartment doors | FD60S (Table C1) | FD60S | Meets |
| Hazard rooms | Rooms of Table 6 enclosed, FD30S doors | 2 dirty utilities (one with the disposal hold), the ward cleaners' room, the central store, 2 linen and clean utilities, 2 pharmacies | Meets |
| Pharmacy hatches | Glazing in a hazard-room enclosure rated (5.44) | Glazed hatches onto the lobby, unrated | Gap |
| Free-swing closers | Ward pantry and relatives' room (Table 6) | Not fitted | Gap |
| A&E cleaners' room | | None | Missing |
| Door opening force | ≤ 30 N (AD M 3.10(a)) | New leaves 9.6–12.4 N at the handle | Meets |
| Bedroom, cubicle and clinic doors | Not fire doors (5.40) | Asset leaves hidden: open doorways | Meets (not modelled as doors) |

At a tier that leaves a door out, the asset's own static leaves stay in that doorway. Some stand ajar or open.

### 5.5 Lifts (HTM 08-02, HBN 00-04)

The main bank is `MainLift1`, `MainLift2` with the call button `MainCall`. It is in the place of the two old WCs,
next to the main entrance, the central stair and the ward entrance.

| Item | Guidance | Design | Status |
|---|---|---|---|
| Pair, duplex, bed cars 1800 × 2700 mm, doors 1400 × 2100 mm | HTM 08-02 4.4, 4.19, 5.23 | `LIFT_SPEC`, as in the wing | Meets |
| Protected lobby, not on a corridor | HBN 00-04 6.19–6.20; HTM 05-02 5.36 | Lobby 4.30 × 6.85 m (29.5 m²); FD30S lobby door held open on magnets | Meets |
| Bed-turning space 4310 × 3370 mm per car | HBN 00-04 Fig 28 | 3370 deep × 4310 wide fits. 4310 deep is 1 cm short | Meets / Partly |
| ≥ 4100 mm clear beyond the door swing | HTM 05-02 5.36 | The door swings out of the lobby; 6.85 m along the door axis | Meets |
| Fire recall | HTM 08-02 4.12; emergency-bed key switch 6.151–6.153 | The controller opens and holds the cars on alarm, as in the wing. No emergency-bed key switch | Partly |
| Floors served | | One landing; the upper floors are notional | Gap |

A 2.05 m public passage stays open between the reception desk and the lift lobby (west of x 6.70).

### 5.6 The link (HBN 00-04)

| Item | Guidance | Design | Status |
|---|---|---|---|
| Link corridor, two beds passing | 2960 mm (3.10) | 3.8 m clear (4 m grid) | Meets |
| Hospital opening | Bed doorway | 2.73 m clear (world x 18.03..20.76; was 2.48 m) | Meets |
| Compartment doorway ecw | ≥ 1740 mm for double doors, beds, heavy traffic (8.21) | 2.29–2.34 m between the held leaves at 90° | Meets |
| FD60 leaf size | Doorset certificate | 1.20 m leaves, 65.5 kg; closer force at the handle 9.7 N | Partly [unverified: certificate evidence needed] |

### 5.7 Ward (HBN 04-01, 00-02, 00-03, 00-04)

| Item | Status | Ref |
|---|---|---|
| En-suite in every single room (6/6), 5.0–9.2 m² ≥ 4.5 m² | Meets | 04-01 3.1, 4.28 |
| En-suite door not onto the bed area (entrance zone; B6 at the window end, outside its bed space) | Meets | 04-01 5.11 |
| En-suite door a single 0.95 m opening, not a sliding and hinged pair (1980 mm for a hoist) | Partly | 00-02 4.40, Fig 46 |
| Bedroom 19.7–19.9 m² at the dado face (≥ 19 m²) | Meets (marginal) | 04-01 3.1 |
| Bed space 3.6 × 3.37 m against 3.6 × 3.7 m | Partly | 04-01 4.24 |
| Bedroom doors: 1.243 m between the jambs (open doorway); about 1.20 m with a leaf | Partly (bare minimum) | 00-04 8.19 |
| Hand-wash basin in each room | Meets | 04-01 4.33 |
| Touchdown bases, 1 per 4 beds, with a window into the room: 2 workstations for 6 beds; B5 and B6 have no vision panel | Meets / Partly | 04-01 4.51–4.52, 4.44 |
| Ward entrance reception and staff base, sight line up the corridor, low counter about 1.5 m | Meets | 04-01 5.40–5.41; 00-03 12.46–12.66 |
| Dirty utility 12.0 m² (≤ 15 beds) on the corridor door; disposal hold 7.5 m² (lockable) behind it, reached through the dirty utility by an open doorway; about 6 m inside the ward entrance | Meets / Partly | 04-01 5.37, 5.39; 00-03 9.15 |
| Clean utility with medicines and closed linen, 19.9 m² (≥ 16 m²) | Meets | 04-01 4.65–4.66, 5.30 |
| Cleaners' room 10.3 m² (≥ 8 m²) | Meets | 04-01 5.38 |
| Pantry 19.9 m² (≥ 12 m²); no kitchen asset | Meets / Partly | 04-01 5.27 |
| Assisted shower and WC 9.1 m², 2.01 m wide (2260 × 3050 mm in 00-02), reached through a shared lobby; assisted bathroom 15 m² | Partly | 04-01 5.13; 00-02 |
| Day room 10.4 m² (≥ 6 m² per 8 beds) | Meets | 04-01 5.24 |
| Resus-trolley bay (WS1), hoist and scales bay (WS2): stand-ins, no hoist asset | Partly | 04-01 schedule; 00-03 11.20 |
| Ward corridor 3.49 m; 2.66 m past the nurse base (≥ 2.96 m for two beds passing) | Meets / Partly | 00-04 3.9–3.10 |
| Interview room, staff rest, isolation suite, treatment room | Missing (treatment and isolation are optional for an all-single ward) | 04-01 5.23, 4.63, S1 |
| 6 beds against a 24-bed reference ward | Gap (scale) | 04-01 4.14 |

### 5.8 A&E (HBN 15-01)

[CS] marks a size from the Gateshead case study.

| Item | Status | Ref |
|---|---|---|
| Separate ambulance (courtyard) and walk-in (main entrance) entrances; walk-in patients never pass the ambulance entrance | Meets | 17.2, 17.4 |
| Ambulance canopy, lit (4.0 m soffit, two luminaires); ambulance bay AE1 at the entrance | Meets | 17.3 |
| Ambulance to resus unimpeded, away from the waiting area: 23 m | Meets | 17.20–17.21 |
| Handover desk and trolley and charging bay at the ambulance entrance | Meets | theme 2; 17.13 |
| Resus, 2 bays of 27.6 m² each (27 m² [CS]), trolleys free-standing for 360° access; pendants not modelled | Meets / Partly | 17.24–17.35 |
| X-ray next to resus (9.8 m), 32.6 m² (25 m² [CS]); no gantry asset | Meets / Partly | Ch 8; [CS] |
| 7 majors single rooms of 19.85 m² (12.6 m² [CS]); one door each | Meets / Partly (two doors, theme 10) | [CS]; theme 10 |
| Isolation room at the front, opposite the ambulance door; no lobby | Partly | 17.14; 04-01 S1 |
| Relatives' room near resus, behind the stair, on a non-public route; WC 25 m away; acoustic separation not modelled | Partly | 17.22, 17.55–17.59 |
| Walk-in reception and 15 seats, 27 m² (1.5 m² per place) | Meets | 00-03 7.23–7.30 |
| WC near the waiting area (3–5 m) | Meets | 00-03 7.30 |
| Triage 10.3 m² (13.2 m² [CS]); minors 10.3 m² (12.6 m² [CS]); 30–40 m walk from the waiting area | Partly | [CS] |
| Staff base in sight of the majors doors | Meets | — |
| Wheelchair and equipment park off the corridor (old SW stair hall) | Meets | 17.4, 17.18 |
| Decontamination, children's area and route, mental-health room, clinical decision unit (CDU), plaster room, A&E cleaners' room | Missing | 17.40–17.46, 17.49 |
| Route to theatres on the same level: resus to theatre about 80 m, through two compartment doors and the 2.73 m link opening | Partly | Ch 8, themes 1 and 8 |

### 5.9 Gaps (main building)

These gaps remain. A simulation for robot testing accepts them. A real building would need a design change or a
fire-engineered solution for each.

| # | Gap | Guidance |
|---|---|---|
| B1 | C1 has one adjoining compartment (C2). Its second escape direction is a final exit. | HTM 05-02 3.20 (Partly) |
| B2 | The stair enclosures have no protected lobbies. They discharge into C2, not to outside. The flights are 2.6 m wide and the foot landings are short. | HTM 05-02 3.56, 5.34, Table 3 |
| B3 | The upper floors are notional. The lifts serve one landing. | — |
| B4 | One building-wide alarm releases every magnet and recalls every lift. There is no alarm zoning per compartment. | — |
| B5 | The 60 m travel limit was not re-read in the source [unverified]. | HTM 05-02 3.34 |
| B6 | The pharmacy hatches onto the lobby are unrated glazing. | HTM 05-02 5.44 |
| B7 | The ward pantry and the relatives' room have no free-swing closers. | HTM 05-02 Table 6 |
| B8 | The A&E has no cleaners' room. | HBN 15-01 (section 5.8) |
| B9 | Bedroom doors are 1.2 m (bare minimum). Bed spaces are 3.6 × 3.37 m, not 3.6 × 3.7 m. | HBN 00-04 8.19; HBN 04-01 4.24 |
| B10 | The A&E has no decontamination, no children's area and route, and no CDU. | HBN 15-01 17.40–17.46 |
| B11 | Scale of the re-plan: a 6-bed ward against a 24-bed reference ward; a small A&E; triage and minors rooms below the case-study sizes. | HBN 04-01 4.14; HBN 15-01 [CS] |
| B12 | The assisted shower is 2.01 m wide and is reached through a shared lobby. | HBN 04-01 5.13; HBN 00-02 |
| B13 | The 1.20 m FD60 leaves of the link doorset need doorset certificate evidence [unverified]. | — |
| B14 | The main-lift bed-turning space is 4.30 m deep, 1 cm short of 4310 mm if that dimension is read as depth. | HBN 00-04 Fig 28 |
| B15 | Locked-shut doors are kept shut by the closer only. A lock is not modelled. | — |

## 6. Sources

- HTM 05-02 (2015) Firecode, NHS England: https://www.england.nhs.uk/wp-content/uploads/2021/05/HTM_05-02_2015.pdf
  (3.9–3.11, 3.20, 3.24–3.36, 3.56, 3.59–3.60, 3.69–3.71, 5.11, 5.34–5.44, Table 3, Table 6, App C, Table C1 as
  amended)
- HTM 08-02 (2016) Lifts, NHS England: https://www.england.nhs.uk/wp-content/uploads/2021/05/Lifts.pdf
  (4.4, 4.12, 4.19, 5.8, 5.23, 5.29–5.32, Table 3, 6.103–6.117, 6.151–6.158, App C)
- HBN 00-04 (2014) Circulation and communication spaces: https://www.nss.nhs.scot/media/1979/hbn-00-04-oct-2014.pdf
  (3.9–3.17, 3.29–3.30, 4.8–4.16, 6.16–6.30, Fig 11, Fig 28, 7.10, 8.9–8.27)
- Approved Document M vol 2 (with 2024 amendments): https://assets.publishing.service.gov.uk/media/66f6c5eec71e42688b65ee11/ADM__V2_with_2024_amendments.pdf
  (3.10, 3.28, 3.34)
- HBN 26 (2004) Facilities for surgical procedures, vol 1: https://www.england.nhs.uk/wp-content/uploads/2021/05/HBN_26.pdf
  (4.34, 4.54, 4.69–4.70, 4.82, 4.101, 4.123, 4.147–4.151, schedule of accommodation)
- HBN 04-01 (2013) Adult in-patient facilities: https://www.england.nhs.uk/wp-content/uploads/2009/12/HBN_04-01_Final.pdf
  (3.1, 4.14, 4.24, 4.28, 4.33, 4.44, 4.51–4.52, 4.63, 4.65–4.66, 5.11, 5.13, 5.23–5.24, 5.27, 5.30, 5.37–5.41, S1)
- HBN 15-01 Accident and emergency departments (17.2–17.59, Ch 8, design themes), with the Gateshead case study
  for the room sizes marked [CS]. Section 5.8 cites it.
- HBN 00-02 Sanitary spaces (4.40, Fig 46) and HBN 00-03 Clinical and clinical support spaces (7.23–7.30, 9.15,
  11.20, 12.46–12.66). Sections 5.7 and 5.8 cite them.
- BS EN 81-20, BS EN 81-70, BS EN 1154: not read (paid standards). Values from them are marked [secondary]
  or [unverified].
