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

## Known limits

- Lite assets: close-up edges on i4h props are simplified and textures are softer. Switch to full on a
  bigger GPU with `scene/select_assets.py full`.
- A few i4h parts were left undecimated because they carry second UV sets or GeomSubsets (endoscope,
  microscope, breathing circuits, the Lightwheel trolley).
- The Isaac hospital's own doors are left as authored, except the SE door the wing uses.
