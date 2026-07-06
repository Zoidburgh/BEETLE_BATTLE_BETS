# Phase 4 Inventory: per-color GPU surface → 4 players

Verified inventory (2026-07-06) of everything bound to blue/red that Phase 4 must
generalize. Line numbers approximate (drift as code changes). 4.1 is DONE.

## Done in 4.1 (commit 55c7f53)
- Voxel ids 51-57 (P3 green) / 58-64 (P4 yellow), part order: body, legs,
  leg_tip, stripe, horn_tip, hook, venom. `MAX_VOXEL_TYPE = 64`.
- `simulation.voxel_owner[]` / `voxel_part[]` lookups + `is_beetle_voxel()` /
  `beetle_owner()` ti.funcs; `PLAYER_VOXEL_IDS` table; `p3_*`/`p4_*` color fields.
- renderer.py palette branches 51-64 + shimmer gate; generic gates in
  clear_beetles, bounded-clear enumerations, ball-beetle voxel check.

## 4.2 Kernel factory + per-player fields (NEXT, biggest item)
- Field sets to x4 (beetle_physics.py ~5735-5802): `{blue,red}_body_cache_{size,x,y,z}`,
  `*_body_{hook,stripe,horn_tip,very_tip}_flags` (MAX_BODY_VOXELS=2000);
  `*_leg_cache_x/y/z` (240) + `*_leg_{start,end}_idx` (8); `*_leg_tip_cache_*` (100)
  + idx; `giraffe_{blue,red}_pivot_x/y` (~5759).
- **Retire the bare aliases** (~5804-5819): `body_cache_x` etc. alias BLUE fields;
  the blue kernel uses aliases, red uses explicit names.
- `place_animated_beetle_blue` (~7135-7699) / `_red` (~7700-8262): near-identical
  565-line kernels. Diffs: field prefix, color constants (STAG_HOOK/VENOM/HORN_TIP/
  STRIPE ids - now available per-slot via PLAYER_VOXEL_IDS), local var names, and a
  REAL DRIFT: blue has a leg Y-compensation block (~7600-7610 "Compensate Y to keep
  legs planted") that red LACKS - standardize on blue's version.
  Factory: `def make_place_kernel(fields, ids): @ti.kernel def place(...)` closure-
  capturing per-player fields; body/leg/leg_tip type ids already come in as params.
  Call sites: real render ~21698/21702, warmup ~16838-39.
- `rebuild_blue_beetle` (~6020) / `rebuild_red_beetle` (~6157): plain Python; copy
  generate_beetle_geometry() lists into caches + set beetles[slot] horn metadata +
  giraffe pivot. → `rebuild_beetle(slot)`. Call sites: 2154/2197, 22262-22349,
  24360-24599.
- `render_assembly_kernel_blue` (~3865) / `_red` (~3901): red flips X and Z for
  180° facing - factory needs a flip param or per-slot facing.
- `calculate_beetle_lowest_point` (~7027): color-agnostic but reads BLUE aliases -
  needs per-slot fields (currently subtly wrong for red when geometries differ!).
- ASSEMBLY_VOXEL_* ids 25-32 (beetle_physics ~3753-3760) are blue/red only; P3/P4
  assembly anim can reuse P3/P4 body ids or add ids 65+.

## 4.3 Owner-parameterized kernels
- `check_collision_kernel` (~4024): color1/color2 owner sets `{5,7,11,13,18}+9` vs
  `{6,8,12,14,19}+10` → use beetle_owner()/voxel_part lookups.
- `calculate_occupied_voxels_kernel` (~8373, takes beetle_color): owner sets incl
  STINGER + VENOM. `beetle1/beetle2_occupied_*` fields (~5824-5830) are 2-slot → 4.
- `calculate_edge_tipping_kernel` (~10093, takes beetle_color): two owner-set passes.
- UFO beam (~12592) writes `ufo_beam_hit_blue/_red` (~12589); comet (~12641) writes
  `comet_hit_blue/_red` + `comet_hit_pos_*` (~12636-39) → 4-slot arrays/fields.
- spray `check_spray_voxel_collision_kernel` (~14175, target_color 0=blue else red).
- `is_horn_zone_voxel` (~11648, is_blue param), `transform_body_voxel_to_world`
  (~11423, is_blue branches to blue/red caches at ~11441/11478).
- horn-tip either-color check ~8453 (`calculate_collision_point_kernel`).

## 4.4 Silk 4-way
- `check_silk_beetle_collision` (~11861): 16 blue_* + 16 red_* flattened params,
  loops both beetles internally → redesign as loop-over-beetles (or per-slot call).
- `count_floor_silk_under_beetles` (~12120): blue/red/ball params → slots.
- Counters (simulation.py ~199-206): silk_on/under_{blue,red,ball} → per-slot;
  `silk_counts_batched` shape 6 → 10, layout [slot*2]=on, [slot*2+1]=under, ball last.
  Packed at simulation.py ~8279-84; read at beetle_physics ~17856 (+ warmup 16925).
  Game code already indexes silk_counts[slot*2] in the merged input loop.

## 4.5 Game logic + bots + perf gate
- `--local4`: active_player_count=4, beetles[] grows to 4 (spawns already handle
  slots 2+ via ring), scores[] len 4, per-player state lists 2→4 entries.
- BotPlayer (host-side): builds 6B MSG_INPUT packets through _handle_packet with
  synthetic sender ids mapped to slots; `--bots N --bot-ai seek|random|idle`.
- Perf gate: >=60fps with 4 beetles brawling in --local4 on the host machine
  BEFORE networking on top (perf_monitor per-phase breakdown + FPS HUD).
- Camera/UI: score display for 4, camera framing.

## Renderer color mechanism (for reference)
`renderer.get_voxel_color` (~177-361): if/elif on voxel_type → reads
`simulation.{blue,red,p3,p4}_*_color[None]` fields. Custom colors set via
MSG_BEETLE_CONFIG apply path in beetle_physics.

## Gotchas
- Taichi: wrap numpy/field-read floats with float() before kernel args.
- Kernel warmup calls needed during phase-2 loading for any NEW kernels
  (offscreen coords like 0.0, -100.0) - add P3/P4 placement warmups.
- `vtype >= BEETLE_BLUE` gate in clear_beetles_bounded (~8274) already catches
  ids 51-64 correctly (excludes SHADOW/SLIPPERY/GOAL).
