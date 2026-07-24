# Flight Boost Plan

> **STATUS: DESIGN DRAFT — NOTHING HERE IS SET IN STONE.**
> This plan was written before implementation started (2026-07-24). The trigger key, forces,
> animation details, and even the phasing are all expected to change as we design and
> play-test. Treat every number as a slider default, not a decision. If this doc and the
> code disagree, the code + commit messages win.

## Concept

The last big mechanic: a **flight boost**. On trigger, wings pop out of the beetle's back,
the beetle gets a single discrete impulse **forward and up relative to its facing**, the
wings flap briefly, then retract. Wings are **visual only — they never collide**.

Uses: escape a bad engagement, open a charge, save yourself near an edge, jump gaps.

Feel compass check: this is a discrete impact-event mechanic (one impulse, then normal
physics), not a continuous force — consistent with everything we've kept. Boost Force 0
= mechanic off (old feel).

## Trigger

**Current pick: SPACE (dedicated key), may change later.**

- Space's only existing use is the title screen "press SPACE to start"
  (`beetle_physics.py:21126`) — state-gated, no in-match conflict.
- **Local 2P shares one keyboard** → space can only serve blue. Red needs its own boost
  key (add a `'boost'` entry to `BLUE_KEYS`/`RED_KEYS`/`NETWORK_KEYS` at
  `beetle_physics.py:1090-1122`; red candidate: `,` or right-shift — pick when wiring).
- **Controller**: map to a face/shoulder button in `get_controller_inputs()`
  (`beetle_physics.py:1191`).
- Edge-triggered: boost fires on **press** (rising edge), not while held. No edge
  detection exists yet anywhere — inputs are level-triggered bitmasks — so per-slot
  `prev_inputs` tracking gets built as part of this feature.

Shelved alternatives (revisit if space feels wrong): forward+backward chord (instant,
accident-proof, but impossible on a controller stick), tap-back-then-forward (stick-
compatible slingshot feel), double-tap backward, triple-tap forward (original idea;
rejected for now — tap-timing lottery under panic, accident risk on the most-hammered key).

## Input plumbing + network

All 8 bits of the input byte are taken (`beetle_physics.py:1080-1087`). Boost needs a 9th:

- Add `INPUT_BOOST = 0x100` and **widen the input mask byte→2 bytes** in network.py:
  `send_input` pack `'>BIB'`→`'>BIH'` (`network.py:620`), per-slot entry in
  `send_inputs_all` `'>BB'`→`'>BH'` (`network.py:631-633`), and both unpack sites
  (`network.py:1166-1171`, `:1181-1184`).
- **Bump `PROTOCOL_VERSION`** (`network.py:40-49`) — mismatched builds must refuse to match.
- Python-side (`InputBuffer`, bot inputs, keyboard reads) all use plain ints — no other
  width changes needed.
- Host-authoritative: the **host** edge-detects and applies the boost from the input
  stream; state sync corrects guests as usual. Guest-side wing visuals: either sync a
  wing-anim timer in state sync, or let guests run the same edge detection off the
  rebroadcast `MSG_INPUTS_ALL` — decide during implementation (prefer whichever avoids
  another state-sync field).
- Bots: no boost initially. Later maybe edge-save / gap-escape use in the bot state
  machine (`get_bot_inputs`, `beetle_physics.py:1457`).

## Physics

One impulse on trigger, applied in the beetle's yaw frame:

- Direction: `forward * cos(BOOST_ANGLE) + up * sin(BOOST_ANGLE)` — angle is a slider,
  not two separate forces, so tuning stays one-knob.
- Delivery: through the existing `apply_force`/`pending_lift` path like everything else,
  **but bypassing `LIFT_CAP`/`AIR_NUDGE_CAP`** — those caps exist to tame received combat
  lift/juggling; this is self-initiated movement.
- **Cooldown** (slider) prevents spam-flying across the arena.
- **Grounded-or-coyote only at first** (`airborne_h` false, or within a short grace),
  with an AIR BOOST toggle to experiment with mid-air boosts later.
- Known interaction, intentional: pressing boost mid-charge doesn't touch
  `forward_hold_time`, but leaving the ground kills traction/speed-ramp benefits —
  boost and ground-charge naturally compete rather than stack. Watch in testing.
- Watch item: boosting mid-knockback (`knockback_timer`) may make combo escapes too
  easy. If so, gate boost while knockback_timer is hot.

### FLIGHT BOOST slider panel (all defaults are guesses)

| Slider | Default | Notes |
|---|---|---|
| Boost Force | ~600 | 0 = mechanic off (old feel) |
| Boost Angle | ~35° | up/forward mix |
| Cooldown | ~4s | |
| Coyote Time | ~0.15s | grounded-grace for edge saves |
| Air Boost | off | toggle |
| Wing Time | ~0.8s | total out→flap→retract, drives animation |

Pattern: keys in `physics_params` (~`beetle_physics.py:20300+`), GUI block near
`=== PHYSICS TUNING ===` (`:29694`).

## Wing animation — modeled on real beetle flight

The takeoff sequence of a real beetle, stage by stage, each mapped to an existing
animation pattern in the codebase:

1. **Crouch dip** (~2-3 frames): slight body lower/pitch-down. Real beetles push off
   with their legs. Free anticipation.
2. **Elytra pop**: shell covers split at the midline and rotate up-outward into a raised
   V, hinged at the FRONT (pronotum edge). They then HOLD that pose — elytra never flap.
   This is the most recognizable beetle-flight silhouette. Implementation = the horn
   trick: flag shell voxels in the body cache, rotate them around a front hinge pivot in
   `place_animated_beetle` (subset-rotation examples: horns `beetle_physics.py:8621`,
   bombardier abdomen `:8803`).
3. **Hindwings unfold** (~0.1s): membranous wings spring out backward+outward from under
   the shell into a swept delta shape ~2x elytra length. Implementation = new
   `wing_cache_x/y/z` voxel group baked in `generate_beetle_geometry()` (`:5900`) /
   `rebuild_beetle()` (`:7384`), placed by a new block in the placement kernel next to
   the leg loop (`:8907`), driven by a `wing_extend` 0→1 kernel arg (passed like
   `butt_wiggle`/`spray_aim_pitch` at `:8551`). Real unfold takes ~200ms — speed hides
   ugly mid-frames.
4. **Flap = blur, not flap.** Big beetles beat ~30-40 Hz; never animate literal strokes.
   Alternate wing voxels between 2-3 stroke positions every frame/substep — at game
   framerate this strobes into a blur, which is what a real beetle looks like.
5. **Body language sells it**: nose-up pitch 20-30° during the boost (big beetles hang
   almost vertically under their wings), legs tucked/frozen (airborne handling already
   stops the gait). Famously clumsy-looking — on brand.
6. **Retract**: reverse fast; elytra clap shut with a small dust puff (dust-as-speed-
   language: the impulse needs readable feedback even before the wings look good —
   debris/dust burst at launch too, via the existing particle pools `simulation.py:187`).

**Collision**: wings are render-only. Write wing voxels with a voxel id that is NOT in
`PLAYER_VOXEL_IDS` — collision kernels (`beetle_physics.py:5494`, `:5547`) only match
beetle ids, so non-beetle-id voxels render but never collide. Zero special-casing.

**Fallback look** if fold-out wings read as "gray slabs" on some beetle types: keep the
shell closed and slip wings out side notches (real flower chafers do this), or lean on
shell-pop + strong particle burst + brief wing-blur only.

## Build phases (offline first — multiplayer v6 still untested)

1. **Input + impulse, no visuals**: boost key entries, per-slot `prev_inputs` edge
   detection at the `p_inputs` consumption point (one path covers local/bots/network),
   impulse + cooldown + coyote gate, FLIGHT BOOST panel. Playable with dust puff only.
2. **Wings**: wing_cache geometry per beetle type, elytra-pop subset rotation, extend +
   blur-flap in placement kernel, nose-up pose, retract + shell-clap puff.
3. **Network**: 2-byte input mask + PROTOCOL_VERSION bump, guest wing visuals, then fold
   into the (still-pending) v6 two-PC test ladder.
4. **Later / optional**: bot usage (edge saves), air-boost tuning, per-type wing shapes.

## Open questions (decide by playing, not on paper)

- Red's local-2P boost key.
- Boost while ball-carrying / mid-scoop — any degenerate ball plays?
- Should boost cost the charge ramp explicitly, or is losing ground traction enough?
- Knockback-escape gating needed?
- Per-beetle-type boost stats (rhino heavy/short, atlas floaty?) — only after the base
  feel is right, via `BEETLE_TYPE_STATS` if wanted.
