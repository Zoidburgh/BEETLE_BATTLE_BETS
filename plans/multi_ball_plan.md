# Multi-Ball Plan — up to 3 simultaneous balls
2026-07-18. Mirrors the beetles[] refactor playbook (phase 2, commits
17b6afe..cd0bb75): alias-preserving mechanical refactor first, verified
identical, THEN new capability.

**STATUS 2026-07-18: MB1 ~60% done, user-tested twice, UNCOMMITTED.**
- DONE (MB1a): all satellite globals folded into ball ATTRIBUTES (has_exploded,
  assembling+timer, squash amount/timer/yaw/horiz, last_render, net_vis_offset,
  scored_this_fall, explosion_*) — 113 attr references, old g{}/module duals
  collapsed (NOTE: the `g` state dict rebinds to globals() on frame 1 at the
  render section's `g = globals()` — the dict literal at ~20050 is dead after
  frame 1; the g['ball_*'] sites were module globals all along).
  `balls = [beetle_ball]` exists; beetle_ball = alias of balls[0].
- DONE (MB1b): floor-collision block (200 lines) + save_previous_state wrapped
  in `for _mb_ball in balls:` via the ZERO-REINDENT transform: replace
  `if beetle_ball.active:` header with for + `if not active: continue` guard at
  the same depth — body indentation untouched, rename inside. USE THIS PATTERN.
- DONE (MB1c): the per-substep physics/goal/beetle-collision SUPER-BLOCK
  (update_physics, friction/rolling/rest-latch, bowl, goal detection, collision
  cluster + anti-teleport governor), explosion trigger + explosion particles,
  and the render block (interp/net-offset/squash/stamp/shadow) — all wrapped
  with the zero-reindent transform. beetle_ball refs 376 → 317; the rest are
  DELIBERATE singletons for count=1:
  * celebration/respawn machinery (match-level; MB4's center-stack replaces it)
  * network send/guest-apply + guest explode (protocol v6 single-ball; MB5)
  * ice detection (`ball_on_ice` is a shared global → make per-ball attr in
    MB3 — friction/roll-blend read it inside the loop already)
  * render identity: ball_last_grid_*/ball_last_rendered Taichi singletons +
    owner slot 4 + shared voxel ids (MB2's whole job)
  * silk/spray/UFO/comet/hazard ball checks, arena-mode setup UI, BALL_TRACE
- GATE NOW: full play regression (score cycle, explosion, respawn, squash,
  bounce feel) + one canary. Then MB2.

## Scope of the problem (surveyed 2026-07-18)

`beetle_ball` is a singleton with **376 direct references** (lines 3105-28121) plus
~227 satellite globals (ball_squash_*, ball_last_*, ball_cache_*, ball_render
state, ball_has_exploded, ball_assembling, ball_dust_cooldown). The renderer
hardwires the ball to **owner slot 4** (frac offset / rot residual / squash fields,
shape=6: beetles 0-3, ball 4, ladybug 5). Ball voxels are types 16/17 — and the
extract attributes sub-voxel smoothing BY VOXEL TYPE, so multiple balls sharing
type 16 would all inherit ball #0's offsets (balls 2-3 would voxel-step and smear).
Network protocol v6 syncs exactly one ball block.

## V1 simplifiers (decisions baked into this plan — revisit later if wanted)

- **All balls share ONE radius** (the slider applies to all). Per-ball radii means
  per-ball voxel caches — defer.
- **Local/offline first**; network multi-ball is its own phase with a protocol bump.
- **Any ball in a goal scores**; that ball alone explodes + respawns. Existing
  pending_scores queue already handles simultaneous events.
- Ball count is a pre-match option (1-3), not spawnable mid-match.

## Phase MB1 — balls[] mechanical refactor (the big one, zero behavior change)

- `balls = [beetle_ball]`; keep `beetle_ball` as ALIAS of balls[0] (the
  blue_*/red_* alias trick from the beetles[] refactor — old names keep working
  while call sites migrate).
- Fold the satellite globals into per-ball attributes: squash (amount/timer/yaw/
  horiz), last_render tuple, has_exploded, assembling+timer, rest-latch state,
  seg_latch (already a ball attr), net vis offset. `ball_dust_cooldown` stays
  GLOBAL (it exists to rate-limit dust visually — shared is correct).
- Wrap the inline ball physics blocks in `for _ball in balls:` loops: gravity/
  integration, floor bounce + settle, bowl slide + rim, goal detection, ice ring,
  silk, carry/parked freeze.
- Collision: the per-beetle ball calls (beetle_collision(beetle, ball)) loop over
  balls; pair-gate distance cull per (beetle, ball).
- GATE MB1: `--canary` + user play with count=1 — behavior must be
  indistinguishable. This phase is 90% of the risk; do nothing else in it.

## Phase MB2 — render identity (per-ball smoothing)

- **New voxel ids** for ball 2 and ball 3 (body + stripe each; next free ids after
  the 65-70 assembly block — VERIFY free ranges, project rule: new ids need care).
  simulation lookups (is_ball check `vtype == 16 or 17`) become a small id-set;
  beetle_owner-style mapping extended: ball index from voxel type.
- **Renderer owner slots**: grow the owner fields shape 6 → 8 (balls at 4, 6, 7;
  ladybug stays 5). The extract's `frac_owner` selection maps the new ball types
  to their slots. Squash/rot-residual/frac writes per ball in the render loop.
- Ball stamp/clear: `clear_and_render_ball_fast` takes the ball's ids + its own
  last-grid fields (fold into per-ball attrs; the Taichi last_grid fields become
  shape=(3,) indexed by ball).
- Assembly: sequential is fine (one ball assembling at a time is the common case;
  if two respawn together, stagger starts by 0.3s — scatter fields are shared
  read-only).
- GATE MB2: 3 balls rolling smoothly side by side (each glides sub-voxel, each
  squashes independently), radius slider resizes all three without ghosts.

## Phase MB3 — ball-on-ball physics (the fun part, and the easiest physics)

Sphere-vs-sphere is the simplest contact in the whole game — fully analytic:
- Pair loop over ball pairs (max 3 pairs). Contact when center distance <
  r1 + r2. Impulse along the center line: equal masses, restitution ~0.55
  (BALL_BALL_BOUNCE slider) — billiard-feel, livelier than beetle contacts.
- Positional de-overlap split 50/50 (velocity-capped, S1 style, never downward
  into a grounded ball).
- Tangential: spin exchange via surface grip at the contact (reuse
  BALL_BOUNCE_GRIP pattern) — a fast ball glancing another sets both spinning.
  Keep simple; billiards, not snooker.
- Visuals for free: horizontal squish on BOTH balls along the center line
  (existing per-ball squish from MB2), dust puff at ground-level contacts
  (shared cooldown).
- Ball-ball counts in collision_stats ('ball_ball_hits') for canary visibility.
- GATE MB3: drop 3 balls together — they settle into a mutually-exclusive rest
  (no interpenetration, no jitter pile); billiard break feels right.

## Phase MB4 — spawn/score/lifecycle

- Ball Count option (1-3) in the ball-mode setup UI.
- **SPAWN = CENTER STACK (user design 2026-07-18)**: every ball spawns at
  center. If the center column is occupied (a ball resting there or one mid-
  assembly), the new ball spawns ABOVE the highest occupant (+2r + margin) —
  they stack, and ball-ball physics (MB3) topples the stack naturally: the
  match-start "break" and post-score re-entry both emerge from physics, no
  placement logic beyond "find the top of the center column". Assembly rain
  targets the stacked height (works today — target y is a parameter).
- Goal: each ball scores independently; only the scoring ball explodes/respawns
  (per-ball has_exploded/assembling from MB1 makes this free). Simultaneous
  crossings both count (pending_scores queue).
- GATE MB4: 3-ball match start topples the stack into play; score one ball while
  another sits at center — respawn stacks on top and topples off; no
  double-counts, no interpenetrating spawns.

## Phase MB5 — network (protocol v7)

- send_state_sync: ball_count byte + N ball blocks (exact mirror of the
  player_count pattern from v4). PROTOCOL_VERSION 6 → 7 (mismatched builds
  refuse — project rule).
- Guest correction: loop balls (per-ball targets + per-ball net vis offsets from
  MB1). MSG_BALL_EXPLODE gains a ball-index byte.
- set_network_ball_mode carries count.
- GATE MB5: 2-PC (or --simlag) 3-ball match; balls stay distinct (no identity
  swaps — sync by INDEX, stable because count is fixed at match start).

## Known hazards (write these on your hand before MB1)

- The renderer extract maps voxel TYPE → owner slot; miss one mapping and that
  ball voxel-steps (the pre-July jitter look) — the visual regression to watch.
- The ball pair-gate's same-column adjacency (±2 rows) and hook "glue aura"
  exclusions reference ball ids — every `== 16 or == 17` site must become the
  id-set check (grep census before starting MB2; memory: color==string class
  of bug, but for ints).
- Goal-lane / bowl-slide / rim logic reads ball singleton state inline — MB1's
  loop must hoist ALL of it or two balls will fight over one rest-latch.
- Scoring double-fire: pending_scores is a queue (fixed 3b7e9c3) — keep using it.
- Perf: ball physics measured ~0.0ms at 1 ball; 3 balls + 3 ball-pairs is noise.
  The real cost is 3x stamp/clear voxel calls — watch scene_render in canary.

## Execution order + effort guess

MB1 (biggest, ~a session, all risk) → gate → MB2 (~half session) → gate → MB3
(small, fun) → MB4 (small) → MB5 (protocol bump session, pairs with the still-
pending v6 multiplayer test — consider testing v6 first so network changes don't
stack untested).
