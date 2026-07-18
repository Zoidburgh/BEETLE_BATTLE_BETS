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
- GATED: MB1 passed full lifecycle regression + canary. Committed 7b9b684.

**STATUS: MB2 IMPLEMENTED (uncommitted), awaiting user test with --balls N.**
- simulation: BALL2/2S=71/72, BALL3/3S=73/74 (stripe ALWAYS body+1),
  MAX_VOXEL_TYPE 74, is_ball_color()/is_ball_voxel() ti.funcs
- renderer: palette maps new ids to shared ball colors; frac_owner 71/72→6,
  73/74→7; ALL owner fields (frac/rot/pivot/squash x3) shape 6→8
- kernels parameterized by body_id: render_ball, render_ball_fast,
  clear_ball_fast (+arg at all 21 call sites), clear_ball sweeps all ids;
  pair-contact + occupied kernels use is_ball_color with color-relative
  stripe checks; leg-classifier uses is_ball_voxel
- per-ball render identity: ball_last_grid_*/rendered fields shape (3,);
  clear_and_render_ball_fast takes the ball (ids/index/owner_slot);
  render-loop squash writes go to ball.owner_slot
- spawn_extra_balls()/deactivate_extra_balls() + `--balls N` dev flag;
  extras spawn SIDE-BY-SIDE for now (center stack needs MB3's ball-ball
  physics or they interpenetrate); hooked into ball-mode on/off + guest
  set_network_ball_mode
- KNOWN LIMITS until MB3/4: no ball-ball collision (they pass through each
  other); extra balls that score/fall off explode but never respawn
  (celebration machinery only respawns ball 1); network syncs ball 1 only.
- GATED 2026-07-18: user confirms 3 balls spawn, roll and bounce smoothly.
  Extras' post-score respawn broken as expected (MB4). NOTE: ball activation
  has THREE sites — arena-select panel (~25815), older init block (~26810),
  guest set_network_ball_mode (deliberately single-ball until MB5); the spawn
  hook lives at the first two (dedupe into a helper during MB4).
- SPAWN DESIGN FINAL (user 2026-07-18, second revision): NO STACKS anywhere.
  Each ball has its own FIXED spawn point by index (center / +z / -z side
  offsets, the current --balls layout). Match start AND mid-play respawn
  both use the ball's own point. Simplifies MB4: respawn = the existing
  celebration timeline but resetting THE SCORING BALL to its own spawn
  point instead of always ball 1 to center.
- MB3 PERF FIX (same day): ball-beetle checks BATCHED (one launch + one
  readback per substep, fields grown to 18 pairs; results one substep
  stale — same staleness pair_collision_last already has). Unbatched was
  ~12 standalone launch+sync round-trips per substep = the FPS-15 report.

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

**STATUS: MB3 IMPLEMENTED (uncommitted) — awaiting user feel test.** Full-3D
center-line contact (air-air/air-ground/stacking all one formula); equal-mass
impulse w/ BALL_BALL_BOUNCE 0.55 + micro-contact guard (below per-step gravity
delta contacts are INELASTIC — the floor's rest-bounce lesson, else resting
stacks bounce forever); spin exchange BALL_BALL_GRIP 0.15; 50/50 positional
de-overlap capped 0.6/substep with grounded-down-share TRANSFER (top ball rides
up — this is what makes stack-toppling work); per-ball squish along the impact
axis (vertical-dominant = classic vertical squash) + ground-contact dust;
ball_ball_hits stat; sliders Ball-Ball Bounce/Grip. Pure Python, <=3 pairs,
zero kernel involvement — perf is noise by construction.

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

**STATUS: MB4 CORE IMPLEMENTED (uncommitted) — per-ball lifecycle complete
offline.** reset_ball_to_spawn(ball) + spawn_x/y/z recorded at both init
hooks; celebration end respawns EVERY exploded ball at its own point;
fallback (no-goal loss) triggers on any exploded ball; assembly starts/
ticks/renders per ball with each ghost at its ball's own spawn. REMAINING
MB4: pre-match Ball Count UI option (currently --balls flag only); dedupe
the two init hooks into one helper. Note: two balls exploding during one
celebration share the timeline and both respawn at its end — simple and
intended.

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

**STATUS: MB-PERF v2 HYBRID (uncommitted) — awaiting user test.**
LESSON (v1 reverted same day): fully-analytic DETECTION shipped ghost
pushes + clipping + weird hovers — the voxel check is the SAFETY NET for
analytic blind spots ("the grid detects, analytics measure" is load-
bearing architecture, not a habit). Gating at the -0.75 engagement zone
also bled impulses across the pre-contact band (weak lofts).
HYBRID KEEPS: voxel batch detection + mid-tick re-stamps (both feed the
net). REPLACES: only the per-contact CLUSTER kernels for ball pairs —
under confirmed voxel contact, contact point (deepest manifold contact),
count, and tips (per-entry horn-arm flag + t > 0.8) synthesize from the
analytic manifold, and the result is reused by the response (was a second
manifold build). Saves the ~1ms occupied+cluster+sync per contacting pair
— the dominant ball_physics cost — with zero blind-spot risk.

## Phase MB-PERF — fully-analytic ball collision (NEXT UP, before MB5)

Perf logs 2026-07-18 (14:03, busy 3-ball + 4 bots): frame 54.8ms (~18 FPS),
ball_physics 15.7ms dominant, substep death-spiral at 3.3 iters/frame.
The check batching fixed detection, but every CONTACTING (beetle, ball)
pair still runs the full voxel cluster pipeline (occupied x2 + cluster
kernel + pack sync ≈ 1ms each) — several per substep under fire.

THE FIX: ball-beetle contact goes FULLY ANALYTIC — the ball already owns
the complete analytic layer (_ball_surface_contact: penetration, true
normals, multi-contact manifold) and the response code already trusts it
for nearly everything. Restructure the ball branch of beetle_collision to
detect AND respond from the manifold alone: zero kernel launches, zero
syncs, pure Python (the MB3 ball-ball model). Cluster-supplied values to
replace: collision point average (use deepest-contact point), contact_count
gates (use pen>0), has_horn_tips/per-side tips (derive from which manifold
segment is a horn arm + its TIP region param). Est: ball_physics -> 3-4ms,
breaks the substep spiral, ~28+ FPS worst case, identical feel.
Also then possible: drop the mid-tick ball re-stamps entirely (the stamp
exists so the VOXEL check can see the ball — analytic doesn't need it;
render-frame stamping suffices) — another ~1ms and less grid churn.

KNOWN-OPEN (minor): resting ball jitter still slightly visible after the
separation deadband ("not that bad" — user). Suspects: stale batched-check
responses (1-substep-late cluster pushes), rest-latch vs ball-ball
interplay. Revisit during MB-PERF (analytic contact may fix it for free —
stale voxel results disappear entirely).

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
